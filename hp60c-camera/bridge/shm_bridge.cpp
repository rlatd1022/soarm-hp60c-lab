// HP60C SDK -> POSIX shared memory bridge
//
// HP60C/ASC60C(Angstrong) 뎁스카메라는 UVC로 직접 열리지 않는다.
// 제조사 C++ SDK의 Listener 패턴을 통해 카메라를 받아 RGB/Depth 프레임을
// /dev/shm/hp60c_frames 에 써서, 어떤 언어든 mmap으로 읽을 수 있게 한다.
//
// SHM layout (/dev/shm/hp60c_frames):
//   [Header: 64 bytes]
//     uint32 magic        = 0x48503630 ("HP60")
//     uint32 rgb_w, rgb_h, rgb_size
//     uint32 depth_w, depth_h, depth_size
//     uint32 _pad
//     uint64 frame_id
//     uint64 timestamp_us
//     uint32 rgb_ready    (0 or 1)
//     uint32 depth_ready  (0 or 1)
//   [RGB data:   MAX_RGB_SIZE   bytes,  BGR uint8]
//   [Depth data: MAX_DEPTH_SIZE bytes,  uint16, mm]

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <atomic>
#include <thread>
#include <chrono>
#include <list>
#include <vector>
#include <string>
#include <dirent.h>

#include "as_camera_sdk_api.h"
#include "as_camera_sdk_def.h"

// Not in public headers; used by HP60C setGain (XU unit 0x47).
extern int AS_SDK_Xu_CustomWrite(AS_CAM_PTR pCamera, unsigned char unit,
                                 unsigned char *data, unsigned int size, int timeout);

#define SHM_NAME        "/hp60c_frames"
#define HEADER_SIZE     64
#define MAX_RGB_SIZE    (1920 * 1080 * 3)
#define MAX_DEPTH_SIZE  (640 * 480 * 2)
#define SHM_TOTAL_SIZE  (HEADER_SIZE + MAX_RGB_SIZE + MAX_DEPTH_SIZE)
#define SHM_MAGIC       0x48503630u  // "HP60"
#define HP60C_XU_GAIN   0x47
#define DEFAULT_GAIN    12  // config default is 4; range 1..28

struct ShmHeader {
    uint32_t magic;
    uint32_t rgb_w, rgb_h, rgb_size;
    uint32_t depth_w, depth_h, depth_size;
    uint32_t _pad;
    uint64_t frame_id;
    uint64_t timestamp_us;
    uint32_t rgb_ready;
    uint32_t depth_ready;
};

// HP60C XuCmdCameraHp60c::setGain lookup (gain 1..28 → R2=3e08 / 3e09)
static const uint8_t kHp60cGainTable[28][2] = {
    {0x03, 0x20}, {0x23, 0x24}, {0x23, 0x35}, {0x27, 0x24},
    {0x27, 0x2d}, {0x27, 0x35}, {0x27, 0x3e}, {0x2f, 0x24},
    {0x2f, 0x28}, {0x2f, 0x2d}, {0x2f, 0x31}, {0x2f, 0x35},
    {0x2f, 0x3a}, {0x2f, 0x3e}, {0x3f, 0x22}, {0x3f, 0x24},
    {0x3f, 0x26}, {0x3f, 0x28}, {0x3f, 0x2a}, {0x3f, 0x2d},
    {0x3f, 0x2f}, {0x3f, 0x31}, {0x3f, 0x33}, {0x3f, 0x35},
    {0x3f, 0x38}, {0x3f, 0x3a}, {0x3f, 0x3c}, {0x3f, 0x3e},
};

static std::atomic<bool> g_running{true};
static uint8_t* g_shm_ptr = nullptr;
static uint64_t g_frame_id = 0;
static AS_CAM_PTR g_camera = nullptr;
static std::string g_config_dir;

static int applyHp60cGain(AS_CAM_PTR cam, int gain) {
    if (gain < 1 || gain > 28) {
        fprintf(stderr, "[shm_bridge] gain %d out of range 1..28\n", gain);
        return -1;
    }
    const uint8_t* pair = kHp60cGainTable[gain - 1];
    char cmd[64];
    int n = snprintf(cmd, sizeof(cmd), "S=C0,R2=3e08,D=%02x", pair[0]);
    int r1 = AS_SDK_Xu_CustomWrite(cam, HP60C_XU_GAIN,
                                   reinterpret_cast<unsigned char*>(cmd),
                                   static_cast<unsigned int>(n + 1), 1000);
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    n = snprintf(cmd, sizeof(cmd), "S=C0,R2=3e09,D=%02x", pair[1]);
    int r2 = AS_SDK_Xu_CustomWrite(cam, HP60C_XU_GAIN,
                                   reinterpret_cast<unsigned char*>(cmd),
                                   static_cast<unsigned int>(n + 1), 1000);
    printf("[shm_bridge] set gain %d (xu ret %d/%d)\n", gain, r1, r2);
    return (r1 < 0 || r2 < 0) ? -1 : 0;
}

// Config applies gain=4 shortly after StartStream; override after that settles.
static void gainOverrideThread(AS_CAM_PTR cam, int gain) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    if (!g_running || g_camera != cam) return;
    applyHp60cGain(cam, gain);
}

static void signal_handler(int) { g_running = false; }

static int scanDir(const std::string& dir, std::vector<std::string>& files) {
    DIR* d = opendir(dir.c_str());
    if (!d) return -1;
    struct dirent* ent;
    while ((ent = readdir(d)) != nullptr) {
        if (ent->d_name[0] == '.') continue;
        std::string p = dir + "/" + ent->d_name;
        if (ent->d_type == DT_REG)      files.push_back(p);
        else if (ent->d_type == DT_DIR) scanDir(p, files);
    }
    closedir(d);
    return 0;
}

static std::string findConfigFile(const std::string& dir, const std::string& key) {
    std::vector<std::string> files;
    scanDir(dir, files);
    for (auto& f : files) {
        if (f.find(key) != std::string::npos) return f;
    }
    return "";
}

// SDK stream callback -> write into shared memory
static void onNewFrame(AS_CAM_PTR /*pCamera*/, const AS_SDK_Data_s* pstData, void* /*priv*/) {
    if (!g_shm_ptr || !g_running) return;

    auto* hdr      = reinterpret_cast<ShmHeader*>(g_shm_ptr);
    uint8_t* rgb   = g_shm_ptr + HEADER_SIZE;
    uint8_t* depth = g_shm_ptr + HEADER_SIZE + MAX_RGB_SIZE;

    g_frame_id++;

    if (pstData->rgbImg.size > 0 && pstData->rgbImg.size <= MAX_RGB_SIZE) {
        hdr->rgb_w    = pstData->rgbImg.width;
        hdr->rgb_h    = pstData->rgbImg.height;
        hdr->rgb_size = pstData->rgbImg.size;
        memcpy(rgb, pstData->rgbImg.data, pstData->rgbImg.size);
        hdr->rgb_ready = 1;
    }

    if (pstData->depthImg.size > 0 && pstData->depthImg.size <= MAX_DEPTH_SIZE) {
        hdr->depth_w    = pstData->depthImg.width;
        hdr->depth_h    = pstData->depthImg.height;
        hdr->depth_size = pstData->depthImg.size;
        memcpy(depth, pstData->depthImg.data, pstData->depthImg.size);
        hdr->depth_ready = 1;
    }

    hdr->frame_id     = g_frame_id;
    hdr->timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                           std::chrono::steady_clock::now().time_since_epoch()).count();
    hdr->magic = SHM_MAGIC;
}

// SDK_GetCameraList()는 HP60C에서 동작하지 않는다.
// Listener를 등록하면 이미 연결된 카메라에 대해서도 onAttached가 호출된다.
static void onAttached(AS_CAM_ATTR_S* attr, void* /*priv*/) {
    if (g_camera) return;

    AS_CAM_PTR cam;
    if (AS_SDK_CreateCamHandle(cam, attr) != 0) {
        fprintf(stderr, "[shm_bridge] CreateCamHandle failed\n");
        return;
    }

    AS_SDK_CAM_MODEL_E model;
    AS_SDK_GetCameraModel(cam, model);
    printf("[shm_bridge] Camera attached, model type: %d\n", model);

    // Config 파일은 'hp60c_' 키워드로 검색 (실제로는 vega_*.cfg 가 매칭되기도 함)
    std::string cfg = findConfigFile(g_config_dir, "hp60c_");
    if (cfg.empty()) cfg = findConfigFile(g_config_dir, "hp60cn_");
    if (cfg.empty()) {
        std::vector<std::string> files;
        scanDir(g_config_dir, files);
        if (!files.empty()) cfg = files[0];
    }
    if (cfg.empty()) {
        fprintf(stderr, "[shm_bridge] No config file found in %s\n", g_config_dir.c_str());
        return;
    }
    printf("[shm_bridge] Using config: %s\n", cfg.c_str());

    if (AS_SDK_OpenCamera(cam, cfg.c_str()) != 0) {
        fprintf(stderr, "[shm_bridge] OpenCamera failed\n");
        return;
    }

    AS_CAM_Parameter_s p;
    if (AS_SDK_GetCamParameter(cam, &p) == 0) {
        printf("[shm_bridge] IR : fx=%.1f fy=%.1f cx=%.1f cy=%.1f\n", p.fxir, p.fyir, p.cxir, p.cyir);
        printf("[shm_bridge] RGB: fx=%.1f fy=%.1f cx=%.1f cy=%.1f\n", p.fxrgb, p.fyrgb, p.cxrgb, p.cyrgb);
        if (FILE* fp = fopen("/tmp/hp60c_params.txt", "w")) {
            fprintf(fp, "fxir=%.6f\nfyir=%.6f\ncxir=%.6f\ncyir=%.6f\n",  p.fxir,  p.fyir,  p.cxir,  p.cyir);
            fprintf(fp, "fxrgb=%.6f\nfyrgb=%.6f\ncxrgb=%.6f\ncyrgb=%.6f\n", p.fxrgb, p.fyrgb, p.cxrgb, p.cyrgb);
            fprintf(fp, "T1=%.6f\nT2=%.6f\nT3=%.6f\n", p.T1, p.T2, p.T3);
            fclose(fp);
        }
    }

    AS_CAM_Stream_Cb_s cb;
    cb.callback    = onNewFrame;
    cb.privateData = nullptr;
    AS_SDK_RegisterStreamCallback(cam, &cb);

    if (AS_SDK_StartStream(cam) != 0) {
        fprintf(stderr, "[shm_bridge] StartStream failed\n");
        return;
    }

    g_camera = cam;
    printf("[shm_bridge] Streaming started.\n");

    int gain = DEFAULT_GAIN;
    if (const char* env = std::getenv("HP60C_GAIN")) {
        int g = atoi(env);
        if (g >= 1 && g <= 28) gain = g;
    }
    std::thread(gainOverrideThread, cam, gain).detach();
}

static void onDetached(AS_CAM_ATTR_S* /*attr*/, void* /*priv*/) {
    printf("[shm_bridge] Camera detached\n");
    if (g_camera) {
        AS_SDK_StopStream(g_camera);
        AS_SDK_CloseCamera(g_camera);
        AS_SDK_DestoryCamHandle(g_camera);
        g_camera = nullptr;
    }
}

int main(int argc, char* argv[]) {
    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    // Config 디렉토리: argv[1] > $HP60C_CONFIG_DIR > ../configurationfiles
    if (argc > 1) {
        g_config_dir = argv[1];
    } else if (const char* env = std::getenv("HP60C_CONFIG_DIR")) {
        g_config_dir = env;
    } else {
        g_config_dir = "../configurationfiles";
    }
    printf("[shm_bridge] Config dir: %s\n", g_config_dir.c_str());

    shm_unlink(SHM_NAME);
    int fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (fd < 0) { perror("shm_open"); return 1; }
    if (ftruncate(fd, SHM_TOTAL_SIZE) != 0) { perror("ftruncate"); return 1; }
    g_shm_ptr = (uint8_t*)mmap(nullptr, SHM_TOTAL_SIZE,
                               PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (g_shm_ptr == MAP_FAILED) { perror("mmap"); return 1; }
    memset(g_shm_ptr, 0, SHM_TOTAL_SIZE);
    printf("[shm_bridge] Shared memory ready: /dev/shm%s (%d bytes)\n",
           SHM_NAME, SHM_TOTAL_SIZE);

    if (AS_SDK_Init() != 0) {
        fprintf(stderr, "[shm_bridge] AS_SDK_Init failed\n");
        return 1;
    }
    printf("[shm_bridge] SDK initialized. Waiting for camera...\n");

    AS_LISTENER_CALLBACK_S listener;
    listener.onAttached  = onAttached;
    listener.onDetached  = onDetached;
    listener.privateData = nullptr;
    AS_SDK_StartListener(listener, AS_LISTENNER_TYPE_USB, true);

    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    printf("\n[shm_bridge] Stopping...\n");
    if (g_camera) {
        AS_SDK_StopStream(g_camera);
        AS_SDK_CloseCamera(g_camera);
        AS_SDK_DestoryCamHandle(g_camera);
    }
    AS_SDK_StopListener();
    AS_SDK_Deinit();

    munmap(g_shm_ptr, SHM_TOTAL_SIZE);
    shm_unlink(SHM_NAME);
    printf("[shm_bridge] Done.\n");
    return 0;
}
