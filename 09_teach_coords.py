# -*- coding: utf-8 -*-
"""09_teach_coords.py — 실물 ↔ 시뮬 연동 손끝 좌표 티칭.

기준점: gripperframe (집게 TCP / 6번 관절 쪽 끝) + FK 검산

키
  U  Unlock  실물 토크 OFF · 손 티칭 → 시뮬 동기화
  L  Lock    현재 티칭 자세 그대로 홀드 (튕김 없음)
  Space / T  손끝 xyz 저장 (터미널 + 뷰어 우하단 + taught_points.json)
  P          목록 출력
  Backspace  목록 비우기
"""
from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from soarm_lab import SCENE
from soarm_lab.fk_core import FKSo101
from soarm_lab.driver_sdk import (
    STS3215Driver,
    CALIBRATION_FILE,
    CALIBRATION_LABEL,
    JOINT_LIMITS,
)

PORT = "/dev/ttyACM1"
POINTS_FILE = Path(__file__).resolve().parent / "taught_points.json"

_KEY_SPACE = 32
_KEY_BACKSPACE = 259
_UNLOCK_FLAGS = (
    int(mujoco.mjtDisableBit.mjDSBL_ACTUATION)
    | int(mujoco.mjtDisableBit.mjDSBL_GRAVITY)
)

_ui = threading.Lock()
_samples: list[dict] = []
_pending = None
_joints_locked = True
_held_q = np.zeros(6)          # Lock 중 유지할 각도(도)
_viewer_ref = None             # overlay 갱신용


# ── 실물 I/O ──────────────────────────────────────────────────────────────────

def _connect_robot(port: str) -> STS3215Driver:
    drv = STS3215Driver(port=port)
    if not drv.connect():
        raise SystemExit(
            f"[연결 실패] {port} 를 열 수 없습니다.\n"
            "  · USB / 전원 확인 · 다른 앱(app.py 등)이 포트 쓰는지 확인"
        )
    offline = [sid for sid in range(1, 7) if not drv.ping(sid)]
    if offline:
        drv.disconnect()
        raise SystemExit(f"[연결 실패] 서보 ping 실패: id={offline}")
    print(f"[연결 OK] {port} · 서보 1–6 ping 성공")
    return drv


def _print_calibration():
    print(f"[캘리브] 파일: {CALIBRATION_FILE}")
    if not CALIBRATION_FILE.exists():
        print("  ⚠ calibration.json 없음")
        return
    print(f"  로드됨: robot='{CALIBRATION_LABEL}'")
    for jid in range(1, 7):
        lim = JOINT_LIMITS.get(jid, {})
        print(f"  id{jid}: min={lim.get('min')} max={lim.get('max')} "
              f"homing_offset={lim.get('homing_offset', '-')}")


def _read_q_deg(drv: STS3215Driver) -> np.ndarray:
    pos = drv.get_all_positions()
    out = []
    for i in range(1, 7):
        d = STS3215Driver.position_to_degrees(pos.get(i))
        out.append(float("nan") if d is None else float(d))
    return np.asarray(out, float)


def _valid_arm(q: np.ndarray) -> bool:
    return q is not None and len(q) >= 5 and not np.isnan(q[:5]).any()


def _apply_pose_to_sim(model, data, q_deg: np.ndarray):
    """시뮬 qpos·ctrl을 같은 각도로 맞춤 (액추에이터 목표 = 현재 자세)."""
    n = min(6, model.nq, model.nu, len(q_deg))
    rad = np.radians(q_deg[:n])
    data.qpos[:n] = rad
    data.ctrl[:n] = rad
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)


def _write_servo_goals(drv: STS3215Driver, q_deg: np.ndarray):
    """토크와 무관하게 goal position만 현재각으로 갱신."""
    for i in range(5):
        if not np.isnan(q_deg[i]):
            drv.set_position(i + 1, STS3215Driver.degrees_to_position(q_deg[i]))
            time.sleep(0.002)


# ── Lock / Unlock ─────────────────────────────────────────────────────────────

def _set_locked(model, data, drv: STS3215Driver, locked: bool, last_q: np.ndarray):
    """Lock 시 현재(last_q) 자세를 먼저 goal/ctrl에 박고 나서 토크·액추에이터 ON.
    (순서 반대로 하면 ctrl=0 또는 옛 목표로 ㄱ자 튕김)"""
    global _joints_locked, _held_q

    if locked:
        q = _read_q_deg(drv)
        if not _valid_arm(q):
            q = last_q.copy()
            print("(Lock) 시리얼 읽기 실패 → 마지막 동기 각도 사용")
        _held_q = q.copy()

        # 1) 목표를 현재 자세로 먼저 설정
        _write_servo_goals(drv, _held_q)
        _apply_pose_to_sim(model, data, _held_q)
        time.sleep(0.03)

        # 2) 그 다음 구동 ON
        model.opt.disableflags &= ~_UNLOCK_FLAGS
        drv.set_all_torque(True)
        _joints_locked = True
        print("\n[LOCK] 현재 자세 홀드 (튕김 방지: goal←현재각 후 토크 ON)\n")
        return _held_q.copy()

    # Unlock
    model.opt.disableflags |= _UNLOCK_FLAGS
    drv.set_all_torque(False)
    data.qvel[:] = 0.0
    _joints_locked = False
    print("\n[UNLOCK] 실물 토크 OFF · 손으로 티칭 → 시뮬 동기화\n")
    return last_q


# ── Overlay (뷰어 우하단) ─────────────────────────────────────────────────────

def _overlay_lines() -> str:
    with _ui:
        pts = list(_samples)
    if not pts:
        return "(Space: 좌표 저장)"
    lines = []
    for s in pts:
        x, y, z = s["xyz"]
        lines.append(f"{s['name']}: [{x:.3f}, {y:.3f}, {z:.3f}]")
    return "\n".join(lines)


def _refresh_overlay(viewer):
    if viewer is None:
        return
    try:
        viewer.set_texts([(
            None,
            mujoco.mjtGridPos.mjGRID_BOTTOMRIGHT,
            "TCP points (m)",
            _overlay_lines(),
        )])
    except Exception as exc:
        print(f"(overlay 갱신 실패: {exc})")


# ── 샘플 / 출력 ───────────────────────────────────────────────────────────────

def _tip_from_sim(model, data, sid) -> np.ndarray:
    mujoco.mj_forward(model, data)
    return data.site_xpos[sid].copy()


def _on_key(keycode):
    global _pending
    if keycode in (_KEY_SPACE, ord("T"), ord("t")):
        _pending = "sample"
    elif keycode in (ord("P"), ord("p")):
        _pending = "print"
    elif keycode == _KEY_BACKSPACE:
        _pending = "clear"
    elif keycode in (ord("L"), ord("l")):
        _pending = "lock"
    elif keycode in (ord("U"), ord("u")):
        _pending = "unlock"


def _label(i):
    return "ABC"[i] if i < 3 else f"P{i + 1}"


def _announce(s: dict):
    x, y, z = s["xyz"]
    print()
    print("=" * 52)
    print(f"  ★ 점 {s['name']}  손끝 좌표 (gripperframe TCP)")
    print(f"     → [{x:.4f}, {y:.4f}, {z:.4f}]  m")
    print(f"  q_deg : {np.round(s['q_deg'], 1).tolist()}")
    print("=" * 52)
    print()


def _save_file():
    with _ui:
        pts = list(_samples)
    payload = {
        "tcp": "gripperframe",
        "port": PORT,
        "calibration": CALIBRATION_LABEL,
        "points": {
            s["name"]: {
                "xyz": [round(float(v), 4) for v in s["xyz"]],
                "fk_xyz": [round(float(v), 4) for v in s["fk_xyz"]],
                "q_deg": [round(float(v), 1) for v in s["q_deg"]],
            }
            for s in pts
        },
    }
    POINTS_FILE.write_text(json.dumps(payload, indent=2, ensure_ascii=False))
    print(f"(저장됨: {POINTS_FILE.name})")


def _print_summary():
    with _ui:
        pts = list(_samples)
    if not pts:
        print("(저장된 점 없음)")
        return
    print("\n######## 저장된 손끝 좌표 ########")
    for s in pts:
        x, y, z = s["xyz"]
        print(f"  {s['name']}: [{x:.4f}, {y:.4f}, {z:.4f}]  m")
    print("POINTS = {")
    for s in pts:
        x, y, z = s["xyz"]
        print(f'    "{s["name"]}": [{x:.4f}, {y:.4f}, {z:.4f}],')
    print("}\n")


def main():
    global _pending, _viewer_ref, _held_q

    print("=== 실물↔시뮬 티칭 ===")
    _print_calibration()
    drv = _connect_robot(PORT)

    model = mujoco.MjModel.from_xml_path(SCENE)
    data = mujoco.MjData(model)
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")
    if sid < 0:
        raise SystemExit("site 'gripperframe' 없음")
    fk = FKSo101()

    q0 = _read_q_deg(drv)
    if not _valid_arm(q0):
        raise SystemExit("[연결 실패] 관절각을 읽지 못했습니다")
    print("실물 현재각(deg):", np.round(q0, 1).tolist())
    last_q = q0.copy()
    _held_q = q0.copy()
    _apply_pose_to_sim(model, data, q0)
    last_q = _set_locked(model, data, drv, True, last_q)

    print("키: U=Unlock  L=Lock  Space=좌표저장  P=목록  Backspace=비우기\n")

    SYNC_DT = 0.1
    last_sync = 0.0

    try:
        with mujoco.viewer.launch_passive(model, data, key_callback=_on_key) as v:
            _viewer_ref = v
            _refresh_overlay(v)

            while v.is_running():
                now = time.time()

                if not _joints_locked:
                    if now - last_sync >= SYNC_DT:
                        q = _read_q_deg(drv)
                        if _valid_arm(q):
                            last_q = q
                            _apply_pose_to_sim(model, data, q)
                        last_sync = now
                else:
                    # Lock: 매 프레임 ctrl을 홀드각에 고정 → 슬라이더/드리프트로 ㄱ자 복귀 방지
                    _apply_pose_to_sim(model, data, _held_q)
                    mujoco.mj_step(model, data)

                v.sync()

                action = None
                with _ui:
                    if _pending:
                        action, _pending = _pending, None

                if action == "unlock":
                    last_q = _set_locked(model, data, drv, False, last_q)
                    _refresh_overlay(v)
                elif action == "lock":
                    last_q = _set_locked(model, data, drv, True, last_q)
                    _refresh_overlay(v)
                elif action == "sample":
                    q = _read_q_deg(drv)
                    if not _valid_arm(q):
                        q = last_q.copy()
                        print("(시리얼 읽기 실패 → 마지막 동기 각도 사용)")
                    else:
                        last_q = q
                    if _joints_locked:
                        _held_q = q.copy()
                    _apply_pose_to_sim(model, data, q)
                    xyz = _tip_from_sim(model, data, sid)
                    fk_xyz, _ = fk.fk_deg(q[:5])
                    with _ui:
                        name = _label(len(_samples))
                        s = {
                            "name": name,
                            "xyz": xyz,
                            "fk_xyz": np.asarray(fk_xyz, float),
                            "q_deg": q[:5].copy(),
                        }
                        _samples.append(s)
                    _announce(s)
                    _save_file()
                    _refresh_overlay(v)
                    if len(_samples) == 3:
                        _print_summary()
                elif action == "print":
                    _print_summary()
                    _refresh_overlay(v)
                elif action == "clear":
                    with _ui:
                        _samples.clear()
                    if POINTS_FILE.exists():
                        POINTS_FILE.unlink()
                    print("(목록 비움)\n")
                    _refresh_overlay(v)

                time.sleep(0.01 if not _joints_locked else model.opt.timestep)
    finally:
        try:
            q = _read_q_deg(drv)
            if not _valid_arm(q):
                q = last_q
            _write_servo_goals(drv, q)
            drv.set_all_torque(True)
        except Exception:
            pass
        drv.disconnect()
        print("[연결 종료]")

    if _samples:
        _print_summary()


if __name__ == "__main__":
    main()
