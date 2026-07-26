# -*- coding: utf-8 -*-
"""03_map.py — 4점 호모그래피: 픽셀 <-> 로봇 좌표.

작업면 네 곳에 공을 두고, 각 지점에서 화면 클릭 픽셀과 로봇 xy 를 짝지어 모은다.
로봇 xy 는 토크를 끄고 팔끝을 공에 댄 뒤 관절각을 FK 로 읽어 얻는다.
결과 H 는 data/H.npy 로 저장되어 04_click_move / 05_track 이 불러 쓴다.
포트 /dev/ttyACM0 단독(컨트롤러 앱·다른 프로세스는 종료).
"""
import os

import cv2
import numpy as np
from hp60c_camera import CameraReader
from soarm_lab.driver_sdk import STS3215Driver
from soarm_lab.fk_core import FKSo101

N = 4
PORT = "/dev/ttyACM0"
OUT = os.path.join("data", "H.npy")


def click_pixel(cam):
    win = "click the ball (q=cancel)"
    got = {}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            got["uv"] = (x, y)

    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    last = 0
    try:
        while "uv" not in got:
            rgb, _d, last = cam.read_blocking(last)
            if rgb is None:
                continue
            cv2.imshow(win, rgb)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                raise KeyboardInterrupt
        return got["uv"]
    finally:
        cv2.destroyWindow(win)


def robot_xy(drv, fk):
    for sid in (1, 2, 3, 4, 5):
        drv.set_torque(sid, False)
    input("  팔끝을 공에 대고 Enter > ")
    pos = drv.get_all_positions()
    deg = [STS3215Driver.position_to_degrees(pos.get(i + 1)) or 0.0 for i in range(5)]
    p, _ = fk.fk_deg(deg)
    return float(p[0]), float(p[1])


def main():
    drv = STS3215Driver(port=PORT)
    drv.connect()
    fk = FKSo101()
    pixels, robots = [], []
    try:
        with CameraReader() as cam:
            for i in range(N):
                print(f"[{i + 1}/{N}] 공을 새 위치에 놓고(작업면 넓게), 화면에서 공을 클릭.")
                uv = click_pixel(cam)
                xy = robot_xy(drv, fk)
                print(f"  픽셀 {uv} -> 로봇 ({xy[0]:+.3f}, {xy[1]:+.3f})")
                pixels.append(uv)
                robots.append(xy)
    except KeyboardInterrupt:
        print("취소 — 저장하지 않음")
        return
    finally:
        cv2.destroyAllWindows()

    H = cv2.getPerspectiveTransform(np.float32(pixels), np.float32(robots))

    worst = 0.0
    for (u, v), (x, y) in zip(pixels, robots):
        px, py = cv2.perspectiveTransform(np.float32([[[u, v]]]), H)[0, 0]
        worst = max(worst, ((px - x) ** 2 + (py - y) ** 2) ** 0.5)
    print(f"재투영 최대오차 {worst * 1000:.1f}mm",
          "(양호)" if worst < 0.01 else "(큼 — 4점을 더 넓게 재측정)")

    os.makedirs("data", exist_ok=True)
    np.save(OUT, H)
    print("저장:", OUT)


if __name__ == "__main__":
    main()
