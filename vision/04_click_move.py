# -*- coding: utf-8 -*-
"""04_click_move.py — 화면을 클릭하면 그 지점으로 로봇 이동. (직접 구현)

목표: 클릭한 픽셀을 H(03_map 이 만든 data/H.npy)로 로봇 (x,y)로 바꿔 arm.go 한 번.
      연속 아니라 클릭할 때 한 번씩만 움직여 안전. 먼저 03_map.py 로 H.npy 를 만든다.

핵심 5줄:
  H = np.load("data/H.npy")
  # 마우스 클릭 (u, v)
  x, y = cv2.perspectiveTransform(np.float32([[[u, v]]]), H)[0, 0]
  arm.go([float(x), float(y), Z_FIXED], real=REAL, down=False)
실물 속도: arm._backend(True).drv.set_speed(sid, SPEED) / set_acceleration(sid, ACC)  (1~5)

⚠️ 클릭하면 팔이 움직인다. 작업면에 손 넣지 말 것. 처음엔 REAL=False.
"""
import os

import cv2
import numpy as np
from hp60c_camera import CameraReader
from soarm_lab import arm

Z_FIXED = 0.12
SPEED = 600            # 작을수록 느림
ACC = 20
REAL = True


def main():
    H = np.load(os.path.join("data", "H.npy"))
    # TODO (구현 순서):
    #  1) REAL 이면 drv = arm._backend(True).drv 로 set_acceleration/set_speed (id 1~5)
    #  2) cv2.setMouseCallback 로 클릭한 (u, v) 저장
    #  3) 카메라 루프: 클릭이 있으면 x, y = cv2.perspectiveTransform(np.float32([[[u,v]]]), H)[0,0]
    #  4) arm.go([float(x), float(y), Z_FIXED], real=REAL, down=False)  (try/except 로 도달밖 스킵)
    raise NotImplementedError("클릭→이동 루프를 구현하세요")


if __name__ == "__main__":
    main()
