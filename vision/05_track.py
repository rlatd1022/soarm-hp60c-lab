# -*- coding: utf-8 -*-
"""05_track.py — 빨간 공을 로봇이 따라간다 (2D 추적). (직접 구현 · 도전)

목표: 검출 → H로 로봇좌표 → hover 높이로 이동, 매 프레임 반복.
      마진(데드밴드)으로 미세 떨림엔 명령을 생략해 팔을 보호한다.
      먼저 03_map.py 로 data/H.npy 를 만든다.

재료: 02 의 검출(빨강 픽셀) + 04 의 픽셀→좌표(perspectiveTransform) + arm.go.
      MARGIN: 직전 목표에서 이 거리(m) 이상 움직였을 때만 새로 명령.

⚠️ 팔이 공을 계속 따라간다. 작업면에 손 넣지 말 것. 처음엔 REAL=False.
"""
import os

import cv2
import numpy as np
from hp60c_camera import CameraReader
from soarm_lab import arm

Z_HOVER = 0.12
MARGIN = 0.01
SPEED = 600
ACC = 20
REAL = True


def main():
    H = np.load(os.path.join("data", "H.npy"))
    # TODO (구현 순서):
    #  1) REAL 이면 서보 속도 낮추기 (04 와 동일)
    #  2) 빨강 검출은 02 의 색마스크(빨강 두 구간 OR)를 재사용 → 픽셀 (u,v)
    #  3) x, y = cv2.perspectiveTransform(np.float32([[[u,v]]]), H)[0,0]
    #  4) 직전 목표와의 거리가 MARGIN 초과일 때만 arm.go([x,y,Z_HOVER], real=REAL, down=False)
    raise NotImplementedError("트래킹 루프를 구현하세요")


if __name__ == "__main__":
    main()
