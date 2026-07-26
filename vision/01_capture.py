# -*- coding: utf-8 -*-
"""01_capture.py — 공 사진 저장 (직접 작성 · 강사와 같이 따라치기).

목표: 브리지를 켠 뒤 카메라 화면을 보며 s 로 shots/ 에 사진을 저장한다.
      빨강·파랑·조명별로 몇 장 찍어두면 다음(02·hsv)에서 임계값 맞추기 쉽다.

힌트:
  from hp60c_camera import CameraReader
  with CameraReader() as cam:
      rgb, depth, last = cam.read_blocking(last)   # rgb = BGR uint8
  cv2.imshow(...) / cv2.waitKey(1) / cv2.imwrite(path, rgb)
키: s=저장 · q=종료
"""
import os

import cv2
from hp60c_camera import CameraReader

OUT = "shots"


def main():
    os.makedirs(OUT, exist_ok=True)
    # TODO (구현 순서):
    #  1) with CameraReader() as cam:  루프에서 rgb, _d, last = cam.read_blocking(last)
    #  2) cv2.imshow 로 rgb 표시, k = cv2.waitKey(1) & 0xFF
    #  3) k == ord('s') 이면 cv2.imwrite(f"{OUT}/shot_{n:03d}.png", rgb) 후 번호 증가
    #  4) k == ord('q') 이면 종료
    raise NotImplementedError("위 순서로 캡처 루프를 구현하세요")


if __name__ == "__main__":
    main()
