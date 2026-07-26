# -*- coding: utf-8 -*-
"""02_detect.py — HSV로 공 검출 → 공의 픽셀(u,v). (직접 구현)

목표: 카메라(또는 저장 사진)에서 빨강/파랑 공을 찾아 중심 픽셀에 표시.
방법: BGR→HSV → inRange 로 색 마스크 → 가장 큰 덩어리의 중심(minEnclosingCircle).
임계값: hsv_tuner 로 뽑은 6개 숫자를 넣는다. 빨강은 색상환 0·180 양끝 두 구간 OR.

힌트: cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV), cv2.inRange, cv2.bitwise_or,
      cv2.findContours(RETR_EXTERNAL), cv2.contourArea, cv2.minEnclosingCircle
      저장 사진으로도: python 02_detect.py shots/shot_000.png
키: q=종료
"""
import sys

import cv2

# TODO: hsv_tuner 로 찾은 값으로 바꾸기
RED1_LO, RED1_HI = (0, 100, 80), (10, 255, 255)
RED2_LO, RED2_HI = (160, 100, 80), (179, 255, 255)
BLUE_LO, BLUE_HI = (95, 120, 60), (130, 255, 255)


def detect_ball(bgr, color):
    """색 마스크에서 가장 큰 덩어리 → 중심 픽셀 (u, v). 없으면 None."""
    # TODO (구현 순서):
    #  1) hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    #  2) color=='red' 면 두 구간 inRange 를 cv2.bitwise_or, 아니면 BLUE 한 구간
    #  3) cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #  4) 가장 큰 컨투어(면적이 충분히 크면) → cv2.minEnclosingCircle 중심 반환
    return None


def main():
    # TODO (구현 순서):
    #  1) 인자가 있으면 cv2.imread 로 사진들, 없으면 CameraReader 루프
    #  2) 각 프레임에서 red·blue 를 detect_ball 로 찾아 원+중심+라벨 그리기
    #  3) cv2.imshow 표시, q 로 종료
    raise NotImplementedError("detect_ball 과 main 루프를 구현하세요")


if __name__ == "__main__":
    main()
