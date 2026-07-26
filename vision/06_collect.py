# -*- coding: utf-8 -*-
"""06_collect.py — 배경차분으로 굴러온 공을 자동 수집 → collected/<색>/. (대회 · 자유)

목표: 고정 ROI 에서 배경차분(MOG2)으로 움직이는 공을 잡아 크롭·색분류(빨강/파랑) 저장.
      대회용 데이터셋 모으기. 카메라만 쓴다.

힌트: cv2.createBackgroundSubtractorMOG2, 워밍업 동안 빈 배경 학습,
      threshold+morphology→findContours→면적 필터, 크롭을 HSV로 빨강/파랑 분류.
키: q=종료 · r=배경 리셋 · s=저장 on/off
"""
import os

import cv2
from hp60c_camera import CameraReader

ROI = (200, 150, 460, 340)     # x1,y1,x2,y2 — 공이 지나갈 영역
OUT = "collected"


def main():
    # TODO (자유 · AI 활용+검수 · 구현 순서):
    #  1) cv2.createBackgroundSubtractorMOG2 생성, ROI 크롭에 warmup 동안 배경 학습
    #  2) 이후 foreground → threshold + morphology → findContours → 면적 필터
    #  3) 박스 크롭을 HSV 로 빨강/파랑 분류 → collected/<색>/ 에 저장
    raise NotImplementedError("배경차분 수집 루프를 구현하세요")


if __name__ == "__main__":
    main()
