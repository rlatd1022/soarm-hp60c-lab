import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from vision.sort_by_color import find_sort_plan


def _scene():
    image = np.full((480, 640, 3), 255, dtype=np.uint8)
    # 대상 영역은 일부러 서로 다른 위치에 둔다. 위치가 고정값이면 이 테스트가 실패한다.
    cv2.rectangle(image, (40, 60), (180, 220), (0, 0, 255), 5)
    cv2.rectangle(image, (430, 260), (600, 430), (255, 0, 0), 5)
    cv2.circle(image, (290, 150), 23, (0, 0, 255), -1)
    cv2.circle(image, (210, 350), 21, (255, 0, 0), -1)
    return image


def test_find_sort_plan_matches_each_ball_to_its_shifted_color_zone():
    plan = find_sort_plan(_scene())

    assert set(plan) == {"red", "blue"}
    assert plan["red"].ball.center == (290, 150)
    assert plan["blue"].ball.center == (210, 350)
    assert plan["red"].target.center == (110, 140)
    assert plan["blue"].target.center == (515, 345)


def test_find_sort_plan_ignores_color_when_only_a_ball_or_only_a_zone_exists():
    image = _scene()
    cv2.rectangle(image, (430, 260), (600, 430), (255, 255, 255), -1)

    plan = find_sort_plan(image)

    assert set(plan) == {"red"}
