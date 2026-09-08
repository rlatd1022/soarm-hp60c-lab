# -*- coding: utf-8 -*-
"""08_pick_place.py — 손끝·공·바구니 좌표를 읽고, 공을 집어 바구니에 넣는다.

리플레이: 뷰어에서 R 또는 Backspace → 초기 상태로 되돌린 뒤 pick & place 재실행.
(MuJoCo UI의 Reset은 물리만 되돌리고 스크립트는 다시 안 돔 → 이 키로 대체)
"""
import threading
import time
import mujoco
from soarm_lab import arm
from soarm_lab.grasp import approach_xy
from soarm_lab.grasp_scene import BALL_R

BASKET = (0.16, -0.12)
_KEY_BACKSPACE = 259

_replay = threading.Event()
_busy = threading.Event()  # pick & place 중이면 키 무시


def make_states(ball_xy, basket_xy, r=BALL_R):
    """공·바구니 좌표 → pick & place 순서표."""
    hx, hy = approach_xy(ball_xy,   r=r, margin=0.014)   # 하강점(공 옆, 여유)
    gx, gy = approach_xy(ball_xy,   r=r, margin=0.0)     # 파지점(공에 붙임)
    px, py = approach_xy(basket_xy, r=r, margin=0.0)     # 놓을 곳
    return [
        ("hover",   [hx, hy, 0.12],  100),   # 공 옆 위
        ("descend", [hx, hy, 0.005], 100),   # 옆으로 하강(공 안 누름)
        ("slide",   [gx, gy, 0.005], 100),   # 파지점으로 슬라이드
        ("close",   None,             15),   # 닫기(공 크기에 맞춤)
        ("lift",    [gx, gy, 0.14],   15),   # 들기
        ("carry",   [px, py, 0.14],   15),   # 바구니 위로
        ("lower",   [px, py, 0.07],   15),   # 내리기
        ("open",    None,            100),   # 놓기
    ]


def _on_key(keycode):
    """R / r / Backspace → 리플레이 요청."""
    if _busy.is_set():
        return
    if keycode in (ord("R"), ord("r"), _KEY_BACKSPACE):
        _replay.set()


def _snapshot(data):
    return {
        "qpos": data.qpos.copy(),
        "qvel": data.qvel.copy(),
        "ctrl": data.ctrl.copy(),
        "act":  data.act.copy() if data.act.size else None,
        "time": float(data.time),
    }


def _restore(model, data, snap):
    data.qpos[:] = snap["qpos"]
    data.qvel[:] = snap["qvel"]
    data.ctrl[:] = snap["ctrl"]
    if snap["act"] is not None and data.act.size:
        data.act[:] = snap["act"]
    data.time = snap["time"]
    mujoco.mj_forward(model, data)


def print_coords():
    """기계팔 손끝 · 빨간 공 · 바구니 월드 좌표를 출력."""
    b = arm._backend(False)
    mujoco.mj_forward(b.model, b.data)

    ee = b.ee()                                      # gripperframe site
    ball = b.data.xpos[b.model.body("obj").id].copy()
    basket = b.data.xpos[b.model.body("basket").id].copy()

    print("손끝 (gripperframe):", ee.round(4))
    print("빨간 공 (obj):      ", ball.round(4))
    print("바구니 (basket):    ", basket.round(4))
    return ee, ball, basket


def run_once():
    """좌표 출력 → pick & place → 결과 출력."""
    _busy.set()
    try:
        print_coords()
        xy = arm.ball_xy()
        print("공 xy (pick용):", xy.round(3))
        arm.run(make_states(xy, BASKET), down=True)

        b = arm._backend(False)
        mujoco.mj_forward(b.model, b.data)
        ball_after = b.data.xpos[b.model.body("obj").id].copy()
        print("놓기 후 공 위치:", ball_after.round(4))
        print("→ R 또는 Backspace: 리플레이 / 창 닫기: 종료")
    finally:
        _busy.clear()


def wait_with_replay(snap):
    """창을 연 채 대기. 리플레이 키가 오면 초기화 후 재실행."""
    b = arm._backend(False)
    if not b.viewer:
        return
    while b.viewer.is_running():
        if _replay.is_set():
            _replay.clear()
            print("\n[리플레이] 초기 상태로 되돌린 뒤 다시 실행합니다…")
            _restore(b.model, b.data, snap)
            b.viewer.sync()
            run_once()
        b.viewer.sync()
        time.sleep(0.02)


if __name__ == "__main__":
    arm.grasp(basket_xy=BASKET, key_callback=_on_key)
    b = arm._backend(False)
    snap = _snapshot(b.data)          # 시작 상태 저장
    run_once()
    wait_with_replay(snap)
