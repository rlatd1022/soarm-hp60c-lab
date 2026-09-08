from soarm_lab.grasp_scene import build, BALL_XY, BALL_R
import numpy as np, mujoco, mujoco.viewer

model, data = build(basket_xy=(0.16, -0.12)) # 공 + 바구니 씬
bid = model.body("obj").id
print("공 위치:", data.xpos[bid])            # [0.24, 0.05, 0.02]

mujoco.viewer.launch(model, data)            # 눈으로 확인
