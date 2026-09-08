import time
import numpy as np
from soarm_lab.ik_core import IKSo101
from soarm_lab.real import RealBackend

ik = IKSo101()
robot = RealBackend(port="/dev/ttyACM1")

deg, err = ik.solve([0.25, 0.0, 0.15])
print("각도", np.round(deg, 1), "잔차(mm):", round(err * 1000, 1))

robot.move(deg)
time.sleep(2)
