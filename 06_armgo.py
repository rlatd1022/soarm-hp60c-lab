from soarm_lab import arm

arm.live()                  # 라이브 창 띄우기
arm.go([0.25, 0.0, 0.15])   # 손끝을 이 좌표로
arm.go([0.20, 0.12, 0.10], grip=90)  # 좌표 + 그리퍼 90도
arm.wait()                  # 창을 열어 둔 채 대기
