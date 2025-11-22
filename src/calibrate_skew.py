import math
import time
import robot_car

def dis_to_ang(dis):
    res = A * math.sqrt(abs(dis)) + B
    return dis / abs(dis) * (A * math.sqrt(abs(dis)) + B)

A, B = 13.3, -21

ang = dis_to_ang(int(input()))

while robot_car.read_gyro() < ang:
    robot_car.move(90, 45)
    time.sleep(0.05)

robot_car.hard_brake()

while abs(robot_car.read_gyro()) > 2:
    robot_car.move(-robot_car.read_gyro() * 3, 20)
    time.sleep(0.05)

robot_car.hard_brake()

while robot_car.ultra.distance * 100 > 10:
    robot_car.move(robot_car.read_gyro() + 3, -30)
    time.sleep(0.05)

robot_car.hard_brake()
