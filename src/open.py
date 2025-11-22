#!/bin/python3

import math
import rclpy
import gpiozero
import robot_car
import rclpy.node
import sensor_msgs.msg

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

FOR_WALL = 0
FIX_GYRO_1 = 1
FIX_GYRO_2 = 2

CW_TRUST_SCORE = .3
CCW_TRUST_SCORE = .5

class miav(rclpy.node.Node):
    def __init__(self):
        super().__init__("miav")
        
        qos = QoSProfile(depth=10,reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(sensor_msgs.msg.LaserScan, '/scan', self.scan_callback, qos)
        self.create_timer(1.0 / 20.0, self.miav_callback)

        self.count = 0
        self.dir = None
        self.exec = None
        self.f_dir = False
        self.waitin = None
        self.pt_one = None
        self.pt_two = None
        self.idle_speed = 30
        self.start_pos = None

        robot_car.read_encoder()

    def scan_callback(self, msg):
        if self.exec is not None:
            return

        gyro_angle = robot_car.read_gyro()
        scan, rad = [], msg.angle_min
        for dis in msg.ranges:
            deg = math.degrees(rad) + 90
            if deg >= 180: deg -= 360
            deg = gyro_angle + deg
            dis *= 100
            if dis != 0 and math.isfinite(dis) and -130 <= deg <= 130 and dis >= 13:
                scan.append((deg, dis))
            rad += msg.angle_increment
        scan.sort()

        if not self.f_dir:
            for_dis = robot_car.find(min, scan, -3, 3)
            if for_dis is None or for_dis > 80:
                gyro_angle = robot_car.read_gyro()
                gyro_error = -gyro_angle
                robot_car.move(head = gyro_error * 2, speed = 60)
            else:
                left = robot_car.find(max, scan, -90, -10)
                right = robot_car.find(max, scan, 10, 90)
                if left is not None and (right is None or left > right):
                    self.dir = -90
                else:
                    self.dir = 90
                self.f_dir = True

        if not self.f_dir:
            return

        if self.dir == 90:
            self.side_dis = robot_car.find(min, scan, -100, -80)
        elif self.dir == -90:
            self.side_dis = robot_car.find(min, scan, 80, 100)

        if self.waitin is not None:
            self.exec, self.waitin = self.waitin, None
            return

        for_dis = robot_car.find(min, scan, -3, 3)
        var = for_dis is None or (self.count == 12 and for_dis > 180) or (self.count != 12 and \
                ((self.dir == 90 and for_dis > 60) or (self.dir == -90 and for_dis > 45)))
        if var:
            if for_dis is not None:
                if self.count != 12:
                    deaccel_speed = robot_car.calc_ccel(60, 80, for_dis, 170)
                else:
                    deaccel_speed = robot_car.calc_ccel(50, 60, for_dis, 100)
            else:
                deaccel_speed = 50
            self.idle_speed = deaccel_speed
        else:
            if self.count == 12:
                robot_car.move(0, 0)
                raise Exception
            else:
                robot_car.shift_angle -= self.dir
                self.exec = FOR_WALL
                self.count += 1

    def miav_callback(self):
        if robot_car.height_rot() > 10:
            robot_car.move(0, 0)
            raise Exception

        if self.exec is None:
            err = -robot_car.read_gyro()
            robot_car.move(err * 1.5, self.idle_speed)
        if self.exec is None: return
        if self.exec == FOR_WALL:
            gyro_angle = robot_car.read_gyro()
            gyro_error = -gyro_angle
            if not -5 <= gyro_error <= 5:
                gyro_angle = robot_car.read_gyro()
                gyro_error = -gyro_angle
                if self.dir == 90:
                    robot_car.move(head = gyro_error, speed = 30)
                else:
                    robot_car.move(head = gyro_error, speed = 30)
            else:
                self.exec = None
                self.waitin = FIX_GYRO_1
                robot_car.move(0, 0)
                print('turned')
        elif self.exec == FIX_GYRO_1:
            if self.pt_one is None:
                self.pt_one = self.side_dis
            if self.pt_one is not None:
                if self.start_pos is None and robot_car.pos is not None:
                    self.start_pos = robot_car.pos
                if robot_car.pos is None or robot_car.rot_to_cm(robot_car.pos - self.start_pos) < 20:
                    err = -robot_car.read_gyro()
                    robot_car.move(err * 2, 30)
                else:
                    self.exec = None
                    self.start_pos = None
                    robot_car.move(0, 0)
                    self.waitin = FIX_GYRO_2
                    print('fix_1')
        elif self.exec == FIX_GYRO_2:
            if self.pt_two is None:
                self.pt_two = self.side_dis
                slope = (self.pt_two - self.pt_one) / 20
                angle = math.degrees(math.atan(slope)) - robot_car.read_gyro()
                if self.dir == -90: angle = -angle + 4
                print('shift angle', angle)
                if self.dir == 90:
                    angle *= CW_TRUST_SCORE
                elif self.dir == -90:
                    angle *= CCW_TRUST_SCORE
                robot_car.shift_angle += angle
                robot_car.move(0, 0)
                self.pt_one = None
                self.pt_two = None
                self.exec = None
                print('fix_2')

b = gpiozero.Button(23)

def main():
    rclpy.init()
    node = miav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    #b.wait_for_press()
    main()
