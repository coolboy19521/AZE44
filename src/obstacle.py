import cv2
import math
import time
import numpy
import gpiozero
import robot_car
import rclpy.node
import sensor_msgs.msg

from rclpy.qos import QoSProfile, QoSReliabilityPolicy

LEFT_BACK = (31.2637317, 16.3783393542)
ROBOT_LENGTH = 22

START = 0
KUBIK_UC = 1
KUBIK_TC = 2
BASE_WALL = 3
FOR_WALL = 4
NARROW = 5
SKEW = 6
SCREW_RIGHT = 7
PUT_BAGS = 8
ADJUST_DIS = 9
FIN_1 = 10
FIN_2 = 11
FIX_GYRO_1 = 12
FIX_GYRO_2 = 13

SKEW_RED = 14
GO_BACK_RED = 15
PUT_BAGS_RED = 16
ADJUST_DIS_RED = 17
SCREW_LEFT_RED = 18

GREEN = 0
RED = 1

class miav(rclpy.node.Node):
    def __init__(self):
        super().__init__('miav')
        qos = QoSProfile(depth=10,reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(sensor_msgs.msg.LaserScan, '/scan', self.scan_callback, qos)
        self.create_subscription(sensor_msgs.msg.CompressedImage, '/camera/image_raw/compressed', self.camera_callback, 1)
        self.create_timer(1 / 20, self.miav_callback)

        self.dir = None
        self.f_dir = False

        self.off_x = 35
        self.off_y = 15

        self.color = None
        self.colors = {
            'green': [((70, 128, 75), (86, 255, 170)), ((67, 106, 22), (78, 188, 182)), ((69, 51, 33), (95, 193, 154)), ((59, 93, 58), (79, 229, 147)), ((18, 27, 88), (97, 188, 156))],
            'red': [((0, 154, 160), (179, 206, 235)), ((0, 113, 0), (8, 255, 255)), ((0, 40, 124), (1, 219, 196)), ((170, 76, 159), (180, 255, 215))],
        }

        self.e = None

        self.kubik = None
        self.turned = False
        self.narrow = False
        self.wall_dis = None
        self.kubik_pos = None
        self.swing_ang = None
        self.start_pos = None
        self.skew_done = False
        self.manoeuvre = False
        self.color_val = False
        self.target_dis = None
        self.gone_back = False
        self.get_close = False
        self.face_kubik = False
        self.last_back_dis = None

        self.exec = START
        self.waitin = None
        self.time = None

        self.idle = False
        self.idle_speed = None

        robot_car.read_encoder()

        self.kubiks = []
        self.kubik_ix = -1
        self.last_kubik = None
        self.last_dis = None
        self.last_side = None
        self.last_ok = False
        self.on_this_side = 0
        self.seen_start = False
        self.lap_cnt = 0

        self.start_time = None

        self.count = 0

        self.park_mode = False

        self.mov_park = 0
        self.initial = None
        self.dis_till = None
        self.entr_ang = None
        self.back_dis = None
        self.left_dis = None
        self.right_dis = None
        self.front_dis = None

        self.last_color = RED
        self.d = True

        self.pt_one = None
        self.pt_two = None

    def find_katets(self, point):
        deg, hyp = point
        beta = math.radians(deg)
        adj = math.cos(beta) * hyp
        opp = math.sin(beta) * hyp
        return adj, opp

    def find_park(self, scan):
        ly1, ly2, lx1, lx2 = None, None, None, None
        for p in scan:
            var = (self.dir == 90 and (p[0] < -90 or p[0] > 30))
            if var:
                continue
            y_dis = abs(p[1] * math.sin(math.radians(90 + p[0])))
            x_dis = abs(p[1] * math.cos(math.radians(90 + p[0])))
            if y_dis > 100: return None
            if ly1 is None or abs(y_dis - ly1) > 3:
                if lx2 is not None and abs(lx2 - lx1) > 3:
                    return (lx2, ly1)
                ly1, ly2, lx1, lx2 = y_dis, y_dis, x_dis, x_dis
            else:
                ly2, lx2 = y_dis, x_dis
        return None

    def find_kubik(self, data, wall_dis):
        if wall_dis is None:
            return None

        left_lim = self.off_x - wall_dis
        right_lim = 100 - self.off_x - wall_dis

        if self.dir == -90:
            left_lim, right_lim = -right_lim, -left_lim

        closest, furthest = None, None

        for point in data:
            y_dis, x_dis = self.find_katets(point)
            if left_lim <= x_dis <= right_lim:
                if y_dis > self.off_y and (closest is None or closest[0] > y_dis):
                    closest = (y_dis, x_dis)
                if y_dis > self.off_y and (furthest is None or furthest[0] < y_dis):
                    furthest = (y_dis, x_dis)

        var = closest is not None and ((self.dir == 90 and closest[0] >= 95) or (self.dir == -90 and closest[0] >= 120))

        if closest is None or furthest is None or \
                furthest[0] - closest[0] <= 60 or \
                closest[0] <= 15 or var:
            return None

        if self.dir == 90:
            from_wall_dis = wall_dis + closest[1]
        elif self.dir == -90:
            from_wall_dis = wall_dis - closest[1]

        if (self.dir == 90 and from_wall_dis <= 50) or (self.dir == -90 and from_wall_dis <= 43):
            return 40, closest[0]
        else:
            return 60, closest[0]

    def find_wall(self, scan):
        wall = None
        for point in scan:
            y_dis, x_dis = self.find_katets(point)
            var = (self.dir == 90 and x_dis < 0) or (self.dir == -90 and x_dis > 0)
            if y_dis <= 10:
                var = wall is None or ((self.dir == 90 and x_dis < wall) or (self.dir == -90 and x_dis > wall))
                if var:
                    wall = x_dis
        if wall is not None:
            return abs(wall)
        return None

    def find_angle(self, a, b):
        (y1, x1), (y2, x2) = a, b
        dx, dy = abs(x2 - x1), abs(y2 - y1)
        return math.degrees(math.atan2(dy, dx))

    def scan_callback(self, msg):
        gyro_angle = robot_car.read_gyro()
        all_scan, scan, rad = [], [], msg.angle_min
        for dis in msg.ranges:
            deg = math.degrees(rad) + 90
            if deg >= 180: deg -= 360
            act_deg = gyro_angle + deg
            dis *= 100
            if dis != 0 and math.isfinite(dis) and -130 <= act_deg <= 130 and dis >= 13:
                scan.append((act_deg, dis))
            if dis != 0 and math.isfinite(dis) and -130 <= deg <= 130 and dis >= 13:
                all_scan.append((deg, dis))
            rad += msg.angle_increment
        scan.sort()
        all_scan.sort()

        self.al_left_dis = robot_car.find(min, all_scan, -95, -85)
        self.al_right_dis = robot_car.find(min, all_scan, 85, 95)

        self.left_dis = robot_car.find(min, scan, -92.5, -87.5)
        self.right_dis = robot_car.find(min, scan, 87.5, 92.5)
        self.front_dis = robot_car.find(min, scan, -2.5, 2.5)

        if self.waitin is not None:
            self.exec = self.waitin
            self.waitin = None
            return

        if self.park_mode:
            if self.exec is not None:
                return
            if self.last_color == GREEN:
                if self.mov_park < 1:
                    if self.dis_till is None:
                        park = self.find_park(scan)
                        self.dis_till = park
                        print('[DIS TILL]', park)
                    if self.dis_till is not None:
                        if self.start_pos is None and robot_car.pos is not None:
                            self.start_pos = robot_car.pos
                        if robot_car.pos is None or robot_car.rot_to_cm(robot_car.pos - self.start_pos) < self.dis_till[1] + \
                                (31, 1)[self.mov_park]:
                            err = -robot_car.read_gyro()
                            print('here?', robot_car.rot_to_cm(robot_car.pos - self.start_pos))
                            robot_car.move(err * 5, 30)
                        else:
                            self.mov_park += 1
                            self.dis_till = None
                            self.start_pos = None
                            robot_car.hard_brake()
                else:
                    self.exec = PUT_BAGS
            elif self.last_color == RED:
                dis_arr = (10, 25)
                if self.mov_park < 2:
                    if self.dis_till is None:
                        park = self.find_park(scan if self.dir == 90 else scan[::-1])
                        self.dis_till = park
                        if self.dis_till is None:
                            return
                        print('[DIS TILL]', park)
                    if self.dis_till is not None:
                        if self.start_pos is None and robot_car.pos is not None:
                            self.start_pos = robot_car.pos
                        if robot_car.pos is None or robot_car.rot_to_cm(robot_car.pos - self.start_pos) < self.dis_till[1] + \
                                dis_arr[self.mov_park]:
                            err = -robot_car.read_gyro()
                            print('here?', robot_car.rot_to_cm(robot_car.pos - self.start_pos))
                            robot_car.move(err * 5, 30)
                        else:
                            self.mov_park += 1
                            self.dis_till = None
                            self.start_pos = None
                            robot_car.hard_brake()
                            if self.dir == -90 and self.mov_park == 1 and not self.d:
                                self.exec = SKEW
                else:
                    if self.dir == 90:
                        self.exec = PUT_BAGS_RED
                    elif self.dir == -90:
                        self.exec = PUT_BAGS
            return

        if not self.f_dir:
            for_dis = robot_car.find(min, scan, -5, 5)
            left = robot_car.find(max, scan, -90, -10)
            right = robot_car.find(max, scan, 10, 90)
            if left is not None and (right is None or left > right):
                self.dir = -90
            else:
                self.dir = 90
            self.f_dir = True

        r_wall_dis = self.find_wall(scan)

        for_dis = robot_car.find(min, scan, -20, 20)

        self.wall_dis = r_wall_dis

        if self.dir == 90:
            self.side_dis = robot_car.find(min, scan, 85, 90)
        elif self.dir == -90:
            self.side_dis = robot_car.find(min, scan, -90, -85)
        
        self.for_dis = for_dis

        if self.exec is not None:
            return

        kubik = self.find_kubik(scan, r_wall_dis)

        if kubik is not None and self.on_this_side < 2:
            if self.count == 0:
                self.seen_start = True
            self.time = None
            self.kubik = kubik
            if self.count < 5:
                print('KUBIK FOUND', kubik)
                robot_car.move(0,101)
                self.exec = KUBIK_UC
                self.on_this_side += 1
            else:
                side, y_dis = self.kubik
                var = self.dir == 90 and (self.last_side is None or side != self.last_side)
                if self.last_dis is None or (y_dis > self.last_dis and not abs(y_dis - self.last_dis) < 2) or var:
                    self.kubik_ix = (self.kubik_ix + 1) % len(self.kubiks)
                    if self.seen_start and self.kubik_ix == len(self.kubiks) - 2:
                        self.lap_cnt += 1
                    elif not self.seen_start and self.kubik_ix == len(self.kubiks) - 1:
                        self.lap_cnt += 1
                    self.last_side = side
                    self.exec = KUBIK_TC
                    self.on_this_side += 1
                else:
                    self.idle_speed = 70
                    self.idle = True
                self.last_dis = y_dis
        else:
            for_dis = robot_car.find(max, scan, -2.5, 2.5)
            if for_dis is not None and for_dis <= 25:
                robot_car.shift_angle -= self.dir
                self.exec = FOR_WALL
                self.last_dis = None
                self.count += 1
            else:
                if for_dis is not None and for_dis <= 85 and self.narrow and self.wall_dis is not None:
                    self.narrow = False
                    if (self.dir == 90 and self.wall_dis < 55) or (self.dir == -90 and self.wall_dis < 60):
                        if self.dir == 90:
                            swing_dis = 60 - self.wall_dis
                        elif self.dir == -90:
                            swing_dis = self.wall_dis - 65
                        self.swing_ang = robot_car.dis_to_ang(swing_dis)
                        self.exec = NARROW
                    elif self.wall_dis > 70:
                        if self.dir == 90:
                            swing_dis = 70 - self.wall_dis
                        elif self.dir == -90:
                            swing_dis = self.wall_dis - 70
                        self.swing_ang = robot_car.dis_to_ang(swing_dis)
                        self.exec = NARROW
                else:
                    self.idle_speed = robot_car.calc_ccel(70, 80, for_dis, 120)
                    if for_dis is None or for_dis <= 70:
                        self.idle_speed = robot_car.calc_ccel(30, 45, for_dis, 120)
                    self.idle = True

    def reset_vars(self):
        self.kubik = None
        self.swing_ang = None
        self.idle_set = False
        self.skew_done = False
        self.manoeuvre = False
        self.color_val = False
        self.start_time = None
        self.gone_back = False
        self.last_ok = False
        self.get_close = False
        self.start_time = None
        self.face_kubik = False
        #self.start_pos = None
        self.last_back_dis = None
        self.back_dis = None

        self.exec = None

    def miav_callback(self):
        if robot_car.height_rot() < -10:
            robot_car.move(0,0)
            raise Exception

        if self.exec is not None:
            self.idle = False
        if not self.f_dir:
            return
        if self.idle:
            gyro_angle = robot_car.read_gyro()
            gyro_error = -gyro_angle
            robot_car.move(head = gyro_error, speed = self.idle_speed)
        if self.exec is None:
            return
        if self.exec == KUBIK_UC:
            if not self.get_close:
                _, y_dis = self.kubik
                go_for = max(y_dis - 120, 0)
                if go_for != 0:
                    print('WILL GO FOR', go_for)
                if robot_car.pos is None:
                    self.start_pos = 0
                elif self.start_pos is None:
                    self.start_pos = robot_car.pos
                if go_for != 0 and (robot_car.pos is None or robot_car.rot_to_cm(robot_car.pos - self.start_pos) < go_for):
                    gyro_angle = robot_car.read_gyro()
                    gyro_error = -gyro_angle
                    if robot_car.pos is not None:
                        speed = robot_car.calc_ccel(25, 35, go_for - robot_car.rot_to_cm(robot_car.pos - self.start_pos), go_for)
                    else:
                        speed = 25
                    robot_car.move(head = gyro_error * 2, speed = speed)
                else:
                    self.start_pos = None
                    self.get_close = True
            if self.get_close:
                if not self.face_kubik:
                    if self.swing_ang is None:
                        k_pos, _ = self.kubik
                        if self.dir == 90:
                            dis = k_pos - self.wall_dis
                        elif self.dir == -90:
                            dis = self.wall_dis - k_pos
                        self.swing_ang = robot_car.dis_to_ang(dis * .9, False)
                        if (dis >= 0) != (self.swing_ang >= 0):
                            self.swing_ang = 0
                    if not self.skew_done:
                        gyro_angle = robot_car.read_gyro()
                        var = self.swing_ang >= 0 and gyro_angle < self.swing_ang or \
                                self.swing_ang < 0 and gyro_angle > self.swing_ang
                        if var:
                            if self.swing_ang >= 0:
                                robot_car.move(head = 90, speed = 40)
                            else:
                                robot_car.move(head = -90, speed = 40)
                        else:
                            self.skew_done = True
                    if self.skew_done:
                        gyro_angle = robot_car.read_gyro()
                        gyro_error = -gyro_angle
                        if self.for_dis is not None:
                            speed = robot_car.calc_ccel(30, 40, self.for_dis, 80)
                        if self.for_dis is not None and self.for_dis >= 25:
                            gyro_angle = robot_car.read_gyro()
                            gyro_error = -gyro_angle
                            robot_car.move(head = gyro_error * 2, speed = speed)
                        else:
                            self.face_kubik = True
                            self.skew_done = False
                if self.face_kubik:
                    if not self.gone_back:
                        if robot_car.pos is None:
                            self.start_pos = 0
                        elif self.start_pos is None:
                            self.start_pos = robot_car.pos
                        if robot_car.pos is None or robot_car.rot_to_cm(robot_car.pos - self.start_pos) < 20:
                            gyro_angle = robot_car.read_gyro()
                            if robot_car.pos is not None:
                                gone = robot_car.rot_to_cm(robot_car.pos - self.start_pos)
                                speed = -robot_car.calc_ccel(25, 30, max(20 - gone, 0), 40)
                            else:
                                speed = -25
                            robot_car.move(head = gyro_angle * 2, speed = speed)
                        else:
                            self.start_pos = None
                            self.gone_back = True
                    if self.gone_back:
                        if not self.color_val:
                            if self.color is None:
                                return
                                gyro_angle = robot_car.read_gyro()
                                gyro_error = -gyro_angle
                                robot_car.move(head = gyro_error * 2, speed = 22)
                            else:
                                k_pos, _ = self.kubik
                                if self.color == GREEN:
                                    if self.dir == 90:
                                        if self.count % 4 != 0:
                                            #swing_dis = k_pos / 2 - self.wall_dis
                                            swing_dis = k_pos - 17 - self.wall_dis
                                        else:
                                            #swing_dis = k_pos / 2 - self.wall_dis + 13
                                            swing_dis = k_pos - 17 - self.wall_dis
                                    elif self.dir == -90:
                                        swing_dis = self.wall_dis - (100 + k_pos) / 2
                                    self.swing_ang = robot_car.dis_to_ang(swing_dis, False)
                                elif self.color == RED:
                                    if self.dir == 90:
                                        #swing_dis = (100 + k_pos) / 2 - self.wall_dis
                                        swing_dis = k_pos + 17 - self.wall_dis
                                    elif self.dir == -90:
                                        if self.count % 4 != 0:
                                            swing_dis = self.wall_dis - k_pos / 2
                                        else:
                                            swing_dis = self.wall_dis - k_pos / 2 - 15
                                    self.swing_ang = robot_car.dis_to_ang(swing_dis, False)
                                self.narrow = True
                                if self.color == GREEN:
                                    robot_car.green_led.on()
                                elif self.color == RED:
                                    robot_car.red_led.on()
                                if self.count > 0:
                                    self.kubiks.append(self.color)
                                self.color_val = True
                        if self.color_val:
                            if not self.skew_done:
                                gyro_angle = robot_car.read_gyro()
                                var = self.swing_ang >= 0 and gyro_angle < self.swing_ang or \
                                        self.swing_ang < 0 and gyro_angle > self.swing_ang
                                if var:
                                    if self.swing_ang >= 0:
                                        robot_car.move(head = 90, speed = 40)
                                    else:
                                        robot_car.move(head = -90, speed = 40)
                                else:
                                    self.skew_done = True
                            if self.skew_done:
                                gyro_angle = robot_car.read_gyro()
                                gyro_error = -gyro_angle
                                if not -4 <= gyro_error <= 4:
                                    gyro_angle = robot_car.read_gyro()
                                    gyro_error = -gyro_angle
                                    robot_car.move(head = gyro_error * 2 + .1, speed = 40)
                                else:
                                    robot_car.green_led.off()
                                    robot_car.red_led.off()
                                    self.reset_vars()
                                    robot_car.move(0, 101)
        elif self.exec == KUBIK_TC:
            color = self.kubiks[self.kubik_ix]
            self.narrow = True
            if not self.skew_done:
                if self.swing_ang is None:
                    k_pos, _ = self.kubik
                    if color == GREEN:
                        if self.dir == 90:
                            if self.count % 4 != 0:
                                #swing_dis = k_pos / 2 - self.wall_dis
                                swing_dis = k_pos - 17 - self.wall_dis
                            else:
                                #swing_dis = k_pos / 2 - self.wall_dis + 7
                                swing_dis = k_pos - 17 - self.wall_dis
                        elif self.dir == -90:
                            swing_dis = self.wall_dis - (100 + k_pos) / 2
                        if self.dir == 90:
                            if self.count != 5:
                                self.swing_ang = robot_car.dis_to_ang(swing_dis * .95)
                            else:
                                self.swing_ang = robot_car.dis_to_ang(swing_dis * .95)
                        elif self.dir == -90:
                            if self.count != 5:
                                self.swing_ang = robot_car.dis_to_ang(swing_dis * .55)
                            else:
                                self.swing_ang = robot_car.dis_to_ang(swing_dis * .55)
                    elif color == RED:
                        if self.dir == 90:
                            #swing_dis = (100 + k_pos) / 2 - self.wall_dis
                            swing_dis = k_pos + 17 - self.wall_dis
                        elif self.dir == -90:
                            if self.count % 4 != 0:
                                swing_dis = self.wall_dis - k_pos / 2
                            else:
                                swing_dis = self.wall_dis - k_pos / 2 - 7
                        if self.dir == 90:
                            if self.count != 5:
                                self.swing_ang = robot_car.dis_to_ang(swing_dis * .95)
                            else:
                                self.swing_ang = robot_car.dis_to_ang(swing_dis * .95)
                        elif self.dir == -90:
                            if self.count != 5:
                                self.swing_ang = robot_car.dis_to_ang(swing_dis, s = True)
                            else:
                                self.swing_ang = robot_car.dis_to_ang(swing_dis, s = True)
                if self.dir == 90 and ((color == GREEN and self.swing_ang >= 0) or (color == RED and self.swing_ang <= 0)):
                    self.exec = None
                    self.reset_vars()
                    return
                gyro_angle = robot_car.read_gyro()
                var = self.swing_ang >= 0 and gyro_angle < self.swing_ang or \
                        self.swing_ang < 0 and gyro_angle > self.swing_ang
                if var:
                    if self.swing_ang >= 0:
                        robot_car.move(head = 90, speed = 45)
                    else:
                        robot_car.move(head = -90, speed = 45)
                else:
                    self.skew_done = True
                    robot_car.move(0, 101)
            if self.skew_done:
                gyro_angle = robot_car.read_gyro()
                gyro_error = -gyro_angle
                if not -4 <= gyro_error <= 4:
                    gyro_angle = robot_car.read_gyro()
                    gyro_error = -gyro_angle
                    if self.count != 5:
                        speed = 70
                    else:
                        speed = 65
                    if self.dir == -90:
                        speed += 5
                    robot_car.move(head = gyro_error * 2.5, speed = 40)
                else:
                    self.exec = None
                    self.reset_vars()
        elif self.exec == FOR_WALL:
            gyro_angle = robot_car.read_gyro()
            gyro_error = -gyro_angle
            back_dis = robot_car.ultra.distance * 100
            print('[FOR_WALL]', back_dis)
            if (not self.last_ok or back_dis != 100) and back_dis >= 18:
                if back_dis < 40:
                    self.last_ok = True
                gyro_angle = robot_car.read_gyro()
                speed = robot_car.calc_ccel(25, 35, back_dis, 60)
                robot_car.move(head = gyro_angle * 4, speed = -speed)
            else:
                self.turned = True
                self.on_this_side = 0
                self.start_pos = robot_car.pos
                self.reset_vars()
                self.waitin = FIX_GYRO_1
        elif self.exec == FIX_GYRO_1:
            if self.pt_one is None:
                robot_car.shift_angle -= robot_car.read_gyro()
                if self.dir == 90:
                    self.pt_one = self.al_left_dis
                elif self.dir == -90:
                    self.pt_one = self.al_right_dis
                self.pt_two = self.pt_one
                print('pt one', self.pt_one)
            if self.start_pos is None and robot_car.pos is not None:
                self.start_pos = robot_car.pos
            if robot_car.pos is None or robot_car.rot_to_cm(robot_car.pos - self.start_pos) < 20:
                err = -robot_car.read_gyro()
                robot_car.move(err * 2, 19)
            else:
                if self.start_time is None:
                    robot_car.hard_brake()
                    self.start_time = time.monotonic()
                if time.monotonic() - self.start_time > .2:
                    robot_car.hard_brake()
                    self.reset_vars()
                    self.waitin = FIX_GYRO_2
        elif self.exec == FIX_GYRO_2:
            if self.dir == 90:
                self.pt_two = self.al_left_dis
            elif self.dir == -90:
                self.pt_two = self.al_right_dis
            gone = robot_car.rot_to_cm(robot_car.pos - self.start_pos)
            print('[GONE]', gone, '[PT_ONE]', self.pt_one, '[PT_TWO]', self.pt_two)
            slope = (self.pt_two - self.pt_one) / gone
            angle = (math.degrees(math.atan(slope))) * 1
            if self.dir == -90: angle = -angle + 2
            print('pt two', self.pt_two)
            print(angle, robot_car.read_gyro())
            robot_car.shift_angle += angle
            self.reset_vars()
            self.pt_one = None
            self.pt_two = None
            self.start_pos = None
            if self.count == 4:
                if self.last_color == GREEN:
                    self.waitin = SKEW
                elif self.last_color == RED:
                    self.waitin = SKEW_RED
                self.park_mode = True
        elif self.exec == START:
            if self.dir == 90:
                if not self.skew_done:
                    gyro_angle = robot_car.read_gyro()
                    if gyro_angle < 85:
                        robot_car.move(head = 1000, speed = 27)
                    else:
                        robot_car.move(0, 101)
                        self.skew_done = True
                if self.skew_done:
                    if not self.face_kubik:
                        if robot_car.pos is None:
                            self.start_pos = 0
                        elif self.start_pos is None:
                            self.start_pos = robot_car.pos
                        if robot_car.pos is None or robot_car.rot_to_cm(robot_car.pos - self.start_pos) < 30:
                            if robot_car.pos is not None:
                                gone = robot_car.rot_to_cm(robot_car.pos - self.start_pos)
                                speed = robot_car.calc_ccel(25, 35, max(40 - gone, 0), 30)
                            else:
                                speed = 35
                            err = (90 - robot_car.read_gyro()) * 2
                            robot_car.move(head = err, speed = speed)
                        else:
                            robot_car.move(0, 101)
                            self.face_kubik = True
                            self.start_pos = None
                    if self.face_kubik:
                        if not self.manoeuvre:
                            gyro_angle = robot_car.read_gyro()
                            if not -1 <= gyro_angle <= 1:
                                robot_car.move(head = gyro_angle * 6, speed = -35)
                            else:
                                self.manoeuvre = True
                                robot_car.move(0, 101)
                        if self.manoeuvre:
                            if robot_car.pos is None:
                                self.start_pos = 0
                            elif self.start_pos is None:
                                self.start_pos = robot_car.pos
                            if robot_car.pos is None or robot_car.rot_to_cm(robot_car.pos - self.start_pos) < 10:
                                gyro_angle = robot_car.read_gyro()
                                if robot_car.pos is not None:
                                    gone = robot_car.rot_to_cm(robot_car.pos - self.start_pos)
                                    speed = -robot_car.calc_ccel(30, 35, max(40 - gone, 0), 30)
                                else:
                                    speed = -30
                                robot_car.move(head = gyro_angle * 5, speed = speed)
                            else:
                                robot_car.move(0, 101)
                                self.start_pos = None
                                self.manouevre = True
                                self.reset_vars()
            elif self.dir == -90:
                if not self.skew_done:
                    gyro_angle = robot_car.read_gyro()
                    if gyro_angle > -120:
                        robot_car.move(head = -1000, speed = 18)
                    else:
                        robot_car.move(0, 0)
                        self.skew_done = True
                if self.skew_done:
                    if not self.manoeuvre:
                        gyro_angle = robot_car.read_gyro()
                        gyro_error = -90 - gyro_angle
                        if self.side_dis >= 25:
                            robot_car.move(head = gyro_error * 1.2, speed = 18)
                        else:
                            self.manoeuvre = True
                            robot_car.move(0, 101)
                    if self.manoeuvre:
                        if not self.face_kubik:
                            gyro_angle = robot_car.read_gyro()
                            if not -3 <= gyro_angle <= 3:
                                gyro_angle = robot_car.read_gyro()
                                robot_car.move(head = -1000, speed = -20)
                            else:
                                self.face_kubik = True
                        if self.face_kubik:
                            if robot_car.pos is None:
                                self.start_pos = 0
                            elif self.start_pos is None:
                                self.start_pos = robot_car.pos
                            if robot_car.pos is None or robot_car.rot_to_cm(robot_car.pos - self.start_pos) < 18:
                                gyro_angle = robot_car.read_gyro()
                                if robot_car.pos is not None:
                                    gone = robot_car.rot_to_cm(robot_car.pos - self.start_pos)
                                    speed = -robot_car.calc_ccel(18, 23, max(25 - gone, 0), 25)
                                else:
                                    speed = -15
                                robot_car.move(head = gyro_angle * 2, speed = speed)
                            else:
                                robot_car.move(0, 0)
                                self.start_pos = None
                                self.reset_vars()
        elif self.exec == NARROW:
            gyro_angle = robot_car.read_gyro()
            var = self.swing_ang >= 0 and gyro_angle < self.swing_ang or \
                self.swing_ang < 0 and gyro_angle > self.swing_ang
            if var:
                if self.swing_ang >= 0:
                    robot_car.move(head = 90, speed = 40)
                else:
                    robot_car.move(head = -90, speed = 40)
            else:
                self.narrow = False
                self.exec = None
                self.reset_vars()
        elif self.exec == SKEW:
            if self.swing_ang is None:
                if self.dir == 90:
                    dis = 38 - self.left_dis
                elif self.dir == -90:
                    dis = self.right_dis - 27
                self.swing_ang = robot_car.dis_to_ang(dis)
                if (dis >= 0) != (self.swing_ang >= 0):
                    self.swing_ang = 0
            if not self.skew_done:
                ang = robot_car.read_gyro()
                var = self.swing_ang >= 0 and ang < self.swing_ang or \
                        self.swing_ang < 0 and ang > self.swing_ang
                if var:
                    if self.swing_ang >= 0:
                        robot_car.move(head = 90, speed = 40)
                    else:
                        robot_car.move(head = -90, speed = 40)
                else:
                    self.skew_done = True
                    robot_car.hard_brake()
            if self.skew_done:
                err = -robot_car.read_gyro() 
                if not -1.5 <= err <= 1.5:
                    err = -robot_car.read_gyro() 
                    robot_car.move(head = (err + 0.1) * 5, speed = 25)
                else:
                    self.reset_vars()
                    robot_car.hard_brake()
        elif self.exec == PUT_BAGS:
            if self.dir == 90:
                err = 90 - robot_car.read_gyro()
            elif self.dir == -90:
                if not self.d:
                    err = -80 - robot_car.read_gyro()
                elif self.d:
                    err = -70 - robot_car.read_gyro()
            if not -2 <= err <= 2:
                if self.dir == 90:
                    robot_car.move(head = -90, speed = -20)
                elif self.dir == -90:
                    robot_car.move(head = -90, speed = -20)
            else:
                robot_car.hard_brake()
                self.reset_vars()
                self.exec = SCREW_LEFT_RED
        elif self.exec == SKEW_RED:
            print('SKEW_RED')
            if self.swing_ang is None:
                if self.dir == 90:
                    dis = 77 - self.left_dis
                elif self.dir == -90:
                    if not self.d:
                        dis = self.right_dis - 80
                    elif self.d:
                        dis = self.right_dis - 33
                self.swing_ang = robot_car.dis_to_ang(dis * .95)
                if (dis >= 0) != (self.swing_ang >= 0):
                    self.swing_ang = 0
            if not self.skew_done:
                ang = robot_car.read_gyro()
                var = self.swing_ang >= 0 and ang < self.swing_ang or \
                        self.swing_ang < 0 and ang > self.swing_ang
                if var:
                    if self.swing_ang >= 0:
                        robot_car.move(head = 90, speed = 40)
                    else:
                        robot_car.move(head = -90, speed = 40)
                else:
                    self.skew_done = True
                    robot_car.hard_brake()
            if self.skew_done:
                err = -robot_car.read_gyro()
                if not -2 <= err <= 2:
                    err = -robot_car.read_gyro()
                    robot_car.move(head = err * 2.5, speed = 35)
                else:
                    self.reset_vars()
                    robot_car.hard_brake()
                    self.start_pos = None
                    self.exec = GO_BACK_RED
        elif self.exec == GO_BACK_RED:
            print('GO_BACK', self.start_pos)
            if self.start_pos is None and robot_car.pos is not None:
                print('start_pos', robot_car.pos)
                self.start_pos = robot_car.pos
            if robot_car.pos is None or robot_car.rot_to_cm(robot_car.pos - self.start_pos) < 40:
                if self.start_pos is not None:
                    print('eyy', robot_car.rot_to_cm(robot_car.pos - self.start_pos) )
                err = robot_car.read_gyro()
                robot_car.move(err * 4 + 5, -28)
            else:
                if self.dir == -90 and self.d:
                    now = time.monotonic()
                    if self.start_time is None or now - self.start_time < .5:
                        if self.start_time is None:
                            print('[GO_BACK] start wait')
                            self.start_time = time.monotonic()
                            robot_car.move(0, 101)
                        return
                self.reset_vars()
                robot_car.hard_brake()
                self.exec = None
        elif self.exec == PUT_BAGS_RED:
            if self.dir == 90:
                err = 90 - robot_car.read_gyro()
            elif self.dir == -90:
                err = -90 - robot_car.read_gyro()
            if (self.dir == 90 and err >= 5) or (self.dir == -90 and err <= -3):
                if self.dir == 90:
                    err = 90 - robot_car.read_gyro()
                elif self.dir == -90:
                    err = -90 - robot_car.read_gyro()
                robot_car.move(head = -err * 2, speed = -25)
            else:
                print('FINISH PUT_BAGS_RED')
                self.reset_vars()
                robot_car.hard_brake()
                self.waitin = ADJUST_DIS_RED
                self.start_pos = None
        elif self.exec == ADJUST_DIS_RED:
            if self.back_dis is None:
                self.back_dis = max(100 - self.right_dis - ROBOT_LENGTH - 11.5, 0)
                print(self.back_dis)
            if self.start_pos is None and robot_car.pos is not None:
                self.start_pos = robot_car.pos
            if robot_car.pos is None or robot_car.rot_to_cm(robot_car.pos - self.start_pos) < self.back_dis:
                if self.dir == 90:
                    err = 90 - robot_car.read_gyro()
                elif self.dir == -90:
                    err = -90 - robot_car.read_gyro()
                robot_car.move(-err * 2 + 6, -20)
            else:
                self.reset_vars()
                robot_car.hard_brake()
                self.exec = SCREW_LEFT_RED
        elif self.exec == SCREW_LEFT_RED:
            if self.dir == 90:
                if self.last_color == GREEN:
                    err = 67 - robot_car.read_gyro()
                elif self.last_color == RED:
                    err = 70 - robot_car.read_gyro()
            elif self.dir == -90:
                if self.last_color == RED:
                    if self.d:
                        err = -58.6 - robot_car.read_gyro()
                    else:
                        err = -63 - robot_car.read_gyro()
                elif self.last_color == GREEN:
                    err = -70 - robot_car.read_gyro()
                    print('screw here')
            print('screw', err)
            if (self.dir == 90 and err <= 0) or (self.dir == -90 and err >= 0):
                print('TURN LEFT', err)
                if self.dir == 90:
                    robot_car.move(head = -90, speed = 22)
                elif self.dir == -90:
                    robot_car.move(head = 90, speed = 22)
            else:
                self.reset_vars()
                self.exec = FIN_1
                robot_car.hard_brake()
        elif self.exec == FIN_1:
            if self.dir == 90:
                err = robot_car.read_gyro() - 3.5
            elif self.dir == -90:
                err = robot_car.read_gyro() + 3.5
            if not -2 <= err <= 2:
                print('[FIN_1]')
                if self.dir == 90:
                    robot_car.move(90, -25)
                elif self.dir == -90:
                    robot_car.move(-90, -30)
            else:
                self.reset_vars()
                self.exec = FIN_2
                robot_car.hard_brake()
        elif self.exec == FIN_2:
            err = -robot_car.read_gyro()
            if not -1.5 <= err <= 1.5:
                robot_car.move(err * 5, 15)
            else:
                self.reset_vars()
                robot_car.hard_brake()
                raise Exception

    def find_colors(self, hsv, colors):
        area = dict()
        for color, codes in self.colors.items():
            if color in colors:
                area[color], mask = 0, None
                for code in codes:
                    color_low = numpy.array(code[0])
                    color_high = numpy.array(code[1])
                    m = cv2.inRange(hsv, color_low, color_high)
                    mask = m if mask is None else cv2.bitwise_or(mask, m)
                if mask is not None:
                    coords = cv2.findNonZero(mask)
                    if coords is not None:
                        area[color] = len(coords)
        return area

    def camera_callback(self, msg):
        np_arr = numpy.frombuffer(msg.data, numpy.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        height, width, _ = frame.shape
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        k_hsv = hsv[height*4//10:,:]
        k_area = self.find_colors(k_hsv, ('green', 'red'))
        if k_area['green'] > k_area['red']: self.color = GREEN
        elif k_area['red'] > k_area['green']: self.color = RED
        else: self.color = None

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
