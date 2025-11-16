import cv2
import math
import time
import numpy
import robot_car
import rclpy.node
import sensor_msgs.msg

START = 0
KUBIK_UC = 1
KUBIK_TC = 2
BASE_WALL = 3
FOR_WALL = 4
NARROW = 5
FINAL = 6

GREEN = 0
RED = 1

class miav(rclpy.node.Node):
    def __init__(self):
        super().__init__('miav')
        self.create_subscription(sensor_msgs.msg.LaserScan, '/scan', self.scan_callback, 1)
        self.create_subscription(sensor_msgs.msg.CompressedImage, '/camera/image_raw/compressed', self.camera_callback, 1)
        self.create_timer(1.0 / 10.0, self.miav_callback)

        """
        Direction variables:
        """
        self.dir = None
        self.f_dir = False

        """
        Find kubix variables:
        """
        self.off_x = 35

        """
        Color detection variables:
        """
        self.color = None
        self.colors = {
            'green': [((70, 128, 75), (86, 255, 170)), ((67, 106, 22), (78, 188, 182)), ((69, 51, 33), (95, 193, 154))],
            'red': [((0, 154, 160), (179, 206, 235)), ((0, 113, 0), (8, 255, 255))],
        }

        """
        Exec variables:
        """
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
        """
        Surpass variables:
        """
        self.time = None

        """
        Idle variables (set in scan, unset in miav):
        """
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
        self.wall_tilt = None

        self.seen_on_for = False
        self.just_started = True

        self.count = 0

    def find_katets(self, point):
        deg, hyp = point
        beta = math.radians(deg)
        adj = math.cos(beta) * hyp
        opp = math.sin(beta) * hyp
        return adj, opp

    def find_park(self, scan):
        ly1, lx1, lx2 = None, None, None
        for p in scan:
            if p[0] < -85 or p[0] > 30:
                continue
            y_dis = abs(p[1] * math.sin(math.radians(90 + p[0])))
            x_dis = abs(p[1] * math.cos(math.radians(90 + p[0])))
            if ly1 is None or abs(y_dis - ly1) > 3:
                if lx2 is not None and abs(lx2 - lx1) > 5:
                    return (lx2, ly1)
                ly1, lx1, lx2 = y_dis, x_dis, x_dis
            else:
                lx2 = x_dis
        return None

    def find_wall_tilt(self, scan):
        clos, perp = None, None
        for deg, dis in scan:
            var = (self.dir == 90 and deg <= -70) or (self.dir == -90 and \
                    deg >= 70)
            if var and (clos is None or dis <= clos):
                clos, perp = dis, deg
        if perp is not None:
            if self.dir == 90:
                return -84.7 - perp
            elif self.dir == -90:
                return 90 - perp
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
                if closest is None or closest[0] > y_dis:
                    closest = (y_dis, x_dis)
                if furthest is None or furthest[0] < y_dis:
                    furthest = (y_dis, x_dis)

        var = closest is not None and ((self.dir == 90 and closest[0] >= 100) or (self.dir == -90 and closest[0] >= 100))

        if closest is None or furthest is None or \
                furthest[0] - closest[0] <= 60 or \
                var or closest[0] <= 15:
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

    def scan_callback(self, msg):
        gyro_angle = robot_car.read_gyro()
        all_scan, scan, rad = [], [], msg.angle_min
        for dis in msg.ranges:
            deg = gyro_angle - math.degrees(rad)
            if math.isfinite(dis):
                if -95 <= deg <= 95:
                    all_scan.append((deg, dis * 100))
                if -90 <= deg <= 90:
                    scan.append((deg, dis * 100))
            rad += msg.angle_increment

        self.wall_tilt = self.find_wall_tilt(all_scan)

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

        for_dis = robot_car.find(min, scan, -2.5, 2.5)

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
            print('kubik', self.kubik)
            if self.count < 5:
                print('KUBIK_UC prog')
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
                    print('KUBIK_TC prog', self.last_dis)
                    self.on_this_side += 1
                else:
                    self.idle_speed = 19
                    self.idle = True
                self.last_dis = y_dis
        else:
            for_dis = robot_car.find(max, scan, -2.5, 2.5)
            if for_dis is not None and for_dis <= 30:
                print('FOR_WALL prog')
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
                    print('NARROW prog')
                else:
                    if for_dis is not None:
                        self.idle_speed = robot_car.calc_ccel(18, 30, for_dis, 120)
                    else:
                        self.idle_speed = 17.5
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
        self.last_back_dis = None

        self.exec = None

    def miav_callback(self):
        if robot_car.height_rot() < -10:
            robot_car.move(0,0)
            robot_car.ser.close()
            robot_car.ultra.close()
            robot_car.green_led.close()
            robot_car.red_led.close()
            raise Exception

        if self.exec is not None:
            self.idle = False
        if not self.f_dir:
            return
        if self.idle:
            gyro_angle = robot_car.read_gyro()
            gyro_error = -gyro_angle
            robot_car.move(head = gyro_error * 2, speed = self.idle_speed)
        if self.exec is None:
            return
        if self.exec == KUBIK_UC:
            if not self.get_close:
                _, y_dis = self.kubik
                go_for = max(y_dis - 95, 0)
                if robot_car.pos is None:
                    self.start_pos = 0
                elif self.start_pos is None:
                    self.start_pos = robot_car.pos
                if go_for != 0 and (robot_car.pos is None or robot_car.rot_to_cm(robot_car.pos - self.start_pos) < go_for):
                    gyro_angle = robot_car.read_gyro()
                    gyro_error = -gyro_angle
                    if robot_car.pos is not None:
                        speed = robot_car.calc_ccel(16, 20, go_for - robot_car.rot_to_cm(robot_car.pos - self.start_pos), go_for)
                    else:
                        speed = 15
                    robot_car.move(head = gyro_error * 2, speed = speed)
                else:
                    self.start_pos = None
                    self.get_close = True
                    robot_car.move(0, 101)
            if self.get_close:
                robot_car.move(0, 101)
                if not self.face_kubik:
                    if self.swing_ang is None:
                        k_pos, _ = self.kubik
                        if self.dir == 90:
                            dis = k_pos - self.wall_dis
                        elif self.dir == -90:
                            dis = self.wall_dis - k_pos
                        self.swing_ang = robot_car.dis_to_ang(dis * .85)
                        if (dis >= 0) != (self.swing_ang >= 0):
                            self.swing_ang = 0
                    if not self.skew_done:
                        gyro_angle = robot_car.read_gyro()
                        var = self.swing_ang >= 0 and gyro_angle < self.swing_ang or \
                                self.swing_ang < 0 and gyro_angle > self.swing_ang
                        if var:
                            if self.swing_ang >= 0:
                                robot_car.move(head = 50, speed = 25)
                            else:
                                robot_car.move(head = -50, speed = 25)
                        else:
                            self.skew_done = True
                            robot_car.move(0, 101)
                    if self.skew_done:
                        gyro_angle = robot_car.read_gyro()
                        gyro_error = -gyro_angle
                        if self.for_dis is not None:
                            speed = robot_car.calc_ccel(15, 21, self.for_dis, 80)
                        if self.for_dis is not None and self.for_dis >= 30:
                            gyro_angle = robot_car.read_gyro()
                            gyro_error = -gyro_angle
                            robot_car.move(head = gyro_error * 2, speed = speed)
                        else:
                            self.face_kubik = True
                            self.skew_done = False
                            robot_car.move(0, 101)
                if self.face_kubik:
                    if not self.gone_back:
                        if robot_car.pos is None:
                            self.start_pos = 0
                        elif self.start_pos is None:
                            self.start_pos = robot_car.pos
                        if robot_car.pos is None or robot_car.rot_to_cm(robot_car.pos - self.start_pos) < 10:
                            gyro_angle = robot_car.read_gyro()
                            if robot_car.pos is not None:
                                gone = robot_car.rot_to_cm(robot_car.pos - self.start_pos)
                                speed = -robot_car.calc_ccel(15, 21, max(20 - gone, 0), 40)
                            else:
                                speed = -20
                            robot_car.move(head = gyro_angle * 2, speed = speed)
                        else:
                            self.start_pos = None
                            self.gone_back = True
                    if self.gone_back:
                        if not self.color_val:
                            if self.color is None:
                                gyro_angle = robot_car.read_gyro()
                                gyro_error = -gyro_angle
                                robot_car.move(head = gyro_error * 2, speed = 16)
                            else:
                                k_pos, _ = self.kubik
                                if self.color == GREEN:
                                    if self.dir == 90:
                                        if self.count % 4 != 0:
                                            swing_dis = k_pos / 2 - self.wall_dis
                                        else:
                                            swing_dis = k_pos / 2 - self.wall_dis + 13
                                    elif self.dir == -90:
                                        swing_dis = self.wall_dis - (100 + k_pos) / 2
                                    self.swing_ang = robot_car.dis_to_ang(swing_dis * .87)
                                    print('GREEN', self.swing_ang)
                                elif self.color == RED:
                                    if self.dir == 90:
                                        swing_dis = (100 + k_pos) / 2 - self.wall_dis
                                    elif self.dir == -90:
                                        if self.count % 4 != 0:
                                            swing_dis = self.wall_dis - k_pos / 2
                                        else:
                                            swing_dis = self.wall_dis - k_pos / 2 - 15
                                    self.swing_ang = robot_car.dis_to_ang(swing_dis * .87)
                                    print('RED', self.swing_ang)
                                self.narrow = True
                                if self.color == GREEN:
                                    robot_car.green_led.on()
                                elif self.color == RED:
                                    robot_car.red_led.on()
                                if self.count > 0:
                                    self.kubiks.append(self.color)
                                    print('ADDED', self.color)
                                self.color_val = True
                                robot_car.move(0, 101)
                        if self.color_val:
                            if not self.skew_done:
                                gyro_angle = robot_car.read_gyro()
                                var = self.swing_ang >= 0 and gyro_angle < self.swing_ang or \
                                        self.swing_ang < 0 and gyro_angle > self.swing_ang
                                if var:
                                    if self.swing_ang >= 0:
                                        robot_car.move(head = 50, speed = 25)
                                    else:
                                        robot_car.move(head = -50, speed = 25)
                                else:
                                    self.skew_done = True
                                    robot_car.move(0, 101)
                            if self.skew_done:
                                gyro_angle = robot_car.read_gyro()
                                gyro_error = -gyro_angle
                                if not -4 <= gyro_error <= 4:
                                    gyro_angle = robot_car.read_gyro()
                                    gyro_error = -gyro_angle
                                    robot_car.move(head = gyro_error * 2, speed = 22)
                                else:
                                    print('KUBIK DONE')
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
                                swing_dis = k_pos / 2 - self.wall_dis
                            else:
                                swing_dis = k_pos / 2 - self.wall_dis + 7
                        elif self.dir == -90:
                            swing_dis = self.wall_dis - (100 + k_pos) / 2
                        print('GREEN')
                        if self.dir == 90:
                            if self.count != 5:
                                self.swing_ang = robot_car.dis_to_ang(swing_dis * .66)
                            else:
                                self.swing_ang = robot_car.dis_to_ang(swing_dis * .66)
                        elif self.dir == -90:
                            if self.count != 5:
                                self.swing_ang = robot_car.dis_to_ang(swing_dis * .66)
                            else:
                                self.swing_ang = robot_car.dis_to_ang(swing_dis * .66)
                    elif color == RED:
                        if self.dir == 90:
                            swing_dis = (100 + k_pos) / 2 - self.wall_dis
                        elif self.dir == -90:
                            if self.count % 4 != 0:
                                swing_dis = self.wall_dis - k_pos / 2
                            else:
                                swing_dis = self.wall_dis - k_pos / 2 - 7
                        print('RED')
                        if self.dir == 90:
                            if self.count != 5:
                                self.swing_ang = robot_car.dis_to_ang(swing_dis * .66)
                            else:
                                self.swing_ang = robot_car.dis_to_ang(swing_dis * .66)
                        elif self.dir == -90:
                            if self.count != 5:
                                self.swing_ang = robot_car.dis_to_ang(swing_dis * .66)
                            else:
                                self.swing_ang = robot_car.dis_to_ang(swing_dis * .66)
                gyro_angle = robot_car.read_gyro()
                var = self.swing_ang >= 0 and gyro_angle < self.swing_ang or \
                        self.swing_ang < 0 and gyro_angle > self.swing_ang
                if var:
                    if self.swing_ang >= 0:
                        robot_car.move(head = 50, speed = 25)
                    else:
                        robot_car.move(head = -50, speed = 25)
                else:
                    self.skew_done = True
                    robot_car.move(0, 101)
                print('KUBIK done')
            if self.skew_done:
                gyro_angle = robot_car.read_gyro()
                gyro_error = -gyro_angle
                if not -3 <= gyro_error <= 3:
                    gyro_angle = robot_car.read_gyro()
                    gyro_error = -gyro_angle
                    if self.count != 5:
                        speed = 45
                    else:
                        speed = 35
                    if self.dir == -90:
                        speed += 5
                    robot_car.move(head = gyro_error, speed = speed)
                else:
                    self.exec = None
                    self.reset_vars()
                    if self.lap_cnt == 2 and self.count % 4 == 0:
                        robot_car.move(0, 0)
                        self.exec = FINAL
        elif self.exec == FOR_WALL:
            gyro_angle = robot_car.read_gyro()
            gyro_error = -gyro_angle
            back_dis = robot_car.ultra.distance * 100
            if (self.start_time is None) and (not self.face_kubik) and (not self.last_ok or back_dis != 100) and back_dis >= 25:
                if back_dis < 40:
                    self.last_ok = True
                gyro_angle = robot_car.read_gyro()
                speed = robot_car.calc_ccel(15, 19, back_dis, 60)
                robot_car.move(head = gyro_angle * 4, speed = -speed)
                self.wall_tilt = None
            else:
                if not self.face_kubik:
                    now = time.monotonic()
                    if self.start_time is None or now - self.start_time < 0:
                        if self.start_time is None:
                            self.start_time = now
                        gyro_error = -robot_car.read_gyro()
                        robot_car.move(head = gyro_error * 2, speed = 19)
                    else:
                        self.face_kubik = True
                        robot_car.move(0, 101)
                        self.start_time = None
                        self.wall_tilt = None
                elif self.face_kubik:
                    now = time.monotonic()
                    if self.start_time is None or now - self.start_time < 1:
                        if self.start_time is None:
                            self.start_time = time.monotonic()
                            robot_car.move(0, 101)
                    else:
                        print('FOR_WALL done')
                        gyro_angle = robot_car.read_gyro()
                        if self.dir == 90:
                            error = self.wall_tilt
                            print('wall tilt', self.wall_tilt)
                            print('SHIFTED BY', robot_car.read_gyro(), ":", error)
                            robot_car.shift_angle += error
                        elif self.dir == -90:
                            #error = (self.wall_tilt - gyro_angle)
                            error = -self.wall_tilt * 0.35
                            if self.count > 4:
                                error -= .7
                            robot_car.shift_angle += error *.85
                            print('SHIFTED BY', error * .85)
                        self.turned = True
                        self.on_this_side = 0
                        self.reset_vars()
                        if self.lap_cnt >= 2 and self.count % 4 == 0:
                            robot_car.move(0, 0)
                            self.seen_on_for = True
                            self.exec = FINAL
                        if self.count == 12:
                            raise Exception
        elif self.exec == START:
            if self.dir == 90:
                if not self.skew_done:
                    gyro_angle = robot_car.read_gyro()
                    if gyro_angle < 85:
                        robot_car.move(head = 1000, speed = 15)
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
                                speed = robot_car.calc_ccel(16, 20, max(40 - gone, 0), 30)
                            else:
                                speed = 15
                            robot_car.move(head = -8, speed = speed)
                        else:
                            robot_car.move(0, 101)
                            self.face_kubik = True
                            self.start_pos = None
                    if self.face_kubik:
                        if not self.manoeuvre:
                            gyro_angle = robot_car.read_gyro()
                            gyro_error = -gyro_angle
                            if not -3 <= gyro_error <= 3:
                                robot_car.move(head = -gyro_error * 1.2, speed = -16)
                            else:
                                self.manoeuvre = True
                                robot_car.move(0, 101)
                        if self.manoeuvre:
                            if robot_car.pos is None:
                                self.start_pos = 0
                            elif self.start_pos is None:
                                self.start_pos = robot_car.pos
                            if robot_car.pos is None or robot_car.rot_to_cm(robot_car.pos - self.start_pos) < 20:
                                gyro_angle = robot_car.read_gyro()
                                if robot_car.pos is not None:
                                    gone = robot_car.rot_to_cm(robot_car.pos - self.start_pos)
                                    speed = -robot_car.calc_ccel(15, 20, max(40 - gone, 0), 30)
                                else:
                                    speed = -15
                                robot_car.move(head = gyro_angle * 2, speed = speed)
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
                        if self.side_dis >= 20:
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
                    robot_car.move(head = 50, speed = 20)
                else:
                    robot_car.move(head = -50, speed = 20)
            else:
                print('NARROW done')
                self.narrow = False
                self.exec = None
                self.reset_vars()
        elif self.exec == FINAL:
            if not self.gone_back:
                if not self.get_close:
                    robot_car.move(0, 0)
                    self.get_close = True
                back_dis = robot_car.ultra.distance * 100
                if back_dis > 30:
                    gyro_angle = robot_car.read_gyro()
                    robot_car.move(head = gyro_angle, speed = -18)
                else:
                    self.gone_back = True
                    robot_car.move(0, 101)
            if self.gone_back:
                if not self.manoeuvre:
                    if robot_car.pos is None:
                        self.start_pos = 0
                    elif self.start_pos is None:
                        self.start_pos = robot_car.pos
                    if self.dir == 90:
                        go_for = 129
                    elif self.dir == -90:
                        go_for = 175
                    if self.seen_on_for:
                        go_for += 20
                    if robot_car.pos is None or robot_car.rot_to_cm(robot_car.pos - self.start_pos) < go_for:
                        gyro_angle = robot_car.read_gyro()
                        gyro_error = -gyro_angle
                        if robot_car.pos is not None:
                            speed = robot_car.calc_ccel(25, 45, go_for - robot_car.rot_to_cm(robot_car.pos - self.start_pos), go_for)
                        else:
                            speed = 25
                        robot_car.move(head = gyro_error * 2, speed = speed)
                    else:
                        robot_car.move(0, 0)
                        self.manoeuvre = True
                if self.manoeuvre:
                    gyro_angle = robot_car.read_gyro()
                    if self.dir == 90:
                        gyro_error = 90 - gyro_angle
                    elif self.dir == -90:
                        gyro_error = -90 - gyro_angle
                    if not -3 <= gyro_error <= 3:
                        gyro_angle = robot_car.read_gyro()
                        if self.dir == 90:
                            gyro_error = 90 - gyro_angle
                        elif self.dir == -90:
                            gyro_error = -90 - gyro_angle
                        robot_car.move(head = -gyro_error, speed = -25)
                    else:
                        back_dis = robot_car.ultra.distance * 100
                        if (not self.last_ok or back_dis != 100) and back_dis >= 22:
                            if back_dis < 50:
                                self.last_ok = True
                            gyro_angle = robot_car.read_gyro()
                            if self.dir == 90:
                                gyro_error = 90 - gyro_angle
                            elif self.dir == -90:
                                gyro_error = -90 - gyro_angle
                            robot_car.move(head = gyro_error, speed = -24)
                        else:
                            robot_car.move(0,0)
                            robot_car.ser.close()
                            robot_car.ultra.close()
                            robot_car.green_led.close()
                            robot_car.red_led.close()
                            robot_car.i2c.deinit()
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

