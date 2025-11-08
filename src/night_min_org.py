import math
import robot_car
import rclpy.node
import sensor_msgs.msg

MIN_SPEED = 20

FOR_WALL = 0

class miav(rclpy.node.Node):
    def __init__(self):
        super().__init__("nights")
        
        self.create_subscription(sensor_msgs.msg.LaserScan, '/scan', self.scan_callback, 1)
        self.create_timer(1.0 / 15.0, self.miav_callback)

        """
        Direction variables:
        """
        self.dir = None
        self.f_dir = False

        """
        Exec variables:
        """
        self.turned = False

        self.exec = None

        """
        Idle variables (set in scan, unset in miav):
        """
        self.idle = False
        self.idle_speed = None

        """
        Turn count:
        """
        self.count = 0
        self.naughty = False

        self.just_started = True

        robot_car.read_encoder()

    def scan_callback(self, msg):
        if self.just_started:
            robot_car.shift_angle -= robot_car.read_gyro()
            self.just_started = False

        if self.exec is not None:
            return

        gyro_angle = robot_car.read_gyro()
        scan, rad = [], msg.angle_min
        for dis in msg.ranges:
            deg = gyro_angle - math.degrees(rad)
            if -90 <= deg <= 90 and math.isfinite(dis):
                scan.append((deg, dis * 100))
            rad += msg.angle_increment

        if not self.f_dir:
            forr_dis = robot_car.find(min, scan, -5, 5)
            if forr_dis is None or forr_dis > 100:
                gyro_angle = robot_car.read_gyro()
                gyro_error = -gyro_angle
                robot_car.move(head = gyro_error, speed = 40)
            else:
                left = robot_car.find(max, scan, -90, -10)
                right = robot_car.find(max, scan, 10, 90)
                if left is not None and (right is None or left > right):
                    self.dir = -90
                else:
                    self.dir = 90
                self.f_dir = True
                print('dir', self.dir)

        if not self.f_dir:
            return

        forr_dis = robot_car.find(min, scan, -5, 5)

        if self.count == 12 and forr_dis is not None and forr_dis <= 180:
            robot_car.move(0, 0)
            robot_car.ser.close()
            raise Exception

        if self.turned and self.count % 4 == 0 and not self.naughty:
            if self.dir == 90:
                robot_car.shift_angle += 6
            else:
                robot_car.shift_angle -= 4.5
            self.naughty = True

        for_dis = robot_car.find(min, scan, -5, 5)

        if for_dis is None:
            return

        if self.dir == 90:
            for_lim = 90
        else:
            for_lim = 90

        if for_dis <= for_lim and not self.turned:
            print('FOR_WALL prog')
            robot_car.shift_angle -= self.dir
            self.exec = FOR_WALL
            self.count += 1
            self.naughty = False
        else:
            accel_speed = robot_car.calc_ccel(30, 60, 300 - for_dis, 70)
            if self.count == 12:
                deaccel_speed = robot_car.calc_ccel(30, 30, for_dis, 300)
            else:
                deaccel_speed = robot_car.calc_ccel(30, 60, for_dis, 130)
            self.idle_speed = min(accel_speed, deaccel_speed)
            if for_dis > 90:
                self.turned = False
            self.idle = True

    def miav_callback(self):
        if robot_car.height_rot() < -10:
            robot_car.move(0, 0)
            raise Exception

        if self.exec is not None:
            self.idle = False
        if not self.f_dir: return
        if self.idle:
            gyro_angle = robot_car.read_gyro()
            gyro_error = -gyro_angle
            print('IDLE', gyro_error, robot_car.pos)
            robot_car.move(head = gyro_error, speed = self.idle_speed)
        if self.exec is None: return
        if self.exec == FOR_WALL:
            gyro_angle = robot_car.read_gyro()
            gyro_error = -gyro_angle
            if not -5 <= gyro_error <= 5:
                gyro_angle = robot_car.read_gyro()
                gyro_error = -gyro_angle
                print('GYRO ERRROR', gyro_error)
                robot_car.move(head = gyro_error, speed = 40)
            else:
                robot_car.move(0, 0)
                self.turned = True
                self.exec = None
