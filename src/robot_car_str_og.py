import pid
import time
import math
import busio
import board
import serial
import gpiozero
import threading
import adafruit_bno055

ser = serial.Serial('/dev/ttyACM1', 115200)

ULTRA_ECHO = 25
ULTRA_TRIG = 24
GREEN_LED = 22
RED_LED = 27

i2c = busio.I2C(board.SCL, board.SDA)
gyro = adafruit_bno055.BNO055_I2C(i2c)
#gyro.mode = adafruit_bno055.CONFIG_MODE
#gyro.offsets = (-26, -57, -30, -3, -337, -92, -1, -3, 0)
#gyro.offsets = (-31, -47, -20, 3, -299, -93, 0, -2, 1)
#gyro.offsets = (-10, -45, -21, -11, -327, -84, 0, -2, 0)
#gyro.offsets = (-11, -73, -28, 6291, 5824, -6956, -1, -3, 0)
#gyro.offsets = (-25, -50, -30, 28, -317, -88, 0, -3, 0)
gyro.mode = adafruit_bno055.NDOF_MODE

ultra = gpiozero.DistanceSensor(echo = ULTRA_ECHO, trigger = ULTRA_TRIG)
green_led = gpiozero.LED(GREEN_LED)
red_led = gpiozero.LED(RED_LED)

PID = pid.PID((1.5, 0, 0))

rot_to_cm_rate = 7 / 50
shift_angle = 0

trans_coeff = 1
max_pos = 140
min_pos = 40
pos = None

A = 12.1
B = -20

def dis_to_ang(dis):
    res = A * math.sqrt(abs(dis)) + B
    return dis / abs(dis) * (A * math.sqrt(abs(dis)) + B)

def calc_ccel(mn, mx, dis, cel):
    return max(min((mx - mn) * dis / cel + mn, mx), mn)

def rot_to_cm(rot):
    return abs(rot) * rot_to_cm_rate

def find(func, data, mn, mx):
    loc = None
    for deg, dis in data:
        if mn <= deg <= mx:
            if loc is None:
                loc = dis
            else:
                loc = func(loc, dis)
    return loc

def read_gyro():
    x = gyro.euler[0]
    if x > 180: x -= 360
    x += shift_angle
    while abs(x) > 360:
        if x > 0:   x -= 360
        else:       x += 360
    if x > 180:     x -= 360
    elif x < -179:  x += 360
    return x

def height_rot():
    return gyro.euler[2]

def norm(steer):
    if steer == 1000:
        return 157
    elif steer == -1000:
        return -157
    return max(min(steer, max_pos), min_pos)

cmd = 0

def move(head, speed):
    global cmd
    speed = int(speed)
    head = int(norm(90 + PID(head) * trans_coeff))
    data = head.to_bytes(1, 'big') + speed.to_bytes(1, 'big', signed=True)
    checksum = (head + data[1]) & 0xFF
    frame = data + bytes([checksum])
    cmd += 1
    ser.write(frame)
    ser.flush()

def _encoder_loop():
    try:
        global pos
        buffer = bytearray()
        while True:
            buffer += ser.read(ser.in_waiting or 1)
            while len(buffer) >= 5:
                frame = buffer[:5]
                buffer = buffer[5:]
                data_bytes = frame[:4]
                checksum = frame[4]
                if sum(data_bytes) & 0xFF == checksum:
                    pos = int.from_bytes(data_bytes, 'big', signed=True)
                else:
                    buffer = buffer[1:]
            time.sleep(0.01)
    except Exception as e:
        print(e)

def read_encoder():
    thread = threading.Thread(target=_encoder_loop, daemon=True)
    thread.start()
