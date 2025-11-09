import time
import board
import pwmio
import usb_cdc
import rotaryio

ENCODER_PIN_1 = board.GP14
ENCODER_PIN_2 = board.GP15

MOTOR1_A = board.GP16
MOTOR1_B = board.GP17
SERVO_PIN = board.GP18

MAX_PWM = 65535
MIN_PWM = 35000

MAX_US = 2500
MIN_US = 500

MOTOR_FREQUENCY = 20000
SERVO_FREQUENCY = 50

pwm_a = pwmio.PWMOut(MOTOR1_A, frequency=MOTOR_FREQUENCY, duty_cycle=0)
pwm_b = pwmio.PWMOut(MOTOR1_B, frequency=MOTOR_FREQUENCY, duty_cycle=0)
pwm_c = pwmio.PWMOut(SERVO_PIN, frequency=SERVO_FREQUENCY)

def move_motor(speed):
    if not -100 <= speed <= 100:
        pwm_a.duty_cycle = MAX_PWM
        pwm_b.duty_cycle = MAX_PWM
    else:
        pwm_value = int((MAX_PWM - MIN_PWM) * abs(speed) / 100) + MIN_PWM

        if speed > 0:
            pwm_a.duty_cycle = pwm_value
            pwm_b.duty_cycle = 0
        elif speed < 0:
            pwm_a.duty_cycle = 0
            pwm_b.duty_cycle = pwm_value
        else:
            pwm_a.duty_cycle = 0
            pwm_b.duty_cycle = 0

def set_angle(head):
    head = max(min(head, 180), 0)
    us = MIN_US + (MAX_US - MIN_US) * head / 180
    duty = int(us * 65535 / (1_000_000 / SERVO_FREQUENCY))
    pwm_c.duty_cycle = duty

encoder = rotaryio.IncrementalEncoder(board.GP14, board.GP15)
last_position = encoder.position

usb_serial = usb_cdc.data
buffer = bytearray()

def debug(msg):
    usb_cdc.console.write((msg + '\r\n').encode())

def read_command():
    global buffer
    if usb_serial.in_waiting > 0:
        buffer += usb_serial.read(usb_serial.in_waiting)
    while len(buffer) >= 3:
        frame = buffer[:3]
        servo_val = frame[0]
        motor_raw = frame[1]
        checksum = frame[2]
        if checksum == ((servo_val + motor_raw) & 0xFF):
            motor_val = motor_raw
            if motor_val >= 128:
                motor_val -= 256
            buffer = buffer[3:]
            return servo_val, motor_val
        else:
            buffer = buffer[1:]
    return None

def send_command(value):
    data = value.to_bytes(4, 'big', signed=True)
    checksum = sum(data) & 0xFF
    if usb_serial.out_waiting < 64:
        usb_serial.write(data + bytes([checksum]))

move_motor(0)
set_angle(90)

try:
    while True:
        cmd = read_command()
        if cmd is not None:
            servo_angle, motor_speed = cmd
            set_angle(servo_angle)
            move_motor(motor_speed)

        # Test
        position = encoder.position
        if position != last_position:
            send_command(position)
            last_position = position

        time.sleep(0.001)

except KeyboardInterrupt:
    set_angle(90)
    move_motor(101)
    print("Stopped")
