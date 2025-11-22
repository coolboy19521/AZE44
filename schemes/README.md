# 1. Bill of Materials (BOM)

<table align="center">
  <tr>
    <th colspan="4">Electronics</th>
  </tr>

  <tr>
    <th>Component</th><th>Picture</th>
    <th>Component</th><th>Picture</th>
  </tr>

  <tr>
    <td>Raspberry Pi 5 x 1</td><td><img src="../media/rasppi.jpg" style="width:170px;height:170px;object-fit:contain;background:white;"></td>
    <td>Motor Driver x 1</td><td><img src="../media/driver.jpg" style="width:170px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>T-MINI-Plus Lidar TOF</td><td><img src="../media/lidar.jpeg" style="width:170px;height:170px;object-fit:contain;background:white;"></td>
    <td>BNO085 x 1</td><td><img src="../media/bno085.jpeg" style="width:170px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>3s Li-Po x 1</td><td><img src="../media/lipo.jpeg" style="width:170px;height:170px;object-fit:contain;background:white;"></td>
    <td>IMAX B6AC battery charger x 1</td><td><img src="../media/charger.jpg" style="width:170px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>Servo Motor TD8120MG x 1</td><td><img src="../media/digital.webp" style="width:170px;height:170px;object-fit:contain;background:white;"></td>
    <td>Raspberry Pi Camera Module 3 x 1</td><td><img src="../media/cam.webp" style="width:170px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>Encoder motor x 1</td><td><img src="../media/motor.jpeg" style="width:170px;height:170px;object-fit:contain;background:white;"></td>
    <td>Switch ON-OFF x 1</td><td><img src="../media/switch.webp" style="width:170px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>2,54 mm JST-XH Connector Male x 5</td><td><img src="../media/male.webp" style="width:170px;height:170px;object-fit:contain;background:white;"></td>
    <td>2,54 mm JST-XH Connector Female x 5</td><td><img src="../media/female.jpeg" style="width:170px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>XT-60 x 1</td><td><img src="../media/lipo_cable.png" style="width:170px;height:170px;object-fit:contain;background:white;"></td>
    <td>2.54 mm Pin Header x 1</td><td><img src="../media/pins.jpeg" style="width:170px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>Fuse x 1</td><td><img src="../media/fuse.jpg" style="width:170px;height:170px;object-fit:contain;background:white;"></td>
    <td>10K ohm resistor x 1</td><td><img src="../media/10k.jpg" style="width:170px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>330ohm resistor x 2</td><td><img src="../media/330.jpg" style="width:170px;height:170px;object-fit:contain;background:white;"></td>
    <td>GPIO cable x 1</td><td><img src="../media/ribbon.webp" style="width:170px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>Li-Po Battery Test Device Amper Monitor x 1</td><td><img src="../media/power_val.jpg" style="width:170px;height:170px;object-fit:contain;background:white;"></td>
    <td>Type-c - Type-c Kabel x 1</td><td><img src="../media/usbc.jpg" style="width:170px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>32 GB Storage Card x 1</td><td><img src="../media/sd.jpg" style="width:170px;height:170px;object-fit:contain;background:white;"></td>
    <td>Heat sink x 1</td><td><img src="../media/fan.webp" style="width:170px;height:170px;object-fit:contain;background:white;"></td>
  </tr>

  <tr>
    <td>HC-SR04 x 1</td><td><img src="../media/hcsr04.webp" style="width:170px;height:170px;object-fit:contain;background:white;"></td>
  </tr>
</table>

# 2. System Overview

### 2.1 Electronic System Architecture

**Raspberry Pi 5 (Main Controller):** The central processing unit
running the ROS-based software. It collects data from all sensors and
generates motor and servo commands.

**Raspberry Pi Pico 2 (Motor Controller):** Controls the encoder motor
and servo motor. It converts commands from the Raspberry Pi into
motor/servo movements and sends feedback data back to the Pi.

**Encoder DC Motor:** Provides forward and backward motion. Through the
encoder, it sends speed and position feedback to the Raspberry Pi via
the Motor Controller.

**Servo Motor:** Handles robot turns, with the servo angle controlled
via the motor controller.

2.2 Block Diagram:

# 3. Hardware and Component Introduction

### 3.1 Raspberry Pi 5

#### 3.1.1 Description and Features

<table align="center" style="width:80%; border-collapse:collapse;">
  <tr>
    <th colspan="3">Raspberry Pi 5</th>
  </tr>
  <tr>
    <td rowspan="3" style="width:400; text-align:center; vertical-align:middle; padding:0; margin:0;">
    <div>
      <img src="../media/raspb_up.png" alt="Raspberry Pi" />
      <p style="margin-top:0;" align="left"><i>Figure 1.1: Raspberry Pi 5</i></p>
    </div>
    </td>
    <td>USB</td>
    <td>Raspberry Pi Pico W and T-MINI-PLUS Lidar sensor</td>
  </tr>
  <tr>
    <td>CSI Camera Port</td>
    <td>Connects to Camera Module 3</td>
  </tr>
  <tr>
    <td>GPIO Pins</td>
    <td>Start button, imu, ultrasonic, leds and buzzer</td>
  </tr>
  <tr>
    <td colspan="3">Quad-core CPU, 64-bit @ 2.4 GHz, 8 GB RAM</td>
  </tr>
  <tr>
    <td colspan="3">2 × USB 3.0, 2 × USB 2.0 ports</td>
  </tr>
  <tr>
    <td colspan="3">40-pin GPIO header</td>
  </tr>
  <tr>
    <td colspan="3">USB-C power input 5V/5A</td>
  </tr>
  <tr>
    <td colspan="3">Powered by Li-Po 3S 1300 mAh battery</td>
  </tr>
</table>

Raspberry Pi 5 is a high-performance single-board computer used as
the main controller of the robot. Its purpose in our robot is:
- Gather lidar, camera, gyro, and encoder data. All of our sensors are connected to Raspberry Pi.
- Proccess the data gathered and make decisions. It could be some kind of a brain, if sensors could be considered as the sense organs.
- Send the decisions made to Raspberry Pi Pico. Raspberry Pi is connected to Raspberry Pi Pico via USB. More information about these kind of connections could be found in [`3.9.7 Scheme 1`](#control-board).

#### 3.1.2 Connections and Pin Configuration

<div>
  <img src="../media/raspberry_pinout.png" height="500" alt="Raspberry Pi Pinout" />
  <p style="margin-top:0;"><i>Figure 1.2: Raspberry Pi Pinout</i></p>
</div>

>[!NOTE]
>Refer to the Raspberry Pi 5 pinout for accurate connections. If you want to see connection diagram, you need to go to [`3.9.7 Scheme 1`](#control-board).

### 3.2 Raspberry Pi Pico W Control Board

#### 3.2.1 Description and Features

<table align="center" style="width:80%; border-collapse:collapse;">
  <tr>
    <th colspan="3">Raspberry Pi Pico 2 W</th>
  </tr>
  <tr>
    <td rowspan="3" style="width:400; text-align:center; vertical-align:middle; padding:0; margin:0;">
    <div>
      <img src="../media/ppico.jpg" alt="Raspberry Pi Pico" />
      <p style="margin-top:0;" align="left"><i>Figure 2.1: Raspberry Pi Pico 2 W</i></p>
    </div>
    </td>
    <td>Wireless Connectivity</td>
    <td>2.4 GHz Wi-Fi</td>
  </tr>
  <tr>
    <td>Communication Interfaces</td>
    <td>USB, UART, I²C, SPI, PWM, ADC</td>
  </tr>
  <tr>
    <td>GPIO Pins</td>
    <td>26 multifunctional pins</td>
  </tr>
  <tr>
    <td colspan="3">RP2350 microcontroller, Dual-core ARM (up to 150 MHz)</td>
  </tr>
  <tr>
    <td colspan="3">USB 5 V input, regulated 3.3 V output (maximum 300 mA)</td>
  </tr>
  <tr>
    <td colspan="3">520 KB SRAM, 4 MB Flash</td>
  </tr>
</table>

**Raspberry Pi Pico W** is a compact and efficient microcontroller board designed for real-time embedded control, IoT, and robotic applications. It features a dual-core ARM Cortex-M0+ processor, integrated Wi-Fi connectivity, and flexible GPIO pins supporting multiple communication and control interfaces. Its purpose in our robot is:
- Read encoder sensor and send them to Raspberry Pi. Encoder sensors are connected to Raspberry Pi Pico. More information on this could be found in [`3.9.7 Scheme 1`](#control-board).
- Read motor command values from Raspberry Pi and send them to motor driver. Motor drivers are connected to Raspberry Pi Pico but the decisions are made in Raspberry Pi. That's why Raspberry Pi Pico should read the decisions from the Raspberry Pi.

#### 3.2.2 Connections and Pin Configuration

<div>
  <img src="../media/pico_pinout.png" height="500" alt="Raspberry Pi Pico 2 Pinout" />
  <p style="margin-top:0;"><i>Figure 2.2: Raspberry Pi Pico 2 Pinout</i></p>
</div>

>[!NOTE]
>Refer to the Raspberry Pi Pico W pinout for accurate connections.  If you want to see connection diagram, you need to go to [`3.9.7 Scheme 2`](#motor-driver).

### 3.3 TB6612FNG Control Board

#### 3.3.1 Description and Features

<table align="center" style="width:80%; border-collapse:collapse;">
  <tr>
    <th colspan="3">TB6612FNG</th>
  </tr>
  <tr>
    <td rowspan="3" style="height:300px; text-align:center; vertical-align:middle; padding:0; margin:0;">
    <div>
      <img src="../media/tb6612fng.jpeg" height="300px" alt="TB6612FNG" />
      <p style="margin-top:0;" align="left"><i>Figure 3.1: TB6612FNG</i></p>
    </div>
    </td>
    <td>Operating voltage</td>
    <td>2.5V – 13.5V</td>
  </tr>
  <tr>
    <td>Continuous output current</td>
    <td>1.2A per channel (peak up to 3.2A)</td>
  </tr>
  <tr>
    <td>Logic voltage</td>
    <td>2.7V – 5.5V</td>
  </tr>
  <tr>
    <td colspan="3">Dual H-Bridge motor driver (can control 2 DC motors)</td>
  </tr>
  <tr>
    <td colspan="3">PWM control supported (up to 100 kHz)</td>
  </tr>
</table>

#### 3.3.2 Connections and Pin Configuration

<div>
  <img src="../media/tb6612fng-pinout.png" height="400" alt="TB6612FNG Pinout" />
  <p style="margin-top:0;"><i>Figure 3.2: TB6612FNG Pinout</i></p>
</div>

## 3.4 Motors and Servo Motor

### 3.4.1 Powering and Controlling

**DC Motor:**
- Connected to Raspberry Pi Pico W via encoder motor ports.
- Encoder feedback allows precise speed and position control.
- Controlled using PWM signals.

**Servo Motor:**
- Controlled via PWM from Raspberry Pi Pico W 


### 3.4.2 DC Gear Motors

#### 3.4.2.1 Description and Features

<table align="center" style="width:80%; border-collapse:collapse;">
  <tr>
    <th colspan="3">Gear Motor</th>
  </tr>
  <tr>
    <td rowspan="3" style="height:350px; text-align:center; vertical-align:middle; padding:0; margin:0;">
    <div>
      <img src="../media/motor.png" height="350px" alt="Gear Motor" />
      <p style="margin-top:0;" align="left"><i>Figure 4.1: Gear Motor</i></p>
    </div>
    </td>
    <td>Operating voltage</td>
    <td>5–12V DC</td>
  </tr>
  <tr>
    <td>Rotation Speed</td>
    <td>450 RPM @ 12V</td>
  </tr>
  <tr>
    <td>Encoder Voltage</td>
    <td>3.3–5V DC</td>
  </tr>
  <tr>
    <td colspan="3">Cylindrical brushed DC gear motors, available in 5 variants (two 6V, three 12V) with wide gear ratios.</td>
  </tr>
  <tr>
    <td colspan="3">No-load Current: 200 mA, Rated Current: 300 mA, Stall Current: 800 mA</td>
  </tr>
</table>

This motor is very advantageous for us, because it also has a built in encoder sensor. We use this encoder sensor to move certain amount of distance. It comes especially handy when going a distance according to lidar decisions. For example:
- Let's say we need to move until robot's front distance is less than some amount (say `40 cm`).
- If we write a conditional and move until some lidar value is achieved we can overshoot. That is because of low lidar frequency (which is `~5-7 hz`, this means robot can move forwards without considering lidar for `~142-200 ms`).
- But if we use encoder values and let's say start with `70 cm` distance from the wall, we can command the robot to go `30 cm`. This conditional is not bound to frequency of any electronical component ans is upto `~15-20 hz`. Which means robot can miss only for `~50-66 ms` (which is less than half).

>[!IMPORTANT]
>Motors can operate above or below nominal voltage; high voltages
may reduce lifespan.

#### 3.4.2.2 Connections and Pin Configuration

<div>
  <img src="../media/motor_pinout.png" width="700" alt="Motor Pinout" />
  <p style="margin-top:0;"><i>Figure 4.2: Gear Motor Pinout</i></p>
</div>

### 3.4.3 Servo Motor (TD8120MG):

#### 3.4.3.1 Description and Features

<table align="center" style="width:80%; border-collapse:collapse;">
  <tr>
    <th colspan="3">Servo Motor</th>
  </tr>
  <tr>
    <td rowspan="4" style="height:350px; text-align:center; vertical-align:middle; padding:0; margin:0;">
    <div>
      <img src="../media/servo.webp" height="350px" alt="Gear Motor" />
      <p style="margin-top:0;" align="left"><i>Figure 5.1: Servo Motor</i></p>
    </div>
    </td>
    <td>Rotation Angle</td>
    <td>0°–180°</td>
  </tr>
  <tr>
    <td>Speed</td>
    <td>0.13 s / 60° @ 6.0V</td>
  </tr>
  <tr>
    <td>Torque</td>
    <td>20 kg·cm @ 6.0V, 23 kg·cm @ 7.4V</td>
  </tr>
  <tr>
    <td>Operating Voltage</td>
    <td>4.8–8.4V</td>
  </tr>
  <tr>
    <td colspan="3">Digital control system, metal gearbox</td>
  </tr>
</table>

We are using this digital servo for our steering (using an Ackermann controller). We previously used an analog servo but we switched because of some reasons:
- Digital servos have more frequency. They accept commands with smaller delays than analog servos.
- They turn faster and more on point. While an analog servo slowly reaches some steering point digital servos change their direction almost instantly.
- Has a greater precision. In comparison to analog servos we observed higher precision on turns, almost all turns made between countless tries were the same.

#### 3.4.3.2 Connections and Pin Configuration

<div>
  <img src="../media/servo_pinout.png" width="700" alt="Servo Pinout" />
  <p style="margin-top:0;"><i>Figure 5.2: Servo Pinout</i></p>
</div>

### 3.5 LiDAR Sensor

#### 3.5.1 Definition and Model Features

Yahboom T-mini Plus LiDAR is a 2D lidar used for simple obstacle detection. Its main advantage is that it utilizes Time-of-Flight (ToF) measurement technology to deliver fast and accurate distance data. With high sampling frequency and good-enough
ambient light resistance, it performs well for our strategy.

<table align="center" style="width:80%; border-collapse:collapse;">
  <tr>
    <th colspan="3">Yahboom T-mini Plus LiDAR</th>
  </tr>
  <tr>
    <td rowspan="4" style="height:370px; text-align:center; vertical-align:middle; padding:0; margin:0;">
    <div>
      <img src="../media/lidar.jpg" height="370px" alt="Gear Motor" />
      <p style="margin-top:0;" align="left"><i>Figure 6.1: Yahboom T-mini Plus LiDAR</i></p>
    </div>
    </td>
    <td>Measurement Range</td>
    <td>0.1 – 12 m (reflectivity-dependent)</td>
  </tr>
  <tr>
    <td>Scan Frequency</td>
    <td>1 – 10 Hz adjustable</td>
  </tr>
  <tr>
    <td>Sampling Rate</td>
    <td>up to 5000 samples/s</td>
  </tr>
  <tr>
    <td>Power requirements</td>
    <td>5V 300mA</td>
  </tr>
</table>

Lidar is used for scanning along the robot's Z-axis. It gives us output in (angle in radians, distance in meters) format. We convert it into (angle in degrees, distance in centimeters) format to make it easier for us to use the values and visualize it. In our robot lidar is used for: detecting the obstacles and finding the tilt of robot according to its surroundings. You can find more info about its usage in [Programming documentation](/src/).

## 3.6 Camera

We are using a camera for detecting colors. There is no other usage of camera in our robot, as we are detecting the obstacle without it (as mentioned before we are using the lidar).

### 3.6.1 Definition and Features

<table align="center" style="width:80%; border-collapse:collapse;">
  <tr>
    <th colspan="3">Camera Module 3</th>
  </tr>
  <tr>
    <td rowspan="3" style="height:370px; text-align:center; vertical-align:middle; padding:0; margin:0;">
    <div>
      <img src="../media/camera.avif" height="370px" alt="Gear Motor" />
      <p style="margin-top:0;" align="left"><i>Figure 7.1: Camera Module </i></p>
    </div>
    </td>
    <td>Resolution</td>
    <td>11.9 MP (4608 × 2592)</td>
  </tr>
  <tr>
    <td>Pixel Size</td>
    <td>1.4 μm × 1.4 μm</td>
  </tr>
  <tr>
    <td>Video Modes</td>
    <td>1080p50 / 720p100 / 480p120</td>
  </tr>
  <tr>
    <td colspan="3">Connects via CSI-2 camera port</td>
  </tr>
  <tr>
    <td colspan="3">Power is supplied directly from the Raspberry Pi board via CSI-2
  interface</td>
  </tr>
</table>

The reason why we selected this camera is that it supports HDR mode, phase-detection autofocus (PDAF) for fast focusing. This is the main take-point for us, as the resolution of the camera is not really different between similar models. We already use it in `1586 x 864` mode for lower system usage and faster proccess time.

### 3.7 Gyroscope Sensor

#### 3.7.1 Definition and Features

<table align="center" style="width:80%; border-collapse:collapse;">
  <tr>
    <th colspan="3">BNO085</th>
  </tr>
  <tr>
    <td rowspan="2" style="height:370px; text-align:center; vertical-align:middle; padding:0; margin:0;">
    <div>
      <img src="../media/bno085.webp" height="370px" alt="Gear Motor" />
      <p style="margin-top:0;" align="left"><i>Figure 8.1: BNO085</i></p>
    </div>
    </td>
    <td>Orientation output</td>
    <td>Euler vector, up to 200 Hz</td>
  </tr>
  <tr>
    <td>Operating voltage</td>
    <td>3.3V typical 3.5mA</td>
  </tr>
</table>

Our gyro has a built-in fusion algorithm, which means it can in theory fuse accelerometer, magnotemeter and gyroscope. Despite all the advantages in theory, our IMU has a great drift over time. To fix this issue we implemented some algorithms and they are referenced in [Programming documentation](/src/).

### 3.8 Ultrasonic Sensor

#### 3.8.1 Definition and Features

<table align="center" style="width:80%; border-collapse:collapse;">
  <tr>
    <th colspan="3">HC-SR04</th>
  </tr>
  <tr>
    <td rowspan="4" style="height:370px; text-align:center; vertical-align:middle; padding:0; margin:0;">
    <div>
      <img src="../media/hcsr04_eyb.webp" height="370px" alt="Gear Motor" />
      <p style="margin-top:0;" align="left"><i>Figure 9.1: HC-SR04</i></p>
    </div>
    </td>
    <td>Operating Voltage & Current</td>
    <td>5V DC, 15 mA</td>
  </tr>
  <tr>
    <td>Ranging Distance</td>
    <td>2 cm to 400 cm</td>
  </tr>
  <tr>
    <td>Ranging Distance</td>
    <td>2 cm to 400 cm</td>
  </tr>
  <tr>
    <td>Accuracy</td>
    <td>~3mm</td>
  </tr>
  <tr>
    <td colspan="3">Measuring Angle,<15∘ cone;</td>
  </tr>
  <tr>
    <td colspan="3">A whopping frequency of 40kHz</td>
  </tr>
</table>

Altough this sensor support `2cm-400cm` range in theory, in reality with Raspberry Pi the best we could achive is a `10cm-70cm` range. Despite the difference this range is enough for us.

As we anticipated, the main technical challenge with our ultrasonic sensor is the voltage mismatch: the HC-SR04 sensor outputs its signal at 5 Volt (5V) logic, but the Raspberry Pi's GPIO pins are strictly limited to 3.3V. Connecting them directly would risk damaging the Pi's processor.

To solve this issue we developed a solution: By using specific resistors (i.e 1kΩ and 2kΩ) on the sensor's ECHO output line, we have successfully stepped down the voltage from 5V to a safe level of approximately 3.3V. This setup guarantees that the Raspberry Pi is protected from overvoltage damage while still receiving an accurate, readable digital signal.

To know which resistors should be used, we took advantage of level-shifting formula:

$$V_{\text{out}} = V_{\text{in}} \times \frac{R_2}{R_1 + R_2} \implies 5\text{V} \times \frac{2\text{k}\Omega}{1\text{k}\Omega + 2\text{k}\Omega} \approx 3.33\text{V}$$

<table align="center" cellspacing="0" cellpadding="0" style="margin:0; padding:0; border-collapse:collapse;">
  <tr>
    <th colspan="2">Level Shifting</th>
  </tr>
  <tr>
    <td align="center" style="margin:0; padding:0;"><img src="../media/shift_1.png" height="300"><br><i>Figure 9.2: Diagram to show how voltage is remapped</i></td>
    <td align="center"><img src="../media/shift_2.png" height="300"><br><i>Figure 9.3: To show the electric circuit</i></td>
  </tr>
</table>

**3.9. Custom PCB Design**

#### 3.9.1. Function and Purpose

To make our electronics more compact we decided to design a custom PCB. The PCB includes our LEDs, buzzer, button, switch and etc.

#### 3.9.2. Circuit Elements and Layer Structure

The circuit is built on a dual-layer PCB and includes the following key components:

- **Two LEDs (red and green):** Used as status and warning indicators.
- **Buzzer:** Provides audible alerts.
- **Push button:** User input control.
- **Fuse:** Overcurrent protection.
- **BNO085:** Orientation and motion sensing.
- **One switch:** Power on/off control.
- **Raspberry Pi 5 GPIO connections:** For system control and
  communication.
- **Ultraconic sensor:** For measure distance.
- **Capacitor:** For sudden surges.
- **One ferrit bead:** For EMI.

On the PCB, power lines are routed with thick copper wires, while signal lines are routed with thinner wires to ensure current handling capacity and minimize interference.

<table align="center" cellspacing="0" cellpadding="0" style="margin:0; padding:0; border-collapse:collapse;">
  <tr>
    <th colspan="2">Board 1</th>
  </tr>
  <tr>
    <td align="center" style="margin:0; padding:0;"><img src="../media/board_1.png" height="300"><br><i>Figure 10.1: 3D view of first PCB board</i></td>
    <td align="center"><img src="../media/board_1_2d.jpg" height="300"><br><i>Figure 10.2: 2D view of first PCB board</i></td>
  </tr>
</table>

<table align="center" cellspacing="0" cellpadding="0" style="margin:0; padding:0; border-collapse:collapse;">
  <tr>
    <th colspan="2">Board 2</th>
  </tr>
  <tr>
    <td align="center" style="margin:0; padding:0;"><img src="../media/board_2.png" height="300"><br><i>Figure 10.3: 3D view of second PCB board</i></td>
    <td align="center"><img src="../media/board_2_2d.jpg" height="300"><br><i>Figure 10.4: 2D view of second PCB board</i></td>
  </tr>
</table>

#### **3.9.3. Connection Points**

- **Power Input:** Operates with an external power supply of **6--14
  V**.
- **Raspberry Pi 5 GPIO:** Direct connections for LEDs, button, buzzer,
  and the sensor.
- **Sensor Connection:** The BNO085 IMU is connected via the **I²C
  interface (SCL, SDA)** to the Raspberry Pi 5.
- **Connectors:** JST and pin header connectors are used for both power
  and signal interfaces.

#### 3.9.4 Hardware Issues and Troubleshooting Process

At the beginning, we were using the **Hiwonder RRC Lite Controller**. At first glance, it appeared to be an ideal solution for our needs --- a single board that could easily interface with the Raspberry Pi, included a buzzer, button, encoder motor driver, and servo
ports, and seemed capable of handling all our system requirements.

However, as we started using it, we began experiencing connection
losses between the Raspberry Pi 5 and the controller while the
robot was operating. Initially, we didn't pay much attention to it since
the disconnections were quite rare. But over time, the issue worsened
--- what started as a once-a-week problem eventually became three to
four disconnections per day during testing.

We investigated the possible causes. Our first assumption was **EMI
(Electromagnetic Interference)** since the data cable was routed close
to the motor. To address this, we added a ferrite bead to the cable,
but it didn't help. Then we suspected a power issue, so we separated
the power connection and supplied the Raspberry Pi from an external
power bank instead. Unfortunately, the problem persisted.

We also added capacitors to the power input and replaced the cables,
yet the RRC Lite Controller continued to malfunction. Eventually, we
decided to design our own motor driver board to ensure stable
operation.

<table align="center" cellspacing="0" cellpadding="0" style="margin:0; padding:0; border-collapse:collapse;">
  <tr>
    <th colspan="2">Motor driver board</th>
  </tr>
  <tr>
    <td align="center" style="margin:0; padding:0;"><img src="../media/board_pico.png" height="300"><br><i>Figure 10.5: 3D view of our motor driver PCB board</i></td>
    <td align="center"><img src="../media/board_pico_2d.jpg" height="300"><br><i>Figure 10.6: 2D view of our motor driver PCB board</i></td>
  </tr>
</table>

#### 3.9.5 PCB Layer Structure and Design Rules

- The PCB design was implemented on a **two-layer FR4 board**,
  manufactured by **JLCPCB**.
- The **top layer** carries most of the signal traces, while the
  **bottom layer** mainly serves as a **ground plane** to ensure stable
  reference potential and reduce noise.
- **Thicker copper traces** were used for power distribution lines,
  while **thinner traces** were used for signal routing.
- Design rules followed JLCPCB's recommended minimum clearance, trace
  width, and via size specifications.
- The layout was optimized to **minimize electromagnetic interference
  (EMI)** between the IMU sensor and the power circuits.
- A solid ground plane helps reduce sensor noise and improve overall
  system stability.

#### 3.9.6 Connectors and Component Placement

- **Connectors:** JST connectors and pin headers were used to allow easy
  connection of external modules such as the battery, button, and
  display.
- **Placement:**
  - The **Raspberry Pi 5 GPIO header** is positioned near the center for
    organized wiring and balanced routing.
  - **LEDs, buzzer, and button** are placed at the edge of the board for
    easy user access.
  - **Power input, fuse, and switch** are grouped together in a
    dedicated **power section**.
  - The **IMU sensor** is placed close to the GPIO pins to keep I²C
    lines short and reduce signal distortion.
- The two-layer layout also allows sensitive signal traces (such as I²C
  lines) to be routed over a continuous ground plane for improved noise
  shielding.

#### 3.9.7 Circuit Schemes

<a id="control-board"></a>

<table align="center" cellspacing="0" cellpadding="0" style="margin:0; padding:0; border-collapse:collapse;">
  <tr>
    <th colspan="1">Circuit Schemes</th>
  </tr>
  <tr>
    <td align="center" style="margin:0; padding:0;"><a id="motor-driver"></a><img src="control_board.svg"><br><i>Figure 10.7: Control Board scheme</i></td>
  </tr>
  <tr>
    <td align="center"><img src="motor_board.svg"><br><i>Figure 10.8: Motor Driver scheme</i></td>
  </tr>
</table>

### 3.9.8 Manufacturing and Assembly Notes

- The PCB was fabricated by **JLCPCB** using **FR4 material (1.6 mm
  thickness, 1 oz copper)** with **green solder mask** and **white
  silkscreen**.
- The inclusion of **solder mask** and **silkscreen layers** simplified
  soldering and improved assembly accuracy.
- Assembly was performed manually, using **through-hole (THT)**
  components and a few **surface-mount (SMD)** parts.
- **Thicker traces** were used for power paths, while **narrower ones**
  were used for signal lines.
- Compared to the initial perfboard prototype, the final 2-layer PCB
  offers **greater durability, improved signal integrity, and a more
  professional appearance**.

**3.9.9 GERBER Files**

>[!NOTE]
> If you want to order PCB(Printed Circuit Board), download these `GERBER` files and follow the steps in the video.

<table align="center" cellspacing="0" cellpadding="0" style="margin:0; padding:0; border-collapse:collapse;">
  <tr>
    <th colspan="4">PCB Files</th>
  </tr>
  <tr>
    <th>Name</th>
    <th>GERBER</th>
    <th>BOM</th>
    <th>Pick & Place</th>
  </tr>
  <tr>
    <td>Raspberry Pi Distributon Board 1</td>
    <td><a href="dist1.zip">dist1.zip</a></td>
    <td><a href="bom1.xlsx">bom1.xlsx</a></td>
    <td><a href="pnp1.xlsx">pnp1.xlsx</a></td>
  </tr>
  <tr>
    <td>Raspberry Pi Distributon Board 2</td>
    <td><a href="dist2.zip">dist1.zip</a></td>
    <td><a href="bom2.xlsx">bom1.xlsx</a></td>
    <td><a href="pnp2.xlsx">pnp1.xlsx</a></td>
  </tr>
  <tr>
    <td>Motor Driver</td>
    <td><a href="dist3.zip">dist1.zip</a></td>
    <td><a href="bom3.xlsx">bom1.xlsx</a></td>
    <td><a href="pnp3.xlsx">pnp1.xlsx</a></td>
  </tr>
</table>

> [!NOTE]
> If you don\'t know how to order a PCB, click the link and watch this [video](https://www.youtube.com/watch?v=SGsfiHOE9Fk&t=466s).

## 3.10 Power Supply (3S Li-Po Battery)

### 3.10.1 Battery Specifications (Voltage, Capacity, C Rating)

The power source of the project is a **JetFire 3S Li-Po battery**,
specifically designed for high-current demanding applications such as
robotics, drones, and defense systems. Thanks to its high discharge rate
and reliable cell quality, it ensures stable and long-term operation.

Technical Specifications:
- Voltage: **11.1 V** (nominal)
- Capacity: **1300 mAh**
- Cells: **3S (3 cells in series)**
- Discharge Rate: **50C continuous / 100C peak (max 10s)**
- Weight: **110 g**
- Dimensions: **74 × 33 × 21 mm**
- Discharge Connector: **XT60 (black)**
- Charging Connector: **JST-XHR (white)**

### 3.10.2 Power Management Circuits (BMS, Charging Circuit)

Li-Po batteries require safe charging and monitoring systems. A
**Battery Management System (BMS)** or balance charger must be used to:
- Monitor individual cell voltages,
- Prevent overcharging or deep discharge,
- Provide safe current limits during operation.

### 3.10.3 Connections and Connectors

The battery is connected to the system using an **XT60 connector** for
power delivery and a **JST-XHR balance connector** for charging and
monitoring. Proper wiring and insulation are critical to ensure safe
operation and prevent short circuits.

### 3.10.4 How to Charge the Battery (with iMAX B6AC)

The **JetFire 3S Li-Po battery** must be charged using a balance charger
such as the **iMAX B6AC** to ensure safe and efficient charging. The
iMAX B6AC allows monitoring of individual cell voltages and prevents
overcharging.

Charging Procedure:
1.  Connection:
    - Plug the **XT60 connector** into the charger's discharge port.
    - Connect the **JST-XHR balance connector** to the charger's balance
      port.
2.  Charger Setup:
    - Select **LiPo BALANCE CHARGE** mode.
    - Set the battery type to **LiPo** and cell count to **3S (11.1V)**.
    - Set the charging current to **1.3A** (equal to the battery's 1C
      rate).
3.  Charging Process:
    - Start the charging process and monitor the charger display.
    - Ensure that each cell voltage remains within the safe range
      (**3.7V -- 4.2V**).
4.  Completion:
    - Charging is complete when the total voltage reaches approximately
      **12.6V (4.2V per cell)**.
    - Disconnect the battery from the charger and store it safely.

>[!IMPORTANT]
>If you want to see charging instructions, you can watch this [video](https://www.youtube.com/watch?v=WvyrB9QOp4Y)

## 4. Power Distribution and Management

### 4.1 Power Distribution Diagram

<div>
  <img src="../media/lipo_bat.webp" height="400px" alt="Lipo" />
  <p style="margin-top:0;"><i>Figure 11.1 Lipo Battery</i></p>
</div>

The 11.1V 3S Li-Po battery supplies the entire system. Power is
distributed through a central board, separating high-current lines
(motors, drivers) and low-current lines (controllers, sensors) for
stability.

### 4.2 Voltage Regulators and Converters (5V, 3.3V, etc.)
- **5V:** Raspberry Pi, LiDAR, camera.
- **3.3V:** IMU and low-power sensors.
- **11.1V Direct:** Motors and motor drivers.  
  Switching regulators are used for efficiency.

### 4.3 Battery Management System (BMS, Charging Control)

The BMS protects the Li-Po battery from overcharge, overdischarge, short
circuit, and overheating. Charging is done with a balance charger (iMAX
B6AC) to ensure safe and balanced operation.

### 4.4 Protection Mechanisms (Fuse, Overcurrent Protection)

Fuses, polyfuses, reverse polarity, and voltage monitoring circuits are
added to prevent system failures and protect sensitive electronics.

<hr>

<p align="center">
  <picture>
  <source media="(prefers-color-scheme: dark)" srcset="../media/my_image.png">
  <img height="400" alt="logo" src="../media/my_image_light.png">
  </picture>
</p>