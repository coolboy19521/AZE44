**1. Bill of Materials**

<table>
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
    <td>HCSR-04 x 1</td><td><img src="../media/hcsr04.webp" style="width:170px;height:170px;object-fit:contain;background:white;"></td>
  </tr>
</table>

**2. System Overview**

**2.1 Electronic System Architecture**

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

**3. Hardware and Component Introduction**

**3.1 Raspberry Pi 5**

**3.1.1 Description and Features**

<table style="width:80%; border-collapse:collapse;">
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
    <td>Raspberry Pi Pico 2W and T-MINI-PLUS Lidar sensor</td>
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

The Raspberry Pi 5 is a high-performance single-board computer used as
the main controller of the robot. It runs ROS to process sensor data,
control actuators, and manage overall system operations.

**3.1.2 Connections and Pin Configuration**

>[!NOTE]
> Refer to the Raspberry Pi 5 pinout for accurate connections.

<div>
  <img src="raspberry_pinout.png" height="500" alt="Raspberry Pi Pinout" />
  <p style="margin-top:0;"><i>Figure 1.2: Raspberry Pi Pinout</i></p>
</div>

**3.2 Raspberry Pi Pico 2W Control Board**

**3.2.1 Description and Features**

<table style="width:80%; border-collapse:collapse;">
  <tr>
    <th colspan="3">Raspberry Pi Pico 2 W</th>
  </tr>
  <tr>
    <td rowspan="3" style="width:400; text-align:center; vertical-align:middle; padding:0; margin:0;">
    <div>
      <img src="../media/ppico.jpg" alt="Raspberry Pi Pico" />
      <p style="margin-top:0;" align="left"><i>Figure 1.3: Raspberry Pi Pico 2 W</i></p>
    </div>
    </td>
    <td>USB</td>
    <td>Raspberry Pi Pico 2W and T-MINI-PLUS Lidar sensor</td>
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

The **Raspberry Pi Pico 2W** is a compact and efficient microcontroller
board designed for real-time embedded control, IoT, and robotic
applications. It features a dual-core ARM Cortex-M0+ processor,
integrated Wi-Fi connectivity, and flexible GPIO pins supporting
multiple communication and control interfaces.

• **Main Control Chip:** RP2350 microcontroller, Dual-core ARM
Cortex-M33 (up to 150 MHz)  
• **Wireless Connectivity:** 2.4 GHz 802.11 b/g/n Wi-Fi (Infineon
CYW43439 chip)  
• **GPIO:** 26 multifunctional pins supporting UART, I²C, SPI, PWM, and
ADC  
• **PWM Control:** Hardware PWM on up to 16 channels for motor or servo
control  
• **ADC Inputs:** 3 × 12-bit analog inputs (up to 3.3 V)  
• **Power Supply:** USB 5 V input, regulated 3.3 V output (maximum 300
mA)  
• **External Power Options:** VSYS pin accepts 1.8 -- 5.5 V input for
battery or external supply  
• **Communication Interfaces:** USB, UART, I²C, SPI, PWM, ADC  
• **Memory:** 520 KB SRAM, 4 MB Flash  
• **Operating Voltage:** 3.3 V logic level  
• **Size:** 51 × 21 mm  
• **Weight:** \~4 g

**3.2.2 Motor and Servo Connections**

• **PWM Control:** Use PWM-capable GPIO pins to drive motor drivers or
servo controllers.  
• **External Motor Driver:** Required for DC/encoder motors (e.g.,
L298N, DRV8833, or TB6612FNG).  
• **Servo Control:** Standard PWM servos can be driven directly (with
external 5 V power).

<div>
  <img src="pico_pinout.png" height="500" alt="Raspberry Pi Pinout" />
  <p style="margin-top:0;"><i>Figure 1.4: Raspberry Pi Pinout</i></p>
</div>

**3.2.3 Power Requirement**

• Accepts power from USB (5 V) or VSYS (1.8 -- 5.5 V).  
• Provides regulated 3.3 V logic output for sensors and communication
modules.  
• External power source (5 V 2 A recommended) required for motors and
servos.  
• On-board protection includes resettable fuse and ESD protection.

**3.2.4 Communication Interfaces**

• **USB:** For programming and serial communication via Micro-USB.  
• **UART / I²C / SPI:** Interfaces available through GPIO pins for
sensors , and external motor drivers.  
• **Wi-Fi:** Supports TCP/IP communication, OTA updates, and wireless
control.

## **3.3 Motors and Servo Motor**

### **3.3.1 Motor Types and Technical Specifications**

**DC Gear Motors:**

- Cylindrical brushed DC gear motors, available in 5 variants (two 6V,
  three 12V) with wide gear ratios.

- Housing Diameter: 25 mm, Shaft: 4 mm D-shaped.

- Operating Voltage: 5--12V DC, Encoder Voltage: 3.3--5V DC

- No-load Current: 200 mA, Rated Current: 300 mA, Stall Current: 800 mA

- Rotation Speed: 1000 RPM @ 12V

- Motor Size: 25 × 71 mm, Weight: 95 g

Note: Motors can operate above or below nominal voltage; high voltages
may reduce lifespan.

### Servo Motor (TD8120MG):

• **High-torque digital servo** designed for precise and powerful motion
control.  
• **Rotation Angle:** 0°--180°  
• **Speed:** 0.13 s / 60° @ 6.0V  
• **Torque:** 20 kg·cm @ 6.0V, 23 kg·cm @ 7.4V  
• **Digital control system** for stable and accurate positioning  
• **Metal gearbox** ensuring strength and long service life  
• **Operating Voltage:** 6.0--7.4V  
• **Dimensions:** 40.5 × 20 × 38 mm  
• **Weight:** 60 g

### **3.3.2 Connections and Driver Integration**

**DC Motors:**

- Connected to **Raspberry Pi Pico 2W** via encoder motor ports.

- Encoder feedback allows precise speed and position control.

- Controlled using PWM signals.

**Servo Motor:**

- Controlled via PWM from **Raspberry Pi Pico 2W** or microcontroller.

- Provides precise angular movement for turning or positioning.

### **3.3.3 Power Requirements**

**DC Motors:** 5--12V DC, regulated by **Raspberry Pi Pico 2W**.  
**Servo Motor:** 4.8--6V DC, powered via regulated supply.

### **3.3.4 Control Signals**

**DC Motors:** PWM signals for speed, encoder feedback for rotation
monitoring.  
**Servo Motor:** PWM signals determine angular position, controlled in
real time by ROS or microcontroller.

**3.4 LiDAR Sensor**

**3.4.1 Definition and Model Features**

The **Yahboom T-mini Plus LiDAR** is a compact, cost-effective
**single-line 2D laser ranging sensor** designed for mapping,
navigation, and obstacle detection in mobile robots. It utilizes
**Time-of-Flight (ToF)** measurement technology to deliver fast and
accurate distance data. With high sampling frequency and excellent
ambient light resistance, it performs reliably in both indoor and
outdoor environments.

**Key Features:**  
• **Measurement Range:** 0.1 -- 12 m (reflectivity-dependent)  
• **Accuracy:** ±3 cm  
• **Scan Frequency:** 1 -- 10 Hz adjustable  
• **Sampling Rate:** up to 5000 samples/s  
• **Light Source:** 850 nm infrared laser, Class 1 eye-safe  
• **Interface:** UART / USB (plug-and-play supported)  
• **Compact and lightweight:** 42 × 15 × 16 mm, 13 g  
• **Stable output and low power consumption**  
• **Suitable for SLAM, robot navigation, obstacle detection, and
localization**

**3.4.2 Connections**

• **Connected to Raspberry Pi 5** (or Pico 2W) **via USB or UART
interface**  
• **Plug-and-play compatible** with Yahboom ROS systems and Python SDK  
• **Motor/scan speed adjustable** via software configuration or PWM
control

**3.4.3 Power Requirements**

• **Power Supply:** 5V DC (via USB or external 5V pin)  
• **Typical Current Consumption:** \< 300 mA  
• **Low power design**, optimized for mobile robot applications

**3.4.4 Communication Protocol**

• **UART / USB serial communication**  
• Provides **real-time distance and angle data**  
• Fully supported by **Yahboom ROS packages** for SLAM mapping, obstacle
avoidance, and path planning

## **3.5 Camera**

### **3.5.1 Definition and Features**

The Raspberry Pi Camera Module v3 is equipped with the **Sony IMX708
image sensor**, offering **11.9 megapixel resolution** with **4608 ×
2592 pixels**. It supports **HDR mode**, **phase-detection autofocus
(PDAF)** for fast focusing, and an **integrated IR cut filter**. Thanks
to its **high signal-to-noise ratio (SNR)** and **dynamic defective
pixel correction (DPC)**, it ensures clear and stable image quality.

**Key Specifications:**

- Resolution: 11.9 MP (4608 × 2592)

- Pixel Size: 1.4 μm × 1.4 μm

- Sensor Size: 7.4 mm

- Video Modes: 1080p50 / 720p100 / 480p120

- Output Format: RAW10

- Size: 25 × 24 × 11.5 mm

- Weight: Lightweight module for robotics applications

### **3.5.2 Connections**

- Interface: **15-pin 1 mm pitch FPC connector**

- Ribbon Cable: **150 mm** included

- Compatible with **all Raspberry Pi boards (except Raspberry Pi 5)**

- Connects via **CSI-2 camera port**

### **3.5.3 Power Requirements**

- Power is supplied directly from the **Raspberry Pi board** via CSI-2
  interface

- Low power consumption, optimized for embedded vision applications

### **3.5.4 Data Transmission Interface**

- **CSI-2 serial data output** for image and video streaming

- **2-wire serial communication** for control signals

- **2-wire autofocus control** for lens mechanism

- Supports high-bandwidth transfer for real-time video processing

## 

## **3.6 Gyroscope (IMU) Sensor**

**3.6.1 Definition and Features**

The **Adafruit BNO085 9-DOF IMU Sensor** is an advanced **smart sensor
module** developed by Bosch and Hillcrest Labs. It integrates a **3-axis
accelerometer**, **3-axis gyroscope**, and **3-axis magnetometer** with
an **onboard 32-bit sensor fusion processor**, delivering accurate,
drift-compensated orientation data without requiring complex external
calculations.  
Compared to the BNO055, the BNO085 offers **higher precision, improved
sensor fusion algorithms (SH-2), lower latency**, and **enhanced
performance for robotics and AR/VR applications**.

**Key Features:**  
• **Absolute orientation output** (Euler vector, up to 200 Hz)  
• **Quaternion output** (4-point data, up to 200 Hz)  
• **Angular velocity vector** (gyroscope, rad/s, up to 400 Hz)  
• **Acceleration vector** (gravity + linear, m/s², up to 400 Hz)  
• **Linear acceleration** (motion only, m/s²)  
• **Gravity vector output** (m/s²)  
• **Magnetic field vector** (µT)  
• **Integrated Hillcrest SH-2 sensor fusion algorithm** for real-time
orientation and motion tracking  
• **High stability and low drift**, ideal for robotics, drones, and
AR/VR applications  
• **Compact size:** 20 × 27 × 4 mm, **Weight:** 3 g

**3.6.2 Connections**

• **Communication Interface:** I²C (default address 0x4A, alternative
0x4B)  
• **Optional Interfaces:** UART or SPI for high-speed data
transmission  
• **Pins:** SDA, SCL, INT, PS0/PS1 (interface select), RST  
• **Mounting holes:** 20 mm × 12 mm spacing for easy installation

**3.6.3 Power Requirements**

• **Operating Voltage:** 3.3 V typical (5 V tolerant I²C/SPI pins via
onboard regulator)  
• **Current Consumption:** \~3.5 mA (typical during active fusion)  
• **Low-power design**, optimized for mobile and embedded robotic
systems

**3.6.4 Communication Protocol**

• **I²C, SPI, and UART supported** (SPI recommended for high-speed
operation)  
• **Output Data:** Euler angles, quaternions, linear acceleration,
angular velocity, magnetic field, and gravity vectors  
• **Sensor Fusion Firmware:** Hillcrest SH-2, enabling accurate and
stable 9-DOF orientation output

**3.7. Custom PCB Design**

#### **3.7.1. Function and Purpose**

This custom control board was designed as the main interface between the
Raspberry Pi 5 and peripheral components of the robotic system. It
integrates sensor connections, user interaction elements (LEDs, button,
buzzer), and power management components, ensuring organized and safe
communication between modules.

#### **3.7.2. Circuit Elements and Layer Structure**

The circuit is built on a **dual-layer PCB (printed cicuit board)** and
includes the following key components:

- **Two LEDs (red and green):** Used as status and warning indicators.

- **One buzzer:** Provides audible alerts.

- **One push button:** User input control.

- **One fuse:** Overcurrent protection.

- **One gyro sensor (BNO085 IMU):** Orientation and motion sensing.

- **One switch:** Power on/off control.

- **Raspberry Pi 5 GPIO connections:** For system control and
  communication.

- **One ultraconic sensor:** For measure distance.

- **Some capacitor:** For sudden surges.

- **One ferrit bead:** For EMI.

On the PCB, **power lines** are routed with thick copper wires, while
**signal lines** are routed with thinner wires to ensure current
handling capacity and minimize interference.

**Board 1:**

![](media/image35.png){width="3.313043525809274in"
height="1.4232655293088363in"}![](media/image36.jpeg){width="3.1304352580927386in"
height="1.3434787839020121in"}

**Board 2:**

![](media/image37.png){width="3.212276902887139in"
height="1.3652176290463691in"}![](media/image38.jpg){width="3.058459098862642in"
height="1.321738845144357in"}

These boards name is "Raspberry Pi Distribution Board"

#### **3.7.3. Connection Points**

- **Power Input:** Operates with an external power supply of **6--14
  V**.

- **Raspberry Pi 5 GPIO:** Direct connections for LEDs, button, buzzer,
  and the sensor.

- **Sensor Connection:** The BNO085 IMU is connected via the **I²C
  interface (SCL, SDA)** to the Raspberry Pi 5.

- **Connectors:** JST and pin header connectors are used for both power
  and signal interfaces.

### **3.7.4 Electronic Design and PCB Implementation**

#### **3.7.5 Electronic Schematic Description*

The electronic schematic was designed using **EasyEDA**.  
It includes the following functional blocks:

- **Power Section:** Battery input (6--14 V), fuse protection, power
  switch, and monitoring connector.

- **Sensor Section:** BNO085 IMU connected to the Raspberry Pi 5 via the
  I²C interface (SDA, SCL).

- **Control Unit:** Raspberry Pi 5 serves as the central controller,
  managing input and output signals.

- **User Interaction Components:** Two LEDs (status indicators), one
  buzzer (alert), and one push button (input).

Each component is clearly assigned to its corresponding GPIO pins,
ensuring reliable communication and power delivery.

**3.7.6 Hardware Issues and Troubleshooting Process**

At the beginning, we were using the **Hiwonder RRC Lite Controller**. At
first glance, it appeared to be an ideal solution for our needs --- a
single board that could easily interface with the **Raspberry Pi**,
included a **buzzer**, **button**, **encoder motor driver**, and **servo
ports**, and seemed capable of handling all our system requirements.

However, as we started using it, we began experiencing **connection
losses** between the **Raspberry Pi 5** and the controller while the
robot was operating. Initially, we didn't pay much attention to it since
the disconnections were quite rare. But over time, the issue worsened
--- what started as a once-a-week problem eventually became **three to
four disconnections per day** during testing.

We investigated the possible causes. Our first assumption was **EMI
(Electromagnetic Interference)** since the data cable was routed close
to the motor. To address this, we added a **ferrite bead** to the cable,
but it didn't help. Then we suspected a **power issue**, so we separated
the power connection and supplied the **Raspberry Pi** from an external
**power bank** instead. Unfortunately, the problem persisted.

We also added **capacitors** to the power input and replaced the cables,
yet the **RRC Lite Controller** continued to malfunction. Eventually, we
decided to **design our own motor driver board** to ensure stable
operation.

![](media/image39.png){width="3.3301881014873143in"
height="1.7099234470691163in"}![](media/image40.jpeg){width="3.036836176727909in"
height="1.594339457567804in"}

**OUR MOTOR DRIVER**

### 3.7.7 PCB Layer Structure and Design Rules

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

### 3.7.8 Connectors and Component Placement

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

### 3.7.9 Manufacturing and Assembly Notes

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

> **3.7.10 GERBER Files**
>
> If you want to order PCB(Printed Circuit Board), download these
> **GERBER** files and follow the steps in the video.

| Raspberry Pi Distributon Board 1 |     |
|----------------------------------|-----|
| Raspberry Pi Distributon Board 2 |     |
| Motor Driver                     |     |

> **Note:** If you don\'t know how to order a PCB, click the link and
> watch the video.
>
> [video](https://www.youtube.com/watch?v=SGsfiHOE9Fk&t=466s)

## **3.8 Power Supply (3S Li-Po Battery)**

### **3.8.1 Battery Specifications (Voltage, Capacity, C Rating)**

The power source of the project is a **JetFire 3S Li-Po battery**,
specifically designed for high-current demanding applications such as
robotics, drones, and defense systems. Thanks to its high discharge rate
and reliable cell quality, it ensures stable and long-term operation.

**Technical Specifications:**

- Voltage: **11.1 V** (nominal)

- Capacity: **1300 mAh**

- Cells: **3S (3 cells in series)**

- Discharge Rate: **50C continuous / 100C peak (max 10s)**

- Weight: **110 g**

- Dimensions: **74 × 33 × 21 mm**

- Discharge Connector: **XT60 (black)**

- Charging Connector: **JST-XHR (white)**

### **3.8.2 Power Management Circuits (BMS, Charging Circuit)**

Li-Po batteries require safe charging and monitoring systems. A
**Battery Management System (BMS)** or balance charger must be used to:

- Monitor individual cell voltages,

- Prevent overcharging or deep discharge,

- Provide safe current limits during operation.

### **3.8.3 Connections and Connectors**

The battery is connected to the system using an **XT60 connector** for
power delivery and a **JST-XHR balance connector** for charging and
monitoring. Proper wiring and insulation are critical to ensure safe
operation and prevent short circuits.

### **3.8.4 How to Charge the Battery (with iMAX B6AC)**

The **JetFire 3S Li-Po battery** must be charged using a balance charger
such as the **iMAX B6AC** to ensure safe and efficient charging. The
iMAX B6AC allows monitoring of individual cell voltages and prevents
overcharging.

**Charging Procedure:**

1.  **Connection:**

    - Plug the **XT60 connector** into the charger's discharge port.

    - Connect the **JST-XHR balance connector** to the charger's balance
      port.

2.  **Charger Setup:**

    - Select **LiPo BALANCE CHARGE** mode.

    - Set the battery type to **LiPo** and cell count to **3S (11.1V)**.

    - Set the charging current to **1.3A** (equal to the battery's 1C
      rate).

3.  **Charging Process:**

    - Start the charging process and monitor the charger display.

    - Ensure that each cell voltage remains within the safe range
      (**3.7V -- 4.2V**).

4.  **Completion:**

    - Charging is complete when the total voltage reaches approximately
      **12.6V (4.2V per cell)**.

    - Disconnect the battery from the charger and store it safely.

**Safety Notes:**

- Never leave the battery unattended during charging.

- Do not exceed the recommended charging current.

- Always charge on a non-conductive, fireproof surface.

## **4. Power Distribution and Management**

### **4.1 Power Distribution Diagram**

The 11.1V 3S Li-Po battery supplies the entire system. Power is
distributed through a central board, separating high-current lines
(motors, drivers) and low-current lines (controllers, sensors) for
stability.

### **4.2 Voltage Regulators and Converters (5V, 3.3V, etc.)**

- **5V:** Raspberry Pi, LiDAR, camera.

- **3.3V:** IMU and low-power sensors.

- **11.1V Direct:** Motors and motor drivers.  
  Switching regulators are used for efficiency.

### **4.3 Battery Management System (BMS, Charging Control)**

The BMS protects the Li-Po battery from overcharge, overdischarge, short
circuit, and overheating. Charging is done with a balance charger (iMAX
B6AC) to ensure safe and balanced operation.

### **4.4 Protection Mechanisms (Fuse, Overcurrent Protection)**

Fuses, polyfuses, reverse polarity, and voltage monitoring circuits are
added to prevent system failures and protect sensitive electronics.

**Circuit Scheme:**

<table cellspacing="0" cellpadding="0" style="margin:0; padding:0; border-collapse:collapse;">
  <tr>
    <td style="margin:0; padding:0;"><img src="scheme_1.png" height="500"><br></td>
    <td><img src="scheme_2.png" height="500"><br></td>
  </tr>
  <tr>
    <td align="center">Raspberry Pi Distribution Board</td>
    <td align="center">Motor Driver</td>
  </tr>
</table>

**6. Motor and Sensor Control Electronics**

**6.1. Motor Driver Circuits (PWM Control, Speed Feedback)**

The robot uses an Motor Controller, which is a dedicated motor and servo
driver board. This controller takes commands from the Raspberry Pi and
translates them into movements for the encoder motor and servo motor.

PWM Control: The DC motors are controlled using PWM signals. The servo
motor is also controlled via PWM from the Motor Controller or a
microcontroller. The PWM signals for the DC motors are used to control
their speed.

Speed Feedback: The encoder DC motor provides forward and backward
motion, and its encoder sends speed and position feedback to the
Raspberry Pi via the Motor Controller.

**6.2. Sensor Interface Circuits (Filtering, Level Shifting)**

The sensors are connected to the Raspberry Pi and other components
through various interfaces and circuits.

Sensor Connections: The BNO085 IMU sensor is connected to the Raspberry
Pi 5 via the I²C interface.

Voltage Levels: The system uses voltage regulators and converters to
provide the correct voltage levels for different components. The
Raspberry Pi, LiDAR, and camera operate on 5V, while the IMU and other
low-power sensors use 3.3V.

Noise Reduction: The custom PCB design, implemented on a PCB, uses
thicker wires for power lines and thinner wires for signal lines to
ensure current handling and minimize interference. The ground lines were
also carefully routed to reduce noise in sensor readings.
