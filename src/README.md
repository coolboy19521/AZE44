# 1. Software Structure and Principles 

In this section of the documentation software solution of our strategy is discussed. Before starting to explain how our code works we would like to present our principles while creating the software:

1. **Keep it simple**: The computing power of our SBC and the accuracy of our sensors are limited. There is no point in designing an extra accurate algorithm, because it most probably would not work in practice most of the time. Also maintaining a complex algorithmic structure is not easy and would require a lot of time, which we could more effectively spend on optimizing existing simpler algorithms or our strategy.

2. **Have a margin of error**: There is nothing such as "*at STP*" in the competition arena. So you can't just solely rely on robot's mechanic design and right perfect code. Software should always be able to tolerate some amount of hardware error. So should hardware, as perfection lies in balance of both.

3. **Test enough**: Sometimes we get new ideas for our strategy or overall robot behaviour. It is not appropriate to use it if it works once. Before moving on and accepting the solution, it should be tested. Many of the ideas look bright at first glance but with proper testing its faults are much more observable.
> [!NOTE]
> For every problem, there is a solution which is simple, fast, and wrong.

4. **Never be afraid of trying new ideas**: Most of our ideas about the robot and strategy has changed since we started working on the actual map. Almost nothing has stayed the same. Because of that, we always think of alternative ways of doing something in parallel to implementing our already existing ideas.

5. **Stay positive**: It is really easy to get pessimistic when a solution does not work, especially when you spend a lot of time on it. In times like these we remind ourselves why we started this journey. To learn something new, experience new stuff and compete fairly.

Now that we got our principles out of the way, we can start with the actual software. Our structure for software flow is such:

<div>
  <img src="../media/CodeComponentDiagram_ManimCE_v0.19.0.png" alt="Diagram" />
  <p style="margin-top:0;"><i>Figure 1.1 Software Flow Diagram</i></p>
</div>

# 2. Implementation details

### 2.1 Motor Driver

From now on we will talk about different sections of the software as described in figure 1.1.

Let's start with the Raspberry Pi Pico - motor driver. We have created our own API for motor and servo control which also supports motor encoders. Before developping our own motor driver we actually tried an already existing one, but it had many issues. The motor driver we used before was consistently losing connection with Raspberry Pi. It was stopping to receive any commands transmitted from the Raspberry Pi and executed the last one. We had many ideas to fix this issue:

1. **Sending large packets over USB**: Our idea for this happening was incorrect use of available transmission space inside the USB adapter. So we modified the API of our previous motor driver. This hypothesis was actually correct to some degree: before making the modification our robot lost connection at approximately `5th minute` of working, but after making the modification we actually quadrupled this number with `20 minutes` lifespan. But robot was still losing connection.

2. **Sending too much motor commands**: This was another hypothesis of ours, and to try it out we lowered the frequency of our Raspberry Pi solution. This actually helped a lot as now the lifespan of the motor driver was up to `40 minutes`. This was some improvement, but motor drivers are designed to work as much as they are powered, not periodically.

3. **Losing bytes when receiving the packets**: We had really good faith in this idea. Because we thought the issue was most probably this. But when we tried adding addition checksums and start flags to the packet for security, nothing changed at all. We did not observe any improvement in lifespan of the motor driver.

After having these issues we decided it was not worth risking a faulty motor driver. When we made this decision there was only a week left before the national qualifications so we were too late to buy a new motor driver. That's why we decided to make our own using Raspberry Pi Pico. Pico receives the commands from Raspberry Pi and sends them to the internat motor driver. When designing the work flow of our API we relied on our previous hypothesis of why a motor drive could fail. The new API we have created has never failed since we have created it and we hope it will stay so. Let's look at some cruicial parts and explain them on code:

```python
while True: # While Pico is alive
    position = encoder.position # Get current position
    if position != last_position: # If current position is not same as last position
        send_command(position) # We send new encoder value
        last_position = position # And update last position

    time.sleep(0.001) # Add a little delay
```

We are only sending when the encoder value changes. Because if we'd always send encoder value usb data storage would be extremely polluted by zeros, and it results in slip.

```python
def read_command():
    global buffer
    if usb_serial.in_waiting > 0:
        buffer += usb_serial.read(usb_serial.in_waiting) # Accumulate buffer globally
    while len(buffer) >= 3: # All packets are siz of 3
        frame = buffer[:3]
        servo_val = frame[0]
        motor_raw = frame[1]
        checksum = frame[2]
        if checksum == ((servo_val + motor_raw) & 0xFF): # If checksums match
            motor_val = motor_raw
            if motor_val >= 128: # Packets are unsigned but to give negative speeds
                motor_val -= 256 # we subtract 256 when speed is over 128.
            buffer = buffer[3:]  # This idea comes from how two's complement works.
            return servo_val, motor_val
        else:
            buffer = buffer[1:] # If checksums mismatch we shift by a byte.
    return None
```

We are securing the commands using checksums. When the checksums don't match we know there has been a byte lost. Despite we have this algorithm we have almost never observed a checksum mismatch.

```python
def send_command(value):
    data = value.to_bytes(4, 'big', signed=True)
    checksum = sum(data) & 0xFF # Value is checksummed
    if usb_serial.out_waiting < 64: # Send data if there is not much queue in usb storage
        usb_serial.write(data + bytes([checksum]))
```

Cruicial part about this function is limiting how much data is sent at any moment. We don't allow a queue of size more than `64 bytes`. This way usb does not get flooded. We actually did not have this logic at some point and we were losing connection between our motor driver.

```python
while True:
    cmd = read_command()
    if cmd is not None: # If there is an incomming command
        servo_angle, motor_speed = cmd
        set_angle(servo_angle)
        move_motor(motor_speed)

    position = encoder.position
    if position != last_position:
        send_command(position)
        last_position = position

    time.sleep(0.001)
```

This is the main loop of the API. As you can see transmitting encoder values and receiving commands happen concurrently. You can view this API further in `src/pico.py`.

There is also another API inside of the Raspberry Pi to communicate with Raspberry Pi Pico. API for Raspberry Pi is almost identical to this code, and you can view it in `src/robot_car.py`.

> [!NOTE]
> Our motor driver API support motor commands at a frequency up to `50 Hz`, and we are using it at `20 Hz`. The limit for our usage is bound by our strategy and the overall logic for the solution.

### 2.2 Maintaining the direction

We would like to talk a little bit about how the robot goes straight. Gyro sensor is not reliable enough, and drifts very often. To fix this we have developed such a strategy: using lidar we get closest point on our left. Geometrically this point is always perpendicular to the robot if the robot is straight. Otherwise the angle between us and the point gives how tilted the robot is. You can inspect the illustration to see what is going on:

<div>
  <img src="../media/RobotWithArcMask.gif" alt="Straighten Strategy" height="600px" />
  <p style="margin-top:0;"><i>Figure 1.5: Straighten Strategy</i></p>
</div>

### 2.3 Open Challange

Now let's continue with Open Challange overall logic. We have a very simple strategy for Open Challange: all we do is to go straight as much as we can, and when we get to close we make a turn. Let's take a look at some cruicial parts:

```python
gyro_angle = robot_car.read_gyro() # read the gyro angle from our robot_car api
scan, rad = [], msg.angle_min
for dis in msg.ranges:
    deg = gyro_angle - math.degrees(rad) # shift every angle by robot yaw
    if -90 <= deg <= 90 and math.isfinite(dis): # if an angle is valid
        scan.append((deg, dis * 100)) # add it to scan list
    rad += msg.angle_increment
```

As you can see from the code, we are shifting lidar angles by robot yaw. The reason for that is, we try to assume that angle `0°` is parallel to the outer wall. This is cruicial because we are making all detections and calculating wall distance according to lidar values. We also only take angles less than `90°` and greater than `-90°` because the back of our lidar is facing itself, and that data is not proprietary.

```python
if not self.f_dir: # if direction is not yet found
    forr_dis = robot_car.find(min, scan, -5, 5) # robot's front distance to outer wall
    if forr_dis is None or forr_dis > 100: # if front distance is too great
        gyro_angle = robot_car.read_gyro()
        gyro_error = -gyro_angle
        robot_car.move(head = gyro_error, speed = 40) # go straight following a straight line
    else: # if some distance is achieved
        left = robot_car.find(max, scan, -90, -10)
        right = robot_car.find(max, scan, 10, 90)
        if left is not None and (right is None or left > right): # if there is more to the left than to the right
            self.dir = -90 # movement direction is towards left
        else:
            self.dir = 90 # otherwise it is to right
        self.f_dir = True # now the direction is found and we can proceed
```

We detect movement direction by detecting if there is a greater gap to the left of the robot than the right of the robot, and vice versa. The reason why we don't do it directly at the start is because robot's front could be placed far away from the outer wall. In this case robot can't really determine which side is more emptier, because the inner walls are not allowing it to.

```python
accel_speed = robot_car.calc_ccel(30, 60, 300 - for_dis, 70) # calculate acceleration according to front distance
if self.count == 12: # go slower if this is the last section
    deaccel_speed = robot_car.calc_ccel(30, 30, for_dis, 300)
else: # other deaccelerate a little bit lighter
    deaccel_speed = robot_car.calc_ccel(30, 60, for_dis, 130)
self.idle_speed = min(accel_speed, deaccel_speed) # we go with minimum value among both
```

At every point we calculate acceleration and deacceleration values. We go as fast as the minimum of them, this creates an effect of speeding up, then going steady and then slowing down. Slowing down when getting closer helps us to make turns more accurate as we have more space to make a turn. When approaching the outer wall fast, the turning algorithm might be triggered later than it should because of the low frequency of our lidar. And speeding slowly up after turns make our robot go more straight considering slow speed bumps make robot slip from the ground less.

```python
if self.count == 12 and forr_dis is not None and forr_dis <= 180: # if it is last section and some distance is achieved
    robot_car.move(0, 0) # soft brake
    robot_car.ser.close() # close the communication with motor driver
    raise Exception # end the opmode
```

This is a pretty basic terminal code which end the opmode when it is the last section of the run and some distance is achieved. You can see the illustration to get a better idea of our open runs:

<div>
  <img src="../media/WROMapScene.gif" alt="Straighten Strategy" />
  <p style="margin-top:0;"><i>Figure 1.6: Open Challange Strategy</i></p>
</div>

### 2.4 Obstacle Challange

Now let's talk about the famous Obstacle Challange! *Everyone is so excited!* Core idea of our obstacle challange is very simple. We break the obstacle challenge into wro subchallanges:

<ol>
  <li><b>UC</b> (Untraced Cubes): In this section, we assume (and we actually do) that we don't know colors of any pillars. Because of this reason we first approach to each pillar and try to identify its color. Only after identifying its color we either surpass it on the right or the left. We also save the color of the pillar for later.</li>
  <li><b>TC</b> (Traced Cubes): In this section we assume (yet we do) know every pillar's color. So only thing left to do is just surpass it from the correct side. We read every pillar's color from the list and behave accordingly. After surpassing every cube we increment the pointer by one and assume that next pillar will have the next color in the list.</li>
</ol>

As you may have noticed we can do TC rounds twice as we only need to see each pillar once to know its color. We find it very interesting that TC rounds are twice as fast than UC rounds. UC rounds take `70 s` in average and TC rounds only take approximately `30 s`. We also speed things up a little bit when doing TC rounds, because we no longer need high precision when detecting the pillars, even if they are detected late we still have enough reaction time to surpass it on the correct side.

Let's look at the code which determins the round to be executed:

```python
kubik = self.find_kubik(scan, r_wall_dis) # this is a method which finds kubik's position

if kubik is not None and self.on_this_side < 2: # if there is at least one pillar yet to be seen
    self.kubik = kubik
    if self.count < 5: # if we are still on first lap
        print('KUBIK_UC prog')
        self.exec = KUBIK_UC # it is a UC round because we have not memorized all colors yet
        self.on_this_side += 1
    else: # otherwise if it is second or third lap
        side, y_dis = self.kubik
        var = self.dir == 90 and (self.last_side is None or side != self.last_side)
        if self.last_dis is None or (y_dis > self.last_dis and not abs(y_dis - self.last_dis) < 2) or var:
            self.kubik_ix = (self.kubik_ix + 1) % len(self.kubiks) # we increase the pointer by one
            if self.seen_start and self.kubik_ix == len(self.kubiks) - 2:
                self.lap_cnt += 1
            elif not self.seen_start and self.kubik_ix == len(self.kubiks) - 1:
                self.lap_cnt += 1
            self.last_side = side
            self.exec = KUBIK_TC
            print('KUBIK_TC prog', self.last_dis)
            self.on_this_side += 1
        self.last_dis = y_dis
```

As you may see we have more checks on pillars when executing TC rounds. That is because, with high speed there is a greater chance of error. The checks we are doing here are sufficient enough to avoid false positives. The main checks we are doing are:
- If the pillar side (left or right, based on distance to the wall which is always either `40 cm` or `60 cm`) does not match with the memorized pillar side;
- If the pillar detected now is closer than the previously detected pillar (we keep track of the pillar distance as we approach it, if it gets less or remains the same it means the robot is detecting the same pillar over again).

```python
def find_kubik(self, data, wall_dis):
    """
    A pillar is detected using two points only: the closest one,
    and the furthest one. If the distance between them is greater than
    some threshhold it means there is some object in sight which is
    further away than the front outer wall. This can either be an inner
    wall or the pillar. To avoid the inner wall case, we limit the
    relative x distance from paralel outer wall. This way the closest
    point can only be a pillar.
    """

    if wall_dis is None: # if lidar values are unsificient
        return None # there can't be any pillars detected

    # limits for relative distance from the outer wall
    left_lim = self.off_x - wall_dis
    right_lim = 100 - self.off_x - wall_dis

    if self.dir == -90: # they switch up if going counterclockwise
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

    # if any unsificient data, or if closest point is too close to the furthest point
    if closest is None or furthest is None or \
            furthest[0] - closest[0] <= 60 or \
            var or closest[0] <= 15:
        return None

    # calculate pillar's relative distance to the parallel outer wall
    if self.dir == 90:
        from_wall_dis = wall_dis + closest[1]
    elif self.dir == -90:
        from_wall_dis = wall_dis - closest[1]

    if (self.dir == 90 and from_wall_dis <= 50) or (self.dir == -90 and from_wall_dis <= 43):
        return 40, closest[0] # it is on the left side, with y distance closest[0]
    else:
        return 60, closest[0] # it is the right side, with y distance closest[0]
```

We have designed this algorithm after many unsuccessful iterations. But this one was pretty efficient. It has a time complexity of `O(N)` with a pretty low constant coefficient. We would like to share a previous idea we had:

Cluster the points according their distance with each other. If there is a group with a diameter of at most `~8 cm` (`√5²+5²`) it is a pillar. This method worked poorly, because clustering the points according their distance took `O(N²)` complexity as we looked at each pair of them. To optimize this method we used an algorithm so called DSU (Disjoing Set Union). Using this technique it is possible to cluster `N` points with almost linear time complexity (with an inverse ackermann coefficient). Even after optimizing this method, we discovered that our lidar does not give precise enough data to use this method.

Thankfully, we switched to our current approach and did not think much to fix the previous one.

# 3. Development Environment

Let's get into setting up the development environment. We have a very generic development environment for the host machine. The only programs you need to develop is `ssh` and any text editor of your choice. All of the modern machines nowadays come with these by default. To develop on the host machine we used `Command Prompt` in our Windows machine and `Terminal` in our Ubuntu machine.

Environment for the Raspberry Pi is a little bit more complex but to make it simpler we developed a solution as such:

[![⬇ ISO File](https://img.shields.io/badge/⬇%20ISO%20File-aze44__live__image.iso-df3e3e)](https://your-download-link)

This is our Raspberry Pi image. If you flash this file into your Raspberry, you will get the exactly same environment as us. To flash this image you can use ``Rufus`` for Windows, ``UNetbootin`` for Linux or ``WonderISO`` for MacOS.

<div>
  <img src="../media/rufus.webp" alt="Rufus" />
  <p style="margin-top:0;"><i>Figure 1.2: Rufus flashing tool</i></p>
</div>

But if you want to customize installation you should flush the Raspberry Pi from the official Raspberry Pi Imager. If not, just skip to the `Code and Strategy` section.

> [!IMPORTANT]
> To be able to execute the commands and instructions below you need a microSD card of at least `16GB` storage. The operating system for Raspberry Pi will be installed in this microSD card. If you have any important files in the microSD card, please back them up because after the installation they will be erased from the card.

We are using `Ubuntu Server 24.04.3 LTS (64-bit)` for our Raspberry Pi. You can install this OS to your microSD as you want, but the recommended way is to use the official [Raspberry PI Imager tool](https://www.raspberrypi.com/software/).

<div>
  <img src="../media/Imager.PNG" alt="Raspberry Pi Imager" />
  <p style="margin-top:0;"><i>Figure 1.3: Raspberry Pi Imager</i></p>
</div>

After selection your Raspberry Pi model press "Choose OS". Afterwards select the `Other general-purpose OS` > `Ubuntu` > `Ubuntu Server 24.04.3 LTS (64-bit)` operating system.

<div>
  <img src="../media/OS.PNG" alt="Ubuntu Server 24.04.3 LTS (64-bit)" />
  <p style="margin-top:0;"><i>Figure 1.4: Ubuntu Server 24.04.3 LTS (64-bit)</i></p>
</div>

> [!NOTE]
> `Ubuntu Server 24.04.3 LTS (64-bit)` is the latest release for Ubuntu Server. Despite this, it support many Raspberry Pi versions. Without the loss of generality, you do not need to install this specific version of Ubuntu to be able to execute the instructions and commands, but it is recommended as it is what we used.

> [!NOTE]
> It is not possible to use `Raspberry Pi OS (Raspbian)` for this project. The reason for that is `ROS2 JAZZY`, a dependency for this project, not being supported by `Raspbian`. There might be some workarounds to get it to work, but it would be a waste of time as `Ubuntu Server 24.04.3 LTS (64-bit)` is already as minimal as official `Raspbian`.

Now to configure the operating system press `Ctrl` `Shift` `X` or `⌘` `Shift` `X`. In the panel that opened, under the `General` tab set your internet settings and your credentials for Raspberry Pi user. After that switch to `Services` tab and enable `SSH` and `Password Authentication`. After doing all the configurations press `Next` and the installation will start.

After the installation is complete plug the microSD card into Raspberry Pi. After it boots, it is recommended to execute following commands:

```
$ sudo apt update
$ sudo apt upgrade
```

After installing the operating system you can install the ROS2 environment.

Start with installing `Git`. This package will be used for installing other packages from the internet:

```bash
$ sudo apt-get install git
```

To be able to use Raspberry Pi camera module, the packages `libcamera` and `rpicam-apps` should be installed. Start with installing the dependencies:

```bash
$ sudo apt install -y libboost-dev
$ sudo apt install -y libgnutls28-dev openssl libtiff5-dev pybind11-dev
$ sudo apt install -y meson cmake
$ sudo apt install -y python3-yaml python3-ply
$ sudo apt install -y libglib2.0-dev libgstreamer-plugins-base1.0-dev
```

And now to install the package:

```bash
$ git clone https://github.com/raspberrypi/libcamera.git
```

Go to the root directory:

```bash
$ cd libcamera
```

To configure the package build:

```bash
$ meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled
```

And finally to install `libcamera` package:

```bash
$ sudo ninja -C build install
```

After installing `libcamera` package we can install `rpicam-apps`. Start with dependencies:

```bash
$ sudo apt install -y cmake libboost-program-options-dev libdrm-dev libexif-dev
$ sudo apt install -y meson ninja-build
```

Go to the root directory and download the package:

```bash
$ git clone https://github.com/raspberrypi/rpicam-apps.git
```

Move to the package directory:

```bash
$ cd rpicam-apps
```

Before doing the configuration install `libav` package to be able to record videos using the camera:

```bash
$ sudo apt install -y libavcodec-dev libavformat-dev libavutil-dev libswscale-dev libavdevice-dev libavfilter-dev
```

Now you can configure the build:

```bash
$ meson setup build -Denable_libav=enabled -Denable_drm=enabled -Denable_egl=disabled -Denable_qt=disabled -Denable_opencv=disabled -Denable_tflite=disabled -Denable_hailo=disabled
```

And finally install `rpicam-apps`:

```bash
$ meson compile -C build
$ sudo meson install -C build
```

You should end the installation with updating the `ldconfig cache`. This is handy for using the installed packages in other directories:

```bash
$ sudo ldconfig
```

Now that all the required packages are installed we can install `ROS2 Jazzy` integration.

[`ROS2 Jazzy`](https://docs.ros.org/en/jazzy/index.html) is a collection of robot integration packages. With the following instructions you should be able to get `ROS2 Jazzy` on your system.

Configure the locale for Raspberry Pi:

```bash
$ locale 
$ sudo apt update && sudo apt install locales
$ sudo locale-gen en_US en_US.UTF-8
$ sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
$ export LANG=en_US.UTF-8
$ locale 
```

Add required repositories to the `apt` package manager to be able to install required apps:

```bash
$ sudo apt install software-properties-common
$ sudo add-apt-repository universe
```

Now add `ros2-apt-source` package:

```bash
$ sudo apt update && sudo apt install curl -y
$ export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
$ curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
$ sudo dpkg -i /tmp/ros2-apt-source.deb
```

To enable the programming environment:

```bash
$ sudo apt update && sudo apt install -y \
  python3-flake8-blind-except \
  python3-flake8-class-newline \
  python3-flake8-deprecated \
  python3-mypy \
  python3-pip \
  python3-pytest \
  python3-pytest-cov \
  python3-pytest-mock \
  python3-pytest-repeat \
  python3-pytest-rerunfailures \
  python3-pytest-runner \
  python3-pytest-timeout \
  ros-dev-tools
```

Now create a `ROS2 Jazzy` workspace and install required `ROS2 Jazzy` packages:

```bash
$ mkdir -p ~/ros2_jazzy/src
$ cd ~/ros2_jazzy
$ vcs import --input https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos src
```

To be confident that all the packages are up-to-date:

```bash
$ sudo apt upgrade
```

Finally install everything:

```bash
$ sudo rosdep init
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
```

After installing the `ROS2 Jazzy` system we can install required packages for the project.

Installing `ROS2 Jazzy` system is not enough to use it. You need to source `/opt/ros/jazzy/setup.bash` everytime you want to use it. If you don't want to explicitely do it everytime execute this command, which add the command to startup:

```bash
$ echo "source /opt/ros/jazzy/setup.bash">>~/.bashrc
```

After executing the command restart the console.

`camera_ros` is a package to integrate camera with `ROS2 Jazzy`. To install `camera_ros` execute the following commands:

```bash
$ cd ~/ros2_jazzy/src
$ sudo apt -y install python3-colcon-meson
$ git clone https://github.com/christianrauch/camera_ros.git
$ source /opt/ros/$ROS_DISTRO/setup.bash
$ cd ~/ros2_jazzy/
$ rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=libcamera
$ colcon build --event-handlers=console_direct+
```

`rplidar_ros` is a package to use RPLidar system. To install it:

```bash
$ cd ~/ros2_jazzy/src
$ git clone https://github.com/Slamtec/rplidar_ros.git
$ rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=libcamera
$ colcon build --symlink-install
```

To use the packages after installing them you should source `~/ros2_jazzy/install/setup.bash` everytime. If you don't want to add it to the `~/.bashrc`:

```bash
$ echo "source ~/ros2_jazzy/install/setup.bash">>~/.bashrc
```

<hr>

<p align="center">
  <picture>
  <source media="(prefers-color-scheme: dark)" srcset="../media/my_image.png">
  <img height="400" alt="logo" src="../media/my_image_light.png">
  </picture>
</p>