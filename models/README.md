# 1. Construction of Chassis and General Design

Our robot’s chassis has a two-platform system. The top platform only takes a couple of seconds to remove, making it easy to do any modifications when needed. The two-platform construction gives us additional space to organize and place the various components. The original form of the chassis had sharper corners and it was narrower, but we have since widened it to `15cm` and extended the length to `28cm` this, enhancing stability during the movements because it was easier for us the places the components on top of the robot to make the center of gravity exactly the center of the robot . The other advantage that the wider size gives the robot is that cables are easier to access. The chassis material is PLA which we 3D printed, and we used aluminum links and shafts where additional strength is required. We printed the materials with a Sidewinder X2 at a temperature of `210°C` and a heated bed of `60°C`.  Aditionally, the robot is also `1400 grams`. 

*(chassis photo and the robot photo)*

# 2. Drive System and Gearbox
We used a 12V DC motor and a 3D printed gear box for the drive system. The motor consumes a maximum of 1A (320mA average current) and rotates at a rate of 450 RPM. And to improve the motor’s performance and enable movement of the robot’s mass of 1400g, we used a gear ratio of 2:3 for the gearbox.

The torque enhancement formula:

$$T_{\text{output}} = T_{\text{motor}} \times \frac{3}{2}$$

Power from the motor is transmitted to the central gear of the gearbox, and then both of the wheels turn. The gearbox structure that we used provides us the feature of allowing one wheel to turn even if the other wheel is blocked, similar to a differential. This gives a smooth movement of the robot along the surface when wheels experience a loss of traction. The gears and casing of the gearbox are PLA, while the shafts and connection parts are aluminum. We also used the 693ZZ metal ball bearings is applied in the shafts of the gearbox and the axes of the rear wheels to ensure that the system experiences less friction and vibration. We used the bearing in the steering pivots of the wheels as well to ensure that the Ackermann linkage system rotates smoothly. 

*(gearbox photo here)*


# 3. Ackermann Steering System 
We used the Ackermann steering system for the front wheels of the robot, meaning the inner wheel turns first when the robot turns, thus preventing the robot from slipping and making sure that the robot turns along a natural path. The Ackermann formula is:

$$\frac{\tan(\theta_\text{inner})}{\tan(\theta_\text{outer})} = \frac{L}{L + T}$$

Where L is the wheelbase and T is the track width. A more accurate formula using the turning radius:

$$\frac{1}{\tan(\theta_\text{inner})} = \frac{1}{R} - \frac{D}{2H} \qquad \frac{1}{\tan(\theta_\text{outer})} = \frac{1}{R} + \frac{D}{2H}$$

R is the turning radius, D is the distance between the left and right wheels, and H is the wheel base. 

The distance between the left and right wheels of the robot is `(eyup soyleyecek) cm`, and the wheel base is `(eyup soyleyecek) cm`. Furthermore, there are aluminum linkage arms in the Ackermann system, which are attached to PLA 3D-printed steering knuckles and a PLA servo mount. We attached the linkages with aluminum rods that are `3 mm` thick, and the pivots are all fitted with 693ZZ bearings. The servo controls one of the arms, which is then basically translated into the movement of the inner wheel turning more than the outer wheel through the Ackermann steering system so that the robot turns with the right angle, arc we could say. We chose aliminum because it is light and stiff, and we chose PLA because it provides low-weight pivots.

*(PUT A PICTURE OF ACKERMANN SIMULATION HERE)*


# 4. Servo System
We used a TD8120MG metal gear servo. It gives a torque of `9.24 kg·cm` at a voltage of 4.8V, a torque of `10.63 kg·cm` at a voltage of 6V, and takes a time of around `0.9 seconds` to make a complete turn of `360°`. The gear turns the linkage arm that further moves the two wheels through the Ackermann linkage system.

*(PLACE SERVO SIMULATION HERE)*
*(Also place the oklu photo describing the places of the components here.)*

And here is the exact placement of the components in our robot.

*(Buraya malzemelerin fusiondaki assemblylerini koy)*

# 5. Wheel Movement 
The DC motor has a clockwise turning motion to make the wheels go forward and a counter-clockwise turning motion to make the wheels go backwards. The gear box’s mechanicsm is basically this: if a wheel stops turning, the other wheel continues turning since the torque of the gear wheels turns independently. 

*(Tell Ahmet to put an arrow simulation diagram for turning motion to the wheels. )*
*(PLACE SIMULATION ARROW DIAGRAM HERE)*
*(PLACE THE WHEELS HERE)*

# 6. Center of Gravity and Weight Distribution
We also placed the electronical and mechanical components in a way to keep the center of gravity balanced. At the front, a LIDAR sensor is also placed close to the middle of the robot for better scanning of the surroundings. Above this, we placed the camera in a 75°-angled 3D printed bracket to give a better look at the obstacles an to also cover more ground ahead of the robot and we have an ultrasonic sensor to sense objects behind the robot, which is useful for the robot to navigate itself without colliding with any object in front of it. The battery is at the middle of the body to keep it balanced.

The cables are also tied back to prevent any movement of the robot’s center of mass. The center of gravity formula:

$$X_\text{cm} = \frac{\sum_{i=1}^{n} x_i \times m_i}{\sum_{i=1}^{n} m_i} \qquad Y_\text{cm} = \frac{\sum_{i=1}^{n} y_i \times m_i}{\sum_{i=1}^{n} m_i} \qquad Z_\text{cm} = \frac{\sum_{i=1}^{n} z_i \times m_i}{\sum_{i=1}^{n} m_i}$$


*(CENTER OF GRAVITY DIAGRAM HERE)*
*(CENTER OF MASS DIAGRAM HERE)*
*(PLACE THE MASS OF THE COMPONENTS HERE)*

- Battery = 
- Lidar =
- Chassis = 
- Chassis 2 =
- Camera =
- Ultrasonic  =
- Battery Holder =
*(Calculate later with Eyup)*


# 7. Cable Manage 
All wires are organized and secured using PLA holders. We didn’t mix the power, data, and servo cables to make sure that there are no interference issues. (Add something here about the  type of cables that you used after eyup tells them, you can write them as bullet points.)

*(PLACE CABLE PHOTO HERE)*


# 8. Mechanical Materials and Bearings 

We used aliminum for linkage arms, shafts, and joints because of its strength and lightness. PLA is used for the 3D printed parts like the gear box housing, steering knuckles, servo mount, camera bracket, and the LIDAR mount. The metal bearings are the 693ZZ bearings and they are fitted into the axes of the rear wheels, the gear boxes, and the pivots of the front steering wheels. (Expand this part and add arrow etc.)

*[PLACE BEARING PHOTO HERE]*

# 9. Sensor and Component placement 
We placed The LIDAR sensor  near the center and towards the front using an inverted mount. The camera is placed at the top with a PLA angle bracket of `75°` for forward visibility. The ultrasonic sensor at the back of the robot for the robot the detect the measurements behind it for ease the movement on the map and to make it precise. (Check if you said that correctly). The Raspberry Pi is placed upon the central deck, the motor driver upon the upper deck, and the battery within the center of the lower platform. The servo motor is attached to the left-side linkage. 

*PLACE COMPONENT LAYOUT PHOTO HERE*

# 10. Specifications of 3D Printing 
All of the printed parts were printed with a Sidewinder X2 3D printer. We used PLA because of its durability, printing accuracy, and dimensional stability. The hot end temperature and the hot bed temperature were set to 210°C and 60°C. You can see the time lapse videos of the printing process below and you can also take a look at the specifications of the 3D  printer used in the proccess. 

*Time-lapse Video and Specification of the 3D Printer Photos Here*

# 11. Exploded View 

We showed the assembly of all the parts through an exploded view of a full SolidWorks model. The exploded view has the two layers of the chassis, the Ackermann steering component, the servo mounting component, the gearbox, the wheels, the electrical and sensor components.

<div>
  <img src="../media/exp.gif" alt="Exploded View" />
  <p style="margin-top:0;"><i>Figure 1.1: Exploded View</i></p>
</div>

<hr>

<p align="center">
  <picture>
  <source media="(prefers-color-scheme: dark)" srcset="../media/my_image.png">
  <img height="400" alt="logo" src="../media/my_image_light.png">
  </picture>
</p>