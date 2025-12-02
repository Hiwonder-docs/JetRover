# 16 ROS2-Motion Control Course

## 16.1 Kinematics Analysis

The JetRover series chassis is primarily categorized into three types: Mecanum wheel chassis, Ackermann chassis, and tracked vehicle chassis. Each chassis type features unique kinematic modes, and this manual will provide a systematic introduction to each of them.

### 16.1.1 Overview:

The Mecanum wheel vehicle, Ackermann chassis-equipped vehicle, and tracked vehicle are three distinct types of vehicle chassis designs, each displaying significant differences in both structure and functionality.

* **Wheel Type**

**Mecanum Wheel Vehicle**: Mecanum wheel vehicles employ multiple small freely rotating wheels to bear the vehicle's weight. Typically situated at the chassis's bottom, these wheels can rotate independently, enhancing the vehicle's ability to turn and maneuver with greater ease.

**Ackermann Chassis Vehicle**: Ackermann chassis vehicles also utilize wheels, often featuring front-wheel drive, while the rear wheels bear the weight. The front wheels are usually steerable, facilitating smoother turning.

**Tracked Vehicle**: Tracked vehicles replace traditional wheels with tracks composed of a series of connected chain links. This design enhances traction and suspension across various terrain conditions, providing a distinct advantage on uneven surfaces.

* **Application**

1. **Mecanum Wheel Vehicle**: Designed for urban environments and road use, Mecanum wheel vehicles excel at navigating sharp turns with high maneuverability.

2. **Ackermann Chassis Vehicle**: Ackermann chassis vehicles are well-suited for general road driving, including cars, trucks, and motorcycles, thanks to their wheels optimized for these applications.

3. **Tracked Vehicle**: Primarily employed for tasks involving challenging terrain conditions, tracked vehicles, such as military vehicles, construction equipment, and agricultural machinery, offer superior performance in demanding environments.

### 16.1.2 Mecanum Wheel:

* **Hardware Structure**

<img class="common_img" src="../_static/media/4/section_12_1. Kinematics Analysis\media\image4.png" style="width:500px"  />

The Mecanum wheel comprises rollers and an axle. The axle functions as the main support structure for the entire wheel, with rollers attached to it. The axle axis is positioned at a 45-degree angle to the roller axis. Typically, Mecanum wheels operate in groups of four, with two left wheels and two right wheels. Wheels A and B are symmetrical.

There are various combinations of four Mecanum wheels, such as AAAA, BBBB, AABB, ABAB, BABA. However, not all combinations allow the robot car to move in all directions, including forward, backward, and sideways. The Mecanum-wheel chassis combination is ABAB, enabling omnidirectional movement.

<img class="common_img" src="../_static/media/4/section_12_1. Kinematics Analysis\media\image5.png" style="width:500px"  />

* **Physical Characteristics**

The vehicle achieves omnidirectional motion by summing up the propelling forces of the ground-engaging rollers. This summation can occur in any direction through adjustments in wheel rotation direction and torque magnitude for the four wheels.

Because of the rollers' specific orientation at a certain angle to the wheel circumference, Mecanum wheels have the ability to slip sideways. The generatrix of these small rollers is unique. As the Mecanum wheel rotates around its fixed axle, each small roller's envelope forms a cylindrical surface, allowing the wheel to continuously roll forward.

* **Motion Principle and Formula**

When conducting kinematic analysis, we can consider the kinematic model of Mecanum wheels, which includes the following parameters:

<img class="common_img" src="../_static/media/4/section_12_1. Kinematics Analysis\media\image6.png" style="width:500px"  />

1.  <img src="../_static/media/4/section_12_1. Kinematics Analysis\media\image9.png" style="width:45px"  />: Velocity of the Mecanum wheel in the X-axis (typically front and rear direction).

2.  <img src="../_static/media/4/section_12_1. Kinematics Analysis\media\image15.png" style="width:45px"  />: Velocity of the Mecanum wheel in the Y-axis (typically left and right direction).

3.  <img src="../_static/media/4/section_12_1. Kinematics Analysis\media\image60.png" style="width:45px"  />：Angular velocity of the Mecanum wheel chassis (rotation speed of the chassis around its own center).

4.  Real-time velocities of the four wheels of the Mecanum wheel.

5.  The motion of the right front wheel in the plane can be decomposed into:

6.  VBx: Velocity of the Mecanum wheel in the X-axis (typically front and rear direction).

7.  VBy: Velocity of the Mecanum wheel in the Y-axis (typically left and right direction).

8.  L: Distance between the centers of the left and right wheels.

9.  H: Distance between the centers of the front and rear wheels.

10.  <img src="../_static/media/4/section_12_1. Kinematics Analysis\media\image16.png" style="width:45px"  />: Angle formed by the chassis body center and the center of the right front wheel, typically 45°.

11.  With these parameters, we can perform kinematic analysis of the Mecanum wheel chassis. The following are key mathematical formulas:

**Kinematics Formula:**

To simplify the mathematical model for kinematics, we make the following two idealized assumptions:

（1）Omni-directional wheels do not slip on the ground, and there is sufficient friction with the ground.

（2）The 4 wheels are distributed at the corners of a rectangle or square, with the wheels parallel to each other.

Here, we decompose the rigid body motion of the car into three components linearly. By calculating the velocities of the four wheels when the output Mecanum wheel chassis translates along the X+ and Y+ directions and rotates along the Z+ direction, we can combine these three simple motions using formulas to determine the required speeds of the four wheels.

In the equations, A, B, C, and D represent the rotational speeds of the four wheels, i.e., the motor speeds. VX is the translation speed of the car along the X-axis, VY is the translation speed along the Y-axis, and ω is the rotational speed along the Z-axis. L/2 is half of the car's wheelbase, and H/2 is half of the car's axle distance.

1\. The velocity components of each wheel during the robot's translation along the X-axis can be calculated using the following formulas:

<img class="common_img" src="../_static/media/4/section_12_1. Kinematics Analysis\media\image7.png" style="width:500px"  />

Where,

<img src="../_static/media/4/section_12_1. Kinematics Analysis\media\image8.png" style="width:100px"  />：Real-time velocities of the four Mecanum wheels，

<img src="../_static/media/4/section_12_1. Kinematics Analysis\media\image9.png" style="width:45px"  />：Velocity of the Mecanum wheel in the X-axis direction

2\. When the robot translates along the Y-axis, the speed component of each wheel can be calculated using the following formula:

<img class="common_img" src="../_static/media/4/section_12_1. Kinematics Analysis\media\image10.png" style="width:500px"  />

Where, <img src="../_static/media/4/section_12_1. Kinematics Analysis\media\image15.png" style="width:45px"  /> is the velocity of the robot in the Y-axis direction.

3\. When the robot rotates along the Z-axis, the speed component of each wheel can be calculated using the following formulas:

<img class="common_img" src="../_static/media/4/section_12_1. Kinematics Analysis\media\image11.png" style="width:500px"  />

<img class="common_img" src="../_static/media/4/section_12_1. Kinematics Analysis\media\image12.png" style="width:500px"  />

Where，<img src="../_static/media/4/section_12_1. Kinematics Analysis\media\image13.png" style="width:150px"  />:The angular velocity of the Mecanum wheel chassis (i.e., the speed at which the chassis rotates around its own center)

4\. Combining the velocities in the X, Y, and Z directions allows for the computation of the rotation speeds of the four wheels based on the motion state of the car.

<img class="common_img" src="../_static/media/4/section_12_1. Kinematics Analysis\media\image14.png" style="width:500px"  />

* **Program Outcome**

The program file is saved in: **ros2_ws\src\driver\controller\controller\mecanum.py**

<img class="common_img" src="../_static/media/4/section_12_1. Kinematics Analysis\media\image32.png" style="width:500px"  />

**1. MecanumChassis Class**

```py
class MecanumChassis:
    # wheelbase = 0.216   # 前后轴距(distance between front and real axles)
    # track_width = 0.195 # 左右轴距(distance between left and right axles)
    # wheel_diameter = 0.097  # 轮子直径(wheel diameter)
    def __init__(self, wheelbase=0.216, track_width=0.195, wheel_diameter=0.097):
        self.wheelbase = wheelbase
        self.track_width = track_width
        self.wheel_diameter = wheel_diameter
```

Mecanum wheel kinematics class, used to calculate wheel speeds and implement Mecanum wheel kinematics.

Init：

```py
    def __init__(self, wheelbase=0.216, track_width=0.195, wheel_diameter=0.097):
        self.wheelbase = wheelbase
        self.track_width = track_width
        self.wheel_diameter = wheel_diameter
```

Initialize the wheel dimensions for convenient subsequent calculations.

speed_convert:

```py
    def speed_covert(self, speed):
        """
        covert speed m/s to rps/s
        :param speed:
        :return:
        """
        # distance / circumference = rotations per second
        return speed / (math.pi * self.wheel_diameter)
```

Convert m/s to rps based on the wheel's parameters.

set_velocity:

```py
    def set_velocity(self, linear_x, linear_y, angular_z):
        """
        Use polar coordinates to control moving
                    x
        v1 motor1|  ↑  |motor3 v3
          +  y - |     |
        v2 motor2|     |motor4 v4
        :param speed: m/s
        :param direction: Moving direction 0~2pi, 1/2pi<--- ↑ ---> 3/2pi
        :param angular_rate:  The speed at which the chassis rotates rad/sec
        :param fake:
        :return:
        """
        # vx = speed * math.sin(direction)
        # vy = speed * math.cos(direction)
        # vp = angular_rate * (self.wheelbase + self.track_width) / 2
        # v1 = vx - vy - vp
        # v2 = vx + vy - vp
        # v3 = vx + vy + vp
        # v4 = vx - vy + vp
        # v_s = [self.speed_covert(v) for v in [v1, v2, -v3, -v4]]
        motor1 = (linear_x - linear_y - angular_z * (self.wheelbase + self.track_width) / 2)
        motor2 = (linear_x + linear_y - angular_z * (self.wheelbase + self.track_width) / 2)
        motor3 = (linear_x + linear_y + angular_z * (self.wheelbase + self.track_width) / 2)
        motor4 = (linear_x - linear_y + angular_z * (self.wheelbase + self.track_width) / 2)
        v_s = [self.speed_covert(v) for v in [motor1, motor2, -motor3, -motor4]]
        data = []
        for i in range(len(v_s)):
            msg = MotorState()
            msg.id = i + 1
            msg.rps = float(v_s[i])
            data.append(msg)
        
        msg = MotorsState()
        msg.data = data
        return msg
```

Based on the input speed parameters, decompose them, calculate the speeds using speed_convert, and then publish the calculated radian speeds to the motors.

### 16.1.3 Ackermann Chassis

* **Hardware Component**

The transmission mechanism of the Ackermann chassis front wheels includes a servo, linkage, and wheels. The servo is linked to the linkage, and the linkage is connected to the wheels. The servo's rotation governs the extent of the linkage's rotation, thereby affecting the steering of the front wheels.

During a turn, the front two wheels are in a parallel state, with both wheels having the same angles of rotation. The control of the rear wheels is managed by the motor and wheels, where the motor's rotation determines the robot's forward, backward, and speed movements.

<img class="common_img" src="../_static/media/4/section_12_1. Kinematics Analysis\media\image37.png" style="width:500px" />

<img class="common_img" src="../_static/media/4/section_12_1. Kinematics Analysis\media\image38.png" style="width:500px" />

* **Physical Characteristics**

The design objective of the Ackermann chassis is to provide excellent steering performance and stability. It employs a principle called "**Ackermann geometry**" to achieve this. Ackermann geometry refers to the difference in steering angles between the front and rear wheels. By allowing the inner front wheel to have a greater steering angle, the Ackermann chassis makes it easier to control the vehicle during turns and reduces the risk of sliding during steering.

Additionally, the Ackermann chassis features a well-designed suspension system. The suspension system is a crucial component connecting the wheels and the vehicle body, significantly influencing the physical characteristics of the chassis. Ackermann chassis typically adopts an independent suspension system, enabling the independent control of each wheel's movement. This design enhances suspension performance, improving vehicle stability and ride comfort.

Furthermore, the Ackermann chassis considers the position of the vehicle's center of gravity. The center of gravity's location has a significant impact on the vehicle's stability and handling performance. Generally, the Ackermann chassis places the center of gravity lower to reduce the risk of vehicle tilt and sliding.

Lastly, the physical characteristics of the Ackermann chassis include the braking system and power transmission system. The braking system influences the vehicle's braking performance and stability by controlling the braking force on the wheels. The power transmission system is responsible for transferring the engine's power to the wheels, affecting the vehicle's acceleration and driving performance.

* **Kinematics Principle and Formula**

When conducting kinematic analysis of the Ackermann chassis, we can use the following mathematical formulas and parameters to describe its motion characteristics:

To achieve pure rolling motion for the Ackermann car (meaning no side slip during turns), it is necessary to ensure that the normals of the four wheels' motion directions (lines perpendicular to the direction of tire rolling) intersect at a single point, which is the center of rotation.

To simplify the model, let's assume that the front wheels have only one wheel (the theoretical concept remains consistent) located in the middle position of the front axle, as depicted by the dashed line in the diagram:

<img class="common_img" src="../_static/media/4/section_12_1. Kinematics Analysis\media\image39.png" style="width:500px" />

1.  **Front Wheel Steering Angle (θ)**: The rotation angle of the front wheels, indicating the angle by which the front wheels deviate from the vehicle's forward direction. It is typically measured in radians (rad).

2.  **Vehicle Linear Velocity (V):** The overall linear speed of the vehicle, representing its translational velocity. It is usually measured in meters per second (m/s). The left rear wheel speed is denoted as (VL), and the right rear wheel speed is denoted as (VR).

3.  **Vehicle Track Width (D):** The distance between the wheels on the left and right sides of the vehicle, measured in meters (m).

4.  **Wheelbase of the Vehicle (H)**: The distance between the front and rear wheels of the vehicle, measured in meters (m)

5.  **Vehicle Turning Radius (R):** The radius of the circle described by the vehicle during a turn, measured in meters (m). The turning radius for the left wheel is (RL), and for the right wheel is (RR).

Process for Calculating Robot Speed and Angle:

6.  Consistency of angular velocity:

<img class="common_img" src="../_static/media/4/section_12_1. Kinematics Analysis\media\image26.png" style="width:200px"  />

In this context:

ω represents the angular velocity of the vehicle.

R denotes the turning radius of the vehicle.

V is the linear velocity of the vehicle.

VL is the linear velocity of the left rear wheel.

VR is the linear velocity of the right rear wheel.

RL is the turning radius of the left wheel.

RR is the turning radius of the right wheel.

7.  The relationship between the front wheel steering angle and the turning radius of the vehicle:

<img class="common_img" src="../_static/media/4/section_12_1. Kinematics Analysis\media\image27.png" style="width:400px"  />

H represents the distance between the front and rear wheels of the vehicle.

R signifies the turning radius of the vehicle.

D denotes the distance between the wheels on the left and right sides of the vehicle.

θ indicates the steering angle of the front wheels.

8.  The speeds of the left and right wheels of the robot can be determined as:

<img class="common_img" src="../_static/media/4/section_12_1. Kinematics Analysis\media\image28.png" style="width:300px"  />

9.  The speeds of the left and right wheels of the robot can be determined as:

<img class="common_img" src="../_static/media/4/section_12_1. Kinematics Analysis\media\image29.png" style="width:300px"  />

By knowing the wheelbase, track width, robot speed, and the steering angle of the servo, it is possible to calculate the speeds of the two rear wheels of the robot.

* **Program Outcome**

The program files are located in: **ros2_ws\src\driver\controller\controller\ackermann.py**

**1. AckermannChassis Class**

```py
class AckermannChassis:
    # wheelbase = 0.213  # 前后轴距(distance between front and real axles)
    # track_width = 0.222  # 左右轴距(distance between left and right axles)
    # wheel_diameter = 0.101  # 轮子直径(wheel diameter)

    def __init__(self, wheelbase=0.213, track_width=0.222, wheel_diameter=0.101):
        self.wheelbase = wheelbase
        self.track_width = track_width
        self.wheel_diameter = wheel_diameter
```

Ackermann wheel kinematics module, used to calculate wheel speeds and implement Ackermann wheel kinematics.

Init：

```py
    def __init__(self, wheelbase=0.213, track_width=0.222, wheel_diameter=0.101):
        self.wheelbase = wheelbase
        self.track_width = track_width
        self.wheel_diameter = wheel_diameter
```

Initialize the wheel dimensions for easier subsequent calculations

speed_covert：

```
    def speed_covert(self, speed):
        """
        covert speed m/s to rps/s
        :param speed:
        :return:
        """
        return speed / (math.pi * self.wheel_diameter)
```

Convert m/s to rps based on the wheel parameters.

set_velocity:

```
    def set_velocity(self, linear_speed, angular_speed, reset_servo=True):
        servo_angle = 500
        data = []
        if abs(linear_speed) >= 1e-8:
            if abs(angular_speed) >= 1e-8:
                theta = math.atan(self.wheelbase*angular_speed/linear_speed)
                steering_angle = theta
                # print(math.degrees(steering_angle))
                if abs(steering_angle) > math.radians(37):
                    steering_angle = math.radians(37)
                    # for i in range(4):
                        # msg = MotorState()
                        # msg.id = i + 1
                        # msg.rps = 0.0
                        # data.append(msg)
                    # msg = MotorsState()
                    # msg.data = data
                    # return None, msg
                servo_angle = 500 + 1000*math.degrees(steering_angle)/240

            vr = linear_speed + angular_speed*self.track_width/2
            vl = linear_speed - angular_speed*self.track_width/2
            v_s = [self.speed_covert(v) for v in [0, vl, 0, -vr]]
            for i in range(len(v_s)):
                msg = MotorState()
                msg.id = i + 1
                msg.rps = float(v_s[i])
                data.append(msg) 
            msg = MotorsState()
            msg.data = data
            return servo_angle, msg
```

Based on the input speed parameters, decompose them, calculate the speeds using speed_convert, and then publish the calculated radian speeds to the motors. Calculate the required steering angle from the linear and angular velocities, convert it, and send it to the servos.

### 16.1.4 Tank Chassis

* **Hardware Structure**

The track is a flexible chain loop composed of a drive wheel, surrounded by idler wheels, load wheels, guide wheels, and carrier wheels. It consists of track plates and track pins.

<img class="common_img" src="../_static/media/4/section_12_1. Kinematics Analysis\media\image49.png" style="width:500px" />

<img class="common_img" src="../_static/media/4/section_12_1. Kinematics Analysis\media\image50.png" style="width:500px"  />

* **Physical Characteristic**

<img class="common_img" src="../_static/media/4/section_12_1. Kinematics Analysis\media\image51.png" style="width:500px"  />

**1. Drive Wheel**

The track propulsion device consists of components that transfer power from the transmission system to the track wheels. Its primary function is to convey power to the track and is usually installed at the rear of the vehicle.

**2. Load Wheel**

The load wheel is mainly used to bear the weight of the vehicle and align the track. Its primary function is to ensure even distribution of pressure from the track to the ground, thereby improving the smoothness of travel.

**3. Guide Wheel**

The position of the guide wheel is determined by the location of the drive wheel and is typically installed at the front of the vehicle. Its main purpose is to guide the track in the correct rotation to prevent deviation and derailment.

**4. Carrier Wheel**

The carrier wheel functions to support the track, preventing excessive sagging and lateral slipping. This helps minimize bouncing of the track during motion.

**5. Chassis**

The tracked chassis utilizes tracks for its movement. When the tracked chassis is in motion, it moves as if on laid tracks. The power generated by the tracked movement is continuously transmitted by the drive wheel, resulting in a continuous rolling motion of the tracks. During the advancement process, on one hand, the tracks unrolled by the guide wheel are laid on the ground and pressed under the load wheel rolling forward. On the other hand, the tracks rolled by the rear load wheel are wound up by the drive wheel.

* **Kinematic Principles and Formulas**

When performing kinematic analysis, we can explore the kinematic model of a tracked chassis, incorporating the following parameters:

<img class="common_img" src="../_static/media/4/section_12_1. Kinematics Analysis\media\image40.png" style="width:500px"  />

1. **B：**Track Width, the distance between the two drive wheels on the track, usually measured in meters (m)

2. **R：**Turning Radius (R) generated when the robot simultaneously moves forward and rotates, usually measured in meters (m)

3. 𝑉𝑥：Target forward/backward velocity of the robot at point O, positive for forward movement, typically measured in meters per second (m/s)

4. 𝑉𝑦：Target left/right velocity of the robot at point O, positive for leftward movement, measured in m/s (ignored for two-wheel configurations)

5. 𝑉𝑤：Target rotational velocity of the robot around point O, positive for counterclockwise rotation, measured in rad/s

6. <img src="../_static/media/4/section_12_1. Kinematics Analysis\media\image41.png" style="width:45px"  />**：**Left wheel velocity of the robot, positive in the forward direction, typically measured in meters per second (m/s)

7. <img src="../_static/media/4/section_12_1. Kinematics Analysis\media\image42.png" style="width:45px"  />**：**Right wheel velocity of the robot, positive in the forward direction, typically measured in meters per second (m/s)

8. <img src="../_static/media/4/section_12_1. Kinematics Analysis\media\image43.png" style="width:45px"  />：Path covered by the left wheel in a certain time t

   <img src="../_static/media/4/section_12_1. Kinematics Analysis\media\image44.png" style="width:45px"  />：Path covered by the midpoint O in a certain time t

   <img src="../_static/media/4/section_12_1. Kinematics Analysis\media\image45.png" style="width:45px"  />：Path covered by the right wheel in a certain time t

9. θ：Angle of rotation of the robot in a certain time θ measured in radians (rad)

**Formula Calculation:**

1.  Next, we will explore their relationships and derive the kinematic forward and inverse formulas for the tracked chassis (two-wheel differential drive type). By integrating velocity with respect to time, we can obtain the displacement:

<img class="common_img" src="../_static/media/4/section_12_1. Kinematics Analysis\media\image54.png" style="width:500px"  />

2.  Dividing the arc length by the radius equals the angle in radians

<img class="common_img" src="../_static/media/4/section_12_1. Kinematics Analysis\media\image55.png" style="width:500px"  />

3.  Dividing both sides of the formula by 't,' i.e., integrating with respect to time, yields.

<img class="common_img" src="../_static/media/4/section_12_1. Kinematics Analysis\media\image56.png" style="width:300px"  />

4.  To solve the inverse kinematics equation based on the above formula, the objective is to determine the drive wheel velocities <img src="../_static/media/4/section_12_1. Kinematics Analysis\media\image41.png" style="width:45px"  /> and <img src="../_static/media/4/section_12_1. Kinematics Analysis\media\image59.png" style="width:45px"  />, given the robot's target velocity <img src="../_static/media/4/section_12_1. Kinematics Analysis\media\image9.png" style="width:45px"  /> and <img src="../_static/media/4/section_12_1. Kinematics Analysis\media\image60.png" style="width:45px"  />.

<img class="common_img" src="../_static/media/4/section_12_1. Kinematics Analysis\media\image57.png" style="width:300px"  />

5.  By using the inverse kinematics equation, we can solve the forward kinematics equation. If the current velocities of the drive wheels <img src="../_static/media/4/section_12_1. Kinematics Analysis\media\image41.png" style="width:45px"  /> and <img src="../_static/media/4/section_12_1. Kinematics Analysis\media\image59.png" style="width:45px"  /> are known, we can calculate the real-time velocity of the robot.

<img class="common_img" src="../_static/media/4/section_12_1. Kinematics Analysis\media\image58.png" style="width:400px"  />

The program file is located in:

**ros2_ws\src\driver\controller\controller\odom_publisher_node.py**

The tracked chassis also involves differential motion, so the kinematics of the Mecanum wheel can be directly applied.

Speed settings:

<img class="common_img" src="../_static/media/4/section_12_1. Kinematics Analysis\media\image73.png" style="width:500px" />

Here, the linear velocity is assessed. If `linear.y` is greater than 1e-8, it indicates that the vehicle needs to translate sideways. Since the tracked chassis cannot perform sideways translation, the linear velocity will be set to zero in this case.

Speed Publishing:

```py
        if self.machine_type == 'JetRover_Mecanum':
            self.angular_z = msg.angular.z
            speeds = self.mecanum.set_velocity(self.linear_x, self.linear_y, self.angular_z)
            self.motor_pub.publish(speeds)
```

The speed is published to the kinematics model of the Mecanum wheels. Since the tracked chassis only uses two motors and `linear_y `must be zero in the input, the two motors will handle differential motion based on `angular_z` to achieve turning.

## 16.2 Motion Control

### 16.2.1 IMU, Linear Velocity and Angular Velocity Calibration

> [!NOTE]
>
> **Note:**
>
> * **The robot has been calibrated before leaving the factory and does not require additional calibration. The information is provided for reference only. If you observe significant deviations during robot movement, such as noticeable drifting to one side when moving forward or an inability to travel straight, you can consult the following tutorial for calibration.**
>
> * **Calibration aims to minimize deviations, but actual hardware variations are inherent. Hence, adjust the calibration to a level that reasonably suits your requirements.**

If the robot exhibits deviations during operation, it may require IMU calibration. Once the calibration process is completed, the robot can resume normal operation.

* **IMU Calibration**

IMU (Inertial Measurement Unit) is a device that measures the three-axis attitude angles (angular velocity) and acceleration of an object. The gyroscope and accelerometer are the main components of the IMU, providing a total of 6 degrees of freedom to measure the angular velocity and acceleration of the object in three-dimensional space. Upon receiving the first IMU message, the node will prompt you to maintain the IMU in a specific orientation and press Enter to record the measurement values. After completing measurements in all 6 directions, the node will calculate calibration parameters and write them to the specified YAML file. The specific steps are as follows:

> [!NOTE]
>
> **Note: The input command is case-sensitive, and keywords can be completed using the Tab key.**

1. Start the robot, and access the robot system desktop using NoMachine.

2. Click-on <img src="../_static/media/4/section_13_2. Motion Control/media/image4.png" style="width:50px" /> to open the ROS1 command-line terminal.

3. Execute the command to disable the app auto-start service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

4. Click-on <img src="../_static/media/4/section_13_2. Motion Control/media/image6.png" style="width:50px" /> to open the ROS2 command-line terminal.

5. Run the command and hit Enter key to enable STM32 control node:

   ```py
   ros2 launch ros_robot_controller ros_robot_controller.launch.py
   ```

6. Open a new ROS2 command-line terminal, then input the following command, and hit Enter key to initiate IMU calibration:

   ```py
   ros2 run imu_calib do_calib --ros-args -r imu:=/ros_robot_controller/imu_raw --param output_file:=/home/ubuntu/ros2_ws/src/calibration/config/imu_calib.yaml
   ```

7. When prompted, align the robot with its front side and press Enter. The initial orientation is considered as forward, and subsequent placements should follow this initial direction.

   <img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image9.png" style="width:500px" />

   <img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image10.png" style="width:500px"  />

   After you successfully calibrate all direction, the following prompt will appear.

   <img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image11.png" style="width:500px" />

8. Align the robot to the rear, then press Enter.

   <img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image12.png" style="width:500px" />

   <img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image13.png" style="width:500px"  />

9. Align the robot to the left, then hit Enter.

   <img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image14.png" style="width:500px" />

   <img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image15.png" style="width:500px"  />

10. Align the robot to the right, then press Enter.

    <img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image16.png" style="width:500px" />

    <img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image17.png" style="width:500px"  />

11. Lift the robot, place it facing upwards, and press Enter. When positioning vertically, be careful to avoid instability or collisions. Use your hand for support to prevent damage to the depth camera or screen.

    <img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image18.png" style="width:500px" />

    <img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image19.png" style="width:500px"  />

12. Place the robot as pictured, then hit Enter.

    <img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image20.png" style="width:500px" />

    <img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image21.png" style="width:500px"  />

13. If the below prompt shows up, it means the calibration is complete. To exit, use short-cut ‘**ctrl+c**’.

    <img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image22.png" style="width:500px" />

14. After calibration, execute the command to verify the calibrated model.

    ```py
    ros2 launch peripherals imu_view.launch.py
    ```

    <img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image24.png" style="width:500px" />

* **Angular Velocity Calibration**

To calibrate the angular velocity, the robot needs to perform a full rotation independently. During testing, it's crucial to mark the robot's orientation to facilitate the observation of any deviations. The specific steps are outlined below:

> [!NOTE]
>
> **Note: The input command is case-sensitive, and keywords can be completed using the Tab key.**

1. Place the robot on a flat surface and place a piece of tape or other marker in front of the robot's center.

2. Start the robot, and connect it to the robot system desktop using NoMachine.

3. Click-on <img src="../_static/media/4/section_13_2. Motion Control/media/image4.png" style="width:50px" /> to open the ROS1 command-line terminal.

4. Run the command and hit Enter to disable the app auto-start service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

5. Click-on <img src="../_static/media/4/section_13_2. Motion Control/media/image6.png" style="width:50px" /> to open the ROS2 command-line terminal.

6. Run the command and press Enter to start adjusting the angular velocity using the "turn" parameter.

   ```py
   ros2 launch calibration angular_calib.launch.py
   ```

Click on "**calibrate_angular**" on the left side. The calibration interface will appear as shown below.

<img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image26.png" style="width:500px" />

The parameters on the left side of the interface are defined as follows:

The first parameter, "**test_angle**," represents the test rotation angle, with a default value of 360°.

The second parameter, "**speed**," represents the linear velocity with a default value of 0.15 meters per second.

The third parameter, "**tolerance**," represents the error value. A smaller error value results in more significant robot oscillations after reaching the target position.

The fourth parameter, "**motor_turn_scale_correction**," represents the motor rotation scale correction.

The fifth parameter, "**odom_angle_scale_correction**," represents the odometry angle scale correction.

The sixth parameter, "**start_test_turn**," is the button to start testing the motor rotation scale correction.

The seventh parameter, "**start_test**," is the button to start testing the odometry angle scale correction.

<img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image27.png" style="width:500px"  />

Ensure the robot is properly aligned, with the marker placed in front of it. Check the "**start_test_turn**" option and the robot will rotate in place. If it fails to complete a full rotation, you need to adjust the "**odom_angule_scale_correction**" value, which controls the motor's rotation scale. It is recommended to adjust this value in increments of 0.01.

<img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image28.png" style="width:500px" />

7. Open a new command-line terminal, and execute the command to navigate to the directory containing calibration configuration files.

   ```py
   cd ~/ros2_ws/src/driver/controller/config
   ```

8. Execute the command to open the configuration file.

   ```py
   vim calibrate_params.yaml
   ```

9. Press ‘**I**’ key to navigate to the editing mode, and modify the value of "**angular_correctqion_factor**" to the adjusted value of "**odom_angule_scale_correction**"

   <img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image31.png" style="width:500px" />

> [!NOTE]
>
> **Note: The aforementioned operations are conducted on the Mecanum-wheel version and are equally applicable to the tank chassis version.**

10) After modification, press the "**ESC**" key, enter "**:wq**" to exit and save the changes.

* **Linear Velocity Calibration**

> [!NOTE]
>
> **Note: The input command is case-sensitive, and keywords can be completed using the Tab key.**

Position the robot on a flat and open surface. Mark the starting point with tape or any other indicator in front of the robot, and position the endpoint tape or another marker 1 meter ahead of the robot.

1. Start the robot, and access the robot system desktop using NoMachine.

2. Click-on <img src="../_static/media/4/section_13_2. Motion Control/media/image4.png" style="width:50px" /> to open the ROS1 command-line terminal.

3. Execute the command and hit Enter to disable the app auto-start service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

4. Click-on <img src="../_static/media/4/section_13_2. Motion Control/media/image6.png" style="width:50px" /> to open the ROS2 command-line terminal.

5. Execute the command and press Enter to activate the adjustment of the linear velocity.

   ```py
   ros2 launch calibration linear_calib.launch.py
   ```

6. Click on "**calibrate_linear**" on the left side, and the calibration interface will appear as follows.

   <img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image33.png" style="width:500px" />

The meanings of parameters on the left side of the interface are as follows:

The first parameter "**test_distance**" represents the testing distance, with a default value of 1 meter.

The second parameter "**speed**" represents the linear velocity, with a default value of 0.2 meters per second.

The third parameter "**tolerance**" represents the error value. A smaller error value results in greater robot shaking after reaching the target position.

The fourth parameter "**odom_linear_scale_correction**" represents the odometer linear scale correction.

The fifth parameter "**start_test**" is the button to start testing the odometer linear scale correction.

<img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image34.png" style="width:500px"  />

7. Ensure the robot is properly aligned and positioned at the starting point marker. Check the box for "**start_test**" and the robot will move forward. Observe if the robot travels in a straight line. If there is deviation, adjust the value of "**odom_linear_scale_correction**". This value adjusts the motor's scaling factor for forward movement. It is recommended to adjust this value by increments of 0.01 each time.

   <img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image35.png" style="width:500px" />

8. Open a new ROS2 command line terminal and enter the command to navigate to the directory containing the configuration file.

   ```py
   cd ~/ros2_ws/src/driver/controller/config
   ```

9. Enter the command to open the configuration file.

   ```py
   vim calibrate_params.yaml
   ```

10. Press the "**I**" key to enter edit mode and modify the value of "**linear_correction_factor**" to the adjusted value of "**odom_linear_scale_correction**."

    <img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image37.png" style="width:500px" />

11. After making the modifications, press the "**ESC**" key, enter "**:wq**" to exit and save the changes.

12. If you need to terminate this program, use short-cut ‘**Ctrl+C**’.

###  16.2.2 Publish IMU and Odometer Data

In robot navigation, accurately calculating real-time position is essential. Normally, we obtain odometer information using motor encoders and the robot's kinematic model. However, in specific situations, like when the robot's wheels rotate in place or when the robot is lifted, it may move a distance without the wheels actually turning.

To address wheel slip or accumulated errors in such cases, combining IMU and odometer data can yield more precise odometer information. This improves mapping and navigation accuracy in scenarios where wheel slip or cumulative errors may occur.

* **Introduction to IMU and Odometer**

The IMU (Inertial Measurement Unit) is a device that measures the three-axis attitude angles (angular velocity) and acceleration of an object. It consists of the gyroscope and accelerometer as its main components, providing a total of 6 degrees of freedom to measure the object's angular velocity and acceleration in three-dimensional space.

An odometer is a method used to estimate changes in an object's position over time using data obtained from motion sensors. This method is widely applied in robotic systems to estimate the distance traveled by the robot relative to its initial position.

There are common methods for odometer positioning, including the wheel odometer, visual odometer, and visual-inertial odometer. In robotics, we specifically use the wheel odometer. To illustrate the principle of the wheel odometer, consider a carriage where you want to determine the distance from point A to point B. By knowing the circumference of the carriage wheels and installing a device to count wheel revolutions, you can calculate the distance based on wheel circumference, time taken, and the number of wheel revolutions.

While the wheel odometer provides basic pose estimation for wheeled robots, it has a significant drawback: accumulated error. In addition to inherent hardware errors, environmental factors such as slippery tires due to weather conditions contribute to increasing odometer errors with the robot's movement distance.

Therefore, both IMU and odometer are essential components in a robot. These two components are utilized to measure the three-axis attitude angles (or angular velocity) and acceleration of the object, as well as to estimate the distance, pose, velocity, and direction of the robot relative to its initial position.

To address these errors, we combine IMU data with odometer data to obtain more accurate information. IMU data is published through the "/imu" topic, and odometer data is published through "/odom". After obtaining data from both sources, the data is fused using the "ekf" package in ROS, and the fused localization information is then republished.

* **IMU Data Publishing**

**1. Initiate Service**

> [!NOTE]
>
> **Note: When entering commands, it is essential to strictly distinguish between uppercase and lowercase letters, and keywords can be autocompleted using Tab key.**

1. Start the robot, and access the robot system desktop using NoMachine.

2. Click-on <img src="../_static/media/4/section_13_2. Motion Control/media/image4.png" style="width:50px" /> to open the ROS1 command-line terminal.

3. Execute the command, and hit Enter to disable the app auto-start service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

4. Click-on <img src="../_static/media/4/section_13_2. Motion Control/media/image6.png" style="width:50px" /> to open the ROS2 command-line terminal.

5. Run the command and press Enter to publish the IMU data.

   ```py
   ros2 launch peripherals imu_filter.launch.py
   ```

**2. Data Viewing**

1. Open a new ROS2 command line terminal, and execute the command to check the current topic.

   ```py
   ros2 topic list
   ```

<img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image39.png" style="width:500px" />

2. Enter the command to view the type, publisher, and subscribers of the "**/imu**" topic. You can replace "**/imu**" with the topic you want to view. The type of this topic is "**sensor_msgs/msg/Imu**".

   ```py
   ros2 topic info /imu
   ```

<img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image40.png" style="width:500px" />

3. Use the following command to display the content of the topic message. Feel free to replace '**imu**' with the name of the topic you wish to view.

   ```py
   ros2 topic echo /imu
   ```

<img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image42.png" style="width:500px" />

The terminal will display the data from the three axes of the IMU.

* **Odometer Data Publishing**

**1. Initiate Service**

> [!NOTE]
>
> **Note: When entering commands, it is essential to strictly distinguish between uppercase and lowercase letters, and keywords can be autocompleted using Tab key.**

1. Start the robot, and access the robot system desktop using NoMachine.

2. Double-click <img src="../_static/media/4/section_13_2. Motion Control/media/image4.png" style="width:50px" /> to open the ROS1 command line terminal.

3. Execute the command, and hit Enter to disable app auto-start service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

4. Double-click <img src="../_static/media/4/section_13_2. Motion Control/media/image6.png" style="width:50px" /> to open the ROS2 command-line terminal.

5. Run the command to publish the odometer data.

   ```py
   ros2 launch controller odom_publisher.launch.py
   ```

**2. Data Viewing**

1. Open a new ROS2 command line terminal, and run the command below to check the current topic.

   ```py
   ros2 topic list
   ```

<img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image44.png" style="width:500px" />

2. Enter the command to view the type, publisher, and subscribers of the "**/odom_raw**" topic. You can replace "**/odom_raw**" with the topic you want to view. The type of this topic is "**nav_msgs/msg/Odometry**".

   ```py
   ros2 topic echo /odom_raw
   ```

   <img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image45.png" style="width:500px" />

3. Enter the command to print the topic message contents. You can replace the topic you want to view as needed.

   ```py
   ros2 topic echo /odom_raw
   ```

<img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image47.png" style="width:500px" />

The message content includes acquired pose and velocity data.

### 16.2.3 Robot Speed Control

Speed control is achieved by adjusting the linear velocity parameter.

* **Program Logic**

Based on the robot's movement characteristics, control the active wheels to achieve forward, backward, and turning movements.

In the program, subscribe to the '**/controller/cmd_vel'** movement control topic to obtain the set linear and angular velocities. Then, analyze and calculate based on these velocities to determine the car's movement speed.

The source code for this program can be found at:

**/home/ubuntu/ros2_ws/src/driver/controller/controller/odom_publisher_node.py**

```py
class Controller(Node):
    
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        self.x = 0.0
        self.y = 0.0
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        self.pose_yaw = 0
        self.last_time = None
        self.current_time = None
        signal.signal(signal.SIGINT, self.shutdown)
        self.wheel_diameter_tank = 0.052
        self.wheel_diameter_mecanum = 0.097
        self.ackermann = ackermann.AckermannChassis(wheelbase=0.216, track_width=0.195, wheel_diameter=self.wheel_diameter_mecanum)
        self.mecanum = mecanum.MecanumChassis(wheelbase=0.216, track_width=0.195, wheel_diameter=self.wheel_diameter_mecanum)

```

* **Disable APP Service and Initiate Speed Control**

> [!NOTE]
>
> **Note: When entering commands, it is essential to strictly distinguish between uppercase and lowercase letters, and keywords can be autocompleted using Tab key.**

1. Start the robot, and access the robot system desktop using NoMachine.

2. Click-on <img src="../_static/media/4/section_13_2. Motion Control/media/image49.png" style="width:50px" /> to open the ROS1 command-line terminal.

3. Execute the command and hit Enter to disable the app auto-start service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

4. Click-on <img src="../_static/media/4/section_13_2. Motion Control/media/image6.png" style="width:50px" /> to open the ROS2 command-line terminal.

5. Enter the command to enable motion control service.

   ```py
   ros2 launch controller controller.launch.py
   ```

6. Open a new ROS2 command-line terminal, then enter the following command to enable the speed control.

   ```py
   ros2 topic pub /controller/cmd_vel geometry_msgs/Twist "linear:
     x: 0.0
     y: 0.0
     z: 0.0
   angular:
     x: 0.0
     y: 0.0
     z: 0.0"
   ```

   In this context, '**linear**' represents the set linear velocity, considering the robot's viewpoint where the X-axis points forward without any influence from the Y or Z directions.

   On the other hand, '**angular**' pertains to the set angular velocity. A positive Z-value induces a left turn in the robot, while a negative Z-value causes the robot to turn right. This configuration has no impact on the X and Y directions.

   > [!NOTE]
   >
   > **Note:**
   >
   > **In this scenario, the linear velocity (x) is measured in meters per second, and it is advisable to maintain it within the range of "-0.6 to 0.6".**
   >
   > **The angular velocity (z) denotes the turning speed and is determined by the formulas V=ωR (linear velocity equals angular velocity times radius) and tanΦA=D/R (where z=ω, D=0.213, and ΦA represents the turning angle). The angle ΦA should be within the range of 0 to 36 degrees.**

Use the arrow keys to navigate and modify the relevant parameters. For example, to make the robot move forward, adjust the linear velocity (X) to 0.1, and then press Enter to execute the action.

<img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image52.png" style="width:500px" />

7. To bring the robot car to a stop, open a new terminal and set the linear velocity to '0.0'.

   <img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image51.png" style="width:500px" />

8. If you need to terminate this program, use short-cut ‘Ctrl+C’.

   > [!NOTE]
   >
   > **Note: To bring the robot car to a stop, please create a new terminal and adjust the linear velocity. Using the 'Ctrl+C' shortcut alone may not effectively halt the robot car.**

* **Change Forward Speed**

By modifying the linear velocity value (X), the robot can achieve forward movement at variable speeds. For instance, to make the robot shift diagonally to the left front, during step 5 in ‘**[16.2.3 Robot Speed Control->Disable App Service and Initiate Speed Control]()**’, set X to 'X: 0.3'.

<img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image53.png" style="width:500px" />

Upon pressing Enter, the robot will move forward at a speed of 0.3 meters per second in a straight direction.

* **Program Outcome**

After the game starts, the robot will go forward at the speed of 0.3m/s.

* **Program Analysis**

<img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image54.png" style="width:500px"  />

The files include '**hiwonder_controller.launch**' for launch configuration, '**calibrate_params.yaml**' for parameter configuration, and '**odom_publisher.py**' for program execution. During startup, the launch file is executed first. It loads the YAML configuration file and passes the parameters to the ROS nodes. Subsequently, the nodes initialize by reading the configuration parameters from the ROS nodes and communicate with other nodes to collaboratively implement functionalities.

**1. Launch File**

<img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image55.png" style="width:500px"  />

The launch file is located in: **/home/ubuntu/ros2_ws/src/driver/controller/launch/controller.launch.py**

```py
import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.conditions import IfCondition
from nav2_common.launch import ReplaceString
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction, OpaqueFunction

def launch_setup(context):
    compiled = os.environ['need_compile']
    namespace = LaunchConfiguration('namespace', default='')
    use_namespace = LaunchConfiguration('use_namespace', default='false').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    enable_odom = LaunchConfiguration('enable_odom', default='true')
    map_frame = LaunchConfiguration('map_frame', default='map')
    odom_frame = LaunchConfiguration('odom_frame', default='odom')
    base_frame = LaunchConfiguration('base_frame', default='base_footprint')
    imu_frame = LaunchConfiguration('imu_frame', default='imu_link')
    frame_prefix = LaunchConfiguration('frame_prefix', default='')

    namespace_arg = DeclareLaunchArgument('namespace', default_value=namespace)
    use_namespace_arg = DeclareLaunchArgument('use_namespace', default_value=use_namespace)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)
    enable_odom_arg = DeclareLaunchArgument('enable_odom', default_value=enable_odom)
    map_frame_arg = DeclareLaunchArgument('map_frame', default_value=map_frame)
    odom_frame_arg = DeclareLaunchArgument('odom_frame', default_value=odom_frame)
    base_frame_arg = DeclareLaunchArgument('base_frame', default_value=base_frame)
    imu_frame_arg = DeclareLaunchArgument('imu_frame', default_value=imu_frame)
    frame_prefix_arg = DeclareLaunchArgument('frame_prefix', default_value=frame_prefix)
```

1. Set the storage path

Retrieve the paths for the three packages: peripherals, controller, and servo_controller.

```py
    if compiled == 'True':
        peripherals_package_path = get_package_share_directory('peripherals')
        controller_package_path = get_package_share_directory('controller')
        servo_controller_package_path = get_package_share_directory('servo_controller')
    else:
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'
        controller_package_path = '/home/ubuntu/ros2_ws/src/driver/controller'
        servo_controller_package_path = '/home/ubuntu/ros2_ws/src/driver/servo_controller'
```

2. Initiate other Launch files

   ```
       odom_publisher_launch = IncludeLaunchDescription(
           PythonLaunchDescriptionSource([os.path.join(controller_package_path, 'launch/odom_publisher.launch.py')
           ]),
           launch_arguments={
               'namespace': namespace,
               'use_namespace': use_namespace,
               'imu_frame': imu_frame,
               'frame_prefix': frame_prefix,
               'base_frame': base_frame,
               'odom_frame': odom_frame
           }.items()
       )
   ```

`odom_publisher_launch` Odometer launch

`imu_filter_launch` IMU launch

3. Initiate Node

Launch the EKF fusion node.

```
    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_param, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('odometry/filtered', 'odom'),
            ('cmd_vel', 'controller/cmd_vel')
        ],
        condition=IfCondition(enable_odom),
    )

    servo_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(servo_controller_package_path, 'launch/servo_controller.launch.py')
        ]),
        launch_arguments={
            'base_frame': base_frame,
        }.items()
    )
```

**2. Python Program**

<img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image60.png" style="width:500px"  />

The Python program is saved in: **/home/ubuntu/ros2_ws/src/driver/controller/launch/controller.launch.py**

1. Import Library

   <img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image61.png" style="width:500px" />

2. Main Function

   

<img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image62.png" style="width:500px" />

The controller class is invoked here, and wait for the node to exit.

3. Global Parameter

<img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image63.png" style="width:500px" />

<img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image64.png" style="width:500px"  />

4. Function

<img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image65.png" style="width:500px" />

The function '**rpy2qua**' is used to convert Euler angles to quaternions.

The function '**qua2rpy**' is used to convert quaternions to Euler angles.

5. Analysis of the Controller Class

Invoke kinematics

<img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image66.png" style="width:500px" />

The '**self.ackermann**' calls the Ackermann kinematics and initializes the Ackermann kinematics object.

The '**self.mecanum**' calls the Mecanum kinematics and initializes the Mecanum kinematics object.

Define ROS parameters:

<img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image67.png" style="width:500px" />

The function `self.declare_parameter` is used to define a certain parameter.

The function `self.get_parameter` is used to obtain a certain parameter.

`pub_odom_topic`: Whether to publish the odometry node

`base_frame_id`: Robot footprint ID

`odom_frame_id`: Robot odometry ID

`linear_correction_factor`: Linear velocity correction factor

`angular_correction_factor`: Angular velocity correction factor

`machine_type`: Type of robot

Publish odometer:

<img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image68.png" style="width:500px" />

Based on the parameter `pub_odom_topic`, determine whether to publish the odometry node. If publishing is required, initialize the node, fill in the corresponding parameters, and publish the odometry using the `self.create_publisher` function. Update the odometry data using the `self.cal_odom_fun` function.

**Topic Publishing:**

<img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image69.png" style="width:500px" />

The function `self.create_subscription` is used to receive topics.

The function `self.create_service` is used to create services.

`self.motor_pub` publishes the motor control topic `ros_robot_controller/set_motor`, with the message type MotorsState.

`self.servo_state_pub` publishes the servo control topic `ros_robot_controller/bus_servo/set_state`, with the message type SetBusServoState.

`self.pose_pub` publishes the servo control topic `set_pose`, with the message type PoseWithCovarianceStamped.

Publishes the topic `set_odom`, with the message type Pose2D, and the callback function `self.set_odom`.

Publishes the topic '**controller/cmd_vel**', with the message type Twist, and the callback function `self.cmd_vel_callback`.

Publishes the topic '**cmd_vel**', with the message type Twist, and the callback function `self.set_app_cmd_vel_callback`.

Publishes the service '**controller/load_calibrate_param**', with the service type Trigger, and the callback function `self.load_calibrate_param`.

Publishes the service '**~/init_finish**', with the service type Trigger, and the callback function `self.get_node_state`.

**Explanation of Controller Class Functions:**

<img class="common_img" src="../_static/media/4/section_13_2. Motion Control/media/image70.png" style="width:500px"  />

* **FAQ**

Q: The robot continues to move forward even after pressing Ctrl+C in the terminal.

A: In such a situation, you need to open a new terminal and enter the command:

**'rostopic pub /hiwonder_controller/cmd_vel geometry_msgs/Twist "linear**:', then press TAB to autocomplete, set the speed to 0, and press Enter to execute.