# 3. ROS1-Chassis Motion Control Lesson

## 3.1 Kinematics Analysis

The JetRover series chassis is primarily categorized into three types: Mecanum wheel chassis, Ackermann chassis, and tracked vehicle chassis. Each chassis type features unique kinematic modes, and this manual will provide a systematic introduction to each of them.

### 3.1.1 Overview:

The Mecanum wheel vehicle, Ackermann chassis-equipped vehicle, and tracked vehicle are three distinct types of vehicle chassis designs, each displaying significant differences in both structure and functionality.

* **Wheel Type**

**Mecanum Wheel Vehicle**: Mecanum wheel vehicles employ multiple small freely rotating wheels to bear the vehicle's weight. Typically situated at the chassis's bottom, these wheels can rotate independently, enhancing the vehicle's ability to turn and maneuver with greater ease.

**Ackermann Chassis Vehicle**: Ackermann chassis vehicles also utilize wheels, often featuring front-wheel drive, while the rear wheels bear the weight. The front wheels are usually steerable, facilitating smoother turning.

**Tracked Vehicle**: Tracked vehicles replace traditional wheels with tracks composed of a series of connected chain links. This design enhances traction and suspension across various terrain conditions, providing a distinct advantage on uneven surfaces.

* **Application**

**Mecanum Wheel Vehicle**: Designed for urban environments and road use, Mecanum wheel vehicles excel at navigating sharp turns with high maneuverability.

**Ackermann Chassis Vehicle**: Ackermann chassis vehicles are well-suited for general road driving, including cars, trucks, and motorcycles, thanks to their wheels optimized for these applications.

**Tracked Vehicle**: Primarily employed for tasks involving challenging terrain conditions, tracked vehicles, such as military vehicles, construction equipment, and agricultural machinery, offer superior performance in demanding environments.

### 3.1.2 Mecanum Wheel:

* **Hardware Structure**

<img class="common_img" src="../_static/media/3/section_137_1. Kinematics Analysis/media/image4.png" style="width:500px"  />

The Mecanum wheel comprises rollers and an axle. The axle functions as the main support structure for the entire wheel, with rollers attached to it. The axle axis is positioned at a 45-degree angle to the roller axis. Typically, Mecanum wheels operate in groups of four, with two left wheels and two right wheels. Wheels A and B are symmetrical.

There are various combinations of four Mecanum wheels, such as AAAA, BBBB, AABB, ABAB, BABA. However, not all combinations allow the robot car to move in all directions, including forward, backward, and sideways. The Mecanum-wheel chassis combination is ABAB, enabling omnidirectional movement.

<img class="common_img" src="../_static/media/3/section_137_1. Kinematics Analysis/media/image5.png" style="width:500px"  />

* **Physical Characteristics**

The vehicle achieves omnidirectional motion by summing up the propelling forces of the ground-engaging rollers. This summation can occur in any direction through adjustments in wheel rotation direction and torque magnitude for the four wheels.

Because of the rollers' specific orientation at a certain angle to the wheel circumference, Mecanum wheels have the ability to slip sideways. The generatrix of these small rollers is unique. As the Mecanum wheel rotates around its fixed axle, each small roller's envelope forms a cylindrical surface, allowing the wheel to continuously roll forward.

* **Motion Principle and Formula**

When conducting kinematic analysis, we can consider the kinematic model of Mecanum wheels, which includes the following parameters:

<img class="common_img" src="../_static/media/3/section_137_1. Kinematics Analysis/media/image6.png" style="width:500px"/>

1.  VX: Velocity of the Mecanum wheel in the X-axis (typically front and rear direction).Velocity of the Mecanum wheel in the X-axis (typically front and rear direction).

2.  VY: Velocity of the Mecanum wheel in the Y-axis (typically left and right direction).

3.  Angular velocity of the Mecanum wheel chassis (rotation speed of the chassis around its own center).

4.  Real-time velocities of the four wheels of the Mecanum wheel.

5.  The motion of the right front wheel in the plane can be decomposed into:

6.  VBx: Velocity of the Mecanum wheel in the X-axis (typically front and rear direction).

7.  VBy: Velocity of the Mecanum wheel in the Y-axis (typically left and right direction).

8.  L: Distance between the centers of the left and right wheels.

9.  H: Distance between the centers of the front and rear wheels.

10.  θ: Angle formed by the chassis body center and the center of the right front wheel, typically 45°.

11.  With these parameters, we can perform kinematic analysis of the Mecanum wheel chassis. The following are key mathematical formulas:

**Kinematics Formula:**

To simplify the mathematical model for kinematics, we make the following two idealized assumptions:

（1）Omni-directional wheels do not slip on the ground, and there is sufficient friction with the ground.

（2）The 4 wheels are distributed at the corners of a rectangle or square, with the wheels parallel to each other.

Here, we decompose the rigid body motion of the car into three components linearly. By calculating the velocities of the four wheels when the output Mecanum wheel chassis translates along the X+ and Y+ directions and rotates along the Z+ direction, we can combine these three simple motions using formulas to determine the required speeds of the four wheels.

In the equations, A, B, C, and D represent the rotational speeds of the four wheels, i.e., the motor speeds. VX is the translation speed of the car along the X-axis, VY is the translation speed along the Y-axis, and ω is the rotational speed along the Z-axis. L/2 is half of the car's wheelbase, and H/2 is half of the car's axle distance.

1\. The velocity components of each wheel during the robot's translation along the X-axis can be calculated using the following formulas:

<img class="common_img" src="../_static/media/3/section_137_1. Kinematics Analysis/media/image7.png" style="width:500px" />

Where,

<img src="../_static/media/3/section_137_1. Kinematics Analysis/media/image8.png" style="width:100px" />：Real-time velocities of the four Mecanum wheels，

<img src="../_static/media/3/section_137_1. Kinematics Analysis/media/image9.png" style="width:50px" />：Velocity of the Mecanum wheel in the X-axis direction

2\. When the robot translates along the Y-axis, the speed component of each wheel can be calculated using the following formula:

<img class="common_img" src="../_static/media/3/section_137_1. Kinematics Analysis/media/image10.png" style="width:500px" />

Where,  <img src="../_static/media/3/section_137_1. Kinematics Analysis/media/image15.png" style="width:50px" />is the velocity of the robot in the Y-axis direction.

3\. When the robot rotates along the Z-axis, the speed component of each wheel can be calculated using the following formulas:

<img class="common_img" src="../_static/media/3/section_137_1. Kinematics Analysis/media/image11.png" style="width:500px" />

<img class="common_img" src="../_static/media/3/section_137_1. Kinematics Analysis/media/image12.png" style="width:500px" />

Where，<img src="../_static/media/3/section_137_1. Kinematics Analysis/media/image13.png" style="width:100px" />:The angular velocity of the Mecanum wheel chassis (i.e., the speed at which the chassis rotates around its own center)

4\. Combining the velocities in the X, Y, and Z directions allows for the computation of the rotation speeds of the four wheels based on the motion state of the car.

<img class="common_img" src="../_static/media/3/section_137_1. Kinematics Analysis/media/image14.png" style="width:500px" />

* **Program Outcome**

The program file is saved to the following folder.

<img class="common_img" src="../_static/media/3/section_137_1. Kinematics Analysis/media/image16.png" style="width:500px" />

<img class="common_img" src="../_static/media/3/section_137_1. Kinematics Analysis/media/image17.png" style="width:500px" />

The set of files includes hiwonder_controller.launch as the launch initiation file, calibrate_params.yaml as the configuration file, and odom_publisher.py as the program file.

During the launch process, the launch file is executed first. This file loads the YAML configuration file, passes the parameters to the ROS node, and then the node initializes by reading configuration parameters from the ROS node. Subsequently, the node communicates with other nodes to collaboratively achieve the desired functionality.

 **1. Chassis control script file:**

The program source code is saved in: **src/hiwonder_driver/hiwonder_controller/scripts**

**(1) Import Necessary Library Files**

```py
import os
import math
import rospy
import tf2_ros
import threading
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from hiwonder_sdk.mecanum import MecanumChassis
from hiwonder_sdk.ackermann  import AckermannChassis
from ros_robot_controller.msg import MotorsState, BusServoState, SetBusServoState
from geometry_msgs.msg import Pose2D, Pose, Twist, PoseWithCovarianceStamped, TransformStamped
```

First, it initializes the necessary libraries and chassis model, and establishes communication channels between ROS nodes to achieve the publication of odometry information.

**(2) Initialize Node and Drive Odometry Calculation Process**

```py
if __name__ == "__main__":
    node = Controller()
    rospy.spin()
```

First, through the initialization of the Controller class, the necessary registrations for publishing and subscribing, parameter reading, and other initialization tasks are completed. Following this, the **`rospy.spin()`** function is employed to enter the main ROS loop. Within this loop, the calculation function inside the Controller class is periodically called, continuously updating odometry data, ultimately achieving the process of publishing and subscribing.

**(3) Subscribe to Command Topic and Drive Speed Calculation**

```py
rospy.Subscriber('cmd_vel', Twist, self.app_cmd_vel_callback)
```

Subscribe to the ‘**/hiwonder_controller/cmd_vel**’ topic through **rospy.Subscriber**

```py
    def app_cmd_vel_callback(self, msg):
        if msg.linear.x > 0.2:
            msg.linear.x = 0.2
        if msg.linear.x < -0.2:
            msg.linear.x = -0.2
        if msg.linear.y > 0.2:
            msg.linear.y = 0.2
        if msg.linear.y < -0.2:
            msg.linear.y = -0.2
        if msg.angular.z > 0.5:
            msg.angular.z = 0.5
        if msg.angular.z < -0.5:
            msg.angular.z = -0.5
        self.cmd_vel_callback(msg)
```

Utilizing the received linear and angular velocity command message (msg), compute the effective linear velocities (self.linear_x, self.linear_y) and angular velocity (self.angular_z) of the robot. Distinct velocity calculation modes are employed for diverse chassis types, including mecanum, tank, and ackermann.

**(4) Scheduled Invocation of Odometry Calculation Update Function**

```py
rospy.Timer(rospy.Duration(self.dt), self.cal_odom_fun)
```

In the main function, initialize a timer object with a period of self.dt, and the callback function for the timer is `cal_odom_fun`.

**(5) Iteratively Compute and Publish New Odometry Data**

```py
    def cal_odom_fun(self, event):
        self.current_time = rospy.Time.now()
       
        if self.last_time is None:
            self.dt = 0
        else:
            # 计算时间间隔
            self.dt = (self.current_time - self.last_time).to_sec()
        
        self.odom.header.stamp = self.current_time
        
        self.x += math.cos(self.pose_yaw)*self.linear_x*self.dt - math.sin(self.pose_yaw)*self.linear_y*self.dt
        self.y += math.sin(self.pose_yaw)*self.linear_x*self.dt + math.cos(self.pose_yaw)*self.linear_y*self.dt 

        self.odom.pose.pose.position.x = self.linear_factor*self.x 
        self.odom.pose.pose.position.y = self.linear_factor*self.y
        self.odom.pose.pose.position.z = 0
       
        self.pose_yaw += self.angular_factor*self.angular_z*self.dt
        
        self.odom.pose.pose.orientation = rpy2qua(0, 0, self.pose_yaw)
        self.odom.twist.twist.linear.x = self.linear_x
        self.odom.twist.twist.linear.y = self.linear_y
        self.odom.twist.twist.angular.z = self.angular_z
```

The `cal_odom_fun`, operating as a scheduled callback, is regularly triggered. During each invocation, numerical integration updates are applied to the pose, leading to the computation of a new odometry state. Subsequently, the odom message is refreshed, and the updated odometry message (odom) is published.

The rospy.Timer, set up with the scheduled callback, coordinates the cyclic execution of `cal_odom_fun`. This iterative process, encompassing state calculation, updating, and message publishing, establishes a loop for the continuous computation and publication of up-to-date odometry data.

**(6) Selecting Different Chassis Types Using the load_calibrate_param Function**

For instance, when the robot chassis is equipped with a Mecanum wheel base, the machine_type parameter is set to JetRover_Mecanum. Similarly, when utilizing an Ackermann chassis, machine_type is changed to JetRover_Tank, and the same principle applies to track-type chassis.

```py
    def load_calibrate_param(self, msg):
        if self.machine_type == 'JetRover_Mecanum':
            self.linear_factor = rospy.get_param('~mecanum/linear_correction_factor', 1.00)
            self.angular_factor = rospy.get_param('~mecanum/angular_correction_factor', 1.00)
        elif self.machine_type == 'JetRover_Tank':
            self.linear_factor = rospy.get_param('~tank/linear_correction_factor', 1.00)
            self.angular_factor = rospy.get_param('~tank/angular_correction_factor', 1.00)
        elif self.machine_type == 'JetRover_Acker':
            self.linear_factor = rospy.get_param('~acker/linear_correction_factor', 1.00)
            self.angular_factor = rospy.get_param('~acker/angular_correction_factor', 1.00)
        
        return [True, 'load_calibrate_param']
```

**2. Launch File Analysis:**

```py
 	<arg name="freq"              default="50"/>
    <arg name="enable_odom"       default="true"/>
    <arg name="odom_topic"        default="odom"/>
    <arg name="odom_raw_topic"    default="odom_raw"/>
    <arg name="odom_frame"        default="odom"/>
    <arg name="base_frame"        default="base_footprint"/>
    <arg name="lidar_frame"       default="lidar_frame"/>
    <arg name="odom_lidar_topic"  default="odom_lidar"/>
    <arg name="scan_topic"        default="scan"/>
    <arg name="map_frame"         default="map"/>
    <argname="cmd_vel"      default="hiwonder_controller/cmd_vel"/>
    <arg name="imu_raw_topic"     default="/board/imu_raw"/>
    <arg name="imu_topic"         default="imu"/>
    <arg name="imu_link"          default="imu_link"/>
    <arg name="tf_prefix"         default=""/>
```

The `<arg>` node is primarily used to define parameters for both internal and external use in the launch file. Here, the following main parameters are defined:

Line 1: Defines the parameter for the node's operating frequency, named freq, with a default value of 50Hz.

Line 2: Defines the parameter for enabling or disabling the odometry feature, named `enable_odom`, with the default setting as enabled.

Line 3: Defines the parameter for the odometry message topic name, named `odom_topic`, with a default value of odom.

Line 4: Defines the parameter for the base frame name of the chassis, named `base_frame`, with a default value of `base_footprint`.

Line 5: Defines the parameter for the lidar frame name, named `lidar_frame`, with a default value of `lidar_frame`.

Line 6: Defines the parameter for configuring the topic where laser data contributes to odometry information. When matching laser point cloud data is detected, odometry results are generated. The parameter is named `odom_lidar_topic`, with a default value of `odom_lidar`.

Line 7: Defines the parameter for the laser scanner's message topic, named `scan_topic`, with a default value of scan.

Line 8: Defines the parameter for the map frame name, named `map_frame`, with a default value of map.

Line 9: The lower-level controller node subscribes to this topic to receive movement commands, and based on the instructions, controls motors for movement. The parameter is named `cmd_vel`, with a default value of **hiwonder_controller/cmd_vel**.

Line 10: Used to configure the raw data topic for the Inertial Measurement Unit (IMU), named `imu_raw_topic`, with a default value of **/board/imu_raw**.

Line 11: Used to configure the post-processed data topic for the IMU, named imu_topic, with a default value of imu.

Line 12: Used to configure the IMU coordinate system parameter, named imu_link, with a default value of imu_link.

Below is the loading of the robot URDF model:

```py
 <param name="robot_description" command="$(find xacro)/xacro '$(find hiwonder_description)/urdf/jetrover.xacro'" />
```

To begin, a parameter named "**robot_description**" is established, explicitly denoting its role in containing the description of the robot model.

Subsequently, the URDF file is processed using xacro. This file encapsulates the physical structure and motion information definitions of various components of the robot. Xacro is capable of parsing the URDF, generating a comprehensive description file in XML or URDF format for the robot model, and then assigning it to the "**robot_description**" parameter.

Ultimately, this parameter can be supplied to tools such as Rviz for visualizing the loaded robot model and to controllers for accessing the robot's structure to implement control actions.

Following this, a **robot_state_publisher** node is defined.

```py
	<node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" >
        <param name="tf_prefix" value="$(arg tf_prefix)" />
    </node>
```

This node is responsible for publishing spatial relationships among different components of the robot in the TF coordinate system, utilizing the robot's description information provided by the "**robot_description**" parameter.

Afterwards, it is essential to retrieve the value of the tf_prefix parameter. The relationships between the coordinates of various components in the world coordinate system are then computed based on the model and tf_prefix. These calculated results are published through TF to establish coordinate transformation relationships. Subsequently, the published TF information is shared with other nodes, completing the entire process from parameter definition to node execution and result publication.

The following section facilitates the transfer of data between the main launch file and the imported nodes.

```py
 	<include file="$(find hiwonder_sdk)/launch/board_node.launch">
        <arg name="freq"    value="$(arg freq)"/>
        <arg name="imu_link" value="$(arg imu_link)"/>
    </include>
```

The primary approach involves using the `<include>` tag to integrate the **board_node.launch** file into the main launch file, facilitating the importation of files.
The `<arg>` tag is employed to declare the "**freq**" and "**imu_link**" parameters that need to be transmitted to the `board_node`. As a result, the board_node file can access the necessary parameter values passed during startup through the main file, enabling parameter sharing between different files and ensuring the correct configuration for the included file.
Subsequently, parameter transmission is simplified by declaring the `<arg>` tag below, integrating the servo control node into the main file to ensure that its dependencies are satisfied for normal operation.

```py
 <include file="$(find hiwonder_servo_controllers)/launch/start.launch">
        <arg name="base_frame"   value="$(arg base_frame)" />
    </include>
```

The following code segment is used to include and configure the IMU data processing node.

```py
   <include file="$(find hiwonder_peripherals)/launch/imu_base.launch">
        <arg name="imu_raw_topic" value="$(arg imu_raw_topic)"/>
        <arg name="imu_topic"     value="$(arg imu_topic)"/>
    </include>
```

By subscribing to the raw IMU data topic, the readings from the gyroscope, accelerometer, and magnetometer are fused. After filtering and denoising, an attitude estimation is obtained.

Subsequently, the data is transformed from the IMU coordinate system to the robot's base coordinate system. Real-time optimization is performed by adjusting IMU parameters such as bias and scaling factors. The processed IMU data is then published to a common topic for use by subsequent nodes involved in localization, navigation, etc.

Below is the initiation of the odometry node, implementing the functionality of odometry.

```py
 	<node name="hiwonder_odom_publisher" pkg="hiwonder_controller" type="odom_publisher.py" required="true" output="screen">
        <rosparam file="$(find hiwonder_controller)/config/calibrate_params.yaml" command="load"/>
        <param name="freq"              value="$(arg freq)"/>
        <param name="odom_topic"        value="$(arg odom_raw_topic)"/>
        <param name="base_frame_id"     value="$(arg base_frame)"/>
        <param name="odom_frame_id"     value="$(arg odom_frame)"/>
        <param name="cmd_vel"           value="$(arg cmd_vel)"/>
    </node>
```

Firstly, the `<node>` tag is used here to add the hiwonder_odom_publisher node to the computation graph. Then, the `<param>` tag is utilized within the node to declare the relevant parameters needed. The actual parameter values are extracted from the main file using the value attribute. Following this, the `<rosparam>` tag is employed to load additional configurations from an external file, ensuring that the node has all the required parameters and settings. This facilitates the implementation of the odometry algorithm, fusion calculation of sensor data, and the publication of odometry output results through the defined topic.

Finally, based on certain conditions, the Extended Kalman Filter (EKF) localization algorithm is loaded, and the necessary support is provided to it through parameter passing. This achieves the integration of global localization functionality, as illustrated in the diagram below:

```py
<group if="$(arg enable_odom)">
        <!--ekf融合-->
        <include file="$(find hiwonder_slam)/launch/include/ekf.launch">
            <arg name="odom_topic"       value="$(arg odom_topic)"/>
            <arg name="odom_raw_topic"   value="$(arg odom_raw_topic)"/>
            <arg name="odom_lidar_topic" value="$(arg odom_lidar_topic)"/>
            <arg name="imu_topic"        value="$(arg imu_topic)"/>
            <arg name="map_frame"        value="$(arg map_frame)"/>
            <arg name="base_frame"       value="$(arg base_frame)"/>
            <arg name="odom_frame"       value="$(arg odom_frame)"/>
        </include>
    </group>
```

* **Parameter Setting File:**
  In the ros_ws workspace of the robot, locate the "**calibrate_params.yaml**" file at the following path: **hiwonder_driver\hiwonder_controller\config\calibrate_params.yaml**
  The screenshot below illustrates the control factor parameters for Mecanum wheel control:
  
  

<img class="common_img" src="../_static/media/3/section_137_1. Kinematics Analysis/media/image25.png" style="width:500px" />

The go_factor is configured as 1.0, acting as an adjustment factor for governing the robot's forward speed. It maintains a direct proportionality to the forward speed command dispatched by the robot control program. For instance, if go_factor is 1.0 and the control program issues a forward speed command of 0.5 m/s, the resulting actual forward speed executed by the robot will be:

Actual speed = go_factor × Commanded speed = 1.0 × 0.5 m/s = 0.5 m/s

In essence, the robot moves forward precisely in accordance with the speed command sent by the control program.

Similarly, the turn_factor parameter serves as an adjustment factor for controlling the robot's rotational speed. With turn_factor set to 1.0, it maintains a direct proportionality to the rotation speed command conveyed by the robot control program. For instance, if the control program transmits a rotation speed command of 1 rad/s and turn_factor is set to 1.0, the actual rotation speed executed by the robot will be:

Actual speed = turn_factor × Commanded speed = 1.0 × 1 rad/s = 1 rad/s

Hence, when turn_factor is set to 1.0, the robot directly executes the rotation speed command dispatched by the control program without any adjustments. This capability stems from the excellent linear and angular motion capabilities of Mecanum wheels, allowing each wheel to independently steer and drive, thus providing exceptional motion flexibility. As a result, in both linear and angular motion control, the robot follows the control commands directly, eliminating the need for additional parameter adjustments and ensuring precise motion functionality.

**3. Ackermann Chassis**
**(1) Hardware Component**

The transmission mechanism of the Ackermann chassis front wheels includes a servo, linkage, and wheels. The servo is linked to the linkage, and the linkage is connected to the wheels. The servo's rotation governs the extent of the linkage's rotation, thereby affecting the steering of the front wheels.

During a turn, the front two wheels are in a parallel state, with both wheels having the same angles of rotation. The control of the rear wheels is managed by the motor and wheels, where the motor's rotation determines the robot's forward, backward, and speed movements.

<img class="common_img" src="../_static/media/3/section_137_1. Kinematics Analysis/media/image37.png" style="width:500px" />

<img class="common_img" src="../_static/media/3/section_137_1. Kinematics Analysis/media/image38.png" style="width:500px" />

**(2) Physical Characteristics**

The design objective of the Ackermann chassis is to provide excellent steering performance and stability. It employs a principle called "**Ackermann geometry**" to achieve this. Ackermann geometry refers to the difference in steering angles between the front and rear wheels. By allowing the inner front wheel to have a greater steering angle, the Ackermann chassis makes it easier to control the vehicle during turns and reduces the risk of sliding during steering.

Additionally, the Ackermann chassis features a well-designed suspension system. The suspension system is a crucial component connecting the wheels and the vehicle body, significantly influencing the physical characteristics of the chassis. Ackermann chassis typically adopts an independent suspension system, enabling the independent control of each wheel's movement. This design enhances suspension performance, improving vehicle stability and ride comfort.

Furthermore, the Ackermann chassis considers the position of the vehicle's center of gravity. The center of gravity's location has a significant impact on the vehicle's stability and handling performance. Generally, the Ackermann chassis places the center of gravity lower to reduce the risk of vehicle tilt and sliding.

Lastly, the physical characteristics of the Ackermann chassis include the braking system and power transmission system. The braking system influences the vehicle's braking performance and stability by controlling the braking force on the wheels. The power transmission system is responsible for transferring the engine's power to the wheels, affecting the vehicle's acceleration and driving performance.

**(3) Kinematics Principle and Formula**

When conducting kinematic analysis of the Ackermann chassis, we can use the following mathematical formulas and parameters to describe its motion characteristics:

To achieve pure rolling motion for the Ackermann car (meaning no side slip during turns), it is necessary to ensure that the normals of the four wheels' motion directions (lines perpendicular to the direction of tire rolling) intersect at a single point, which is the center of rotation.

To simplify the model, let's assume that the front wheels have only one wheel (the theoretical concept remains consistent) located in the middle position of the front axle, as depicted by the dashed line in the diagram:

<img class="common_img" src="../_static/media/3/section_137_1. Kinematics Analysis/media/image39.png" style="width:500px" />

1.  **Front Wheel Steering Angle (θ)**: The rotation angle of the front wheels, indicating the angle by which the front wheels deviate from the vehicle's forward direction. It is typically measured in radians (rad).

2.  **Vehicle Linear Velocity (V):** The overall linear speed of the vehicle, representing its translational velocity. It is usually measured in meters per second (m/s). The left rear wheel speed is denoted as (VL), and the right rear wheel speed is denoted as (VR).

3.  **Vehicle Track Width (D):** The distance between the wheels on the left and right sides of the vehicle, measured in meters (m).

4.  **Wheelbase of the Vehicle (H)**: The distance between the front and rear wheels of the vehicle, measured in meters (m)

5.  **Vehicle Turning Radius (R):** The radius of the circle described by the vehicle during a turn, measured in meters (m). The turning radius for the left wheel is (RL), and for the right wheel is (RR).

Process for Calculating Robot Speed and Angle:

6.  Consistency of angular velocity:

<img class="common_img" src="../_static/media/3/section_137_1. Kinematics Analysis/media/image26.png" style="width:500px" />

In this context:

ω represents the angular velocity of the vehicle.

R denotes the turning radius of the vehicle.

V is the linear velocity of the vehicle.

VL is the linear velocity of the left rear wheel.

VR is the linear velocity of the right rear wheel.

RL is the turning radius of the left wheel.

RR is the turning radius of the right wheel.

7.  The relationship between the front wheel steering angle and the turning radius of the vehicle:

<img class="common_img" src="../_static/media/3/section_137_1. Kinematics Analysis/media/image27.png" style="width:500px" />

H represents the distance between the front and rear wheels of the vehicle.

R signifies the turning radius of the vehicle.

D denotes the distance between the wheels on the left and right sides of the vehicle.

θ indicates the steering angle of the front wheels.

8.  The speeds of the left and right wheels of the robot can be determined as:

<img class="common_img" src="../_static/media/3/section_137_1. Kinematics Analysis/media/image28.png" style="width:500px" />

9.  The speeds of the left and right wheels of the robot can be determined as:

<img class="common_img" src="../_static/media/3/section_137_1. Kinematics Analysis/media/image29.png" style="width:500px" />

By knowing the wheelbase, track width, robot speed, and the steering angle of the servo, it is possible to calculate the speeds of the two rear wheels of the robot.

* **Program Outcome**

The kinematic analysis and fundamental chassis control code for the robot are consolidated and organized within the script files and configuration files located in the **'hiwonder_driver/hiwonder_controller**' path.

> [!NOTE]
>
> Note: In the following code section, the '**machine_type**' specifies the chassis type as '**JetRover_Acker**' for the Ackermann chassis. The remaining launch files and configuration files are generally similar. Readers can refer to them for a more comprehensive understanding.

```py
    def load_calibrate_param(self, msg):
        if self.machine_type == 'JetRover_Mecanum':
            self.linear_factor = rospy.get_param('~mecanum/linear_correction_factor', 1.00)
            self.angular_factor = rospy.get_param('~mecanum/angular_correction_factor', 1.00)
        elif self.machine_type == 'JetRover_Tank':
            self.linear_factor = rospy.get_param('~tank/linear_correction_factor', 1.00)
            self.angular_factor = rospy.get_param('~tank/angular_correction_factor', 1.00)
        elif self.machine_type == 'JetRover_Acker':
            self.linear_factor = rospy.get_param('~acker/linear_correction_factor', 1.00)
            self.angular_factor = rospy.get_param('~acker/angular_correction_factor', 1.00)
        
        return [True, 'load_calibrate_param']
```

### 3.1.3 Tank Chassis

* **Hardware Structure**

The track is a flexible chain loop composed of a drive wheel, surrounded by idler wheels, load wheels, guide wheels, and carrier wheels. It consists of track plates and track pins.

<img class="common_img" src="../_static/media/3/section_137_1. Kinematics Analysis/media/image49.png" style="width:500px" />

<img class="common_img" src="../_static/media/3/section_137_1. Kinematics Analysis/media/image50.png" style="width:500px"  />

* **Physical Characteristic**

<img class="common_img" src="../_static/media/3/section_137_1. Kinematics Analysis/media/image51.png" style="width:500px"  />

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

<img class="common_img" src="../_static/media/3/section_137_1. Kinematics Analysis/media/image40.png" style="width:500px"  />

1.**B：**Track Width, the distance between the two drive wheels on the track, usually measured in meters (m)

2\. **R：**Turning Radius (R) generated when the robot simultaneously moves forward and rotates, usually measured in meters (m)

3\. 𝑉𝑥：Target forward/backward velocity of the robot at point O, positive for forward movement, typically measured in meters per second (m/s)

4\. 𝑉𝑦：Target left/right velocity of the robot at point O, positive for leftward movement, measured in m/s (ignored for two-wheel configurations)

5\. 𝑉𝑤：Target rotational velocity of the robot around point O, positive for counterclockwise rotation, measured in rad/s

6.<img src="../_static/media/3/section_137_1. Kinematics Analysis/media/image41.png" style="width:50px" />**：**Left wheel velocity of the robot, positive in the forward direction, typically measured in meters per second (m/s)

7\. <img src="../_static/media/3/section_137_1. Kinematics Analysis/media/image42.png" style="width:50px" />**：**Right wheel velocity of the robot, positive in the forward direction, typically measured in meters per second (m/s)

8\. <img src="../_static/media/3/section_137_1. Kinematics Analysis/media/image43.png" style="width:50px" />：Path covered by the left wheel in a certain time t

<img src="../_static/media/3/section_137_1. Kinematics Analysis/media/image44.png" style="width:50px" />：Path covered by the midpoint O in a certain time t

<img src="../_static/media/3/section_137_1. Kinematics Analysis/media/image45.png" style="width:50px" />：Path covered by the right wheel in a certain time t

9\.<img src="../_static/media/3/section_137_1. Kinematics Analysis/media/image53.png" style="width:50px" /> ：Angle of rotation of the robot in a certain time measured in radians (rad)

**Formula Calculation:**

1.  Next, we will explore their relationships and derive the kinematic forward and inverse formulas for the tracked chassis (two-wheel differential drive type). By integrating velocity with respect to time, we can obtain the displacement:

<img class="common_img" src="../_static/media/3/section_137_1. Kinematics Analysis/media/image54.png" style="width:500px" />

2.  Dividing the arc length by the radius equals the angle in radians

<img class="common_img" src="../_static/media/3/section_137_1. Kinematics Analysis/media/image55.png" style="width:500px" />

3.  Dividing both sides of the formula by 't,' i.e., integrating with respect to time, yields.

<img class="common_img" src="../_static/media/3/section_137_1. Kinematics Analysis/media/image56.png" style="width:500px" />

4.  To solve the inverse kinematics equation based on the above formula, the objective is to determine the drive wheel velocities <img src="../_static/media/3/section_137_1. Kinematics Analysis/media/image41.png" style="width:50px" /> and <img src="../_static/media/3/section_137_1. Kinematics Analysis/media/image59.png" style="width:50px" />, given the robot's target velocity <img src="../_static/media/3/section_137_1. Kinematics Analysis/media/image9.png" style="width:50px" />and <img src="../_static/media/3/section_137_1. Kinematics Analysis/media/image60.png" style="width:50px" /> .

<img class="common_img" src="../_static/media/3/section_137_1. Kinematics Analysis/media/image57.png" style="width:500px" />

5.  By using the inverse kinematics equation, we can solve the forward kinematics equation. If the current velocities of the drive wheels <img src="../_static/media/3/section_137_1. Kinematics Analysis/media/image41.png" style="width:50px" />and <img src="../_static/media/3/section_137_1. Kinematics Analysis/media/image59.png" style="width:50px" /> are known, we can calculate the real-time velocity of the robot.

<img class="common_img" src="../_static/media/3/section_137_1. Kinematics Analysis/media/image58.png" style="width:500px" />

* **Program Outcome**

The kinematic analysis and fundamental chassis control code for the robot are consolidated and organized within the script files and configuration files located in the '**hiwonder_driver/hiwonder_controller**' path.

> [!NOTE]
>
> Note: In the code section below, the chassis type specified by '**machine_type**' is '**JetRover_Tank**' for the tank chassis. The remaining launch files and configuration files are generally similar. Readers can refer to them for further understanding.

```py
    def load_calibrate_param(self, msg):
        if self.machine_type == 'JetRover_Mecanum':
            self.linear_factor = rospy.get_param('~mecanum/linear_correction_factor', 1.00)
            self.angular_factor = rospy.get_param('~mecanum/angular_correction_factor', 1.00)
        elif self.machine_type == 'JetRover_Tank':
            self.linear_factor = rospy.get_param('~tank/linear_correction_factor', 1.00)
            self.angular_factor = rospy.get_param('~tank/angular_correction_factor', 1.00)
        elif self.machine_type == 'JetRover_Acker':
            self.linear_factor = rospy.get_param('~acker/linear_correction_factor', 1.00)
            self.angular_factor = rospy.get_param('~acker/angular_correction_factor', 1.00)
        
        return [True, 'load_calibrate_param']
```

## 3.2 Motion Control

### 3.2.1 IMU, Linear Velocity and Angular Velocity Calibration

> [!NOTE]
>
> **Note:**
>
> 1.  **The robot has been calibrated before leaving the factory and does not require additional calibration. The information is provided for reference only. If you observe significant deviations during robot movement, such as noticeable drifting to one side when moving forward or an inability to travel straight, you can consult the following tutorial for calibration.**
>
> 2.  **Calibration aims to minimize deviations, but actual hardware variations are inherent. Hence, adjust the calibration to a level that reasonably suits your requirements.**

If the robot exhibits deviations during operation, it may require IMU calibration. Once the calibration process is completed, the robot can resume normal operation.

* **IMU Calibration**

IMU (Inertial Measurement Unit) is a device that measures the three-axis attitude angles (angular velocity) and acceleration of an object. The gyroscope and accelerometer are the main components of the IMU, providing a total of 6 degrees of freedom to measure the angular velocity and acceleration of the object in three-dimensional space. Upon receiving the first IMU message, the node will prompt you to maintain the IMU in a specific orientation and press Enter to record the measurement values. After completing measurements in all 6 directions, the node will calculate calibration parameters and write them to the specified YAML file. The specific steps are as follows:

> [!NOTE]
>
> **Note: The input command is case-sensitive, and keywords can be completed using the Tab key.**

1. Start the robot, and access the robot system desktop using NoMachine.

2. Click-on <img class="common_img" src="../_static/media/3/section_1/media/image4.png" style="width:50px" /> to open the command-line terminal.

3. Execute the command ‘**sudo systemctl stop start_app_node.service**’ to disable the app auto-start service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

4. Run the command ‘**roslaunch hiwonder_calibration calibrate_imu.launch**’ to start IMU calibration.

   ```py
   roslaunch hiwonder_calibration calibrate_imu.launch
   ```

5. When prompted, align the robot with its front side and press Enter. The initial orientation is considered as forward, and subsequent placements should follow this initial direction.

   <img class="common_img" src="../_static/media/3/section_1/media/image7.png" style="width:500px" />

   <img class="common_img" src="../_static/media/3/section_1/media/image8.png" style="width:500px"  />

   After you successfully calibrate all direction, the following prompt will appear.

   <img class="common_img" src="../_static/media/3/section_1/media/image9.png" style="width:500px" />

6. Align the robot to the rear, then press Enter.

   <img class="common_img" src="../_static/media/3/section_1/media/image10.png" style="width:500px" />

   <img class="common_img" src="../_static/media/3/section_1/media/image11.png" style="width:500px"  />

7. Align the robot to the left, then hit Enter.

   <img class="common_img" src="../_static/media/3/section_1/media/image12.png" style="width:500px" />

   <img class="common_img" src="../_static/media/3/section_1/media/image13.png" style="width:500px"  />

8. Align the robot to the right, then press Enter.

   <img class="common_img" src="../_static/media/3/section_1/media/image14.png" style="width:500px" />

   <img class="common_img" src="../_static/media/3/section_1/media/image15.png" style="width:500px"  />

9. Lift the robot, place it facing upwards, and press Enter. When positioning vertically, be careful to avoid instability or collisions. Use your hand for support to prevent damage to the depth camera or screen.

   <img class="common_img" src="../_static/media/3/section_1/media/image16.png" style="width:500px" />

   <img class="common_img" src="../_static/media/3/section_1/media/image17.png" style="width:500px"  />

10. Place the robot as pictured, then hit Enter.

    <img class="common_img" src="../_static/media/3/section_1/media/image18.png" style="width:500px" />

    <img class="common_img" src="../_static/media/3/section_1/media/image19.png" style="width:500px"  />

11. If the below prompt shows up, it means the calibration is complete. To exit, use short-cut ‘**ctrl+c**’.

    <img class="common_img" src="../_static/media/3/section_1/media/image20.png" style="width:500px" />

12. After calibration, execute the command '**roslaunch hiwonder_peripherals imu.launch debug:=true**' to verify the calibrated model.

    <img class="common_img" src="../_static/media/3/section_1/media/image21.png" style="width:500px" />

    <img class="common_img" src="../_static/media/3/section_1/media/image22.png" style="width:500px" />

* **Angular Velocity Calibration**

To calibrate the angular velocity, the robot needs to perform a full rotation independently. During testing, it's crucial to mark the robot's orientation to facilitate the observation of any deviations. The specific steps are outlined below:

> [!NOTE]
>
> **Note: The input command is case-sensitive, and keywords can be completed using the Tab key.**

1. Place the robot on a flat surface and place a piece of tape or other marker in front of the robot's center.

2. Start the robot, and connect it to the robot system desktop using NoMachine.

3. Click-on <img src="../_static/media/3/section_1/media/image4.png" style="width:50px" /> to open the command-line terminal.

4. Run the command ‘**sudo systemctl stop start_app_node.service**’ and hit Enter to disable the app auto-start service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

5. Run the command '**roslaunch hiwonder_calibration calibrate_angular.launch turn:=true**' and press Enter to start adjusting the angular velocity using the "turn" parameter.

   ```py
   roslaunch hiwonder_calibration calibrate_angular.launch turn:=true
   ```

Click on "**calibrate_angular**" on the left side. The calibration interface will appear as shown below.

<img class="common_img" src="../_static/media/3/section_1/media/image24.png" style="width:500px" />

The parameters on the left side of the interface are defined as follows:

The first parameter, "**test_angle**," represents the test rotation angle, with a default value of 360°.

The second parameter, "**speed,**" represents the linear velocity with a default value of 0.15 meters per second.

The third parameter, "**tolerance**," represents the error value. A smaller error value results in more significant robot oscillations after reaching the target position.

The fourth parameter, "**motor_turn_scale_correction,**" represents the motor rotation scale correction.

The fifth parameter, "**odom_angle_scale_correction**," represents the odometry angle scale correction.

The sixth parameter, "**start_test_turn,**" is the button to start testing the motor rotation scale correction.

The seventh parameter, "**start_test_angular,**" is the button to start testing the odometry angle scale correction.

<img class="common_img" src="../_static/media/3/section_1/media/image25.png" style="width:500px" />

Ensure the robot is properly aligned, with the marker placed in front of it. Check the "**start_test_turn**" option and the robot will rotate in place. If it fails to complete a full rotation, you need to adjust the "**motor_turn_scale_correction**" value, which controls the motor's rotation scale. It is recommended to adjust this value in increments of 0.01.

<img class="common_img" src="../_static/media/3/section_1/media/image26.png" style="width:500px" />

6. Open a new command-line terminal, and execute the command ‘**roscd hiwonder_controller/config/**’ to navigate to the directory containing calibration configuration files.

   ```py
   roscd hiwonder_controller/config/
   ```

7. Execute the command ‘**vim calibrate_params.yaml**’ to open the configuration file.

   ```py
   vim calibrate_params.yaml
   ```

8. Press ‘**I**’ key to navigate to the editing mode, and modify the value of "**turn_factor**" to the adjusted value of "**motor_turn_scale_correction**"

   <img class="common_img" src="../_static/media/3/section_1/media/image29.png" style="width:500px" />

> [!NOTE]
>
> **Note: The aforementioned operations are conducted on the Mecanum-wheel version and are equally applicable to the tank chassis version.**

9)  After modification, press the "**ESC**" key, enter "**:wq**" to exit and save the changes.

* **Linear Velocity Calibration**

> [!NOTE]
>
> **Note: The input command is case-sensitive, and keywords can be completed using the Tab key.**

Position the robot on a flat and open surface. Mark the starting point with tape or any other indicator in front of the robot, and position the endpoint tape or another marker 1 meter ahead of the robot.

1. Start the robot, and access the robot system desktop using NoMachine.

2. Click-on <img src="../_static/media/3/section_1/media/image4.png" style="width:50px" /> to open the command-line terminal.

3. Execute the command ‘**sudo systemctl stop start_app_node.service**’ and hit Enter to disable the app auto-start service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

4. Execute the command '**roslaunch hiwonder_calibration calibrate_linear.launch go:=true**' and press Enter to activate the adjustment of the linear velocity using the '**go**' parameter.

   ```py
   roslaunch hiwonder_calibration calibrate_linear.launch go:=true
   ```

Click on "**calibrate_linear**" on the left side, and the calibration interface will appear as follows.

<img class="common_img" src="../_static/media/3/section_1/media/image32.png" style="width:500px" />

The parameters on the left side of the interface have the following meanings:

The first parameter "**test_distance**" is the test distance, with a default value of 1 meter.

The second parameter "**speed**" is the linear velocity, with a default value of 0.15 meters per second.

The third parameter "**tolerance**" is the error value. The smaller the error value, the greater the robot's shaking amplitude after reaching the target position.

The fifth parameter "**motor_go_scale_correction**" is the motor forward scaling factor.

The fourth parameter "**odom_linear_scale_correction**" is the odometer linear scaling factor.

The sixth parameter "**start_test_go**" is the button to start testing the motor forward scaling factor.

The seventh parameter "**start_test_linear**" is the button to start testing the odometer linear scaling factor.

<img class="common_img" src="../_static/media/3/section_1/media/image33.png" style="width:500px" />

Ensure that the robot is aligned and positioned at the starting marker. Check the "**start_test_go**" option and the robot will move forward. Observe if the robot can travel in a straight line. If there is any deviation, you need to adjust the value of "**motor_go_scale_correction**," which is the scaling factor for the motor's forward movement. It is recommended to adjust this value by 0.01 each time.

<img class="common_img" src="../_static/media/3/section_1/media/image34.png" style="width:500px" />

5. Open a new command line terminal and enter the command "**roscd hiwonder_controller/config/**" to navigate to the directory containing the configuration file.

   ```py
   roscd hiwonder_controller/config/
   ```

6. Enter the command "**vim calibrate_params.yaml**" to open the configuration file.

   ```py
   vim calibrate_params.yaml
   ```

7. Press the "**I**" key to enter edit mode and modify the value of "**go_factor**" to the adjusted value of "**motor_go_scale_correction**."

   <img class="common_img" src="../_static/media/3/section_1/media/image36.png" style="width:500px" />

8. After making the modifications, press the "ESC" key, enter ":wq" to exit and save the changes.

9. If you need to terminate this program, use short-cut ‘**Ctrl+C**’.

10. After calibration, you can activate the mobile app service either by using a command or restarting the robot. If the mobile app service is not activated, related app functions will be disabled. In the case of a robot restart, the mobile app service will start automatically. Use the command '**sudo systemctl restart start_app_node.service**' to restart the mobile app service, and wait for the buzzer to make sound, signaling the completion of the service startup.

    ```py
    sudo systemctl restart start_app_node.service
    ```

* **FAQ**

Q: What should I do if the robot shows movement deviation even after calibration, especially when walking in a straight line?

A: For instance, when dealing with the adjustment of linear velocity and encountering deviations, we suggest performing multiple tests. Adjust the associated parameters, specifically '**motor_go_scale_correction**' and '**odom_linear_scale_correction**,' until the robot achieves stable straight-line movement. This method helps prevent incomplete calibration by modifying only one parameter.

### 3.2.2 Publish IMU and Odometer Data

In robot navigation, accurately calculating real-time position is essential. Normally, we obtain odometer information using motor encoders and the robot's kinematic model. However, in specific situations, like when the robot's wheels rotate in place or when the robot is lifted, it may move a distance without the wheels actually turning.

To address wheel slip or accumulated errors in such cases, combining IMU and odometer data can yield more precise odometer information. This improves mapping and navigation accuracy in scenarios where wheel slip or cumulative errors may occur.

* **Introduction to IMU and Odometer**

The IMU (Inertial Measurement Unit) is a device that measures the three-axis attitude angles (angular velocity) and acceleration of an object. It consists of the gyroscope and accelerometer as its main components, providing a total of 6 degrees of freedom to measure the object's angular velocity and acceleration in three-dimensional space.

An odometer is a method used to estimate changes in an object's position over time using data obtained from motion sensors. This method is widely applied in robotic systems to estimate the distance traveled by the robot relative to its initial position.

There are common methods for odometer positioning, including the wheel odometer, visual odometer, and visual-inertial odometer. In robotics, we specifically use the wheel odometer. To illustrate the principle of the wheel odometer, consider a carriage where you want to determine the distance from point A to point B. By knowing the circumference of the carriage wheels and installing a device to count wheel revolutions, you can calculate the distance based on wheel circumference, time taken, and the number of wheel revolutions.

While the wheel odometer provides basic pose estimation for wheeled robots, it has a significant drawback: accumulated error. In addition to inherent hardware errors, environmental factors such as slippery tires due to weather conditions contribute to increasing odometer errors with the robot's movement distance.

Therefore, both IMU and odometer are essential components in a robot. These two components are utilized to measure the three-axis attitude angles (or angular velocity) and acceleration of the object, as well as to estimate the distance, pose, velocity, and direction of the robot relative to its initial position.

To address these errors, we combine IMU data with odometer data to obtain more accurate information. IMU data is published through the "**/imu**" topic, and odometer data is published through "**/odom**". After obtaining data from both sources, the data is fused using the "**ekf**" package in ROS, and the fused localization information is then republished.

* **IMU Data Publishing**

 **1. Initiate Service**

> [!NOTE]
>
> **Note: When entering commands, it is essential to strictly distinguish between uppercase and lowercase letters, and keywords can be autocompleted using Tab key.**

1. Start the robot, and access the robot system desktop using NoMachine.

2. Click-on <img src="../_static/media/3/section_1/media/image4.png" style="width:50px" /> to open the command line terminal.

3. Execute the command ‘**sudo systemctl stop start_app_node.service**’, and hit Enter to disable the app auto-start service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

4. Run the command ‘**roslaunch hiwonder_peripherals imu.launch**’ and press Enter to publish the IMU data.

   ```py
   roslaunch hiwonder_peripherals imu.launch
   ```

After calibration, you can activate the mobile app service either by using a command or restarting the robot. If the mobile app service is not activated, related app functions will be disabled. In the case of a robot restart, the mobile app service will start automatically. Use the command '**sudo systemctl restart start_app_node.service**' to restart the mobile app service, and wait for the buzzer to make sound, signaling the completion of the service startup.

```py
sudo systemctl restart start_app_node.service
```

**2. Data Viewing**

1)  Open a new command line terminal, and execute the command ‘**rostopic list**’ to check the current topic.

<img class="common_img" src="../_static/media/3/section_1/media/image41.png" style="width:500px" />

2)  Run the command ‘**rostopic info /imu**’ to check the type, publisher and subscriber of ‘**/imu**’ topic.

<img class="common_img" src="../_static/media/3/section_1/media/image42.png" style="width:500px" />

The topic is of type '**sensor_msgs/Imu**,' published under '**/imu_filter**,' and currently has no subscribers.

3)  Use the command '**rostopic echo /imu**' to display the content of the topic message. Feel free to replace '**imu**' with the name of the topic you wish to view.

<img class="common_img" src="../_static/media/3/section_1/media/image43.png" style="width:500px" />

<img class="common_img" src="../_static/media/3/section_1/media/image44.png" style="width:500px" />

The terminal will display the data from the three axes of the IMU.

* **Odometer Data Publishing**

**1. Initiate Service**

> [!NOTE]
>
> **Note: When entering commands, it is essential to strictly distinguish between uppercase and lowercase letters, and keywords can be autocompleted using Tab key.**

1. Start the robot, and access the robot system desktop using NoMachine.

2. Double-click <img src="../_static/media/3/section_1/media/image45.png" style="width:50px" /> to open the command line terminal.

3. Execute the command ‘**sudo systemctl stop start_app_node.service**’, and hit Enter to disable app auto-start service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

4. Run the command ‘**roslaunch hiwonder_controller hiwonder_controller.launch**’ to publish the odometer data.

   ```py
   roslaunch hiwonder_controller hiwonder_controller.launch
   ```

After calibration, you can activate the mobile app service either by using a command or restarting the robot. If the mobile app service is not activated, related app functions will be disabled. In the case of a robot restart, the mobile app service will start automatically. Use the command '**sudo systemctl restart start_app_node.service**' to restart the mobile app service, and wait for the buzzer to make sound, signaling the completion of the service startup.

```py
sudo systemctl restart start_app_node.service
```

**1. Data Viewing**

1)  Open a new command line terminal, and run the command ‘**rostopic list**’ to check the current topic.

<img class="common_img" src="../_static/media/3/section_1/media/image47.png" style="width:500px" />

2)  Enter the command ‘**rostopic info /odom_raw**’ to check the type, publisher and subscriber of the ‘**/odom_raw**’ topic.

<img class="common_img" src="../_static/media/3/section_1/media/image48.png" style="width:500px" />

The type of this topic is '**nav_msgs/Odometry**,' published by '**/hiwonder_odom_publisher**,' and subscribed to by '**/ekf_localization**'.

3. Execute the command '**rostopic echo /odom_raw**' to display the content of the topic message. Feel free to substitute the topic name with the one you wish to observe.

   ```py
   rostopic echo /odom_raw
   ```

<img class="common_img" src="../_static/media/3/section_1/media/image50.png" style="width:500px" />

The message comprises acquired pose and velocity twist data.

### 3.2.3 Robot Speed Control

Speed control is achieved by adjusting the linear velocity parameter.

* **Program Logic**

Utilizing the mobility features of the robot, we accomplish forward and backward motion, as well as turning, by manipulating the active wheel's forward and reverse rotation.

Within the program, the movement control node of the **hiwonder_controller** is subscribed to, allowing the retrieval of the configured linear and angular velocities. Following this, an analysis and calculation are executed based on these parameters to ascertain the robot's movement speed.

The source code for this program can be found at:

**/home/ros_ws/src/hiwonder_driver/hiwonder_controller/scripts/odom_publisher.py**

```py
        self.x = 0
        self.y = 0
        self.linear_x = 0
        self.linear_y = 0
        self.angular_z = 0
        self.pose_yaw = 0
        self.last_time = None
        self.current_time = None
        self.motor_pub = rospy.Publisher('ros_robot_controller/set_motor', MotorsState, queue_size=1)
        self.servo_state_pub = rospy.Publisher('ros_robot_controller/bus_servo/set_state', SetBusServoState, queue_size=10)
        self.ackermann = AckermannChassis(wheelbase=0.206, track_width=0.194, wheel_diameter=0.0965)  
        self.mecanum = MecanumChassis(wheelbase=0.206, track_width=0.194, wheel_diameter=0.0965)
```

* **Disable APP Service and Initiate Speed Control**

> [!NOTE]
>
> **Note: When entering commands, it is essential to strictly distinguish between uppercase and lowercase letters, and keywords can be autocompleted using Tab key.**

1. Start the robot, and access the robot system desktop using NoMachine.

2. Click-on <img src="../_static/media/3/section_1/media/image52.png" style="width:50px" /> to open the command-line terminal.

3. Execute the command ‘**sudo systemctl stop start_app_node.service**’ and hit Enter to disable the app auto-start service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

4. Run the command ‘**roslaunch hiwonder_controller hiwonder_controller.launch**’ to launch the motion control service.

   ```py
   roslaunch hiwonder_controller hiwonder_controller.launch
   ```

5. Open a new terminal, and enter the command ‘**rostopic pub /hiwonder_controller/cmd_vel geometry_msgs/Twist "linear:**’, then press ‘**TAB**’ key to autocomplement the parameters.

   ```py
   rostopic pub /hiwonder_controller/cmd_vel geometry_msgs/Twist "linear:
   ```

   <img class="common_img" src="../_static/media/3/section_1/media/image54.png" style="width:500px" />

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

<img class="common_img" src="../_static/media/3/section_1/media/image55.png" style="width:500px" />

6. To bring the robot car to a stop, open a new terminal and set the linear velocity to '0.0'.

   <img class="common_img" src="../_static/media/3/section_1/media/image54.png" style="width:500px" />

7. If you need to terminate this program, use short-cut ‘Ctrl+C’.

   > [!NOTE]
   >
   > **Note: To bring the robot car to a stop, please create a new terminal and adjust the linear velocity. Using the 'Ctrl+C' shortcut alone may not effectively halt the robot car.**

After calibration, you can activate the mobile app service either by using a command or restarting the robot. If the mobile app service is not activated, related app functions will be disabled. In the case of a robot restart, the mobile app service will start automatically. Use the command '**sudo systemctl restart start_app_node.service**' to restart the mobile app service, and wait for the buzzer to make sound, signaling the completion of the service startup.

```py
sudo systemctl restart start_app_node.service
```

* **Change Forward Speed**

By modifying the linear velocity value (X), the robot can achieve forward movement at variable speeds. For instance, to make the robot shift diagonally to the left front, during step 5 in Disable App Service and Initiate Speed Control, set X to 'X: 0.3'.

<img class="common_img" src="../_static/media/3/section_1/media/image56.png" style="width:500px" />

Upon pressing Enter, the robot will move forward at a speed of 0.3 meters per second in a straight direction.

* **Program Outcome**

After the game starts, the robot will go forward at the speed of 0.3m/s.

* **Program Analysis**

**1. launch File**

The launch file is saved in

**/home/ros_ws/src/hiwonder_driver/hiwonder_controller/launch/hiwonder_controller.launch**

**(1) Parameter Definition**

In a launch file, the **\<arg\>** tag serves as a parameter label, declaring a parameter. The subsequent '**\<default\>**' provides a specific definition for this parameter.

```py
    <arg name="cmd_vel"           default="hiwonder_controller/cmd_vel"/>
```

**(2) Initiate Node**

The `<node>` tag represents a node launched in the file, and the subsequent `<name>`, `<pkg>`, `<type>`, `<required>`, `<output>`, etc., are parameters within that node.

`<name>` is the name of the node.

`<pkg>` is the package in which the node is located.

`<type>` is the name of the file that the node executes; in this case, it runs the content of the **odom_publisher.py** file.

`<required>` determines whether the node is essential. If set to `True`, the node will close along with other relevant nodes when deactivated.

`<output>` indicates the destination of the output; here, the output is directed to the screen.

```py
    <!--odom发布-->
    <node name="hiwonder_odom_publisher" pkg="hiwonder_controller" type="odom_publisher.py" required="true" output="screen">
        <rosparam file="$(find hiwonder_controller)/config/calibrate_params.yaml" command="load"/>
        <param name="freq"              value="$(arg freq)"/>
        <param name="odom_topic"        value="$(arg odom_raw_topic)"/>
        <param name="base_frame_id"     value="$(arg base_frame)"/>
        <param name="odom_frame_id"     value="$(arg odom_frame)"/>
        <param name="cmd_vel"           value="$(arg cmd_vel)"/>
    </node>
```

**2. Python Program**

**(1) Published Topic**

`board/set_motor`: Publishes the robot 's motor status. Message type is MotorsState.

`ros_robot_controller/bus_servo/set_state`: Publishes the state of the bus servos. Message type is SetBusServoState.

```py
        self.motor_pub = rospy.Publisher('ros_robot_controller/set_motor', MotorsState, queue_size=1)
        self.servo_state_pub = rospy.Publisher('ros_robot_controller/bus_servo/set_state', SetBusServoState, queue_size=10)
```

If `self.pub_odom_topic` is true, it will also publish:

```py
if self.pub_odom_topic:
```

`odom_raw` (or another name set through parameters): Publishes the robot's odometry data. Message type is Odometry.

```py
self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
```

`set_pose`: Publishes pose information with covariance. Message type is PoseWithCovarianceStamped.

```py
self.pose_pub = rospy.Publisher('set_pose', PoseWithCovarianceStamped, queue_size=1)
```

**(2) Subscribe to Motion Control Node**

Subscribe to topic messages published by the '**hiwonder_controller**' node to retrieve pertinent movement data.

'**set_odom**': Listens for messages to set odometry. Message type is Pose2D. Upon receiving a message of this type, it activates the 'set_odom' method.

'**/hiwonder_controller/cmd_vel**' (or an alternative name set through parameters): Listens for velocity commands. Message type is Twist. When a message of this type is received, it activates the `cmd_vel_callback` method.

'**cmd_vel**': Listens for velocity commands initiated by the application. Message type is Twist. Upon receiving a message of this type, it activates the `app_cmd_vel_callback` method.

```py
        rospy.Subscriber('set_odom', Pose2D, self.set_odom)
        rospy.Subscriber(self.cmd_vel, Twist, self.cmd_vel_callback)
        rospy.Subscriber('cmd_vel', Twist, self.app_cmd_vel_callback)
```

**(3) Service**

This node additionally offers a ROS service:

'**hiwonder_controller/load_calibrate_param**': The service type is Trigger, allowing activation of the `load_calibrate_param` method.

```py
rospy.Service('hiwonder_controller/load_calibrate_param', Trigger, self.load_calibrate_param)
```

**(4) Load Parameter**

Load the speed parameter of the command.

```py
            self.linear_factor = rospy.get_param('~acker/linear_correction_factor', 1.00)
            self.angular_factor = rospy.get_param('~acker/angular_correction_factor', 1.00)
```

```py
def cmd_vel_callback(self, msg):
```

```py
elif self.machine_type == 'JetRover_Acker':
            if msg.angular.z != 0:
                r = self.linear_x/msg.angular.z
                if r == 0:
                    self.angular_z = 0
                else:
                    self.angular_z = msg.angular.z
            else:
                self.angular_z = 0
            servo_state = BusServoState()
            servo_state.present_id = [1, 9]
            speeds = self.ackermann.set_velocity(self.linear_x, self.angular_z)
            self.motor_pub.publish(speeds[1])

            if speeds[0] is not None:
                servo_state.position = [1, speeds[0]]
                data = SetBusServoState()
                data.state = [servo_state]
                data.duration = 0.02
                self.servo_state_pub.publish(data)
```

To regulate forward and backward speed, only the `X` value is essential for the `linear` parameter. Considering the robot's perspective as the first-person view, the positive direction of `X` corresponds to forward movement. The unit is meters per second, and it is advisable to maintain the value within the range of "-0.6 to 0.6".

As for angular speed, solely the `Z` value is necessary for the `angular` parameter. A positive `Z` induces a left turn, while a negative `Z` prompts the robot to turn right. The unit is radians per second, and this value is calculated using the linear velocity-angular velocity formula and trigonometric functions.

**(5) Control Motor Rotation and Parameter Transmission**

By analyzing and calculating velocity, the necessary parameters for motor rotation are determined. The `set_velocity` function is then utilized to control motor rotation, enabling the robot's movement.

```py
speeds = self.ackermann.set_velocity(self.linear_x, self.angular_z)
```

The first parameter, '**linear_x**,' establishes the motor rotation speed in millimeters per second within the range of "**-0.6 to 0.6**." A negative value indicates reverse motor rotation.

The second parameter, '**angular_z**,' signifies the lateral offset rate of the robot, ranging from "**-1.0 to 1.0**." A positive value turns the robot left, while a negative value turns it right.

```py
self.motor_pub.publish(speeds[1])
```

Subsequently, the velocity values are transmitted to the ROS topic linked with '**self.motor_pub**.' The chassis drive control node can subscribe to this topic, utilizing the received linear and angular velocities to govern the robot's motion.

* **FAQ**

**Q:** Why pressing ‘Ctrl+C’ in the terminal does not stop the robot and it continues moving forward?

**A:** In this case, please open a new terminal. Within the new terminal, input the command "**rostopic pub /hiwonder_controller/cmd_vel geometry_msgs/Twist "linear:**" and press the TAB key for autocomplete. Then, set the speed to 0 and press Enter to execute the command.

