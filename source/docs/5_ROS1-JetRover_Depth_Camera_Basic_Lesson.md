# 5. ROS1-JetRover Depth Camera Basic Lesson

## 5.1 Depth Camera Configuration

> [!NOTE]
>
> Note: Ubuntu 18.04 and ROS Melodic are used as examples for installation. Ensure that the system and ROS version are not lower than Ubuntu 16.04 and ROS Kinetic. Using outdated software not only results in missing security and maintenance updates but also poses significant compatibility challenges, potentially leading to unsolvable issues.
>

### 5.1.1 Network Configuration

1. Open virtual machine <img src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image4.png" style="width:70px" />.

2. Right click the imported virtual machine “**Ubuntu**” and select “**Settings**”.

   <img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image5.png" style="width:500px" />

3. Click on “**Network Adaptor**”. If your computer adopts wired connection, select NAT mode. If your computer adopts wireless connection, select bridge mode.

   <img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image6.png" style="width:500px"  />

   After the mode is confirmed, click on ‘OK’.

   <img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image7.png" style="width:500px" />

4. Select the imported virtual machine, then click on ‘**Power on this virtual machine**’.

   <img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image8.png" style="width:500px" />

5. Click on <img src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image9.png" style="width:50px" /> to open terminal. Or press the short-cut ‘**Ctrl+Alt+t**’ to open the command-line terminal.

6. Enter command “ping www.baidu.com” to check whether the network connection is successful.

   ```py
   ping www.baidu.com
   ```

   If the terminal prints following content, that means network connection is successful.

   <img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image11.png" style="width:500px" />

   If it shows that “**Name or service not known**”, that means virtual machine fails to connect to the network.

   <img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image12.png" style="width:500px" />

7. Execute this step only when network connection is failed. Enter command “**ip a**” to check the ID of network card.

   <img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image13.png" style="width:500px"  />

8. Execute this step only when network connection is failed. Enter command “**sudo dhclient ens33**” and press Enter. “ens33” is the ID of network card. You need to modify the entered command based to suit the specific case at hand.

   ```py
   sudo dhclient ens33
   ```

   Repeat step 6. If the terminal prints following content, that means network connection is successful.

<img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image11.png" style="width:500px" />

### 5.1.2 **Install Dependency**

1. Run the command ‘**sudo apt update**’ to update apt library.

   ```py
   sudo apt update
   ```

After update, the following messages will occur.

<img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image16.png" style="width:500px" />

2. Input command “**sudo apt install ros-\$ROS_DISTRO-rgbd-launch**” to install dependency library.

   ```py
   sudo apt install ros-$ROS_DISTRO-rgbd-launch
   ```

If the following prompt occurs, you need to input the command manually.

<img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image18.png" style="width:500px" />

3. Input command “**sudo apt install ros-\$ROS_DISTRO-libuvc**” to install dependency library.

   ```py
   sudo apt install ros-$ROS_DISTRO-libuvc
   ```

4. Input command “**sudo apt install ros-\$ROS_DISTRO-libuvc-camera**” to install dependency library.

   ```py
   sudo apt install ros-$ROS_DISTRO-libuvc-camera
   ```

5. Input command “**sudo apt install ros-\$ROS_DISTRO-libuvc-ros**” to install dependency library.

   ```py
   sudo apt install ros-$ROS_DISTRO-libuvc-ros
   ```

6. Execute the command ‘**sudo apt-get install libgoogle-glog-dev**’ to install the dependency library.

   ```py
   sudo apt-get install libgoogle-glog-dev
   ```

### 5.1.3 **Construct Workspace**

1. Execute the command ‘**mkdir -p ~/ros_ws/src && cd ~/ros_ws**’ to build the workspace.

   ```py
   mkdir -p ~/ros_ws/src && cd ~/ros_ws
   ```

2. Execute the command '**catkin_make**' to compile the new workspace. Ensure that the compilation is performed within the '**ros_ws**' workspace.

   ```py
   catkin_make
   ```

   If the terminal displays the following message, it indicates that the compilation was successful.

<img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image25.png" style="width:500px" />

3. Run the command ‘**source ~/ros_ws/devel/setup.bash**’ to set a new workspace.

   ```py
   source ~/ros_ws/devel/setup.bash
   ```

4. Enter the command '**echo "source \$HOME/ros_ws/devel/setup.bash" \>\> ~/.bashrc**' to append the initialization action of environment variables to the terminal initialization file.

   ```py
   echo "source \$HOME/ros_ws/devel/setup.bash" >> ~/.bashrc
   ```

## 5.2 Install Depth Camera ROS SDK

1. Copy the file “**orbbec-ros-sdk.zip**” to the virtual machine.

2. Execute the command ‘**cd ~/Desktop**’ to enter the desktop.

   ```py
   cd ~/Desktop
   ```

3. Run the command ‘**unzip orbbec-ros-sdk.zip**’ to extract the file.

   ```py
   unzip orbbec-ros-sdk.zip
   ```

The file will be extracted to the desktop.

<img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image30.png" style="width:500px" />

4)  Right-click this folder to select “**copy**”.

<img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image31.png" style="width:500px" />

5)  Navigate to the folder as pictured, and paste the folder ‘**orbbec-ros-sdk**’ to the workspace.

<img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image32.png" style="width:500px" />

6. Execute the command “**cd ~/ros_ws/src/**” to enter the work space.

   ```py
   cd ~/ros_ws/src/
   ```

7. Run the command ‘**roscd orbbec_camera**’ to access the function pack.

   ```py
   roscd orbbec_camera
   ```

8. Enter the command ‘**chmod a+x ./script/99-obsensor-libusb.rules**’ to add permission.

   ```py
   chmod a+x ./script/99-obsensor-libusb.rules
   ```

9. Execute the command ‘**cd script**’ to navigate to the file directory.

   ```py
   cd script
   ```

10. Enter the command '**sudo cp 99-obsensor-libusb.rules /etc/udev/rules.d/99-obsensor-libusb.rules**'. If prompted for a password, please enter the password you have set.

    ```py
    sudo cp 99-obsensor-libusb.rules /etc/udev/rules.d/99-obsensor-libusb.rules
    ```

11. Run the command ‘**sudo udevadm control --reload && sudo udevadm trigger**’.

    ```py
    sudo udevadm control --reload && sudo udevadm trigger
    ```

12. Execute the command ‘**cd ~/ros_ws/**’ to navigate to the workspace.

    ```py
    cd ~/ros_ws/
    ```

13. Input the command ‘**catkin_make**’ to recompile the workspace. Ensure that the compilation is performed within the workspace.

    ```py
    catkin_make
    ```

If the terminal prints the following content, you need to recompile for multiple times.

<img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image41.png" style="width:500px" />

14. Enter the command ‘**source ~/ros_ws/devel/setup.bash**’ to set a new workspace.

    ```py
    source ~/ros_ws/devel/setup.bash
    ```

15. After you complete the above operations, the installation of ROS SDK is done.

## 5.3 Usage of Depth Camera

### 5.3.1 Enable Camera Service

1. Attach the depth camera to the computer and establish its connection with the virtual machine.

   <img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image42.png" style="width:500px" />

   <img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image43.png" style="width:500px" />

2. Run the command ‘**roslaunch orbbec_camera dabai_dcw.launch**’ to turn on the camera.

   ```py
   roslaunch orbbec_camera dabai_dcw.launch
   ```

   We offer two approaches for image visualization. One option involves utilizing rqt_image_view to observe 2D images, while the alternative is to employ rviz for visualizing 3D images. Users are free to select the method that aligns with their particular needs. It is crucial to emphasize that during image viewing, the camera service should remain active and not be closed.

* **View Image Using rqt_image_view Tool**

1. Open a new terminal, and execute the command ‘**rosrun rqt_image_view rqt_image_view**’ to start the image viewing tool.

   ```py
   rosrun rqt_image_view rqt_image_view
   ```

When the below interface appears, it means that the tool is started successfully.

<img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image46.png" style="width:500px" />

2. To view the image, you need to choose the appropriate tab from the drop-down menu.

   <img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image47.png" style="width:500px" />

3. To view the infrared image, please select the corresponding topic.

   <img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image48.png" style="width:500px" />

* **View Image Using rviz**

1. Execute the command ‘**rosrun rviz rviz**’ to initiate the tool.

   ```py
   rosrun rviz rviz
   ```

When the below interface appears, it means that the tool is started successfully.

<img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image50.png" style="width:500px" />

2)  To access the camera feed, start by configuring the "**Fixed Frame**" to "**camera_link**." Then, click on "**Add**," select "**By topic -\> camera -\> color -\> Image_raw -\> Image**," and finally, click "**OK**."

<img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image51.png" style="width:500px" />

When the interface below appears, it indicates that the tool has been successfully launched.

<img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image52.png" style="width:500px" />

3. To view the depth map, set ‘**Fixed Frame**’ as ‘**camera_link**’, then click-on ‘**Add**’ to select ‘**By topic -\>camera-\>depth-\>Image_raw-\>Image**’, and click-on ‘**OK**’.

   <img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image53.png" style="width:500px" />

When the interface below appears, it indicates that the tool has been successfully launched.

<img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image54.png" style="width:500px" />

### 5.3.2 LDP Protection

LDP protection," commonly known as "**Lens Data Protection**," safeguards the camera lens against accidental damage or interference in the realm of cameras. When utilizing the DaBai camera, if an object comes within a specified distance, DaBai will disable the infrared camera, and this distance is determined by the intensity of reflected light. Users have the flexibility to tailor the activation or deactivation of LDP protection according to their specific needs.

1. Run the command ‘**roslaunch orbbec_camera dabai_dcw.launch**’ to turn on the camera.

   ```py
   roslaunch orbbec_camera dabai_dcw.launch
   ```

2. Open a new terminal, and execute the command ‘**rosrun rqt_image_view rqt_image_view**’ to initiate the image viewing tool. Then choose the corresponding tap in the drop-down menu.

   ```py
   rosrun rqt_image_view rqt_image_view
   ```

   <img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image48.png" style="width:500px" />

When the object reaches a certain distance from the camera, the depth map will transition to black.

<img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image56.png" style="width:500px" />

3. Open a new command-line terminal, and execute the command ‘**rosservice call /camera/set_ldp "data: false"**’ to close LDP protection.

   ```py
   rosservice call /camera/set_ldp "data: false"
   ```

   <img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image57.png" style="width:500px" />

   At this stage, the image remains visible even as an object approaches the camera.

4. Execute the command ‘**rosservice call /camera/set_ldp "data: true"**’ to initiate LDP protection.

   ```py
   rosservice call /camera/set_ldp "data: true"
   ```

   <img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image58.png" style="width:500px" />

## 5.4 Camera Calibration

### 5.4.1 Preface

As the camera lens possesses both concave and convex elements, it may introduce distortions to the captured image. Nonetheless, by performing camera calibration, you can obtain internal, external, and distortion parameters. These parameters enable the calibration of distorted images and the reconstruction of a 3D scenario.

> [!NOTE]
>
> Note:
>
> * Before performing the calibration process, please make sure to print the chessboard pictures stored in the same directory. Ensure that the resolution of these pictures is not smaller than 640\*480.
>
> * It is crucial to operate the calibration procedure in a well-lit environment. However, be cautious to avoid direct sunlight during the calibration process.

### 5.4.2 Operation Steps

<img src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image59.png" style="width:50px" />The input command must be case-sensitive, and you can supplement keywords using the "Tab" key.

1. Start the robot, and enter the robot system desktop using NoMachine.

2. Click <img src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image60.png" style="width:50px" /> to open command line terminal.

3. Input command “**sudo systemctl stop start_app_node.service**” and press Enter to disable app service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

4. Input command “**roslaunch hiwonder_calibration depth_cam_rgb_calibration.launch**” and press Enter to enable camera calibration service.

   ```py
   roslaunch hiwonder_calibration depth_cam_rgb_calibration.launch
   ```

Before calibration, please ensure the size of the chessboard and the length of the square you input are correct. There are 8 squares on the first row except the first one, and 6 squares on the first column except the first one also, so the size of this chessboard is 8 x 6.

<img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image63.png" style="width:500px"  />

<img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image64.png" style="width:500px"  />

You can measure the side length of the square with rule, and the unit is meter. And this side length is 0.014m.

<img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image65.png" style="width:500px"  />

5. Face the chessboard picture horizontally to the camera, then quickly move and tilt the picture till the parameters of x, y, size and scale at right all turn green. Then click “**calibrate**” to start calculating the calibration.

   > [!NOTE]
   >
   > **Note: please always hold the chessboard picture horizontally. The time taken to calculate the calibration depends on the quantity of the pictures to be calibrated. The more pictures, the longer it takes.**

Parameter “**x**” represents the right and left zone of the camera field of view, and you need to move the chessboard left and right. Parameter “**y**” indicates top and bottom zone of the camera field of view, and you need to move the chessboard up and down. Parameter “**size**” is the distance between the picture and the camera, and you need to move the picture away from or close to the camera. Parameter “**skew**” stands for the tilt of the picture, and you need to tilt the picture at any angle while keeping the chessboard horizontal.

<img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image66.png" style="width:500px" />

6. After the program has finished calculated the calibration, you can move the picture within the camera frame to check whether the picture is displayed normally without Fisheye effect. If the displayed picture is normal, click-on “**commit**” button to save the calibration data which can be checked in this folder: **vim .ros/camera_info/camera.yaml**

   Click “**save**” button, and then the picture and calculation data will be saved in “**tmp**” folder. You can enter “**cd /tmp**” command to open “**tmp**” folder to find the related file.

   <img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image67.png" style="width:500px" />

## 5.5 Data Type and Point Cloud

### 5.5.1 Point Cloud Description

Human possess 3D vision, however computer only has 2D vision that is it only obtains 2D information from the pictures. To enable computer to display 3D objects, it is required to empower computer with 3D vision. And point cloud is a specific example of 3D vision.

### 5.5.2 Data Type of Point Cloud

Point cloud data is a set of vectors in 3D coordinate system, which are generally represented by X, Y, Z of 3D coordinate system. Usually, it is used to indicate the outer shape of an object, and also RGB color, gray value, depth, etc.

JetAuto’s point cloud data is obtained by Lidar and depth camera which can be used for measurement.

Point cloud data is commonly stored in the format of pts, asc, dat, stl, imw, xyz, txt, csv, etc. Working like attribute list, it records the position of X, Y and Z axes of the corresponding point, as well as various attributes.

### 5.5.3 Point Cloud

* **Depth Camera Point Cloud**

<img src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image59.png" style="width:50px" />The input command should be case sensitive, and the keywords can be complemented bat “**Tab**” key.

1. Start the robot, and enter the robot system desktop using NoMachine.

2. Click <img src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image60.png" style="width:50px" /> <span class="mark">to open command line</span> terminal<span class="mark">.</span>

3. Execute the command ‘**sudo systemctl stop start_app_node.service**’ to disable the app service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

4. Open a new terminal, and execute the command ‘**roslaunch hiwonder_example depth_cam_point_cloud_view.launch**’ to open depth camera point cloud.

   ```py
   roslaunch hiwonder_example depth_cam_point_cloud_view.launch
   ```

When the following interface pops up, you can view the point cloud. The colored parts are the point cloud data of the depth camera which are obtained by the infrared sensor on the depth camera.

<img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image70.png" style="width:500px" />

* **Depth Camera Infrared Map**

> [!NOTE]
>
> **Note: the input command should be case sensitive, and keywords can be complemented using Tab key.**

1. Start the robot, and enter the robot system desktop using NoMachine.

2. Click-on <img src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image60.png" style="width:50px" /> to open the command-line terminal.

3. Execute the command “**sudo systemctl stop start_app_node.service**” to disable app service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

4. Open a new terminal, and run the command “**roslaunch hiwonder_example depth_cam_ir_view.launch**” to open depth camera infrared map.

   ```py
   roslaunch hiwonder_example depth_cam_ir_view.launch
   ```

5)  The appearance of the interface below signals a successful initialization. The colored content within the yellow box in the image represents the point cloud data from the depth camera, obtained through the infrared sensor on the depth camera.

<img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image72.png" style="width:500px" />

##  5.6 Web Real-Time Monitoring

We can achieve real-time monitoring by viewing the robot's live feedback images through the webpage.

### 5.6.1 Access Live Camera Feed Through Web

1)  Open any web browser, and input the IP address ‘http://192.168.149.1:8080/’, then hit Enter.

> [!NOTE]
>
> **Note: if the prompt ‘This site can’t be reached’ occurs, please restart the app auto-start service.**

2)  Click-on ‘**image_raw(Snapshot)**’ to access the live camera feed.

<img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image73.png" style="width:500px" />

<img class="common_img" src="../_static/media/3/section_46_Depth Camera Basic Lesson/media/image74.png" style="width:500px" />

### 5.6.2 Enable APP Auto-Start Service

> [!NOTE]
>
> **Note: If the app auto-start service is turned off in previous operation, please follow the below steps to initiate the app service.**

1. Start the robot, and access the robot system desktop using NoMachine.

2. Execute the command ‘**sudo systemctl restart start_app_node.service**’, and hit Enter to enablr the app service. When the buzzer on the robot emits a beep, it means the app auto-start service restarts successfully.

   ```py
   sudo systemctl restart start_app_node.service
   ```

3. Once the application's auto-start service is active, proceed with the steps outlined in '**[5.6.1 Access Live Camera Feed Through Web]()**’.