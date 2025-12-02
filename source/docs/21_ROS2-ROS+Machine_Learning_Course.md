# 21 ROS2-ROS+Machine Learning Course

## 21.1 Autonomous Driving Debugging Lesson-v1.0

Please click [**10 ROS1-ROS+Machine Learning Lesson->10.4 Autonomous Driving Debugging Lesson V1.0**]() to get the Autonomous Driving Debugging Lesson-v1.0.

## 21.2 **Autonomous Driving v1.0**

###  21.2.1 Lane Keeping

This lesson focuses on controlling the car to move forward and maintain lane alignment through instructions.

<img class="common_img" src="../_static/media/4/section_19_1. Autonomous Driving v1.0/media/image4.png" style="width:500px" />

* **Preparation**

Before starting, ensure the map is laid out flat and free of wrinkles, with smooth roads and no obstacles. For specific map laying instructions, please refer to "**Autonomous Driving Debugging Lesson-v1.0**" in the same directory as this section for guidance. (In this lesson, we are only experiencing the road driving-line patrol function, so there is no need to place props such as traffic lights and signboards.)

When experiencing this game, ensure it is conducted in a well-lit environment, but avoid direct light shining on the camera to prevent misrecognition.

It is essential to adjust the color threshold beforehand and set the color threshold of the yellow line to avoid misidentification during subsequent recognition. For specific color threshold adjustment, please refer to the "**[20. ROS+OpenCV Courses]()**" for reference.

It is recommended to position the car in the middle of the road for easy identification!

* **Program Logic**

Lane keeping can be divided into three parts: obtaining real-time images, image processing, and result comparison.

Firstly, real-time images are obtained by capturing images using the camera.

Next, image processing includes color recognition, converting recognized images into different color spaces, erosion and dilation processing, and binarization processing.

The result comparison part involves processing the images to select the region of interest (ROI), outlining the processed images, and further comparing and calculating.

Finally, based on the comparison results, the direction of advancement is adjusted to keep the robot in the middle of the lane.

The source code for this program can be found at:

**/home/ubuntu/ros2_ws/src/example/example/self_driving/lane_detect.py**

```py
import os
import cv2
import math
import queue
import threading
import numpy as np
import sdk.common as common
from cv_bridge import CvBridge

bridge = CvBridge()

lab_data = common.get_yaml_data("/home/ubuntu/software/lab_tool/lab_config.yaml")

class LaneDetector(object):
    def __init__(self, color):
        # 车道线颜色(lane color)
        self.target_color = color
        # 车道线识别的区域(the recognized area of lane)
        if os.environ['DEPTH_CAMERA_TYPE'] == 'Dabai':
            self.rois = ((338, 360, 0, 320, 0.7), (292, 315, 0, 320, 0.2), (248, 270, 0, 320, 0.1))
        else:
            self.rois = ((450, 480, 0, 320, 0.7), (390, 480, 0, 320, 0.2), (330, 480, 0, 320, 0.1))
        self.weight_sum = 1.0

    def set_roi(self, roi):
        self.rois = roi

    @staticmethod
    def get_area_max_contour(contours, threshold=100):
        '''
        (retrieve the contour corresponding to the largest area)
        :param contours:
        :param threshold:
        :return:
        '''
        contour_area = zip(contours, tuple(map(lambda c: math.fabs(cv2.contourArea(c)), contours)))
        contour_area = tuple(filter(lambda c_a: c_a[1] > threshold, contour_area))
        if len(contour_area) > 0:
            max_c_a = max(contour_area, key=lambda c_a: c_a[1])
            return max_c_a
        return None
```

* **Operation Steps**

> [!NOTE]
>
> **Note: The input command should be case sensitive, and keywords can be complemented using Tab key.**
>
> **As ROS2 is placed within a container and cannot directly access the GPU of the mainboard, it needs to utilize ROS1 to start the mainboard's GPU, and then transfer the recognition information to ROS2 for use.**

Start the robot, and access the robot system desktop using NoMachine.

**Enable the model:**

1. Click-on <img src="../_static/media/4/section_19_1. Autonomous Driving v1.0/media/image6.png" style="width:50px" /> to open the ROS1 command-line terminal.

2. Execute the command and hit Enter to disable the app service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

3. Run the command to enable the autonomous driving service.

   ```py
   roslaunch hiwonder_example self_driving_base.launch
   ```

4. If you need to terminate the game, press ‘Ctrl+C’. If the game cannot be terminated, please retry.

   **Bridge:**

1. Click-on <img src="../_static/media/4/section_19_1. Autonomous Driving v1.0/media/image9.png" style="width:50px" /> to open the ROS2 command-line terminal.

2. Execute the following command in the 1<sup>st</sup> command-line terminal, and hit Enter key:

   ```py
   source ~/noetic_ws/install_isolated/setup.zsh
   ```

3. In the first command line, enter the command and press Enter:

   ```py
   source ~/third_party_ros2/ros1_bridge_ws/install/setup.zsh
   ```

4. In the first command line, enter the command and press Enter:

   ```py
   ros2 run ros1_bridge parameter_bridge
   ```

5. In the second command line, enter the command and press Enter:

   ```py
   ros2 launch example self_driving.launch.py only_line_follow:=true
   ```

6)  To close the program, click on the corresponding terminal window, and use the shortcut "Ctrl+C" to close the program.

* **Program Outcome**

After starting the game, place the robot on the road, and it will automatically detect the yellow line at the edge of the road. The robot will then adjust its position based on the detection results.

<img class="common_img" src="../_static/media/4/section_19_1. Autonomous Driving v1.0/media/image14.png" style="width:500px" />

* **Program Analysis**

The source code of this program is saved in

**ros2_ws/src/example/example/self_driving/lane_detect.py**

<img class="common_img" src="../_static/media/4/section_19_1. Autonomous Driving v1.0/media/image15.png" style="width:500px"  />

**Function:**

image_callback：

```py
	def image_callback(ros_image):
    global image
    rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面(original RGB image)
    image = rgb_image
```

Image callback function is used to read the camera node.

**Class:**

**LaneDetector:**

```py
class LaneDetector(object):
    def __init__(self, color):
        # 车道线颜色(lane color)
        self.target_color = color
        # 车道线识别的区域(the recognized area of lane)
        self.rois = ((450, 480, 0, 320, 0.7), (390, 420, 0, 320, 0.2), (330, 360, 0, 320, 0.1))
        self.weight_sum = 1.0

    def set_roi(self, roi):
        self.rois = roi
```

**Init:**

```py
    def __init__(self, color):
        # 车道线颜色(lane color)
        self.target_color = color
        # 车道线识别的区域(the recognized area of lane)
        self.rois = ((450, 480, 0, 320, 0.7), (390, 420, 0, 320, 0.2), (330, 360, 0, 320, 0.1))
        self.weight_sum = 1.0
```

Initialize the required parameters and set the ROI to lock the recognition range.

**set_roi:**

```py
    def set_roi(self, roi):
        self.rois = roi
```

Used to set the Region of Interest (ROI) for recognition.

**get_area_max_contour:**

```py
	    def get_area_max_contour(contours, threshold=100):
        '''
        获取最大面积对应的轮廓(retrieve the contour corresponding to the largest area)
        :param contours:
        :param threshold:
        :return:
        '''
        contour_area = zip(contours, tuple(map(lambda c: math.fabs(cv2.contourArea(c)), contours)))
        contour_area = tuple(filter(lambda c_a: c_a[1] > threshold, contour_area))
        if len(contour_area) > 0:
            max_c_a = max(contour_area, key=lambda c_a: c_a[1])
            return max_c_a
        return None
```

Obtains the contour with the maximum area from the list of contours obtained through OpenCV.

**add_horizontal_line:**

```py
	    def add_horizontal_line(self, image):
        #   |____  --->   |————   ---> ——
        h, w = image.shape[:2]
        roi_w_min = int(w/2)
        roi_w_max = w
        roi_h_min = 0
        roi_h_max = h
        roi = image[roi_h_min:roi_h_max, roi_w_min:roi_w_max]  # 截取右半边(capture the right half)
        flip_binary = cv2.flip(roi, 0)  # 上下翻转(flip vertically)
        max_y = cv2.minMaxLoc(flip_binary)[-1][1]  # 提取最上，最左数值为255的点坐标(extract coordinates of points with the highest and leftmost values of 255)

        return h - max_y
```

Adds a horizontal recognition line based on the width and height of the frame and the ROI settings.

**add_vertical_line_far:**

```py
	    def add_vertical_line_far(self, image):
        h, w = image.shape[:2]
        roi_w_min = int(w/8)
        roi_w_max = int(w/2)
        roi_h_min = 0
        roi_h_max = h
        roi = image[roi_h_min:roi_h_max, roi_w_min:roi_w_max]
        flip_binary = cv2.flip(roi, -1)  # 图像左右上下翻转(flip the image horizontally and vertically)
        #cv2.imshow('1', flip_binary)
        # min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(ret)
	        # minVal：最小值(the minimum value)
	        # maxVal：最大值(the maximal value)
	        # minLoc：最小值的位置(the position of minimum value)
	        # maxLoc：最大值的位置(the position of the maximal value)
        # 遍历的顺序，先行再列，行从左到右，列从上到下(the traversal order is row-major, iterating through rows from left to right and columns from top to bottom)
        (x_0, y_0) = cv2.minMaxLoc(flip_binary)[-1]  # 提取最上，最左数值为255的点坐标(extract coordinates of points with the highest and leftmost values of 255)
        y_center = y_0 + 55
        roi = flip_binary[y_center:, :]
        (x_1, y_1) = cv2.minMaxLoc(roi)[-1]
        down_p = (roi_w_max - x_1, roi_h_max - (y_1 + y_center))
        
        y_center = y_0 + 65
        roi = flip_binary[y_center:, :]
        (x_2, y_2) = cv2.minMaxLoc(roi)[-1]
        up_p = (roi_w_max - x_2, roi_h_max - (y_2 + y_center))

        up_point = (0, 0)
        down_point = (0, 0)
        if up_p[1] - down_p[1] != 0 and up_p[0] - down_p[0] != 0:
            up_point = (int(-down_p[1]/((up_p[1] - down_p[1])/(up_p[0] - down_p[0])) + down_p[0]), 0)
            down_point = (int((h - down_p[1])/((up_p[1] - down_p[1])/(up_p[0] - down_p[0])) + down_p[0]), h)

        return up_point, down_point
```

Adds a recognition vertical line for the part of the frame farther from the robot based on the ROI settings.

**get_binary**

```py
	    def get_binary(self, image):
        # 通过lab空间识别颜色(recognize color through lab space)
        img_lab = cv2.cvtColor(image, cv2.COLOR_RGB2LAB)  # rgb转lab(convert rgb to lab)
        img_blur = cv2.GaussianBlur(img_lab, (3, 3), 3)  # 高斯模糊去噪(Gaussian blur for noise reduction)
        mask = cv2.inRange(img_blur, tuple(lab_data['lab']['Stereo'][self.target_color]['min']), tuple(lab_data['lab']['Stereo'][self.target_color]['max']))  # 二值化
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀(corrosion)
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀(dilation)

        return dilated
```

Performs color recognition based on the color space and processes the binarized image.

**add_vertical_line_near:**

```py
    def add_vertical_line_near(self, image):
        # ——|         |——        |
        #   |   --->  |     --->
        h, w = image.shape[:2]
        roi_w_min = 0
        roi_w_max = int(w/2)
        roi_h_min = int(h/2)
        roi_h_max = h
        roi = image[roi_h_min:roi_h_max, roi_w_min:roi_w_max]
        flip_binary = cv2.flip(roi, -1)  # 图像左右上下翻转(flip the image horizontally and vertically)
        #cv2.imshow('1', flip_binary)
        (x_0, y_0) = cv2.minMaxLoc(flip_binary)[-1]  # 提取最上，最左数值为255的点坐标(extract coordinates of points with the highest and leftmost values of 255)
        down_p = (roi_w_max - x_0, roi_h_max - y_0)

        (x_1, y_1) = cv2.minMaxLoc(roi)[-1]
        y_center = int((roi_h_max - roi_h_min - y_1 + y_0)/2)
        roi = flip_binary[y_center:, :] 
        (x, y) = cv2.minMaxLoc(roi)[-1]
        up_p = (roi_w_max - x, roi_h_max - (y + y_center))

        up_point = (0, 0)
        down_point = (0, 0)
        if up_p[1] - down_p[1] != 0 and up_p[0] - down_p[0] != 0:
            up_point = (int(-down_p[1]/((up_p[1] - down_p[1])/(up_p[0] - down_p[0])) + down_p[0]), 0)
            down_point = down_p

        return up_point, down_point, y_center

    def get_binary(self, image):
        # 通过lab空间识别颜色(recognize color through lab space)
        img_lab = cv2.cvtColor(image, cv2.COLOR_RGB2LAB)  # rgb转lab(convert rgb to lab)
        img_blur = cv2.GaussianBlur(img_lab, (3, 3), 3)  # 高斯模糊去噪(Gaussian blur for noise reduction)
        mask = cv2.inRange(img_blur, tuple(lab_data['lab']['Stereo'][self.target_color]['min']), tuple(lab_data['lab']['Stereo'][self.target_color]['max']))  # 二值化
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀(corrosion)
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀(dilation)

        return dilated
```

Adds a recognition vertical line for the part of the frame closer to the robot based on the ROI and frame width and height settings.

**\_\_call\_\_:**

```py
    def __call__(self, image, result_image):
        # 按比重提取线中心(extracting line centers based on weight)
        centroid_sum = 0
        h, w = image.shape[:2]
        max_center_x = -1
        center_x = []
        for roi in self.rois:
            blob = image[roi[0]:roi[1], roi[2]:roi[3]]  # 截取roi(crop roi)
            contours = cv2.findContours(blob, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  # 找轮廓(find contour)
            max_contour_area = self.get_area_max_contour(contours, 30)  # 获取最大面积对应轮廓(retrieve the contour corresponding to the largest area)
            if max_contour_area is not None:
                rect = cv2.minAreaRect(max_contour_area[0])  # 最小外接矩形(the minimum bounding rectangle)
                box = np.intp(cv2.boxPoints(rect))  # 四个角(four corners)
                for j in range(4):
                    box[j, 1] = box[j, 1] + roi[0]
                cv2.drawContours(result_image, [box], -1, (255, 255, 0), 2)  # 画出四个点组成的矩形(draw a rectangle formed by four points)

                # 获取矩形对角点(retrieve the diagonal points of the rectangle)
                pt1_x, pt1_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                # 线的中心点(the center point of line)
                line_center_x, line_center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2

                cv2.circle(result_image, (int(line_center_x), int(line_center_y)), 5, (0, 0, 255), -1)  # 画出中心点(draw center point)
                center_x.append(line_center_x)
            else:
                center_x.append(-1)
        for i in range(len(center_x)):
            if center_x[i] != -1:
                if center_x[i] > max_center_x:
                    max_center_x = center_x[i]
                centroid_sum += center_x[i] * self.rois[i][-1]
        if centroid_sum == 0:
            return result_image, None, max_center_x
        center_pos = centroid_sum / self.weight_sum  # 按比重计算中心点(calculate the centroid based on weight)
        angle = math.degrees(-math.atan((center_pos - (w / 2.0)) / (h / 2.0)))
        
        return result_image, angle, max_center_x

```

Callback function for the entire class. Performs color recognition here, draws the recognized yellow lines using OpenCV, and then outputs the image, angle, and pixel coordinates X of each ROI recognition contour.

### 21.2.2 Road Sign Detection

* **Preparation**

Before starting, ensure the map is laid out flat and free of wrinkles, with smooth roads and no obstacles. For specific map laying instructions, please refer to "**[21.1 Autonomous Driving Debugging]()**" in the same directory as this section for guidance.

When experiencing this game, ensure it is conducted in a well-lit environment, but avoid direct light shining on the camera to prevent misrecognition.

* **Program Logic**

Firstly, acquire the real-time image from the camera and apply operations such as erosion and dilation.

Next, invoke the YOLOv5 model and compare it with the target screen image.

Finally, execute the appropriate landmark action based on the comparison results.

You can find the source code for this program at:

**/home/ros_ws/src/hiwonder_example/scripts/yolov5_detect/yolov5_trt_7_0.py**

```py
class YoLov5TRT(object):
    """
    description: A YOLOv5 class that warps TensorRT ops, preprocess and postprocess ops.
    """

    def __init__(self, engine_file_path, plugin, classes, conf_thresh=0.8, iou_threshold=0.4):
        self.CONF_THRESH = conf_thresh
        self.IOU_THRESHOLD = iou_threshold
       
        PLUGIN_LIBRARY = plugin
        self.engine_file_path = engine_file_path
        
        # load labels
        self.categories = classes

        ctypes.CDLL(PLUGIN_LIBRARY)

        # Create a Context on this device,
        self.ctx = cuda.Device(0).make_context()
        stream = cuda.Stream()
        TRT_LOGGER = trt.Logger(trt.Logger.INFO)
        runtime = trt.Runtime(TRT_LOGGER)

        # Deserialize the engine from file
        with open(self.engine_file_path, "rb") as f:
            engine = runtime.deserialize_cuda_engine(f.read())
        context = engine.create_execution_context()

        host_inputs = []
        cuda_inputs = []
        host_outputs = []
        cuda_outputs = []
        bindings = []
```

* **Operation Steps**

> [!NOTE]
>
> **Note: The following steps exclusively activate road sign detection in the return screen, without executing associated actions. Users seeking direct experience with autonomous driving may bypass this lesson and proceed to "Integrated Application" within the same file.**
>
> **Since ROS2 is located inside a container, it cannot directly access the GPU of the mainboard. Therefore, we need to use ROS1 to start the GPU of the mainboard. Here, starting the YOLO model requires the use of GPU, so it's better to check the results through ROS1.**
>
> **Please make sure to enter commands with strict attention to capitalization, spacing, and you can use the "Tab" key to autocomplete keywords.**

1. Start the robot, and access the robot system desktop using NoMachine.

2. Click-on <img src="../_static/media/4/section_19_1. Autonomous Driving v1.0/media/image6.png" style="width:50px" /> to open the command-line terminal.

3. Execute the command, and hit Enter to disable the app service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

4. Run the command to navigate to the directory containing programs.

   ```py
   roscd hiwonder_example/scripts/yolov5_detect/
   ```

5. Type the command to open the program source code.

   ```py
   vim yolov5_trt_7_0.py
   ```

6. Press the "i" key to enter insertion mode. Find the code enclosed in the red box and toggle commenting by adding or removing comments as needed. Once done, press the "ESC" key, then type "**:wq**" and press Enter to save and exit.

   <img class="common_img" src="../_static/media/4/section_19_1. Autonomous Driving v1.0/media/image29.png" style="width:500px" />

7. Run the command to initiate game program.

   ```py
   python3 yolov5_trt_7_0.py
   ```

8. Position the road sign in front of the camera, and the robot will recognize the road sign automatically.

9. If you need to terminate this game, press ‘**Ctrl+C**’.

10. Once you've completed the game experience, you can initiate the mobile app service by following the instructions or restarting the robot.

    If the mobile app service is not activated, the associated app functions will be disabled. (The mobile app service will automatically start when the robot restarts).

To restart the mobile app service, enter the command "**sudo systemctl start start_app_node.service**" and wait for the buzzer to emit a single beep, indicating that the service startup is complete.

> [!NOTE]
>
> **Note: In the event that the model struggles to recognize traffic-related signs, it may be necessary to lower the confidence level. Conversely, if the model consistently misidentifies traffic-related signs, raising the confidence level might be advisable.**

1. Run the command to navigate to the directory containing programs.

   ```py
   cd ros_ws/src/hiwonder_example/scripts/self_driving
   ```

2. Enter the command to access the game program.

   ```py
   vim self_driving.launch
   ```

   The red box represents the confidence value, which can be adjusted to modify the effectiveness of target detection.

   <img class="common_img" src="../_static/media/4/section_19_1. Autonomous Driving v1.0/media/image33.png" style="width:500px" />

* **Program Outcome**

After initiating the game, place the robot on the road within the map. Once the robot identifies landmarks, it will highlight the detected landmarks and annotate them based on the highest confidence level learned from the model.

<img class="common_img" src="../_static/media/4/section_19_1. Autonomous Driving v1.0/media/image34.png" style="width:500px" />

### 21.2.3 Traffic Light Recognition

* **Preparation**

1)  Before starting, ensure the map is laid out flat and free of wrinkles, with smooth roads and no obstacles. For specific map laying instructions, please refer to "**[21.1 Autonomous Driving Debugging Lesson]()**" in the same directory as this section for guidance.

2)  When experiencing this game, ensure it is conducted in a well-lit environment, but avoid direct light shining on the camera to prevent misrecognition.

* **Program Logic**

Firstly, capture a real-time image from the camera and apply operations such as erosion and dilation.

Next, invoke the YOLOv5 model to compare it with the target screen image.

Finally, execute corresponding landmark actions based on the comparison results.

The source code for this program can be found at:

**/home/ros_ws/src/hiwonder_example/scripts/yolov5_detect/yolov5_trt_7_0.py**

```py
class YoLov5TRT(object):
    """
    description: A YOLOv5 class that warps TensorRT ops, preprocess and postprocess ops.
    """

    def __init__(self, engine_file_path, plugin, classes, conf_thresh=0.8, iou_threshold=0.4):
        self.CONF_THRESH = conf_thresh
        self.IOU_THRESHOLD = iou_threshold
       
        PLUGIN_LIBRARY = plugin
        self.engine_file_path = engine_file_path
        
        # load labels
        self.categories = classes

        ctypes.CDLL(PLUGIN_LIBRARY)

        # Create a Context on this device,
        self.ctx = cuda.Device(0).make_context()
        stream = cuda.Stream()
        TRT_LOGGER = trt.Logger(trt.Logger.INFO)
        runtime = trt.Runtime(TRT_LOGGER)

        # Deserialize the engine from file
        with open(self.engine_file_path, "rb") as f:
            engine = runtime.deserialize_cuda_engine(f.read())
        context = engine.create_execution_context()

        host_inputs = []
        cuda_inputs = []
        host_outputs = []
        cuda_outputs = []
        bindings = []
```

* **Operation Steps**

> [!NOTE]
>
> **Note: The following steps exclusively activate road sign detection in the return screen, without executing associated actions. Users seeking direct experience with autonomous driving may bypass this lesson and proceed to "Integrated Application" within the same file.**
>
> **Since ROS2 is located inside a container, it cannot directly access the GPU of the mainboard. Therefore, we need to use ROS1 to start the GPU of the mainboard. Here, starting the YOLO model requires the use of GPU, so it's better to check the results through ROS1.**
>
> **Please make sure to enter commands with strict attention to capitalization, spacing, and you can use the "Tab" key to autocomplete keywords.**

1. Start the robot, and access the robot system desktop using NoMachine.

2. Click-on <img src="../_static/media/4/section_19_1. Autonomous Driving v1.0/media/image6.png" style="width:50px" /> to open the command-line terminal.

3. Execute the command, and hit Enter to disable the app service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

4. Run the command to navigate to the directory containing programs.

   ```py
   roscd hiwonder_example/scripts/yolov5_detect/
   ```

5. Type the command to open the program source code.

   ```py
   vim yolov5_trt_7_0.py
   ```

6. Press the "i" key to enter insertion mode. Find the code enclosed in the red box and toggle commenting by adding or removing comments as needed. Once done, press the "ESC" key, then type "**:wq**" and press Enter to save and exit.

   <img class="common_img" src="../_static/media/4/section_19_1. Autonomous Driving v1.0/media/image29.png" style="width:500px" />

7. Run the command to initiate game program.

   ```py
   python3 yolov5_trt_7_0.py
   ```

8. Position the road sign in front of the camera, and the robot will recognize the road sign automatically.

9. If you need to terminate this game, press ‘**Ctrl+C**’.

10. Once you've completed the game experience, you can initiate the mobile app service by following the instructions or restarting the robot.

    If the mobile app service is not activated, the associated app functions will be disabled. (The mobile app service will automatically start when the robot restarts).

To restart the mobile app service, enter the command "**sudo systemctl start start_app_node.service**" and wait for the buzzer to emit a single beep, indicating that the service startup is complete.

> [!NOTE]
>
> **Note: In the event that the model struggles to recognize traffic-related signs, it may be necessary to lower the confidence level. Conversely, if the model consistently misidentifies traffic-related signs, raising the confidence level might be advisable.**

1. Run the command to navigate to the directory containing programs.

   ```py
   cd ros_ws/src/hiwonder_example/scripts/self_driving
   ```

2. Enter the command to access the game program.

   ```py
   vim self_driving.launch
   ```

   The red box represents the confidence value, which can be adjusted to modify the effectiveness of target detection.

   <img class="common_img" src="../_static/media/4/section_19_1. Autonomous Driving v1.0/media/image33.png" style="width:500px" />

* **Program Outcome**

After initiating the game, position the robot on the road depicted on the map. Upon recognizing the traffic signal, the robot will assess the color of the signal light and identify frames corresponding to red and green signal lights accordingly.

<img class="common_img" src="../_static/media/4/section_19_1. Autonomous Driving v1.0/media/image35.png" style="width:500px" />

###  21.2.4 Turning Decision Making

* **Preparation**

1)  Before starting, ensure the map is laid out flat and free of wrinkles, with smooth roads and no obstacles. For specific map laying instructions, please refer to "**[21.1 Autonomous Driving Debugging Lesson]()**" in the same directory as this section for guidance.

2)  The roadmap model discussed in this section is trained using YOLOv5. For further information on YOLOv5 and related content, please refer to "**[10.1 Machine Learning Fundamentals]()**”.

3)  When experiencing this game, ensure it is conducted in a well-lit environment, but avoid direct light shining on the camera to prevent misrecognition.

* **Program Logic**

Firstly, capture the real-time image from the camera and apply operations such as erosion and dilation.

Next, invoke the YOLOv5 model to compare the obtained image with the target screen image.

Finally, based on the comparison outcomes, recognize the steering sign and direct the car accordingly.

The source code of this program is saved in

**/home/ros_ws/src/hiwodner_example/scripts/yolov5_detect/yolov5_trt_7_0.py**

```py
import ctypes
import time
import cv2
import random
import numpy as np
import pycuda.autoinit
import pycuda.driver as cuda
import tensorrt as trt

LEN_ALL_RESULT = 38001
LEN_ONE_RESULT = 38

class Colors:
    # Ultralytics color palette https://ultralytics.com/
    def __init__(self):
        # hex = matplotlib.colors.TABLEAU_COLORS.values()
        hex = ('FF3838', 'FF9D97', 'FF701F', 'FFB21D', 'CFD231', '48F90A', '92CC17', '3DDB86', '1A9334', '00D4BB',
               '2C99A8', '00C2FF', '344593', '6473FF', '0018EC', '8438FF', '520085', 'CB38FF', 'FF95C8', 'FF37C7')
        self.palette = [self.hex2rgb('#' + c) for c in hex]
        self.n = len(self.palette)

    def __call__(self, i, bgr=False):
        c = self.palette[int(i) % self.n]
        return (c[2], c[1], c[0]) if bgr else c

    @staticmethod
    def hex2rgb(h):  # rgb order (PIL)
        return tuple(int(h[1 + i:1 + i + 2], 16) for i in (0, 2, 4))

colors = Colors()  # create instance for 'from utils.plots import colors'

```

* **Operation Steps**

> [!NOTE]
>
> **Notice:**
>
> * **The following steps exclusively activate road sign detection in the return screen, without executing associated actions. Users seeking direct experience with autonomous driving may bypass this lesson and proceed to "Integrated Application" within the same file.**
>
> * **Since ROS2 is located inside a container, it cannot directly access the GPU of the mainboard. Therefore, we need to use ROS1 to start the GPU of the mainboard. Here, starting the YOLO model requires the use of GPU, so it's better to check the results through ROS1.**
>
> * **Please make sure to enter commands with strict attention to capitalization, spacing, and you can use the "Tab" key to autocomplete keywords.**

1. Start the robot, and access the robot system desktop using NoMachine.

2. Click-on <img src="../_static/media/4/section_19_1. Autonomous Driving v1.0/media/image6.png" style="width:50px" /> to open the command-line terminal.

3. Execute the command, and hit Enter to disable the app service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

4. Run the command to navigate to the directory containing programs.

   ```py
   roscd hiwonder_example/scripts/yolov5_detect/
   ```

5. Type the command to open the program source code.

   ```py
   vim yolov5_trt_7_0.py
   ```

6. Press the "i" key to enter insertion mode. Find the code enclosed in the red box and toggle commenting by adding or removing comments as needed. Once done, press the "ESC" key, then type "**:wq**" and press Enter to save and exit.

   <img class="common_img" src="../_static/media/4/section_19_1. Autonomous Driving v1.0/media/image29.png" style="width:500px" />

7. Run the command to initiate game program.

   ```py
   python3 yolov5_trt_7_0.py
   ```

8. Position the road sign in front of the camera, and the robot will recognize the road sign automatically.

9. If you need to terminate this game, press ‘**Ctrl+C**’.

10. Once you've completed the game experience, you can initiate the mobile app service by following the instructions or restarting the robot.

    If the mobile app service is not activated, the associated app functions will be disabled. (The mobile app service will automatically start when the robot restarts).

To restart the mobile app service, enter the command "**sudo systemctl start start_app_node.service**" and wait for the buzzer to emit a single beep, indicating that the service startup is complete.

> [!NOTE]
>
> **Note: In the event that the model struggles to recognize traffic-related signs, it may be necessary to lower the confidence level. Conversely, if the model consistently misidentifies traffic-related signs, raising the confidence level might be advisable.**

1. Run the command to navigate to the directory containing programs.

   ```py
   cd ros_ws/src/hiwonder_example/scripts/self_driving
   ```

2. Enter the command to access the game program.

   ```py
   vim self_driving.launch
   ```

   The red box represents the confidence value, which can be adjusted to modify the effectiveness of target detection.

   <img class="common_img" src="../_static/media/4/section_19_1. Autonomous Driving v1.0/media/image33.png" style="width:500px" />

* **Program Outcome**

Once the game begins, position the robot onto the road within the map. As the robot approaches a turning road sign, it will adjust its direction in accordance with the instructions provided by the sign.

<img class="common_img" src="../_static/media/4/section_19_1. Autonomous Driving v1.0/media/image37.png" style="width:500px" />

###  21.2.5 Autonomous Parking

* **Preparation**

1)  Before starting, ensure the map is laid out flat and free of wrinkles, with smooth roads and no obstacles. For specific map laying instructions, please refer to "**[21.1 Autonomous Driving Debugging Lesson]()**" in the same directory as this section for guidance.

2)  The roadmap model discussed in this section is trained using YOLOv5. For further information on YOLOv5 and related content, please refer to "**[10.1 Machine Learning Fundamentals]()**”.

3)  When experiencing this game, ensure it is conducted in a well-lit environment, but avoid direct light shining on the camera to prevent misrecognition.

* **Program Logic**

Begin by capturing the real-time image from the camera and applying operations such as erosion and dilation.

Next, invoke the YOLOv5 model to compare the obtained image with the target screen image.

Finally, based on the comparison results, identify the parking road sign and autonomously guide the car to park in the designated parking space.

The source code of this program is saved in

**/home/ros_ws/src/hiwonder_example/scripts/Yolov5_detect/yolov5_trt_7_0.py**

```py
import ctypes
import time
import cv2
import random
import numpy as np
import pycuda.autoinit
import pycuda.driver as cuda
import tensorrt as trt

LEN_ALL_RESULT = 38001
LEN_ONE_RESULT = 38

class Colors:
    # Ultralytics color palette https://ultralytics.com/
    def __init__(self):
        # hex = matplotlib.colors.TABLEAU_COLORS.values()
        hex = ('FF3838', 'FF9D97', 'FF701F', 'FFB21D', 'CFD231', '48F90A', '92CC17', '3DDB86', '1A9334', '00D4BB',
               '2C99A8', '00C2FF', '344593', '6473FF', '0018EC', '8438FF', '520085', 'CB38FF', 'FF95C8', 'FF37C7')
        self.palette = [self.hex2rgb('#' + c) for c in hex]
        self.n = len(self.palette)

    def __call__(self, i, bgr=False):
        c = self.palette[int(i) % self.n]
        return (c[2], c[1], c[0]) if bgr else c

    @staticmethod
    def hex2rgb(h):  # rgb order (PIL)
        return tuple(int(h[1 + i:1 + i + 2], 16) for i in (0, 2, 4))

colors = Colors()  # create instance for 'from utils.plots import colors'

```

* **Operation Steps**

> [!NOTE]
>
> **Notice:**
>
> * **The following steps exclusively activate road sign detection in the return screen, without executing associated actions. Users seeking direct experience with autonomous driving may bypass this lesson and proceed to "Integrated Application" within the same file.**
>
> * **Since ROS2 is located inside a container, it cannot directly access the GPU of the mainboard. Therefore, we need to use ROS1 to start the GPU of the mainboard. Here, starting the YOLO model requires the use of GPU, so it's better to check the results through ROS1.**
>
> * **Please make sure to enter commands with strict attention to capitalization, spacing, and you can use the "Tab" key to autocomplete keywords.**

1. Start the robot, and access the robot system desktop using NoMachine.

2. Click-on <img src="../_static/media/4/section_19_1. Autonomous Driving v1.0/media/image6.png" style="width:50px" /> to open the command-line terminal.

3. Execute the command, and hit Enter to disable the app service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

4. Run the command to navigate to the directory containing programs.

   ```py
   roscd hiwonder_example/scripts/yolov5_detect/
   ```

5. Type the command to open the program source code.

   ```py
   vim yolov5_trt_7_0.py
   ```

6. Press the "i" key to enter insertion mode. Find the code enclosed in the red box and toggle commenting by adding or removing comments as needed. Once done, press the "ESC" key, then type "**:wq**" and press Enter to save and exit.

   <img class="common_img" src="../_static/media/4/section_19_1. Autonomous Driving v1.0/media/image29.png" style="width:500px" />

7. Run the command to initiate game program.

   ```py
   python3 yolov5_trt_7_0.py
   ```

8. Position the road sign in front of the camera, and the robot will recognize the road sign automatically.

9. If you need to terminate this game, press ‘**Ctrl+C**’.

10. Once you've completed the game experience, you can initiate the mobile app service by following the instructions or restarting the robot.

    If the mobile app service is not activated, the associated app functions will be disabled. (The mobile app service will automatically start when the robot restarts).

To restart the mobile app service, enter the command "**sudo systemctl start start_app_node.service**" and wait for the buzzer to emit a single beep, indicating that the service startup is complete.

> [!NOTE]
>
> **Note: In the event that the model struggles to recognize traffic-related signs, it may be necessary to lower the confidence level. Conversely, if the model consistently misidentifies traffic-related signs, raising the confidence level might be advisable.**

1. Run the command to navigate to the directory containing programs.

   ```py
   cd ros_ws/src/hiwonder_example/scripts/self_driving
   ```

2. Enter the command to access the game program.

   ```py
   vim self_driving.launch
   ```

   The red box represents the confidence value, which can be adjusted to modify the effectiveness of target detection.

   <img class="common_img" src="../_static/media/4/section_19_1. Autonomous Driving v1.0/media/image33.png" style="width:500px" />

* **Program Outcome**

After initiating the game, position the robot on the road within the map. As the robot progresses towards the parking sign, it will automatically park in the designated parking space based on the instructions provided by the road sign.

* **Parameter Adjustment**

If the robot stops upon recognizing the parking sign and the parking position is not optimal, adjustments to the parameters in the program source code can be made.

1. Click-on <img src="../_static/media/4/section_19_1. Autonomous Driving v1.0/media/image6.png" style="width:50px" /> to open the ROS1 command-line terminal.

2. Execute the command to navigate to the directory containing game programs.

   ```py
   cd ros2_ws/src/example/example/self_driving/self_driving.py
   ```

3. Run the command to access the source code.

   ```py
   vim self_driving.py
   ```

4. Press the "i" key to enter insert mode and locate the code within the red box. Adjusting the parameters within the red box allows you to control the starting position for the robot to initiate the parking operation. Decreasing the parameters will result in the robot stopping closer to the zebra crossing, while increasing them will cause the robot to stop further away. Once adjustments are made, press the "**ESC**" key, type "**:wq**", and press Enter to save and exit.

<img class="common_img" src="../_static/media/4/section_19_1. Autonomous Driving v1.0/media/image40.png" style="width:500px" />

You can adjust the parking processing function to alter the parking position of the robot. Initially, the parking action sets the linear speed in the negative direction of the Y-axis (right of the robot) to 0.2 meters per second, with a forward movement time of (0.38/2) seconds. To position the robot in the ideal location on the left side of the parking space, modify the speed and time accordingly.

<img class="common_img" src="../_static/media/4/section_19_1. Autonomous Driving v1.0/media/image41.png" style="width:500px" />

###  21.2.6 Integrated Application

This lesson provides instructions for implementing comprehensive driverless game on the robot, covering lane keeping, road sign detection, traffic light recognition, steering decision-making, and self-parking.

* **Preparation**

**1. Map Setup**

To ensure accurate navigation, place the map on a flat, smooth surface, free of wrinkles and obstacles. Position all road signs and traffic lights at designated locations on the map, facing clockwise. The starting point and locations of road signs are indicated below:

> [!NOTE]
>
> Note: Tools required for autonomous driving are available for separate purchase. If you are interested, kindly reach out to us at support@hiwonder.com.

<img class="common_img" src="../_static/media/4/section_19_1. Autonomous Driving v1.0/media/image42.png" style="width:500px"  />

**2. Color Threshold Adjustment**

Due to variations in light sources, it's essential to adjust the color thresholds for 'black, white, red, green, blue, and yellow' based on the guidelines provided in the '**[20 ROS+OpenCV Course]()**' prior to starting. If the robot encounter inaccurate recognition while moving forward, readjust the color threshold specifically in the map area where recognition fails.

* **Program Logic**

<img class="common_img" src="../_static/media/4/section_19_1. Autonomous Driving v1.0/media/image43.png" style="width:500px" />

Actions implemented so far include:

1.  Following the yellow line in the outermost circle of the patrol map.

2.  Slowing down and passing if a zebra crossing is detected.

3.  Making a turn upon detection of a turn sign.

4.  Parking the vehicle and entering the parking lot upon detection of a stop sign.

5.  Halting when a red light is detected.

6.  Slowing down when passing a detected street light.

First, load the model file trained by YOLOv5 and the required library files, obtain real-time camera images, and perform operations such as erosion and dilation on the images. Next, identify the target color line segment in the image and gather information such as size and center point of the target image. Then, invoke the model through YOLOv5 and compare it with the target screen image. Finally, adjust the forward direction based on the offset comparison of the target image's center point to keep the robot in the middle of the road. Additionally, perform corresponding actions based on different recognized landmark information during map traversal.

The source code for this program can be found at:

**/home/ubuntu/ros2_ws/src/example/example/self_driving/self_driving.py**

* **Operation Steps**

> [!NOTE]
>
> **Note: the input command should be case sensitive, and keywords can be implemented using Tab key.**
>
> **As ROS2 is located within a container, it cannot directly access the GPU of the mainboard. Thus, it's necessary to initiate the GPU of the mainboard through ROS1 and then transfer the recognition information for use within ROS2.**

Start the robot, and access the robot system desktop using NoMachine.

**Enable the model:**

1. Click-on <img src="../_static/media/4/section_19_1. Autonomous Driving v1.0/media/image6.png" style="width:50px" /> to open the ROS1 command-line terminal.

2. Execute the command, and hit Enter to disable the app service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

3. Run the command to enable the autonomous driving service.

   ```py
   roslaunch hiwonder_example self_driving_base.launch
   ```

4. If you need to close the program, simply click on the corresponding terminal window of the program and use the shortcut "Ctrl+C" to exit the program.

   **Bridge:**

1. Click-on <img src="../_static/media/4/section_19_1. Autonomous Driving v1.0/media/image9.png" style="width:50px" /> to start the ROS2 command-line terminal.

2. Right-click on a blank area, select the option shown in the image below, and then choose "**Split**":

3. Execute the command in the 1<sup>st</sup> command-line terminal, and hit Enter key.

   ```py
   source ~/noetic_ws/install_isolated/setup.zsh
   ```

4. Execute the command in the 1<sup>st</sup> command-line terminal, and hit Enter key.

   ```py
   source ~/third_party_ros2/ros1_bridge_ws/install/setup.zsh
   ```

5. Execute the command in the 1<sup>st</sup> command-line terminal, and hit Enter key

   ```py
   ros2 run ros1_bridge parameter_bridge
   ```

6. Execute the command in the 2<sup>nd</sup> command-line terminal, and hit Enter key.

   ```py
   ros2 launch example self_driving.launch.py
   ```

7. If you need to close the program, simply click on the corresponding terminal window of the program and use the shortcut "Ctrl+C" to exit the program.

   <img class="common_img" src="../_static/media/4/section_19_1. Autonomous Driving v1.0/media/image45.png" style="width:500px" />

* **Program Outcome**

**1. Lane Keeping**

Upon initiating the game, the car will track the line and identify the yellow line at the road's edge. It will execute forward and turning actions based on the straightness or curvature of the yellow line to maintain lane position.

**2. Traffic Light Recognition**

When the car encounters a traffic light, it will halt if the light is red and proceed if it's green. Upon approaching a zebra crossing, the car will automatically decelerate and proceed cautiously.

**3. Turn and Parking Signs**

Upon detecting traffic signs while moving forward, the car will respond accordingly. If it encounters a right turn sign, it will execute a right turn and continue forward. In the case of a parking sign, it will execute a parking maneuver.

Following these rules, the robot will continuously progress forward within the map.

* **Program Analysis**

The source code of this program is saved in:

**ros2_ws/src/example/example/self_driving/self_driving.py**

<img class="common_img" src="../_static/media/4/section_19_1. Autonomous Driving v1.0/media/image46.png" style="width:500px"  />

**Function:**

```py
def main():
    node = SelfDrivingNode('self_driving')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
```

Initiate autonomous driving class.

**Class:**

```
class SelfDrivingNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        self.is_running = True
        self.pid = pid.PID(0.01, 0.0, 0.0)
        self.param_init()
```

init：

```py
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        self.is_running = True
        self.pid = pid.PID(0.01, 0.0, 0.0)
        self.param_init()

        self.image_queue = queue.Queue(maxsize=2)
        self.classes = ['go', 'right', 'park', 'red', 'green', 'crosswalk']
        self.dispaly =False
        self.bridge = CvBridge()
        self.lock = threading.RLock()
        self.colors = common.Colors()
        signal.signal(signal.SIGINT, self.shutdown)
        self.machine_type = os.environ.get('MACHINE_TYPE')
        self.lane_detect = lane_detect.LaneDetector("yellow")

        self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1) # 舵机控制(servo control)
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)

        self.create_service(Trigger, '~/enter', self.enter_srv_callback) # 进入玩法(enter game)
        self.create_service(Trigger, '~/exit', self.exit_srv_callback) # 退出玩法(exit game)
        self.create_service(SetBool, '~/set_running', self.set_running_srv_callback)
        # self.heart = Heart(self.name + '/heartbeat', 5, lambda _: self.exit_srv_callback(None))
        timer_cb_group = ReentrantCallbackGroup()
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
        self.client = self.create_client(Trigger, '/yolov5/init_finish')
        self.client.wait_for_service()
        self.start_yolov5_client = self.create_client(Trigger, '/yolov5/start', callback_group=timer_cb_group)
        self.start_yolov5_client.wait_for_service()
        self.stop_yolov5_client = self.create_client(Trigger, '/yolov5/stop', callback_group=timer_cb_group)
        self.stop_yolov5_client.wait_for_service()

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)
```

Initialize the required parameters, obtain the current robot category, set the line-following color to yellow, start the chassis control, servo control, and image reading. Set up three types of services: enter, exit, and start, and read the YOLOv5 node.

`init_process`：

```py
    def init_process(self):
        self.timer.cancel()

        self.mecanum_pub.publish(Twist())
        if not self.get_parameter('only_line_follow').value:
            self.send_request(self.start_yolov5_client, Trigger.Request())
        if self.machine_type != 'JetRover_Tank':
            set_servo_position(self.joints_pub, 1, ((10, 500), (5, 500), (4, 250), (3, 0), (2, 750), (1, 500)))  # 初始姿态(initial posture)
        else:
            set_servo_position(self.joints_pub, 1, ((10, 500), (5, 500), (4, 230), (3, 0), (2, 750), (1, 500)))  # 初始姿态(initial posture)
        time.sleep(1)
        
        if self.get_parameter('start').value:
            self.dispaly = True
            self.enter_srv_callback(Trigger.Request(), Trigger.Response())
            request = SetBool.Request()
            request.data = True
            self.set_running_srv_callback(request, SetBool.Response())

        #self.park_action() 
        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
```

Initialize the current robotic arm and start the main function.

`param_init`:

```py
    def param_init(self):
        self.start = False
        self.enter = False

        self.have_turn_right = False
        self.detect_turn_right = False
        self.detect_far_lane = False
        self.park_x = -1  # 停车标识的x像素坐标(the x-coordinate of the parking sign pixel)

        self.start_turn_time_stamp = 0
        self.count_turn = 0
        self.start_turn = False  # 开始转弯(begin turning)
```

Initialize parameters required for position recognition or usage.

`get_node_state`:

```py
    def get_node_state(self, request, response):
        response.success = True
        return response
```

Obtain the current state of the node.

`send_request`:

```py
    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()
```

Used to publish service requests.

`enter_srv_callback`:

```py
    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "self driving enter")
        with self.lock:
            self.start = False
            camera = 'depth_cam'#self.get_parameter('depth_camera_name').value
            self.create_subscription(Image, '/%s/rgb/image_raw' % camera, self.image_callback, 1)
            self.create_subscription(ObjectsInfo, '/yolov5/object_detect', self.get_object_callback, 1)
            self.mecanum_pub.publish(Twist())
            self.enter = True
        response.success = True
        response.message = "enter"
        return response
```

Service for entering autonomous driving gameplay, start reading images and YOLOv5 recognition content, initialize speed.

`exit_srv_callback`:

```py
    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "self driving exit")
        with self.lock:
            try:
                if self.image_sub is not None:
                    self.image_sub.unregister()
                if self.object_sub is not None:
                    self.object_sub.unregister()
            except Exception as e:
                self.get_logger().info('\033[1;32m%s\033[0m' % str(e))
            self.mecanum_pub.publish(Twist())
        self.param_init()
        response.success = True
        response.message = "exit"
        return response
```

Service for exiting autonomous driving gameplay, stop reading images and YOLOv5 recognition content, initialize speed, reset parameters.

`set_running_srv_callback`:

```py
    def set_running_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_running")
        with self.lock:
            self.start = request.data
            if not self.start:
                self.mecanum_pub.publish(Twist())
        response.success = True
        response.message = "set_running"
        return response
```

Start autonomous driving game, set the start parameter to True.

`Shutdown`:

```py
    def shutdown(self, signum, frame):  # ctrl+c关闭处理(press Ctrl+C to terminate the process)
        self.is_running = False
```

Callback function after closing the program, used to stop the currently running program.

`image_callback`:

```py
    def image_callback(self, ros_image):  # 目标检查回调(target inspection callback)
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面(original RGB image)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(if the queue is full, discard the oldest image)
            self.image_queue.get()
        # 将图像放入队列(put the image to the queue)
        self.image_queue.put(rgb_image)
```

Image callback function, enqueues images and discards expired ones.

`park_action`:

```py
    # 泊车处理(parking process)
    def park_action(self):
        if self.machine_type == 'JetRover_Mecanum': 
            twist = Twist()
            twist.linear.y = -0.2
            self.mecanum_pub.publish(twist)
            time.sleep(0.38/0.2)
        elif self.machine_type == 'JetRover_Acker':
            twist = Twist()
            twist.linear.x = 0.15
            twist.angular.z = twist.linear.x*math.tan(-0.6)/0.213
            self.mecanum_pub.publish(twist)
            time.sleep(3)

            twist = Twist()
            twist.linear.x = 0.15
            twist.angular.z = -twist.linear.x*math.tan(-0.6)/0.213
            self.mecanum_pub.publish(twist)
            time.sleep(2)

            twist = Twist()
            twist.linear.x = -0.15
            twist.angular.z = twist.linear.x*math.tan(-0.6)/0.213
            self.mecanum_pub.publish(twist)
            time.sleep(1.5)

            set_servo_position(self.joints_pub, 0.1, ((9, 500), ))
        else:
            twist = Twist()
            twist.angular.z = -1
            self.mecanum_pub.publish(twist)
            time.sleep(1.5)
            self.mecanum_pub.publish(Twist())
            twist = Twist()
            twist.linear.x = 0.2
            self.mecanum_pub.publish(twist)
            time.sleep(0.65/0.2)
            self.mecanum_pub.publish(Twist())
            twist = Twist()
            twist.angular.z = 1
            self.mecanum_pub.publish(twist)
            time.sleep(1.5)
        self.mecanum_pub.publish(Twist())
```

Parking logic, runs three different parking strategies according to three different chassis types.

`get_object_callback`:

```py
    # 获取目标检测结果(get target detected result)
    def get_object_callback(self, msg):
        self.objects_info = msg.objects
        if self.objects_info == []:  # 没有识别到时重置变量(reset variables when no detection is made)
            self.traffic_signs_status = None
            self.crosswalk_distance = 0
        else:
            min_distance = 0
            for i in self.objects_info:
                class_name = i.class_name
                center = (int((i.box[0] + i.box[2])/2), int((i.box[1] + i.box[3])/2))
                
                if class_name == 'crosswalk':  
                    if center[1] > min_distance:  # 获取最近的人行道y轴像素坐标(retrieve the closest pedestrian crossing y-axis pixel coordinate)
                        min_distance = center[1]
                elif class_name == 'right':  # 获取右转标识(retrieve the right turn sign)
                    self.count_right += 1
                    self.count_right_miss = 0
                    if self.count_right >= 10:  # 检测到多次就将右转标志至真(if detected multiple times, set the right turn sign to true)
                        self.turn_right = True
                        self.count_right = 0
                elif class_name == 'park':  # 获取停车标识中心坐标(retrieve the center coordinates of the stop sign)
                    self.park_x = center[0]
                elif class_name == 'red' or class_name == 'green':  # 获取红绿灯状态(get the state of traffic light)
                    self.traffic_signs_status = i
        
            self.crosswalk_distance = min_distance
```

Callback function for reading YOLOv5, obtains the categories currently recognized by YOLOv5.

Main:

```py
    def main(self):
        while self.is_running:
            time_start = time.time()
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            result_image = image.copy()
            if self.start:
                h, w = image.shape[:2]
```

The main function within the class, runs different line-following strategies according to different chassis types.

### 21.2.7 FAQ

1. The robot exhibits inconsistent performance during line patrolling, often veering off course.

   Adjust the color threshold to better suit the lighting conditions of the actual scene. For precise instructions on color threshold adjustment, please consult "**[20 ROS+OpenCV Lesson]()**" for detailed guidance.

2. The robot's turning radius appears to be either too large or too small.

1. Ensure correct adjustment of the robot arm deviation. For detailed instructions on robot arm deviation adjustment, please refer to "**[22 ROS2-Robot Arm Control Course->22.2 Robot Arm Deviation Adjustment (Optional) V1.0]()**" for comprehensive learning.

2. Modify the line patrol processing code

   Navigate to the game program path by entering the command:

   ```py
   cd ros2_ws/src/example/example/self_driving
   ```

   Open the game program by entering the command:

   ```py
   vim self_driving.py
   ```

   The red box denotes the lane's center point, which can be adjusted to fine-tune the turning effect. Decreasing the value will result in earlier turns, while increasing it will cause later turns.

   <img class="common_img" src="../_static/media/4/section_19_1. Autonomous Driving v1.0/media/image63.png" style="width:500px" />



3. The parking location is suboptimal.

   You can adjust the parking processing function or modify the starting position of the parking operation. For detailed instructions, please consult "**[21.2.5 Autonomous Parking->Parameter Adjustment]()**" for reference and learning.

4. Inaccurate traffic sign recognition.

   Adjust the target detection confidence. For detailed instructions, please refer to "**[21.2.6 Integrated Application->Operation Steps]()**" for comprehensive learning.

## 21.3 MediaPipe Man-Robot Interaction

### 21.3.1 MediaPipe **Introduction**

* **MediaPipe Description**

MediaPipe is an open-source framework of multi-media machine learning models. Cross-platform MediaPipe can run on mobile devices, workspace and servers, as well as support mobile GPU acceleration. It is also compatible with TensorFlow and TF Lite Inference Engine, and all kinds of TensorFlow and TF Lite models can be applied on it. Besides, MediaPipe supports GPU acceleration of mobile and embedded platform.

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image4.png" style="width:500px" />

* **MediaPipe Pros and Cons**

**1. MediaPipe Pros**

1)  MediaPipe supports various platforms and languages, including iOS, Android, C++, Python, JAVAScript, Coral, etc.

2)  Swift running. Models can run in real-time.

3)  Models and codes are with high reuse rate.

**2. MediaPipe Cons**

1)  For mobile devices, MediaPipe will occupy 10M or above.

2)  As it greatly depends on Tensorflow, you need to alter large amount of codes if you want to change it to other machine learning frameworks, which is not friendly to machine learning developer.

3)  It adopts static image which can improve efficiency, but make it difficult to find out the errors.

* **How to use MediaPipe**

The figure below shows how to use MediaPipe. The solid line represents the part to coded, and the dotted line indicates the part not to coded. MediaPipe can offer the result and the function realization framework quickly.

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image5.png" style="width:500px"  />

**1. Dependency**

MediaPipe utilizes OpenCV to process video, and uses FFMPEG to process audio data. Furthermore, it incorporates other essential dependencies, including OpenGL/Metal, Tensorflow, and Eigen.

For seamless usage of MediaPipe, we suggest gaining a basic understanding of OpenCV. To delve into OpenCV, you can find detailed information in "**[20 ROS2-ROS+OpenCV Course]()**".

**2. MediaPipe Solutions**

Solutions is based on the open-source pre-constructed sample of TensorFlow or TFLite. MediaPipe Solutions is built upon a framework, which provides 16 Solutions, including face detection, Face Mesh, iris, hand, posture, human body and so on.

The Solutions are developed using open-source pre-constructed samples from TensorFlow or TFLite. MediaPipe Solutions are built upon a versatile framework that offers 16 different components, like face detection, Face Mesh, iris tracking, hand tracking, posture estimation, human body tracking, and more.

* **MediaPipe Learning Resources**

MediaPipe website：https://developers.google.com/mediapipe

MediaPipe Wiki：http://i.bnu.edu.cn/wiki/index.php?title=Mediapipe

MediaPipe github：<https://github.com/google/mediapipe>

Dlib website: http://dlib.net/

dlib github: https://github.com/davisking/dlib

### 21.3.2 Image Background Segmentation

This lesson provides instructions on utilizing MediaPipe's selfie segmentation model to accurately segment trained models, such as human faces and hands, from their backgrounds. Once separated, you can easily add virtual backgrounds to these models.

* **Program Logic**

To begin, import the selfie segmentation model from MediaPipe and subscribe to the corresponding topic to access the live camera feed.

Next, flip the image and apply the segmentation to the background image. For improved boundary segmentation, implement dual-border segmentation.

Finally, complete the process by replacing the background with a virtual background.

* **Operation Steps**

> [!NOTE]
>
> **Note: the input command should be case sensitive, and keywords can be complemented using Tab key.**

1. Start the robot, and enter the robot system desktop using NoMachine.

2. Click-on <img src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image6.png" style="width:50px" /> to open the ROS1 command-line terminal.

3. Run the command to disable app auto-start app service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

4. Click-on <img src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image8.png" style="width:50px" /> to open the ROS2 command-line terminal.

5. Execute the command to enable the camera node.

   ```py
   ros2 launch peripherals depth_camera.launch.py
   ```

6. Enter the command and hit Enter key to run the game program.

   ```py
   cd ~/ros2_ws/src/example/example/mediapipe_example && python3 self_segmentation.py
   ```

7)  If you want to exit this game, please use shortcut key “Ctrl+C”. If the game cannot be closed, please retry.

8)  Next, press "**Ctrl+C**" in the terminal. If it fails to close, please try again.

* **Program Outcome**

Once the game starts, the screen will transition to a gray virtual background. As soon as a human figure appears, the program will automatically execute background removal, effectively separating the human from the virtual background.

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image11.png" style="width:500px" />

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image12.png" style="width:500px" />

* **Program Analysis**

The source code of this program locates in:

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image13.png" style="width:500px"  />

**/ros2_ws/src/example/example/mediapipe_example/self_segmentation.py**

**1. Function**

Main:

```py
def main():
    node = SegmentationNode('self_segmentation')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        print('shutdown')
    finally:
        print('shutdown finish')
```

Used to start the background control node.

**2. Class**

SegmentationNode：

```py
class SegmentationNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.running = True
        self.mp_selfie_segmentation = mp.solutions.selfie_segmentation
        self.mp_drawing = mp.solutions.drawing_utils
        self.fps = fps.FPS()
        self.image_queue = queue.Queue(maxsize=2)
        self.BG_COLOR = (192, 192, 192)  # gray
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        threading.Thread(target=self.main, daemon=True).start()
```

Init：

```
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.running = True
        self.mp_selfie_segmentation = mp.solutions.selfie_segmentation
        self.mp_drawing = mp.solutions.drawing_utils
        self.fps = fps.FPS()
        self.image_queue = queue.Queue(maxsize=2)
        self.BG_COLOR = (192, 192, 192)  # gray
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        threading.Thread(target=self.main, daemon=True).start()
```

Initialize the parameters required for background segmentation, call the image callback function, and start the model recognition function.

image_callback：

```py 
    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                               buffer=ros_image.data)  # 原始 RGB 画面(original RGB image)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(if the queue is full, discard the oldest image)
            self.image_queue.get()
            # 将图像放入队列(put the image into the queue)
        self.image_queue.put(rgb_image)
```

Image callback function, used to read data from the camera node and enqueue it.

Main:

```py
    def main(self):
        with self.mp_selfie_segmentation.SelfieSegmentation(
            model_selection=1) as selfie_segmentation:
            bg_image = None
            while self.running:
                try:
                    image = self.image_queue.get(block=True, timeout=1)
                except queue.Empty:
                    if not self.running:
                        break
                    else:
                        continue
                # To improve performance, optionally mark the image as not writeable to
                # pass by reference.
                image.flags.writeable = False
                results = selfie_segmentation.process(image)
                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                # Draw selfie segmentation on the background image.
                # To improve segmentation around boundaries, consider applying a joint
                # bilateral filter to "results.segmentation_mask" with "image".
                condition = np.stack(
                        (results.segmentation_mask,) * 3, axis=-1) > 0.1
                # The background can be customized.
                #   a) Load an image (with the same width and height of the input image) to
                #      be the background, e.g., bg_image = cv2.imread('/path/to/image/file')
                #   b) Blur the input image by applying image filtering, e.g.,
                #      bg_image = cv2.GaussianBlur(image,(55,55),0)
                if bg_image is None:
                  bg_image = np.zeros(image.shape, dtype=np.uint8)
                  bg_image[:] = self.BG_COLOR
                output_image = np.where(condition, image, bg_image)
                self.fps.update()
                result_image = self.fps.show_fps(output_image)
                cv2.imshow('MediaPipe Selfie Segmentation', result_image)
                key = cv2.waitKey(1)
                if key == ord('q') or key == 27:  # 按q或者esc退出(press q or esc to exit)
                    break
        cv2.destroyAllWindows()
        rclpy.shutdown()
```

Load the model from MediaPipe, input the image, and display the output image using OpenCV.

### 21.3.3 3D Object Detection

* **Program Logic**

To get started, import the 3D Objectron module from MediaPipe, and subscribe to the topic message to receive the real-time camera image.

Next, flip the image to ensure proper alignment for 3D object detection.

Finally, draw the 3D boundary frame on the image.

* **Operation Steps**

1. Start the robot, and enter the robot system desktop using NoMachine.

2. Click-on <img src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image6.png" style="width:50px" /> to open the ROS1 command-line terminal.

3. Run the command to disable app auto-start app service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

4. Click-on <img src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image8.png" style="width:50px" /> to open the ROS2 command-line terminal.

5. Execute the following command to enable the camera node:

   ```py
   ros2 launch peripherals depth_camera.launch.py
   ```

6. In a new ROS2 command line terminal, enter the command and press Enter to run the gameplay program:

   ```py
   cd ~/ros2_ws/src/example/example/mediapipe_example && python3 objectron.py
   ```

7)  To close this game, press the "**Esc**" key in the image interface to exit the camera image interface.

8)  Press "**Ctrl+C**" in the command line terminal interface. If the closure fails, please try again repeatedly.

* **Program Outcome**

Once the game starts, the 3D frame will be drawn around the boundary of the recognized object. The system can identify several objects, including a cup (with handle), shoe, chair, and camera.

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image20.png" style="width:500px" />

* **Program Analysis**

The program file corresponding to this section of the course documentation is located at: **/ros2_ws/src/example/example/mediapipe_example/objectron.py**

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image21.png" style="width:500px"  />

**1. Function**

Main：

```py
def main():
    node = ObjectronNode('objectron')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        print('shutdown')
    finally:
        print('shutdown finish')
```

Used to initiate 3D detection node.

**2. Class**

ObjectronNode：

```py
class ObjectronNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.running = True
        self.mp_objectron = mp.solutions.objectron
        self.mp_drawing = mp.solutions.drawing_utils
        self.fps = fps.FPS()
        self.image_queue = queue.Queue(maxsize=2)
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        threading.Thread(target=self.main, daemon=True).start()
```

Init：

```py
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.running = True
        self.mp_objectron = mp.solutions.objectron
        self.mp_drawing = mp.solutions.drawing_utils
        self.fps = fps.FPS()
        self.image_queue = queue.Queue(maxsize=2)
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        threading.Thread(target=self.main, daemon=True).start()
```

Initialize the parameters required for 3D recognition, call the image callback function, and start the model recognition function.

`image_callback`：

```py
    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                               buffer=ros_image.data)  # 原始 RGB 画面(original RGB image)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(if the queue is full, discard the oldest image)
            self.image_queue.get()
            # 将图像放入队列(put the image into the queue)
        self.image_queue.put(rgb_image)
```

Image callback function, used to read data from the camera node and enqueue it.

Main ：

```py
    def main(self):
        with self.mp_objectron.Objectron(static_image_mode=False,
                                max_num_objects=1,
                                min_detection_confidence=0.4,
                                min_tracking_confidence=0.5,
                                model_name='Cup') as objectron:
            while self.running:
                try:
                    image = self.image_queue.get(block=True, timeout=1)
                except queue.Empty:
                    if not self.running:
                        break
                    else:
                        continue
                # To improve performance, optionally mark the image as not writeable to
                # pass by reference.
                image.flags.writeable = False
                results = objectron.process(image)

                # Draw the box landmarks on the image.
                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                if results.detected_objects:
                    for detected_object in results.detected_objects:
                        self.mp_drawing.draw_landmarks(
                          image, detected_object.landmarks_2d, self.mp_objectron.BOX_CONNECTIONS)
                        self.mp_drawing.draw_axis(image, detected_object.rotation,
                                             detected_object.translation)
                self.fps.update()
                result_image = self.fps.show_fps(cv2.flip(image, 1))
                # Flip the image horizontally for a selfie-view display.
                cv2.imshow('MediaPipe Objectron', result_image)
                key = cv2.waitKey(1)
                if key == ord('q') or key == 27:  # 按q或者esc退出(press q or esc to exit)
                    break

        cv2.destroyAllWindows()
        rclpy.shutdown()
```

Read the model inside MediaPipe, input the image, draw the edges of the objects after obtaining the output image, and display using OpenCV.

### 21.3.4 Face Detection

Face detection is realized by face detection model of MediaPipe.

This model offers a swift and efficient solution for detecting faces, equipped with 6 facial landmarks to provide detailed information. It is designed to detect multiple human faces with accuracy and speed. Based on BlazeFace, it has been optimized for mobile GPU inference, ensuring a lightweight and high-performance face detection experience on mobile devices.

* **Program Logic**

To begin, import the human face detection model from MediaPipe and subscribe to the relevant topic message to obtain the live camera feed.

Next, utilize OpenCV to flip the image and convert the color space for further processing.

Then, using the face detection model's minimum confidence threshold, determine whether a human face has been successfully detected. If a human face is recognized, proceed to collect the necessary information about each detected face, including the bounding frame and the 6 key points (right eye, left eye, nose tip, right ear, and left ear).

Finally, frame the human face and mark the 6 key points on each detected face for visual clarity and further analysis.

* **Operation Steps**

> [!NOTE]
>
> **Note: the input command should be case sensitive, and keywords can be complemented using Tab key.**

1. Start the robot, and enter the robot system desktop using NoMachine.

2. Click-on <img src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image6.png" style="width:50px" /> to open ROS1 the command-line terminal.

3. Run the command to disable app auto-start app service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

4. Click-on <img src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image8.png" style="width:50px" /> to start the ROS2 command-line terminal.

5. Execute the command to enable the camera node:

   ```py
   ros2 launch peripherals depth_camera.launch.py
   ```

6. In a new ROS2 command line terminal, enter the command and press Enter to run the gameplay program:

   ```py
   cd ~/ros2_ws/src/example/example/mediapipe_example && python3 face_detect.py
   ```

7)  If you need to close this gameplay, you need to press the "Esc" key in the image interface to exit the camera image interface.

8)  Then press "**Ctrl+C**" in the command line terminal interface. If closing fails, please try again.

* **Program Outcome**

After the game starts, depth camera will start detecting human face, and human face will framed on the live camera feed.

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image27.png" style="width:500px" />

* **Program Analysis**

The source code of this program is saved in

**/ros2_ws/src/example/example/mediapipe_example/face_detect.py**

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image28.png" style="width:500px"  />

**1. Function**

Main：

```py
def main():
    node = FaceDetectionNode('face_detection')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        print('shutdown')
    finally:
        print('shutdown finish')
```

Used to initiate face detection node.

Class:

FaceDetectionNode：

```py
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.running = True
        model_path = os.path.join(os.path.abspath(os.path.split(os.path.realpath(__file__))[0]), 'model/detector.tflite')
        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.FaceDetectorOptions(base_options=base_options)
        self.detector = vision.FaceDetector.create_from_options(options)
        self.fps = fps.FPS()
        self.image_queue = queue.Queue(maxsize=2)
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        threading.Thread(target=self.main, daemon=True).start()
```

Init：

```py
        super().__init__(name)
        self.running = True
        model_path = os.path.join(os.path.abspath(os.path.split(os.path.realpath(__file__))[0]), 'model/detector.tflite')
        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.FaceDetectorOptions(base_options=base_options)
        self.detector = vision.FaceDetector.create_from_options(options)
        self.fps = fps.FPS()
        self.image_queue = queue.Queue(maxsize=2)
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        threading.Thread(target=self.main, daemon=True).start()
```

Initialize the parameters required for face recognition, call the image callback function, and start the model recognition function.

`image_callback`：

```py
    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                               buffer=ros_image.data)  # 原始 RGB 画面(original RGB image)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(if the queue is full, discard the oldest image)
            self.image_queue.get()
            # 将图像放入队列(put the image into the queue)
        self.image_queue.put(rgb_image)
```

Image callback function, used to read data from the camera node and enqueue it.

Main:

```py
    def main(self):
        while self.running:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            image = cv2.flip(image, 1)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
            detection_result = self.detector.detect(mp_image)

            annotated_image = visualize(image, detection_result)
            self.fps.update()
            result_image = self.fps.show_fps(cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR))
            cv2.imshow('face_detection', result_image)
            key = cv2.waitKey(1)
            if key == ord('q') or key == 27:  # 按q或者esc退出(press q or esc to exit)
              break

        cv2.destroyAllWindows()
        rclpy.shutdown()
```

Read the model from MediaPipe, input the image, and after obtaining the output image, use OpenCV to draw the facial keypoints and display the feedback image.

### 21.3.5 3D Face Detection

In this program, MediaPipe Face Mesh is utilized to detect human face within the camera image.

MediaPipe Face Mesh is a powerful model capable of estimating 468 3D facial features, even when deployed on a mobile device. It employs machine learning to infer the 3D face contour accurately. Additionally, this model ensures real-time detection by utilizing a lightweight model architecture and GPU acceleration.

Furthermore, this model is integrated with a face conversion module that compensates for any differences between face landmark estimation and AR (Augmented Reality) applications. It establishes a metric 3D space and utilizes the facial landmark screen positions to estimate facial conversion within this space. The facial conversion data consists of common 3D primitives, including facial gesture conversion matrices and triangle facial mesh information.

* **Program Logic**

Firstly, you need to learn that machine learning pipeline is composed of two real-time deep neural network models. The system consists of two components: a face detector that processes the entire image and calculates the locations of faces, and a 3D face landmark model that uses these locations to predict an approximate 3D surface through regression.

To achieve 3D facial landmarks, we utilize transfer learning and train a network with multiple objectives: predicting 3D landmark coordinates on synthetic rendered data and 2D semantic contours on annotated real-world data simultaneously. This approach yields plausible 3D landmark predictions not only based on synthetic data but also on real-world data.

The 3D landmark network takes cropped video frames as input without requiring additional depth input. The model outputs the location of a 3D point and the probability that a face appears in the input and is properly aligned.

Once the face mesh model is imported, real-time images can be obtained from the camera by subscribing to topic messages.

Next, image preprocessing techniques like flipping the image and converting the color space are applied. The face detection model's minimum confidence is then used to determine whether the face has been successfully detected.

Finally, the detected face on the screen is projected into a 3D grid for visualization and display.

* **Operation Steps**

> [!NOTE]
>
> **Note: the input command should be case sensitive, and keywords can be complemented using Tab key.**

1. Start the robot, and enter the robot system desktop using NoMachine.

2. Click-on <img src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image6.png" style="width:50px" /> to open ROS1 the command-line terminal.

3. Run the command to disable app auto-start app service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

4. Click-on <img src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image8.png" style="width:50px" /> to start the ROS2 command-line terminal.

5. Execute the command to enable the camera node:

   ```py
   ros2 launch peripherals depth_camera.launch.py
   ```

6. Enter the command in a new ROS2 command-line terminal and press Enter to run the game program:

   ```py
   cd ~/ros2_ws/src/example/example/mediapipe_example && python3 face_mesh.py
   ```

7)  If you need to close this gameplay, you need to press the "Esc" key in the image interface to exit the camera image interface.

8)  Then press "**Ctrl+C**" in the command line terminal interface. If closing fails, please try again.

* **Program Outcome**

After starting the game, when the depth camera detects a face, it will outline the face in the feedback image.

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image27.png" style="width:500px" />

* **Program Analysis**

The source code of this program is located in

**/ros2_ws/src/example/example/mediapipe_example/face_mesh.py**

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image28.png" style="width:500px"  />

**1. Function**

Main：

```py
def main():
    node = FaceDetectionNode('face_detection')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        print('shutdown')
    finally:
        print('shutdown finish')
```

Used to initiate the 3D face detection node.

**2. Class**

FaceMeshNode：

```py
class FaceDetectionNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.running = True
        model_path = os.path.join(os.path.abspath(os.path.split(os.path.realpath(__file__))[0]), 'model/detector.tflite')
        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.FaceDetectorOptions(base_options=base_options)
```

Init：

```py
class FaceDetectionNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.running = True
        model_path = os.path.join(os.path.abspath(os.path.split(os.path.realpath(__file__))[0]), 'model/detector.tflite')
        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.FaceDetectorOptions(base_options=base_options)
        self.detector = vision.FaceDetector.create_from_options(options)
        self.fps = fps.FPS()
        self.image_queue = queue.Queue(maxsize=2)
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        threading.Thread(target=self.main, daemon=True).start()
```

Initialize the parameters required for 3D face detection, call the image callback function, and start the model recognition function.

`image_callback`：

```py
    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                               buffer=ros_image.data)  # 原始 RGB 画面(original RGB image)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(if the queue is full, discard the oldest image)
            self.image_queue.get()
            # 将图像放入队列(put the image into the queue)
        self.image_queue.put(rgb_image)
```

The image callback function is used to retrieve data from the camera node and encapsulate it into a queue.

Main:

```py
    def main(self):
        while self.running:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            image = cv2.flip(image, 1)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
            detection_result = self.detector.detect(mp_image)

            annotated_image = visualize(image, detection_result)
            self.fps.update()
            result_image = self.fps.show_fps(cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR))
            cv2.imshow('face_detection', result_image)
            key = cv2.waitKey(1)
            if key == ord('q') or key == 27:  # 按q或者esc退出(press q or esc to exit)
              break

        cv2.destroyAllWindows()
        rclpy.shutdown()
```

Reading the model inside MediaPipe, inputting the image, and then using OpenCV to draw facial keypoints on the output image, and display the feedback image.

### 21.3.6 Hand Key Point Detection

MediaPipe's hand detection model is employed to showcase the key points of the hand and the connecting lines of these key points on the live camera feed.

MediaPipe Hands is an advanced hand and finger detection model that delivers high-fidelity results. Through the power of machine learning (ML), it accurately infers 21 3D landmarks of the hand from a single frame.

* **Program Logic**

Firstly, it's important to understand that MediaPipe's palm detection model employs a machine learning pipeline consisting of multiple models (including a linear model). This model processes the entire image and returns an oriented hand bounding box. On the other hand, the hand landmark model operates on cropped image regions defined by the palm detectors and provides high-fidelity 3D hand keypoints.

To begin, after importing the palm detection model, we subscribe to the topic message to obtain real-time camera images.

Next, various image pre-processing steps, such as flipping the image and converting the color space, are applied. These steps significantly reduce the need for data augmentation for the hand landmark model.

Furthermore, our pipeline allows for generating crops based on hand landmarks identified in the previous frame. The palm detection is invoked only when the landmark model is unable to recognize the presence of the hand accurately.

Afterward, by comparing the minimum confidence level of the hand detection model, we determine whether the palm has been successfully detected.

Lastly, the hand keypoints are detected and drawn on the camera image to visualize the detected hand in real-time.

* **Operation Steps**

1. Start the robot, and enter the robot system desktop using NoMachine.

2. Click-on <img src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image6.png" style="width:50px" /> to open ROS1 the command-line terminal.

3. Run the command to disable app auto-start app service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

4. Click-on <img src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image8.png" style="width:50px" /> to start the ROS2 command-line terminal.

5. Execute the command to enable the camera node:

   ```py
   ros2 launch peripherals depth_camera.launch.py
   ```

6. Enter the command in a new ROS2 command-line terminal and press Enter to run the game program:

   ```py
   cd ~/ros2_ws/src/example/example/mediapipe_example && python3 face_mesh.py
   ```

7)  If you need to close this gameplay, you need to press the "Esc" key in the image interface to exit the camera image interface.

Then press "Ctrl+C" in the command line terminal interface. If closing fails, please try again.6.3 Program Outcome

Once the game starts, the depth camera will begin detecting the hand and display the hand key points on the camera image, with the key points connected.

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image38.png" style="width:500px" />

* **Program Analysis**

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image39.png" style="width:500px"  />

The program file corresponding to this section of the course documentation is located at:

**/ros2_ws/src/example/example/mediapipe_example/hand.py**

**1. Function**

**Main：**

```py
def main():
    node = HandNode('hand_landmarker')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        print('shutdown')
    finally:
        print('shutdown finish')
```

Used to initiate the 3D face detection node.

**2. Class**

HandNode：

```py
class HandNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.running = True
        model_path = os.path.join(os.path.abspath(os.path.split(os.path.realpath(__file__))[0]), 'model/hand_landmarker.task')
        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.HandLandmarkerOptions(base_options=base_options,
                                       num_hands=2)
        self.detector = vision.HandLandmarker.create_from_options(options)
        self.fps = fps.FPS()
        self.image_queue = queue.Queue(maxsize=2)
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        threading.Thread(target=self.main, daemon=True).start()
```

Init：

```py
        self.detector = vision.HandLandmarker.create_from_options(options)
        self.fps = fps.FPS()
        self.image_queue = queue.Queue(maxsize=2)
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        threading.Thread(target=self.main, daemon=True).start()
        
   def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                               buffer=ros_image.data)  # 原始 RGB 画面(original RGB image)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(if the queue is full, discard the oldest image)
            self.image_queue.get()
            # 将图像放入队列(put the image into the queue)
        self.image_queue.put(rgb_image)
```

Initialize the parameters required for hand keypoint detection, call the image callback function, and start the model recognition function.

image_callback：

```py
    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                               buffer=ros_image.data)  # 原始 RGB 画面(original RGB image)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(if the queue is full, discard the oldest image)
            self.image_queue.get()
            # 将图像放入队列(put the image into the queue)
        self.image_queue.put(rgb_image)
```

The image callback function is used to read data from the camera node and enqueue it.

Main:

```py
    def main(self):
        while self.running:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            image = cv2.flip(image, 1)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
            detection_result = self.detector.detect(mp_image)
            annotated_image = draw_hand_landmarks_on_image(image, detection_result)
            self.fps.update()
            result_image = self.fps.show_fps(cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR))
            cv2.imshow('hand_landmarker', result_image)
            key = cv2.waitKey(1)
            if key == ord('q') or key == 27:  # 按q或者esc退出(press q or esc to exit)
                break
        cv2.destroyAllWindows()
        rclpy.shutdown()
```

Read the model from MediaPipe, input the image, and after obtaining the output image, use OpenCV to draw the key points of the hand and display the feedback image.

### 21.3.7 Body Key Points Detection

The MediaPipe body detection model is utilized to detect key points on the human body, which are then displayed on the live camera feed. This implementation incorporates MediaPipe Pose, a high-fidelity posture tracking model that leverages BlazePose to infer 33 3D key points. Additionally, this approach offers support for the ML Kit Pose Detection API.

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image43.png" style="width:500px" />

* **Program Logic**

Firstly, import body detection model.

Subsequently, flip over the image and convert the color space of the image. Check whether the human body is successfully detected based on the minimum confidence of the body detection model.

Next, define the tracked posture by comparing the minimum tracking confidence. If the confidence does not meet the minimum threshold, perform automatic human detection on the next input image.

In the pipeline, a detector is employed to initially localize the region of interest (ROI) corresponding to a person's pose within a frame. Subsequently, a tracker utilizes the cropped ROI frame as input to predict pose landmarks and segmentation masks within the ROI.

For video applications, the detector is invoked selectively, only when necessary. Specifically, it is used for the first frame or when the tracker fails to recognize the body pose in the preceding frame. In all other frames, the pipeline derives ROIs based on the pose landmarks detected in the previous frame.

After MediaPipe body detection model is imported, access the live camera feed through subscribing the related topic message.

Lastly, draw the key points representing the human body.

* **Operation Steps**

1. Start the robot, and enter the robot system desktop using NoMachine.

2. Click-on <img src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image6.png" style="width:50px" /> to open ROS1 the command-line terminal.

3. Run the command to disable app auto-start app service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

4. Click-on <img src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image8.png" style="width:50px" /> to start the ROS2 command-line terminal.

5. Execute the command to enable the camera node:

   ```py
   ros2 launch peripherals depth_camera.launch.py
   ```

6. Enter the command in a new ROS2 command-line terminal and press Enter to run the game program:

   ```py
   cd ~/ros2_ws/src/example/example/mediapipe_example && python3 pose.py
   ```

7)  If you need to close this gameplay, you need to press the "Esc" key in the image interface to exit the camera image interface.

8)  Then press "Ctrl+C" in the command line terminal interface. If closing fails, please try again.

* **Program Outcome**

After the game starts, depth camera will begin detecting human pose, and body key points can be displayed and connected on the live camera feed.

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image45.png" style="width:500px" />

* **Program Analysis**

The program file is saved in:

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image46.png" style="width:500px"  />

**/ros2_ws/src/example/example/mediapipe_example/pose.py**

**1. Function**

Main：

```py
def main():
    node = PoseNode('pose_landmarker')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        print('shutdown')
    finally:
        print('shutdown finish')
```

Used to initiate the 3D face detection node.

**2. Class**

PoseNode：

```py
class PoseNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.running = True
        model_path = os.path.join(os.path.abspath(os.path.split(os.path.realpath(__file__))[0]), 'model/pose_landmarker.task')
        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.PoseLandmarkerOptions(
            base_options=base_options,
            output_segmentation_masks=True)
        self.detector = vision.PoseLandmarker.create_from_options(options)
        self.fps = fps.FPS()
        self.image_queue = queue.Queue(maxsize=2)
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        threading.Thread(target=self.main, daemon=True).start()

```

Init：

```py
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.running = True
        model_path = os.path.join(os.path.abspath(os.path.split(os.path.realpath(__file__))[0]), 'model/pose_landmarker.task')
        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.PoseLandmarkerOptions(
            base_options=base_options,
            output_segmentation_masks=True)
        self.detector = vision.PoseLandmarker.create_from_options(options)
        self.fps = fps.FPS()
        self.image_queue = queue.Queue(maxsize=2)
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        threading.Thread(target=self.main, daemon=True).start()
```

Initialize the parameters required for limb detection, call the image callback function, and start the model recognition process.

`image_callback`：

```py
    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                               buffer=ros_image.data)  # 原始 RGB 画面(original RGB image)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(if the queue is full, discard the oldest image)
            self.image_queue.get()
            # 将图像放入队列(put the image into the queue)
        self.image_queue.put(rgb_image)
```

Image callback function, used to read data from the camera node and enqueue it.

Main:

```py
    def main(self):
        while self.running:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            image = cv2.flip(image, 1)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
            detection_result = self.detector.detect(mp_image)
            annotated_image = draw_pose_landmarks_on_image(image, detection_result)
            self.fps.update()
            result_image = self.fps.show_fps(cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR))
            cv2.imshow('pose_landmarker', result_image)
            key = cv2.waitKey(1)
            if key == ord('q') or key == 27:  # 按q或者esc退出(press q or esc to exit)
                break
        cv2.destroyAllWindows()
        rclpy.shutdown()
```

Read the model inside MediaPipe, input the image, and after obtaining the output image, use OpenCV to draw facial keypoints and display the feedback image.

### 21.3.8 Fingertip Trajectory Recognition

Identify hand joints using MediaPipe's hand detection model. Once a specific gesture is recognized, the robot will initiate fingertip locking on the screen, track the fingertips, and generate their movement trajectory.

* **Program Logic**

First, invoke the MediaPipe hand detection model to capture the camera image. Next, flip and process the image to extract hand information. Utilizing the connection lines between key points of the hand, calculate the finger angles to determine the gesture. Upon recognition of a specific gesture, the robot will proceed to identify and lock the fingertips on the screen, simultaneously tracing the movement trajectory of the fingertips on the display.

* **Operation Steps**

> [!NOTE]
>
> **Note: the input command should be case sensitive, and the keyword can be complemented by “Tab” key.**

1. Start the robot, and enter the robot system desktop using NoMachine.

2. Click-on <img src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image6.png" style="width:50px" /> to open ROS1 the command-line terminal.

3. Run the command to disable app auto-start app service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

4. Click-on <img src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image8.png" style="width:50px" /> to start the ROS2 command-line terminal.

5. Execute the command to enable the camera node:

   ```py
   ros2 launch peripherals depth_camera.launch.py
   ```

6. Enter the command in a new ROS2 command-line terminal and press Enter to run the game program:

   ```py
   cd ~/ros2_ws/src/example/example/mediapipe_example && python3 hand_gesture.py
   ```

7)  The program will enable the camera automatically. The detailed recognition process can be found in Program Outcome.

8)  If you need to close this gameplay, you need to press the "Esc" key in the image interface to exit the camera image interface.

9)  Then press "Ctrl+C" in the command line terminal interface. If closing fails, please try again.

* **Program Outcome**

Once the game starts, position your hand within the camera's field of view. Upon recognition, the hand keypoints will be highlighted on the camera feed.

If the robot detects the "1" gesture, the trajectory of your fingertip motion will begin to be recorded on the camera feed. If it detects the "5" gesture, the recorded fingertip trajectory will be cleared.

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image52.png" style="width:500px" />

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image53.png" style="width:500px" />

* **Program Analysis**

The program file is saved in

**/ros2_ws/src/example/example/mediapipe_example/hand_gesture.py**

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image54.png" style="width:500px"  />

> [!NOTE]
>
> **Note: Prior to making any alterations to the program, ensure to create a backup of the original factory program. Modify it only after creating the backup. Directly editing the source code file is prohibited to prevent inadvertent parameter modifications that could render the robot dysfunctional and irreparable!**

Based on the game's impact, the process logic of this game is organized as depicted in the figure below:

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image55.png" style="width:500px"  />

As depicted in the image above, the purpose of this game is to capture an image using the camera, preprocess it by converting its color space for easier identification, extract feature points corresponding to hand gestures from the converted image, and determine different gestures (based on angles) through logical analysis of key feature points. Finally, the trajectory of the recognized gesture is drawn on the display screen.

The program's logic flowchart extracted from the program files is illustrated in the figure below.

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image56.png" style="width:500px"  />

From the above diagram, it can be seen that the program's logical flow is mainly divided into initialization functions and recognition processing functions (with Xinghao being relatively important). The following document content will be written according to the program logic flow chart mentioned above.

**1. Function**

Main:

```py
def main():
    node = HandGestureNode('hand_gesture')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        print('shutdown')
    finally:
        print('shutdown finish')
```

The main function is used to start the fingertip trajectory recognition node.

`get_hand_landmarks`:

```py
def get_hand_landmarks(img, landmarks):
    """
    将landmarks从medipipe的归一化输出转为像素坐标(convert landmarks from normalized output of Mediapipe to pixel coordinates)
    :param img: 像素坐标对应的图片(the image corresponding to pixel coordinates)
    :param landmarks: 归一化的关键点(the normalized key points)
    :return:
    """
    h, w, _ = img.shape
    landmarks = [(lm.x * w, lm.y * h) for lm in landmarks]
    return np.array(landmarks)
```

Convert the normalized data from madipipe into pixel coordinates.

`hand_angle`：

```py
def hand_angle(landmarks):
    """
    计算各个手指的弯曲角度(calculate the bending angle of each finger)
    :param landmarks: 手部关键点(hand key point)
    :return: 各个手指的角度(the angle of each finger)
    """
    angle_list = []
    # thumb 大拇指
    angle_ = vector_2d_angle(landmarks[3] - landmarks[4], landmarks[0] - landmarks[2])
    angle_list.append(angle_)
    # index 食指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[6], landmarks[7] - landmarks[8])
    angle_list.append(angle_)
    # middle 中指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[10], landmarks[11] - landmarks[12])
    angle_list.append(angle_)
    # ring 无名指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[14], landmarks[15] - landmarks[16])
    angle_list.append(angle_)
    # pink 小拇指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[18], landmarks[19] - landmarks[20])
    angle_list.append(angle_)
    angle_list = [abs(a) for a in angle_list]
    return angle_list
```

After extracting the hand feature points into the 'results' variable, it is necessary to logically process these points. By evaluating the angular relationship between the feature points, specific finger types (thumb, index finger) can be identified. The `hand_angle` function accepts the landmark feature point set (results) as input, and subsequently employs the 'vector_2d_angle' function to compute the angles between the corresponding feature points. The feature points corresponding to the elements of the landmark set are depicted in the figure below:

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image60.png" style="width:500px" />

Taking the thumb's angle calculation as an example: the vector_2d_angle function is used to calculate the angle between joint points. landmarks\[3\], landmarks\[4\], landmarks\[0\], and landmarks\[2\] correspond to feature points 3, 4, 0, and 2 in the hand feature extraction diagram. By calculating the angles of these joint points, the thumb's posture characteristics can be determined. Similarly, the processing logic for the other finger joints is analogous.

To ensure the accuracy of recognition, the parameters and basic logic (addition and subtraction of angle calculations) in the hand_angle function should remain at their default settings.

`h_gesture`：

```py
def h_gesture(angle_list):
    """
    通过二维特征确定手指所摆出的手势(determining the hand gesture displayed by the fingers through two-dimensional features)
    :param angle_list: 各个手指弯曲的角度(calculate the bending angle of each finger)
    :return : 手势名称字符串(gesture name string)
    """
    thr_angle = 65.
    thr_angle_thumb = 53.
    thr_angle_s = 49.
    gesture_str = "none"
    if (angle_list[0] > thr_angle_thumb) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "fist"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "hand_heart"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] < thr_angle_s):
        gesture_str = "nico-nico-ni"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "hand_heart"
    elif (angle_list[0] > 5) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "one"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "two"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] > thr_angle):
        gesture_str = "three"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] > thr_angle) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = "OK"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = "four"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = "five"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] < thr_angle_s):
        gesture_str = "six"
    else:
        "none"
    return gesture_str
```

After identifying the different finger types of the hand and determining their positions on the image, logical recognition processing of various gestures can be performed by implementing the `h_gesture` function.

In the `h_gesture` function depicted above, the parameters `thr_angle`, `thr_angle_thenum`, and `thr_angle_s` represent the angle threshold values for corresponding gesture logic points. These values have been empirically tested to ensure stable recognition effects. It is not recommended to alter them unless the logic processing effect is unsatisfactory, in which case adjustments within a range of ±5 values are sufficient. The 'angle_list\[0,1,2,3,4\]' corresponds to the five finger types associated with the palm.

Here's an example using the gesture "one":

```py
    elif (angle_list[0] > 5) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "one"
```

The code presented represents the logical angle evaluation of the fingers for the "one" gesture. 'angle_list\[0\]\>5' checks whether the angle value of the thumb joint feature point in the image is greater than 5. 'angle_list\[1\]\<thr_angle_s' checks if the angle feature of the index finger joint feature point is less than the predetermined value 'thr_angle_s'. Similarly, 'angle_list\[2\]\<thr_angle' verifies if the angle feature of the middle finger feature point is less than the predetermined value 'thr_angle'. The logical processing for the other two fingers, 'angle_list\[3\]' and 'angle_list\[4\]', follows a similar method. When the above conditions are met, the current gesture feature is recognized as "one", and the same principle applies to recognizing other gesture features.

Different gesture recognitions involve distinct logical processing, but the overall logical framework remains similar. For recognizing other gesture features, refer to the previous paragraph.

draw_points:

```py
def draw_points(img, points, thickness=4, color=(255, 0, 0)):
    points = np.array(points).astype(dtype=np.int64)
    if len(points) > 2:
        for i, p in enumerate(points):
            if i + 1 >= len(points):
                break
            cv2.line(img, p, points[i + 1], color, thickness)
```

Draw the currently recognized hand shape and each joint point.

**2. Class**

State:

```py
class State(enum.Enum):
    NULL = 0
    START = 1
    TRACKING = 2
    RUNNING = 3
```

An enumeration class used to set the current state of the program.

HandGestureNode：

```py
class HandGestureNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.running = True
        self.drawing = mp.solutions.drawing_utils

        self.hand_detector = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_tracking_confidence=0.05,
            min_detection_confidence=0.6
        )
        
        self.fps = fps.FPS()  # fps计算器
        self.state = State.NULL
        self.points = []
        self.count = 0
        self.image_queue = queue.Queue(maxsize=2)
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        threading.Thread(target=self.main, daemon=True).start()
```

The HandGestureNode is a fingertip trajectory recognition node that contains three functions: an initialization function, a main function, and an image callback function.

Init:

```py
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.running = True
        self.drawing = mp.solutions.drawing_utils

        self.hand_detector = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_tracking_confidence=0.05,
            min_detection_confidence=0.6
        )
        
        self.fps = fps.FPS()  # fps计算器
        self.state = State.NULL
        self.points = []
        self.count = 0
        self.image_queue = queue.Queue(maxsize=2)
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        threading.Thread(target=self.main, daemon=True).start()
```

Initialize each component needed and call the camera node.

### 21.3.9 Posture Control

The human posture estimation model, trained using the MediaPipe machine learning framework, detects the human body feature in the captured image, identifies relevant joint positions, and subsequently recognizes a variety of sequential actions. This process enables direct control of the robot through somatosensory input.

Viewed from the perspective of the robot, the following actions correspond to specific movements:

1. If the user lifts their left arm, the robot will move a certain distance to the right.

2. If the user lifts their right arm, the robot will move a certain distance to the left.

3. If the user lifts their left leg, the robot will move forward a certain distance.

4. If the user lifts their right leg, the robot will move backward a certain distance.

* **Program Logic**

First, import MediaPipe's human pose estimation model and subscribe to topic messages to obtain real-time footage from the camera.

MediaPipe is an open-source multimedia machine learning model application framework that runs cross-platform on mobile devices, workstations, and servers. It supports mobile GPU acceleration and inference engines such as TensorFlow and TF Lite.

Next, utilize the built model to detect key points of the human body in the screen. Connect these key points to display the human body and determine the human body posture.

Finally, if a specific action is detected in the human body posture, the robot will respond accordingly.

* **Operation Steps**

> [!NOTE]
>
> **Note: the input command should be case sensitive, and keywords can be complemented using Tab key.**

1. Start the robot, and enter the robot system desktop using NoMachine.

2. Click-on <img src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image6.png" style="width:50px" /> to open ROS1 the command-line terminal.

3. Run the command to disable app auto-start app service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

4. Click-on <img src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image8.png" style="width:50px" /> to start the ROS2 command-line terminal.

5. Execute the command to run the game program:

   ```py
   ros2 launch example body_control.launch.py
   ```

6)  The program will launch the camera's image interface. For specific recognition steps, refer to Section Program Outcome.

7)  If you need to close this gameplay, you need to press the "Esc" key in the image interface to exit the camera image interface.

8)  Then press "Ctrl+C" in the command line terminal interface. If closing fails, please try again.

* **Program Outcome**

Once the game is initiated, stand within the camera's field of view. When a person is detected, the screen will display key points of the body and lines connecting them.

From the perspective of the robot, lifting the left arm will cause the robot to turn left; lifting the right arm will make the robot turn right; lifting the left leg will make the robot move forward a certain distance; lifting the right leg will make the robot move backward a certain distance.

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image68.png" style="width:500px" />

* **Program Analysis**

The program file is saved in

**ros2_ws/src/example/example/body_control/include/body_control.py**

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image69.png" style="width:500px"  />

> [!NOTE]
>
> **Note: Prior to making any alterations to the program, ensure to create a backup of the original factory program. Modify it only after creating the backup. Directly editing the source code file is prohibited to prevent inadvertent parameter modifications that could render the robot dysfunctional and irreparable!**

The game process logic is outlined below:

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image70.png" style="width:500px"  />

1.  Capture an image through the camera.

2.  After performing a demonstration action, the car will execute the corresponding action.

3.  From the car's perspective, lifting the left arm will cause the car to turn left; lifting the right arm will cause the car to turn right in a circle; lifting the left leg will make the car move forward a certain distance; lifting the right leg will make the car move backward a certain distance.

The program's logic flowchart, obtained from the program files, is presented below:

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image71.png" style="width:500px"  />

1.  Initialization function (init(self.name)) defines relevant parameters, including:

- Definition of the image tool (self.drawing) object.

- Points used to draw recognized features.

- Definition of the limb detection object (self.body_detector).

2.  Identified feature points' output results undergo logical processing for recognition.

3.  Actions are determined and stored based on key point distance conditions.

4.  Finally, the output results are generated, and the car executes corresponding actions.

**1. Function**

Main：

```py
def main():
    node = BodyControlNode('body_control')
    rclpy.spin(node)
    node.destroy_node()
```

Used to start the body sensation control node.

`get_joint_landmarks`：

```py
def get_joint_landmarks(img, landmarks):
    """
    将landmarks从medipipe的归一化输出转为像素坐标(Convert landmarks from medipipe's normalized output to pixel coordinates)
    :param img: 像素坐标对应的图片(picture corresponding to pixel coordinate)
    :param landmarks: 归一化的关键点(normalized keypoint)
    :return:
    """
    h, w, _ = img.shape
    landmarks = [(lm.x * w, lm.y * h) for lm in landmarks]
    return np.array(landmarks)

```

Used to convert the recognized information into pixel coordinates.

`joint_distance`：

```py
def joint_distance(landmarks):
    distance_list = []

    d1 = landmarks[LEFT_HIP] - landmarks[LEFT_SHOULDER]
    d2 = landmarks[LEFT_HIP] - landmarks[LEFT_WRIST]
    dis1 = d1[0]**2 + d1[1]**2
    dis2 = d2[0]**2 + d2[1]**2
    distance_list.append(round(dis1/dis2, 1))
   
    d1 = landmarks[RIGHT_HIP] - landmarks[RIGHT_SHOULDER]
    d2 = landmarks[RIGHT_HIP] - landmarks[RIGHT_WRIST]
    dis1 = d1[0]**2 + d1[1]**2
    dis2 = d2[0]**2 + d2[1]**2
    distance_list.append(round(dis1/dis2, 1))
    
    d1 = landmarks[LEFT_HIP] - landmarks[LEFT_ANKLE]
    d2 = landmarks[LEFT_ANKLE] - landmarks[LEFT_KNEE]
    dis1 = d1[0]**2 + d1[1]**2
    dis2 = d2[0]**2 + d2[1]**2
    distance_list.append(round(dis1/dis2, 1))
   
    d1 = landmarks[RIGHT_HIP] - landmarks[RIGHT_ANKLE]
    d2 = landmarks[RIGHT_ANKLE] - landmarks[RIGHT_KNEE]
    dis1 = d1[0]**2 + d1[1]**2
    dis2 = d2[0]**2 + d2[1]**2
    distance_list.append(round(dis1/dis2, 1))
    
    return distance_list
```

Used to calculate the distance between each joint point based on pixel coordinates.

**2. Class**

```py
class BodyControlNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        self.drawing = mp.solutions.drawing_utils
        self.body_detector = mp_pose.Pose(
            static_image_mode=False,
            min_tracking_confidence=0.5,
            min_detection_confidence=0.5)
        self.running = True
        self.fps = fps.FPS()  # fps计算器(fps calculator)
        signal.signal(signal.SIGINT, self.shutdown)

        self.move_finish = True
        self.stop_flag = False
        self.left_hand_count = []
        self.right_hand_count = []
        self.left_leg_count = []
        self.right_leg_count = []

```

This class is the body control node.

Init:

```py
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        self.drawing = mp.solutions.drawing_utils
        self.body_detector = mp_pose.Pose(
            static_image_mode=False,
            min_tracking_confidence=0.5,
            min_detection_confidence=0.5)
        self.running = True
        self.fps = fps.FPS()  # fps计算器(fps calculator)
        signal.signal(signal.SIGINT, self.shutdown)
```

Initialize the parameters required for body control, read the image callback node from the camera, initialize nodes such as servos, chassis, buzzers, motors, etc., and finally start the main function within the class.4

`get_node_state`：

```py
    def get_node_state(self, request, response):
        response.success = True
        return response
```

Set the initialization state of the current node.

shutdown：

```py
    def shutdown(self, signum, frame):
        self.running = False
```

Program exit callback function used to terminate recognition.

`image_callback`：

```py
    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)  # 将自定义图像消息转化为图像(convert the custom image information to image)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(discard the oldest image if the queue is full)
            self.image_queue.get()
        # 将图像放入队列(put the image into the queue)
        self.image_queue.put(rgb_image)
```

Image node callback function used to process images and enqueue them.

Move:

```py
    def move(self, *args):
        if args[0].angular.z == 1:
            set_servos(self.joints_pub, 0.1, ((9, 650), ))
            time.sleep(0.2)
            motor1 = MotorState()
            motor1.id = 2
            motor1.rps = -0.1
            motor2 = MotorState()
            motor2.id = 4
            motor2.rps = 1
            self.motor_pub.publish([motor1, motor2])
            time.sleep(7.5)
            set_servos(self.joints_pub, 0.1, ((9, 500), ))
            motor1 = MotorState()
            motor1.id = 2
            motor1.rps = 0
            motor2 = MotorState()
            motor2.id = 4
            motor2.rps = 0
            self.motor_pub.publish([motor1, motor2])
        elif args[0].angular.z == -1:
            set_servos(self.joints_pub, 0.1, ((9, 350), ))
            time.sleep(0.2)
            motor1 = MotorState()
            motor1.id = 2
            motor1.rps = 0.1
            motor2 = MotorState()
            motor2.id = 4
            motor2.rps = -1
            self.motor_pub.publish([motor1, motor2])
            time.sleep(8)
            set_servos(self.joints_pub, 0.1, ((9, 500), ))
            motor1 = MotorState()
            motor1.id = 2
            motor1.rps = 0
            motor2 = MotorState()
            motor2.id = 4
            motor2.rps = 0
            self.motor_pub.publish([motor1, motor2])
        else:
            self.mecanum_pub.publish(args[0])
            time.sleep(args[1])
            self.mecanum_pub.publish(Twist())
            time.sleep(0.1)
        self.stop_flag =True
        self.move_finish = True
```

Movement strategy function that moves the vehicle according to the recognized limb actions.

`buzzer_warn`:

```py
    def buzzer_warn(self):
        msg = BuzzerState()
        msg.freq = 1900
        msg.on_time = 0.2
        msg.off_time = 0.01
        msg.repeat = 1
        self.buzzer_pub.publish(msg)
```

Buzzer control function used for buzzer alarms.

`image_proc`:

```py
    def image_proc(self, image):
        image_flip = cv2.flip(cv2.cvtColor(image, cv2.COLOR_RGB2BGR), 1)
        results = self.body_detector.process(image)
        if results is not None and results.pose_landmarks is not None:
            if self.move_finish:
                twist = Twist()
                landmarks = get_joint_landmarks(image, results.pose_landmarks.landmark)
                distance_list = (joint_distance(landmarks))
              
                if distance_list[0] < 1:
                    self.detect_status[0] = 1
                if distance_list[1] < 1:
                    self.detect_status[1] = 1
                if 0 < distance_list[2] < 2:
                    self.detect_status[2] = 1
                if 0 < distance_list[3] < 2:
                    self.detect_status[3] = 1
```

Function for recognizing limbs, which invokes the model to draw key points of the human body based on the recognized information, and then performs movements according to the recognized posture.

Main:

```py
    def main(self):
        while self.running:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            try:
                result_image = self.image_proc(np.copy(image))
            except BaseException as e:
                self.get_logger().info('\033[1;32m%s\033[0m' % e)
                result_image = cv2.flip(cv2.cvtColor(image, cv2.COLOR_RGB2BGR), 1)
            self.fps.update()
            result_image = self.fps.show_fps(result_image)
            cv2.imshow(self.name, result_image)
            key = cv2.waitKey(1)
            if key == ord('q') or key == 27:  # 按q或者esc退出(press q or esc to exit)
                self.mecanum_pub.publish(Twist())
                self.running = False
```

The main function within the BodyControlNode class, used to input image information into the recognition function and display the returned image.

### 21.3.10 Human Body Tracking

> [!NOTE]
>
> **Note: This game is best suited for indoor environments. Outdoor settings may significantly interfere with its effectiveness!**

Utilize the yolov5 framework to import a pre-trained human pose model for detecting human bodies. The center point of the detected human body will be indicated in the returned image. When a human body approaches, the robot will retreat; conversely, if the human body is distant, the robot will move forward. This ensures that the distance between the human body and the robot remains approximately 3 meters at all times.

* **Program Logic**

First, import the human pose estimation model from yolov5, and subscribe to topic messages to obtain real-time camera images. Next, utilize the trained model to detect key points of the human body in the images, and calculate the coordinates of the human body's center point based on all detected key points. Finally, update the PID controller based on the coordinates of the human body's center point and the screen's center point to control the robot's movement in sync with the human body's movement.

* **Operation Steps**

> [!NOTE]
>
> **Note: When entering commands, strict case sensitivity is required, and you can use the "Tab" key to complete keywords.**
>
> **As ROS2 is placed within a container and cannot directly access the GPU of the mainboard, it needs to utilize ROS1 to start the mainboard's GPU, and then transfer the recognition information to ROS2 for use.**

1. Start the robot, and enter the robot system desktop using NoMachine.

2. Click-on <img src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image6.png" style="width:50px" /> to open ROS1 the command-line terminal.

3. Run the command to disable app auto-start app service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

4. Enter the command and press Enter to start the human tracking game:

   ```py
   roslaunch hiwonder_example body_track_base.launch
   ```

5. Click-on <img src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image85.png" style="width:50px" /> to start the ROS2 command-line terminal.

6. Enter the command to update the environment for bridging information between ROS1 and ROS2.

   ```py
   source ~/third_party_ros2/ros1_bridge_ws/install/setup.sh
   ```

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image86.png" style="width:500px" />

If the above prompt appears, you can ignore it without affecting the normal functionality.

7. Run the following command and hit Enter key:

   ```py
   ros2 run ros1_bridge parameter_bridge
   ```

8. Open a new ROS2 command line terminal, enter the command, and press Enter:

   ```py
   ros2 launch example body_track.launch.py
   ```

9. Finally, open a new command line terminal and launch the rqt tool to view the recognition screen.

   ```py
   rqt
   ```

10) The program will launch the camera's image interface. For specific recognition steps, refer to Section Program Outcome.

11) If you need to close this gameplay, you need to press the "Esc" key in the image interface to exit the camera image interface.

12) Then press "Ctrl+C" in the command line terminal interface. If closing fails, please try again.

* **Program Outcome**

Upon initiating the game, the camera captures the human body within its field of view. Once detected, the center point of the human body is highlighted in the displayed image.

From the robot's perspective, if the human body is in close proximity, the robot will retreat. Conversely, if the human body is distant, the robot will move forward, ensuring that the distance between the human body and the robot remains approximately 3 meters at all times.

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image90.png" style="width:500px" />

* **Program Analysis**

The program file is saved in

**ros2_ws/src/example/example/body_control/include/body_track.py**

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image91.png" style="width:500px"  />

> [!NOTE]
>
> **Note: Prior to making any alterations to the program, ensure to create a backup of the original factory program. Modify it only after creating the backup. Directly editing the source code file is prohibited to prevent inadvertent parameter modifications that could render the robot dysfunctional and irreparable!**

Based on the game's effectiveness, the procedural logic is delineated as follows:

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image92.png" style="width:500px"  />

1.  The car captures images through the camera.

2.  It identifies the human body's position in the image and calculates the distance between the human body and the car.

3.  Finally, the car adjusts its movement to follow within the preset distance limit.

The program's logic flowchart, derived from the program files, is illustrated below:

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image93.png" style="width:500px"  />

Initialization Function of the BodyControlNode Class:

**Configuration of Linear and Angular Speed**: Adjusts the car's speed based on the detected distance from the human body.

**Definition of Chassis Publisher**: Publishes the chassis's position and orientation to control the car's forward and backward movement.

**Yolov5 Human Body Detection**: Utilizes the yolov5 single-target detection algorithm to detect human bodies and obtain their positions.

**PID Parameter Initialization**: Initializes parameters related to the PID controller, governing sensitivity to the distance between the car and the human body.

**1. Function**

Main：

```py
def main():
    node = BodyControlNode('body_control')
    rclpy.spin(node)
    node.destroy_node()
```

Used to start the human tracking node.

**2. Class**

```py
class BodyControlNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
       
        self.pid_d = pid.PID(0.1, 0, 0)
        #self.pid_d = pid.PID(0, 0, 0)
        
        self.pid_angular = pid.PID(0.002, 0, 0)
        #self.pid_angular = pid.PID(0, 0, 0)
```

This class is the human tracking node.

Init:

```py
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
       
        self.pid_d = pid.PID(0.1, 0, 0)
        #self.pid_d = pid.PID(0, 0, 0)
        
        self.pid_angular = pid.PID(0.002, 0, 0)
        #self.pid_angular = pid.PID(0, 0, 0)
        
        self.go_speed, self.turn_speed = 0.007, 0.04
        self.linear_x, self.angular = 0, 0
        self.running = True
```

Initialize the parameters required for human tracking, read nodes such as the camera's image callback, depth information, chassis, YOLOv5 recognition, etc., and then synchronize the time to align depth information and image information, finally start the main function within the class.

`get_node_state`：

```py
    def get_node_state(self, request, response):
        response.success = True
        return response
```

Set the initialization state of the current node.

shutdown：

```py
    def shutdown(self, signum, frame):
        self.running = False
```

Program exit callback function used to terminate recognition.

`get_object_callback`：

```py
    # 获取目标检测结果(get the results of target detection)
    def get_object_callback(self, msg):
        for i in msg.objects:
            class_name = i.class_name
            if class_name == 'person':
                if i.box[1] < 10:
                    self.center = [int((i.box[0] + i.box[2])/2), int(i.box[1]) + abs(int((i.box[1] - i.box[3])/4))]
                else:
                    self.center = [int((i.box[0] + i.box[2])/2), int(i.box[1]) + abs(int((i.box[1] - i.box[3])/3))]
```

Callback function for YOLOv5 recognition node, which converts recognition information into pixel coordinates.

`image_proc`:

```py
    def image_proc(self, bgr_image):
        twist = Twist()
        if self.center is not None:
            h, w = bgr_image.shape[:-1]
            cv2.circle(bgr_image, tuple(self.center), 10, (0, 255, 255), -1) 
            #################
            roi_h, roi_w = 5, 5
            w_1 = self.center[0] - roi_w
            w_2 = self.center[0] + roi_w
            if w_1 < 0:
                w_1 = 0
            if w_2 > w:
                w_2 = w
            h_1 = self.center[1] - roi_h
            h_2 = self.center[1] + roi_h
            if h_1 < 0:
                h_1 = 0
            if h_2 > h:
                h_2 = h
```

Function for tracking humans, which invokes the model to draw the position of the person based on the recognized information, and uses PID to control the movement according to the recognized person.

Main：

```py
    def main(self):
        while self.running:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            try:
                result_image = self.image_proc(image)
            except BaseException as e:
                self.get_logger().info('\033[1;32m%s\033[0m' % e)
            self.center = None
            # cv2.imshow(self.name, result_image)
            key = cv2.waitKey(1)
            if key == ord('q') or key == 27:  # 按q或者esc退出(press q or esc to exit)
                self.mecanum_pub.publish(Twist())
                self.running = False
```

The main function within the BodyControlNode class, used to input image information into the recognition function and display the returned image.

* **Function Expansion**

**The default tracking speed in the program is set to a fixed value. If you wish to alter the tracking speed of the robot, you can do so by adjusting the PID parameters within the program.**

1. Open the ROS1 terminal, enter the command to navigate to the directory where the program is stored:

   ```py
   cd /home/ubuntu/ros2_ws/src/example/example/body_control/include/
   ```

2. Run the following command to open the program file:

   ```py
   sudo vim body_control.py
   ```

3. Locate the `self.pid_d` and `self.pid_angular` functions, where the values inside the parentheses are the PID-related parameters. One is for the tracking linear velocity PID, and the other is for the tracking angular velocity PID.

   <img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image103.png" style="width:500px" />

   The three PID parameters are proportional, integral, and derivative. The proportional parameter adjusts the response level, the integral parameter adjusts the smoothness, and the derivative parameter adjusts whether there is overshoot.

4. Press "i" to enter edit mode. If you want to increase the robot's tracking speed, you can correspondingly increase the value. For example, here we set the tracking linear velocity PID to 0.05.

   > [!NOTE]
   >
   > **Note: It is recommended not to adjust the parameters too high. Excessive parameters will cause the robot to track too quickly, affecting the experience.**

   <img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image104.png" style="width:500px" />

5. After completing the modifications, press "Esc" to exit edit mode, then press ":wq" to save and exit.

   <img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image105.png" style="width:500px" />

6. Start the game according to the instructions provided in section 10.2.

### 21.3.11 Integration of Body Posture and RGB Control

The depth camera combines RGB capabilities, enabling both color recognition and somatosensory control. This lesson will leverage color recognition, as discussed in "**[21.3.9 Posture Control]()**", to identify individuals wearing clothing of a predefined color (which can be calibrated through operations). The robot's movements are then controlled based on different body gestures.

If an individual wearing the specified color is not recognized, the robot remains unresponsive, ensuring accurate identification and control over the individual operating the robot.

* **Program Logic**

First, import MediaPipe's human pose estimation model and subscribe to topic messages to obtain real-time camera footage.

Next, utilize the built model to detect key points of the human torso on the screen. Connect these key points to display the human torso and determine the human posture. Calibrate the center point of the human body based on all key points.

Finally, if it is detected that the human body is standing with hands on hips, calibrate the color of the clothes to identify the control object. The robot enters control mode, and when the human body performs specific actions, the robot responds with corresponding actions.

* **Operation Steps**

> [!NOTE]
>
> **Note: the input command should be case sensitive, and keywords can be complemented using Tab key.**

1. Start the robot, and enter the robot system desktop using NoMachine.

2. Click-on <img src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image6.png" style="width:50px" /> to open ROS1 the command-line terminal.

3. Run the command to disable app auto-start app service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

4. Click-on <img src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image85.png" style="width:50px" /> to start the ROS2 command-line terminal.

5. Run the following command and hit Enter key to initiate the game:

   ```py
   ros2 launch example body_and_rgb_control.launch.py
   ```

6)  The program will launch the camera's image interface. For specific recognition steps, refer to Section Program Outcome.

7)  If you need to close this gameplay, you need to press the "Esc" key in the image interface to exit the camera image interface.

8)  Then press "Ctrl+C" in the command line terminal interface. If closing fails, please try again.

* **Program Outcome**

After starting the gameplay, stand within the camera's field of view. When a person is detected, the screen will display key points of the human torso, lines connecting these points, and the center point of the human body.

**First Step:** Slightly adjust the camera to maintain a certain distance, ensuring it can detect the entire human body.

**Second Step:** When the person to be controlled appears on the camera screen, they can assume the posture of hands on hips. If the buzzer emits a short beep, the robot completes calibration of the human body's center point and clothing color, entering control mode.

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image107.png" style="width:500px" />

**Third Step:** At this point, with the robot as the primary viewpoint, raising the left arm causes the robot to move to the right; raising the right arm causes the robot to move to the left; raising the left leg causes the robot to move forward; raising the right leg causes the robot to move backward.

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image108.png" style="width:500px" />

**Fourth Step:** If a person wearing a different clothing color enters the camera's field of view, they will not control the robot.

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image109.png" style="width:500px" />

* **Program Analysis**

The program file is saved in:

**ros2_ws/src/example/example/body_control/include/body_and_rgb_control.py**

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image110.png" style="width:500px"  />

> [!NOTE]
>
> **Note: Prior to making any alterations to the program, ensure to create a backup of the original factory program. Modify it only after creating the backup. Directly editing the source code file is prohibited to prevent inadvertent parameter modifications that could render the robot dysfunctional and irreparable!**

Based on the game's effectiveness, the procedural logic is delineated as follows:

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image111.png" style="width:400px"  />

Retrieve the captured image from the camera, analyze the key characteristics of the human body, initially detect and assess the "**akimbo**" posture using the designated function. Subsequently, determine if it's the same individual in the picture based on clothing color. If confirmed, identify specific body movements (raising the left arm, right arm, left leg, and right leg) and instruct the car to execute corresponding actions. Otherwise, reanalyze the key feature points.

The program's logic flowchart, derived from the program files, is illustrated below:

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image112.png" style="width:500px"  />

As shown in the above diagram, the program first uses the initialization function of the BodyControlNode class to set the default values for the relevant parameters. This primarily includes defining the drawing tool object, the limb detection object, and initializing the posture state and count parameters. After this setup, the program can logically process the incoming images. It begins by identifying and outputting key body feature points, calculating joint angles from the obtained (arm) parameters to mark the hands-on-hips posture. Then, it matches colors based on the recognized key feature points to determine if the person has been previously marked. Finally, the program controls the movement of the vehicle by recognizing demonstration actions (raising an arm, lifting a leg).

**1. Function**

Main：

```py
def main():
    node = BodyControlNode('body_control')
    rclpy.spin(node)
    node.destroy_node()
```

Used to start the RGB body sensation control node.

`get_body_center`：

```py
def get_body_center(h, w, landmarks):
    landmarks = np.array([(lm.x * w, lm.y * h) for lm in landmarks])
    center = ((landmarks[LEFT_HIP] + landmarks[LEFT_SHOULDER] + landmarks[RIGHT_HIP] + landmarks[RIGHT_SHOULDER])/4).astype(int)
    return center.tolist()
```

Used to obtain the currently recognized body contours.

`get_joint_landmarks`：

```py
def get_joint_landmarks(img, landmarks):
    """
    将landmarks从medipipe的归一化输出转为像素坐标(Convert landmarks from medipipe's normalized output to pixel coordinates)
    :param img: 像素坐标对应的图片(picture corresponding to pixel coordinate)
    :param landmarks: 归一化的关键点(normalized keypoint)
    """
    h, w, _ = img.shape
    landmarks = [(lm.x * w, lm.y * h) for lm in landmarks]
    return np.array(landmarks)
```

Used to convert the recognized information into pixel coordinates.

`get_dif`：

```py
def get_dif(list1, list2):
    if len(list1) != len(list2):
        return 255*3
    else:
        d = np.absolute(np.array(list1) - np.array(list2))
        return sum(d)
```

Used to compare the color of clothes on the body contours.

`joint_angle`:

```py
def joint_angle(landmarks):
    """
    计算各个关节弯曲角度(calculate flex angle of each joint)
    :param landmarks: 手部关键点(hand keypoints)
    :return: 关节角度(joint angle)
    """
    angle_list = []
    left_hand_angle1 = vector_2d_angle(landmarks[LEFT_SHOULDER] - landmarks[LEFT_ELBOW], landmarks[LEFT_WRIST] - landmarks[LEFT_ELBOW])
    angle_list.append(int(left_hand_angle1))
   
    left_hand_angle2 = vector_2d_angle(landmarks[LEFT_HIP] - landmarks[LEFT_SHOULDER], landmarks[LEFT_WRIST] - landmarks[LEFT_SHOULDER])
    angle_list.append(int(left_hand_angle2))

    right_hand_angle1 = vector_2d_angle(landmarks[RIGHT_SHOULDER] - landmarks[RIGHT_ELBOW], landmarks[RIGHT_WRIST] - landmarks[RIGHT_ELBOW])
    angle_list.append(int(right_hand_angle1))

    right_hand_angle2 = vector_2d_angle(landmarks[RIGHT_HIP] - landmarks[RIGHT_SHOULDER], landmarks[RIGHT_WRIST] - landmarks[RIGHT_SHOULDER])
    angle_list.append(int(right_hand_angle2))
    
    return angle_list
```

This function is used to calculate the recognition angles of various joints between the body parts.

`joint_distance`:

```py
def joint_distance(landmarks):
    distance_list = []

    d1 = landmarks[LEFT_HIP] - landmarks[LEFT_SHOULDER]
    d2 = landmarks[LEFT_HIP] - landmarks[LEFT_WRIST]
    dis1 = d1[0]**2 + d1[1]**2
    dis2 = d2[0]**2 + d2[1]**2
    distance_list.append(round(dis1/dis2, 1))
   
    d1 = landmarks[RIGHT_HIP] - landmarks[RIGHT_SHOULDER]
    d2 = landmarks[RIGHT_HIP] - landmarks[RIGHT_WRIST]
    dis1 = d1[0]**2 + d1[1]**2
    dis2 = d2[0]**2 + d2[1]**2
    distance_list.append(round(dis1/dis2, 1))
    
    d1 = landmarks[LEFT_HIP] - landmarks[LEFT_ANKLE]
    d2 = landmarks[LEFT_ANKLE] - landmarks[LEFT_KNEE]
    dis1 = d1[0]**2 + d1[1]**2
    dis2 = d2[0]**2 + d2[1]**2
    distance_list.append(round(dis1/dis2, 1))
   
    d1 = landmarks[RIGHT_HIP] - landmarks[RIGHT_ANKLE]
    d2 = landmarks[RIGHT_ANKLE] - landmarks[RIGHT_KNEE]
    dis1 = d1[0]**2 + d1[1]**2
    dis2 = d2[0]**2 + d2[1]**2
    distance_list.append(round(dis1/dis2, 1))
    
    return distance_list
```

This function is used to calculate the distance between each joint point based on pixel coordinates.

**2. Class**

```py
class BodyControlNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.name = name
        self.drawing = mp.solutions.drawing_utils
        self.body_detector = mp_pose.Pose(
            static_image_mode=False,
            min_tracking_confidence=0.5,
            min_detection_confidence=0.5)
        
        self.color_picker = ColorPicker(Point(), 2)
        signal.signal(signal.SIGINT, self.shutdown)
        self.fps = fps.FPS()  # fps计算器(fps calculator)
```

This class is the body control node.

Init:

```py
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.name = name
        self.drawing = mp.solutions.drawing_utils
        self.body_detector = mp_pose.Pose(
            static_image_mode=False,
            min_tracking_confidence=0.5,
            min_detection_confidence=0.5)
        
        self.color_picker = ColorPicker(Point(), 2)
        signal.signal(signal.SIGINT, self.shutdown)
        self.fps = fps.FPS()  # fps计算器(fps calculator)

```

Initialize the parameters required for body control, read the camera's image callback node, initialize nodes such as servos, chassis, buzzers, motors, etc., and finally start the main function within the class.

`get_node_state`:

```py
    def get_node_state(self, request, response):
        response.success = True
        return response
```

Set the initialization state of the current node.

shutdown:

```py
    def shutdown(self, signum, frame):
        self.running = False
```

Program exit callback function used to terminate recognition.

`image_callback`:

```py
    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)  # 将自定义图像消息转化为图像(convert the custom image information to image)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(discard the oldest image if the queue is full)
            self.image_queue.get()
        # 将图像放入队列(put the image into the queue)
        self.image_queue.put(rgb_image)
```

Image node callback function used to process images and enqueue them.

Move:

```py
    def move(self, *args):
        if args[0].angular.z == 1:
            set_servos(self.joints_pub, 0.1, ((9, 650), ))
            time.sleep(0.2)
            motor1 = MotorState()
            motor1.id = 2
            motor1.rps = -0.1
            motor2 = MotorState()
            motor2.id = 4
            motor2.rps = 1
            self.motor_pub.publish([motor1, motor2])
            time.sleep(7.5)
            set_servos(self.joints_pub, 0.1, ((9, 500), ))
            motor1 = MotorState()
            motor1.id = 2
            motor1.rps = 0
            motor2 = MotorState()
            motor2.id = 4
            motor2.rps = 0
            self.motor_pub.publish([motor1, motor2])
        elif args[0].angular.z == -1:
            set_servos(self.joints_pub, 0.1, ((9, 350), ))
            time.sleep(0.2)
            motor1 = MotorState()
            motor1.id = 2
            motor1.rps = 0.1
            motor2 = MotorState()
            motor2.id = 4
            motor2.rps = -1
            self.motor_pub.publish([motor1, motor2])
            time.sleep(8)
            set_servos(self.joints_pub, 0.1, ((9, 500), ))
            motor1 = MotorState()
            motor1.id = 2
            motor1.rps = 0
            motor2 = MotorState()
            motor2.id = 4
            motor2.rps = 0
            self.motor_pub.publish([motor1, motor2])
        else:
            self.mecanum_pub.publish(args[0])
            time.sleep(args[1])
            self.mecanum_pub.publish(Twist())
            time.sleep(0.1)
        self.stop_flag =True
        self.move_finish = True
```

Movement strategy function that moves the vehicle according to the recognized limb actions.

`buzzer_warn`:

```py
    def buzzer_warn(self):
        msg = BuzzerState()
        msg.freq = 1900
        msg.on_time = 0.2
        msg.off_time = 0.01
        msg.repeat = 1
        self.buzzer_pub.publish(msg)
```

Buzzer control function used for buzzer alarms.

`image_proc`:

```py
    def image_proc(self, image):
        image_flip = cv2.flip(cv2.cvtColor(image, cv2.COLOR_RGB2BGR), 1)
        results = self.body_detector.process(image)
        if results is not None and results.pose_landmarks is not None:
            twist = Twist()
            
            landmarks = get_joint_landmarks(image, results.pose_landmarks.landmark)
            
            # 叉腰标定
            angle_list = joint_angle(landmarks)
            #print(angle_list)
            if -150 < angle_list[0] < -90 and -30 < angle_list[1] < -10 and 90 < angle_list[2] < 150 and 10 < angle_list[3] < 30:
                self.count_akimbo += 1  # 叉腰检测+1(hands-on-hips detection+1)
                self.count_no_akimbo = 0  # 没有叉腰检测归零(clear no hands-on-hips detection)
            else:
                self.count_akimbo = 0  # 叉腰检测归零(clear hands-on-hips detection)
                self.count_no_akimbo += 1  # 没有叉腰检测+1(no hands-on-hips detection+1)
                # 当连续5次都检测到叉腰且当前不在标定状态(If hands-on-hips posture is detected for 5 consecutive times, and not under calibrated status)
```

Function for recognizing limbs, which invokes the model to draw key points of the human body based on the recognized information. Then, it performs color recognition based on the position of each limb's different contours, finally determining and moving according to the recognized posture.

Main:

```py
    def main(self):
        while self.running:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            try:
                result_image = self.image_proc(np.copy(image))
            except BaseException as e:
                self.get_logger().info('\033[1;32m%s\033[0m' % e)
                result_image = cv2.flip(cv2.cvtColor(image, cv2.COLOR_RGB2BGR), 1)
            self.fps.update()
            result_image = self.fps.show_fps(result_image)
            cv2.imshow(self.name, result_image)
            key = cv2.waitKey(1)
            if key == ord('q') or key == 27:  # 按q或者esc退出(press q or esc to exit)
                self.mecanum_pub.publish(Twist())
                self.running = False
        rclpy.shutdown()
```

The main function within the BodyControlNode class, used to input image information into the recognition function and display the returned image.

### 21.3.12 Pose Detection

Through the human pose estimation model in the MediaPipe machine learning framework, the human body posture is detected. When the robot detects a person falling, it will sound an alarm and sway from side to side.

* **Program Logic**

First, import the human pose estimation model from MediaPipe and subscribe to topic messages to obtain real-time footage from the camera.

Then, process the image, such as flipping, to detect human body information in the image. Based on the lines connecting the key points of the human body, calculate the limb height to determine the body movement.

Finally, if "**falling**" is detected, the robot will sound an alarm and move forwards and backwards.

* **Operation Steps**

> [!NOTE]
>
> **Note: the input command should be case sensitive, and keywords can be complemented using Tab key.**

1. Start the robot, and enter the robot system desktop using NoMachine.

2. Click-on <img src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image6.png" style="width:50px" /> to open ROS1 the command-line terminal.

3. Run the command to disable app auto-start app service.

   ```py
   sudo systemctl stop start_app_node.service
   ```

4. Click-on <img src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image85.png" style="width:50px" /> to start the ROS2 command-line terminal.

5. Run the following command and hit Enter key to initiate the game:

   ```py
   ros2 launch example body_and_rgb_control.launch.py
   ```

6)  The program will launch the camera's image interface. For specific recognition steps, refer to Section Program Outcome.

7)  If you need to close this gameplay, you need to press the "Esc" key in the image interface to exit the camera image interface.

8)  Then press "Ctrl+C" in the command line terminal interface. If closing fails, please try again.

* **Program Outcome**

Once the game starts, ensure the human body remains as fully within the camera's field of view as possible. Upon recognizing the human body, the key points will be highlighted in the returned image.

At this point, the individual can sit down briefly. Upon detecting the "**falling**" posture, the robot will continuously sound an alarm and make repeated forward and backward movements as a reminder.

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image120.png" style="width:500px" />

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image121.png" style="width:500px" />

* **Program Analysis**

The program file is saved in

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image122.png" style="width:500px"  />

**ros2_ws/src/example/example/body_control/include/fall_down_detect.py**

> [!NOTE]
>
> **Note: Prior to making any alterations to the program, ensure to create a backup of the original factory program. Modify it only after creating the backup. Directly editing the source code file is prohibited to prevent inadvertent parameter modifications that could render the robot dysfunctional and irreparable!**

Based on the game's effectiveness, the procedural logic is delineated as follows:

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image123.png" style="width:500px"  />

The car captures images via the camera, identifies the key feature points of the human body, and assesses whether the current posture indicates a "**fall**". If a fall is detected, the car's buzzer will emit a continuous "**beep**" sound while the car moves backward. Otherwise, the buzzer will only emit a single "**beep**" sound.

The program logic flow chart obtained from the program files is depicted in the figure below.

<img class="common_img" src="../_static/media/4/section_17_MediaPipe Man-Robot Interaction/media/image124.png" style="width:500px"  />

**1. Function**

Main：

```py
def main():
    node = FallDownDetectNode('fall_down_detect')
    rclpy.spin(node)
    node.destroy_node()
```

Used to start the body sensation control node.

`get_joint_landmarks`:

```py
def get_joint_landmarks(img, landmarks):
    """
    将landmarks从medipipe的归一化输出转为像素坐标(Convert landmarks from medipipe's normalized output to pixel coordinates)
    :param img: 像素坐标对应的图片(picture corresponding to pixel coordinate)
    :param landmarks: 归一化的关键点(normalized keypoint)
    :return:
    """
    h, w, _ = img.shape
    landmarks = [(lm.x * w, lm.y * h) for lm in landmarks]
    return np.array(landmarks)
```

Used to convert the recognized information into pixel coordinates.

`height_cal`:

```py
def height_cal(landmarks):
    y = []
    for i in landmarks:
        y.append(i[1])
    height = sum(y)/len(y)

    return height
```

Calculates the height of the limbs based on the recognized information.

**2. Class**

```py
class FallDownDetectNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        self.drawing = mp.solutions.drawing_utils
        self.body_detector = mp_pose.Pose(
            static_image_mode=False,
            min_tracking_confidence=0.5,
            min_detection_confidence=0.5)
        self.running = True
        self.fps = fps.FPS()  # fps计算器(fps calculator)
        
        self.fall_down_count = []
        self.move_finish = True
        self.stop_flag = False
        signal.signal(signal.SIGINT, self.shutdown)
        self.image_queue = queue.Queue(maxsize=2)
```

This class is the fall detection node.

Init:

```py
class FallDownDetectNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        self.drawing = mp.solutions.drawing_utils
        self.body_detector = mp_pose.Pose(
            static_image_mode=False,
            min_tracking_confidence=0.5,
            min_detection_confidence=0.5)
        self.running = True
        self.fps = fps.FPS()  # fps计算器(fps calculator)
```

Initialize the parameters required for body control, read the camera's image callback node, initialize nodes such as chassis, buzzers, and others, and finally start the main function within the class.

`get_node_state`:

```py
    def get_node_state(self, request, response):
        response.success = True
        return response
```

Set the initialization state of the current node.

shutdown:

```py
    def shutdown(self, signum, frame):
        self.running = False
```

Program exit callback function used to terminate recognition.

`image_callback`:

```py
    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)  # 将自定义图像消息转化为图像(convert the custom image message into image)age))
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(discard the oldest image if the queue is full)
            self.image_queue.get()
        # 将图像放入队列(put the image into the queue)
        self.image_queue.put(rgb_image)
```

Image node callback function used to process images and enqueue them.

Move:

```py
    def move(self):
        for i in range(5):
            twist = Twist()
            twist.linear.x = 0.2
            self.mecanum_pub.publish(twist)
            time.sleep(0.2)
            twist = Twist()
            twist.linear.x = -0.2
            self.mecanum_pub.publish(twist)
            time.sleep(0.2)
        self.mecanum_pub.publish(Twist())
        self.stop_flag =True
        self.move_finish = True
```

Movement strategy function that moves the vehicle according to the recognized limb height.

`buzzer_warn`:

```py
    def buzzer_warn(self):
        if not self.stop_flag:
            while not self.stop_flag:
                msg = BuzzerState()
                msg.freq = 1000
                msg.on_time = 0.1
                msg.off_time = 0.1
                msg.repeat = 1
                self.buzzer_pub.publish(msg)
                time.sleep(0.2)
        else:
            msg = BuzzerState()
            msg.freq = 1900
            msg.on_time = 0.2
            msg.off_time = 0.01
            msg.repeat = 1
            self.buzzer_pub.publish(msg)
```

Buzzer control function used for buzzer alarms.

`image_proc`:

```py
    def image_proc(self, image):
        image_flip = cv2.flip(cv2.cvtColor(image, cv2.COLOR_RGB2BGR), 1)
        results = self.body_detector.process(image)
        if results is not None and results.pose_landmarks:
            if self.move_finish:
                landmarks = get_joint_landmarks(image, results.pose_landmarks.landmark)
                h = height_cal(landmarks)
                if h > 240:
                    self.fall_down_count.append(1)
                else:
                    self.fall_down_count.append(0)
                if len(self.fall_down_count) == 3:
                    count = sum(self.fall_down_count)
```

Function for recognizing limbs, which invokes the model to draw key points of the human body based on the recognized information, and moves according to the recognized height.

Main:

```py
    def main(self):
        while self.running:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            try:
                result_image = self.image_proc(np.copy(image))
            except BaseException as e:
                self.get_logger().info('\033[1;32m%s\033[0m' % e)
                result_image = cv2.flip(cv2.cvtColor(image, cv2.COLOR_RGB2BGR), 1)
            self.fps.update()
            result_image = self.fps.show_fps(result_image)
            cv2.imshow(self.name, result_image)
            key = cv2.waitKey(1)
            if key == ord('q') or key == 27:  # 按q或者esc退出(press q or esc to exit)
                self.mecanum_pub.publish(Twist())
                self.running = False
```

The main function within the FallDownDetectNode class, used to input image information into the recognition function and display the returned image.
