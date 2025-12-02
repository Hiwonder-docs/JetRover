# 1. Quick Start Guide

## 1.1 JetRover Version Description

JetRover Pro is a comprehensive ROS robot for ROS educational scenarios. Its chassis can be seamlessly switched between Mecanum-wheels chassis, tank chassis and Ackermann chassis.

The main controller supports Jetson Nano, Jetson Orin Nano, Jetson Orin NX and Raspberry Pi. Its chassis adopts an STM32 controller, and its body is armed with high-performance hardward such as Lidar, depth camera, microphone array, etc. It can enable mapping and navigation, path planing, human-machine interaction, autonomous driving, deep learning and voice interaction.

The packing list below uses Macanum-wheel chassis as an example. Other accessories are applicable to all robot version.

* **JetRover Standard Packing List**

  (The Lidar model is SLAMTEC A1)

| No.  |            Product Name            |                           Picture                            |
| :--: | :--------------------------------: | :----------------------------------------------------------: |
|  1   | JetRover (Lida included,assembled) | <img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/产品.png" style="width:500px" /> |
|  2   |             Robot arm              | <img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/标准版机械臂.png" style="width:300px" /> |
|  3   |  12.6V 2A charger(DC5.5*2.5 male)  | <img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/12.6V-2A充电器.png" style="width:250px" /> |
|  4   |            Card reader             | <img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/读卡器.png" style="width:70px" /> |
|  5   |        Wireless controller         | <img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/清单_手柄XY.png" style="width:250px" /> |
|  6   |          3D depth camera           | <img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/相机.png" style="width:250px" /> |
|  7   |        Type-C cable(800mm)         | <img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/连接线.png" style="width:280px" /> |
|  8   |           Camera bracket           | <img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/相机固定板.png" style="width:100px" /> |
|  9   |      Colored blocks(30*30mm)       | <img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/三色方块_30或40mm.png" style="width:200px" /> |
|  10  |           Tags(30*30mm)            | <img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/标签卡片.png" style="width:200px" /> |
|  11  |           Accessory bag            | <img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/大小六角扳手.png" style="width:70px"/><img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/标准版螺丝包.png" style="width:200px" /><img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/清单_螺丝刀.png" style="width:28px" /> |
|  12  |            User manual             | <img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/手册.png" style="width:300px" /> |

> [!NOTE]
>
> Notice: The packing list takes the mecanum wheel chassis as an example. Users can choose Ackerman or tank chassis. All other accessories are the same except for the chassis.

* **JetRover Advanced Packing List**

  (The Lidar model is SLAMTEC A1)

<table  class="docutils-nobg" style="margin:0 auto" border="1">
    <thead>
            <tr>
      <th style="padding: 8px; text-align: center;">No.</th>
      <th style="padding: 8px; text-align: center;">Product Name</th>
      <th style="padding: 8px; text-align: center;">Picture</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td style="padding: 8px; text-align: center;">1</td>
      <td style="padding: 8px; text-align: center;">JetRover (Lida included,assembled)</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/产品.png" style="width:500px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">2</td>
      <td style="padding: 8px; text-align: center;">Robot arm</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/标准版机械臂.png" style="width:300px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">3</td>
      <td style="padding: 8px; text-align: center;">12.6V 2A charger(DC5.5*2.5 male)</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/12.6V-2A充电器.png" style="width:250px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">4</td>
      <td style="padding: 8px; text-align: center;">Card reader</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/读卡器.png" style="width:70px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">5</td>
      <td style="padding: 8px; text-align: center;">Wireless controller</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/清单_手柄XY.png" style="width:250px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">6</td>
      <td style="padding: 8px; text-align: center;">3D depth camera</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/相机.png" style="width:250px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">7</td>
      <td style="padding: 8px; text-align: center;">Type-C cable(800mm)</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/连接线.png" style="width:280px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">8</td>
      <td style="padding: 8px; text-align: center;">Camera bracket</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/相机固定板.png" style="width:100px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">9</td>
      <td style="padding: 8px; text-align: center;">Colored blocks(30*30mm)</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/三色方块_30或40mm.png" style="width:200px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">10</td>
      <td style="padding: 8px; text-align: center;">Tags(30*30mm)</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/标签卡片.png" style="width:200px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">11</td>
      <td style="padding: 8px; text-align: center;">Accessory bag</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/大小六角扳手.png" style="width:70px"/><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/标准版螺丝包.png" style="width:200px" /><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/清单_螺丝刀.png" style="width:28px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">12</td>
      <td style="padding: 8px; text-align: center;">User manual</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/手册.png" style="width:300px" /></td>
    </tr>
    <tr>
        <td rowspan="3" style=" padding: 8px; text-align: center; vertical-align: middle;">13</td>
        <td rowspan="3" style=" padding: 8px; text-align: center; vertical-align: middle;">7-inch LCD screen Expansion Pack</td>
        <td style=" padding: 8px; text-align: center;">7-inch LCD screen<br /><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/7寸显示器.png" style="width:300px" /></td>
      </tr>
      <tr>
          <td style=" padding: 8px; text-align: center;">HDMI cable(300mm)+micro-USB cable(450mm)<br /><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/type-C长.png" style="width:200px;height:20px" /><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/type-C长.png" style="width:300px;height:15px" />			</td>
      </tr>
      <tr>
          <td style=" padding: 8px; text-align: center;">Screen bracket<br /><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/显示屏支架.png" style="width:150px" /><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/螺丝包_豪华版2.png" style="width:100px" />
          </td>
      </tr>
    </tbody>
</table>

> [!NOTE]
>
> Notice: The packing list takes the mecanum wheel chassis as an example. Users can choose Ackerman or tank chassis. All other accessories are the same except for the chassis.

* **JetRover Developer Packing List**

  (The Lidar model is SLAMTEC A1. Please note that the microphone array must be manually installed onto the car by the user)

<table  class="docutils-nobg" style="margin:0 auto" border="1">
  <thead>
    <tr>
      <th style="padding: 8px; text-align: center;">No.</th>
      <th style="padding: 8px; text-align: center;">Product Name</th>
      <th style="padding: 8px; text-align: center;">Picture</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td style="padding: 8px; text-align: center;">1</td>
      <td style="padding: 8px; text-align: center;">JetRover (Lida included,assembled)</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/产品.png" style="width:500px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">2</td>
      <td style="padding: 8px; text-align: center;">Robot arm</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/标准版机械臂.png" style="width:300px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">3</td>
      <td style="padding: 8px; text-align: center;">12.6V 2A charger(DC5.5*2.5 male)</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/12.6V-2A充电器.png" style="width:250px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">4</td>
      <td style="padding: 8px; text-align: center;">Card reader</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/读卡器.png" style="width:70px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">5</td>
      <td style="padding: 8px; text-align: center;">Wireless controller</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/清单_手柄XY.png" style="width:250px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">6</td>
      <td style="padding: 8px; text-align: center;">3D depth camera</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/相机.png" style="width:250px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">7</td>
      <td style="padding: 8px; text-align: center;">Type-C cable(800mm)</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/连接线.png" style="width:280px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">8</td>
      <td style="padding: 8px; text-align: center;">Camera bracket</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/相机固定板.png" style="width:100px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">9</td>
      <td style="padding: 8px; text-align: center;">Colored blocks(30*30mm)</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/三色方块_30或40mm.png" style="width:200px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">10</td>
      <td style="padding: 8px; text-align: center;">Tags(30*30mm)</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/标签卡片.png" style="width:200px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">11</td>
      <td style="padding: 8px; text-align: center;">Accessory bag</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/大小六角扳手.png" style="width:70px"/><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/标准版螺丝包.png" style="width:200px" /><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/清单_螺丝刀.png" style="width:28px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">12</td>
      <td style="padding: 8px; text-align: center;">User manual</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/手册.png" style="width:300px" /></td>
    </tr>
    <!-- 合并13号产品的空白行 -->
    <tr>
      <td rowspan="3" style="padding: 8px; text-align: center; vertical-align: middle;">13</td>
      <td rowspan="3" style="padding: 8px; text-align: center; vertical-align: middle;">7-inch LCD screen Expansion Pack</td>
      <td style="padding: 8px; text-align: center;">7-inch LCD screen<br/><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/7寸显示器.png" style="width:300px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">HDMI cable(300mm)+micro-USB cable(450mm)<br /><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/type-C长.png" style="width:200px;height:20px" /><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/type-C长.png" style="width:300px;height:15px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">Screen bracket<br /><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/显示屏支架.png" style="width:150px" /><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/螺丝包_豪华版2.png" style="width:100px" /></td>
    </tr>
    <!-- 合并14号产品的空白行 -->
    <tr>
      <td rowspan="2" style="padding: 8px; text-align: center; vertical-align: middle;">14</td>
      <td rowspan="2" style="padding: 8px; text-align: center; vertical-align: middle;">6-Microphone array Expansion Pack</td>
      <td style="padding: 8px; text-align: center;">6-Microphone array<br /><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image317.png" style="width:200px" /><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/螺丝包_旗舰版2.png" style="width:100px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">Type-C cable(350mm)<br /><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/type-C.png" style="width:130px" /></td>
    </tr>
  </tbody>
</table>

> [!NOTE]
>
> Notice: The packing list takes the mecanum wheel chassis as an example. Users can choose Ackerman or tank chassis. All other accessories are the same except for the chassis.

* **JetRover Ultimate Packing List**

  (Note: The Lidar has been upgraded to the EAl G4 model. The microphone array must be manually installed onto the car by the user.

<table  class="docutils-nobg" style="margin:0 auto" border="1">
  <thead>
    <tr>
      <th style="padding: 8px; text-align: center;">No.</th>
      <th style="padding: 8px; text-align: center;">Product Name</th>
      <th style="padding: 8px; text-align: center;">Picture</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td style="padding: 8px; text-align: center;">1</td>
      <td style="padding: 8px; text-align: center;">JetRover (Lida included,assembled)</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/产品.png" style="width:500px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">2</td>
      <td style="padding: 8px; text-align: center;">Robot arm</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/标准版机械臂.png" style="width:300px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">3</td>
      <td style="padding: 8px; text-align: center;">12.6V 2A charger(DC5.5*2.5 male)</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/12.6V-2A充电器.png" style="width:250px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">4</td>
      <td style="padding: 8px; text-align: center;">Card reader</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/读卡器.png" style="width:70px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">5</td>
      <td style="padding: 8px; text-align: center;">Wireless controller</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/清单_手柄XY.png" style="width:250px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">6</td>
      <td style="padding: 8px; text-align: center;">3D depth camera</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/相机.png" style="width:250px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">7</td>
      <td style="padding: 8px; text-align: center;">Type-C cable(800mm)</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/连接线.png" style="width:280px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">8</td>
      <td style="padding: 8px; text-align: center;">Camera bracket</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/相机固定板.png" style="width:100px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">9</td>
      <td style="padding: 8px; text-align: center;">Colored blocks(30*30mm)</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/三色方块_30或40mm.png" style="width:200px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">10</td>
      <td style="padding: 8px; text-align: center;">Tags(30*30mm)</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/标签卡片.png" style="width:200px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">11</td>
      <td style="padding: 8px; text-align: center;">Accessory bag</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/大小六角扳手.png" style="width:70px"/><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/标准版螺丝包.png" style="width:200px" /><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/清单_螺丝刀.png" style="width:28px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">12</td>
      <td style="padding: 8px; text-align: center;">User manual</td>
      <td style="padding: 8px; text-align: center;"><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/手册.png" style="width:300px" /></td>
    </tr>
    <!-- 合并13号产品的空白行 -->
    <tr>
      <td rowspan="3" style="padding: 8px; text-align: center; vertical-align: middle;">13</td>
      <td rowspan="3" style="padding: 8px; text-align: center; vertical-align: middle;">7-inch LCD screen Expansion Pack</td>
      <td style="padding: 8px; text-align: center;">7-inch LCD screen<br/><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/7寸显示器.png" style="width:300px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">HDMI cable(300mm)+micro-USB cable(450mm)<br /><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/type-C长.png" style="width:200px;height:20px" /><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/type-C长.png" style="width:300px;height:15px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">Screen bracket<br /><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/显示屏支架.png" style="width:150px" /><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/螺丝包_豪华版2.png" style="width:100px" /></td>
    </tr>
    <!-- 合并14号产品的空白行 -->
    <tr>
      <td rowspan="2" style="padding: 8px; text-align: center; vertical-align: middle;">14</td>
      <td rowspan="2" style="padding: 8px; text-align: center; vertical-align: middle;">6-Microphone array Expansion Pack</td>
      <td style="padding: 8px; text-align: center;">6-Microphone array<br /><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image317.png" style="width:200px" /><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/螺丝包_旗舰版2.png" style="width:100px" /></td>
    </tr>
    <tr>
      <td style="padding: 8px; text-align: center;">Type-C cable(350mm)<br /><img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/type-C.png" style="width:130px" /></td>
    </tr>
  </tbody>
</table>

> [!NOTE]
>
> Notice: The packing list takes the mecanum wheel chassis as an example. Users can choose Ackerman or tank chassis. All other accessories are the same except for the chassis.

## 1.2 Hardware Installation and Power-on Preparation

### 1.2.1 Install Depth Camera and Robot Arm

1)  Fix 3D depth camera bracket to robot arm using M3\*6 round-head machine screw.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image8.png" style="width:5.76667in;height:3.0125in" />

2)  Mount 3D depth camera to camera bracket with M3\*6 round-head machine screw.

> [!NOTE]
>
> **Note: the port of 3D depth camera is on robot’s right side.**

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image9.png" style="width:500px" />

3)  Attach robot arm to the pan-tilt with M3\*6 round-head machine screw.

> [!NOTE]
>
> **Note: the gripper should face towards the front of the car.**

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image10.png" style="width:500px" />

4)  Connect the wire of pan-tilt servo to the servo on U-shaped bracket.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image11.png" style="width:500px" />

5)  Connect camera cable to USB port of 3D depth camera. Then pass the cable through bracket as pictured. Finally, connect it to port 2 on Jetson Nano board.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image12.png" style="width:500px" />

> [!NOTE]
>
> Note: For specific wiring instruction, refer to “**[1.2.4 Wiring Instruction.]()**”

### 1.2.2 Install Main Control Board (Applicable to the robot version without Controller)

* **Jetson Nano Control Board**

1)  Mount the control board onto the robot with M2.5\*12 double-pass copper columns.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image13.png" style="width:500px"  />

2)  Fix the antennas with M2.5\*6 round-head machine screws.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image14.png" style="width:500px"  />

3)  Fix the expansion board with M2.5\*6 round-head machine screws.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image15.png" style="width:500px"  />

* **Jetson Orin Nano/Jetson Orin NX Control Board**

1)  Mount the control board onto the robot with M2.5\*12 double-pass copper columns.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image16.png" style="width:500px"  />

2)  Fix the antennas with M2.5\*6 round-head machine screws.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image17.png" style="width:500px"  />

3)  Fix the expansion board with M2.5\*6 round-head machine screws.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image18.png" style="width:500px"  />

* **Raspberry Pi5 Control Board**

1)  Secure the four M2.5\*5+6 singe-pass nylon columns to the adapter plate using M2.5\*6 round head screws.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image19.png" style="width:500px"  />

2)  Press down firmly on the push-type screw of the heat sink to mount it on the main board.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image20.png" style="width:500px"  />

3)  Attach the main board to the adapter plate using double-pass nylon columns.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image21.jpeg" style="width:500px"  />

4)  Insert the expansion board into the pins of Raspberry Pi 5.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image22.png" style="width:500px"  />

5)  Attach the adapter plate onto the robot with M2.5\*6 round-head screws.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image23.png" style="width:500px"  />

### 1.2.3 Install 7-inch LCD Screen

1)  Install 7-inch LCD screen to columns with M4\*6 round-head machine screws.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image24.jpeg" style="width:500px"  />

2)  Connect HDMI cable to HDMI interface on the 7-inch LCD screen, and connect power cable to CTOUCH port.

> [!NOTE]
>
> **Note: 7-inch LCD screen doesn't support touch control if the cable is not connected to CTOUCH port.**

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image25.jpeg" style="width:500px"  />

### 1.2.4 Wiring Instructions (Key Points)

> [!NOTE]
>
> **Note：Upon receiving the robot, it is necessary to follow the wiring tutorials below to connect the control board. Each accessory must be properly connected its corresponding port; otherwise, the robot's functions will not work properly.**

* **Wiring Instructions（Jetson Nano Controller）**

Please refer to the table below and connect each accessory to its corresponding port on Jetson Nano board. If any of the listed accessories are not included in your kit, please ignore the corresponding port number.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image26.png" style="width:500px" />

| **NO.** | **Name** |
|:--:|:--:|
| 1 | Jetson Nano board supply interface (DC round head cable) |
| 2 | Display HDMI interface |
| 3 | Depth camera port |
| 4 | USB hub Communication interface |
| 5 | STM32 controller |
| 6 | Display power supply port |
| 7 | Lidar port |
| 8 | Reserved custom expansion interface |
| 9 | Reserved custom expansion port |
| 10 | Microphone array module (For developer kit and ultimate kit only ) |

* **Wiring Instructions（Jetson Orin Nano Controller/Jetson Orin NX Controller）**

Please refer to the table below and connect each accessory to its corresponding port on Jetson Orin Nano board and USB hub. If any of the listed accessories are not included in your kit, please ignore the corresponding port number.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image27.png" style="width:500px" />

<table  class="docutils-nobg" style="margin:0 auto" border="1">
<colgroup>
<col style="width: 13%" />
<col style="width: 86%" />
</colgroup>
<thead>
<tr>
<th style="text-align: center;"><strong>NO.</strong></th>
<th style="text-align: center;"><strong>Name</strong></th>
</tr>
</thead>
<tbody>
<tr>
<td style="text-align: center;">1</td>
<td style="text-align: center;"><p>Power supply port of Jetson Orin Nano</p>
<p><strong>（DC round head cable)</strong></p></td>
</tr>
<tr>
<td style="text-align: center;">2</td>
<td style="text-align: center;">DP interface of 7-inch display</td>
</tr>
<tr>
<td style="text-align: center;">3</td>
<td style="text-align: center;">Depth camera interface</td>
</tr>
<tr>
<td style="text-align: center;">4</td>
<td style="text-align: center;">USB hub communication interface</td>
</tr>
<tr>
<td style="text-align: center;">5</td>
<td style="text-align: center;">STM32 controller communication cable</td>
</tr>
<tr>
<td style="text-align: center;">6</td>
<td style="text-align: center;">Display Power Supply Interface</td>
</tr>
<tr>
<td style="text-align: center;">7</td>
<td style="text-align: center;">Lidar port</td>
</tr>
<tr>
<td style="text-align: center;">8</td>
<td style="text-align: center;">Reserved custom expansion interface</td>
</tr>
<tr>
<td style="text-align: center;">9</td>
<td style="text-align: center;">Reserved custom expansion port</td>
</tr>
<tr>
<td style="text-align: center;">10</td>
<td style="text-align: center;">Microphone array module (For developer kit and ultimate kit only )</td>
</tr>
</tbody>
</table>

* **Wiring Instructions（Jetson Orin NX Controller）**

Please refer to the table below and connect each accessory to its corresponding port on Jetson Orin NX board and USB hub. If any of the listed accessories are not included in your kit, please ignore the corresponding port number.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image27.png" style="width:500px" />

<table  class="docutils-nobg" style="margin:0 auto" border="1">
<colgroup>
<col style="width: 13%" />
<col style="width: 86%" />
</colgroup>
<thead>
<tr>
<th style="text-align: center;"><strong>NO.</strong></th>
<th style="text-align: center;"><strong>Name</strong></th>
</tr>
</thead>
<tbody>
<tr>
<td style="text-align: center;">1</td>
<td style="text-align: center;"><p>Power supply port of Jetson Orin NX</p>
<p><strong>（DC round head cable)</strong></p></td>
</tr>
<tr>
<td style="text-align: center;">2</td>
<td style="text-align: center;">DP interface of 7-inch display</td>
</tr>
<tr>
<td style="text-align: center;">3</td>
<td style="text-align: center;">USB hub communication interface</td>
</tr>
<tr>
<td style="text-align: center;">4</td>
<td style="text-align: center;">7-inch LCD display power cable (not included in the standard kit)</td>
</tr>
<tr>
<td style="text-align: center;">5</td>
<td style="text-align: center;">Depth camera interface</td>
</tr>
<tr>
<td style="text-align: center;">6</td>
<td style="text-align: center;">STM32 controller communication cable</td>
</tr>
<tr>
<td style="text-align: center;">7</td>
<td style="text-align: center;">Lidar port</td>
</tr>
<tr>
<td style="text-align: center;">8</td>
<td style="text-align: center;">Reserved custom interface</td>
</tr>
<tr>
<td style="text-align: center;">9</td>
<td style="text-align: center;">Reserved custom expansion port</td>
</tr>
<tr>
<td style="text-align: center;">10</td>
<td style="text-align: center;">Microphone array module（not included in the standard and advanced kits）</td>
</tr>
</tbody>
</table>

* **Wiring Instructions（Raspberry Pi 5 Control Board）**

Please refer to the table below and connect each accessory to its corresponding port on Raspberry Pi 5 board and USB hub. If any of the listed accessories are not included in your kit, please ignore the corresponding port number.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image28.png" style="width:500px" />

| **NO.** | **Name** |
|:--:|:--:|
| 1 | Depth camera port |
| 2 | USB hub CommunicationiInterface |
| 3 | STM32 controller communication cable |
| 4 | 7-inch LCD display power cable (not included in the standard kit) |
| 5 | Ethernet interface |
| 6 | Lidar port |
| 7 | Reserved custom interface |
| 8 | Reserved custom expansion port |
| 9 | Microphone array module（not included in the standard and advanced kits） |

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image29.png" style="width:500px" />

| **NO.** | **Name** |
|:--:|:--:|
| 12 | Power supply for ROS main controller (Raspberry Pi) |
| 13 | Micro HDMI interface (connect to the HDMI interface of the 7-inch LCD display) |

## 1.3 Charging Instructions and Battery Usage Guide

It is mandatory for the robot to disconnect power cables during transportation, and the battery is not fully charged. Before the first use, users need to connect the battery wires and then proceed with charging. The charging duration is approximately 3 hours. Charging the robot from 10V to 12.3V takes approximately 3 hours.

### 1.3.1 Charging Instructions

1.  Charge battery with the provided charger. Turn off the switch on the power supply board and never use it while charging

2.  Indicator on charger changes green when it connects to battery but not plugged into a power outlet. Indicator turns green after the battery is fully charged.

3.  **Do not directly connect the charger to DC power port of Jetson Nano board, otherwise the board will be burned.**

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image30.png" style="width:500px" />

4.  Unplug charger in time to avoid overcharging.

5.  If battery voltage drops below 10V, the buzzer will sound an alarm. Please charge the battery promptly.

6.  In case robot won’t be used in a long period of time, please fully charge battery first, then disconnect battery wires.

7.  Always keep battery in cool and dry environment, otherwise battery lifespan gets shortened. Never intentionally hit, throw or step on your Lipo battery.

8.  Avoid using the battery in environments with strong static electricity or a strong magnetic field, as this may damage the safety protection device of the battery.

9.  Do not plug the battery into a power outlet, and do not use metal objects to connect the positive and negative poles of the battery.

10. If robot won’t be used in a long period of time, please fully charge battery.

11. It is strictly prohibited to modify, weld, or convert battery charger or Lipo battery.

12. Keep battery away from high temperature and liquid to prevent overheating, fire and damp, which result in function decline.

> [!NOTE]
>
> Warning tip: please strictly follow the guide. Our company is not responsible for any product damage, economic losses and accident caused by improper use.

### 1.3.2 Charging Operation

1)  Make sure the switch is in the closed position.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image31.png" style="width:500px"  />

2)  Connect the battery wires ensuring that each wire matches its corresponding color.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image32.png" style="width:500px"  />

Macanum/Tank Chassis Wiring Connection Diagram

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image33.png" style="width:500px"  />

Ackermann Chassis Wiring Connection Diagram

2)  Take out the DC connector of the lithium battery from the side or bottom. Connect the charger to charge the battery.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image34.png" style="width:500px"  />

Mecanum/ Tank chassis Charging Port diagram

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image35.png" style="width:500px"  />

Ackerman chassis charging port diagram

3)  Users can view the charging status by observing the indicator on the charger. The red light indicates that the battery is charging, while the green indicates that the charging is complete.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image36.png" style="width:500px" />

> [!NOTE]
>
> **Note: After the charging is complete, please promptly unplug the charger to avoid overcharging.**

## 1.4 First Power-on

After the robot installation is finished, this section begins the learning of robot’s start-up state and functionality testing of each module. Following by this, you can proceed to the next chapter to learn about with the operations for app control and handle control.

For exploring its functionality and view the program, you need to connect to the robot’s system through the remote connection tool, please refer to the subsequent content under “**[1.6 Development Environment Setup and Configuration]()**”.

### 1.4.1 Usage Guideline

1)  To ensure the stable performance of the robot, when the battery voltage is below 10V, charge the robot before proceeding with operations.

2)  Avoid placing the robot at the edge of a high platform to prevent it falling and get damaged.

3)  Using the robot on a flat platform.

4)  Do not stack the servos on robotic arm to avoid the servo damage due to the motor being stuck when powered on.

5)  Avoid keeping the robotic arm under a high load for an extended period because it can curtail the servo's lifespan or directly damage the servo.

6)  Maintain a certain safety distance from the robot to prevent injury in case of contact after powering on.

7)  The test should be conducted under the conditions that the wiring is correct, the controller receiver is properly connected, the sound is turned on at the top-right corner of the desktop, and the robot is fully charged.

8)  In terms of Lidar, different Lidar models also have different performances when starting up. The left image below is A1 Lidar and the right is the G4 Lidar:

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image37.png" style="width:500px" />

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image38.png" style="width:500px" />

If you choose the ultimate kit with Lidar G4, the Lidar will spin for a few round, then stop until the robot boots up successfully.

**For other robot kits with Lidar A1, you need to push the switch to the “ON” side. The Lidar will continue to rotating after a short period of time after being turned on.**

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image39.png" style="width:500px" />

### 1.4.2 Robot Startup State & Testing Instruction

1)  Press the switch at right side of the robot.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image40.png" style="width:500px"  />

2)  The blue LED1 on the expansion board will light up and keep blinking. At this moment, only the network configuration service is enabled, but ROS and other services have not yet completed this process. Wait for **a short “Beep”** from the buzzer to indicate that the device has started.

- **The LED indicator for Jetson Nano, Jetson Orin Nano and Jetson Orin NX versions is as pictured:**

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image41.png" style="width:500px" />

- **The LED indicator for Raspberry Pi5 version is as pictured:**

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image42.png" style="width:500px" />

3)  After robot boots up successfully, robot is default to AP direct connection mode and generates a WiFi starting with “HW”. The initial WiFi password is “**hiwonder**”.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image43.png" style="width:500px" />

> [!NOTE]
>
> **Attention:**
>
> If you can not find the generated WIFI, please troubleshoot from the following:
>
> * Follow above three steps in “**Robot Startup State & Testing Instruction**” for troubleshooting.
>
> * If the LED1 on expansion board is flashing blue and keeps on, it may be set to LAN mode. In this case, please press and hold the Key1 button for 5-10 seconds. If LED1 starts flashing, indicating that a WI-Fi starting with “**HW**” is generated.
>
> * If the KEY1 button doesn’t start flashing after the above operation, SD card or SSD may not be detected, try re-inserting the SD card or SSD.
>
> * If the LED1 on expansion board always remains on after re-inserting the SD card or SSD, it may indicate an issue with the SD card or SSD.
>
> * If you have completed all five steps above and the issue persists, it may lie with Raspberry Pi board or Jetson board. Please reach out to our support team via email at <support@hiwonder.com> for further assistance.

4)  After the 7-inch display is installed, it will display the system image after the robot starts up. If you’re the user of advanced kit or ultimate kit, you will also hear a “**I’m ready**” broadcast.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image44.png" style="width:500px" />

Please refer to the below table to test the hardware modules:

| Module                                      | Operation                                                    | Outcome                                                      |
| ------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| LED on expansion board                      | Observe the LED status                                       | The robot is set to AP direct connection mode by default. The flashing blue LED indicated that the network service configuration has been completed. |
| Buzzer                                      | Examine the short beep                                       | A short beep from the buzzer indicates that the onboard hardware of the expansion board are functional. |
| Lidar                                       | Observe the rotation status                                  | Lidar model:<br />1. Lidar G4: Upon initial startup, it will rotate for a few round, then stop. After the robot startup is complete, it resumes rotating.<br />2. Lidar A1: it will keep rotating after a brief wait following startup. |
| 7-inch HD touch screen                      | Tap the icons on system desktop to confirm the following:<br />1. Whether the HDMI cable is connected normally.<br />2. Whether the power cable is connected to the C-TOUCH interface of the screen. | The screen can display the system desktop and can be touched normally. |
| KEY1 button on expansion board              | Switch the network status                                    | After connecting to STA LAN mode via app, long press KEY1 and then check if LED1 indicator flashes. |
| Microphone, sound card, speaker             | After turning on the robot, say “**Hello, Hiwonder**”.       | The robot responds “**I’ m here**”. (the voice function is only targeted for the user who purchased advanced kit and ultimate kit.) |
| Depth camera + robotic arm                  | 1. Connect to the robot via app.<br />2. Open “**Robot Control**” to view the live camera feed from the depth camera.<br />3. Swipe the screen to simultaneously control the servo on the pan-tilt.<br />4. Slide the robot arm control buttons for controlling each joint’s servo. | Display the live camera feed and rotate.                     |
| STM32 controller + DC  encoder geared motor | After robot powers on, perform the “Robot control” function on app using the wireless handle or mobile phone to check the hardware functions. | Robot is able to operate normally.                           |


## 1.5 APP Control

Users can control the robot via the app “**WonderAi**”, and experience the AI vision features. This section will elaborate the operation instructions for each function in the app.

Reminder: if you have purchased the robot kit without Jetson or Raspberry Pi board, please first go through the content from “**[1.6 Development Environment Setup and Configuration]()**” to “**[1.6.7 Robot Version Configuration Tool Instructions]()**”.

### 1.5.1 APP Installation

> [!NOTE]
>
> Note:
>
> * Please ensure all permissions are granted for the APP, otherwise certain app functions may be affected or unavailable.
>
> * Turn on location and Wi-Fi before using the APP.

1)  **Android user:** the installation package can be found in **“[2. Software/ APP Installation Package]()”** and please transfer it to your phone for downloading.

2)  **iOS user:** you can download “**WonderAi**” in App Store.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image45.png" style="width:500px" />

### 1.5.2 Connection Mode Introduction

There are two network modes, including AP direct connection mode and STA LAN mode.

1.  **AP direct connection mode**: Jetson Nano generates a WiFi which can be connected by phones. This WiFi has no internet access.

2.  **STA LAN mode**: Jetson Nano actively connects to specific WiFi. You can access Internet in this mode.

**The default connection mode is AP direct connection mode. Whether you choose AP direct connection mode or STA LAN mode, robot performs the same.**

**It is recommended to experience robot games under Direct Connection Mode.**

* **Direct Connection(Must Read)**

**iOS system requires version 11.0 or above, and Android system requires 5.0 or above.**

Special attention for Android users:

Please ensure all permissions are granted for the APP, otherwise certain app functions may be affected or unavailable.

This section will take the app of Android system to control JetRover macanum version as an example. The same operations apply to iOS system.

1)  Open “**WonderAi**” APP, then tap “**[Advanced->JetRover(Mecanum)]()**” in sequence.

    > [!NOTE]
    >
    > Note: “**JetRover (Track)**” for tank chassis version, and “**JetRover(Acker)**” for Ackerman chassis version.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image46.png" style="width:500px" />

2)  Click “+” button, and select “**Direct Connection Mode**”.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image47.png" style="width:500px" />

3)  Click “**Go to connect device hotspots**” to join the WiFi generated by JetRover.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image48.jpeg" style="width:500px"  />

4)  The device WiFi starts with “**HW**” and the password is “**hiwonder**”.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image49.png" style="width:500px" />

> [!NOTE]
>
> **Note: for iOS user, please do not return back to the APP until the WiFi icon** <img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image50.png" style="width:50px" /> **appears above, otherwise JetAuto cannot be searched. If JetAuto cannot be searched, please click** <img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image51.png" style="width:50px" /> **to refresh.**

5)  Return to the APP, and click the robotic icon to enter home interface.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image52.png" style="width:500px" />

**If you are informed of “No Internet Connection. Whether to keep connection”, just select “Keep Connection”.**

6)  When the following window pops up, it means that you have selected the wrong version. Click “**Confirm**”, then the APP will automatically switch to the home interface of the right version.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image53.png" style="width:500px"  />

7)  After clicking on the robot icon, the mode selection interface is as pictured:

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image54.png" style="width:500px" />

* **LAN Mode (Optional)**

1)  Firstly, join 5G network, for example “**Hiwonder_5G**”. (The router supporting dual-frequency will distinguish the Wi-Fi name by default under the situation that 2.4 G and 5G are separated. For example, Wi-Fi “**Hiwonder**” is 2.4 frequency band while “**Hiwonder_5G**” is 5G frequency band)

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image55.png" style="width:500px" />

2)  After connection, open WonderAi. Then click “**Advanced-\>JetRover (Mecanum)**” in sequence.

    > [!NOTE]
    >
    > Note: “**JetRover (Track)**” for tank chassis version, and “**JetRover(Acker)**” for Ackerman chassis version.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image56.png" style="width:500px"  />

3)  Click “+” button at the lower right corner, then select **LAN Mode**.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image57.png" style="width:500px" />

4)  Continue, enter the Wi-Fi password. Having entered the password, click “OK”. Please ensure the password you enter is correct.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image58.png" style="width:500px" />

5)  Click “**Go to connect device hotspots**”.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image59.png" style="width:500px" />

6)  Join the WiFi starting with HW, and input the password “**hiwonder**”. After connection, return back to the APP interface.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image60.png" style="width:500px" />

7)  At this point, WonderAi APP is connecting to the robot.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image61.png" style="width:500px" />

8)  After a while, the robotic icon will show up. And at the same time, LED1 on the expansion board will light up.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image52.png" style="width:500px" />

9)  By long pressing the robotic icon, you can check the IP address and ID.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image62.png" style="width:500px" />

10) Input this IP address in the search bar on remote desktop software. Then you can enter the robot system desktop. For how to operate, please refer to “**[1.6 Development Environment Setup and Configuration]()**”.

11) If you want switch back to Direct Connect Mode, press KEY1 button on Jetson Nano expansion board until blue LED flashes, which means the current mode is Direct Connect Mode.

### 1.5.3 APP Control Instruction

There are 6 games on APP, including robot control, lidar, object tracking, Intelligent patrolling, gesture control, and AR.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image63.png" style="width:500px" />

The brief instruction for each game is shown in the table below:

| **Icon** | **Game** | **Instruction** |
|:--:|:--:|:---|
| <img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image64.png" style="width:100px" /> | Robot Control | Control the movement of the robot. |
| <img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image65.png" style="width:100px" /> | Lidar | There are three lidar functions, including avoid obstacle, lidar following and lidar guarding. |
| <img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image66.png" style="width:0.59028in;height:0.59028in" /> | Object Tracking | Select the color for the target, allowing the robot to track the target. |
| <img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image67.png" style="width:100px" /> | Line Following | Pave the line, choose the line color as the recognizable color and the robot will move along the line. |
| <img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image68.png" style="width:100px" /> | Gesture Control | Control the robot’s movement through the movement trajectory of the palm such as, forward and backward, left and right turns, left and right translations (mecanum chassis only) |

### 1.5.4 Robot Control 

- **Mecanum/Tank Chassis Version Interface**

Tap “**Remote Control**” in the game selection interface to enter the operation interface. **The interface is as follow:**

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image69.png" style="width:500px" />

1.  The buttons on the left, from top to bottom, are gravity control button, robot’s movement control and speed adjustment slider.

2.  The middle is the live camera feed (you can swipe the screen to rotate the camera’s pant -tilt).

3.  The buttons on the right, from top to bottom, are left and right turn control buttons and pan-tilt rotation control buttons.

4.  The icons in the top menu bar are respectively <img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image70.png" style="width:50px" /> for robotic arm control, <img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image71.png" style="width:0.39375in;height:0.325in" /> screenshot, <img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image72.png" style="width:0.39375in;height:0.35903in" /> close menu bar, switch to full-screen mode (this function is usually used with wireless controller).

- **Ackerman Chassis Version Interface:**

Tap “**Remote Control**” in the game selection interface to enter the operation interface. **The interface is as follow:**

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image73.png" style="width:500px" />

1.  The buttons on the left from top to bottom are forward and backward, speed adjustment.

2.  The middle is the the live camera feed (you can swipe the screen to rotate the camera’s pant -tilt).

3.  The buttons on the right, from top to bottom, are left and right turn control buttons and pan-tilt rotation control buttons.

4.  The icons in the top menu bar are respectively <img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image70.png" style="width:0.39375in;height:0.34167in" /> for robotic arm control, <img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image71.png" style="width:0.39375in;height:0.325in" /> screenshot, <img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image72.png" style="width:0.39375in;height:0.35903in" /> close menu bar, switch to full-screen mode (this function is usually used with PS2 wireless handle).

**The operation interface for the robotic arm is as follow:**

5.  Tap <img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image74.png" style="width:0.40486in;height:0.38403in" />to display the control panel for each servo of the robotic arm. It allows the robot to be controlled

    <img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image75.png" style="width:4.33542in;height:4.87431in" />

### 1.5.5 Lidar

> [!NOTE]
>
> **Note:**
>
> * Before starting the game, please place the robot on a flat surface to ensure an enough space for the operation.
>
> * In “**Avoid obstacle**” and “**Lidar following**”, the robot has a detection range of a 90 degrees sector in front.
>
> * JetRover Ackerman chassis version does not support “**Lidar guarding**” function.
>
> * The obstacle height must be greater than the car body.

- **Avoid Obstacle**

When detecting the obstacle, the robot will turn automatically to avoid the obstacle.

- **Lidar following**

When detecting the obstacle, the robot will adjust its posture to keep a certain distance between it and the obstacle.

- **Lidar guarding**

After the game starts, JetRover will adjust its body to face the obstacle when detecting the obstacle.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image76.png" style="width:500px" />

### 1.5.6 Target Tracking 

> [!NOTE]
>
> **Note:**
>
> * Place the object and the robot on the same surface, and move the object in a translational manner to achieve a better performance.
>
> * Choose a moderate color range, not too large or too small. If the range is too large, it may include colors outside the target; if too small, the target may be easily lost. Additionally, do not allow objects with colors similar to the target to appear in the camera view.

2.  Tap “**Object tracking**” in the game selection interface to access its operation interface. Here, we use a red block for demonstration. Tap “**Pick**” button.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image77.png" style="width:500px" />

2)  Firstly, click “**pick**”, then drag the red circle on the camera returned image to the target to extract the color.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image78.png" style="width:500px" />

3)  After clicking “**OK**”, the “**selected color**” box will display the color.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image79.png" style="width:500px" />

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image80.png" style="width:500px" />

4)  Click “**Start**” button. When moving the target object, JetRover will move following the object.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image81.png" style="width:500px" />

### 1.5.7 Line Following

> [!NOTE]
>
> * Before starting the game, please pave the track using a tape and place the robot on the track.
>
> * The color picking range should not be moderate. If the range is too large, the color outside the target will also be selected. If the range is too small, it will be easy to lost the target. Additionally, please do not have any objects in the camera’s field of view that are similar in color to the target.

1)  Click "**Line Following**" on the mode selection screen to enter the operation interface. A red tape is used as a line for demonstration. Tap “**Pick**” button to pick the line color.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image82.png" style="width:500px" />

2)  Drag the red circle on the camera returned image to the path to pick the color, then click “**Pick color**.”

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image83.png" style="width:500px" />

3)  Next click “**Confirm**” button. The color you pick will display at the right box.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image84.png" style="width:500px" />

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image85.png" style="width:500px" />

4)  Lastly, click “**Start**” button, then the robot will move along the path.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image86.png" style="width:500px" />

### 1.5.8 Gesture Control

> [!NOTE]
>
> **Note:**
>
> * **This game supports the right hand.**
>
> * **During the recognition, it is necessary to appear the entire palm within the camera’s field of view.**
>
> * **Please move the hand at a moderate speed, do not too fast.**

1)  Click “**Gesture Control**” at the mode selection interface to enter the operation interface of this game.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image87.png" style="width:500px" />

2)  Click “**Start**” and follow the instructions for this game. Once the gesture movement trajectory is recognized, the robot will move accordingly.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image88.png" style="width:500px" />

### 1.5.9 AR

Place a random tag within the camera’s field of view and select any 3D image option on the right. Take “**bicycle**” in here. The three-dimensional image of a bicycle will be displayed at the location of the tag.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image89.png" style="width:500px"  />

## 1.6 Development Environment Setup and Configuration

### 1.6.1 Remote Connection Tool Introduction & Installation

Regarding the two methods of remote control for the robot: graphical and command-line control.

NoMachine and VNC are graphical remote control software. After installation, you can directly control the robot on the computer by connecting to the robot's hotspot. For the Jetson Nano, Jetson Orin Nano, and Jetson Orin NX main controllers, NoMachine is used for connection, while the Raspberry Pi 5 main controller uses VNC. With these two software options, users can clearly see the robot’s system desktop, making it easier to operate intuitively.

In contrast to NoMachine and VNC, MobaXterm is an SSH connection that focuses more on command-line control. It does not display the full desktop of the robot system, only a command-line window. For users who are familiar with command-line operations, MobaXterm allows quicker control of the robot via commands while reducing computational load and memory usage.

MobaXterm also has a built-in lightweight X11 server that can directly display graphical application interfaces. Regardless of which control board you use, MobaXterm’s SSH connection method is applicable.

In short, NoMachine and VNC are suitable for scenarios that require intuitive operation, while MobaXterm is better for quickly executing commands. Choose the software that best fits your remote control needs.

**Before operation, users with desktop computers should prepare a wireless network card that supports the 5G frequency band.**

* **NoMachine Installation**

> [!NOTE]
>
> **Note: this tool is applicable to Jetson controller versions.**

1)  Enter the same directory as this document to open the installation package “**nomachine_8.4.2_10_x64**” in the folder “**[2. Software -\> 2. Remote Connection Tool]()**”.

2)  Click “**Next**”.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image90.png" style="width:500px" />

3)  Then click "**I accept the agreement**" in the prompt box, and set the “**Language**” as “**English**”, then click “**Next**”.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image91.png" style="width:500px" />

4)  Remain the default installation path. Click “**Next**”.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image92.png" style="width:500px" />

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image93.png" style="width:500px" />

5)  Click “**Yes**” to restart the computer (**Please do not skip this step**).

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image94.png" style="width:500px" />

* **VNC Installation**

> [!NOTE]
>
> **Note: this tool is applicable to Raspberry Pi 5 version.**

Enter the same directory as this document to open the installation package “**VNC-Viewer-6.17.731-Windows**” in the folder “**[2. Software -> 2. Remote Connection Tool]()**”. In the pop-up dialog box, select '**English**' as the installation language and click the '**OK**' button.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image95.png" style="width:500px" />

1)  Click “**Next**”.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image96.png" style="width:500px" />

2)  Then click "**I accept the agreement**" in the prompt box, and continue.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image97.png" style="width:500px" />

6)  Click “**Install**”.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image98.png" style="width:500px" />

7)  Click “**Finish**” to complete the installation.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image99.png" style="width:500px" />

8)  Once VNC is connected, simply click-on <img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image100.png" style="width:1.61111in;height:0.51389in"  /> to access it.

* **MobaXterm Installation**

> [!NOTE]
>
> **Note: This tool is applicable to any robot version.**

1)  Locate the “**MobaXterm**” installation package in this folder ‘**[2. Software\ 2. Remote Connection Tool]()**’.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image101.png" style="width:500px" />

2)  Click “**Next**”.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image102.png" style="width:500px" />

3)  Then click "**I accept the agreement**" in the prompt box, and click “**Next”**.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image103.png" style="width:500px" />

4)  Keep the default installation position,and click “**Next**”.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image104.png" style="width:500px" />

5)  Click “**Install**”.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image105.png" style="width:500px" />

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image106.png" style="width:500px" />

### 1.6.2 AP Direct Connection Mode Operations

**AP Direct Connection mode: The development board can generate a hotspot that can be connected to by your phone (it cannot access the external network).**

* **Establish Connection via NoMachine**

1)  The robot is set to AP direct connection mode by default. After booting up successfully, it will create a hotspot starting with "**HW**". As the picture shown below, the connection password is “**hiwonder**”.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image107.png" style="width:500px" />

2)  Open NoMachine, input the IP address “**192.168.149.1**” in the search bar in AP mode, and click “**Configure connection to new host 192.168.149.1**”.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image108.png" style="width:500px" />

3)  After opening, follow these steps: click on **Address \> Name, enter "robot" \> Host, enter "192.168.149.1" \> click Add**.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image109.png" style="width:500px" />

4)  Double click on “Robot” to open it.

    <img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image110.png" style="width:500px" />

5)  **Fill in the provided username and password based on the control board version：**

Initial configuration for **Jetson Nano** board:

username: hiwonder, password: hiwonder

Initial configuration for **Jetson Orin Nano** board:

username: ubuntu, password：ubuntu

Initial configuration for **Jetson Orin NX** board:

username: ubuntu, password: ubuntu

6)  Check the "**Save this password**" box, then click the **Login** button. Below will take “**Jetson Nano**” version as an example:

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image111.png" style="width:500px" />

7)  Afterwards, you will see the remote desktop of the Jetson Nano open up.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image112.png" style="width:500px" />

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image113.png" style="width:500px" />

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image114.png" style="width:500px" />

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image115.png" style="width:500px" />

* **Establish Connection via VNC**

1)  Search for and connect to the hotspot starting with “HW”, as shown in the following figure. The password for the connection is hiwonder.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image116.png" style="width:500px" />

2)  Next, open VNC software. In the VNC Viewer, enter the IP address “**192.168.149.1”** in the search bar in AP mode, and press Enter. If you receive a security warning, click '**Continue**'

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image117.png" style="width:500px" />

3)  Wait for the connection window to pop up, then enter the following in order: Username \> Password \> Click 'Remember password' \> Click 'OK':

    **Username: pi**

    **Password: raspberrypi**

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image118.png" style="width:500px" />

4)  Once connected, the Raspberry Pi's remote desktop will appear as shown in the image below.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image119.png" style="width:500px" />

* **Establish Connection via MobaXterm**

This section takes “**AP Direct Connection**” as an example to illustrate. The same operation method is applicable to LAN mode. You just need to change the IP address.

1)  In the main interface, click “**Session**” on the top right corner to create a session. In the session interface, enter the recorded Raspberry Pi 5 IP address “**192.168.149.1**”, and then click “**OK**”.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image120.png" style="width:500px" />

2)  Choose SSH.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image121.png" style="width:500px" />

3)  Enter the fixed IP address in AP direction connection mode:

    **192.168.149.1**

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image122.png" style="width:500px" />

4)  As pictured, you need to select the third option.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image123.png" style="width:500px" />

5)  The interface will prompt users to enter the login as and password. It is necessary to fill in the username and password based on the control board version (Below will take Jetson Nano version as an example)：

Initial configuration for **Jetson Nano** board:

username: hiwonder, password：hiwonder

Initial configuration for **Jetson Orin Nano** board:

username: ubuntu, password：ubuntu

Initial configuration for **Jetson Orin NX** board:

username: ubuntu, password: ubuntu

Initial configuration for **Raspberry Pi 5** board:

username: pi, password: raspberrypi

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image124.png" style="width:500px" />

> [!NOTE]
>
> **Note:**
>
> * The username must be entered in lowercase mode. Even if the username includes uppercase letters during setup, it must be entered in lowercase when logging in.
>
> * The username will be visually displayed. After entering it, press Enter to proceed to the password input.
>
> * The password will not be visually displayed. After entering the password, press Enter to log in.

8)  When the input password is correct, you will get access to the system. The system interface is as pictured:

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image125.png" style="width:500px" />

### 1.6.3 LAN Mode Connection

**STA LAN Mode: the development board is able to actively connect to the designated hotpot/ WIFI. (It can connect external network)**

> [!NOTE]
>
> **Attention:**
>
> * If you are uncertain how the robot connect the robot to the STA local network and obtains an IP address, it is recommended to connect via the app.
>
> * The system image has made special configurations for WI-FI, so the WI-FI option cannot be directly selected from the menu bar, as shown in the image below. This does not affect the normal operation of the robot. Users can refer to the "**Method to Restore Wi-Fi Option in the Menu Bar**" for the necessary settings.
>
> * This section will use connecting to the "**Hiwonder_5G**" Wi-Fi as an example. Users should refer to the Wi-Fi that you have set up themselves when configuring the local network connection.

* **Establish Connection via Nomachine**

1)  In LAN mode, search for and connect to the Wi-Fi you have set up on the computer, as shown in the image below.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image126.png" style="width:500px" />

2)  Open NoMachine, input the IP address “**192.168.11.134**” in the search bar in AP mode, and click “**Configure connection to new host** 192.168.11.134”.

3)  Note: If you are unsure the IP address, please refer to LAN Mode Connection (Optional).

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image127.png" style="width:500px" />

4)  After opening NoMachine, change Name to “**Robot**” and click “**add**”.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image109.png" style="width:500px" />

5)  Then double the name “**Robot**”.

    <img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image110.png" style="width:500px" />

6)  Fill in the provided username and password based on the control board version：

Initial configuration for **Jetson Nano** board:

username: hiwonder, password: hiwonder

Initial configuration for **Jetson Orin Nano** board:

username: ubuntu, password: ubuntu

Initial configuration for **Jetson Orin NX** board:

username: ubuntu, password: ubuntu

7)  Check the "**Save this password**" box, then click the Login button. Below will take “**Jetson Nano**” version as an example:

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image111.png" style="width:500px" />

8)  Afterwards, you will see the remote desktop of the Jetson Nano open up.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image112.png" style="width:500px" />

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image113.png" style="width:500px" />

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image114.png" style="width:500px" />

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image115.png" style="width:500px" />

* **Establish Connection via VNC**

1)  In LAN mode, search for and connect to the Wi-Fi you have set up on the computer, as shown in the image below.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image126.png" style="width:500px" />

2)  Next, open VNC software. In the VNC Viewer, enter the IP address “**192.168.11.134**” in the search bar in LAN mode, and press Enter. If you receive a security warning, click '**Continue**'.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image128.png" style="width:500px" />

3)  Wait for the connection window to pop up, then enter the following in order: Username \> Password \> Click 'Remember password' \> Click 'OK':

**Username: pi**

**Password: raspberrypi**

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image118.png" style="width:500px" />

4)  Once connected, the Raspberry Pi's remote desktop will appear as shown in the image below.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image119.png" style="width:500px" />

* **Establish Connection via MobaXterm**

1)  n the main interface, click “**Session**” on the top right corner to create a session. In the session interface, enter the recorded robot IP address “**192.168.11.134**”, and then click “**OK**”.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image120.png" style="width:500px" />

2)  Choose SSH.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image121.png" style="width:500px" />

3)  Enter the IP address obtained in LAN mode: 192.168.11.134

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image129.png" style="width:500px" />

4)  As pictured, you need to select the third option.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image123.png" style="width:500px" />

5)  The interface will prompt users to enter the login as and password. It is necessary to fill in the username and password based on the control board version (Below will take Jetson Nano version as an example)：

Initial configuration for **Jetson Nano** board:

username: hiwonder, password：hiwonder

Initial configuration for **Jetson Orin Nano** board:

username: ubuntu, password：ubuntu

Initial configuration for **Jetson Orin NX** board:

username: ubuntu, password: ubuntu

Initial configuration for **Raspberry Pi 5** board:

username: pi, password: raspberrypi

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image124.png" style="width:500px" />

> [!NOTE]
>
> **Note:**
>
> * The username must be entered in lowercase mode. Even if the username includes uppercase letters during setup, it must be entered in lowercase when logging in.
>
> * The username will be visually displayed. After entering it, press Enter to proceed to the password input.
>
> * The password will not be visually displayed. After entering the password, press Enter to log in.

6)  When the input password is correct, you will get access to the system. The system interface is as pictured:

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image125.png" style="width:500px" />

### 1.6.4 Method to Restore WI-FI Option in Menu Bar

1. Double click on <img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image130.png" style="width:0.36736in;height:0.30347in" /> to open the command-line terminal.

2. Enter the command “**cd hiwonder-toolbox/**” and press Enter to access the configuration file directory.

   ```py
   cd hiwonder-toolbox/
   ```

3. Enter the command “**vim hiwonder_wifi_conf.py**” and press Enter to open the configuration file.

   ```py
   vim hiwonder_wifi_conf.py
   ```

4)  Firstly, you need to change the value of “**HW_WIFI_MODE**” to 2. The value “1” represents AP direction connection mode, while “2” represents LAN mode.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image133.png" style="width:500px" />

5) Then, modify “**HW_WIFI_STA_SSID**” and “**HW_WIFI_STA_PASSWORD**” to match the name and password of your router’s WIFI.

> [!NOTE]
>
> Note: Choosing a 5G Wi-Fi signal will provide higher transmission speeds. If you experience lag when connected to a regular Wi-Fi network, you can switch to a 5G Wi-Fi network for better performance.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image134.png" style="width:500px" />

6\) Press “**Esc**” and enter “**:wq**” to save and exit the file.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image135.png" style="width:500px" />

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image136.png" style="width:500px" />

7. Enter the command “**sudo systemctl restart hw_wifi.service**” to restart the robot's Wi-Fi service. After a short wait, the LED1 on the expansion board will remain solid.

   ```py
   sudo systemctl restart hw_wifi.service
   ```

Reminder: After restarting the robot's Wi-Fi service, NoMachine will automatically disconnect. This is because the device has been set to LAN mode, and the connection to a different Wi-Fi network causes the IP address to change from the original one.

8)  First, connect your phone to the 5G network. In this case, we will use the "**Hiwonder_5G**" Wi-Fi as an example.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image55.png" style="width:500px" />

8)  Open “**WonderAi**” APP, then tap “Advanced**-\>JetRover(Mecanum)**” in sequence.

    > [!NOTE]
    >
    > Note: “**JetRover (Track)**” for tank chassis version, and “**JetRover(Acker)**” for Ackerman chassis version.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image56.png" style="width:500px"  />

9)  Wait for a moment. The main interface will display the corresponding robot icon and name.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image138.png" style="width:500px" />

10) Long and hold the robot icon in app to view the IP address and device ID assigned to the robot.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image139.png" style="width:500px" />

11) Enter the obtained IP address into NoMachine to establish a remote connection. For detailed instructions, please refer to the tutorial "**[1.6.2 AP Direct Connection Mode.]()**"

12) To switch to direct connection mode, re-edit the configuration file, comment out all the lines, then save and restart the robot.

### 1.6.5 USB Cable Fixed IP Connection

> [!NOTE]
>
> **Note: This section is only supported for Jetson series main control versions! It is not supported for Raspberry Pi 5 main control versions!**

The robot can improve the remote operation smoothness through enabling remote NDIS compatible device, using the fixed IP “**192.168.55.1**”. This method does not require connecting to the robot’s hotspot or WiFi in the local network. The specific operation steps are as follows:

1)  Connect the Jetson Nano board to your computer using Micro-USB cable.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image140.png" style="width:500px" />

2)  For Jetson Orin Nano/Jetson Orin NX board, connect it to the computer using a Type-C cable.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image141.png" style="width:500px" />

3)  Right click “**This computer**” to select “**Manage**”.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image142.png" style="width:500px" />

4)  Click on "Device Manager". In the "**Network Adapters**" section, locate the NDIS driver. Right-click to select "**Update Driver**".

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image143.png" style="width:500px" />

5)  After the driver is installed, you need to refer to either “**AP Direction Connection Mode**” or “**LAN Mode Connection**” to log in the system. You only need to modify the address bar to “**192.168.55.1**”.

### 1.6.6 System Command-line Terminal Introduction

During the subsequent chapters of the course, it is especially important to pay attention to the ROS version corresponding to the command line terminal. If an incompatible terminal is opened, entering commands will result in errors. For example, entering ROS1 commands in a ROS2 terminal, and so on.

The following sections will introduce the terminals for Jetson Nano, Jetson Orin Nano, Jetson Orin NX, and Raspberry Pi 5 versions.

* **Jetson Nano Terminal**

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image144.png" style="width:100px" />

The command-line terminal within the red box is for ROS1 environment :

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image145.png" style="width:500px" />

The command-line terminal within the yellow box is for ROS2 environment :

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image146.png" style="width:500px" />

* **Jetson Orin Nano Version**

Jetson Orin Nano version only has the command-line terminal for ROS2 environment. <img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image147.png" style="width:0.71875in;height:0.63542in"  />

* **Jetson Orin NX Version**

Jetson Orin NX version only has the command-line terminal for ROS2 environment. <img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image147.png" style="width:0.71875in;height:0.63542in"  />

* **Raspberry Pi5 Version**

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image148.png" style="width:1.30208in;height:1.11458in" />

The image above shows the ROS2 environment command-line terminal, which includes robot-related functions and features. It will be frequently used in the subsequent courses, and the icon style is shown in the image below:

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image149.png" style="width:500px" />

The box in the above image highlights the system’s command-line terminal, which contains system files related to the robot. It is located in the task toolbar at the top of the system interface and is used less frequently.

> [!NOTE]
>
> **Note: In the subsequent operations, please follow the tutorial steps carefully and use the appropriate command-line terminal to execute commands, as failure to do so may affect the robot's functions and features.**

### 1.6.7 Robot Version Configuration Tool Instruction

This section will introduce the robot system’s built-in version configuration tool, which allows switching between different chassis configurations, Lidar types, camera models, and Chinese/English voice settings.

It is applicable to the following users:

1.  Users who have purchased a robot kit without control board.

2.  Users who have re-flashed the system image.

3.  Users who have replaced other accessories supported by the system, such as switching from a Mecanum wheel chassis to a tracked chassis.

For users who have purchased a complete robot kit with control board, they only need to be aware of the content and do not need to take action. Before learning how to use the tool, you can first confirm the system version configuration based on the control board model you purchased.

* **Tool Introduction and Usage** 

1)  Use the Nomachine remote desktop connection tool to connect to the robot. For specific connection methods, refer to **[1.6.1 Remote Connection Tool Introduction & Installation]()**.

2)  Once connected to the remote desktop, double-click on <img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image150.png" style="width:0.72917in;height:0.79167in" /> or<img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image151.png" style="width:0.52569in;height:0.63542in" /> for robot system configuration.

3)  In the interface, select the desired robot type options, as shown in the red-boxed area in the following image.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image152.png" style="width:500px"  />

4)  Users can check your order information to obtain the hardware version. It typically needs to configure the following ares in the tool: Depth Camera, Lidar, Machine, ASR. Please refer to the table below. **Keep the other options at their default settings and avoid incorrect operation to prevent issues!**

<table  class="docutils-nobg" style="margin:0 auto" border="1">
<colgroup>
<col style="width: 50%" />
<col style="width: 50%" />
</colgroup>
<thead>
<tr>
<th style="text-align: center;"><strong>Option</strong></th>
<th style="text-align: center;"><strong>Configuration</strong></th>
</tr>
</thead>
<tbody>
<tr>
<td style="text-align: center;"><strong>Depth camera</strong></td>
<td style="text-align: center;"><strong>Dabai/usb_cam</strong></td>
</tr>
<tr>
<td style="text-align: center;"><strong>Lidr</strong></td>
<td style="text-align: center;"><strong>A1/A2/G4/S2L/LD14P</strong></td>
</tr>
<tr>
<td style="text-align: center;"><strong>Machine</strong></td>
<td style="text-align: center;"><p><strong>JetRover_Mecanum</strong></p>
<p><strong>JetRover_Acker<br />
JetRover_Tank</strong></p></td>
</tr>
<tr>
<td style="text-align: center;"><strong>ASR</strong></td>
<td style="text-align: center;"><strong>Chinese/English</strong></td>
</tr>
</tbody>
</table>


5)  For instance, if you need to switch the machine type from Ackerman version to Mecanum version.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image152.png" style="width:500px"  />

6)  After selecting, click **Save \> Apply \> Quit** in sequence. It is crucial to follow this order to successfully switch the chassis type.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image153.png" style="width:500px"  />

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image154.png" style="width:500px"  />

7)  Wait for the robot body to emit a "beep" sound, indicating that the chassis type switch was successful.

* **Configuration Modification Verification** 

The following demonstrations are provided for four types of cotrol boards: Jetson Nano, Jetson Orin Nano, Jetson Orin NX, and Raspberry Pi 5. For an introduction to the differences in terminals, please refer to the "System Command-line Terminal Introduction.

**1. Jetson Nano Control Board：**

1)  If using ROS1, you can verify by clicking on <img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image155.png" style="width:0.39375in;height:0.34792in" />. For ROS2, use the ROS2 command-line terminal<img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image156.png" style="width:0.39375in;height:0.37847in" />.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image157.png" style="width:500px" />

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image158.png" style="width:500px" />

**2. Jetson Orin Nano/Jetson Orin NX Board**

Verify by clicking on <img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image155.png" style="width:0.39375in;height:0.34792in" />.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image159.png" style="width:5.76597in;height:2.01597in" />

**3. Raspberry Pi 5 Board**

Click on <img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image155.png" style="width:0.39375in;height:0.34792in" /> to verify the ROS2 system environment.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image160.png" style="width:500px" />

### 1.6.8 Remote Desktop Resolution Adjustment

If you use the robot with an additional monitor, the remote connection tool may incur low or blurry resolution situations. Users can refer to the following steps to adjust the desktop resolution, typically setting the screen resolution to 1920x1080.

> [!NOTE]
>
> **Special Reminder: For JetRover Jetson Nano version, the HDMI cable of the screen must be disconnected before booting up to synchronize the correct desktop resolution.**

* **Jetson Orin Nano/Jetson Orin NX Version**

After establishing remote connection via Nomachine, follow the steps below to configure the settings (It is necessary to reset the parameters after reconnection).

1)  Move the mouse cursor to the upper-right corner, and wait for the fold-out menu to appear, as indicated by the red arrow in the image below:

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image161.png" style="width:500px" />

2)  Then click on it and choose “**Display**” in the pop-up.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image162.png" style="width:500px" />

3)  Then, click “**Change settings**”.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image163.png" style="width:500px" />

4)  Adjust the Resolution slider to 1920x1080 (this value is not fixed, set it to match you computer), and click "Modify" to apply the changes.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image164.png" style="width:500px" />

5)  Click the back button consecutively to return to the desktop. You will notice that the remote desktop resolution has been changed (If the robot is connected with a display, which will also be updated. However, this change will be lost after a reboot, and you will need to follow the above steps again to adjust the resolution.)

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image165.png" style="width:500px" />

* **Raspberry Pi 5 version**

1)  Click “**logo \> Preferences \> Screen Configuration**” in sequence.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image166.png" style="width:500px" />

2)  Then, click “**Layout \> Screens \> HDMI-1 \> Resolution \> 1920x1080**” in the pop-up window.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image167.png" style="width:500px" />

3)  This will adjust the remote desktop resolution to 1920x1080.

## 1.7 Wireless Handle Control

### 1.7.1 Usage Guideline

1)  Please first confirm if the handle receiver is already plugged in before starting the robot. Ignore this step if it is already plugged in. (the USB handle receiver has already been inserted into the robot during installing )

2)  When installing the batteries, please distinguish the positive and negative poles of the battery.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image168.png" style="width:500px"  />

3)  After powering on the robot, the app auto-start service will be automatically enabled (app service includes handle control service). If this service is not disabled, no more operations require for it, the robot is allowed to be controlled directly after connecting the wireless handle.

4)  Due to the interactivity of the handle control, when multiple robots are in the same field, it’s not recommended to use this function to avoid accidental connection and control errors.

5)  After turning on the wireless handle, if it does not connect the robot within 30 seconds, or no operations with the handle within 5 minutes after connecting to the robot, it will automatically enter sleep mode. If you need to wake up the handle, please press “**START**” button to exit the sleep mode.

### 1.7.2 Device Connection

1)  After the robot is powered on, turn on the switch of the handle. At this point, two LED lights (red and green lights) on the handle will blink simultaneously.

2)  Wait for a few seconds, and the robot and handle will be paired automatically. When pairing is successful, the green LED light will remain on, and the red LED light will turn off.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image169.png" style="width:500px" />

### 1.7.3 Button Instructions

The below table will introduce the functions of the buttons and joysticks (take robot as the first person view):

> [!NOTE]
>
> **Note: Gently pushing the joystick in any direction enables low-speed movement.**

<table  class="docutils-nobg" style="margin:0 auto" border="1">
<colgroup>
<col style="width: 27%" />
<col style="width: 36%" />
<col style="width: 36%" />
</colgroup>
<thead>
<tr>
<th style="text-align: center;"><strong>Button</strong></th>
<th style="text-align: center;"><strong>Function</strong></th>
<th style="text-align: center;"><strong>Instruction</strong></th>
</tr>
</thead>
<tbody>
<tr>
<td style="text-align: center;">START</td>
<td style="text-align: center;">Stop and restore the robot’s body</td>
<td style="text-align: center;">Short press</td>
</tr>
<tr>
<td style="text-align: center;">Push the left joystick up</td>
<td style="text-align: center;">Move forward</td>
<td style="text-align: center;">Push</td>
</tr>
<tr>
<td style="text-align: center;">Push the left joystick down</td>
<td style="text-align: center;">Move backward</td>
<td style="text-align: center;">Push</td>
</tr>
<tr>
<td style="text-align: center;">Left push the left joystick</td>
<td style="text-align: center;"><p>Left translation</p>
<p>(mecanum chassis only)</p></td>
<td style="text-align: center;">Push</td>
</tr>
<tr>
<td style="text-align: center;">Right push the left joystick</td>
<td style="text-align: center;"><p>Right translation</p>
<p>(mecanum chassis only)</p></td>
<td style="text-align: center;">Push</td>
</tr>
<tr>
<td style="text-align: center;">Left push he right joystick</td>
<td style="text-align: center;"><p>Steer left</p>
<p>(Ackerman chassis only)</p></td>
<td style="text-align: center;">Push</td>
</tr>
<tr>
<td style="text-align: center;">Right push he right joystick</td>
<td style="text-align: center;"><p>Steer right</p>
<p>Ackerman chassis only)</p></td>
<td style="text-align: center;">Push</td>
</tr>
</tbody>
</table>


## 1.8 Manual Mapping

This section will help you have a quick experience of mapping and navigation functions. No complicated operations are required, users simply click the corresponding icons on the touch screen to perform functions.

The manual mapping requires users to control the movement of the robot using a wireless handle or keyboard. It’s recommended you to use wireless handle to control robot if only a single robot appears in the application scenario. If there are multiple robots in the scenario, it’s recommended you to a keyboard for control. This is because the wireless handle is interactive, and in environment with multiple robots, it’s advisable not to use this function to avoid accidental connections and control errors.

After mapping is complete, the corresponding map is saved. Then, users can activate the autonomous navigation function to view the mapping results. It's important to note that the map opened with the autonomous navigation function will be the last map created. Regardless of which mapping method is used, the newly created map will overwrite the previous one.

### 1.8.1 Preparation for Mapping

Before powering on the robot, it is necessary to ensure that the touch screen is already installed and connected. 

> [!NOTE]
>
> **Attention: the power cable of the touch screen needs to connect to the C-Touch interface, otherwise, the touch functionality is ineffective.**

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image170.png" style="width:500px" />

Next, you need to confirm whether the handle receiver is securely plugged into the USB port of the robot.

> [!NOTE]
>
> **Note: the handle received is already plugged in the USB hub upon receiving the  robot.**

### 1.8.2 Operation Steps

* **ROS1 Mapping**

> [!NOTE]
>
> **Note**：
>
> * **This section is applicable to Jetson Nano controller**
>
> * **In this mode, it's necessary to set up a closed space beforehand on a flat surface. If obstacles are placed, their height should be at least above the radar's horizontal position.**

1)  Place the robot inside the constructed space.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image171.jpeg" style="width:500px"  />

2)  Click on the screen desktop. Open <img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image172.png" style="width:0.65625in;height:0.55208in" />

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image173.png" style="width:0.88472in;height:2.01111in" />

3)  At this time, Several terminals will be opened to run the program simultaneously. Wait for a while at this point.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image174.png" style="width:500px"  />

4)  When you see the below interface, it indicated that the function is enabled successfully.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image175.png" style="width:500px" />

5)  Now, let’s use control the robot to map using the wireless handle. The function instruction for each button is as follow:

> [!NOTE]
>
> **Note: when using the handling, keep the distance from the robot not too far to avoid disconnection.**

<table  class="docutils-nobg" style="margin:0 auto" border="1">
<colgroup>
<col style="width: 26%" />
<col style="width: 34%" />
<col style="width: 39%" />
</colgroup>
<thead>
<tr>
<th style="text-align: center;"><strong>Button/Joystick</strong></th>
<th style="text-align: center;"><strong>Operation</strong></th>
<th style="text-align: center;"><strong>Function</strong></th>
</tr>
</thead>
<tbody>
<tr>
<td style="text-align: center;">START</td>
<td style="text-align: center;">Short press</td>
<td style="text-align: center;">Exit sleep mode</td>
</tr>
<tr>
<td rowspan="2" style="text-align: center;">Left joystick</td>
<td style="text-align: center;">Push forward/backward</td>
<td style="text-align: center;">Control robot to move forward and backward</td>
</tr>
<tr>
<td style="text-align: center;">Push to left/right</td>
<td style="text-align: center;"><p>Control the movement of left and right translation</p>
<p>(Only supported for Mecanum chassis)</p></td>
</tr>
<tr>
<td style="text-align: center;">Right joystick</td>
<td style="text-align: center;">Push to left</td>
<td style="text-align: center;">Control robot to turn left</td>
</tr>
</tbody>
</table>


You can also choose to control the robot using the keyboard, but it's not recommended here. Keyboard control will be introduced in later courses. If you prefer keyboard control, you'll need to connect to the remote desktop and modify the terminal to enable keyboard control.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image176.png" style="width:500px" />

For the remote system desktop connection, please refer to “**[1.6.1 Remote Connection Tool Introduction & Installation]()”**.

The button instruction for keyboard control is as follow:

| **Buttons** | **Function** | **Instruction** |
|:--:|:--:|:--:|
| W | Move forward | Short press to this button to forward status. Robot will keep moving forward. |
| S | Move backward | Short press to this button to backward status. Robot will keep moving backward. |
| A | Turn left | Long press to interrupt moving forward or backward and make a counter-clockwise turn to the left in place |
| D | Turn right | Long press to interrupt moving forward or backward and make a right turn. |

When the "**W**" or "**S**" key is pressed, the robot will continue moving forward or backward respectively. When the "**A**" or "**D**" key is pressed, the robot will interrupt the forward or backward motion and rotate counterclockwise or clockwise in place respectively. When the "**A**" or "**D**" key is released, the rotation will stop, and the robot will remain stationary.

> [!NOTE]
>
> **Attention: regardless of whether it's a tack robot or a wheeled robot, using keyboard control cannot make the robot perform lateral movement.**

6)  After the movement is complete, press the save button in the bottom left corner, and the robot will save the completed map.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image177.png" style="width:500px" />

The standard for a completed map is as follow:

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image178.jpeg" style="width:500px"  />

7)  After the mapping is complete, if you want to close this function, click interface and tap<img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image179.png" style="width:100px" />.

* **ROS2 Mapping**

> [!NOTE]
>
> **Note**：
>
> * **This section is applicable to Jetson Orin Nano/Jetson Orin NX and Raspberry Pi 5.**
>
> * **In this mode, it's necessary to set up a closed space beforehand on a flat surface. If obstacles are placed, their height should be at least above the radar's horizontal position.**

1)  Place the robot inside the constructed space.

2)  Click on the screen desktop. Open the SLAM icon.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image180.png" style="width:100px" />

3)  At this time, Several terminals will be opened to run the program simultaneously. Wait for a while at this point.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image174.png" style="width:500px"  />

4)  When you see the below interface, it indicated that the function is enabled successfully.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image175.png" style="width:500px" />

5)  Now, let’s use control the robot to map using the wireless handle. The function instruction for each button is as follow:

> [!NOTE]
>
> **Note: when using the handling, keep the distance from the robot not too far to avoid disconnection.**

<table  class="docutils-nobg" style="margin:0 auto" border="1">
<colgroup>
<col style="width: 26%" />
<col style="width: 34%" />
<col style="width: 39%" />
</colgroup>
<thead>
<tr>
<th style="text-align: center;"><strong>Button/Joystick</strong></th>
<th style="text-align: center;"><strong>Operation</strong></th>
<th style="text-align: center;"><strong>Function</strong></th>
</tr>
</thead>
<tbody>
<tr>
<td style="text-align: center;">START</td>
<td style="text-align: center;">Short press</td>
<td style="text-align: center;">Exit sleep mode</td>
</tr>
<tr>
<td rowspan="2" style="text-align: center;">Left joystick</td>
<td style="text-align: center;">Push forward/backward</td>
<td style="text-align: center;">Control robot to move forward and backward</td>
</tr>
<tr>
<td style="text-align: center;">Push to left/right</td>
<td style="text-align: center;"><p>Control the movement of left and right translation</p>
<p>(Only supported for Mecanum chassis)</p></td>
</tr>
<tr>
<td style="text-align: center;">Right joystick</td>
<td style="text-align: center;">Push to left</td>
<td style="text-align: center;">Control robot to turn left</td>
</tr>
</tbody>
</table>


You can also choose to control the robot using the keyboard, but it's not recommended here. Keyboard control will be introduced in later courses. If you prefer keyboard control, you'll need to connect to the remote desktop and modify the terminal to enable keyboard control.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image181.png" style="width:500px" />

For the remote system desktop connection, please refer to “[**1.6.1 Remote Connection Tool Introduction & Installation**]()”.

The button instruction for keyboard control is as follow:

| **Button** | **Function** | **Instruction** |
|:--:|:--:|:--:|
| W | Move forward | Short press to this button to forward status. Robot will keep moving forward. |
| S | Move backward | Short press to this button to backward status. Robot will keep moving backward. |
| A | Turn left | Long press to interrupt moving forward or backward and make a counter-clockwise turn to the left in place |
| **Button** | **Function** | **Instruction** |

When the "**W**" or "**S**" key is pressed, the robot will continue moving forward or backward respectively. When the "**A**" or "**D**" key is pressed, the robot will interrupt the forward or backward motion and rotate counterclockwise or clockwise in place respectively. When the "**A**" or "**D**" key is released, the rotation will stop, and the robot will remain stationary.

> [!NOTE]
>
> **Attention: For Mecanum version, using keyboard control cannot make the robot perform lateral movement.**

6)  After the movement is complete, press the “**Save Map**” button in the bottom left corner, and the robot will save the completed map.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image182.png" style="width:500px" />

7)  The standard for a completed map is as follow:

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image183.png" style="width:500px" />

8)  After the mapping is complete, if you want to close this function, click interface and tap<img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image184.png" style="width:50px" />.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image185.png" style="width:500px" />

## 1.9 Autonomous Mapping

> [!NOTE]
>
> **Note:**
>
> * **This functionality is only applicable to the Jetson Nano controller.**
>
> * **Once this function is enabled, JetAuto will directly start to move. In this case, it requires you to place the robot on a flat surface.**
>
> * **In this mode, users need to set up a closed space in advance, preferably on a flat surface. If obstacles are set inside the space, the minimum height of the obstacle must be higher than the horizontal position of the Lidar. The robot can stop after completing autonomous mapping in the closed space.**

1)  Click the SLAM Automatic to start the autonomous mapping.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image186.png" style="width:100px" />

2)  After opening, the program will automatically run the program in terminal. When the below interface shows up, it indicates that the interface is enabled successfully.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image187.png" style="width:500px" />

3)  After successful startup, the robot will begin to move autonomously and map its surroundings.

4)  When the robot completes autonomous mapping in this closed space, it will stop and automatically save the map.

## 1.10 Autonomous Navigation 

> [!NOTE]
>
> **Note:**
>
> * Autonomous navigation will read the most recently created map, whether manually and autonomously built. If both methods are used simultaneously, it will save the last map created.
>
> * If you want to learn about how to save multiple maps, please delve into the content in “**[6. Mapping and Navigation Courses]()**” .

### 1.10.1 ROS1 Autonomous Navigation

> [!NOTE]
>
> **Reminder: This section is only applicable to the Jetson Nano controller.**

1)  Start the robot. Click the Navigation icon to experience the quick navigation function.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image188.png" style="width:100px" />

2)  After opening, the terminal will run the program. When you see the following interface, it indicates that the opening was successful.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image189.jpeg" style="width:500px"  />

3)  In the manual bar, “**2D Pose Estimate**” is used to set the initial position of JetAuto, “**2D Nav Goal**” is used to set a target point and “**Publish Point**” is used to set multiple target points.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image190.png" style="width:500px" />

4)  Click on <img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image191.png" style="width:100px" />to select one point by clicking the map interface as the target destination. If you press and drag with the mouse, you can also select the orientation of the robot after it reaches the target point. After the selection is complete, the robot will automatically generate a route and move to the target point. (Note: <img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image192.png" style="width:500px" /> changes the initial position of the robot on the map and <img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image191.png" style="width:100px" />is the target point for single-point navigation).

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image193.jpeg" style="width:500px"  />

5)  Click<img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image194.png" style="width:100px" />and press the left mouse at a certain point on the map to set it as the destination point. If you need to perform multi-point navigation, please follow the same operation steps to set multiple destination points.

> [!NOTE]
>
> **Note: you need to click “Publish Point” once before setting each destination point.**

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image195.jpeg" style="width:500px"  />

6)  After setting the destination point, click “**Start Navigation**” in the bottom left corner to start navigation. During navigation, the robot will automatically avoid obstacles. If need to stop navigation, click “**Stop Navigation**”, and then robot will stop after reaching the destination point.

For example, in the multi-point navigation shown in the above figure, marked as 1, 2 and 3 on the map (take three marks as example). If the robot is traveling between marks 1 and 2, and when you press “**Stop Navigation**”, it will stop moving after reaching mark 2.

7)  If clicking “**Clear Goals**”, all marks will be cleared.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image196.jpeg" style="width:500px"  />

### 1.10.2 ROS2 Autonomous Navigation

> [!NOTE]
>
> **Reminder: This section is only applicable to Jetson Orin Nano, Jetson Orin NX and Raspberry Pi 5 controllers.**

1)  Start the robot. Click the Navigation icon to experience the quick navigation function.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image197.png" style="width:500px" />

2)  After opening, the terminal will run the program. When you see the following interface, it indicates that the opening was successful.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image198.png" style="width:500px" />

3)  In the manual bar, “**2D Pose Estimate**” is used to set the initial position of the robot, “**2D Nav Goal**” is used to set a target point and “**Publish Point**” is used to set multiple target points, “**Nav2 Gaol**” is used to set more complex navigation targets, such as specifying a target point, a target pose or a target field.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image199.png" style="width:500px" />

4)  <img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image192.png" style="width:500px" />changes the initial position of the robot on the map and mark the robot’s actual position.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image200.png" style="width:500px" />

5)  Click on <img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image192.png" style="width:500px" />to select one point by clicking the map interface as the target destination. If you press and drag with the mouse, you can also select the orientation of the robot after it reaches the target point.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image201.png" style="width:500px" />

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image202.png" style="width:500px" />

6)  Click<img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image203.png" style="width:500px" /> in the bottom left corner to enable multiple-point navigation. Then click <img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image204.png" style="width:100px" /> to set a target point. Hold and drag that point to select the target point direction.

> [!NOTE]
>
> **Note: you need to click “Nav2 Goal” once before setting each destination point.**

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image205.png" style="width:500px" />

7)  After setting the destination point, click “**Start Nav Through Navigation**” in the bottom left corner to start navigation. During navigation, the robot will automatically avoid obstacles.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image206.png" style="width:500px"  />

## 1.11 Hardware Introduction

This chapter focuses on the hardware components of ROS robots, including electronic control system, ROS controller, Lidar and depth camera, etc.

### 1.11.1 Hardware System Wiring Diagram

The hardware connection system diagram of the robot based in the mini STM32 controller is shown below. The specific model and ROS controller may var, and the detailed instructions will be provided in the following content.

* **Jetson Nano Controller Version：**

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image207.png" style="width:500px"  />

* **Jetson Orin Nano Controller Version/Jetson Orin NX Controller Version：**

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image208.jpeg" style="width:500px"  />

* **Jetson Orin NX Controller Version**

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image209.jpeg" style="width:500px"  />

* **Raspberry Pi 5 Controller Version：**

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image210.png" style="width:500px"  />

### 1.11.2 Electronic Control System Introduction

* **STM32 Controller**

The robot's electronic control system uses a mini STM32 controller (hereafter referred to as the STM32 controller) as the low-level motion controller. It connects multiple DC encoder reduction motors and has a built-in IMU accelerometer and gyroscope sensor. This setup enables the control of the robot's chassis movement and sensor data collection.

The controller uses the STM32F407VET6 as the main controller, which features ARM's Cortex-M4 core, a 168 MHz main frequency, 512K of on-chip FLASH, 192K of on-chip SRAM, and integrates FPU and DSP instructions. The system block diagram of the STM32F40x/41x series is shown below:

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image211.jpeg" style="width:500px"  />

The resource layout on the front side of the control board is shown in the diagram below:

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image212.jpeg" style="width:500px"  />

The control board also provides a schematic diagram, feature an SWD debugging interface, supports USB serial port program download, and enables STM32 programming for secondary development. It offers onboard resources and peripheral example code for users to learn and use conveniently.

* **Power Supply Instruction**

The 11.1V 6000mAh Lithium battery is tailored for the robot, with a charging voltage of 12.6V. It features overcharge protection, overcurrent protection, over-discharge protection, and short-circuit protection.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image213.png" style="width:500px" />

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image214.png" style="width:500px" />

Please use the special charged provided in the kit to charge the robot. Users can view the charging status by observing the indicator on the charger. The red light indicates that the battery is charging, while the green indicates that the charging is complete. (Green light when idle, red light when charging, and green light when fully charged)

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image215.jpeg" style="width:500px"  />

### 1.11.3 Robot Controller

JetRover offers comprehensive support for various ROS controllers, with similar usage methods across controllers. Raspberry Pi 5 uses the Debian 12 system, while the Jetson series runs on the Ubuntu system. Below is a comparison of the controller specifications for Raspberry Pi 5, Jetson Nano, Jetson Orin Nano, and Jetson Orin NX:

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image216.png" style="width:500px" />

* **Jetson Nano Version**

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image217.png" style="width:500px" />

The robot utilizes Jetson Nano as its ROS controller, consisting of the Jetson Nano board and Jetson expansion board. The motherboard is a compact yet powerful computer capable of running the mainstream deep learning frameworks, providing the computational power required for most artificial intelligence projects.

The expansion board features connections for LEDs and buttons, enabling users to monitor network status through LED flashes and switch network modes using the button. It also includes reserved GPIO and I2C interfaces. **The mainboard is installed with the Ubuntu18.04 system, and establishes the ROS Melodic environment for robot operations. Additionally, the ROS2 Humble environment is deployed within a Docker container.**

* **Jetson Orin Nano Version**

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image218.png" style="width:500px" />

The robot is equipped with the Jetson Orin Nano main controller, which comprises a Jetson Orin Nano motherboard and a Jetson expansion board. The motherboard is a compact yet powerful computer that can run mainstream deep learning frameworks, providing the computational power necessary for most artificial intelligence projects.

The expansion board features connections for LEDs and buttons, enabling users to monitor network status through LED flashes and switch network modes using the button. It also includes reserved GPIO and I2C interfaces. **The controller is installed with Ubuntu 22.04 and features the ROS2 Humble environment for robotic operating systems.**

* **Jetson Orin NX Version**

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image218.png" style="width:500px" />

The robot is equipped with the Jetson Orin NX 8/16GB main controller, which comprises a Jetson Orin NX motherboard and a Jetson expansion board. Compared to the Jetson Orin Nano 4GB, the Jetson Orin NX 16GB offers faster processing speeds, improved memory read/write speeds, and a fivefold increase in performance.

The expansion board features connections for LEDs and buttons, enabling users to monitor network status through LED flashes and switch network modes using the button. It also includes reserved GPIO and I2C interfaces. **The controller is installed with Ubuntu 22.04 and features the ROS2 Humble environment for robotic operating systems.**

* **Raspberry Pi5 Version**

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image219.png" style="width:500px" />

The robot uses a Raspberry Pi 5 as its ROS main controller, which includes the Raspberry Pi 5 controller and a mini expansion board. The controller is a compact yet powerful computer capable of running major deep learning The mini expansion board adds functionality with LEDs and buttons, allowing users to monitor network status via LED indicators and switch network modes using a mobile phone. It also features reserved GPIO and I2C interfaces for additional expansion.

**The controller operates with the Raspberry Pi Debian12 official system and has a ROS2 environment set up in Docker.**

Overall, the mini expansion board for the Raspberry Pi 5 is stable, energy-efficient, and highly expandable, capable of supporting mainstream deep learning frameworks and meeting the computational needs of most AI applications.

### 1.11.4 Bus Servo

JetRover is equipped with a 6-degree-of-freedom robotic arm, composed of intelligent bus servos and metal components.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image220.png" style="width:200px" />

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image221.png" style="width:200px" />

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image222.png" style="width:200px" />

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image223.png" style="width:200px" />

HTD-35H\*3 (robot body) + HTS-20H servo \*1 (pan-tilt) + HTS-31H (gripper) + HTD-35H bus servo (wrist)

### 1.11.5 Hall Encoder DC Gear Motor

The Hall speed encoder is a speed measurement module that uses a Hall sensor encoder with a robust magnetic disk, and generates AB two-phase output pulse signals. In this setup, the motor operates at 12V, and the diagram below illustrates the motor used in the robot, along with their pin configurations.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image224.png" style="width:500px" />

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image225.jpeg" style="width:500px"  />

### 1.11.6 Vision Module

* **Depth Camera**

JetRover uses the Dabai DCW binocular structured light depth camera. The depth image resolution can reach up to 1920×1080@5/10fps, with an average power consumption of less than 1.2W. The terminal only requires a USB2.0 interface to obtain high-precision 3D depth information for backend usage, enabling the robot to perform functions such as perception, obstacle avoidance, and navigation.

On the robot, it is primarily used to implement functions like OpenCV and can also be utilized for deep learning and KCF object tracking, among other visual applications.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image226.png" style="width:500px" />

### 1.11.7 Lidar

Lidar utilizes laser technology to accurately pinpoint the locations. Its applications extend to various scenarios, including the field of robotics, enabling functions such as Lidar obstacle avoidance, following, SLAM mapping, and navigation.

This robot is compatible with different Lidar models, including A1 Lidar and G4 Lidar. Users can choose Lidar versions according to your requirement. Below are listed two types of Lidar:

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image227.png" style="width:500px" />

<p style="text-align:center">Lidar A1</p>

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image228.png" style="width:500px" />

<p style="text-align:center"> Lidar G4</p>

### 1.11.8 Others

* **Microphone Array Module**

This module is optional in robot kit. It can allow robot to realize voice wake-up and voice control capabilities.

This robot uses the R818 noise reduction board and a ring-shaped six-way microphone array, which features a planar distribution structure composed of six microphones. It is a system that samples and processes the spatial characteristics of the sound field. It can perform sound source localization, suppress background noise, interference, reverberation, and echo, and achieve 360° equivalent sound reception.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image229.png" style="width:500px" />

The noise reduction board can suppress background noise, interference, reverberation, and echoes, as shown in the image below:

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image230.png" style="width:500px" />

* **7-inch LCD Screen**

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image231.png" style="width:500px" />

**(The system desktop shown in image is for demonstration purpose only, please rely on the actual product for accuracy.)**

The 7-inch LCD screen is optional hardware with both display and touch functionalities. In addition to directly display the robot’s system desktop, it allows for a quick experience of mapping and navigation function by touching icons on the desktop.

* **PS2 Wireless Controller**

The STM32 control board has USB PS2 wireless handle receiver connected, allowing chassis movement control through a PS2 controller.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image232.png" style="width:500px" />

The handle receiver is connected to the position highlighted in the below red box:

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image233.png" style="width:500px" />

* **OLED Display Module**

The OLED display module mainly uses 0.91-inch blue light OLED screen, featuring a viewing range, fast respond, stable graphics, high brightness and high resolution. Its driver chip is the SSD1306, and by controlling this chip, we can mange the content displayed on OLED module. It is used on robot for displaying WIFI name and voltage information.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image234.png" style="width:500px" />

##  1.12 ROS Usage Introduction

The core of ROS robot mainly consists of two parts. The first part is the chassis, primarily involving the underlying STM32 controller responsible for robot motion control and sensor data acquisition. The second part is the ROS main control unit (Jetson Nano/Jetson Orin Nano/Jetson Orin NX/RaspberryPi5), running the ROS system and relevant functional routine algorithms.

### 1.12.1 ROS Main Control Hardware Connection

The standard connection method requires a power cable and a USB serial cable to communicate with the ROS main controller via the onboard USB serial port. The STM32 requires a power supply of 9-24V, and the ROS main controllers (Jetson Nano and Raspberry Pi 5) can be directly connected to the STM32's power output port (5V) for power supply through the STM32 power input. The Jetson Orin Nano/Jetson Orin NX requires a 12V power supply.

### 1.12.2 ROS Serial Port Communication Instruction

Serial communication is a common output and transmission method used in microcontroller development and robot manufacturing. This product also utilizes serial communication to communicate with the upper-level machine, Jetson Nano/Jetson Orin Nano/Jetson Orin NX/Raspberry Pi 5, and the lower-level STM32 controller.

In order to facilitate communication between software tools and various products, Hiwonder has standardized a communication protocol based on hexadecimal data transmission called the RRC Communication Protocol. Subsequent Wonder products use this communication protocol for programming and communication between the upper and lower-level machines.

* **Communication Protocol** 

The instruction is written in hexadecimal. If you are unfamiliar with the calculation method, you can refer to the following: use a calculator tool for base conversion. For converting negative numbers and floating-point numbers into hexadecimal, please search for tutorials online

Command format:

<table  class="docutils-nobg" style="margin:0 auto" border="1">
<colgroup>
<col style="width: 19%" />
<col style="width: 19%" />
<col style="width: 19%" />
<col style="width: 20%" />
<col style="width: 20%" />
</colgroup>
<thead>
<tr>
<th style="text-align: center;">Frame header</th>
<th style="text-align: center;">Function code</th>
<th style="text-align: center;">Data length</th>
<th style="text-align: center;">Parameter</th>
<th style="text-align: center;">Checksum</th>
</tr>
</thead>
<tbody>
<tr>
<td style="text-align: center;">0xAA 0x55</td>
<td style="text-align: center;"><p>（uint8_t）</p>
<p>Function</p></td>
<td style="text-align: center;"><p>（uint8_t）</p>
<p>Length</p></td>
<td style="text-align: center;">Data</td>
<td style="text-align: center;"><p>（uint8_t）</p>
<p>CRC</p></td>
</tr>
</tbody>
</table>


**Frame Header**: Receiving consecutive 0xAA and 0x55 indicates that a data packet has arrived.

**Function Code**: Indicates the purpose of an information frame.

**Data Length**: Specifies the number of parameters.

**Parameter**: Additional control information required apart from the function instruction.

**Checksum**: Verifies whether the data is correct, using the CRC check method (calculates the CRC value of Function, Length, and Data, taking the lower 8 bits).

* **User Sending Data to Control Board**

The control board already has a dedicated UART-to-USB circuit. Simply connect the UART3 port to the PC software using a data cable to enable communication. Here, we use LED control as an example.

**LED light control: Command name PACKET_FUNC_LED, velue 1**

<table  class="docutils-nobg" style="margin:0 auto" border="1">
<colgroup>
<col style="width: 15%" />
<col style="width: 20%" />
<col style="width: 12%" />
<col style="width: 36%" />
<col style="width: 14%" />
</colgroup>
<thead>
<tr>
<th style="text-align: center;"><strong>Frame header</strong></th>
<th style="text-align: center;"><strong>Function code</strong></th>
<th style="text-align: center;"><strong>Data length</strong></th>
<th style="text-align: center;"><strong>Parameter</strong></th>
<th style="text-align: center;"><strong>Checksum</strong></th>
</tr>
</thead>
<tbody>
<tr>
<td style="text-align: center;">0xAA 0x55</td>
<td style="text-align: center;">PACKET_FUNC_LED</td>
<td style="text-align: center;">7</td>
<td style="text-align: left;"><p>Parameter 1:（uint8_t）led_id Parameter 2: (uint16_t) On duration (ms)</p>
<p>Parameter 3: (uint16_t) Off duration (ms)</p>
<p>Parameter 4: (uint16_t) Number of cycles</p></td>
<td style="text-align: center;">CRC</td>
</tr>
</tbody>
</table>


For example,

① Control LED light to blink 5 times, with each blink lasting 100ms on and 100ms off:

<table  class="docutils-nobg" style="margin:0 auto" border="1">
<colgroup>
<col style="width: 15%" />
<col style="width: 20%" />
<col style="width: 12%" />
<col style="width: 36%" />
<col style="width: 14%" />
</colgroup>
<thead>
<tr>
<th style="text-align: center;"><strong>Frame header</strong></th>
<th style="text-align: left;"><strong>Function code</strong></th>
<th style="text-align: center;"><strong>Data length</strong></th>
<th style="text-align: center;"><strong>Parameter</strong></th>
<th style="text-align: center;"><strong>Checksum</strong></th>
</tr>
</thead>
<tbody>
<tr>
<td style="text-align: left;">0xAA 0x55</td>
<td style="text-align: left;">PACKET_FUNC_LED</td>
<td style="text-align: left;">7</td>
<td style="text-align: left;"><p>Parameter 1：0x01（1）</p>
<p>Parameter 2：0x64 0x00（100）</p>
<p>Parameter 3：0x64 0x00（100）</p>
<p>Parameter 4：0x05 0x00（5）</p></td>
<td style="text-align: left;">CRC</td>
</tr>
</tbody>
</table>

> [!NOTE]
>
> **Note: This is in little-endian mode. For example, the value 5 (decimal) of uint32_t would be written as 0x00 0x05 in little-endian mode, where the lower byte comes first. Therefore, when sending data, it should be written as 0x05 0x00.**

* **Control Board Sending Data to User**

**Bus servo data upload: Command name PACKET_FUNC_BUS_SERVO, value 5**

In this example, we will upload the bus servo position.

<table  class="docutils-nobg" style="margin:0 auto" border="1">
<colgroup>
<col style="width: 15%" />
<col style="width: 24%" />
<col style="width: 10%" />
<col style="width: 38%" />
<col style="width: 10%" />
</colgroup>
<thead>
<tr>
<th style="text-align: center;"><strong>Frame header</strong></th>
<th style="text-align: left;"><strong>Function code</strong></th>
<th style="text-align: center;"><strong>Data length</strong></th>
<th style="text-align: center;"><strong>Parameter</strong></th>
<th style="text-align: center;"><strong>Checksum</strong></th>
</tr>
</thead>
<tbody>
<tr>
<td style="text-align: left;">0xAA 0x55</td>
<td style="text-align: left;">PACKET_FUNC_BUS_SERVO</td>
<td style="text-align: left;">5</td>
<td style="text-align: left;">Parameter 1: (uint8_t) servo_id<br />
Parameter 2: (uint8_t) 0x05 (Sub-command)<br />
Parameter 3: (int8_t) Success status (0: Success; -1: Failure)<br />
Parameter 4: (int16_t) Servo position</td>
<td style="text-align: left;">CRC</td>
</tr>
</tbody>
</table>


When a read command is received, the corresponding servo position parameter is read and uploaded to the PC software.

For example,

Upload the angle of servo 5 as 30°, corresponding to a pulse width of 833.

<table  class="docutils-nobg" style="margin:0 auto" border="1">
<colgroup>
<col style="width: 15%" />
<col style="width: 24%" />
<col style="width: 10%" />
<col style="width: 39%" />
<col style="width: 10%" />
</colgroup>
<thead>
<tr>
<th style="text-align: center;"><strong>Frame header</strong></th>
<th style="text-align: center;"><strong>Function code</strong></th>
<th style="text-align: center;"><strong>Data length</strong></th>
<th style="text-align: center;"><strong>Parameter</strong></th>
<th style="text-align: center;"><strong>Checksum</strong></th>
</tr>
</thead>
<tbody>
<tr>
<td style="text-align: center;">0xAA 0x55</td>
<td style="text-align: center;">PACKET_FUNC_BUS_SERVO</td>
<td style="text-align: center;">5</td>
<td style="text-align: left;">Parameter 1: 0x05 (5)<br />
Parameter 2: 0x05 (Sub-command)<br />
Parameter 3: 0x00 (0 - Success)<br />
Parameter 4: 0x41 0x03 (833 in little-endian format)</td>
<td style="text-align: center;">CRC</td>
</tr>
</tbody>
</table>


##  1.13 System Software Framework 

Before starting this section, you need to use Nomachine remote desktop connection software. For detailed instructions, please refer to "**[1.6 Development Environment Setup and Configuration]()**".

### 1.13.1 ROS1 Directory and Functional File Introduction

**Reminder: this section is only applicable to the Jetson Nano controller.**

1. Click<img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image147.png" style="width:50px"  />to open the command terminal. Enter the below command, and then press Enter to view each file under the home directory.

   ```py
   ls
   ```

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image236.png" style="width:500px" />

The introduction to each folder is shown in the below table:

| **Directory/folder** | **Instruction** |
|:--:|:--:|
| hiwonder-toolbox | Wi-Fi management tool |
| Jetauto_software | Store the software |
| jetauto_ws | Workspace (including the function of each game) |
| Third_party | Store the functional package, such as the model trained with YOlOv5 |
| Music | Store music files |
| Pictures | Store music files |
| Public | User custom folder |
| Templates | template folder (custom) |
| Videos | Store video files |

2. Enter the command and press “Enter” to access the directory of the functional packages. Then enter “ls” to view each file under the directory.

   ```py
   cd ros_ws
   ```

   ```py
   ls
   ```

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image237.png" style="width:500px" />

The introduce of each folder:

| **Directory/folder** | **Instruction** |
|:--:|:--:|
| build | Compilation space, storing the cache generated during compiling |
| command | Store the commands for each function |
| devel | Store the target file and executable file after compilation |
| logs | Store the logs |
| src | Store the source code of the function packages |

3. Enter command and press Enter to access the directory for the function packages. Enter “ls” command to view each files under the directory.

   ```py
   cd src
   ```

   ```py
   ls
   ```

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image238.png" style="width:500px" />

The introduce of each folder:

| **Directory** | **Type** | **Function** |
|:---|:---|:---|
| hiwonder_app | App function packages storage directory | Gesture control, Lidar, intelligent line patrolling and other games |
| hiwonder_interfaces | Communication interface file directory | ROS message communication and service communication files |
| hiwonder_slam | Mapping-related function storage directory | Various algorithms for mapping and map saving |
| hiwonder_bringup | System service storage directory | Lunch functions such as app and wireless handle control |
| hiwonder_multi | Multi-robot combination function directory | Multi-robot mapping, multi-robot navigation, etc |
| third_party | Third-party environment function package directory | The ROS function package such as Apriltag, Lidar, depth camera. |
| hiwonder_calibration | Calibration parameter adjustment directory | IMU, linear velocity, angular velocity calibration. |
| hiwonder_navigation | Navigation-related function storage directory | Publishing navigation points, rviz navigation |
| xf_mic_asr_offline | Voice-related storage directory | Voice-controlled games |
| hiwonder_driver | Driving file directory | Kinematics, communicate between Jetson Nano board and STM32 |
| hiwonder_peripherals | Peripheral setting directory | Including different lidar models, handle control and keyboard control |
| hiwonder_example | Game example storage directory | Creative game: gesture control, posture control, color tracking. |
| hiwonder_simulations | Simulation storage directory | Gazebo, moveIt simulation, URDF file |

- **Game File Directory Introduction**

Take the game file in **/ros_ws/src/hiwonder_app** as example to explain.

1. Enter the following command to access the game file directory. There are two folders, including launch and scripts.

   ```py
   cd hiwonder_app
   ```

   ```py
   ls
   ```

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image239.png" style="width:500px" />

2)  The “**launch**” folder corresponds to the launch files, while the “**scripts**” folder corresponds to the source code of the games.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image240.png" style="width:500px" />

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image241.png" style="width:500px" />

The corresponding launch directory and scripts directory in other function packages are similar as well.

### 1.13.2 ROS2 Directory and Functional File Introduction

> [!NOTE]
>
> **Reminder: This section is only applicable to Jetson Nano in Docker, Jetson Orin Nano, Jetson Orin NX and Raspberry Pi 5 controller.**

- **File Directory Introduction**

1. Click-on <img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image242.png" style="width:50px" /> to initiate ROS2 command-line terminal. Then input the following command to check the content contained in the directory.

   ```py
   ls
   ```

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image243.png" style="width:500px" />

| **File Name** | **Function** |
|:--:|:--:|
| noetic_ws | Workspace for bridging with ROS1 |
| ros2_ws | Workspace for ROS2 functional play |
| share | Directory shared with the main system（/home/hiwonder/docker/tmp） |
| third_party_ros2 | Relevant third-party software libraries |

2. Next, enter the command to access the ROS2 workspace and check the file directory distribution.

   ```py
   cd ros2_ws
   ```

   ```py
   ls
   ```

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image244.png" style="width:500px" />

3)  The table below provides detailed introduction to each folder.

| **File name** | **Function** |
|:--:|:---|
| build | Compilation space, storing cache information during the compilation process |
| command | Stores instructions that implement various functions for easy lookup |
| install | Stores compiled target files and executable files |
| logs | Folder for storing logs |
| src | Folder for storing the source code of function packages |

4. Enter the command and press Enter to go to the robot function package directory and check the files in the src directory.

   ```py
   cd src
   ```

   ```py
   ls
   ```

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image245.png" style="width:500px" />

The table below provides an introduction to each folder.

| **Directory Name** | **Type Description** | **Function Description** |
|:--:|:--:|:--:|
| app | Directory for storing various game options for the mobile app | Gesture control, radar, line following, etc. |
| example | Directory for storing related vision game options | Color sorting, gesture control, waste classification, etc. |
| interfaces | Communication interface file directory | ROS message communication and service communication files |
| slam | Directory for storing mapping-related gameplay options | Various mapping algorithms, map saving |
| calibration | Calibration parameter adjustment directory | IMU, linear velocity, angular velocity calibration, etc. |
| navigation | Directory for storing navigation-related gameplay options | Publishing navigation points, RViz navigation, etc. |
| xf_mic_asr_offline | Directory for storing voice control game options | Voice control gameplay options |
| xf_mic_asr_offline_msgs | Directory for storing voice control game messages | Voice control gameplay directory |
| peripherals | External device settings directory | Includes different types of radar, joystick control, keyboard control, etc. |
| simulations | Directory for storing simulation files | Gazebo, MoveIt simulation, URDF, etc. |
| driver | Driver file directory | Kinematics, communication between Jetson controller and STM32 |

- **Introduction to Function File**

The following explains the game files **using /ros2_ws/src/app** as an example.

1. Enter the below command to navigate to the directory containing the game files. The launch and app folders are included in it.

   ```py
   cd app
   ```

   ```py
   ls
   ```

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image246.png" style="width:500px" />

2)  ‘launch’ folder contains the launch files, and the ‘app’ folder contains the game source codes.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image247.png" style="width:500px" />

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image248.png" style="width:500px" />

The corresponding launch directories and app directories (with the same name as the function packages) for other function packages are similar.

##  1.14 STM32 Source Code Instruction

### 1.14.1 Source Code Introduction

The STM32 controller is used as the robot underlying motion controller for running STM32 code. The robot chassis has already been burned with corresponding code for direct usage.

It supports ISP code updates through the USB serial port and can also be updated or debugged through the SWD interface. As the underlying chassis driver board for the robot, it is primarily responsible for the tasks such as motor PID control, encoder and IMU data acquisition, RGB light control, etc. Various control methods are supported, including PS2 wireless control, app control, and remote control via an RC transmitter. Additionally, it communicates through a serial port with the ROS base layer’s chassis control node, receiving target vector velocities from the basic layer and sending the real-time speed, IMU data, and battery voltage data calculated by the odometry. To enhance these functionalities, STM32 controller utilizes the FreeRTOS embedded operating system for software design.

### 1.14.2 Control Process

The principle of various robot control methods is based on varying robot’s velocity. Robot’s target velocity is obtained by operating inverse kinematics calculation, which is used as input for the motor velocity PID controller. After PID computation, the STM32 timer outputs PWM motor control signals to the motor driver. The motor driver, in turn, controls the motor's rotation. The encoder collects real-time motor speed, providing feedback to the PID controller, thus realizing closed-loop speed control. The flowchart for the robot's STM32 motor control process is illustrated below:

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image249.jpeg" style="width:700px"  />

> [!NOTE]
>
> **Note: Different robot types have variations. Mecanum wheel require four motors, while tank robots and Ackerman robots require two motors. Additionally, the front wheels of Ackerman robots require a servo for steering.**

### 1.14.3 Program Framework

The underlying source code is developed based on FreeRTOS. Unlike interrupt control, RTOS executes tasks in a round-robin fashion, with tasks of higher priority being executed first. Interrupts have a higher priority than task priorities. The robot task allocation is shown in the figure below:

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image250.jpeg" style="width:600px"  />

【The main task of the robot is responsible for robot control, kinematics processing, IMU data acquisition, data transmission matters. Additionally, I also need to handle the trivial tasks such as battery level management, IMU calibration, buzzer alarming, and other low-frequency management issues.】

### 1.14.4 Program Analysis

For more detailed STM32 code instructions, you can go through the source code, which contains very detailed comments. You can also refer to the "**[2. Basic Development Courses->2.2 STM32 Development Fundamentals]()**" for additional information.

### 1.14.5 Kinematics Model

The software supports various robot chassis, each mode featuring different motion characteristics.

Mecanum and tank chassis allow for 360-degree omnidirectional movement. They can not move forward and steer but also achieve movement in any direction within a 360-degree plane.

The Ackermann robot steers by controlling the front wheels with a servo.

For detailed explanations of the kinematic models of different robot chassis, you can refer to the relevant documentation tutorials.

- **ROS1：**

**[3.ROS1-Chassis Motion Control Lesson\3.1 Kinematics Analysis]()**

**[3.ROS1-Chassis Motion Control Lesson\3.2 Motion Control]()**

- **ROS2：**

**[16 ROS2-Motion Control Course\16.1 Kinematics Analysis]()**

**[16 ROS2-Motion Control Course\16.2 Motion Control]()**

### 1.14.6 Project Compilation

After writing the program, we need to compile it into machine language so that it can run on the embedded system. Keil5 includes a built-in compiler that can compile the source code into an executable file. 
The compiler can generate different target files based on different processor architectures and instruction sets. Additionally, the compiler can perform code optimization to make the generated executable files more efficient and stable. The steps for compiling the project are as follows:

1)  Open options for the generated hex file:

In the Keil5 interface, Click the “**Project**” in the menu bar, and then click the “**Options for File 'app.c**” button.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image251.png" style="width:500px"  />

Click the "**Output**" option, check the three red-marked options below, and then click "**OK**".

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image252.png" style="width:500px" />

2)  In the Keil5 software interface, click the "**Project**" option in the menu bar, and from the drop-down menu, select "**Build Target**" to compile the project, as shown in the image below:

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image253.png" style="width:500px" />

You can also click the icon on the toolbar in the interface to compile, as shown in the image below:

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image254.png" style="width:500px" />

If the following content appears in the "**Build Output**" window at the bottom of the software, it indicates that the compilation was successful.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image255.png" style="width:500px" />

> [!NOTE]
>
> Note: If the "**Build Output**" window shows "**Error(s)**" after compiling the project, you need to double-click on the specific line to jump to the corresponding location and make the necessary corrections, then compile again. If "**Warning(s)**" messages appear, you can ignore them.

### 1.14.7 Download Program Using USB Cable 

After compiling the project, you can download the generated hex file to the **STM32 main control board**. The following hardware and software materials are required:

- **Preparations**

**Software**: ATK-XISP (available in "**[2 Software/4 Firmware Download Software/ATK-XISP.exe]()**"):

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image256.png" style="width:100px" />

**Hardware**: Type-data cable, STM32 main control board  

The Type-C cable is used to connect the computer and the STM32 main control board.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image257.png" style="width:500px" />

- **Download Steps**

The specific operation steps are as follows, with the example of the program "**RosRobotControllerM4-ros**":

**（1）Hardware Connection：**

Insert the Type-C cable into the Type-C port on the STM32 control board (as shown in the red box in the image below) and connect it to the USB port on the computer:

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image258.png" style="width:500px" />

**（2）Initial Settings：**

Open the **ATK-XISP** software, select the correct **serial port** (Port) in the software, for example, COM22 (which is recognized as starting with USB), and then set the **baud rate (bps)** to **115200**:

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image259.png" style="width:500px" />

In the software interface, select "**Run After Programming,**" "**Verify,**" and "**Perform Full Chip Erase Before Programming,**" as shown in the image below:

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image260.png" style="width:500px" />

Then choose the following option:

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image261.png" style="width:500px" />

**（3）Software Flashing:**

In the **ATK-XISP** software interface, click the button within the red box in the image below, and then select the hex file that needs to be flashed.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image262.png" style="width:500px" />

Click the "**Start Programming**" button to flash the generated hex file onto the STM32 main control board:

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image263.png" style="width:500px" />

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image264.png" style="width:500px" />

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image265.png" style="width:500px" />

The following prompt indicates that the flashing process is complete.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image266.png" style="width:500px" />

> [!NOTE]
>
> Note: The STM32 main control board comes with pre-installed firmware. You can flash the RRC_20240702.hex file located in the "**[3 Source Code Materials\STM32]()**" folder.

##  1.15 Flash Image

### 1.15.1 Preparations

**Hardware：**

For **Jetson Nano and Raspberry Pi 5** versions, you need to prepare an SD card. (The storage size depends on the size if the image to be flashed. The example below uses a 64GB SD card), a card reader, and a computer (It’s recommended to use Windows 10 operating system).

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image267.png" style="width:200px" />

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image268.png" style="width:200px" />

For **Jetson Orin Nano and Jetson Orin NX** versions, you need to prepare a SSD (The storage size depends on the size if the image to be flashed), an SSD flasher (to be prepared separately), and a computer (Windows 10 operating system is recommended).

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image269.png" style="width:500px" />

Solid-state driver

**Software:** Install the SSD initialization tool (DiskGenius.exe) and the image flashing tool (Win32DiskImager).

> [!NOTE]
>
> **Note:**
>
> * Before flashing the image, you can use the SSD initialization tool (the compressed file can be found in “**[2 Software \3 Image Flashing Tools\1. SSD Initialization Tool]()**”) to delete any unnecessary partitions on the disk, then proceed with the flashing.
>
> * After the image flashing is complete, multiple independent disk prompts may appear. Do not click "**Format**"; simply cancel the prompts.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image270.png" style="width:500px"  />

### 1.15.2 SD Card/SSD(Solid-state Drive) Formatting

> [!NOTE]
>
> **Note：If the SD card or SSD is empty, no formatting is required.**

1)  Remove the SD card from Jetson Nano or Raspberry Pi 5, and remove the solid-state drive from Jetson Orin Nano or Jetson Orin NX.

    **Jetson Nano**

    <img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image271.png" style="width:500px" />

    **Raspberry Pi5**

    <img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image272.png" style="width:500px" />

    **Jetson Orin Nano/Jetson Orin NX**

    <img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image273.png" style="width:500px" />

2)  Locate the compressed file in "**[2 Software\3 Image Flashing Tools\1. SSD Initialization Tool]().**" After extracting it, use the **DiskGenius.exe** tool to format the SD card or solid-state drive. Be cautious to select the correct drive letter to avoid formatting your computer's drives by mistake.

3)  Once the SD card or solid-state drive is inserted into the computer, you will notice additional drive letters apart from those of your computer.

    <img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image274.png" style="width:500px" />

4)  Right-click and select "**Delete All Partitions.**"

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image275.png" style="width:500px" />

5)  Create a new partition so that the computer can recognize it properly. If a prompt appears, click "**OK**" to proceed, as shown in the image below:

    <img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image276.png" style="width:500px" />

6)  Then click “**Save**” to save the modification.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image277.png" style="width:500px" />

7)  Once successful, you will see the information displayed as shown below. This indicates that the SD card/solid-state drive has been successfully formatted.

    <img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image278.png" style="width:500px" />

### 1.15.3 Flash Image

1)  Open the image flashing tool (**Win32DiskImager**), click <img src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image279.png" style="width:50px" /> to select the image file (the file must be downloaded and extracted by the user; the image shown is for reference only, and the actual image should be used). Set the “**Device”** field to the drive letter of the SD card or solid-state drive, then click the “**Write”** button to begin flashing the image.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image280.png" style="width:500px" />

> [!NOTE]
>
> **Note: the storage path of the image file must not contain any Chinese characters.**

2)  If the prompt appears, simply click the "**Yes**" button to proceed.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image281.png" style="width:500px" />

3)  If the prompt "**Write Successful**" appears, the flashing process has been completed successfully. If an error occurs, please disable any firewall or similar software, reinsert the solid-state drive, and repeat the steps in this section.

<img class="common_img" src="../_static/media/1/section_153_JetRover User Manual (2025)/media/image282.png" style="width:300px" />

> [!NOTE]
>
> **Note: After successful flashing, if a prompt asking whether to format the partition appears, you can ignore it.**

4)  Wait for the image flashing to complete. Insert the SD card or solid-state drive back into the control board. Once powered on for a short period, the system should boot successfully.
