# <center>SHU_USV<center>
## 一. 快速使用
### 1.1 使用仿真
+ 运行VRX仿真平台
  ```shell
  roslaunch vrx_2019 sandisland.launch
  ```
+ 运行USV仿真程序
  ```shell
  rosrun shu_sim shu_sim_node
  ```
+ 启动rviz可视化
  ```shell
  rviz
  ```
  `Path`-----订阅话题为：`/usv_sim_pose`
  `Marker`--订阅话题为：`visualization_marker`
### 1.2 使用网络摄像头
+ 确保硬件连通
  ```shell
  ping 192.168.1.10
  ```
+ 运行网络摄像头视频流获取程序
  ```
  roslaunch rocon_rtsp_camera_relay rtsp_camera_relay.launch
  ```
+ 查看网络摄像头图像
  ```shell
  rqt_image_view
  ```
  订阅话题为：`/rtsp_camera_relay/image`
### 1.3 使用Aruco码识别
+ 需要确保网络摄像头能正常获取图像
+ 运行Aruco码识别程序
  ```shell
  roslaunch aruco_ros single.launch
  ```
+ 识别结果显示
  ```shell
  rqt_image_view
  ```
  订阅话题为：`/aruco_single/result`
### 1.4 使用USV控制

## 二. 硬件选型
<center></center>

|           名称           |          型号           |
| :----------------------: | :---------------------: |
|           电池           |   格氏 5300mAh 4S电池    |
|        上位机处理器       |       Intel NUC         |
|        下位机控制器       |      Pixhawk 2.4.8      |
|          IMU           |       Pixhawk内置IMU      |
|          GPS           |         乐迪SE100         |
|         摄像头           |    MiniHomer网络摄像头    |
|      部分机械结构件       |         PLA打印          |
|        无人艇艇架         |       天富龙小公主        |
|          遥控器          |      乐迪AT9S遥控器       |
|         数传图传          |    MiniHomer数传图传     |

## 三. 参数调试
### 3.1 仿真参数调试
+ 海洋环境参数
  文件：vrx/wave_gazebo/world_models/ocean_waves/model.xacro
  ```xacro
  <xacro:macro name="ocean_waves" params="
          gain:=0.3 
          period:=2
          direction_x:=0.7 direction_y:=0.7
          angle:=0.6 
          scale:=2.0">
  ```
  `gain`：控制波浪的振幅。值越大，波浪越高，水面起伏越剧烈。
  `period`：波浪的周期。值越小，波浪越密集，频率越高。
  `direction`：波浪传播的方向。通常以二维向量形式表示，方向向量决定了波浪的传播方向。
  `angle`：波浪的随机性。值越大，波浪越不规则，模拟效果更自然。
  `scale`：波浪网格的缩放比例。值越大，波浪覆盖范围越广，视觉上更宽广。
+ 传统LOS制导参数
  ```yaml
  %YAML:1.0
  ---
  usv_length: 3.78
  lammda: 3.0
  delta: 0.0
  max_heading_rate: 0.0
  ```
+ PID角度环参数
  ```yaml
  %YAML:1.0
  ---
  Pid.Name: usv_sim_angle_pid
  Kp: -0.3
  Ki: -0.05
  Kd: -0.02
  CalculateTime: 50
  Integal.Limit: 0.4
  Use.Integal.Limit: true
  Output.Limit: 0.4
  Use.Output.Limit: true
  ```