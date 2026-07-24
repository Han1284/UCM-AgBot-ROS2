# RoMu4o / AgBot ROS 2 工作区

本仓库用于复现并继续开发 [mehradmrt/UCM-AgBot-ROS2](https://github.com/mehradmrt/UCM-AgBot-ROS2)。当前开发主线是 `leaf-manipulator` 相关工作，原始 UCM-AgBot 工程及 TM、RealSense、VectorNav、SLLidar、OnRobot 等组件作为源码依赖一起参与构建。

当前环境以 Ubuntu 22.04、ROS 2 Humble、MoveIt 2 和 Gazebo Fortress 6 为准。原始工程基于 Foxy，leaf 操作主线已经迁移到现代 Gazebo；本文只给出已经核对过的 Humble + Fortress 主入口。

## 仓库结构

```text
ros2_ws/
├── src/
│   ├── UCM-AgBot-ROS2/          # AgBot 主源码及 leaf-manipulator 改动
│   │   ├── agbot/               # 底盘、整机描述、仿真与叶片操作
│   │   ├── cobot/tmr_ros2/      # TM5-900 驱动及 MoveIt 配置
│   │   ├── devices/             # OnRobot 等设备驱动
│   │   ├── sensors/             # 相机、IMU、GNSS、雷达等驱动
│   │   └── vision/              # 叶片感知
│   └── moveit_visual_tools/     # MoveIt 可视化依赖
├── build/                       # colcon 生成目录
├── install/                     # colcon 安装目录
└── log/                         # colcon 日志
```

## 环境与依赖

推荐环境如下：

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2 for Humble
- Nav2 for Humble
- Gazebo Fortress 6

首次使用时安装当前 leaf 仿真所需的 Humble 控制器：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-gz \
  ros-humble-gz-ros2-control \
  ros-humble-joint-state-broadcaster \
  ros-humble-position-controllers \
  ros-humble-moveit-task-constructor-core \
  ros-humble-moveit-task-constructor-visualization
```

然后在工作区根目录安装能够由 rosdep 解析的其余依赖：

```bash
cd /home/han1284/projects/ros2_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y \
  --skip-keys robot_server_msgs
```

`robot_server_msgs` 只被 TM 调试 GUI 的旧清单引用，目前没有 Humble rosdep 规则，不影响 leaf 仿真和机械臂控制链。`rosdep` 同时服务于 ROS 1 和 ROS 2；它不是 ROS 1 专用命令，实际解析哪个发行版由当前环境和 `--rosdistro humble` 决定。

## 获取源码和构建

克隆主工作区时应递归取得所有子模块：

```bash
git clone --recurse-submodules https://github.com/Han1284/ros2_ws.git
cd ros2_ws
git submodule update --init --recursive
```

构建并加载环境：

```bash
source /opt/ros/humble/setup.bash
colcon build --continue-on-error
source install/setup.bash
```

本工作区目前使用普通 colcon 安装方式。不要在未清理对应包生成目录的情况下突然改用 `--symlink-install`，否则 ament 的 Python 目录和符号链接可能发生冲突。

每次新开终端都需要执行：

```bash
cd /home/han1284/projects/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Leaf 机械臂 Gazebo 仿真

### 完整 Gazebo Fortress 控制仿真

这是当前唯一的 leaf 仿真主入口，会启动 Gazebo Fortress、TM5-900、RG2 夹爪、植物、MoveIt、RViz 以及 `gz_ros2_control` 控制器，但不会自动执行画圆：

```bash
ros2 launch leaf_manipulation_sim simulation.launch.py gui:=true rviz:=true
```

只启动服务端、不打开 Gazebo 窗口：

```bash
ros2 launch leaf_manipulation_sim simulation.launch.py gui:=false rviz:=false
```

启动后可核对控制器状态：

```bash
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers '{}'
```

正常情况下，`joint_state_broadcaster`、`arm_position_controller` 和 `gripper_position_controller` 都应处于 `active` 状态。也可以直接发送一组位置命令进行低风险联调：

```bash
ros2 topic pub --once /arm_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 1.2, 0.0, 1.2, 0.0]}"

ros2 topic pub --once /gripper_position_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.20, -0.20, 0.20, -0.20, -0.20, 0.20]}"
```

### 圆轨迹 Gazebo 与 MoveIt 联动演示

leaf 仿真后端已从 Gazebo Classic 11 整体迁移到 Gazebo Fortress 6。机械臂由
`ros_gz_sim` 生成，控制插件为 `gz_ros2_control`；RGB-D、点云和接触传感器经
`ros_gz_bridge` 接入 ROS。四个叶片接触输出仍保持原来的
`gazebo_msgs/msg/ContactsState` 类型和 `/plant/leaf_N_contacts` 名称。

圆轨迹演示固定使用两个终端。终端一只启动 Gazebo、机械臂控制器、MoveIt、
RViz、盆栽 Marker 和叶片接触传感器：

```bash
ros2 launch leaf_manipulation_sim simulation.launch.py \
  gui:=true rviz:=true
```

等待三个控制器均为 `active` 后，在终端二单独启动圆周节点：

```bash
ros2 launch leaf_manipulation_sim run_circle_demo.launch.py \
  radius:=0.10 repetitions:=3 samples_per_circle:=60 \
  velocity_scale:=0.2 acceleration_scale:=0.2 finger_position:=0.10
```

`simulation.launch.py` 不会自动执行圆周运动；`run_circle_demo.launch.py` 也不会
启动第二份 Gazebo、MoveIt 或 RViz。不要同时运行 `fixed_arm_gazebo.launch.py`
或单独启动内部的 `gazebo.launch.py`，否则会争用同一个 Fortress world。

`circle_motion_demo` 会等待 `/arm_position_controller/commands` 和
`/gripper_position_controller/commands` 出现订阅者，再发送六轴位置及夹爪位置。
`/joint_states` 只由 Gazebo 的 `joint_state_broadcaster` 发布，因此不会与演示
节点争夺 TF。演示结束后节点会持续发送最后一帧控制命令；查看完毕后按
`Ctrl+C` 结束。

圆轨迹演示使用简化碰撞几何。由于立柱与固定安装座的碰撞体存在重叠，画圆段关闭了碰撞拒绝，但仍执行 IK 与关节限位检查；该设置只适用于 Gazebo 演示，不应直接用于实机路径规划。

### 指定位姿抓取演示

先在终端一启动 Gazebo、MoveIt、控制器和 RViz：

```bash
ros2 launch leaf_manipulation_sim simulation.launch.py \
  gui:=true rviz:=true
```

终端二使用明确的目标位姿运行抓取轨迹演示：

```bash
ros2 launch leaf_manipulation_sim run_pose_grasp_demo.launch.py \
  target_x:=0.52 target_y:=-0.22 target_z:=0.25 \
  target_roll:=3.14159 target_pitch:=0.0 target_yaw:=1.5708 \
  velocity_scale:=0.2 acceleration_scale:=0.2
```

交互式输入目标位姿可运行：

```bash
ros2 run leaf_manipulation_sim prompt_pose_grasp_demo
```

### 已标定叶片轻夹演示

这个入口使用第 1 片叶子的外侧第 5 段碰撞盒作为目标。夹爪闭合轴与叶面法向
对齐，运动过程中保持 `link_6`、夹爪基座和 D435 不接触叶片。它只执行动作，
不会启动第二份 Gazebo 或 RViz。

先在终端一启动仿真：

```bash
ros2 launch leaf_manipulation_sim simulation.launch.py \
  gui:=true rviz:=true
```

等待三个控制器均为 `active` 后，在终端二单独执行：

```bash
ros2 launch leaf_manipulation_sim run_leaf_pinch_demo.launch.py
```

演示会依次张开夹爪、移动到已标定叶尖位姿、闭合一次、重新张开并返回
竖直 `home`，完成后自动退出。可通过 `grasp_hold_sec:=3.0` 调整闭合保持时间。

### MoveIt Task Constructor 候选解与评分

RViz 配置中已经加入 MoveIt 2 官方 MTC 的 `Motion Planning Tasks` 面板。
仿真启动时会读取 Gazebo 使用的 `simple_potted_plant/model.sdf`，将花盆和
20 个叶片薄盒按 `leaf_bench.world` 中的植物位姿预先加入 MoveIt
PlanningScene，因此这些场景初始化操作不会占用 MTC 主任务树。

完整任务依次规划：张开夹爪、无碰撞接近叶片、闭合夹爪、重新张开夹爪、
返回竖直 `home`。只有目标位姿需要 IK；夹爪开合和返回 `home` 是明确的
关节空间目标，也会各自产生可预览轨迹。叶片保持为正常碰撞对象，不再
通过允许碰撞来让夹爪穿过薄盒。夹持目标位姿位于叶片厚度中心，闭合角
保留两侧各约 1 mm 间隙，用于模拟接触但不继续挤压叶片。这个入口只规划
和发布预览，不会自动驱动 Gazebo 中的机械臂。

仍然在终端一单独启动仿真：

```bash
ros2 launch leaf_manipulation_sim simulation.launch.py \
  gui:=true rviz:=true
```

等待机械臂状态稳定后，在终端二生成候选解：

```bash
ros2 launch leaf_manipulation_sim run_leaf_mtc_demo.launch.py
```

在 RViz 的 `Motion Planning Tasks` 面板中展开任务阶段，右侧列表按 cost 排序。
点击某个 cost 行即可预览对应轨迹。当前 Gazebo 使用位置组控制器，而且
`move_group` 禁止轨迹执行，因此该面板只用于比较和预览；确认候选位姿后，
使用 `run_leaf_pinch_demo.launch.py` 执行已经验证过的轻夹动作。

## 整机、传感器与导航入口

启动传感器，可按需关闭不使用的设备：

```bash
ros2 launch robot_bringup sensors.launch.py \
  imu:=true gnss:=true lidar2d:=true realsense:=true encoders:=true
```

启动整机底层、定位、导航或 SLAM：

```bash
ros2 launch robot_bringup bringup.launch.py
ros2 launch robot_bringup localization.launch.py
ros2 launch robot_bringup navigation.launch.py
ros2 launch robot_bringup slam.launch.py
```

只启动机械臂侧传感器：

```bash
ros2 launch robot_bringup sensors_arm.launch.py
```

运行叶片实例分割：

```bash
ros2 run leaf_extraction instance_segmentation
```

## TM5-900 实机和驱动 demo

以下命令面向真实 TM 机械臂，不应与纯 Gazebo 演示混为一谈。先在终端一启动驱动，将 IP 地址替换为 TMflow 中配置的机器人地址：

```bash
ros2 run tm_driver tm_driver robot_ip:=192.168.1.19
```

使用 TM5-900 MoveIt C++ 演示：

```bash
ros2 launch tm_moveit_cpp_demo tm5-900_run_moveit_cpp.launch.py \
  robot_ip:=192.168.1.19
```

TM 驱动自带的 C++ demo 均使用相同形式运行：

```bash
ros2 run demo demo_send_script
ros2 run demo demo_ask_item
ros2 run demo demo_ask_sta
ros2 run demo demo_connect_tm
ros2 run demo demo_set_event
ros2 run demo demo_set_io
ros2 run demo demo_set_positions
ros2 run demo demo_write_item
ros2 run demo demo_leave_listen_node
ros2 run demo demo_get_feedback
ros2 run demo demo_get_torque_feedback
ros2 run demo demo_get_sct_response
ros2 run demo demo_get_sta_response
ros2 run demo demo_get_svr_response
```

其中 `demo_set_positions`、`demo_send_script`、`demo_set_event`、`demo_set_io` 和 `demo_leave_listen_node` 可能改变实机状态或使机械臂运动。运行前应确认急停、速度限制、工作空间和 TMflow Listen 节点状态；`demo_leave_listen_node` 执行后，需要在 TMflow 中重新恢复 Listen 任务。

读取 TMvision 图像：

```bash
ros2 run custom_package sub_img
```

启动 TM 调试 GUI：

```bash
ros2 launch ui_for_debug_and_demo tm_gui.launch.py robot_ip:=192.168.1.19
```

该 GUI 仍声明了未纳入当前源码树的 `robot_server_msgs`，如果运行时报缺包，应先补入对应消息包；这不影响 Gazebo leaf 主线。

## 常见问题

### Gazebo 中能生成实体但看不到机械臂

TM5 和 RG2 的网格使用 `package://tm_description/...` 与 `package://onrobot_rg_description/...`。Gazebo Fortress 需要通过 `GZ_SIM_RESOURCE_PATH` 或兼容变量 `IGN_GAZEBO_RESOURCE_PATH` 找到这些资源。内部的 `gazebo.launch.py` 已自动加入相关安装包的 `share` 父目录；修改后必须重新构建并重新加载工作区：

```bash
colcon build --packages-select leaf_manipulation_sim
source install/setup.bash
ros2 launch leaf_manipulation_sim simulation.launch.py gui:=true rviz:=true
```

如果仍不可见，直接检查当前启动日志中的资源错误：

```bash
rg -n "No mesh specified|Unable to find uri|Unable to find file" \
  ~/.ros/log/latest/* 2>/dev/null
```

### 没有 `/camera` 图像话题

Fortress 使用原生 RGB-D 传感器，并由 `ros_gz_bridge` 转换成 ROS 图像、深度图和点云。若话题缺失，先确认迁移依赖存在，再重新启动整个仿真：

```bash
sudo apt install -y ros-humble-ros-gz ros-humble-gz-ros2-control
```

启动仿真后可检查图像、深度图和点云：

```bash
ros2 topic list -t | rg /camera
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /camera/depth/color/points
```

`leaf_manipulation.rviz` 保留主 Displays 面板，不再启动名为 Camera 的额外面板；植物继续由 `/plant_marker` 显示。需要检查相机时，可临时添加 Image 显示并选择上述话题。

### 子模块提示 `not our ref`

这通常说明父仓库固定到了远端已经无法取得的提交。不要反复执行同一条 `git submodule update`；应先核对 `.gitmodules` URL 和父仓库记录的 gitlink，再切换到远端实际存在、与 Humble 兼容的提交。

### 构建模式冲突

普通构建和 `--symlink-install` 会让同一个 Python 包生成不同类型的路径。出现 `existing path cannot be removed: Is a directory` 时，只清理出错包对应的 `build/<包名>` 和 `install/<包名>` 后重建，不要无差别删除整个工作区。

### 画圆结束后机械臂或夹爪 TF 消失、跳变

Gazebo 联动模式下，圆轨迹节点只发布控制器命令，动态 TF 来自 `robot_state_publisher`，`/joint_states` 来自 `joint_state_broadcaster`。如果 `/joint_states` 有多个发布者，通常是误启动了第二套仿真，应先关闭重复节点：

```bash
ros2 topic info /joint_states --verbose
```

正常情况下 `/joint_states` 应只有 `joint_state_broadcaster` 一个发布者。还可以直接检查末端 TF：

```bash
ros2 run tf2_ros tf2_echo base gripper
```

## 参考文献

Mortazavi, M., Cappelleri, D. J., Ehsani, R. (2025). *RoMu4o: A Robotic Manipulation Unit for Orchard Operations Automating Proximal Hyperspectral Leaf Sensing*. arXiv:2409.19786.

TM5-900 的关节命名、MoveIt 配置、Gazebo 联动方式和驱动接口以 Techman 官方 [tmr_ros2 Humble 分支](https://github.com/TechmanRobotInc/tmr_ros2/tree/humble)为参考，不使用 `tmr_ros1` 作为实现基线或 Humble 构建依赖。
