# Launch 启动清单

所有命令默认先执行：

```bash
cd /home/han1284/projects/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

1. 末端画圆 Demo

```bash
ros2 launch leaf_manipulation_sim run_circle_demo.launch.py
```

2. 指定位姿抓取 Demo

```bash
ros2 launch leaf_manipulation_sim run_pose_grasp_demo.launch.py
```

3. 叶片轻夹 Demo

```bash
ros2 launch leaf_manipulation_sim run_leaf_pinch_demo.launch.py
```

4. 叶片 MTC 候选解 Demo

```bash
ros2 launch leaf_manipulation_sim run_leaf_mtc_demo.launch.py
```

5. 多视角叶片观察 Demo

```bash
ros2 launch leaf_manipulation_sim run_multi_view_observation.launch.py
```

6. TM5 叶片仿真环境

```bash
ros2 launch leaf_manipulation_sim simulation.launch.py gui:=true rviz:=true
```

7. Pro450 基础仿真环境

```bash
ros2 launch pro450_sim pro450_sim.launch.py control:=true
```

8. Pro450 单独叶片管线

```bash
ros2 launch pro450_sim pro450_leafpipeline_sim.launch.py execute:=true
```

9. Pro450 叶片 MTC

```bash
ros2 launch pro450_sim pro450_leaf_mtc.launch.py execute:=true
```

10. Pro450 完整叶片管线

```bash
ros2 launch pro450_sim pro450_leaf_pipeline.launch.py execute:=true
```

11. Pro450 多视角观察

```bash
ros2 launch pro450_sim pro450_multi_view_observation.launch.py maximum_views:=6
```

12. Pro450 移动底盘环境

```bash
ros2 launch pro450_sim pro450_myagv_leaf_environment.launch.py
```

13. Pro450 移动底盘控制

```bash
ros2 launch pro450_sim pro450_myagv_control.launch.py
```

14. Pro450 移动底盘 MoveIt

```bash
ros2 launch pro450_sim pro450_myagv_moveit.launch.py
```

15. Pro450 移动底盘巡检环境

```bash
ros2 launch pro450_sim pro450_myagv_leaf_inspection.launch.py
```

16. Pro450 移动底盘巡检任务

```bash
ros2 launch pro450_sim pro450_myagv_leaf_mission.launch.py plant_id:=1
```

17. Pro450 移动底盘抓叶 MTC

```bash
ros2 launch pro450_sim pro450_myagv_leaf_mtc.launch.py execute:=true
```

18. Pro450 移动底盘叶片管线

```bash
ros2 launch pro450_sim pro450_myagv_leaf_pipeline.launch.py execute:=true
```

19. Pro450 移动底盘多视角观察

```bash
ros2 launch pro450_sim pro450_myagv_multi_view_observation.launch.py maximum_views:=6
```

20. Pro450 回字环境巡检

```bash
ros2 launch pro450_sim pro450_myagv_atrium_patrol.launch.py
```

21. Pro450 回字环境 SLAM

```bash
ros2 launch pro450_sim pro450_myagv_atrium_slam.launch.py
```

22. Pro450 普通 SLAM

```bash
ros2 launch pro450_sim pro450_myagv_slam.launch.py
```

23. Pro450 移动底盘显示

```bash
ros2 launch pro450_sim pro450_mobile_display.launch.py
```

24. Pro450 MoveIt 配置

```bash
ros2 launch pro450_moveit_config pro450_moveit.launch.py
```

25. 回字 Gazebo 环境

```bash
ros2 launch pro450_sim atrium_env.launch.py
```

26. 果园 Gazebo 环境

```bash
ros2 launch pro450_sim orchard_env.launch.py
```

27. 机械臂 Gazebo 基础环境

```bash
ros2 launch leaf_manipulation_sim gazebo.launch.py
```

28. 固定机械臂 Gazebo 环境

```bash
ros2 launch leaf_manipulation_sim fixed_arm_gazebo.launch.py
```

29. 机械臂 MoveIt 环境

```bash
ros2 launch leaf_manipulation_sim moveit.launch.py
```

30. 感知相机测试

```bash
ros2 launch leaf_manipulation_sim perception_test_camera.launch.py
```
