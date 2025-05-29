## 简介

该项目目的是发布nav2所需的必要tf变换。

节点会获取算法的位姿数据，然后根据设备和机器人的外参关系，发布map->odom -> base_link的tf变换。

map目录下的地图为空地图，可用于无先验地图时的nav2使用。

## 使用说明

### 1、编译该项目

```bash
colcon build
```

## 2、运行节点

```
source install/setup.bash
ros2 run TF_Pub TF_node
```

### 3、运行nav2启动文件

```
ros2 launch nav2_bringup bringup_launch.py  map:=/home/ubuntu22/tf_ws/src/map.yaml
```

上面的地图路径需要根据具体路径进行修改

### 4、开始导航

```
rviz2
```

在地图上给出Nav2 Goal

此时就会看到规划出一条轨迹

### 5、运行车辆控制节点

此时运行你们的车辆控制节点即可

## 注意事项

代码里面需要设备跟机器人之间的外参，需要根据实际情况做出修改。

经测试，直接使用机械图纸的参数也能满足正常使用