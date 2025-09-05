# Xmartev Block Challenge Baseline

这是Xmartev Block Challenge Baseline的部署和使用文档，包含环境配置、启动流程和常见问题解答。

## 环境部署

### 1. 拉取镜像

```bash
# 从镜像仓库拉取（待补充镜像地址）
docker pull [镜像地址]

# 或从本地tar包加载镜像
docker load -i [镜像tar包路径]

# 查看是否成功获取 xmartev/block_challenge_baseline 镜像
docker images
```

### 2. 创建Docker容器

下载`create_container_client.sh`脚本到本地，然后执行：

```bash
bash create_container_client.sh

# 查看容器是否创建成功
docker ps
```

### 3. 进入Docker容器

```bash
docker exec -it block_challenge_baseline bash
```

### 4. 验证ROS2通信

1. 在第一个终端，进入容器并运行发布节点：
   ```bash
   docker exec -it block_challenge_baseline bash
   ros2 run demo_nodes_cpp talker
   ```
   成功运行会看到类似以下输出：
   ```
   [INFO] [1620000000.000000000] [talker]: Publishing: 'Hello World: 1'
   ```

2. 在第二个终端，在宿主机上设置环境变量并运行接收节点：
   ```bash
   # 安装对应版本的中间件
   sudo apt update
   sudo apt install ros-<distro>-rmw-cyclonedds-cpp
   
   # 设置环境变量（可写入.bashrc或.zshrc中持久化）
   export ROS_DOMAIN_ID=99
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   
   # 运行接收节点
   ros2 run demo_nodes_cpp listener
   ```
   通信正常会看到类似以下输出：
   ```
   [INFO] [1620000000.000000000] [listener]: I heard: 'Hello World: 1'
   ```

3. 如通信异常，检查两边的ROS_DOMAIN_ID是否一致：
   ```bash
   echo $ROS_DOMAIN_ID
   ```

## Material Detection Baseline 使用

### 1. 进入工作目录

```bash
docker exec -it block_challenge_baseline bash
cd /workspace/block_challenge_baseline
```

### 2. 运行baseline

> 注意：请提前开启server端服务，并确保ros2话题可以正常传入此baseline容器内

```bash
python3 baseline_round1_seed99.py
```

## ROS2 Topic 含义

```yaml
Subscribed topics:
 * /clock [rosgraph_msgs/msg/Clock] 1 subscriber
	# 仿真时钟
 * /airbot_play/arm_camera/aligned_depth_to_color/image_raw [sensor_msgs/msg/Image] 1 subscriber
 	# airbotplay机械臂环境深度相机 内参
 * /airbot_play/arm_camera/color/image_raw [sensor_msgs/msg/Image] 1 subscriber
 	# airbotplay机械臂环境rgb相机的rgb图像，编码格式rgb8
 * /airbot_play/side_camera/aligned_depth_to_color/image_raw [sensor_msgs/msg/Image] 1 subscriber
 	# airbotplay机械臂相机 内参
 * /airbot_play/side_camera/color/image_raw [sensor_msgs/msg/Image] 1 subscriber
 	# airbotplay机械臂相机的rgb图像，编码格式rgb8
 * /airbot_play/joint_states [sensor_msgs/msg/JointState] 1 subscriber
 	# airbotplay机械臂全身关节状态量，顺序为 joint_names: [
    # - arm_joint1
    # - arm_joint2
    # - arm_joint3
    # - arm_joint4
    # - arm_joint5
    # - arm_joint6
    # - arm_eef_gripper_joint
    # -]

Published topics:
 * /arm_forward_position_controller/commands [std_msgs/msg/Float64MultiArray] 1 publisher
 	# 控制mmk2手臂移动
 * /yolo/detections [yolov8_msgs/msg/Detections] 1 publisher
 	# yolo检测结果
 * /yolo/result_image [sensor_msgs/msg/Image] 1 publisher
 	# yolo检测结果可视化图像
```

## 常见问题(FAQ)

1. 运行baseline时出现特定错误提示，提示需要启动server端
   - 解决方案：请确保已正确启动对应的server端服务，再重新运行baseline程序

2. ROS2通信失败
   - 检查宿主机和容器内的`ROS_DOMAIN_ID`是否一致
   - 确认已正确安装并配置`rmw_cyclonedds_cpp`中间件
   - 检查网络连接和防火墙设置