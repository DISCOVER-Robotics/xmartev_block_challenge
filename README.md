# XMARTEV 机器人控制系统教程

环境依赖：
```
ubuntu >= 20.04
cuda >= 11.8
显存 >= 6GB
空余硬盘空间 >= 80G
```
查看cuda版本
```
nvcc --version
```
## server部署指南

### 1. 安装git

打开终端输入
```
sudo apt update
sudo apt install git -y
```
终端输入下面命令git clone到本地
```
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/DISCOVER-Robotics/xmartev_block_challenge.git
cd xmartev_block_challenge
```
### 2. 安装docker

若本地尚未安装docker：

```bash
cd ../scripts
bash docker_install.sh
```

验证docker安装：

```bash
docker --version
```

安装参考链接，[docker install](https://docs.docker.com/engine/install/ubuntu/)。

### 3. 安装nvidia driver

推荐使用Software & Updates中Additional Drivers安装，创建镜像和容器前需要检查宿主机的显卡驱动是否正常。

打开终端，输入nvidia-smi检查驱动是否安装成功。
出现显卡信息和驱动版本信息即为安装成功。
![image-2025022019304150341](doc/readme_assets/nvidiasmi.png)

### 4.安装 nvidia-docker2

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update

export NVIDIA_CONTAINER_TOOLKIT_VERSION=1.17.8-1
sudo apt-get install -y \
      nvidia-container-toolkit=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      nvidia-container-toolkit-base=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      libnvidia-container-tools=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      libnvidia-container1=${NVIDIA_CONTAINER_TOOLKIT_VERSION}

```

### 5. 从docker hub拉取镜像

docker hub是一个类似于github的平台，只不过不用来存放代码，而是存放镜像

在国内拉取镜像可能会遇到网络问题，所以我们提供了国内镜像仓库（阿里云ACR服务）


```bash
# 从docker hub拉取
docker pull xmartev/block_challenge_server:release_v0

# 如果因为网络问题拉取失败，提供了国内的镜像仓库
docker pull crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/xmartev/block_challenge_server:release_v0

# 查看是否成功获取 xmartev/block_challenge_server 镜像，如果有输出则说明成功拉取到本地
docker images | grep block_challenge_server
```

**注意：如果您使用的是50系显卡（如RTX 5090等），请使用专门优化的镜像：**
```bash
# 50系显卡专用镜像（国内仓库）
docker pull crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/xmartev/block_challenge_server:release_cuda128_v0
```

### 6. Run server container

!!!
最新tag名称：release_v0.0.1或release_cuda128_v0.0.1

打开[`scripts/create_container_server.sh`](scripts/create_container_server.sh)并修改镜像 和 tag名称（tag名称以最新的版本为准,如按照上面的版本名tag改为release_v0，50系显卡用户请使用release_cuda128_v0），如果使用国内镜像源拉取，则需要将第15行的`xmartev/`修改成`crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/xmartev/`

![alt text](doc/readme_assets/create_container_server.png)

创建server container：

```bash
cd xmartev_block_challenge/scripts
bash create_container_server.sh
```

终端中进入server container,同目录（xmartev_block_challenge/scripts）下运行

```bash
bash exec_server.sh
```

电脑重启后，需要重新启动容器

```bash
docker start block_challenge_server
```

## Client部署指南
### 1. 拉取client镜像

```bash
# 从镜像仓库拉取
docker pull xmartev/block_challenge_client:release_v0

# 如果因为网络问题拉取失败，提供了国内的镜像仓库
docker pull crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/xmartev/block_challenge_client:release_v0

# 查看是否成功获取 xmartev/block_challenge_client 镜像，如果有输出则说明成功拉取到本地
docker images | grep block_challenge_client
```

**注意：如果您使用的是50系显卡（如RTX 5090等），请使用专门优化的镜像：**
```bash
# 50系显卡专用镜像（国内仓库）
docker pull crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/xmartev/block_challenge_client:release_cuda128_v0
```

### 2. 创建Docker容器

打开[`scripts/create_container_client.sh`](scripts/create_container_client.sh)并修改镜像 和 tag名称，tag`example_tag`需要修改为实际的最新tag,名称以最新的版本为准,如按照上面的版本名tag改为release_v0，如果是从国内镜像源拉取，第15行的`xmartev/`需要修改为国内镜像源名称，例如`crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/xmartev/`

![image-20250907022216927](doc/readme_assets/20250907-074208.jpg)

然后执行

```bash
cd xmartev_block_challenge/scripts
bash create_container_client.sh
```

### 3. 进入Docker容器
同目录下(xmartev_block_challenge/scripts)运行
```
bash exec_client.sh
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

## Block Challenge Baseline 使用

### 1. 进入工作目录

```bash
docker exec -it block_challenge_client bash
cd /workspace/block_challenge_client
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

关于baseline进一步的说明，请参考[baseline说明](doc/baseline.md)。

## 常见问题(FAQ)

1. 运行baseline时出现特定错误提示，提示需要启动server端
   - 解决方案：请确保已正确启动对应的server端服务，再重新运行baseline程序

2. ROS2通信失败
   - 检查宿主机和容器内的`ROS_DOMAIN_ID`是否一致
   - 确认已正确安装并配置`rmw_cyclonedds_cpp`中间件
   - 检查网络连接和防火墙设置

## 完成开发后上传client镜像

选手在client中开发算法，将可执行脚本命名为`client_block.py`. 裁判会执行该脚本来评分。

开发完成后打包上传至阿里云ACR服务，由官方拉取后进行测试，测试使用电脑配置为：

```
cpu : 13th Gen Intel Core i7 13700KF x24
gpu : GeForce RTX 4090 24G
Memory : 64GB
```

### 1. 创建阿里云ACR仓库

参赛队伍在[阿里云容器镜像服务](https://cr.console.aliyun.com/)中创建私有仓库（个人版即可），名字为xmartev

>   非常重要：仓库名必须设置为xmartev，不可自定义，否则评委无法拉取选手镜像
>

![create_repo](doc/readme_assets/1.png)

![alt text](doc/readme_assets/2.png)

代码源设置为本地仓库

![code_base](doc/readme_assets/13.png)

### 2. 将client镜像push到阿里云ACR仓库
#### 先登录阿里云ACR服务：
可在阿里云容器镜像服务/实例列表/镜像仓库/基本信息中查看登录命令

输入命令后，会提示输入阿里云ACR密码，密码是您在开通ACR服务时设置的密码，不是阿里云账号密码

![view_command](doc/readme_assets/12.png)

#### 查看本地镜像id：

```
docker images
```
如图，红框中即为镜像id：

![docker_images](doc/readme_assets/15.png)

#### 将镜像推送到Registry：
```
docker tag <ImageId> <阿里云ACR地址>:<tag_name>
docker push <阿里云ACR地址>:<tag_name>
```

注意：
1. ImageId参考`docker images`输出
2. 阿里云ACR地址可参考下图方式获取
2. <tag_name>命名规则：
- 如参赛团队名为中文，tag_name为小写拼音+电话号码格式。例如：参赛团队名为“映体科技”，则tag_name：yingtikeji13419923657
- 如团队名包含英文，tag_name为团队英文名+电话号码格式，例如：参赛团队名为”Enbody Lab”，则tag_name：Enbody_Lab13419923657 （注意：tag_name中不能有空格，可用“_”代替空格）

![change_docker_tag](doc/readme_assets/3.png)

### 3. 开发比赛任务

根据private repo和tag名字，修改create_container_client.sh里的镜像名和tag,这里第三行的block_challenge_client是container_id

![image-20250220181043385](doc/readme_assets/bash.png)

运行create_client.sh，创建新容器。

运行exec_client.sh脚本，进入客户端镜像终端开始进行开发工作。

强烈推荐使用 Git 工具进行代码版本管理，以确保代码的安全性和可追溯性。

在 Docker 环境中，您也可以通过 Visual Studio Code（VS Code）安装docker插件 进行高效开发。

### 4. docker commit（使用 docker commit 保存容器内修改）

本地保存镜像修改内容，使用原有的tag会覆盖之前tag版本的内容，如这里我们的container_id为block_challenge_client,。
可以终端输入docker ps来查看container_id,如果还有疑问可以查看[docker commit官方说明](https://docs.docker.com/reference/cli/docker/container/commit/)
```
docker ps
```
```
docker commit container_id <阿里云ACR地址>:<new_tag_name>
```

注意：<new_tag_name>可在原有tag_name之后，拼接上版本号，例如：原有tag_name为“yingtikeji13419923657”，则new_tag_name为“yingtikeji13419923657_release_1”


### 5. docker push（推送新镜像）

通过docker push到阿里云ACR仓库保存当前docker镜像
```
docker push <阿里云ACR地址>:<new_tag_name>
```

### 6. 设置固定密码（用于评测拉取）

>   非常重要：您稍后会在比赛系统中提交固定密码，评委会用此密码拉取您的镜像。您在开通ACR服务时已经设置过固定密码了，如遗忘，请按照下面操作修改。
>
进入ACR服务控制台

![enter_account_setting](doc/readme_assets/8.png)

修改固定密码

![create_token_pos](doc/readme_assets/9.png)

### 7.在收集表中提交信息

选手需要在收集表中提交以下信息，以方便裁判拉取镜像评分：

收集表地址：https://w79rvfxw83.feishu.cn/share/base/form/shrcnGl69hAadXWlNm9ZNSZ4uOe

1. aliyun_username ：如下图方式获取
2. 固定密码 : 您在上一步中设置的密码
3. 阿里云ACR地址 ：如下图方式获取
4. new_tag_name ：最新版本的tag名称
5. [极市开发者平台](https://cvmart.net/)队长用户ID和用户名 ： 队长登录进入平台，在右上角点击头像可查看自己的ID和用户名

![get_session](doc/readme_assets/14.png)



