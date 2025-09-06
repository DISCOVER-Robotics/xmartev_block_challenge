# Block Challenge Baseline 说明

```bash
# 从镜像仓库拉取（

docker pull crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/xmartev/block_challenge_client:dev_v1.0


# 查看是否成功获取 xmartev/block_challenge_client 镜像
docker images
```
代码位于创建的容器 /workspace/block_challenge_baseline 中

## 1. 系统架构与原理介绍

### 1.1 系统整体架构

本系统基于ROS2实现，主要由三部分组成：
- **基础控制系统**（`task_base_play.py`）：提供AirbotPlay的基础控制接口
- **视觉感知系统**（`yolo_detect_obb.py`）：负责目标检测和位置估计
- **任务执行系统**（`baseline_round1_seed99.py`）：通过状态机实现具体任务流程

系统工作流程如下：
1. 视觉系统接收摄像头图像，进行目标检测并估计3D位置
2. 基础控制系统接收关节状态数据，提供机械臂控制接口
3. 任务执行系统根据状态机逻辑，控制机械臂执行抓取、移动等操作

### 1.2 坐标系统说明

系统中涉及多个坐标系：
- **世界坐标系**：固定在世界中的绝对坐标系
- **机器人基座标系**：以机器人底盘中心为原点的坐标系
- **相机坐标系**：以头部相机为原点的坐标系，其中X向右，Y向下，Z向前

坐标转换流程：
1. YOLO检测得到2D图像坐标
2. 使用深度信息和相机内参将2D坐标转换为相机坐标系中的3D位置
3. 通过坐标变换将相机坐标系中的位置转换到机器人基座标系
4. 在基座标系中进行运动规划和控制

## 2. 关键模块原理解析

### 2.1 基础控制模块（task_base_play.py）

`AIRBOTPLAYTaskBase`类提供基础控制功能：
- 关节状态监控和控制
- 机器人运动学计算（正向和逆向）
- 坐标系转换
- ROS话题订阅与发布

核心功能包括：
```python
# 机械臂末端位置设置（逆运动学）
setArmEndTarget(target_pose, arm_action, arm, q_ref, a_rot)

# 底盘移动控制
base_move(vx, vyaw)

# 坐标系转换：相机坐标系到基座标系
pcam2base(target)
```

### 2.2 视觉感知模块（yolo_detect_obb.py）

`YOLODetectorNode`负责目标检测与位置估计：
- 加载YOLO模型进行目标检测
- 使用深度信息计算3D位置
- 发布检测结果给其他模块

目标3D定位核心原理：
```python
# 使用针孔相机模型进行反投影
Z = depth_m  # 深度
X = (center_x - cx) * Z / fx  # X坐标计算
Y = (center_y - cy) * Z / fy  # Y坐标计算
```

### 2.3 任务执行模块（baseline_round1_seed99.py）

`SimNode`类实现具体任务：
- 使用简单状态机（`SimpleStateMachine`）管理任务流程
- 每个状态对应特定动作（移动、抓取等）
- 通过检测完成条件自动转换状态

状态机示例：
```python
# 状态2：检测盒子并准备抓取
elif self.stm.state_idx == 2:
    # 检测盒子位置
    self.tar_box_posi = self.update_target_pose("box", [0.3, 0.7])
    # 设置左臂位置为盒子上方
    tmp_lft_arm_target_pose = self.tar_box_posi + np.array([-0.14, 0.09, 0.05])
    # 设置左臂末端姿态
    self.setArmEndTarget(...)
```

## 3. 代码扩展指南

### 3.1 添加新的目标检测类别

在`yolo_detect.py`中：
1. 修改`self.class_names`列表添加新类别：
```python
self.class_names = ["box", "carton", "disk", "sheet", "airbot", "blue_circle", "新类别名"]
```
2. 确保YOLO模型已训练识别新类别
3. 根据需要调整置信度阈值`self.conf_thresh`

### 3.2 自定义机器人运动控制

调整机械臂运动参数：
```python
# 修改机械臂目标位置偏移
tmp_lft_arm_target_pose = self.tar_box_posi + np.array([偏移X, 偏移Y, 偏移Z])

# 调整末端姿态
a_rot = Rotation.from_euler("zyx", [角度X, 角度Y, 角度Z]).as_matrix()
```

### 3.3 扩展状态机任务

修改状态机大小：
```python
self.stm.max_state_cnt = 新状态数  # 修改状态机最大状态数
```

### 3.4 实现新任务流程示例

例如，添加圆形物体识别和分类任务：

1. 添加物体检测功能：
```python
# 检测特定形状物体
target_obj = self.update_target_pose("新形状", [0.3, 0.7])
```

2. 添加颜色分类状态：
```python
elif self.stm.state_idx == 新状态:
    # 根据颜色分类
    if obj['color'] == "red":
        # 放入红色区域
        self.target_posi_x = 红色区域X
    else:
        # 放入其他区域
        self.target_posi_x = 其他区域X
```

### 3.5 调试与参数优化

关键参数调整：
- 调整YOLO置信度阈值：`self.conf_thresh`
- 控制速度参数：移动速度和旋转速度限制
- 抓取位置偏移：根据实际抓取效果调整偏移量
- 时间延迟：`self.delay_cnt`参数

推荐调试流程：
1. 先进行目标检测调试，确保位置准确
2. 调整单个动作（移动、抓取）参数
3. 测试完整任务流程
4. 优化时间和稳定性

## 4. 常见问题与解决方案

1. **目标检测不准确**：
   - 检查相机标定参数是否正确
   - 调整`conf_thresh`阈值
   - 考虑在不同光照条件下重新训练模型

2. **机械臂无法到达目标**：
   - 检查目标位置是否在工作空间内
   - 调整目标位置的偏移量
   - 修改逆运动学参数或使用不同的参考姿态

3. **状态机卡在某个状态**：
   - 检查完成条件判断逻辑
   - 增加超时处理机制
   - 添加调试输出信息

4. **坐标转换错误**：
   - 验证各坐标系的定义是否一致
   - 检查四元数到旋转矩阵的转换
   - 使用可视化工具验证转换结果# Xmartev Block Challenge Baseline
