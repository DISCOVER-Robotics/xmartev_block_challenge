import os
import threading
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

from discoverse.robots import AirbotPlayIK
from discoverse.utils.joy_stick_ros2 import JoyTeleopRos2

class AirbotPlayROS2JoyCtl(Node):
    def __init__(self):
        super().__init__('airbot_play_joy_control')
        
        # 初始化逆运动学
        self.arm_ik = AirbotPlayIK()
        
        # 目标末端位置和欧拉角
        self.tar_end_pose = np.array([0.295, -0., 0.219])
        self.tar_end_euler = np.zeros(3)
        
        # 目标关节角度（7个：6个臂关节 + 1个夹爪）
        self.tar_jq = np.zeros(7)
        
        # 初始位置（用于reset）
        self.initial_end_pose = np.array([0.295, -0., 0.219])
        self.initial_end_euler = np.zeros(3)
        self.initial_jq = np.zeros(7)
        
        # 当前关节角度（从话题获取）
        self.sensor_joint_qpos = np.zeros(7)
        
        # 订阅手柄
        self.teleop = JoyTeleopRos2()
        self.sub_joy = self.create_subscription(Joy, '/joy', self.teleop.joy_callback, 10)
        
        # 发布关节命令
        self.publisher_joints = self.create_publisher(
            Float64MultiArray, 
            '/airbot_play/forward_position_controller/commands', 
            10
        )
        
        # 订阅关节状态
        self.sub_joint_states = self.create_subscription(
            JointState, 
            '/airbot_play/joint_states', 
            self.joint_states_callback, 
            10
        )
        
        # 时间间隔
        self.delta_t = 0.01  # 100Hz
        
        # 创建定时器，定期处理手柄输入并发布命令
        self.timer = self.create_timer(self.delta_t, self.teleopProcess)
        
        # 运行标志
        self.running = True
        
        self.get_logger().info("Airbot Play Joy Control initialized")
    
    def joint_states_callback(self, msg):
        # 假设关节顺序为：6个臂关节 + 1个夹爪关节
        # 注意：需要根据实际话题中的关节顺序调整
        if len(msg.position) >= 7:
            self.sensor_joint_qpos = np.array(msg.position[:7])
    
    def reset(self):
        """
        重置机械臂到初始位置
        Reset arm to initial position
        """
        self.get_logger().info("Resetting arm to initial position")
        # 重置目标位置和姿态
        self.tar_end_pose = self.initial_end_pose.copy()
        self.tar_end_euler = self.initial_end_euler.copy()
        
        # 重置目标关节角度
        self.tar_jq = self.initial_jq.copy()
    
    def teleopProcess(self):
        calc_ik = False
        tmp_arm_target_pose = self.tar_end_pose.copy()

        # 检查是否按下重置按钮（B按钮）
        if self.teleop.joy_cmd.buttons[1]:  # 注意：buttons索引从0开始，B按钮通常是索引1
            self.reset()
            return  # 重置后直接返回，避免在重置过程中处理其他输入
        
        # 检查手柄输入是否有变化
        if self.teleop.joy_cmd.axes[0] or self.teleop.joy_cmd.axes[1] or self.teleop.joy_cmd.axes[4]:
            calc_ik = True
            # 更新目标位置
            tmp_arm_target_pose[0] += 0.15 * self.teleop.joy_cmd.axes[1] * self.delta_t
            tmp_arm_target_pose[1] += 0.15 * self.teleop.joy_cmd.axes[0] * self.delta_t
            tmp_arm_target_pose[2] += 0.1 * self.teleop.joy_cmd.axes[4] * self.delta_t

        el = self.tar_end_euler.copy()
        if self.teleop.joy_cmd.axes[3] or self.teleop.joy_cmd.axes[6] or self.teleop.joy_cmd.axes[7]:
            calc_ik = True
            # 更新目标欧拉角
            el[0] += 0.01 * self.teleop.joy_cmd.axes[3]
            el[1] += 0.01 * self.teleop.joy_cmd.axes[7]
            el[2] += 0.01 * self.teleop.joy_cmd.axes[6]

        if calc_ik:
            # 计算旋转矩阵
            rot = Rotation.from_euler('xyz', el).as_matrix()
            try:
                # 计算逆运动学
                tarjq = self.arm_ik.properIK(self.tar_end_pose, rot, self.sensor_joint_qpos[:6])
                # 更新目标位置和欧拉角
                self.tar_end_pose[:] = tmp_arm_target_pose[:]
                self.tar_end_euler[:] = el[:]
            except ValueError:
                tarjq = None

            if not tarjq is None:
                self.tar_jq[:6] = tarjq
            else:
                self.get_logger().warn("Fail to solve inverse kinematics trans={} euler={}".format(self.tar_end_pose, self.tar_end_euler))

        # 处理夹爪控制
        if self.teleop.joy_cmd.axes[2] - self.teleop.joy_cmd.axes[5]:
            self.tar_jq[6] += 1. * (self.teleop.joy_cmd.axes[2] - self.teleop.joy_cmd.axes[5]) * self.delta_t
            self.tar_jq[6] = np.clip(self.tar_jq[6], 0, 1.)
            
        # 发布关节命令
        self.publish_joint_command()
        
        # 打印状态信息
        self.printMessage()

    def publish_joint_command(self):
        """发布关节命令到控制器"""
        msg = Float64MultiArray()
        msg.data = self.tar_jq.tolist()
        self.publisher_joints.publish(msg)
        self.get_logger().info(f'Publishing joint positions: {self.tar_jq}', throttle_duration_sec=1.0)

    def printMessage(self):
        """打印当前状态信息"""
        self.get_logger().info(f"Target end position: {self.tar_end_pose}", throttle_duration_sec=1.0)
        self.get_logger().info(f"Target end euler: {self.tar_end_euler}", throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = AirbotPlayROS2JoyCtl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()