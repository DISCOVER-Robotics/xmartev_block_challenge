import threading
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

from discoverse.robots import MMK2FIK
from discoverse.utils.joy_stick_ros2 import JoyTeleopRos2

class Ros2JoyCtl(Node):
    init_joint_ctrl = np.array([
        0.   ,  0.   ,  0.   ,  0.   ,  0.   ,  
        0.   , -0.166,  0.032,  0.   ,  1.571,  2.223,  0., 
        0.   , -0.166,  0.032,  0.   , -1.571, -2.223,  0.
    ])
    arm_action_init_position = {
        "pick" : {
            "l" : np.array([0.223,  0.21, 1.07055]),
            "r" : np.array([0.223, -0.21, 1.07055]),
        },
    }
    target_control = np.zeros(19)

    def __init__(self, freq):
        Node.__init__(self, 'joy_node')
        self.fps = freq

        self.arm_action = "pick"
        self.tctr_base = self.target_control[:2]
        self.tctr_slide = self.target_control[2:3]
        self.tctr_head = self.target_control[3:5]
        self.tctr_left_arm = self.target_control[5:11]
        self.tctr_lft_gripper = self.target_control[11:12]
        self.tctr_right_arm = self.target_control[12:18]
        self.tctr_rgt_gripper = self.target_control[18:19]

        self.lft_arm_target_pose = self.arm_action_init_position[self.arm_action]["l"].copy()
        self.lft_end_euler = np.zeros(3)
        self.rgt_arm_target_pose = self.arm_action_init_position[self.arm_action]["r"].copy()
        self.rgt_end_euler = np.zeros(3)

        # command publisher
        self.cmd_vel_puber = self.create_publisher(Twist, '/cmd_vel', 5)
        self.spine_cmd_puber = self.create_publisher(Float64MultiArray, '/spine_forward_position_controller/commands', 5)
        self.head_cmd_puber = self.create_publisher(Float64MultiArray, '/head_forward_position_controller/commands', 5)
        self.left_arm_cmd_puber = self.create_publisher(Float64MultiArray, '/left_arm_forward_position_controller/commands', 5)
        self.right_arm_cmd_puber = self.create_publisher(Float64MultiArray, '/right_arm_forward_position_controller/commands', 5)

        # joy subscriber
        self.teleop = JoyTeleopRos2()
        self.sub = self.create_subscription(Joy, '/joy', self.teleop.joy_callback, 10)
        self.reset()

    def reset(self):
        self.target_control[:] = self.init_joint_ctrl[:]
        self.lft_arm_target_pose = self.arm_action_init_position[self.arm_action]["l"].copy()
        self.lft_end_euler = np.zeros(3)
        self.rgt_arm_target_pose = self.arm_action_init_position[self.arm_action]["r"].copy()
        self.rgt_end_euler = np.zeros(3)
        self.teleop.reset()

    def pubros2cmd(self):
        # cmd_vel
        cmd_vel = Twist()
        cmd_vel.linear.x = self.tctr_base[0]
        cmd_vel.angular.z = self.tctr_base[1]
        self.cmd_vel_puber.publish(cmd_vel)
        # spine
        spine_cmd = Float64MultiArray()
        spine_cmd.data = self.tctr_slide.tolist()
        self.spine_cmd_puber.publish(spine_cmd)
        # head
        head_cmd = Float64MultiArray()
        head_cmd.data = self.tctr_head.tolist()
        self.head_cmd_puber.publish(head_cmd)
        # left arm
        left_arm_cmd = Float64MultiArray()
        left_arm_cmd.data = self.tctr_left_arm.tolist() + self.tctr_lft_gripper.tolist()
        self.left_arm_cmd_puber.publish(left_arm_cmd)
        # right arm
        right_arm_cmd = Float64MultiArray()
        right_arm_cmd.data = self.tctr_right_arm.tolist() + self.tctr_rgt_gripper.tolist()
        self.right_arm_cmd_puber.publish(right_arm_cmd)

    def teleopProcess(self):
        linear_vel  = 0.0
        angular_vel = 0.0

        if self.teleop.joy_cmd.buttons[2]:   # B
            self.reset()
        
        if self.teleop.joy_cmd.buttons[4]:   # left arm
            tmp_lft_arm_target_pose = self.lft_arm_target_pose.copy()
            tmp_lft_arm_target_pose[0] += self.teleop.joy_cmd.axes[7] * 0.1 / self.fps
            tmp_lft_arm_target_pose[1] += self.teleop.joy_cmd.axes[6] * 0.1 / self.fps
            tmp_lft_arm_target_pose[2] += self.teleop.joy_cmd.axes[1] * 0.1 / self.fps

            delta_gripper = (self.teleop.joy_cmd.axes[2] - self.teleop.joy_cmd.axes[5]) * 1. / self.fps
            self.tctr_lft_gripper[0] += delta_gripper
            self.tctr_lft_gripper[0] = np.clip(self.tctr_lft_gripper[0], 0, 1)
            el = self.lft_end_euler.copy()
            el[0] += self.teleop.joy_cmd.axes[4] * 0.35 / self.fps
            el[1] += self.teleop.joy_cmd.axes[3] * 0.35 / self.fps
            el[2] += self.teleop.joy_cmd.axes[0] * 0.35 / self.fps
            try:
                self.tctr_left_arm[:] = MMK2FIK().get_armjoint_pose_wrt_footprint(tmp_lft_arm_target_pose, self.arm_action, "l", self.tctr_slide[0], self.tctr_left_arm, Rotation.from_euler('zyx', el).as_matrix())
                self.lft_arm_target_pose[:] = tmp_lft_arm_target_pose
                self.lft_end_euler[:] = el
            except ValueError:
                print("Invalid left arm target position:", tmp_lft_arm_target_pose)

        if self.teleop.joy_cmd.buttons[5]: # right arm
            tmp_rgt_arm_target_pose = self.rgt_arm_target_pose.copy()
            tmp_rgt_arm_target_pose[0] += self.teleop.joy_cmd.axes[7] * 0.1 / self.fps
            tmp_rgt_arm_target_pose[1] += self.teleop.joy_cmd.axes[6] * 0.1 / self.fps
            tmp_rgt_arm_target_pose[2] += self.teleop.joy_cmd.axes[1] * 0.1 / self.fps

            delta_gripper = (self.teleop.joy_cmd.axes[2] - self.teleop.joy_cmd.axes[5]) * 1. / self.fps
            self.tctr_rgt_gripper[0] += delta_gripper
            self.tctr_rgt_gripper[0] = np.clip(self.tctr_rgt_gripper[0], 0, 1)
            el = self.rgt_end_euler.copy()
            el[0] -= self.teleop.joy_cmd.axes[4] * 0.35 / self.fps
            el[1] += self.teleop.joy_cmd.axes[3] * 0.35 / self.fps
            el[2] -= self.teleop.joy_cmd.axes[0] * 0.35 / self.fps
            try:
                self.tctr_right_arm[:] = MMK2FIK().get_armjoint_pose_wrt_footprint(tmp_rgt_arm_target_pose, self.arm_action, "r", self.tctr_slide[0], self.tctr_right_arm, Rotation.from_euler('zyx', el).as_matrix())
                self.rgt_arm_target_pose[:] = tmp_rgt_arm_target_pose
                self.rgt_end_euler[:] = el
            except ValueError:
                print("Invalid right arm target position:", tmp_rgt_arm_target_pose)

        if (not self.teleop.joy_cmd.buttons[4]) and (not self.teleop.joy_cmd.buttons[5]):
            delta_height = (self.teleop.joy_cmd.axes[2] - self.teleop.joy_cmd.axes[5]) * 0.1 / self.fps
            if self.tctr_slide[0] + delta_height < -0.04:
                delta_height = -0.04 - self.tctr_slide[0]
            elif self.tctr_slide[0] + delta_height > 0.87:
                delta_height = 0.87 - self.tctr_slide[0]
            self.tctr_slide[0] += delta_height
            self.lft_arm_target_pose[2] -= delta_height
            self.rgt_arm_target_pose[2] -= delta_height

            self.tctr_head[0] += self.teleop.joy_cmd.axes[3] * 1. / self.fps
            self.tctr_head[1] += self.teleop.joy_cmd.axes[4] * 1. / self.fps
            self.tctr_head[0] = np.clip(self.tctr_head[0], -0.5 , 0.5)
            self.tctr_head[1] = np.clip(self.tctr_head[1], -1.18, 0.16)

            linear_vel  = 1.0 * self.teleop.joy_cmd.axes[1]**2 * np.sign(self.teleop.joy_cmd.axes[1])
            angular_vel = 2.0 * self.teleop.joy_cmd.axes[0]**2 * np.sign(self.teleop.joy_cmd.axes[0])

        self.tctr_base[0] = linear_vel
        self.tctr_base[1] = angular_vel

    def printMessage(self):
        super().printMessage()
        print("    lta local = {}".format(self.lft_arm_target_pose))
        print("    rta local = {}".format(self.rgt_arm_target_pose))
        print("       euler  = {}".format(self.lft_end_euler))
        print("       euler  = {}".format(self.rgt_end_euler))

if __name__ == "__main__":
    rclpy.init()

    pub_freq = 30
    joy_node = Ros2JoyCtl(pub_freq)
    joy_node.reset()

    spin_thread = threading.Thread(target=lambda:rclpy.spin(joy_node))
    spin_thread.start()

    rate = joy_node.create_rate(pub_freq)    
    while rclpy.ok():
        joy_node.teleopProcess()
        joy_node.pubros2cmd()
        rate.sleep()

    joy_node.destroy_node()
    rclpy.shutdown()
    spin_thread.join()