import os
import sys

current_dir = os.path.dirname(os.path.realpath(__file__))
project_root = os.path.dirname(current_dir)
models_dir = os.path.join(project_root, "models")
ply_dir = os.path.join(project_root, "models", "3dgs")
os.environ["DISCOVERSE_ASSERT_DIR"] = models_dir
sys.path.insert(0, project_root)

json_name = "s2_rules.json"
json_dir = f"{current_dir}/referee_json/{json_name}"
# print(json_dir)
# print(models_dir)



import glfw
import mujoco
import argparse
import threading
import numpy as np
from scipy.spatial.transform import Rotation
from discoverse.robots_env.airbot_play_base import AirbotPlayCfg
from discoverse.examples.ros2.airbot_play_ros2 import AirbotPlayROS2
from discoverse.utils import get_body_tmat, step_func, SimpleStateMachine, get_site_tmat

import rclpy 
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock

from referee import Referee

# 场景与仿真配置
cfg = AirbotPlayCfg()
# cfg.mjcf_file_path = os.path.join(models_dir, "mjcf/jishijiao_s2_tmp.xml")
cfg.mjcf_file_path = os.path.join(models_dir, "mjcf/block_challenge.xml")

cfg.timestep       = 0.003
cfg.decimation     = 2

cfg.init_key = "pick"
cfg.sync     = True
cfg.headless = False
cfg.render_set = {
    "fps"    : 24,
    "width"  : 640,
    "height" : 480,
}


# 更新后的 cfg.gs_model_dict
cfg.gs_model_dict = {
    # 机器人部件模型
    "arm_base": os.path.join(ply_dir, "airbot_play", "arm_base.ply"),
    "link1": os.path.join(ply_dir, "airbot_play", "link1.ply"),
    "link2": os.path.join(ply_dir, "airbot_play", "link2.ply"),
    "link3": os.path.join(ply_dir, "airbot_play", "link3.ply"),
    "link4": os.path.join(ply_dir, "airbot_play", "link4.ply"),
    "link5": os.path.join(ply_dir, "airbot_play", "link5.ply"),
    "link6": os.path.join(ply_dir, "airbot_play", "link6.ply"),
    "left": os.path.join(ply_dir, "airbot_play", "left.ply"),
    "right": os.path.join(ply_dir, "airbot_play", "right.ply"),    
    # 场景物体模型
    "background": os.path.join(ply_dir, "scene", "scene.ply"),
    "code1_1": os.path.join(ply_dir, "block_challenge", "code1.ply"),
    "code1_2": os.path.join(ply_dir, "block_challenge", "code1.ply"),
    "code1_3": os.path.join(ply_dir, "block_challenge", "code1.ply"),
    "code1_4": os.path.join(ply_dir, "block_challenge", "code1.ply"),
    "code2_1": os.path.join(ply_dir, "block_challenge", "code2.ply"),
    "code2_2": os.path.join(ply_dir, "block_challenge", "code2.ply"),
    "code2_3": os.path.join(ply_dir, "block_challenge", "code2.ply"),
    "code3_1": os.path.join(ply_dir, "block_challenge", "code3.ply"),
    "code3_2": os.path.join(ply_dir, "block_challenge", "code3.ply"),
    "code4": os.path.join(ply_dir, "block_challenge", "code4.ply"),
    "tuopan": os.path.join(ply_dir, "block_challenge", "tuopan.ply"),
    "mark": os.path.join(ply_dir, "block_challenge", "mark.ply")
 
}

cfg.obj_list = [
    # 机器人部件
    "arm_base",
    "link1", "link2", "link3", "link4", "link5", "link6",
    "left", "right",
    # 场景物体
    "background",
    "code1_1",
    "code1_2",
    "code1_3",
    "code1_4",
    "code2_1", 
    "code2_2",
    "code2_3",
    "code3_1", 
    "code3_2",
    "code4",
    "tuopan",
    "mark"
]



# 传感器配置
cfg.obs_rgb_cam_id = [0]  # 保留RGB相机
cfg.obs_depth_cam_id = [0]      # 保留深度相机
cfg.use_gaussian_renderer = True  # 启用高斯渲染

class SceneROS2Node(AirbotPlayROS2):
    def __init__(self, config):
        super().__init__(config)
        # 初始化时钟发布器
        self.clock_publisher_ = self.create_publisher(Clock, '/clock', 10)
        timer_period = 0.01  # 100Hz时钟发布
        self.clock_timer = self.create_timer(timer_period, self.timer_callback)      
        self.referee = Referee(self.mj_model, os.path.join(json_dir))
        self.round_id = self.config.round_id         

    # 时钟发布回调
    def timer_callback(self):
        msg = Clock()
        msg.clock.sec = int(self.mj_data.time)
        msg.clock.nanosec = int((self.mj_data.time - int(self.mj_data.time)) * 1e9)
        self.clock_publisher_.publish(msg)

    # 模型加载后初始化（移除任务相关逻辑）
    def post_load_mjcf(self):
        super().post_load_mjcf()

    # 重置场景（保留基础重置逻辑）
    def reset(self):
        ret = super().reset()
        # 移除任务相关的道具位置随机化和任务初始化
        mujoco.mj_forward(self.mj_model, self.mj_data)

        r = f"round{self.round_id}"

        task_info = f"{r}: Build blocks with reference to specified shapes."
        print(f"{task_info}")
        return ret

    def on_key(self, window, key, scancode, action, mods):
        super().on_key(window, key, scancode, action, mods)

        if key in {glfw.KEY_R, glfw.KEY_G, glfw.KEY_D}:
            key = glfw.KEY_UNKNOWN
        super().on_key(window, key, scancode, action, mods)

    # 物理步后处理（移除任务评分逻辑）
    def post_physics_step(self):
        # 仅保留必要的场景更新，移除所有任务检查逻辑
        self.referee.update(self.mj_data)
        # pass

    # 终止条件检查（始终返回False，保持场景运行）
    def checkTerminated(self):
        return False

    def pubRos2TopicOnce(self):
        rate = self.create_rate(30)
        while rclpy.ok():
            time_stamp = self.get_clock().now().to_msg()

            self.joint_state.header.stamp = time_stamp
            self.joint_state.position = self.sensor_joint_qpos.tolist()
            self.joint_state.velocity = self.sensor_joint_qvel.tolist()
            self.joint_state.effort = self.sensor_joint_force.tolist()
            self.joint_state_puber.publish(self.joint_state)

            side_color_img_msg = self.bridge.cv2_to_imgmsg(self.obs["img"][0], encoding="rgb8")
            side_color_img_msg.header.stamp = time_stamp
            side_color_img_msg.header.frame_id = "side_camera"
            self.side_color_puber.publish(side_color_img_msg)

            side_depth_img = np.array(np.clip(self.obs["depth"][0]*1e3, 0, 65535), dtype=np.uint16)
            side_depth_img_msg = self.bridge.cv2_to_imgmsg(side_depth_img, encoding="mono16")
            side_depth_img_msg.header.stamp = time_stamp
            side_depth_img_msg.header.frame_id = "side_camera"
            self.side_depth_puber.publish(side_depth_img_msg)
            
            rate.sleep()


    def thread_pubGameInfo(self, freq=1):
        rate1 = self.create_rate(freq)
        r = f"round{self.round_id}"

        self.task_info_puber = self.create_publisher(String, '/xmartev_block_challenge/taskinfo', 2)
        task_info_msg = String()

        task_info_msg.data = f"{r}: Build blocks with reference to specified shapes."
        

        self.game_info_puber = self.create_publisher(String, '/xmartev_block_challenge/gameinfo', 2)

        while rclpy.ok() and self.running:

            self.task_info_puber.publish(task_info_msg)
            self.game_info_puber.publish(self.referee.game_info_msg)
            

            # print(task_info_msg.data)
            # print(self.referee.game_info_msg.data)

            rate1.sleep()


if __name__ == "__main__":
    rclpy.init()
    np.set_printoptions(precision=3, suppress=True, linewidth=200)
 
    # 简化参数解析（仅保留必要配置）
    parser = argparse.ArgumentParser(description='ROS2 Scene Server for xmartev_block_challenge Environment')
    parser.add_argument('--round_id', type=int, choices=[1, 2, 3], help='tasks round index', required=True)
    args = parser.parse_args()
    # 初始化场景节点
    
    cfg.round_id = args.round_id

    sim_node = SceneROS2Node(cfg)
    obs = sim_node.reset()

    # 启动ROS2自旋线程
    spin_thread = threading.Thread(target=lambda: rclpy.spin(sim_node))
    spin_thread.start()

    # 启动传感器话题发布线程
    pubtopic_thread = threading.Thread(target=sim_node.pubRos2TopicOnce)
    pubtopic_thread.start()

    pubgameinfo_thread = threading.Thread(target=sim_node.thread_pubGameInfo)
    pubgameinfo_thread.start()

    try:
        # 主仿真循环
        while rclpy.ok() and sim_node.running:
            obs, _, _, ter, _ = sim_node.step(sim_node.tar_jq)

            if ter:
                print(f"get final score:{sim_node.referee.total_score[0]}")
                sim_node.running = False
                break
            
            
            sim_time = sim_node.mj_data.time
            if sim_time >= 300:
                print(f"get final score:{sim_node.referee.total_score[0]}")
                sim_node.running = False
                break



    except KeyboardInterrupt:
        pass

    finally:
        # 资源清理
        sim_node.destroy_node()
        rclpy.shutdown()
        print("Scene ROS2 Server shutdown complete")
