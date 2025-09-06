import os
os.environ["DISCOVERSE_ASSETS_DIR"] = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../models")

import glfw

import mujoco
import argparse
import threading
import numpy as np
from discoverse.robots_env.airbot_play_base import AirbotPlayCfg
from discoverse.examples.ros2.airbot_play_ros2 import AirbotPlayROS2

import rclpy 
from rosgraph_msgs.msg import Clock

from referee import Referee

json_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "referee_json/s2_rules.json")

# 场景与仿真配置
cfg = AirbotPlayCfg()
cfg.mjcf_file_path = "mjcf/block_challenge.xml"

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

cfg.gs_model_dict["background"] = "scene/scene.ply"
cfg.gs_model_dict["code1_1"]    = "block_challenge/code1.ply"
cfg.gs_model_dict["code1_2"]    = "block_challenge/code1.ply"
cfg.gs_model_dict["code1_3"]    = "block_challenge/code1.ply"
cfg.gs_model_dict["code1_4"]    = "block_challenge/code1.ply"
cfg.gs_model_dict["code2_1"]    = "block_challenge/code2.ply"
cfg.gs_model_dict["code2_2"]    = "block_challenge/code2.ply"
cfg.gs_model_dict["code2_3"]    = "block_challenge/code2.ply"
cfg.gs_model_dict["code3_1"]    = "block_challenge/code3.ply"
cfg.gs_model_dict["code3_2"]    = "block_challenge/code3.ply"
cfg.gs_model_dict["code4"]      = "block_challenge/code4.ply"
cfg.gs_model_dict["tuopan"]     = "block_challenge/tuopan_trans.ply"
cfg.gs_model_dict["mark"]       = "block_challenge/mark.ply"

cfg.obj_list = [ "code1_1",  "code1_2",  "code1_3", "code1_4", "code2_1",  "code2_2", "code2_3", "code3_1", "code3_2", "code4", "tuopan", "mark"]

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
        return ret

    def on_key(self, window, key, scancode, action, mods):
        if key in {glfw.KEY_R, glfw.KEY_G, glfw.KEY_D}:
            key = glfw.KEY_UNKNOWN
        super().on_key(window, key, scancode, action, mods)

    # 物理步后处理（移除任务评分逻辑）
    def post_physics_step(self):
        # 仅保留必要的场景更新，移除所有任务检查逻辑
        self.referee.update(self.mj_data)

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

if __name__ == "__main__":
    rclpy.init()
    np.set_printoptions(precision=3, suppress=True, linewidth=200)
 
    # 简化参数解析（仅保留必要配置）
    parser = argparse.ArgumentParser(description='ROS2 Scene Server for S2R2025 Environment')
    parser.add_argument('--random_seed', type=int, help='Random seed for environment', default=None, required=False)
    args = parser.parse_args()

    # 初始化场景节点
    sim_node = SceneROS2Node(cfg)
    obs = sim_node.reset()

    # 启动ROS2自旋线程
    spin_thread = threading.Thread(target=lambda: rclpy.spin(sim_node))
    spin_thread.start()

    # 启动传感器话题发布线程
    pubtopic_thread = threading.Thread(target=sim_node.pubRos2TopicOnce)
    pubtopic_thread.start()

    try:
        # 主仿真循环
        while rclpy.ok() and sim_node.running:
            obs, _, _, ter, _ = sim_node.step(sim_node.tar_jq)
            if ter or sim_node.mj_data.time >= 180:
                print(f"最终得分：{sim_node.referee.total_score[0]}分")
                sim_node.running = False
                break

    except KeyboardInterrupt:
        pass

    finally:
        # 资源清理
        sim_node.destroy_node()
        rclpy.shutdown()
        print("Scene ROS2 Server shutdown complete")
