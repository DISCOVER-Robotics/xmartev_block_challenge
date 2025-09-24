import os
import json
import time
import numpy as np
from scipy.spatial.transform import Rotation
from std_msgs.msg import String



import mujoco

SUFFIX = "_main"
# ROBOT_NAME = "mmk2"

class Referee:
    invalid_names = set()
    def __init__(self, mj_model, rules_file_path):
        self.time_stamp = time.time()
        self.mj_model = mj_model
        self.rules = json.load(open(rules_file_path, 'r', encoding='utf-8'))
        for k in self.rules:
            self.rules[k]['status'] = None
            self.rules[k]['sim_time'] = None


        self.robot_name = "mmk2"
        self.game_info_msg = String()


    def update(self, mj_data):
        self.mj_data = mj_data
        sim_time = self.mj_data.time

        # print(self.game_info_msg.data)

        # breakpoint() 
        # print(self.robot_name)
        # print("哈哈")

        for k, v in self.rules.items():
            # print("键", k)
            # print("值", v)

            # Check if the rule is already satisfied
            if v['status'] in ["success", "fail"]:
                # sim_time = self.referee.rules[k]['sim_time']
                # print(self.referee.rules[k]['sim_time'])
                # description = v['description']
                # self.game_info_msg.data = str({
                # f">>>>>> {sim_time:5.2f}s: '{k}' / '{description}' {v['status']}."})

                continue
            
            # Check if prerequisites are met
            prerequisite_done = True
            if "prerequisite" in v:
                for prereq in v['prerequisite']:
                    if self.rules[prereq]['status'] != "success":
                        prerequisite_done = False
                        break
            if not prerequisite_done:
                continue

            # Check if before until
            if "until" in v and self.rules[v['until']]['status'] == "success":
                if v['status'] is None:
                    v['sim_time'] = sim_time
                    v['status'] = "success"
                    description = v['description']
                    print("<<<")
                    print(f">>>>>> {sim_time:5.2f}s: get score {v['score']:3}, '{k}' / '{description}' succeeded.")
                    print(f">>>>>> {sim_time:5.2f}s: get total score {self.total_score[0]}")
                    print("<<<")

                    continue

            if v['type'] == 'in_2d_range':
                if self.check_in_2d_range(v['range']):
                    v['status'] = "success"

            elif v['type'] == 'in_3d_range':
                if self.check_in_3d_range(v['object1_name'], v['range']):
                    v['status'] = "success"

            elif v['type'] == 'no_collision':
                # print("no_collision")

                for group in v['collision_group']:
                    robot_bodies = group[0]
                    obstacle_bodies = group[1]
                    collision_detected = False
                    for robot_body in robot_bodies:
                        for obstacle_body in obstacle_bodies:
                            if self.check_contact(robot_body, obstacle_body):
                                collision_detected = True
                                break
                        if collision_detected:
                            break
                    if collision_detected:
                        v['status'] = "fail"
                        break


                # if self.check_contact("rgt_arm_link6", "cabinet"):
                #     v['status'] = "fail"



            elif v['type'] == 'touch':
                if self.robot_name == "mmk2":
                    if (self.check_contact(f"lft_finger_left_link", v['object1_name']) or \
                        self.check_contact(f"lft_finger_right_link", v['object1_name']) or \
                        self.check_contact(f"lft_arm_link6", v['object1_name'])
                        ) or ( \
                        self.check_contact(f"rgt_finger_left_link", v['object1_name']) or \
                        self.check_contact(f"rgt_finger_right_link", v['object1_name']) or \
                        self.check_contact(f"rgt_arm_link6", v['object1_name'])
                    ):
                        v['status'] = "success"
                        # print("touch")

                if self.robot_name == "airbot_play":
                    if (self.check_contact(f"left", v['object1_name']) or \
                            self.check_contact(f"right", v['object1_name'])  
                    ):
                        v['status'] = "success"        


            elif v['type'] == 'pick_up':
                # print(self.robot_name)
                if (self.check_contact(f"left", v['object1_name']) and \
                        self.check_contact(f"right", v['object1_name'])  
                ):
                    v['status'] = "success"


            elif v['type'] == 'pick_up_dual':
                # print("检测接触")
                # print(self.check_contact(f"lft_finger_left_link", v['object1_name']) )
                # print(self.check_contact(f"lft_finger_right_link", v['object1_name']))
                # print(self.check_contact(f"rgt_finger_left_link", v['object1_name']) )
                # print(self.check_contact(f"rgt_finger_right_link", v['object1_name']))
                
                # print(self.check_contact(f"lft_arm_link6", v['object1_name']))
                # print(self.check_contact(f"rgt_arm_link6", v['object1_name']))


                if (self.check_contact(f"lft_finger_left_link", v['object1_name']) or \
                    self.check_contact(f"lft_finger_right_link", v['object1_name']) or \
                    self.check_contact(f"lft_arm_link6", v['object1_name'])
                    ) and ( \
                    self.check_contact(f"rgt_finger_left_link", v['object1_name']) or \
                    self.check_contact(f"rgt_finger_right_link", v['object1_name']) or \
                    self.check_contact(f"rgt_arm_link6", v['object1_name'])
                    ):

                    v['status'] = "success"

            elif v['type'] == 'multi_object_in_3d_range':
                place_detected = False
                for object_name in v['object_names']:
                    # if self.check_in_3d_range(object_name, v['range']) and v['placed_flags'][v['object_names'].index(object_name)] == "False":
                    if self.check_in_3d_range(object_name, v['range']) and self.check_xquat(object_name):
                        place_detected = True
                        break
                if place_detected:
                    v['status'] = "success"



            if v['status'] == "fail":
                self.rules[k]['sim_time'] = sim_time
                description = v['description']
                message = f">>>>>> {sim_time:5.2f}s: '{k}' / '{description}' {v['status']}.\n"

                self.game_info_msg.data += message

                print(f"\n>>>>>> {sim_time:5.2f}s: '{k}' / '{description}' failed.")


            elif v['status'] == "success":
                self.rules[k]['sim_time'] = sim_time
                description = v['description']
                message = f">>>>>> {sim_time:5.2f}s: '{k}' / '{description}' {v['status']}.\n"
                self.game_info_msg.data += message
                print("\n")
                print(f">>>>>> {sim_time:5.2f}s: get score {v['score']:3}, '{k}' / '{description}' succeeded.")
                print(f">>>>>> {sim_time:5.2f}s: get total score {self.total_score[0]}")


    def get_body_tmat(self, body_name):
        # body_name += SUFFIX if not body_name.endswith(SUFFIX) else ""
        tmat = np.eye(4)
        tmat[:3,:3] = Rotation.from_quat(self.mj_data.body(body_name).xquat[[1,2,3,0]]).as_matrix()
        tmat[:3,3] = self.mj_data.body(body_name).xpos
        return tmat
    
    def get_site_tmat(self, site_name):
        tmat = np.eye(4)
        tmat[:3,:3] = self.mj_data.site(site_name).xmat.reshape((3,3))
        tmat[:3,3] = self.mj_data.site(site_name).xpos
        return tmat
    
    # 检查机器人的位置是不是在2D平面位置
    def check_in_2d_range(self, range):
        robot_posi = self.get_site_tmat("base_link")[:3,3]
        # print(robot_posi)
        return (range['x'][0] <= robot_posi[0] <= range['x'][1]) and (range['y'][0] <= robot_posi[1] <= range['y'][1])
    
    # 在参考物体坐标系下，某个物体是否到达3D区域
    # def check_in_3d_range(self, object_name, range_3d, ref_body_name=None):
    def check_in_3d_range(self, object_name, range_3d):
        # ref_body_tmat = self.get_body_tmat(ref_body_name) if ref_body_name else np.eye(4)
        # object_tmat = self.get_body_tmat(object_name)
        # object_wrt_ref = np.linalg.inv(ref_body_tmat) @ object_tmat
        # ref_posi = object_wrt_ref[:3, 3]
        # return (range_3d['x'][0] <= ref_posi[0] <= range_3d['x'][1]) and \
        #        (range_3d['y'][0] <= ref_posi[1] <= range_3d['y'][1]) and \
        #        (range_3d['z'][0] <= ref_posi[2] <= range_3d['z'][1])
    

        object_tmat = self.get_body_tmat(object_name)
        absolute_posi = object_tmat[:3, 3]
        return (range_3d['x'][0] <= absolute_posi[0] <= range_3d['x'][1]) and \
               (range_3d['y'][0] <= absolute_posi[1] <= range_3d['y'][1]) and \
               (range_3d['z'][0] <= absolute_posi[2] <= range_3d['z'][1])
    
    def check_xquat(self,object_name) :
        if object_name == "code3_1" or object_name == "code3_2":
            object_quat = self.mj_data.body(object_name).xquat
            # ref_quat_0 = [0, 0, 0, 1]
            # ref_quat_1 = [1, 0, 0, 0]
            ref_quat_0 = [0.707, 0, 0, 0.707]
            ref_quat_1 = [-0.707, 0, 0, 0.707]
            return np.allclose(object_quat,ref_quat_0, rtol=0.01, atol=0.07) or np.allclose(object_quat,ref_quat_1, rtol=0.01, atol=0.07)
        else:
            object_quat = self.mj_data.body(object_name).xquat
            # ref_quat_0 = [0.707, 0, 0, 0.707]
            # ref_quat_1 = [-0.707, 0, 0, 0.707]
            ref_quat_0 = [0, 0, 0, 1]
            ref_quat_1 = [1, 0, 0, 0]
            return np.allclose(object_quat,ref_quat_0, rtol=0.01, atol=0.07) or np.allclose(object_quat,ref_quat_1, rtol=0.01, atol=0.07)


    def check_contact(self, body1, body2):
        # body1 += SUFFIX if (not body1.endswith(SUFFIX) and not body1.startswith(f"{ROBOT_NAME}/")) else ""
        # body2 += SUFFIX if (not body2.endswith(SUFFIX) and not body2.startswith(f"{ROBOT_NAME}/")) else ""
        try:
            body1_gemo_id_range = (self.mj_model.body(body1).geomadr[0], self.mj_model.body(body1).geomadr[0]+self.mj_model.body(body1).geomnum[0])
        except KeyError:
            if body1 not in self.invalid_names:
                print(f"Warning: Body '{body1}' not found in the model.")
            self.invalid_names.add(body1)
            return False
        try:
            body2_gemo_id_range = (self.mj_model.body(body2).geomadr[0], self.mj_model.body(body2).geomadr[0]+self.mj_model.body(body2).geomnum[0])
        except KeyError:
            if body2 not in self.invalid_names:
                print(f"Warning: Body '{body2}' not found in the model.")
            self.invalid_names.add(body2)
            return False
        b1r = body1_gemo_id_range if body1_gemo_id_range[0] < body2_gemo_id_range[0] else body2_gemo_id_range
        b2r = body2_gemo_id_range if body1_gemo_id_range[0] < body2_gemo_id_range[0] else body1_gemo_id_range

        for i in range(self.mj_data.ncon):
            geom_id = sorted(self.mj_data.contact.geom[i].tolist())
            if b1r[0] <= geom_id[0] < b1r[1] and b2r[0] <= geom_id[1] < b2r[1]:
                return True
        return False

    @property
    def total_score(self):
        score_sum = 0
        score_sim_state_based = 0
        for k, v in self.rules.items():
            if v['status'] == "success":
                score_sum += v['score']
                if 'state_score' in v:
                    score_sim_state_based += v['state_score']
                else:
                    score_sim_state_based += v['score']
        return score_sum, score_sim_state_based


    @property
    def task_status(self):
        status = {}
        for k, v in self.rules.items():
            score = v['score'] if v['status'] == "success" else 0
            status[v['description']] = {
                "phase": v['phase'],
                "status": v['status'],
                "sim_time": v['sim_time'],
                "score": score,
            }
            if hasattr(v, 'state_score'):
                if score > 0:
                    status[v['description']]["state_score"] = v['state_score']
                else:
                    status[v['description']]["state_score"] = 0
            else:
                status[v['description']]["state_score"] = None
        return status
    
    def save_results(self, file_path=None):
        if file_path is None:
            file_path = os.path.join(os.path.dirname(__file__), f'results_{time.strftime("%Y_%m_%d-%H_%M_%S", time.localtime(self.time_stamp))}.json')
        else:
            file_path = file_path.replace('.json', f'_{time.strftime("%Y_%m_%d-%H_%M_%S", time.localtime(self.time_stamp))}.json')
        with open(file_path, 'w', encoding='utf-8') as f:
            res = {}
            s1, s2 = self.total_score
            res["total_score"] = s1
            res["state_based_score"] = s2
            res.update(self.task_status)
            json.dump(res, f, indent=4, ensure_ascii=False)
        print(f"Results saved to {file_path}")
    

if __name__ == "__main__":
    print("Running referee example...")

    print("""
    # make a env
    env = None

    # ---------- Add referee ---------
    # The referee will check the rules and give scores
    # You can modify the rules in referee/rules.json
    import os
    try:
        from ..referee.referee import Referee
    except ImportError:
        import sys
        sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../referee"))
        from referee import Referee
    referee = Referee(env.simulator.model._model, os.path.join(os.path.dirname(os.path.abspath(__file__)), "../referee/rules.json"))
    env.simulator.add_physics_callback("referee_callback", lambda: referee.update(env.simulator.data._data))

    # -----------
    try:
        env.run()
    except KeyboardInterrupt:
        print("Simulation interrupted by user.")
    finally:
        print("-" * 100)
        referee.save_results()
        print(f"Total score: /{referee.total_score/}")
        print(f"Task status:")
        for k, v in referee.task_status.items():
            print(f"  /{k/}: {v['status']} (sim_time: {v['sim_time']})")
        print("-" * 100)
    """)