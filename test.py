from time import sleep

import mujoco as mj
import mujoco.viewer
import numpy as np

paused = False

def key_callback(keycode):
    if chr(keycode) == ' ':
        global paused
        paused = not paused
    elif chr(keycode) == 'p':
        print(data.qpos[:7])
        print(data.qvel[:6])

# 加载 URDF 或 MJCF
model = mj.MjModel.from_xml_path("/media/kibi/TOSHIBA_EXT/Works/Unitree/catkin_ws/src/unitree_ros/robots/g1_description/g1_29dof_with_hand_rev_1_0.xml")  # 确保 G1 机器人模型已正确转换
data = mj.MjData(model)

# help(type(data))

# 创建仿真器
viewer = mj.viewer.launch_passive(model, data, key_callback=key_callback)

num_joints = model.nu  # 获取关节数量

# target_positions = np.array(([0, 0.5, -1, 0, -0.5, 1] * 4) + [0.1] * 19)  # 设定目标关节角度
target_positions = data.qpos[:num_joints].copy()
# Kp = 10  # P 控制增益

# for i in range(model.njnt):
#     joint_name = model.joint(i).name
#     qpos_addr = model.jnt_qposadr[i]
#     qvel_addr = model.jnt_dofadr[i]
#     print(f"Joint {i}: {joint_name}")
#     print(f"  qpos index: {qpos_addr}")
#     print(f"  qvel index: {qvel_addr}")

Kp = [0] * 43
Kd = [0] * 43
Qdes = [0] * 43
DQdes = [0] * 43
tau = [0] * 43

# left_hip_pitch_joint
Kp[0] = 50
Kp[1] = 50
# right_hip_pitch_joint
Kp[6] = 50
Kp[7] = 50
Kp[3] = 50
Kp[9] = 50
Kp[12] = 50
Kp[13] = 50
# waist_pitch_joint
Kp[14] = 250
# left_elbow_joint
Kp[18] = 50
# left_wrist_pitch_joint
Kp[20] = 50
# right_elbow_joint
Kp[32] = 50
# right_wrist_pitch_joint
Kp[34] = 50

# left_hip_pitch_joint
Kd[0] = 10
Kd[1] = 10
# right_hip_pitch_joint
Kd[6] = 10
Kd[7] = 10
Kd[3] = 10
Kd[9] = 10
Kd[12] = 10
Kd[13] = 10
# waist_pitch_joint
Kd[14] = 10
# left_elbow_joint
Kd[18] = 10
# left_wrist_pitch_joint
Kp[20] = 10
# right_elbow_joint
Kd[32] = 10
# right_wrist_pitch_joint
Kp[34] = 10

while viewer.is_running():
    if not paused:
        # data.ctrl = data.qvel
        # data.ctrl[0] = Kp * (0 - data.qpos[7])
        # data.ctrl[6] = Kp * (0 - data.qpos[13])
        # data.ctrl *= 0.1

        data.ctrl = np.array(tau) + np.array(Kp) * (np.array(Qdes) - np.array(data.qpos[7:])) + np.array(Kd) * (np.array(DQdes) - np.array(data.qvel[6:]))

        # data.ctrl = target_positions
        mj.mj_inverse(model, data)
        # data.ctrl = data.qfrc_inverse[:num_joints] * 0.1
        # print(data.qpos)
        # print(data.qvel)
        # print(data.ctrl)
        # print()
        mj.mj_step(model, data)
        viewer.sync()

# while viewer.is_running():
#
#     num_joints = model.nu  # 获取关节数量
#     data.ctrl = 1 * np.ones(num_joints)  # 让所有关节以正弦波摆动
#
#     mj.mj_step(model, data)  # 进行仿真
#     viewer.sync()

viewer.close()
