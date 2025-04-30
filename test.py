from time import sleep

import mujoco as mj
import mujoco.viewer
import numpy as np

from scipy.spatial.transform import Rotation as R

paused = False

def key_callback(keycode):
    if chr(keycode) == ' ':
        global paused
        paused = not paused
    elif chr(keycode) == 'p':
        print(data.qpos[:7])
        print(data.qvel[:6])

# 加载 URDF 或 MJCF
model = mj.MjModel.from_xml_path("/media/kibi/TOSHIBA_EXT/Works/Unitree/unitree-rl/unitree_rl_gym/resources/robots/g1_description/g1_29dof_with_hand_rev_1_0.xml")  # 确保 G1 机器人模型已正确转换
data = mj.MjData(model)

# help(type(data))

# 创建仿真器
viewer = mj.viewer.launch_passive(model, data, key_callback=key_callback)

num_joints = model.nu  # 获取关节数量

# target_positions = np.array(([0, 0.5, -1, 0, -0.5, 1] * 4) + [0.1] * 19)  # 设定目标关节角度
target_positions = data.qpos[:num_joints].copy()
# Kp = 10  # P 控制增益



Kp = [10] * 43
Kd = [1] * 43
Qdes = [0] * 43
DQdes = [0] * 43
tau = [0] * 43

Qdes = [-0.1,0,0,0.3,-0.2,0,-0.1,0,0,0.3,-0.2,0,0,0,0,-0.09432,1.676,-0.10472,-0.654375,-0.03944,-0.9684,0.50034,0,0.453614,0,-0.54985,-1.17788,-0.573415,-1.19533,-0.32468,-1.5416,0.47124,-1.047,-0.0986,-0.37122,-0.5649,-0.07329,-0.568749,0,0.61269,0.968475,0.64411,0.9772]
# Qdes[15:] = [0] * (43 - 15)
Qdes[14] = 0
Qdes[3] = Qdes[9] = 0.6
Qdes[1] = 0.05
# Qdes[0] = Qdes[6] = -0.2
# Qdes[18] = Qdes[32] = 1.56


# left_hip_pitch_joint
Kp[0] = 50
Kp[1] = 50
# right_hip_pitch_joint
Kp[6] = 50
Kp[7] = 50
Kp[3] = 50
Kp[9] = 50

# ankle
Kp[4] = 40
Kp[5] = 40

# ankle
Kp[10] = 40
Kp[11] = 40

Kp[12] = 50
Kp[13] = 50
# waist_pitch_joint
Kp[14] = 250
# left_elbow_joint
Kp[18] = 50
# left_wrist_pitch_joint
Kp[20] = 50

# Kp[22:29] = [0.01] * 7

# right_elbow_joint
Kp[32] = 50
# right_wrist_pitch_joint
Kp[34] = 50

# Kp[36:43] = [0.01] * 7

# left_hip_pitch_joint
Kd[0] = 10
Kd[1] = 10
# right_hip_pitch_joint
Kd[6] = 10
Kd[7] = 10
Kd[3] = 10
Kd[9] = 10

# ankle
Kd[4] = 2
Kd[5] = 2

Kd[12] = 10
Kd[13] = 10
# waist_pitch_joint
Kd[14] = 10

# ankle
Kd[10] = 2
Kd[11] = 2

# left_elbow_joint
Kd[18] = 10
# left_wrist_pitch_joint
Kd[20] = 10

# Kd[21:28] = [0] * 7

# right_elbow_joint
Kd[32] = 10
# right_wrist_pitch_joint
Kd[34] = 10

# Kd[35:42] = [0] * 7

# Kp = [0] * 43
# Kd = [0] * 43
# Qdes = [0] * 43
# DQdes = [0] * 43
# tau = [0] * 43



Kp = [
    # 腿部（索引1-12）
    100, 100, 100, 150, 40, 40,  # 左腿：hip_pitch(1), hip_roll(2), hip_yaw(3), knee(4), ankle_pitch(5), ankle_roll(6)
    100, 100, 100, 150, 40, 40,  # 右腿：hip_pitch(7), hip_roll(8), hip_yaw(9), knee(10), ankle_pitch(11), ankle_roll(12)

    # 腰部（索引13-15）
    150, 200, 180,                # waist_yaw(13), waist_roll(14), waist_pitch(15)

    # 左臂（索引16-22）
    100, 100, 100,                # shoulder_pitch(16), shoulder_roll(17), shoulder_yaw(18)
    150, 100, 100, 100,           # elbow(19), wrist_roll(20), wrist_pitch(21), wrist_yaw(22)

    # 左手（索引23-29）
    50, 50, 50, 50, 50, 50, 50,  # thumb_0(23), thumb_1(24), thumb_2(25), middle_0(26), middle_1(27), index_0(28), index_1(29)

    # 右臂（索引30-36）
    100, 100, 100,                # shoulder_pitch(30), shoulder_roll(31), shoulder_yaw(32)
    150, 100, 100, 100,           # elbow(33), wrist_roll(34), wrist_pitch(35), wrist_yaw(36)

    # 右手（索引37-43）
    50, 50, 50, 50, 50, 50, 50   # thumb_0(37), thumb_1(38), thumb_2(39), middle_0(40), middle_1(41), index_0(42), index_1(43)
]

Kd = [
    # 腿部
    2, 2, 2, 4, 2, 2,
    2, 2, 2, 4, 2, 2,

    # 腰部
    3, 4, 3.5,

    # 左臂
    4, 4, 4,
    3, 2, 2, 2,

    # 左手
    1, 1, 1, 1, 1, 1, 1,

    # 右臂
    4, 4, 4,
    3, 2, 2, 2,

    # 右手
    1, 1, 1, 1, 1, 1, 1
]

Kp[:12] = [100, 100, 100, 150, 80, 80, 100, 100, 100, 150, 80, 80]
Kd[:12] = [2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2]

# data.qpos = [0,0,0.793,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-0.09432,1.676,-0.10472,-0.654375,-0.03944,-0.9684,0.50034,0,0.453614,0,-0.54985,-1.17788,-0.573415,-1.19533,-0.32468,-1.5416,0.47124,-1.047,-0.0986,-0.37122,-0.5649,-0.07329,-0.568749,0,0.61269,0.968475,0.64411,0.9772]

# for i in range(1, model.njnt):
#     joint_name = model.joint(i).name
#     qpos_addr = model.jnt_qposadr[i]
#     qvel_addr = model.jnt_dofadr[i]
#     print(f"Joint {i}: {joint_name}")
#     print(f"  qpos index: {qpos_addr}")
#     print(f"  qvel index: {qvel_addr}")
#     print(f"  qpos value: {Qdes[i - 1]}")

for i in range(60):
    data.ctrl = np.array(tau) + np.array(Kp) * (np.array(Qdes) - np.array(data.qpos[7:])) + np.array(Kd) * (np.array(DQdes) - np.array(data.qvel[6:]))
    data.ctrl[15:] = [0] * (43 - 15)
    mj.mj_step(model, data)
    viewer.sync()

while viewer.is_running():
    if not paused:
        # data.ctrl = data.qvel
        # data.ctrl[0] = Kp * (0 - data.qpos[7])
        # data.ctrl[6] = Kp * (0 - data.qpos[13])
        # data.ctrl *= 0.1
        # data.qpos = [0,0,0.793,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-0.09432,1.676,-0.10472,-0.654375,-0.03944,-0.9684,0.50034,0,0.453614,0,-0.54985,-1.17788,-0.573415,-1.19533,-0.32468,-1.5416,0.47124,-1.047,-0.0986,-0.37122,-0.5649,-0.07329,-0.568749,0,0.61269,0.968475,0.64411,0.9772]
        # Qdes[14] = -data.qvel[0] * 0.1


        qw, qx, qy, qz = data.qpos[3], data.qpos[4], data.qpos[5], data.qpos[6]
        quat = [qw, qx, qy, qz]
        r = R.from_quat([qx, qy, qz, qw])  # 注意顺序是 [x, y, z, w]
        roll, pitch, yaw = r.as_euler('xyz', degrees=True)

        # Qdes[0] = Qdes[6] = -0.2 + pitch
        # Qdes[14] = - pitch * 0.02 - data.qvel[4] * 0.04
        # Qdes[3] = Qdes[9] = 0.72 + pitch * 0.035
        # Qdes[0] = Qdes[6] = -0.2 - (data.qpos[11] + 0.2) * 0.1 + Qdes[14] * 0.1 + pitch * 0.01
        Qdes[3] = Qdes[9] = 0.7 + pitch * 0.055 + (data.qpos[11] + data.qpos[17] + 0.4) / 2 * -0.15
        # Qdes[3] = Qdes[9] = 0.72
        # Qdes[4] = Qdes[10] = 0
        # Qdes[14] -= data.qvel[4] * 0.003

        data.ctrl = np.array(tau) + np.array(Kp) * (np.array(Qdes) - np.array(data.qpos[7:])) + np.array(Kd) * (np.array(DQdes) - np.array(data.qvel[6:]))


        # print(data.qvel[16:18], data.qvel[16:18] * 2 + data.qpos[17:19] * 40, data.ctrl[10:12])

        # data.ctrl[4] = 0
        # data.ctrl[5] = 0

        # data.ctrl = target_positions
        # mj.mj_inverse(model, data)
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
