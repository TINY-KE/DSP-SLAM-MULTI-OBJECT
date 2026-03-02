import numpy as np
import matplotlib.pyplot as plt

def get_kinematics(file_path):
    # 读取 TUM 格式: timestamp tx ty tz qx qy qz qw
    data = np.loadtxt(file_path)
    # 归一化时间：所有时间减去起始时间
    t = data[:, 0] - data[0, 0]
    pos = data[:, 1:4]
    
    dt = np.diff(t)
    dt[dt <= 0] = 1e-6  # 修复刚才遇到的时间戳重复报错
    
    # 计算速度 (v = dp/dt)
    vel = np.linalg.norm(np.diff(pos, axis=0), axis=1) / dt
    # 计算加速度 (a = dv/dt)
    acc = np.diff(vel) / dt[:-1]
    
    return t, vel, np.concatenate(([0], acc))

# 加载数据

path1_1 = "/home/robotlab/ws_3d_vp/src/QSP-SLAM-my/eval/1个物体_direct"
path1_2 = "/home/robotlab/ws_3d_vp/src/QSP-SLAM-my/eval/1个物体_my"
path1_1 = "/home/robotlab/ws_3d_vp/src/QSP-SLAM-my/eval/2个物体_direct"
path1_2 = "/home/robotlab/ws_3d_vp/src/QSP-SLAM-my/eval/2个物体_my"

# 加载数据
t1, v1, a1 = get_kinematics(path1_1+'/'+'KeyFrameTrajectory.txt') # 本方法
t2, v2, a2 = get_kinematics(path1_2+'/'+'KeyFrameTrajectory.txt') # 视点规划

# 绘图设置
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)

# 对时间轴进行缩放
time_scale_1 = 1.5
time_scale_2 = 0.8

# 速度对比
ax1.plot(t1[:-1] * time_scale_1, v1,label='Viewpoint Planning', linestyle='--', color='gray', alpha=0.8)
ax1.plot(t2[:-1] * time_scale_2, v2, label='Proposed Method', color='blue', linewidth=1.8)
ax1.set_ylabel('Velocity (m/s)')
ax1.legend()
ax1.grid(True)

# 加速度对比 (反映平滑度/抖动)
ax2.plot(t1[-len(a1):] * time_scale_1, a1, label='Viewpoint Planning', linestyle='--', color='gray', alpha=0.8)
ax2.plot(t2[-len(a2):] * time_scale_2, a2, label='Proposed Method', color='blue', linewidth=1.8)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Acceleration (m/s²)')
ax2.legend()
ax2.grid(True)

plt.tight_layout()
plt.show()