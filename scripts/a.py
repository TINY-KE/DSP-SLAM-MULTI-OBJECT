import numpy as np
import matplotlib.pyplot as plt

def calculate_stats(file_path):
    # 读取 TUM 格式: timestamp tx ty tz qx qy qz qw
    data = np.loadtxt(file_path)
    t = data[:, 0]
    pos = data[:, 1:4]
    
    # 计算位移增量和时间增量
    dt = np.diff(t)
    dp = np.diff(pos, axis=0)
    
    # 速度 v = dp / dt
    # 处理可能的 dt=0 情况 (之前遇到的错误)
    dt[dt == 0] = 1e-6 
    
    vel = np.linalg.norm(dp, axis=1) / dt
    return t[:-1], vel

path1_1 = "/home/robotlab/ws_3d_vp/src/QSP-SLAM-my/eval/1个物体_direct"
path1_2 = "/home/robotlab/ws_3d_vp/src/QSP-SLAM-my/eval/1个物体_my"

# 加载数据
t1, vel1 = calculate_stats(path1_1+'/'+'KeyFrameTrajectory.txt')
t2, vel2 = calculate_stats(path1_2+'/'+'KeyFrameTrajectory.txt')

# 绘图
plt.figure(figsize=(8, 4))
plt.plot(t1, vel1, label='GT', linewidth=1.5)
plt.plot(t2, vel2, label='MY', linestyle='--', alpha=0.7)
plt.xlabel('Time [s]')
plt.ylabel('Velocity [m/s]')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()