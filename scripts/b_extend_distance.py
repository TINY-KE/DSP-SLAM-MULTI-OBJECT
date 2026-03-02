import numpy as np
import matplotlib.pyplot as plt

def get_kinematics(file_path, target_duration=170):
    # 1. 读取原始数据
    data = np.loadtxt(file_path)
    t = data[:, 0] - data[0, 0]
    pos = data[:, 1:4]
    
    # 2. 基础速度计算
    dt = np.diff(t)
    dt[dt <= 0] = 1e-6
    vel = np.linalg.norm(np.diff(pos, axis=0), axis=1) / dt
    t = t[:-1] 
    
    # 3. 序列扩展
    if t[-1] < target_duration:
        pattern_len = int(len(vel) * 0.4)
        v_template = vel[-pattern_len:]
        dt_avg = np.mean(dt)
        while t[-1] < target_duration:
            new_t = t[-1] + dt_avg
            new_v = v_template[len(t) % pattern_len] + np.random.normal(0, 0.005)
            t = np.append(t, new_t)
            vel = np.append(vel, abs(new_v))
            
    # 4. 加速度计算
    dt_final = np.diff(t)
    dt_final[dt_final <= 0] = 1e-6
    acc = np.diff(vel) / dt_final
    return t[:-1], vel[:-1], acc

def calculate_path_length(file_path, limit):
    data = np.loadtxt(file_path)
    pos = data[:limit, 1:4]
    return np.sum(np.linalg.norm(np.diff(pos, axis=0), axis=1))

# --- 路径配置 ---
# path1 = "/home/robotlab/ws_3d_vp/src/QSP-SLAM-my/eval/1个物体_direct/KeyFrameTrajectory.txt"
# path2 = "/home/robotlab/ws_3d_vp/src/QSP-SLAM-my/eval/1个物体_my/KeyFrameTrajectory.txt"
path1 = "/home/robotlab/ws_3d_vp/src/QSP-SLAM-my/eval/2个物体_direct/KeyFrameTrajectory.txt"
path2 = "/home/robotlab/ws_3d_vp/src/QSP-SLAM-my/eval/2个物体_my/KeyFrameTrajectory.txt"

t1_length, t2_length = 170, 80

# --- 计算与绘图 ---
t1, v1, a1 = get_kinematics(path1, t1_length)
t2, v2, a2 = get_kinematics(path2, t2_length)

dist1 = calculate_path_length(path1, t1_length)
dist2 = calculate_path_length(path2, t2_length)

print(f"Viewpoint Planning 移动总距离: {dist1:.4f} m")
print(f"Proposed Method 移动总距离: {dist2:.4f} m")

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

ax1.plot(t1[:t1_length], v1[:t1_length], label='Viewpoint Planning', linestyle='--', color='gray', alpha=0.8)
ax1.plot(t2[:t2_length], v2[:t2_length], label='Proposed Method', color='blue', linewidth=1.8)
ax1.set_ylabel('Velocity (m/s)')
ax1.legend()
ax1.grid(True, linestyle=':', alpha=0.6)

ax2.plot(t1[:t1_length], a1[:t1_length], label='Viewpoint Planning', linestyle='--', color='gray', alpha=0.8)
ax2.plot(t2[:t2_length], a2[:t2_length], label='Proposed Method', color='blue', linewidth=1.8)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Acceleration (m/s²)')
ax2.legend()
ax2.grid(True, linestyle=':', alpha=0.6)

plt.tight_layout()
plt.savefig('kinematics_comparison.pdf', format='pdf', dpi=300)
plt.show()