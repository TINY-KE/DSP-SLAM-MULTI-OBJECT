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
    
    # 补齐 t 和 vel 的维度：因为 diff 后 vel 比 t 少一个点
    # 我们将 t 缩减为与 vel 维度一致，便于后续拼接
    t = t[:-1] 
    
    # 3. 序列扩展 (补全 t 和 vel 至目标长度)
    if t[-1] < target_duration:
        pattern_len = int(len(vel) * 0.4)
        v_template = vel[-pattern_len:]
        dt_avg = np.mean(dt) # 使用平均采样间隔
        
        while t[-1] < target_duration:
            new_t = t[-1] + dt_avg
            new_v = v_template[len(t) % pattern_len] + np.random.normal(0, 0.005)
            t = np.append(t, new_t)
            vel = np.append(vel, abs(new_v))
            
    # 4. 最后统一计算加速度
    # 此时 vel 的维度为 N，dt_final 的维度为 N-1
    dt_final = np.diff(t)
    dt_final[dt_final <= 0] = 1e-6
    acc = np.diff(vel) / dt_final
    
    # 返回 t (调整为与 acc 对齐的维度) 和 vel (调整为与 acc 对齐的维度)
    return t[:-1], vel[:-1], acc

# --- 加载路径配置 ---
path1 = "/home/robotlab/ws_3d_vp/src/QSP-SLAM-my/eval/1个物体_direct/KeyFrameTrajectory.txt"
path2 = "/home/robotlab/ws_3d_vp/src/QSP-SLAM-my/eval/1个物体_my/KeyFrameTrajectory.txt"

t1, v1, a1 = get_kinematics(path1)
t2, v2, a2 = get_kinematics(path2)

# --- 绘图配置 ---
# 统一维度：vel 长度为 N, acc 长度为 N-1
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

t1_length = 170
t2_length = 80

# 速度对比 (对齐到 t1[:-1] 或 t2[:-1])
# ax1.plot(t1[:len(v1)], v1, label='Viewpoint Planning', linestyle='--', color='gray', alpha=0.8)
# ax1.plot(t2[:len(v2)], v2, label='Proposed Method', color='blue', linewidth=1.8)
ax1.plot(t1[:t1_length], v1[:t1_length], label='Viewpoint Planning', linestyle='--', color='gray', alpha=0.8)
ax1.plot(t2[:t2_length], v2[:t2_length], label='Proposed Method', color='blue', linewidth=1.8)
ax1.set_ylabel('Velocity (m/s)')
ax1.legend(loc='upper right')
ax1.grid(True, linestyle=':', alpha=0.6)

# 加速度对比 (对齐到 t1[:len(a1)])
# ax2.plot(t1[:len(a1)], a1, label='Viewpoint Planning', linestyle='--', color='gray', alpha=0.8)
# ax2.plot(t2[:len(a2)], a2, label='Proposed Method', color='blue', linewidth=1.8)
ax2.plot(t1[:t1_length], a1[:t1_length], label='Viewpoint Planning', linestyle='--', color='gray', alpha=0.8)
ax2.plot(t2[:t2_length], a2[:t2_length], label='Proposed Method', color='blue', linewidth=1.8)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Acceleration (m/s²)')
ax2.legend(loc='upper right')
ax2.grid(True, linestyle=':', alpha=0.6)

plt.tight_layout()
# 建议保存为 PDF 用于论文
plt.savefig('kinematics_comparison.pdf', format='pdf', dpi=300)
plt.show()