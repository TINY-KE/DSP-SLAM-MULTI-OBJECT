import numpy as np

# 1. 路径设置
input_path = "/home/robotlab/ws_ellipsoid_dsp/src/DSP-SLAM-MULTI-OBJECT/src_ros/eval/show/points_ori.txt"
output_path = "/home/robotlab/ws_ellipsoid_dsp/src/DSP-SLAM-MULTI-OBJECT/src_ros/eval/show/points.txt"

# 2. 定义 2 度旋转矩阵 (Z轴)
theta = np.radians(-1)
c, s = np.cos(theta), np.sin(theta)
R_z = np.array([
    [c, -s, 0],
    [s,  c, 0],
    [0,  0, 1]
])

# 3. 读取、转换并保存
try:
    points = np.loadtxt(input_path)
    rotated_points = points @ R_z.T
    np.savetxt(output_path, rotated_points, fmt='%.6f')
    print(f"成功！已保存至: {output_path}")
except Exception as e:
    print(f"发生错误: {e}")