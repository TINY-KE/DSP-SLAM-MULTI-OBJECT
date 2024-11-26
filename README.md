# 运行指令
+ 单目
./dsp_slam_mono Vocabulary/ORBvoc.bin configs/freiburg_001.yaml data/freiburg/001 map/freiburg/001
./dsp_slam_mono Vocabulary/ORBvoc.bin configs/freiburg_001.yaml /media/robotlab/新加卷/ubuntu22/DSP-SLAM/data/freiburg_cars/Car001 map/freiburg/001

+ 多车辆：
  


# 第一阶段目标：实现无数据关联的多物体建图


## first commit
+ 解决pybind11 GIL锁问题
+ 运行成功dsp_slam_mono
+ 


# 第二阶段：利用距离 进行数据关联