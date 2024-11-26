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

## 
+ 在python对应的json文件中添加
    "image": {
    "mRow": 540,
    "mCol": 960,
    "mEdge": 15
  }
+ 
+ 使用单个床的rosbag，但是还是建模成为汽车

+ 实现多个物体（为了debug， 第一个是床， 第二个沙发）


# 第二阶段：利用距离 进行数据关联