# 常用指令 [要记得在yaml中修改DatasetPathRoot]
+ 单目
./dsp_slam_mono Vocabulary/ORBvoc.bin configs/freiburg_001.yaml data/freiburg_cars/Car001 map/freiburg/001
./dsp_slam_mono Vocabulary/ORBvoc.bin configs/freiburg_001.yaml /media/robotlab/新加卷/ubuntu22/DSP-SLAM/data/freiburg_cars/Car001 map/freiburg/001

+ rgbd
./dsp_slam_rgbd Vocabulary/ORBvoc.bin configs/self_allobject_ground.yaml /media/robotlab/新加卷/ubuntu22/QSP-SLAM-all/MySimDataset/GroundObjects /media/robotlab/新加卷/ubuntu22/QSP-SLAM-all/MySimDataset/GroundObjects/associate.txt map/self/GroundObjects

转移到本地硬盘：
./dsp_slam_rgbd Vocabulary/ORBvoc.bin configs/self_allobject_ground.yaml /home/robotlab/ws_3d_vp/src/QSP-SLAM-my/data/MySimDataset/GroundObjects /home/robotlab/ws_3d_vp/src/QSP-SLAM-my/data/MySimDataset/GroundObjects/associate.txt map/self/GroundObjects

+ 多车辆：

+ ros版slam:
./dsp_slam_ros Vocabulary/ORBvoc.bin configs/self_allobject_ground_ros.yaml /home/robotlab/ws_3d_vp/src/QSP-SLAM-my/data/MySimDataset/GroundObjects map/self/GroundObjects
  
+ 录制rosbag
rosbag record -O circle_bed-moveitvp-1***.bag /rgb/image_raw /depth_to_rgb/image_raw  /tf  /joint_states
rosbag record -O circle-***-3.5-2-30.bag /rgb/image_raw /depth_to_rgb/image_raw  /tf  /joint_states
rosbag record -O realhoom.bag /rgb/image_raw /depth_to_rgb/image_raw  /tf  /scan

+ 启动建模单个物体的gazebo环境
source ws_kinect/devel/setup.bash   &&  roslaunch sim_env test_demo.launch
+ 控制相机运行circle
rosrun sim_env circle  3.5 1.3 30.0  0.1   #旋转半径  相机高度  俯仰角  移动1角度需要的时间
rosrun sim_env circle  3 1.3 30.0  0.5   #旋转半径  相机高度  俯仰角  移动1角度需要的时间
 
+ 修改物体识别的物体种类
/home/robotlab/ws_3d_vp/src/QSP-SLAM-my/reconstruct/contents.py

# pybind11的报错
pybind11::handle::dec_ref() is being called while the GIL is either not held or invalid.
解决方法：在LocalMapping.h中添加了 #define PYBIND11_NO_ASSERT_GIL_HELD_INCREF_DECREF


# 第四阶段  完成EDSPslam的椭球体多帧优化
+ 计算地面
    使用SetGroundPlaneMannually(，设置默认值为0，0，1

+ 运行icl数据集

+ 为什么要用到json中的"slam_config_path"
+ 修改为icl数据集的读取本地？  答：实现根据association.txt读取图片
+ 相机第一帧的位姿 怎么设置？  答：修改了相机内参。
+ 单帧处理时间很长  答：main函数中加了delay，已删除
+ 为什么无法生成dsp物体i 答：数据集中图片太少，降低对图片的帧数控制
+ 物体识别的成功率很低，很多物体识别不到  答：待解决

+ 如何使用物体检测框
    + 获取框
    + 生成平面
+ 多帧生成椭球





# 第五阶段  实现椭球体的二维椭圆投影和隐式形状的二维掩码投影
