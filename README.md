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
+ 物体识别的成功率很低，很多物体识别不到  答：是不是因为关键帧太少了，应该每一帧都检测

+ 获取物体检测框   答：已实现
+ 获取点云     答：已实现
+ 多帧生成椭球   答：已实现

+ 实现可以物体检测每一帧
    + 和李建用的是同一个conda环境，问题在哪？
    + 将李建的python移植过来
    + 是不是我的cpp中过滤掉了呢？ 似乎确实是的

+ 可视化物体的深度点云
    + ExtractPointCloud提取点云
    + 以上函数中，VisualizePointCloud可视化点云
    + 通过mpMap->AddPointCloudLis，添加到mmPointCloudLists中
    + 降低Viewer的频率
    + 利用drawPointCloudLists，绘制mmPointCloudLists
    + 用很多点在原点，LJ程序中是不是也有很多point离相机特别近（错误定位）
    + 关闭单帧点的可视化，先完成椭球体融合,【会不会过滤后的点是可以可视化的？】
    + 要使用曼哈顿平面，得先进行TaskManhattanPlanes  和  extractManhattanPlanes
    + 注意TaskManhattanPlanes中 g2o::plane local_ground = mGroundPlane，要转换为相机坐标系下的平面；
    + 目前判断：单帧中深度点云是严重不够的，必须多帧融合；
    + 多帧融合位置：？？？
    + 在AssociateObjectsByProjection中通过投影，进行关联
    + 在UpdateObjectsToMap()中更新地图中的椭球体
    + 在哪进行椭球体的融合：
    + 展示椭球体的融合
    + Tracking::DenseBuild()中添加了当前帧的深度点云，有问题
    + cv::Mat I = frame->frame_img;   // U16C1 , ushort
    + 为什么是片状的 ??
        + 
    + 写一个程序展示读取后的深度点云

+ 椭球体的融合
    + 

+ 可视化当前帧中的椭球体  答：在pangolin中实现
+ 多帧联合优化

+ 添加回来 VisualizeRelations和



# 第五阶段  实现椭球体的二维椭圆投影和隐式形状的二维掩码投影


