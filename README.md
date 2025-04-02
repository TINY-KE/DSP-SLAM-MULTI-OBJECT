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

+ 控制相机运行circle
source ws_kinect/devel/setup.bash   &&  roslaunch sim_env test_demo.launch
rosrun sim_env circle  3.5 2 30.0  0.1   #旋转半径  相机高度  俯仰角  移动1角度需要的时间
rosrun sim_env circle  3 1.3 30.0  0.5   #旋转半径  相机高度  俯仰角  移动1角度需要的时间
 
+ 修改物体识别的物体种类
/home/robotlab/ws_3d_vp/src/QSP-SLAM-my/reconstruct/contents.py

# 第一阶段目标：实现无数据关联的多物体建图


## first commit
+ 解决pybind11 GIL锁问题
+ 运行成功dsp_slam_mono
+ 

## 实现RGBD模式下多物体的dsp建模  commit f49700af3beebcbcf35977ad7daf908e5599375b
+ 在python对应的json文件中添加
    "image": {
    "mRow": 540,
    "mCol": 960,
    "mEdge": 15
  }
+ 在LocalMapping_util.cc根据物体label选择不同的pyOptimizer，进行选择不同的deepsdf参数，从而生成不同的物体模型
+ 实现了多物体dsp模型导入，其中桌子模型是默认模型，一定要保留。  
+ 实现rgbd模式的导入。
+ Python物体识别改为读取默认根目录下的png
+ 问：  mono模式中，第一个观测到的物体，怎么用python建模
  答： 第一个物体添加到map中后，并没有建模。是这个物体第二次被观测到时（根据feature point归属确定的），设置isNew = false， 才进行dsp建模的。
+ 利用AssociateObjectsByProjection，生成地图中的多个dsp物体


## TODO:
+ 在物体关联之后，将新的feature point添加进物体中，从而改进dsp模型
  + 转到世界坐标系
  + 将点加入到std::vector<MapPoint*> points_on_object = pMO->GetMapPointsOnObject();中
  + 在每次AddMapPoints(pMP)之后，根据已有point的min和max xy，将0~maxz的点都加入pMO->GetMapPointsOnObject()中。
  + 生成的时候，物体y轴要与地面z轴重合
+ 程序偶尔会卡死，但是在debug模式下没有遇到过卡死，这是为什么？
+ 

+ 待： GetMapPointsWithinBoundingCubeToGround()
     应该与什么其他程序不要冲突呢？？
+ 待：优化物体关联，修改AssociateObjectsByProjection。暂时按照距离进行关联。
+ 待：使用单个床的rosbag，但是还是建模成为汽车
+ 待： 原物体没有classid？ auto pNewObj = new MapObject(mpCurrentKeyFrame, mpMap, class_id);






# 第二阶段：利用距离 进行数据关联


# 移除离群点的方法
  + ComputeCuboidPCA(bool updatePose)
  + void MapObject::RemoveOutliersModel()
  + 

# 改为ros后pybind报错
  + 解决方法，按照李建版本的程序，移植cmakelist
  + 可能是这一句起作用 include_directories(/usr/include/vtk-7.1)








# 第三阶段：年后，


# 准备阶段：
  + 实现gazebo绕桌子
  + 用于mask提取的本地图片： 存储到DatasetPathRoot
  + 实现了ros topic的rgbd图像输入slam
  + 

# 关闭图优化中的物体部分

# 修改yaml文件使在gazebo中稳定运行
  + 使用了TUM3的 yaml文件
  + 可视化中加上mask和bbox
  + 录制新的数据集

# mask rcnn的检测效果不好，
  + 沙发背面，无法识别
  + 黄桌子 无法识别

# commit 8f3a65a836a05f19cd04718d9c90247d747835eb  使得物体模型z轴与地面对齐，修改基于点云的初始cube估计方法
  + 修改ComputeCuboidPCA,直接将与地面之间的全部点加入
  + 启用AssociateObjects3D();  根据距离的关联
  + 只有一个物体，为什么不是所有点都参与生成物体模型
    + 关闭pMO->RemoveOutliersModel();
  + 根据GetMapPointsOnObject可视化一个cube。
  + 录制rosbag 
  rosbag record -O circle.bag /rgb/image_raw /depth_to_rgb/image_raw
  + 最终现象：可以运行，没有出现调用python计算物体位姿报错的现象

# commit 16825da7edfe1751aa8d1d4fd1f5a09803507214  初始旋转矩阵错误，绕z轴90度
  + 根据x-right,y-up,z-back，重新调整物体的初始位姿。也就是z轴从沙发的背面出去。
  + 调整RemoveOutliersSimple()中的阈值： 1改为4
  + 最终现象：可以运行，没有出现调用python计算物体位姿报错的现象
  
# 调整距离过滤的阈值
  + 每次dsp重建前都运行ComputeCuboidPCA  答:不行，虽然物体确实到了所需的中心位置，但是尺度不对。
  + 如何让它尺度正确：
    尝试修改 T.topLeftCorner(3, 3) = 0.40 * l * R;  

# commit 40eaa84d5aae6bbe67229dc46ca39f4c0dc1c814  对沙发和床有不错的建模效果
  + 之前模型效果差应该是因为：Outlier点太多，进而影响了deepsdf生成时的尺度
  + 恢复了RemoveOutliersModel()，去除了Outlier
  + 启用AssociateObjects3D();  根据距离的关联
  + 可视化的时候颜色还是有些问题，紫色点和红色北京点重合
  + 对同一物体，基于先验的cube初始化，只会进行一次。且cube的效果比较差，应该是因为
  + 

# commit a4f174d9583833cef79a22a4a155c21b33363359  RVIZ显示出椭球体
  + 椭球体是从SDF模型中提取出来的。
  + 

# 实现单个沙发_my的所需元素  commit 48e0f11514d830c3564d8d43596645d535029c5d
  + 保存sdf模型，每个物体单独保存一个文件
  + 保存world点
  + 显示相机轨迹
  + 

# 备份单个沙发的my和direct结果  commit ad2ff1cc93d3f6a030199bf06ced50dc13a9b1dc
  + 下一步：布置新的场景，核心是加个房间模型，用来防止不同物体间转移时丢失定位
  

#  预生成各个场景下物体的模型
  + 完成

# 备份： 
  + bedroom_my 完成, 对应bedroom_my_2.bag
  + livingroom_my 完成, 对应living_room_my.bag
  + bedroom_direct 完成, 对应bedroom_direct.bag
  + livingroom_direct 完成, 对应living_room_direct.bag
  + suv_my 完成, 对应suv_my.bag
  + suv_direct 完成, 对应suv_d.bag

# 实验数据后续处理：
  + 进行物体真值比较是，使用cube代表物体的真值。
  + 保存底盘真值：bedroom残缺
  + 已完成所需的数据


# todo：
  + 保存相机位姿，
  + 房间布局：
      + 床 
      + 长沙发，圆桌子
      + 床，短沙发，椅子
  + 导入李建的sdf模型
  + 最后一个困难：机器人在房间内的轨迹是否会丢失定位
  
# （未用）修改数据关联: 
  + 目前的物体关联，应该是用的AssociateObjectsByProjection
    错误反思：这里的关联是当前帧中的点云与map中物体的关联
  + 对于背景物体（床），距离小于2m，则认为是同一个物体。
    在后端中merge背景物体。
  + 在图片中显示mask，以便于debug

# todo： detection与最近物体的融合
  + 遇到问题： 物体class为56的Detection包含point较少，设置为bad。
  + sim3和se3的区别。 双目模式中，物体的初始位姿哪来的、