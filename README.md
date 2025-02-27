# 运行指令 [要记得在yaml中修改DatasetPathRoot]
+ 单目
./dsp_slam_mono Vocabulary/ORBvoc.bin configs/freiburg_001.yaml data/freiburg_cars/Car001 map/freiburg/001
./dsp_slam_mono Vocabulary/ORBvoc.bin configs/freiburg_001.yaml /media/robotlab/新加卷/ubuntu22/DSP-SLAM/data/freiburg_cars/Car001 map/freiburg/001

+ rgbd
./dsp_slam_rgbd Vocabulary/ORBvoc.bin configs/self_allobject_ground.yaml /media/robotlab/新加卷/ubuntu22/QSP-SLAM-all/MySimDataset/GroundObjects /media/robotlab/新加卷/ubuntu22/QSP-SLAM-all/MySimDataset/GroundObjects/associate.txt map/self/GroundObjects

转移到本地硬盘：
./dsp_slam_rgbd Vocabulary/ORBvoc.bin configs/self_allobject_ground.yaml /home/robotlab/ws_3d_vp/src/QSP-SLAM-my/data/MySimDataset/GroundObjects /home/robotlab/ws_3d_vp/src/QSP-SLAM-my/data/MySimDataset/GroundObjects/associate.txt map/self/GroundObjects

+ 多车辆：
+ ros:
./dsp_slam_ros Vocabulary/ORBvoc.bin configs/self_allobject_ground.yaml /home/robotlab/ws_3d_vp/src/QSP-SLAM-my/data/MySimDataset/GroundObjects map/self/GroundObjects
  


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


## 
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



# 添加椭球体
+ UpdateObjectObservation_GenerateEllipsoid


+ lj这里的用意是什么？
if (keep_raw_pose) {
            cout << "Draw Sim3Two_raw " << endl;
            pMO->UpdateReconstruction(Sim3Two_raw, code);
        }
        else {
            pMO->UpdateReconstruction(Sim3Two, code);
            
        }






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

# 使得物体模型z轴与地面对齐
  + 修改ComputeCuboidPCA,直接将与地面之间的全部点加入
  + 启用AssociateObjects3D();  根据距离的关联

  + 只有一个物体，为什么不是所有点都参与生成物体模型
    + 关闭pMO->RemoveOutliersModel();
  + 根据GetMapPointsOnObject可视化一个cube。
  + 录制rosbag 
  rosbag record -O circle.bag /rgb/image_raw /depth_to_rgb/image_raw
  +

# todo： 初始旋转矩阵错误，绕z轴90度
  + 初始旋转矩阵错误，绕z轴90度
  

# （未用）修改数据关联: 
  + 目前的物体关联，应该是用的AssociateObjectsByProjection
    错误反思：这里的关联是当前帧中的点云与map中物体的关联
  + 对于背景物体（床），距离小于2m，则认为是同一个物体。
    在后端中merge背景物体。
  + 在图片中显示mask，以便于debug

