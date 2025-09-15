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
    + 写一个程序展示读取后的深度点云

+ global点云可视化
    + 设置过滤倍数
    + 图片和local点如何同时显示
    + 画出地面

+ 椭球体的融合
    + 显示每一帧的椭球体观测平面
    + 多针融合
    + 提取曼哈顿平面
    + 

+ 利用椭球体设置物体位姿
    + else if(mnComputeCuboidType==3) 没起作用
    + pMap->addPlane(pPlane, visual_group); 曼哈顿平面的分组没修改
    + 读取以下lj程序的mh平面的数量
    [debug] Tracking::RefineObjectsWithRelations 1, 共有 1 个支撑关系
    + *pcd_ptr = *pCloudPCL;  !!!!  这是什么问题？？？？
    + 要不要把点云添加到观测中
        // if (pcd_ptr_of_frame!=NULL) {
                //     std::cout << "  - !!! det->setPcdPtr(pcd_ptr_of_frame);" << std::endl;
                //     det->setPcdPtr(pcd_ptr_of_frame);
                // }

+ 如何使用曼哈顿平面
    + mbOpenMHPlanesFilter = false;  //zhjd: 暂时使用地面进行过滤，之后可以改为使用曼哈顿平面
    + 前一，TaskManhattanPlanes(ORB_SLAM2::Frame *pFrame)
    + 添加 SetManhattanPlanes(, 开启mbOpenMHPlanesFilter， 激活ApplyMHPlanesFilter
    + lj没开启Plane.ManhattanPlane.Open

+   mvpPotentialMHPlanes与
    mvpDominantStructuralMHPlanes的区别：
    mvpPotentialStructuralMHPlanes

+ 测试新程序有没有问题
    + 看看曼哈顿平面作为支撑平面，是否运行成功
+ 

+ 可视化当前帧中的椭球体  答：在pangolin中实现
+ 多帧联合优化

+ 添加回来 VisualizeRelations和

+ mpEllipsoidExtractor->OpenSymmetry(); 对成型用到了吗
    + 似乎这个在lzw中，就被废弃了

+ EstimateLocalEllipsoidUsingMultiPlanes中，为什么地平面不起到作用
    + 答案：只使用点云生成，并没有用到地平面
    + 

+ 研究明白为什么

+ 观察李建的可视化结果中，为什么地面和bbox平面没有起作用？ 有优化吗
    + 蓝色平面和绿色平面的区别
    + 
    + 
    
+ 既然dsp本身就有物体与相机之间的优化，那么只需要做好两件事
    + 数据关联：数据关联用李建的椭球体projection
        + 利用原本的AssociateObjectsByProjection
        + 单帧观测的存储形式为 pLocalEllipsoidOneFrame
    + 存储数据关联：
        + 还是存储在ObjectDetection中
    + 优化： 将object中的椭球体与Observations相关联，使用OptimizeWithDataAssociationUsingMultiplanes()
        + lzw三类优化的区别是什么？  
    + 优化结果的回填
        + 直接修改全局物体中的椭球体

+ 使用椭球体筛选点云，并生成dsp物体
    + 先不使用联合优化，
        + addEllipsoidVisual和AddMapObject不同
        + 问题就在于，利用椭球体投影进行数据关联，有问题。debug
        + AssociateObjectWithEllipsold 失效的原因，在于没有椭球体的话，则无法使用椭球体尽心关联，因此物体内点的数量，就不够
        + SetPoseByEllipsoid失败的原因是track和localmapping不同步，这也是为什么lj要将两者强行同步的原因
            + 在点云融合之后，将之前的点云变换到当前坐标系，再直接利用EstimateLocalEllipsoidUsingMultiPlanes生成一个椭球体
            + 同时这些点云还能用于生辰dsp物体
            + 而且要关闭前端的椭球体提取，tracker中只做数据关联

+ 问题：
    + 不过滤离边界近的就会导致有错误的椭球体，但过滤的话，有会导致椭球体生成的太慢
    + 物体的pose为什么会中途改变？？ 应该设置永远是朝上的
    + 用深度点云生成物体
    + [完]要检查y轴是否朝上？？
    + 物体生成的有问题，第一个沙发生成的完全歪了或者完全无法生成，第一个椅子没有生成物体
        + 统计一下有多少物体完成SetPoseByEllipsoid了，但没有reconstructed成功
        + 我觉得是因为数据关联不好用，导致物体内部的点云太少，
        + 我觉得：用深度点云生成物体，应该可以解决这个问题
    + 物体和墙面不垂直，如果可以通过墙面调整SetPoseByEllipsoid，



# 第五阶段 画图

+ 竖直的曼哈顿平面
    + 修改extractManhattanPlanes程序
    + 以50图片，b所有点云，c四个平面（修改extractPlanes），d挑选后的曼哈顿平面    
    + 修改地面颜色 
        + 改了
    + 挑选最佳竖直平面和水平平面
        + 
    + 没有物体，所以无法挑选MHP
    + 为什么前100帧，生成了两个椭球体，但是却没有加入到global中
    + 在数据关联中加上，椭球体在三维空间中的距离判断

+ 第2个沙发为什么加入map很慢 
    + 已解决
+ 为什么会有scale很小的椭球体在mvpGlobalEllipsolds中
    + 在track中过滤一下
    + 加上距离的数据关联
    
# relations提取
+ 当前问题:
    + 倚靠平面为什么总是沙发的前面
        + 已解决，椭球体的六个平面方向量是指向物体外，
    + 地面不是手动加入到vplanes里面了吗，  为什么不生效
        + 不解决了
    
    + 基于MHP优化椭球体，并用金色椭球体展示
        + OptimizeEllipsoidWithBboxPlanesAndMHPlanes 函数怎么使用？
        + MHP也考虑方向
            double dis;
            if(mbNormalDirection)
                dis = GetDistanceWithDirection(pl, e);      // 若开启了则考虑其方向
            else 
                dis = distanceFromPlaneToEllipsoid(pl, e);
        + if(angle_norm_z < M_PI/180.0 * 30)
        + 直接将墙面和地面的约束分开
        + 

    + 支撑平面也得加上基于corner的判断，太远的不能留
        + TODO: 
        
# 第六阶段 椭球体节点约束
    + 如何实现

# 第六阶段  联合优化中加上relations平面（水平面和垂直面）
