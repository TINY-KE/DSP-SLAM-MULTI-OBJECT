# 运行指令
+ 单目
./dsp_slam_mono Vocabulary/ORBvoc.bin configs/freiburg_001.yaml data/freiburg/001 map/freiburg/001
./dsp_slam_mono Vocabulary/ORBvoc.bin configs/freiburg_001.yaml /media/robotlab/新加卷/ubuntu22/DSP-SLAM/data/freiburg_cars/Car001 map/freiburg/001

+ rgbd
./dsp_slam_rgbd Vocabulary/ORBvoc.bin configs/self_allobject_ground.yaml /media/robotlab/新加卷/ubuntu22/QSP-SLAM-all/MySimDataset/GroundObjects /media/robotlab/新加卷/ubuntu22/QSP-SLAM-all/MySimDataset/GroundObjects/associate.txt map/self/GroundObjects

+ 多车辆：
  


# 第一阶段目标：实现无数据关联的多物体建图


## first commit
+ 解决pybind11 GIL锁问题
+ 运行成功dsp_slam_mono
+ 

## 实现RGBD模式下多物体的dsp建模
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

# 

+ 待： 添加地面
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