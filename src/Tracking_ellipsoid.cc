/**
* This file is part of https://github.com/JingwenWang95/DSP-SLAM
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

// #include <ConstrainPlane.h>

#include "Tracking.h"
#include "ObjectDetection.h"
#include "ORBmatcher.h"
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

using namespace std;

namespace ORB_SLAM2 {

    // // ZHJD 移植
    // Matrix3Xd generateProjectionMatrix(const SE3Quat &campose_cw, const Matrix3d &Kalib) {
    //     Matrix3Xd identity_lefttop;
    //     identity_lefttop.resize(3, 4);
    //     identity_lefttop.col(3) = Vector3d(0, 0, 0);
    //     identity_lefttop.topLeftCorner<3, 3>() = Matrix3d::Identity(3, 3);

    //     Matrix3Xd proj_mat = Kalib * identity_lefttop;

    //     proj_mat = proj_mat * campose_cw.to_homogeneous_matrix();

    //     return proj_mat;
    // }

    // MatrixXd fromDetectionsToLines(Vector4d &detections) {
    //     bool flag_openFilter = false; // filter those lines lying on the image boundary

    //     double x1 = detections(0);
    //     double y1 = detections(1);
    //     double x2 = detections(2);
    //     double y2 = detections(3);

    //     Vector3d line1(1, 0, -x1);
    //     Vector3d line2(0, 1, -y1);
    //     Vector3d line3(1, 0, -x2);
    //     Vector3d line4(0, 1, -y2);

    //     // those lying on the image boundary have been marked -1
    //     MatrixXd line_selected(3, 0);
    //     MatrixXd line_selected_none(3, 0);

    //     int config_border_pixel = 10;
    //     int miImageCols = Config::Get<int>("Camera.width");
    //     int miImageRows = Config::Get<int>("Camera.height");
    //     if (!flag_openFilter || (x1 > config_border_pixel && x1 < miImageCols - config_border_pixel)) {
    //         line_selected.conservativeResize(3, line_selected.cols() + 1);
    //         line_selected.col(line_selected.cols() - 1) = line1;
    //     }
    //     if (!flag_openFilter || (y1 > config_border_pixel && y1 < miImageRows - config_border_pixel)) {
    //         line_selected.conservativeResize(3, line_selected.cols() + 1);
    //         line_selected.col(line_selected.cols() - 1) = line2;
    //     }
    //     if (!flag_openFilter || (x2 > config_border_pixel && x2 < miImageCols - config_border_pixel)) {
    //         line_selected.conservativeResize(3, line_selected.cols() + 1);
    //         line_selected.col(line_selected.cols() - 1) = line3;
    //     }
    //     if (!flag_openFilter || (y2 > config_border_pixel && y2 < miImageRows - config_border_pixel)) {
    //         line_selected.conservativeResize(3, line_selected.cols() + 1);
    //         line_selected.col(line_selected.cols() - 1) = line4;
    //     }

    //     return line_selected;
    // }

    // MatrixXd GenerateBboxPlanes(g2o::SE3Quat &campose_wc, Eigen::Vector4d &bbox, Matrix3d &calib) {
    //     MatrixXd planes_all(4, 0);
    //     // std::cout << " [debug] calib : \n " << calib << std::endl;
    //     // get projection matrix
    //     MatrixXd P = generateProjectionMatrix(campose_wc.inverse(), calib);

    //     MatrixXd lines = fromDetectionsToLines(bbox);
    //     MatrixXd planes = P.transpose() * lines;

    //     // add to matrix
    //     for (int m = 0; m < planes.cols(); m++) {
    //         planes_all.conservativeResize(planes_all.rows(), planes_all.cols() + 1);
    //         planes_all.col(planes_all.cols() - 1) = planes.col(m);
    //     }

    //     return planes_all;
    // }

    // // 新版本则基于bbox生成，不再与RGBD版本有任何关联
    // // replace_detection: 是否将推测物体放入 pFrame 中生效。
    // void Tracking::InferObjectsWithSemanticPrior(Frame* pFrame, bool use_input_pri = true, bool replace_detection = false)
    // {
    //     // 要求有地平面估计再启动
    //     if(miGroundPlaneState != 2)
    //     {
    //         std::cout << "Close Infering, as the groundplane is not set." << std::endl;
    //         return;
    //     }

    //     Pri pri = Pri(1,1);
    //     pri.print();

    //     // ********* 测试1： 所有先验都是 1:1:1 *********
    //     // // 读取 pri;  调试模式从全局 Config 中读取
    //     double weight = Config::ReadValue<double>("SemanticPrior.Weight");
    //     std::cout << "weight:"<<weight << std::endl;
    //     std::cout << "Begin infering ... " << std::endl;

    //     // 对于帧内每个物体，做推断，并可视化新的物体
    //     auto& meas = pFrame->meas;
    //     int meas_num = meas.size();

    //     if(replace_detection) {
    //         pFrame->mpLocalObjects.clear();
    //         pFrame->mpLocalObjects.resize(meas_num);
    //     }
    //     for(int i=0;i<meas_num;i++)
    //     {
    //         Measurement& m = meas[i];
    //         Vector4d bbox = m.ob_2d.bbox;
    //         // 检测bbox是否正确
    //         // int x1 = (int)(bbox(0)), y1 = (int)(bbox(1)), \
    //         // x2 = (int)(bbox(2)), y2 = (int)(bbox(3));
    //         // std::cout<< " [zhjd-debug] bbox: " << "x1:"<<x1 
    //         // << ", y1:" << y1 << ", x2:" << x2 << ", y2:" << y2
    //         // << std::endl;

    //         // Check : 确保该物体类型是在地面之上的
    //         // if(!CheckLabelOnGround(m.ob_2d.label)) continue;

    //         // Check : 该 bbox 不在边缘
    //         // bool is_border = calibrateMeasurement(bbox, mRows, mCols, Config::Get<int>("Measurement.Border.Pixels"), Config::Get<int>("Measurement.LengthLimit.Pixels"));
    //         // if(is_border) continue;

    //         // 生成Pri
    //         Pri pri = Pri(1,1);

    //         std::cout << "Pri for label : " << m.ob_2d.label << std::endl;
    //         pri.print();

    //         // RGB_D + Prior
    //         g2o::plane ground_pl_local = mGroundPlane;   //世界坐标系下的
    //         ground_pl_local.transform(pFrame->cam_pose_Tcw);  // 相机坐标系下的
    //         std:;cout<<"[InferObjectsWithSemanticPrior] 0 地平面： world参数:"<< mGroundPlane.param.transpose()
    //         << " local参数:" << ground_pl_local.param.transpose() <<std::endl;
            
    //         priorInfer pi(mRows, mCols, mCalib);

    //         // *********************************
    //         // 生成一个新的 Initguess
    //         // *********************************
    //         std::cout<<"[InferObjectsWithSemanticPrior] 1 准备 初始化一个椭球体"<<std::endl;
    //         std::cout << "LastCost: " << pi.GetLastCost() << std::endl;
            
    //         // pi.GenerateInitGuess(bbox, ground_pl_local.param);
    //         // g2o::ellipsoid e_init_guess = pi.GenerateInitGuess(bbox, ground_pl_local.param);
    //         int debug_init_guess = Config::ReadValue<double>("OptimizeEllipsoidWithMultiPlanes.debug_init_guess");

    //         if(debug_init_guess==5){
                
    //             g2o::ellipsoid e_init_guess = pi.GenerateInitGuess(bbox, ground_pl_local.param);
              
    //             // 如果已经有了椭球体，则从e_init_guess中获取plane in camera， 转为plane in world后，添加到map中的物体中
    //             auto MapObjects = mpMap->GetAllEllipsoidsVisual();
    //             if( ! MapObjects.empty()){
    //                 // 还需要传入 bbox. x4
    //                 int bbox_planes_num = e_init_guess.mvCPlanesInCamera.size();
    //                 // std::vector<g2o::plane> planes_bbox; planes_bbox.resize(bbox_planes_num);
    //                 // 只存储左右两面，也就是i=0,i=2
    //                 for(int i=0;i<bbox_planes_num;i+=2){
    //                     auto pCP = e_init_guess.mvCPlanesInCamera[i];
    //                     if (pCP && pCP->pPlane) {
    //                             g2o::plane* pl = new g2o::plane(pCP->pPlane->param);
    //                             pl->transform(pFrame->cam_pose_Twc);

    //                             // 与所有平面比较，如果有相似的，则不添加
    //                             Vector3d normVec = pl->param.head(3);  // 当前平面的法向量
    //                             bool similar_found = false;  // 标记是否找到相似的平面

    //                             // 与 MapObjects[0] 中现有的平面比较法向量
    //                             double angle_thresh = M_PI / 180 * 20; // 10 deg
    //                             for (auto& existingPlane : MapObjects[0]->GetPlanes()) {
    //                                 Vector3d existingNormVec = existingPlane->param.head(3);  // 现有平面的法向量
                                    
    //                                 // 计算法向量之间的夹角
    //                                 double cos_angle = normVec.dot(existingNormVec) / (normVec.norm() * existingNormVec.norm());
    //                                 double angle = acos(cos_angle);  // 夹角

    //                                 if (std::abs(angle) < angle_thresh || std::abs(M_PI - angle) < angle_thresh) {
    //                                     similar_found = true;
    //                                     break;  // 找到相似平面，跳出循环
    //                                 }
    //                             }

    //                             // 如果没有找到相似的平面，则添加到 mpPlanes 中
    //                             if (!similar_found) {
    //                                 MapObjects[0]->addFilteredPlanesInWorld(pl);
    //                             }
    //                     }
    //                 }

    //                 std::cout<< "[MultiPlanes Add] Num of planes for first ellipsoid:  " <<MapObjects[0]->GetPlanes().size()<<std::endl;
    //                 if(MapObjects[0]->GetPlanes().size()>=7){
    //                     // 原始的 vector 容器，存储指向 g2o::plane 的指针
    //                     std::vector<g2o::plane*> mpPlanes =  MapObjects[0]->GetPlanes();
    //                     // 新的 vector 容器，存储 g2o::plane 对象
    //                     std::vector<g2o::plane> mPlanes;
    //                     // 遍历 mpPlanes，将每个指针指向的 g2o::plane 对象复制到新的容器中
    //                     for (g2o::plane* planePtr : mpPlanes) {
    //                         if (planePtr) {  // 检查指针是否为空
    //                             mPlanes.push_back(*planePtr);  // 解引用并复制到新的容器
    //                         }
    //                     }
    //                     g2o::ellipsoid e_new = pi.optimizeEllipsoidWithMultiPlanes(*MapObjects[0], mPlanes, pri);
    //                     MapObjects[0]->fromVector(e_new.toVector());
    //                 }

    //                 break;
    //             }


    //             std::vector<g2o::plane*> planes_world;
    //             // 1.生成 远近平面 in world
    //             double dis_thresh_near = Config::ReadValue<double>("OptimizeEllipsoidWithMultiPlanes.dis_thresh_near");
    //             double dis_thresh_far = Config::ReadValue<double>("OptimizeEllipsoidWithMultiPlanes.dis_thresh_far");
    //             Eigen::Vector3d farest, nearest;
    //             Eigen::Vector3d normal_far, normal_near;
    //             double dis_far=0, dis_near=1000;
    //             cv::Mat Ow = pFrame->GetCameraCenter();
    //             std::cout<<"[生成远近平面] 1 begin"<<std::endl;
    //             for(auto pMP:m.mvpObjectPoints){
    //                 if (!pMP)
    //                     continue;
    //                 if (pMP->isBad())
    //                     continue;
    //                 if (pMP->isOutlier())
    //                     continue;

    //                 auto p_pose = pMP->GetWorldPos();
    //                 // 计算p和相机的距离
    //                 cv::Mat normal = p_pose - Ow;
    //                 // 计算normal的长度
    //                 double dis = cv::norm(normal);
    //                 if(dis>dis_thresh_far)
    //                     continue;
    //                 if(dis>dis_far){
    //                     dis_far = dis;
    //                     farest[0] = p_pose.at<float>(0, 0);  
    //                     farest[1] = p_pose.at<float>(1, 0);  
    //                     farest[2] = p_pose.at<float>(2, 0);  
    //                     normal_far[0] = -1*normal.at<float>(0, 0);
    //                     normal_far[1] = -1*normal.at<float>(1, 0);
    //                     normal_far[2] = -1*normal.at<float>(2, 0);
    //                 }
    //                 if(dis<dis_thresh_near)
    //                     continue;
    //                 if(dis<dis_near){
    //                     dis_near = dis;
    //                     nearest[0] = p_pose.at<float>(0, 0);  
    //                     nearest[1] = p_pose.at<float>(1, 0);  
    //                     nearest[2] = p_pose.at<float>(2, 0);  
    //                     normal_near[0] = normal.at<float>(0, 0);
    //                     normal_near[1] = normal.at<float>(1, 0);
    //                     normal_near[2] = normal.at<float>(2, 0);
    //                 }
    //             }
    //             std::cout<<"[生成远近平面] 5";
    //             cerr << "  dis_near:"<<dis_near <<", dis_far:"<<dis_far<< endl;

    //             if(dis_near<dis_thresh_near || dis_far>dis_thresh_far)
    //             {
    //                 cerr << " [Error] 近平面过近，或 远平面过远。"<<dis_far<< endl;
    //                 exit(-1);
    //             }
                
    //             g2o::plane* plane_far = new g2o::plane();
    //             bool useFar_vertical = Config::ReadValue<double>("SemanticPrior.useFar_vertical");
    //             if(useFar_vertical){
    //                 farest[0] = 0;  
    //                 farest[1] = 0;  
    //                 farest[2] = dis_far;  
    //                 normal_far[0] = 0;
    //                 normal_far[1] = 0;
    //                 normal_far[2] = -1;
    //             }
    //             plane_far->fromPointAndNormal(farest, normal_far);
    //             plane_far->mvPlaneCenter = farest;
    //             plane_far->color = Vector3d(0,0,1.0);
    //             plane_far->transform(pFrame->cam_pose_Twc); 
    //             bool useFar = Config::ReadValue<double>("SemanticPrior.useFar");
    //             if(useFar)
    //                 planes_world.push_back(plane_far);
    //             g2o::plane* plane_near = new g2o::plane();
    //             bool useNear_vertical = Config::ReadValue<double>("SemanticPrior.useNear_vertical");
    //             if(useNear_vertical){
    //                 nearest[0] = 0;  
    //                 nearest[1] = 0;  
    //                 nearest[2] = dis_near;  
    //                 normal_near[0] = 0;
    //                 normal_near[1] = 0;
    //                 normal_near[2] = 1;
    //             }
    //             plane_near->fromPointAndNormal(nearest, normal_near);
    //             plane_near->mvPlaneCenter = nearest; 
    //             plane_near->color = Vector3d(0,0,1.0);
    //             plane_near->transform(pFrame->cam_pose_Twc); 
    //             bool useNear = Config::ReadValue<double>("SemanticPrior.useNear");
    //             if(useNear)
    //                 planes_world.push_back(plane_near);

    //             // mpMap->clearPlanes();
    //             // mpMap->addPlane(&mGroundPlane);
    //             // for(auto p:planes_world){
    //             //     double plane_size=0.5;
    //             //     p->InitFinitePlane(p->mvPlaneCenter, plane_size);
    //             //     mpMap->addPlane(p);
    //             // }

    //             bool optimizeEllipsoidWithMultiPlanes = Config::ReadValue<double>("optimizeEllipsoidWithMultiPlanes.optimizeEllipsoidWithMultiPlanes");
    //             std::vector<g2o::plane> planesNearFar;
    //             if(optimizeEllipsoidWithMultiPlanes) 
    //             {
    //                 // 将bbox planes_world 和 远近平面， 转为planes_in_current_camera
    //                 for(auto p:planes_world){
    //                     g2o::plane* plane_new = new g2o::plane(p->param);
    //                     plane_new->transform(pFrame->cam_pose_Tcw);    //转到当前相机坐标系
    //                     planesNearFar.push_back(*plane_new);
    //                 }
    //                 // 将 远近平面 转为planes_in_current_camera

    //                 g2o::ellipsoid e_infer_mono_guess;
    //                 double ground_weight = Config::ReadValue<double>("SemanticPrior.GroundWeight");
    //                 e_infer_mono_guess = pi.MonocularInferWithNearFarPlane(e_init_guess, pri, weight, ground_pl_local, planesNearFar);
    //                 // 设置椭球体label, prob
    //                 e_infer_mono_guess.miLabel = m.ob_2d.label;
    //                 e_infer_mono_guess.prob = m.ob_2d.rate; // 暂时设置为 bbox 检测的概率吧
    //                 e_infer_mono_guess.bbox = m.ob_2d.bbox;
    //                 e_infer_mono_guess.prob_3d =  1.0; // 暂定!
    //                 g2o::ellipsoid* pEInfer_mono_guess = new g2o::ellipsoid(e_infer_mono_guess.transform_from(pFrame->cam_pose_Twc));

    //                 Vector3d color_rgb(144,238,144); color_rgb/=255.0;
    //                 if(!use_input_pri) color_rgb = Vector3d(1,0,0); // 默认版本为红色
    //                 pEInfer_mono_guess->setColor(color_rgb);
    //                 // mpMap->addEllipsoidObservation(pEInfer_mono_guess); // 可视化
    //                 mpMap->addEllipsoidVisual(pEInfer_mono_guess); // 可视化

    //                 // TODO:
    //                 // 需在地图中椭球体 传入地面和bbox的四个面
    //                 pEInfer_mono_guess->addFilteredPlanesInWorld(new g2o::plane(mGroundPlane.param));
    //                 int bbox_planes_num = e_init_guess.mvCPlanesInCamera.size();
    //                 for(int i=0;i<bbox_planes_num;i++){
    //                     auto pCP = e_init_guess.mvCPlanesInCamera[i];
    //                     if (pCP && pCP->pPlane) {
    //                             g2o::plane* pl = new g2o::plane(pCP->pPlane->param);
    //                             pl->transform(pFrame->cam_pose_Twc);
    //                             pEInfer_mono_guess->addFilteredPlanesInWorld(pl);
    //                     }
    //                 }

    //                 std::cout << " Before Monocular Infer: " << e_init_guess.toMinimalVector().transpose() << std::endl;
    //                 std::cout << " After Monocular Infer: " << e_infer_mono_guess.toMinimalVector().transpose() << std::endl;


    //                 // --------- 将结果放到frame中存储
    //                 if(replace_detection)
    //                     pFrame->mpLocalObjects[i] = new g2o::ellipsoid(e_infer_mono_guess);
                
    //                 // DEBUGING: 调试为何Z轴会发生变化， 先输出在局部坐标系下的两个rotMat
    //                 std::cout << "InitGuess RotMat in Camera: " << std::endl << e_init_guess.pose.rotation().toRotationMatrix() << std::endl;
    //                 std::cout << "Infered RotMat in Camera: " << std::endl << e_infer_mono_guess.pose.rotation().toRotationMatrix() << std::endl;
    //                 std::cout << "GroundPlaneNorma in Camera: " << std::endl << ground_pl_local.normal().head(3).normalized() << std::endl;

    //                 // 可视化bbox的约束平面
    //                 mpMap->clearPlanes();
    //                 mpMap->addPlane(&mGroundPlane);
    //                 VisualizeConstrainPlanes(e_infer_mono_guess, pFrame->cam_pose_Twc, mpMap); // 中点定在全局坐标系
    //             }

    //             // debug
    //             int frame_by_frame = Config::ReadValue<double>("frame_by_frame_zhjd");
    //             if(frame_by_frame) {
    //                 std::cout << "*****************************" << std::endl;
    //                 std::cout << "Press [ENTER] to continue ... , [y] to autonomous mode" << std::endl;
    //                 std::cout << "*****************************" << std::endl;
    //                 char key = getchar();
    //                 if (key=='y')
    //                 {
    //                     frame_by_frame = false;
    //                 }
    //                 else if (key=='e'){
    //                     break;
    //                 }
    //             }
            
    //        }
    //     }       

    //     std::cout << "Finish infering for " << meas_num << " objects..." << std::endl;
    //     return;
    // }




    // [改进]
    void Tracking::SetGroundPlaneMannually(const Eigen::Vector4d &param)
    {
        std::cout << "[GroundPlane] Set groundplane mannually: " << param.transpose() << std::endl;
        miGroundPlaneState = true;
        mGroundPlane.param = param;
        mGroundPlane.color = Vector3d(0,1,0);
    }


    void Tracking::SetRealPose(){
        // std::cout << "[Set real pose for the first frame from] : "<< mStrSettingPath << std::endl;
        cv::FileStorage fSettings(mStrSettingPath, cv::FileStorage::READ);
        int ConstraintType = fSettings["ConstraintType"];
        if ( ConstraintType != 1 && ConstraintType != 2 && ConstraintType != 3){
            std::cerr << ">>>>>> [WARRNING] USE NO PARAM CONSTRAINT TYPE!" << std::endl;
            // ConstraintType = 1;
            std::exit(EXIT_FAILURE);  // 或者：std::abort();
        }
        if (ConstraintType == 1){// robot_camera tf
            float qx = fSettings["Tworld_camera.qx"], qy = fSettings["Tworld_camera.qy"], qz = fSettings["Tworld_camera.qz"], qw = fSettings["Tworld_camera.qw"],
                    tx = fSettings["Tworld_camera.tx"], ty = fSettings["Tworld_camera.ty"], tz = fSettings["Tworld_camera.tz"];
            //float qx = fSettings["Tgroud_firstcamera.qx"], qy = fSettings["Tgroud_firstcamera.qy"], qz = fSettings["Tgroud_firstcamera.qz"], qw = fSettings["Tgroud_firstcamera.qw"],
            //       tx = fSettings["Tgroud_firstcamera.tx"], ty = fSettings["Tgroud_firstcamera.ty"], tz = fSettings["Tgroud_firstcamera.tz"];
            mCurrentFrame.mGroundtruthPose_mat = cv::Mat::eye(4, 4, CV_32F);
            Eigen::Quaterniond quaternion(Eigen::Vector4d(qx, qy, qz, qw));
            Eigen::AngleAxisd rotation_vector(quaternion);
            Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
            T.rotate(rotation_vector);
            T.pretranslate(Eigen::Vector3d(tx, ty, tz));
            Eigen::Matrix4d GroundtruthPose_eigen = T.matrix();
            cv::Mat cv_mat_32f;
            cv::eigen2cv(GroundtruthPose_eigen, cv_mat_32f);
            cv_mat_32f.convertTo(mCurrentFrame.mGroundtruthPose_mat, CV_32F);

        } else if(ConstraintType == 2){
            // TODO: IMU
        } else if (ConstraintType == 3){// ros tf
            // tf::TransformListener listener;
            // tf::StampedTransform transform;
            // cv::Mat T_w_camera = cv::Mat::eye(4,4,CV_32F);
            // try
            // {
            //     listener.waitForTransform("/map", "/camera_depth_optical_frame", ros::Time(0), ros::Duration(1.0));
            //     listener.lookupTransform("/map", "/camera_depth_optical_frame", ros::Time(0), transform);
            //     T_w_camera = Converter::Quation2CvMat(
            //                     transform.getRotation().x(),
            //                     transform.getRotation().y(),
            //                     transform.getRotation().z(),
            //                     transform.getRotation().w(),
            //                     transform.getOrigin().x(),
            //                     transform.getOrigin().y(),
            //                     transform.getOrigin().z()
            //             );
            // }
            // catch (tf::TransformException &ex)
            // {
            //     ROS_ERROR("%s -->> lost tf from /map to /base_footprint",ex.what());
            // }

            // mCurrentFrame.mGroundtruthPose_mat = T_w_camera;
        }

        // std::cout << "[Set real pose for the first frame from] : End" << std::endl;
    }

    
    void Tracking::SetImageNames(vector<string>& vstrImageFilenamesRGB)
    {
        mvstrImageFilenamesRGB.resize(vstrImageFilenamesRGB.size());
        mvstrImageFilenamesRGB = std::vector<string>(vstrImageFilenamesRGB.begin(), vstrImageFilenamesRGB.end());
    }


    // TODO: 更新物体观测
    void Tracking::UpdateObjectEllipsoidObservation(ORB_SLAM2::Frame *pFrame, KeyFrame* pKF, bool withAssociation) {
        
        // [1] 尝试提取房间的主导曼哈顿平面，并开启Ellipsoid Extractor的物体点云曼哈顿过滤
        ExtractManhattanPlanes(pFrame);

        // [2] process single-frame ellipsoid estimation
        // 使用深度图像估计物体椭球体
        UpdateDepthEllipsoidEstimation(pFrame, pKF, withAssociation);

        int type = Config::Get<int>("Debug.EllipsoidExtraction.OpenRelations");
        if(type){
            // // [3] Extract Relationship
            // 构建椭球体与曼哈顿平面之间的关联关系
            TaskRelationship(pFrame);

            // [4] Use Relationship To Refine Ellipsoids
            // 注意: Refine时必然在第一步可以初始化出有效的物体.
            RefineObjectsWithRelations(pFrame, pKF);
            std::cout << "Finish RefineObjectsWithRelations" << std::endl;
        }
        

    }

    void Tracking::ExtractManhattanPlanes(ORB_SLAM2::Frame *pFrame)
    {
        // 提取曼哈顿平面
        g2o::plane local_ground = mGroundPlane;
        local_ground.transform(pFrame->cam_pose_Tcw);
        Vector3d local_gt = local_ground.param.head(3);
        bool success_extract = pPlaneExtractorManhattan->extractManhattanPlanes(pFrame->pointcloud_img, local_gt, pFrame->cam_pose_Twc);
        
        // 为椭球体提取器，添加 SetManhattanPlanes(, 开启mbOpenMHPlanesFilter， 激活ApplyMHPlanesFilter
        bool bOpenMHPlane = Config::Get<int>("EllipsoidExtraction.ManhattanPlanesFilter.Open") > 0;
        std::cout<< "[Tracking::ExtractManhattanPlanes] bOpenMHPlane: " << bOpenMHPlane << std::endl;
        if(bOpenMHPlane && success_extract){
            auto HomeDominantStructuralMHPlanes = pPlaneExtractorManhattan->GetHomeDominantStructuralMHPlanes();
            mpEllipsoidExtractor->OpenManhattanPlanesFilter(HomeDominantStructuralMHPlanes);
        }
        else{
            mpEllipsoidExtractor->CloseManhattanPlanes();
        }
    }

    // Process Ellipsoid Estimation for every boundingboxes in current frame.
    // Finally, store 3d Ellipsoids into the member variable mpLocalObjects of pFrame.
    // 为当前帧中的每个包围框处理椭球体估计
    // 最后，将3D椭球体存储到每一帧的成员变量mpLocalObjects中
    void Tracking::UpdateDepthEllipsoidEstimation(ORB_SLAM2::Frame* pFrame, KeyFrame* pKF, bool withAssociation)
    {
        // 1. 初始化部分
        // 获取物体观测、位姿
        auto mvpObjectDetections = pKF->GetObjectDetections(); 
        Eigen::MatrixXd &obs_mat = pFrame->mmObservations; 

        int rows = obs_mat.rows();

        Eigen::VectorXd pose = pFrame->cam_pose_Twc.toVector(); // 当前帧相机的位姿

        // 每次清除一下椭球体提取器的【用于可视化】的点云
        mpEllipsoidExtractor->ClearPointCloudList();    // clear point cloud visualization

        bool bPlaneNotClear = true;

        // 每次更新深度观测的时候都清除
        bool bEllipsoidNotClear = true;
        std::cout << "[Tracking::UpdateDepthEllipsoid Estimation] " << std::endl;
        std::cout << "共有 " << rows << " 个检测结果" << std::endl;
        std::string pcd_suffix = "";

        // 2. 遍历每个检测结果
        for(int i = 0; i < rows; i++){

            Eigen::VectorXd det_vec = obs_mat.row(i);  // id x1 y1 x2 y2 label rate instanceID

            std::cout << "\n=> Det " << i << ": " << det_vec.transpose().matrix() << std::endl;

            int label = round(det_vec(5));
            double measurement_prob = det_vec(6);

            Eigen::Vector4d measurement = Eigen::Vector4d(det_vec(1), det_vec(2), det_vec(3), det_vec(4));

            // 3. 筛选条件
            // is_border：包围框是否靠近图像边界。
            // c5_prob_check：置信度是否高于阈值 mProbThresh。
            // c1：包围框不在边界上。
            // c2：地面平面估计是否成功（miGroundPlaneState == 2）。
            // c3：如果启用了物体关联，必须保证关联有效。
            // c4：过滤特定类别（如人类 label=0）。
            // Filter those detections lying on the border.
            // 筛选条件1：离边界的距离
            bool is_border = calibrateMeasurement(measurement, mRows, mCols, mBorderPixels, mMeasurementLengthLimitPixels);

            // FIXME: 这里涉及到对观测框靠近边界的物体观测如何处理的问题：暂时在python检测中去除靠近边界的检测
            // 筛选条件5：物体识别的概率
            bool c5_prob_check = (measurement_prob > mProbThresh);

            g2o::ellipsoid* pLocalEllipsoidThisObservation = NULL;
            g2o::ellipsoid* pGlobalEllipsoidThisObservation = NULL;
            // 2 conditions must meet to start ellipsoid extraction:
            // C1 : the bounding box is not on border
            // C1 : 包围框是否不在边界上
            bool c1 = !is_border;

            // C2 : the groundplane has been estimated successfully
            // C2 : 地面是否被成功估计
            bool c2 = miGroundPlaneState == true;
            
            // in condition 3, it will not start
            // C3 : under with association mode, and the association is invalid, no need to extract ellipsoids again.
            // C3 : 在关联模式下，但是关联关系非法，则不再对其进行椭球体提取
            bool c3 = false;

            if( withAssociation )
            {
                int instance = round(det_vec(7));
                if ( instance < 0 ) c3 = true;  // invalid instance
            }

            // C4 : 物体过滤
            // 部分动态物体，如人类， label=0，将被过滤不考虑
            bool c4 = true;
            std::set<int> viIgnoreLabelLists = {
                0 // Human
            };

            if(viIgnoreLabelLists.find(label) != viIgnoreLabelLists.end())
                c4 = false;

            cout << "[Tracking::UpdateDepthEllipsoid Estimation]  - prob|NotBorder|HasGround|NotAssociation|NotFiltered:" \
                << c5_prob_check << "," << c1 << "," << c2 << "," << !c3 << "," << c4 << std::endl;

            // 对观测进行椭球体提取的几大条件
            if( c5_prob_check && c1 && c2 && !c3 && c4 ){
                
                mpMap->clearPlanes();
                mpMap->addPlane(&mGroundPlane);
                
                // 使用多平面估计局部椭球体 (depth, label, bbox, prob, mCamera)
                // TODO： 这里有待将物体对应的深度点云添加给MapObject，可以先通过椭球体进行关联
                // 得到的椭球体模型表示在相机坐标系中

                // 4. 椭球体估计
                // TODO: 这里要将物体点云添加给观测

                // FIXME: 需要判断返回的 e_extractByFitting_newSym 是否合法（初始化完成）
                // 同时提取点云，存入pcd_ptr_of_frame中
                std::cout<< "[Tracking::UpdateDepthEllipsoid Estimation] 利用地面和bbox切面估计椭球体" << std::endl;
                g2o::ellipsoid e_extractByFitting_newSym;
                int type = Config::Get<int>("Debug.EllipsoidExtraction.UsingMultiPlanes");
                if(type == 1){
                    std::cout<<"[debug] Tracking::UpdateDepthEllipsoidEstimation, Using Multi Planes" << std::endl;
                    pcl::PointCloud<PointType>::Ptr pcd_ptr_of_frame(new pcl::PointCloud<PointType>);
                    e_extractByFitting_newSym = \
                        mpEllipsoidExtractor->EstimateLocalEllipsoidUsingMultiPlanes(\
                            pFrame->pointcloud_img, measurement, label, measurement_prob, pose, mCamera, pcd_ptr_of_frame);
                    auto det = mvpObjectDetections[i];
                    if (pcd_ptr_of_frame==NULL){
                        std::cerr << "[Tracking::UpdateDepthEllipsoid Estimation]  椭球体提取中，当前帧点云为空" << std::endl;
                        det->isValidPcd = false;
                    }
                    else{
                        det->isValidPcd = true;
                    }
                }
                else if(type == 2)
                {
                    std::cout<<"[debug] Tracking::UpdateDepthEllipsoidEstimation, Using Supporting Planes" << std::endl;
                    g2o::plane* pSupPlaneLocal = new g2o::plane(mGroundPlane);
                    pSupPlaneLocal->transform(pFrame->cam_pose_Twc.inverse());
                    e_extractByFitting_newSym = \
                        mpEllipsoidExtractor->EstimateLocalEllipsoidWithSupportingPlane( \
                            pFrame->pointcloud_img, measurement, label, measurement_prob, pose, mCamera, pSupPlaneLocal);
                    auto det = mvpObjectDetections[i];  det->isValidPcd = true;
                }

                
                // 5. 椭球体结果处理
                // 判断是否拿到可靠椭球体
                bool c0 = mpEllipsoidExtractor->GetResult();
                std::cout << "[Tracking::UpdateDepthEllipsoid Estimation] 检测是否提取到椭球体： " << c0 << std::endl;

                g2o::ellipsoid* pObjByFitting;
                
                // 可视化部分
                if( c0 )
                {
                    // Visualize estimated ellipsoid
                    // 将相机坐标系的椭球体转换到世界坐标系内
                    pObjByFitting = new g2o::ellipsoid(e_extractByFitting_newSym.transform_from(pFrame->cam_pose_Twc));
                    
                    if(pObjByFitting->prob_3d > 0.5)
                        pObjByFitting->setColor(Vector3d(0.8,0.0,0.0), 1); // Set green color
                    else{
                        // prob_3d
                        // FIXME： 如果 prob_3d < 0.5, 使用bbox边界对ellipsold进行再次refine
                        pObjByFitting->setColor(Vector3d(0.8,0,0), 0.5); // 透明颜色
                    }

                    // 临时更新： 此处显示的是 3d prob
                    // pObjByFitting->prob = pObjByFitting->prob_3d;

                    // 第一次添加时清除上一次观测!
                    if(bEllipsoidNotClear)
                    {
                        // mpMap->ClearEllipsoidsVisual(); // Clear the Visual Ellipsoids in the map
                        // mpMap->ClearBoundingboxes();
                        bEllipsoidNotClear = false;
                    }

                    std::cout<< "  - Add EllipsoidVisual to Map" << std::endl;
                    mpMap->addEllipsoidVisual(pObjByFitting);

                    // std::cout << "Add Ellipsold" << std::endl;
                    
                    // cout << "detection " << i << " = " << pObjByFitting->pose << endl;
                    
                    // std::cout << "*****************************" << std::endl;
                    // std::cout << "Show EllipsoidVisual, press [ENTER] to continue ... " << std::endl;
                    // std::cout << "*****************************" << std::endl;
                    // getchar();
                    // 添加debug, 测试筛选图像平面内的bbox平面
                    // VisualizeCuboidsPlanesInImages(e_extractByFitting_newSym, pFrame->cam_pose_Twc, mCalib, mRows, mCols, mpMap);

                }   // successful estimation.

                // // 存储条件1: 该检测 3d_prob > 0.5
                // // 最终决定使用的估计结果
                if( c0 ){
                    g2o::ellipsoid *pE_extractByFitting = new g2o::ellipsoid(e_extractByFitting_newSym);
                    pLocalEllipsoidThisObservation = pE_extractByFitting;   // Store result to pE_extracted.

                    g2o::ellipsoid *pE_extractByFittingGlobal = new g2o::ellipsoid(*(pObjByFitting));
                    pGlobalEllipsoidThisObservation = pE_extractByFittingGlobal;
                }

            }

            // 若不成功保持为NULL
            pFrame->mpLocalObjects.push_back(pLocalEllipsoidThisObservation);
            // ellipsoid-verison
            pKF->AddEllipsoldsGlobal(pGlobalEllipsoidThisObservation);

        }

        return;
    }

    // 构建椭球体与曼哈顿平面之间的关联关系
    void Tracking::TaskRelationship(ORB_SLAM2::Frame *pFrame)
    {
        std::vector<g2o::ellipsoid*>& vpEllipsoids = pFrame->mpLocalObjects;

        // 获得曼哈顿planes.
        std::vector<g2o::plane*> vpPlanes = pPlaneExtractorManhattan->GetPotentialMHPlanes();

        // 检查曼哈顿平面与椭球体的关系
        Relations rls = mpRelationExtractor->ExtractSupporttingRelations(vpEllipsoids, vpPlanes, pFrame, QUADRIC_MODEL);

        if(rls.size()>0)
        {
            // 将结果存储到 frame 中
            pFrame->mbSetRelation = true;
            pFrame->relations = rls;
        }

        // ****************************
        //          可视化部分
        // ****************************
        g2o::SE3Quat Twc = pFrame->cam_pose_Twc;
        std::vector<PointCloudPCL> vPlanePoints = pPlaneExtractorManhattan->GetPotentialMHPlanesPoints();
        mpMap->AddPointCloudList("Relationship.Relation Planes", vPlanePoints, Twc, REPLACE_POINT_CLOUD);

        // 可视化该关系
        // VisualizeRelations(rls, mpMap, Twc, vPlanePoints); // 放到地图中去显示?

        // std::cout << "EllipObjects: " << vpEllipsoids.size() << std::endl;
        // std::cout << "Relation Planes : " << vpPlanes.size() << std::endl;
        // std::cout << "Relations : " << rls.size() << std::endl;
    }

    // *******
    // 
    // 1) 基于局部提取的平面，做一次分割以及椭球体提取
    // 2) 若该椭球体满足 IoU >0.5, 则替换掉之前的
    // 3) 若不满足，则使用点云中心+bbox产生点模型椭球体
    void Tracking::RefineObjectsWithRelations(ORB_SLAM2::Frame *pFrame, KeyFrame* pKF)
    {
        
        // 获取该帧
        Relations& rls = pFrame->relations;
        int num = rls.size();

        Eigen::VectorXd pose = pFrame->cam_pose_Twc.toVector();

        int success_num = 0;
        std::cout<< "[debug] Tracking::RefineObjectsWithRelations 1, 共有 " << num << " 个曼哈顿平面相切关系" << std::endl;
        for(int i=0;i<num;i++){
            // 对于支撑关系, 且平面非地平面
            // 将该新平面加入到 MHPlanes 中，重新计算一遍提取.
            Relation& rl = rls[i];
            std::cout<< "[debug] Tracking::RefineObjectsWithRelations 2" << std::endl;

            if(rl.type == 1){   // 支撑关系
                g2o::plane* pSupPlane = rl.pPlane;  // 局部坐标系的平面位置. TODO: 检查符号
                int obj_id = rl.obj_id;
                // 此处需要bbox位置.
                // cv::Mat& depth, Eigen::Vector4d& bbox, int label, double prob, Eigen::VectorXd &pose, camera_intrinsic& camera
                Eigen::VectorXd det_vec = pFrame->mmObservations.row(obj_id);  // id x1 y1 x2 y2 label rate imageID
                int label = round(det_vec(5));
                Eigen::Vector4d bbox = Eigen::Vector4d(det_vec(1), det_vec(2), det_vec(3), det_vec(4));
                double prob = det_vec(6);

                std::cout<< "[debug] Tracking::RefineObjectsWithRelations 3" << std::endl;
                g2o::ellipsoid e = mpEllipsoidExtractor->EstimateLocalEllipsoidWithSupportingPlane(pFrame->pointcloud_img, bbox, label, prob, pose, mCamera, pSupPlane); // 取消
                // 该提取不再放入 world? 不, world MHPlanes 还是需要考虑的.

                // 可视化该 Refined Object
                bool c0 = mpEllipsoidExtractor->GetResult();
                std::cout << "[debug] Refined mpEllipsoidExtractor->GetResult()结果为： " << c0 << std::endl;
                if( c0 )
                {
                    // Visualize estimated ellipsoid
                    g2o::ellipsoid* pObjRefined = new g2o::ellipsoid(e.transform_from(pFrame->cam_pose_Twc));
                    pObjRefined->setColor(Vector3d(0,0.8,0), 1); 
                    mpMap->addEllipsoidVisual(pObjRefined);

                    // 存储条件1: 该检测 3d_prob > 0.5
                    // bool c1 = (e.prob_3d > 0.5);
                    // 最终决定使用的估计结果
                    // if( c0 && c1 ){
                        // (*pFrame->mpLocalObjects[obj_id]) = e;                    
                        // success_num++;

                    // }

                    // 此处设定 Refine 一定优先.
                    (*pFrame->mpLocalObjects[obj_id]) = e;

                    g2o::ellipsoid e_global = e.transform_from(pFrame->cam_pose_Twc);
                    // (pKF->mpGlobalEllipsolds[obj_id]) = e_global;
                    pKF->ReplaceEllipsoldsGlobal(obj_id, &e_global);

                    success_num++;

                    std::cout << "success_num++ " << std::endl;
                }
            }
        }
        std::cout << "Refine result : " << success_num << " objs." << std::endl;
    }

    int Tracking::associateDetWithObject(ORB_SLAM2::KeyFrame *pKF, MapObject* pMO, int d_i, ObjectDetection* detKF1, vector<MapPoint*>& mvpMapPoints)
    {
        // 设置该帧的某个观测对应的物体
        pKF->AddMapObject(pMO, d_i);
        pMO->AddObservation(pKF, d_i);

        // 设置物体所包含的观测
        detKF1->isNew = false;

        int associate_object_id = pMO->mnId;
        // pMO

        // 将新观测的特征点，添加到物体中
        int newly_matched_points = 0;
        for (int k_i : detKF1->GetFeaturePoints()) {
            auto pMP = mvpMapPoints[k_i];
            if (pMP && !pMP->isBad())
            {
                // new map points
                if (pMP->object_id < 0)
                {
                    pMP->in_any_object = true;
                    pMP->object_id = associate_object_id;
                    pMO->AddMapPoints(pMP);
                    newly_matched_points++;
                }
                else
                {
                    // if pMP is already associate to a different object, set bad flag
                    // 一个特征点在不同帧可以在不同物体的mask内
                    if (pMP->object_id != associate_object_id)
                        pMP->SetBadFlag();
                }
            }
        }

        return newly_matched_points;

        // cout <<  "Matches: " << max_matches << ", New points: " << newly_matched_points << ", Keypoints: " <<
        //     detKF1->mvKeysIndices.size() << ", Associated to object by projection " << object_id_max_matches
        //     << endl << endl;
        /*cout <<  "Matches: " << max_matches << ", New points: " << newly_matched_points << ", Keypoints: " <<
            detKF1->mvKeysIndices.size() << ", Associated to object by projection " << object_id_max_matches
            << endl << endl;*/
    }



    void Tracking::DenseBuild()
    {
        bool mbOpenBuilder = Config::Get<int>("Visualization.Builder.Open") > 0;
        if(mbOpenBuilder)
        {
            double depth_range = Config::ReadValue<double>("EllipsoidExtractor_DEPTH_RANGE");   // Only consider pointcloud within depth_range

            if(!mCurrentFrame.color_img.empty()){    // RGB images are needed.
                Eigen::VectorXd pose = mCurrentFrame.cam_pose_Twc.toVector();
                // cv::imshow("mCurrentFrame->rgb_img", mCurrentFrame->rgb_img);
                // cv::waitKey(20);

                // cout << "DenseBuild: before processFrame ";
                // printMemoryUsage();

                // TODO： 下面这一步产生了较大的内存使用
                mpBuilder->processFrame(mCurrentFrame.color_img, mCurrentFrame.pointcloud_img, pose, depth_range);
                // cout << "DenseBuild: after processFrame ";
                // printMemoryUsage();

                double voxel_size = Config::Get<double>("Visualization.Builder.VoxelSize");
                // std::cout<< "[DenseBuild] Voxel size: " << voxel_size << std::endl;

                mpBuilder->voxelFilter(voxel_size);   // Down sample threshold; smaller the finer; depend on the hardware.
                // cout << "DenseBuild: after voxelFilter ";
                // printMemoryUsage();

                PointCloudPCL::Ptr pCurrentCloudPCL = mpBuilder->getCurrentMap();
                // cout << "DenseBuild: after getCurrentMap ";
                // printMemoryUsage();

                auto pCloudLocal = pclToQuadricPointCloudPtr(pCurrentCloudPCL);
                // cout << "DenseBuild: after pclToQuadricPointCloudPtr ";
                // printMemoryUsage();

                mpMap->AddPointCloudList("Builder.Local Points", pCloudLocal);
                // cout << "DenseBuild: after AddPointCloudList ";
                // printMemoryUsage();

                // Get and visualize global pointcloud.
                PointCloudPCL::Ptr pCloudPCL = mpBuilder->getMap();
                auto pCloud = pclToQuadricPointCloudPtr(pCloudPCL);
                mpMap->AddPointCloudList("Builder.Global Points", pCloud);
            }
        }
    }


}
