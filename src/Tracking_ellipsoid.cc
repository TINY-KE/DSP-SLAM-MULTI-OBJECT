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

// relations
    void VisualizeRelations(Relations& rls, Map* pMap, g2o::SE3Quat &Twc, std::vector<PointCloudPCL>& vPlanePoints)
    {
        int num = rls.size();
        // std::cout << "Relation Num: " << num << std::endl;
        
        int mode = 0;   //REPLACE_POINT_CLOUD = 0, ADD_POINT_CLOUD = 1

        pMap->clearArrows();
        for(int i=0;i<num;i++)
        {
            Relation &rl = rls[i];
            g2o::ellipsoid* pEllip = rl.pEllipsoid;
            g2o::plane* pPlane = rl.pPlane;
            if(pEllip==NULL || pPlane==NULL) {
                std::cout << "[Relation] NULL relation : " << rl.obj_id << ", " << rl.plane_id << std::endl;
                continue;
            }
            g2o::ellipsoid e_world = pEllip->transform_from(Twc);
            g2o::plane* plane_world = new g2o::plane(*pPlane); plane_world->transform(Twc);
            Vector3d obj_center = e_world.pose.translation();
            Vector3d norm = plane_world->param.head(3); norm.normalize();

            if(rl.type == g2o::MANHATTAN_PLANE_TYPE::HORIZONTAL)    // 支撑
            {
                // 即在物体底端产生一个向上大竖直箭头.
                // 以物体为中心.
                // 以平面法向量为方向.
                double z_aix_half_length = e_world.scale(2); 
                norm = norm * z_aix_half_length;
                pMap->addArrow(obj_center, norm, Vector3d(0, 1, 1));
                // 同时高亮平面.
                plane_world->InitFinitePlane(obj_center, 1);
                plane_world->color = Vector3d(0, 1, 1);    // 青色显示supporting关系面
                pMap->addPlane(plane_world);
            }
            else if(rl.type == g2o::MANHATTAN_PLANE_TYPE::VERTICAL) // 倚靠
            {
                // 同上
                double z_aix_half_length = e_world.scale(1); 
                norm = norm * z_aix_half_length;
                pMap->addArrow(obj_center, norm, Vector3d(1, 0, 1));
                // 同时高亮平面.
                plane_world->InitFinitePlane(obj_center, 1);
                plane_world->color = Vector3d(1, 0, 1);    // 使用品红色显示backing关系面
                pMap->addPlane(plane_world);
            }

            

            // // 高亮对应平面的点云
            // int plane_id = rl.plane_id;
            // if(plane_id >= 0 && plane_id < vPlanePoints.size())
            // {
            //     PointCloudPCL::Ptr pCloudPCL(new PointCloudPCL(vPlanePoints[rl.plane_id]));
            //     ORB_SLAM2::PointCloud cloudQuadri = pclToQuadricPointCloud(pCloudPCL);
            //     ORB_SLAM2::PointCloud* pCloudGlobal = transformPointCloud(&cloudQuadri, &Twc);
                

            //     if(rl.type == RELATION_TYPE::SUPPORTING){    // 支撑
            //         int r = 0;
            //         int g = 255;
            //         int b = 255;
            //         SetPointCloudProperty(pCloudGlobal, r, g, b, 4);
            //         pMap->AddPointCloudList(string("Relationship.Activiate Sup-Planes"), pCloudGlobal, mode);
            //     }
            //     else if(rl.type == RELATION_TYPE::BACKING){ // 倚靠
            //         int r = 255;
            //         int g = 0;
            //         int b = 255;
            //         SetPointCloudProperty(pCloudGlobal, r, g, b, 4);
            //         pMap->AddPointCloudList(string("Relationship.Activiate Back-Planes"), pCloudGlobal, mode);
            //     }

            //     delete pCloudGlobal;    // 该指针对应的点云已被拷贝到另一个指针点云,清除多余的一个
            //     pCloudGlobal = NULL;
            // }
            // else 
            // {
            //     std::cout << "Invalid plane_id : " << plane_id << std::endl;
            // }
            
        }
    }


    // [改进]
    void Tracking::SetGroundPlaneMannually(const Eigen::Vector4d &param)
    {
        std::cout << "[GroundPlane] Set groundplane mannually: " << param.transpose() << std::endl;
        miGroundPlaneState = true;
        mGroundPlane.param = param;
        mGroundPlane.color = Vector3d(0,0,0);
        mGroundPlane.miMHType = g2o::MANHATTAN_PLANE_TYPE::GROUND;
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
    void Tracking::UpdateObjectEllipsoidObservation(ORB_SLAM2::Frame *pFrame, KeyFrame* pKF) {
        
        // [1] 尝试提取房间的主导曼哈顿平面，并开启Ellipsoid Extractor的物体点云曼哈顿过滤
        ExtractManhattanPlanes(pFrame);

        // [2] process single-frame ellipsoid estimation
        // 使用深度图像估计物体椭球体
        UpdateDepthEllipsoidEstimation(pFrame, pKF);


        // // [3] Extract Relationship
        // 构建椭球体与曼哈顿平面之间的关联关系
        TaskRelationship(pFrame, pKF);

        int type = Config::Get<int>("Debug.EllipsoidExtraction.OpenRelations");
        if(type){
            // [4] Use Relationship To Refine Ellipsoids
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
    void Tracking::UpdateDepthEllipsoidEstimation(ORB_SLAM2::Frame* pFrame, KeyFrame* pKF)
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
        // std::cout << "[Tracking::UpdateDepthEllipsoid Estimation] KeyFrame id: "<< pKF->mnId << ", 共有 " << rows << " 个检测结果" << std::endl;
        std::string pcd_suffix = "";
        int num_success_ellipsoid = 0;

        // 2. 遍历每个检测结果
        for(int i = 0; i < rows; i++){

            Eigen::VectorXd det_vec = obs_mat.row(i);  // id x1 y1 x2 y2 label rate instanceID

            // std::cout << "\n=> Det " << i << ": " << det_vec.transpose().matrix() << std::endl;

            int label = round(det_vec(5));
            double measurement_prob = det_vec(6);

            Eigen::Vector4d measurement = Eigen::Vector4d(det_vec(1), det_vec(2), det_vec(3)-Config::Get<double>("EllipsoidExtractor.RefineEllipsoid.Border.x2.Pixels"), det_vec(4)-Config::Get<double>("EllipsoidExtractor.RefineEllipsoid.Border.y2.Pixels"));

            // 3. 筛选条件
            // is_border：包围框是否靠近图像边界。
            // c5_prob_check：置信度是否高于阈值 mProbThresh。
            // c1：包围框不在边界上。
            // c2：地面平面估计是否成功（miGroundPlaneState == 2）。
            // c3：如果启用了物体关联，必须保证关联有效。
            // c4：过滤特定类别（如人类 label=0）。
            // Filter those detections lying on the border.
            // 筛选条件1：离边界的距离
            bool is_border = calibrateMeasurement(measurement, mRows, mCols, Config::Get<int>("Measurement.Border.Pixels"), Config::Get<int>("Measurement.LengthLimit.Pixels"));

            // FIXME: 这里涉及到对观测框靠近边界的物体观测如何处理的问题：暂时在python检测中去除靠近边界的检测
            // 筛选条件5：物体识别的概率
            bool c5_prob_check = (measurement_prob > Config::Get<double>("Measurement.Probability.Thresh"));

            g2o::ellipsoid* pLocalEllipsoidThisObservation = NULL;
            g2o::ellipsoid* pGlobalEllipsoidThisObservation = NULL;
            // 2 conditions must meet to start ellipsoid extraction:
            // C1 : the bounding box is not on border
            // C1 : 包围框是否不在边界上
            bool c1_not_on_border = !is_border;

            // // C2 : the groundplane has been estimated successfully
            // // C2 : 地面是否被成功估计
            // bool c2 = miGroundPlaneState == true;
            
            // // in condition 3, it will not start
            // // C3 : under with association mode, and the association is invalid, no need to extract ellipsoids again.
            // // C3 : 在关联模式下，但是关联关系非法，则不再对其进行椭球体提取
            // bool c3 = false;

            // if( withAssociation )
            // {
            //     int instance = round(det_vec(7));
            //     if ( instance < 0 ) c3 = true;  // invalid instance
            // }

            // C4 : 物体过滤
            // 部分动态物体，如人类， label=0，将被过滤不考虑
            bool c4_not_human = true;
            std::set<int> viIgnoreLabelLists = {
                0 // Human
            };

            if(viIgnoreLabelLists.find(label) != viIgnoreLabelLists.end())
                c4_not_human = false;
            

            // cout << "[Tracking::UpdateDepthEllipsoid Estimation]  - prob|NotBorder|HasGround|NotAssociation|NotFiltered:" \
            //     << c5_prob_check << "," << c1_not_on_border << "," << c2 << "," << !c3 << "," << c4_not_human << std::endl;
            
            // 对观测进行椭球体提取的几大条件
            if( c5_prob_check && c1_not_on_border /* && c2 && !c3 */ && c4_not_human ){
                
                mpMap->clearPlanes();
                mpMap->addPlane(&mGroundPlane);
                
                // 使用多平面估计局部椭球体 (depth, label, bbox, prob, mCamera)
                // TODO： 这里有待将物体对应的深度点云添加给MapObject，可以先通过椭球体进行关联
                // 得到的椭球体模型表示在相机坐标系中

                // 4. 椭球体估计
                // TODO: 这里要将物体点云添加给观测

                // FIXME: 需要判断返回的 e_extractByFitting_newSym 是否合法（初始化完成）
                // 同时提取点云，存入pcd_ptr_of_frame中
                // std::cout<< "[Tracking::UpdateDepthEllipsoid Estimation] 利用地面和bbox切面估计椭球体" << std::endl;
                g2o::ellipsoid e_extractByFitting_newSym;
                int DetectSource = Config::Get<int>("Debug.EllipsoidExtraction.DetectSource");
                if(DetectSource == 1){
                    // std::cout<<"[debug] Tracking::UpdateDepthEllipsoidEstimation, Using Multi Planes" << std::endl;
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
                        det->setPcdPtr(pcd_ptr_of_frame);
                        ORB_SLAM2::PointCloud* pDeepPointsInObject = pclXYZToQuadricPointCloudPtr(pcd_ptr_of_frame); // normalized coordinate
                        mpMap->AddPointCloudList("ObjectPCDCloud - Newest Detection", pDeepPointsInObject, 0);
                    }
                }
                else if(DetectSource == 2){
                    cv::Mat mask_cv = mvImObjectMasks[i];
                    pcl::PointCloud<PointType>::Ptr pcd_ptr_of_frame(new pcl::PointCloud<PointType>);
                    e_extractByFitting_newSym = \
                        mpEllipsoidExtractor->EstimateLocalEllipsoidUsingNormalVoters(\
                            pFrame->pointcloud_img, measurement, mask_cv, label, measurement_prob, pose, mCamera, pcd_ptr_of_frame);
                    auto det = mvpObjectDetections[i];
                    if (pcd_ptr_of_frame==NULL){
                        std::cerr << "[Tracking::UpdateDepthEllipsoid Estimation]  椭球体提取中，当前帧点云为空" << std::endl;
                        det->isValidPcd = false;
                    }
                    else{
                        det->setPcdPtr(pcd_ptr_of_frame);
                        ORB_SLAM2::PointCloud* pDeepPointsInObject = pclXYZToQuadricPointCloudPtr(pcd_ptr_of_frame); // normalized coordinate
                        mpMap->AddPointCloudList("ObjectPCDCloud - Newest Detection", pDeepPointsInObject, 0);
                    }
                }

                
                // 5. 椭球体结果处理
                // 判断是否拿到可靠椭球体
                bool c0 = mpEllipsoidExtractor->GetResult();
                // std::cout << "[Tracking::UpdateDepthEllipsoid Estimation] 检测是否提取到椭球体： " << c0 << std::endl;

                g2o::ellipsoid* pObjByFitting;
                
                // 可视化部分
                if( c0 )
                {                    
                    // Visualize estimated ellipsoid
                    // 将相机坐标系的椭球体转换到世界坐标系内
                    pObjByFitting = new g2o::ellipsoid(e_extractByFitting_newSym.transform_from(pFrame->cam_pose_Twc));
                    
                    if(pObjByFitting->prob_3d > 0.5)
                        pObjByFitting->setColor(Vector3d(0.0,0.0,0.8), 1); // Set green color
                    else{
                        // prob_3d
                        // FIXME： 如果 prob_3d < 0.5, 使用bbox边界对ellipsold进行再次refine
                        pObjByFitting->setColor(Vector3d(0,0,0.8), 0.5); // 透明颜色
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

                    // std::cout<< "  - Add EllipsoidVisual to Map" << std::endl;
                    mpMap->addEllipsoidVisual(pObjByFitting);

                    // std::cout << "Add Ellipsold" << std::endl;
                    
                    // cout << "detection " << i << " = " << pObjByFitting->pose << endl;
                    
                    // std::cout << "*****************************" << std::endl;
                    // std::cout << "Show EllipsoidVisual, press [ENTER] to continue ... " << std::endl;
                    // std::cout << "*****************************" << std::endl;
                    // getchar();
                    // 添加debug, 测试筛选图像平面内的bbox平面
                    // VisualizeCuboidsPlanesInImages(e_extractByFitting_newSym, pFrame->cam_pose_Twc, mCalib, mRows, mCols, mpMap);

                    g2o::ellipsoid *pE_extractByFitting = new g2o::ellipsoid(e_extractByFitting_newSym);
                    pLocalEllipsoidThisObservation = pE_extractByFitting;   // Store result to pE_extracted.

                    g2o::ellipsoid *pE_extractByFittingGlobal = new g2o::ellipsoid(*(pObjByFitting));
                    pGlobalEllipsoidThisObservation = pE_extractByFittingGlobal;

                    // TODO: 手动标定物体检测标签
                    pE_extractByFittingGlobal->miLabel = CheckManualLabel(pE_extractByFittingGlobal);
                    mvpObjectDetections[i]->label = pE_extractByFittingGlobal->miLabel;

                    num_success_ellipsoid ++;

                    // KeyFrame id: "<< mpCurrentKeyFrame->mnId << " => Det["
                    // std::cout << "\t KeyFrame id: "<< pKF->mnId << ", => Det[" << i << "] Yes  提取椭球体， pose: "<< pE_extractByFittingGlobal->pose.toXYZPRYVector().transpose() << "， scale: "<< pE_extractByFittingGlobal->scale.transpose() << std::endl;
                }
                else{
                    // std::cout << "\t KeyFrame id: "<< pKF->mnId << ", => Det[" << i << "] Fail 提取椭球体" << std::endl;
                }

            }
            else{
                // std::cout << "\t KeyFrame id: "<< pKF->mnId << ", => Det[" << i << "] Fail 提取椭球体, ";
                // cout << " - LowProb|OnBorder|IsHuman:" << !c5_prob_check << "," << !c1_not_on_border << "," << !c4_not_human << std::endl;
            }
            // 若不成功保持为NULL
            // 将椭球体观测结果存入Frame
            // std::cout<< "[Tracking::UpdateDepthEllipsoid Estimation] 当前帧椭球体提取结果中的切面数量 1: " << std::endl;
            pFrame->mpLocalObjects.push_back(pLocalEllipsoidThisObservation);
            // std::cout<< "[Tracking::UpdateDepthEllipsoid Estimation] 当前帧椭球体提取结果中的切面数量 2: " << std::endl;
            // 将椭球体观测结果存入KeyFrame
            mvpObjectDetections[i]->pLocalEllipsoidOneFrame = pLocalEllipsoidThisObservation;  // 用于椭球体联合优化
            // std::cout<< "[Tracking::UpdateDepthEllipsoid Estimation] 当前帧椭球体提取结果中的切面数量 2-2: " << std::endl;
            // if(mvpObjectDetections[i]->pLocalEllipsoidOneFrame != NULL)
            //     std::cout<< "[Tracking::UpdateDepthEllipsoid Estimation] 当前帧椭球体提取结果中的切面数量 3: " << mvpObjectDetections[i]->pLocalEllipsoidOneFrame->mvCPlanes.size() << std::endl;
            // ellipsoid-verison
            pKF->AddEllipsoldsGlobal(pGlobalEllipsoidThisObservation);

        }

        // std::cout << "[debug] Tracking::UpdateDepthEllipsoid Estimation, KeyFrame id: "<< pKF->mnId << ", 共有 " << rows << " 个检测结果, 成功提取椭球体数量: " << num_success_ellipsoid << std::endl;
        return;
    }

    // 构建椭球体与曼哈顿平面之间的关联关系
    void Tracking::TaskRelationship(ORB_SLAM2::Frame *pFrame, KeyFrame* pKF)
    {
        std::vector<g2o::ellipsoid*>& vpEllipsoids = pFrame->mpLocalObjects;

        // 获得曼哈顿planes.
        std::vector<g2o::plane*> vpPlanes = pPlaneExtractorManhattan->GetAllMHPlanes();
        std::vector<PointCloudPCL> vPlanePoints = pPlaneExtractorManhattan->GetAllMHPlanesPoints();

        // 检查曼哈顿平面与椭球体的关系
        // Relations rls = mpRelationExtractor->ExtractSupporttingRelations(vpEllipsoids, vpPlanes, pFrame, QUADRIC_MODEL);
        Relations rls = mpRelationExtractor->ExtractRelations(vpEllipsoids, vpPlanes, pKF, vPlanePoints);
        
        // ****************************
        //          可视化部分
        // ****************************
        // std::cout<<"[debug] Tracking::TaskRelationship, 1"<< std::endl;
        g2o::SE3Quat Twc = pFrame->cam_pose_Twc;
        mpMap->AddPointCloudList("Relationship.All MH Planes", vPlanePoints, Twc, REPLACE_POINT_CLOUD);

        std::vector<PointCloudPCL>  vSupportingPlanePoints;

        // 可视化该关系
        VisualizeRelations(rls, mpMap, Twc, vPlanePoints); // 放到地图中去显示?
    }

    // *******
    // 
    // 1) 基于局部提取的平面，做一次分割以及椭球体提取
    // 2) 若该椭球体满足 IoU >0.5, 则替换掉之前的
    // 3) 若不满足，则使用点云中心+bbox产生点模型椭球体
    void Tracking::RefineObjectsWithRelations(ORB_SLAM2::Frame *pFrame, KeyFrame* pKF)
    {
        
        Eigen::VectorXd camera_pose = pFrame->cam_pose_Twc.toVector();
        std::vector<g2o::ellipsoid*>& vpEllipsoids = pFrame->mpLocalObjects;
        
        for(int i=0;i<vpEllipsoids.size();i++){

            std::cout<<"[debug] RefineObjectsWithRelations 1, Object id: " << i ;
            g2o::ellipsoid* e = vpEllipsoids[i];
            if(e==NULL) {
                std::cout << ", NULL ellipsoid, continue..." << std::endl;
                continue;
            }
            std::cout << "flag:" << e->mbBackingPlaneDefined << "/" << e->mbSupportingPlaneDefined << std::endl;
            
            if(e->mbBackingPlaneDefined && e->mbSupportingPlaneDefined ){   

                std::cout<<"[debug] RefineObjectsWithRelations 2, 存在支撑和倚靠平面" << std::endl;

                double supproting_weight = Config::ReadValue<double>("EllipsoidExtractor.Optimizer.SupportingWeight");
                g2o::plane* pSupPlane = e->mpSupportingPlane->pPlane;
                std::cout<<"[debug] RefineObjectsWithRelations 2-1" << std::endl;
                double backing_weight = Config::ReadValue<double>("EllipsoidExtractor.Optimizer.BackingWeight");
                g2o::plane* pBackPlane = e->mpBackingPlane->pPlane;

                std::cout<<"[debug] RefineObjectsWithRelations 2-2" << std::endl;
                double bbox_weight = Config::ReadValue<double>("EllipsoidExtractor.Optimizer.BboxWeight");
                std::vector<g2o::plane> vBboxPlanes;
                std::vector<g2o::ConstrainPlane*> vBboxConstrainPlanes = e->mvBboxPlanesLocal;
                std::cout<<"[debug] RefineObjectsWithRelations 2-3" << std::endl;
                for(auto cp : vBboxConstrainPlanes)
                    vBboxPlanes.push_back(*cp->pPlane);

                std::cout<<"[debug] RefineObjectsWithRelations 3, 开始优化" << std::endl;

                g2o::ellipsoid e_refined = mpEllipsoidExtractor->OptimizeEllipsoidWithBboxPlanesAndMHPlanes(
                        *e, vBboxPlanes, bbox_weight, *pSupPlane, supproting_weight, *pBackPlane, backing_weight);
                



                // 可视化 Refined Object，并变换到世界坐标系下
                bool c0 = mpEllipsoidExtractor->GetResult();
                std::cout << "[debug] RefineObjectsWithRelations 4, mpEllipsoidExtractor->GetResult()结果为： " << c0 << std::endl;
                if( c0 )
                {
                    // Visualize estimated ellipsoid
                    g2o::ellipsoid* pObjRefined = new g2o::ellipsoid(e_refined.transform_from(pFrame->cam_pose_Twc));
                    // pObjRefined->setColor(Vector3d(189/255.0, 183/255.0, 107/255.0), 1); 
                    // pObjRefined->setColor(Vector3d(0/255.0, 128/255.0, 0/255.0), 1); 
                    pObjRefined->setColor(Vector3d(255/255.0, 128/255.0, 0/255.0), 1); 
                    mpMap->addRefinedEllipsoidVisual(pObjRefined);
                    
                    // 用优化后的
                    // (*pFrame->mpLocalObjects[i]) = e_refined;

                    // g2o::ellipsoid e_global = e_refined.transform_from(pFrame->cam_pose_Twc);
                    // pKF->ReplaceEllipsoldsGlobal(i, &e_global);

                }
            }
            else if(e->mbSupportingPlaneDefined ){   

                double supproting_weight = Config::ReadValue<double>("EllipsoidExtractor.Optimizer.SupportingWeight");
                g2o::plane* pSupPlane = e->mpSupportingPlane->pPlane;

                double bbox_weight = Config::ReadValue<double>("EllipsoidExtractor.Optimizer.BboxWeight");
                std::vector<g2o::plane> vBboxPlanes;
                std::vector<g2o::ConstrainPlane*> vBboxConstrainPlanes = e->mvBboxPlanesLocal;
                for(auto cp : vBboxConstrainPlanes)
                    vBboxPlanes.push_back(*cp->pPlane);

                g2o::ellipsoid e_refined = mpEllipsoidExtractor->OptimizeEllipsoidWithBboxPlanesAndMHPlanes(
                        *e, vBboxPlanes, bbox_weight, *pSupPlane, supproting_weight);
                



                // 可视化 Refined Object，并变换到世界坐标系下
                bool c0 = mpEllipsoidExtractor->GetResult();
                std::cout << "[debug] RefineObjectsWithRelations 4, mpEllipsoidExtractor->GetResult()结果为： " << c0 << std::endl;
                if( c0 )
                {
                    // Visualize estimated ellipsoid
                    g2o::ellipsoid* pObjRefined = new g2o::ellipsoid(e_refined.transform_from(pFrame->cam_pose_Twc));
                    // pObjRefined->setColor(Vector3d(189/255.0, 183/255.0, 107/255.0), 1); 
                    pObjRefined->setColor(Vector3d(255/255.0, 255/255.0, 0/255.0), 1); 
                    mpMap->addRefinedEllipsoidVisual(pObjRefined);
                    
                    // 用优化后的
                    // (*pFrame->mpLocalObjects[i]) = e_refined;

                    // g2o::ellipsoid e_global = e_refined.transform_from(pFrame->cam_pose_Twc);
                    // pKF->ReplaceEllipsoldsGlobal(i, &e_global);

                }
            }
        }

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

                bool open_global = false;

                if(open_global){
                    // Get and visualize global pointcloud.
                    PointCloudPCL::Ptr pCloudPCL = mpBuilder->getMap();
                    auto pCloud = pclToQuadricPointCloudPtr(pCloudPCL);
                    mpMap->AddPointCloudList("Builder.Global Points", pCloud);
                }
                
            }
        }
    }


PointCloud* filterCloudAsHeight(PointCloud* pCloud,  double dis_thresh)
{
    // std::cout << "Filter Cloud using thresh : " << dis_thresh << std::endl;
    PointCloud* pCloudFiltered = new PointCloud;
    int num = pCloud->size();
    for(int i=0;i<num;i++)
    {
        PointXYZRGB p = (*pCloud)[i];
        Vector3d center; center << p.x, p.y, p.z;

        double dis = p.z;
        // if(dis < dis_thresh)
            pCloudFiltered->push_back(p);
    }
    return pCloudFiltered;
}
    
void Tracking::LoadPointcloud(const string& strPcdDir, const string& strPointcloud_name)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB>(strPcdDir.c_str(), *cloud);

    //  = Ttrans.to_homogeneous_matrix();
    Matrix4d transform = Converter::toMatrix4d(mCurrentFrame.mTcw.inv());
    pcl::transformPointCloud (*cloud, *cloud, transform);

    auto pCloud = pclToQuadricPointCloudPtr(cloud);

    // 临时过滤顶部
    double dis_thresh = Config::ReadValue<double>("Visualization.Map.Filter.DisThresh");
    if(dis_thresh > 0){
        auto pCloud_filtered = filterCloudAsHeight(pCloud, dis_thresh);
        delete pCloud;
        pCloud = pCloud_filtered;
    }
    mpMap->AddPointCloudList(strPointcloud_name, pCloud);

    return;
}

}
