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

#include <LocalMapping.h>
#include <ORBmatcher.h>

using namespace std;

namespace ORB_SLAM2
{

/*
 * Tracking utils for stereo+lidar on KITTI
 */
void LocalMapping::MapObjectCulling()
{
    // Check Recent Added MapObjects
    list<MapObject*>::iterator lit = mlpRecentAddedMapObjects.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    const int cnThObs = 2;

    // Treat static and dynamic objects differently
    while(lit != mlpRecentAddedMapObjects.end())
    {
        MapObject* pMO = *lit;
        if (pMO->isDynamic())
        {
            if ((int) nCurrentKFid - (int) pMO->mpNewestKF->mnId  >= 2)
            {
                pMO->SetBadFlag();
                lit = mlpRecentAddedMapObjects.erase(lit);
                mpMap->mnDynamicObj--;
            }
        }

        if(pMO->isBad())
        {
            lit = mlpRecentAddedMapObjects.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMO->mnFirstKFid) >= 2 && pMO->Observations() <= cnThObs)
        {
            pMO->SetBadFlag();
            lit = mlpRecentAddedMapObjects.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMO->mnFirstKFid) >= 3)
            lit = mlpRecentAddedMapObjects.erase(lit);
        else
            lit++;
    }

    // Dynamic objects that aren't recently added
    if (mpMap->mnDynamicObj > 0)
    {
        std::vector<MapObject*> pMOs = mpMap->GetAllMapObjects();
        for (MapObject *pMO : pMOs)
        {
            if (pMO->isDynamic())
            {
                if ((int) nCurrentKFid - (int) pMO->mpNewestKF->mnId  >= 2)
                {
                    pMO->SetBadFlag();
                    mpMap->mnDynamicObj--;
                }
            }
        }
    }
}

void LocalMapping::GetNewObservations()
{
    // PyThreadStateLock PyThreadLock;

    // // cout << "LocalMapping: Estimating new poses for associated objects" << endl;

    // auto Tcw = Converter::toMatrix4f(mpCurrentKeyFrame->GetPose());
    // auto mvpAssociatedObjects = mpCurrentKeyFrame->GetMapObjectMatches();
    // auto mvpObjectDetections = mpCurrentKeyFrame->GetObjectDetections();

    // for (int i = 0; i < mvpObjectDetections.size(); i++)
    // {
    //     auto det = mvpObjectDetections[i];
    //     if (det->isNew)
    //         continue;
    //     if (!det->isGood)
    //         continue;

    //     auto pMO = mvpAssociatedObjects[i];
    //     if (pMO)
    //     {
    //         // Tco obtained by transforming Two to camera frame
    //         Eigen::Matrix4f iniSE3Tco = Tcw * pMO->GetPoseSE3();
    //         g2o::SE3Quat Tco = Converter::toSE3Quat(iniSE3Tco);
    //         // Tco after running ICP, use Tco provided by detector
    //         Eigen::Matrix4f SE3Tco = pyOptimizer.attr("estimate_pose_cam_obj")
    //                 (det->SE3Tco, pMO->scale, det->SurfacePoints, pMO->GetShapeCode()).cast<Eigen::Matrix4f>();
    //         g2o::SE3Quat Zco = Converter::toSE3Quat(SE3Tco);
    //         // error
    //         Eigen::Vector3f dist3D = SE3Tco.topRightCorner<3, 1>() - iniSE3Tco.topRightCorner<3, 1>();
    //         Eigen::Vector2f dist2D; dist2D << dist3D[0], dist3D[2];
    //         Eigen::Matrix<double, 6, 1> e = (Tco.inverse() * Zco).log();

    //         if (pMO->isDynamic()) // if associated with a dynamic object
    //         {
    //             auto motion = pMO->SE3Tow * Tcw.inverse() * SE3Tco;
    //             float deltaT = (float)(mpCurrentKeyFrame->mnFrameId - mpLastKeyFrame->mnFrameId);
    //             auto speed = motion.topRightCorner<3, 1>() / deltaT;
    //             pMO->SetObjectPoseSE3(Tcw.inverse() * SE3Tco);
    //             pMO->SetVelocity(speed);
    //         }
    //         else // associated with a static object
    //         {
    //             if (dist2D.norm() < 1.0 && e.norm() < 1.5) // if the change of translation is very small, then it really is a static object
    //             {
    //                 det->SetPoseMeasurementSE3(SE3Tco);
    //             }
    //             else // if change is large, it could be dynamic object or false association
    //             {
    //                 // If just observed, assume it is dynamic
    //                 if (pMO->Observations() <= 2)
    //                 {
    //                     pMO->SetDynamicFlag();
    //                     auto motion = pMO->SE3Tow * Tcw.inverse() * SE3Tco;
    //                     float deltaT = (float)(mpCurrentKeyFrame->mnFrameId - mpLastKeyFrame->mnFrameId);
    //                     auto speed = motion.topRightCorner<3, 1>() / deltaT;
    //                     pMO->SetObjectPoseSE3(Tcw.inverse() * SE3Tco);
    //                     pMO->SetVelocity(speed);
    //                     mpMap->mnDynamicObj++;
    //                 }
    //                 else
    //                 {
    //                     det->isNew = true;
    //                     mpCurrentKeyFrame->EraseMapObjectMatch(i);
    //                     pMO->EraseObservation(mpCurrentKeyFrame);
    //                 }
    //             }
    //         }
    //     }
    // }
}

void LocalMapping::CreateNewMapObjects()
{
    // PyThreadStateLock PyThreadLock;

    // // cout << "LocalMapping: Started new objects creation" << endl;

    // auto SE3Twc = Converter::toMatrix4f(mpCurrentKeyFrame->GetPoseInverse());
    // auto mvpObjectDetections = mpCurrentKeyFrame->GetObjectDetections();

    // for (int i = 0; i < mvpObjectDetections.size(); i++)
    // {
    //     // This might happen when a new KF is created in Tracking thread
    //     if (mbAbortBA)
    //         return;

    //     auto det = mvpObjectDetections[i];

    //     if (det->nRays == 0)
    //         continue;
    //     if (!det->isNew)
    //         continue;
    //     if (!det->isNew)
    //         continue;
    //     auto pyMapObject = pyOptimizer.attr("reconstruct_object")
    //             (det->Sim3Tco, det->SurfacePoints, det->RayDirections, det->DepthObs);
    //     if (!pyMapObject.attr("is_good").cast<bool>())
    //         continue;

    //     if (mbAbortBA)
    //         return;

    //     auto Sim3Tco = pyMapObject.attr("t_cam_obj").cast<Eigen::Matrix4f>();
    //     det->SetPoseMeasurementSim3(Sim3Tco);
    //     // Sim3, SE3, Sim3
    //     Eigen::Matrix4f Sim3Two = SE3Twc * Sim3Tco;
    //     auto code = pyMapObject.attr("code").cast<Eigen::Matrix<float, 64, 1>>();
    //     auto pNewObj = new MapObject(Sim3Two, code, mpCurrentKeyFrame, mpMap);

    //     auto pyMesh = pyMeshExtractor.attr("extract_mesh_from_code")(code);
    //     pNewObj->vertices = pyMesh.attr("vertices").cast<Eigen::MatrixXf>();
    //     pNewObj->faces = pyMesh.attr("faces").cast<Eigen::MatrixXi>();

    //     pNewObj->AddObservation(mpCurrentKeyFrame, i);
    //     mpCurrentKeyFrame->AddMapObject(pNewObj, i);
    //     mpMap->AddMapObject(pNewObj);
    //     mpObjectDrawer->AddObject(pNewObj);
    //     mlpRecentAddedMapObjects.push_back(pNewObj);
    // }
    // // cout << "LocalMapping: Finished new objects creation" << endl;
}

/*
 * Tracking utils for monocular input on Freiburg Cars and Redwood OS
 */
void LocalMapping::CreateNewObjectsFromDetections()   // 用于单目模式
{
    // cout << "LocalMapping: Started new objects creation" << endl;

    cv::Mat Rcw = mpCurrentKeyFrame->GetRotation();
    cv::Mat tcw = mpCurrentKeyFrame->GetTranslation();
    auto mvpObjectDetections = mpCurrentKeyFrame->GetObjectDetections();

    // Create new objects first, otherwise data association might fail
    for (int det_i = 0; det_i < mvpObjectDetections.size(); det_i++)
    {
        auto det = mvpObjectDetections[det_i];

        // If the detection is a new object, create a new map object.
        if (!det->isNew)
            continue;
        if (!det->isGood)
            continue;

        // Create object with associated feature points
        auto pNewObj = new MapObject(mpCurrentKeyFrame, mpMap);
        mpCurrentKeyFrame->AddMapObject(pNewObj, det_i);
        mpMap->AddMapObject(pNewObj);

        auto mvpMapPoints = mpCurrentKeyFrame->GetMapPointMatches();
        int n_valid_points = 0;
        for (int k_i : det->GetFeaturePoints())
        {
            auto pMP = mvpMapPoints[k_i];
            if (!pMP)
                continue;
            if (pMP->isBad())
                continue;
            pMP->in_any_object = true;
            pMP->object_id = pNewObj->mnId;
            pMP->keyframe_id_added_to_object = int(mpCurrentKeyFrame->mnId);
            pNewObj->AddMapPoints(pMP);
            n_valid_points++;
        }
        return;  // for mono sequences, we only focus on the single object in the middle
    }
}

void LocalMapping::ProcessDetectedObjects_byPythonReconstruct()
{
    auto SE3Twc = Converter::toMatrix4f(mpCurrentKeyFrame->GetPoseInverse());
    auto SE3Tcw = Converter::toMatrix4f(mpCurrentKeyFrame->GetPose());
    cv::Mat Rcw = mpCurrentKeyFrame->GetRotation();
    cv::Mat tcw = mpCurrentKeyFrame->GetTranslation();
    auto mvpObjectDetections = mpCurrentKeyFrame->GetObjectDetections();
    auto mvpAssociatedObjects = mpCurrentKeyFrame->GetMapObjectMatches();

    for (int det_i = 0; det_i < mvpObjectDetections.size(); det_i++)
    {
        auto det = mvpObjectDetections[det_i];

        std::cout<< "[zhjd-debug] Detection "<<det_i<<": isNew:"<< det->isNew<< ", isGood:"<< det->isGood<< std::endl;
        // If the detection is associated with an existing map object, we consider 2 different situations:
        // 1. the object has been reconstructed: update observations 2. the object has not been reconstructed:
        // check if it's ready for reconstruction, reconstruct if it's got enough points
        if (det->isNew)   //疑问：如果是一个新的检测，为什么不创建物体？？
            continue;
        if (!det->isGood)
            continue;

        MapObject *pMO = mvpAssociatedObjects[det_i];
        if (!pMO)
            continue;
        // We only consider the object in the middle
        if (pMO->mnId != 0)
            continue;

        int numKFsPassedSinceInit = int(mpCurrentKeyFrame->mnId - pMO->mpRefKF->mnId);

        if (numKFsPassedSinceInit < 50)
            pMO->ComputeCuboidPCA(numKFsPassedSinceInit < 15);
        else  // when we have relative good object shape
            pMO->RemoveOutliersModel();
        // only begin to reconstruct the object if it is observed for enough amoubt of time (15 KFs)
        if(numKFsPassedSinceInit < 15)
            continue;

        if ((numKFsPassedSinceInit - 15) % 5 != 0)
            continue;

//        int numKFsPassedSinceLastRecon = int(mpCurrentKeyFrame->mnId) - nLastReconKFID;
//        if (numKFsPassedSinceLastRecon  < 8)
//            continue;

        std::vector<MapPoint*> points_on_object = pMO->GetMapPointsOnObject();
        int n_points = points_on_object.size();
        int n_valid_points = 0;
        for (auto pMP : points_on_object)
        {
            if (!pMP)
                continue;
            if (pMP->isBad())
                continue;
            if (pMP->isOutlier())
                continue;
            n_valid_points++;
        }

        int n_rays = 0;
        auto map_points_vector = mpCurrentKeyFrame->GetMapPointMatches();
        for (auto idx : det->GetFeaturePoints())
        {
            auto pMP = map_points_vector[idx];
            if (!pMP)
                continue;
            if (pMP->isBad())
                continue;
            if (pMP->object_id != pMO->mnId)
                continue;
            if (pMP->isOutlier())
                continue;
            n_rays++;
        }
        // cout << "Object " << pMO->mnId << ": " << n_points << " points observed, " << "with " << n_valid_points << " valid points, and " << n_rays << " rays" << endl;

        // Surface points
        if (n_valid_points >= 50 && n_rays > 20)
        {
            Eigen::MatrixXf surface_points_cam = Eigen::MatrixXf::Zero(n_valid_points, 3);
            int p_i = 0;
            for (auto pMP : points_on_object)
            {
                if (!pMP)
                    continue;
                if (pMP->isBad())
                    continue;
                if (pMP->isOutlier())
                    continue;

                cv::Mat x3Dw = pMP->GetWorldPos();
                cv::Mat x3Dc = Rcw * x3Dw + tcw;
                float xc = x3Dc.at<float>(0);
                float yc = x3Dc.at<float>(1);
                float zc = x3Dc.at<float>(2);
                surface_points_cam(p_i, 0) = xc;
                surface_points_cam(p_i, 1) = yc;
                surface_points_cam(p_i, 2) = zc;
                p_i++;
            }

            // Rays
            Eigen::MatrixXf ray_pixels = Eigen::MatrixXf::Zero(n_rays, 2);
            Eigen::VectorXf depth_obs = Eigen::VectorXf::Zero(n_rays);
            int k_i = 0;
            for (auto point_idx : det->GetFeaturePoints())
            {
                auto pMP = map_points_vector[point_idx];
                if (!pMP)
                    continue;
                if(pMP->isBad())
                    continue;
                if(pMP->object_id != pMO->mnId)
                    continue;
                if (pMP->isOutlier())
                    continue;

                cv::Mat x3Dw = pMP->GetWorldPos();
                cv::Mat x3Dc = Rcw * x3Dw + tcw;
                depth_obs(k_i) = x3Dc.at<float>(2);
                ray_pixels(k_i, 0) = mpCurrentKeyFrame->mvKeysUn[point_idx].pt.x;
                ray_pixels(k_i, 1 ) = mpCurrentKeyFrame->mvKeysUn[point_idx].pt.y;
                k_i++;
            }

            Eigen::MatrixXf u_hom(n_rays, 3);
            u_hom << ray_pixels, Eigen::MatrixXf::Ones(n_rays, 1);
            Eigen::MatrixXf fg_rays(n_rays, 3);
            Eigen::Matrix3f invK = Converter::toMatrix3f(mpTracker->GetCameraIntrinsics()).inverse();
            for (int i = 0; i  < n_rays; i++)
            {
                auto x = u_hom.row(i).transpose();
                fg_rays.row(i) = (invK * x).transpose();
            }
            Eigen::MatrixXf rays(fg_rays.rows() + det->background_rays.rows(), 3);
            rays << fg_rays, det->background_rays;

            PyThreadStateLock PyThreadLock;

            // 获取dsp优化器
            int class_id = 60; //det->label;  //临时设置为table，用于debug
            py::object* optimizer_ptr;
            if(mmPyOptimizers.count(class_id) > 0) {
                py::object* optimizer_ptr_local = &(mmPyOptimizers[class_id]);
                optimizer_ptr = optimizer_ptr_local;
            }
            else{
                cout << " [ProcessDetectedObjects_byPythonReconstruct] class " << class_id << " is not in yolo_classes" << endl;
                int default_class_id = 60;  //默认物体设置为桌子
                py::object* optimizer_ptr_local = &(mmPyOptimizers[default_class_id]);
                optimizer_ptr = optimizer_ptr_local;
            }

            cout << "Before reconstruct_object" << std::endl;

            auto pyMapObject = optimizer_ptr->attr("reconstruct_object")
                    (SE3Tcw * pMO->Sim3Two, surface_points_cam, rays, depth_obs, pMO->vShapeCode);

            // If not initialized, duplicate optimization to resolve orientation ambiguity
            if (!pMO->reconstructed)
            {
                auto flipped_Two = pMO->Sim3Two;
                flipped_Two.col(0) *= -1;
                flipped_Two.col(2) *= -1;
                auto pyMapObjectFlipped = optimizer_ptr->attr("reconstruct_object")
                        (SE3Tcw * flipped_Two, surface_points_cam, rays, depth_obs, pMO->vShapeCode);

                if (pyMapObject.attr("loss").cast<float>() > pyMapObjectFlipped.attr("loss").cast<float>())
                    pyMapObject = pyMapObjectFlipped;
            }

            auto Sim3Tco = pyMapObject.attr("t_cam_obj").cast<Eigen::Matrix4f>();
            det->SetPoseMeasurementSim3(Sim3Tco);
            // Sim3, SE3, Sim3
            Eigen::Matrix4f Sim3Two = SE3Twc * Sim3Tco;
            int code_len = optimizer_ptr->attr("code_len").cast<int>();
            Eigen::Matrix<float, 64, 1> code = Eigen::VectorXf::Zero(64);
            if (code_len == 32)
            {
                auto code_32 = pyMapObject.attr("code").cast<Eigen::Matrix<float, 32, 1>>();
                code.head(32) = code_32;
            }
            else
            {
                code = pyMapObject.attr("code").cast<Eigen::Matrix<float, 64, 1>>();
            }

            
            // 获取mesh提取器
            py::object* mesh_extracter_ptr;
            if(mmPyOptimizers.count(class_id) > 0) {
                py::object* mesh_extracter_ptr_local = &(mmPyMeshExtractors[class_id]);
                mesh_extracter_ptr = mesh_extracter_ptr_local;
            }
            else{
                cout << " [ProcessDetectedObjects_byPythonReconstruct] class " << class_id << " is not in yolo_classes" << endl;
                int default_class_id = 60;  //默认物体设置为桌子
                py::object* mesh_extracter_ptr_local = &(mmPyMeshExtractors[default_class_id]);
                mesh_extracter_ptr = mesh_extracter_ptr_local;
            }

            pMO->UpdateReconstruction(Sim3Two, code);
            auto pyMesh = mesh_extracter_ptr->attr("extract_mesh_from_code")(code);
            pMO->vertices = pyMesh.attr("vertices").cast<Eigen::MatrixXf>();
            pMO->faces = pyMesh.attr("faces").cast<Eigen::MatrixXi>();
            pMO->reconstructed = true;
            pMO->AddObservation(mpCurrentKeyFrame, det_i);
            mpCurrentKeyFrame->AddMapObject(pMO, det_i);
            mpObjectDrawer->AddObject(pMO);
            mlpRecentAddedMapObjects.push_back(pMO);

            nLastReconKFID = int(mpCurrentKeyFrame->mnId);
        }
    }
}




/*
 * Tracking utils for monocular input on Freiburg Cars and Redwood OS
 */
void LocalMapping::Create_Multi_NewObjectsFromDetections()  // 用于RGBD模式
{
    // cout << "LocalMapping: Started new objects creation" << endl;

    cv::Mat Rcw = mpCurrentKeyFrame->GetRotation();
    cv::Mat tcw = mpCurrentKeyFrame->GetTranslation();
    auto mvpObjectDetections = mpCurrentKeyFrame->GetObjectDetections();

    // Create new objects first, otherwise data association might fail
    for (int det_i = 0; det_i < mvpObjectDetections.size(); det_i++)
    {
        auto det = mvpObjectDetections[det_i];

        // If the detection is a new object, create a new map object.
        if (!det->isNew)
            continue;
        if (!det->isGood)
            continue;

        // Create object with associated feature points
        auto pNewObj = new MapObject(mpCurrentKeyFrame, mpMap);
        mpCurrentKeyFrame->AddMapObject(pNewObj, det_i);
        mpMap->AddMapObject(pNewObj);

        auto mvpMapPoints = mpCurrentKeyFrame->GetMapPointMatches();
        int n_valid_points = 0;
        for (int k_i : det->GetFeaturePoints())
        {
            auto pMP = mvpMapPoints[k_i];
            if (!pMP)
                continue;
            if (pMP->isBad())
                continue;
            pMP->in_any_object = true;
            pMP->object_id = pNewObj->mnId;
            pMP->keyframe_id_added_to_object = int(mpCurrentKeyFrame->mnId);
            pNewObj->AddMapPoints(pMP);
            n_valid_points++;
        }
        // pNewObj->GetMapPointsWithinBoundingCubeToGround();
        // std::cout<<"[GetMapPointsWithinBoundingCubeToGround] end"<<std::endl;
        
    }
}

void LocalMapping::Process_Multi_DetectedObjects_byPythonReconstruct()
{
    auto SE3Twc = Converter::toMatrix4f(mpCurrentKeyFrame->GetPoseInverse());
    auto SE3Tcw = Converter::toMatrix4f(mpCurrentKeyFrame->GetPose());
    cv::Mat Rcw = mpCurrentKeyFrame->GetRotation();
    cv::Mat tcw = mpCurrentKeyFrame->GetTranslation();
    auto mvpObjectDetections = mpCurrentKeyFrame->GetObjectDetections();
    auto mvpAssociatedObjects = mpCurrentKeyFrame->GetMapObjectMatches();

    for (int det_i = 0; det_i < mvpObjectDetections.size(); det_i++)
    {
        auto det = mvpObjectDetections[det_i];

        std::cout<< "[zhjd-debug] Process_Multi_DetectedObjects "<<det_i<<": isNew:"<< det->isNew<< ", isGood:"<< det->isGood<< std::endl;
        // If the detection is associated with an existing map object, we consider 2 different situations:
        // 1. the object has been reconstructed: update observations 2. the object has not been reconstructed:
        // check if it's ready for reconstruction, reconstruct if it's got enough points
        if (det->isNew)   //只有track中数据关联上的物体才会被重建
            continue;
        if (!det->isGood)
            continue;

        MapObject *pMO = mvpAssociatedObjects[det_i];
        if (!pMO)
            continue;

        int numKFsPassedSinceInit = int(mpCurrentKeyFrame->mnId - pMO->mpRefKF->mnId);

        if (numKFsPassedSinceInit < 50)
            pMO->ComputeCuboidPCA(numKFsPassedSinceInit < 15);
        else  // when we have relative good object shape
            pMO->RemoveOutliersModel();
        // // only begin to reconstruct the object if it is observed for enough amoubt of time (15 KFs)
        // 修改：原程序中只有在观测到15帧之后才开始重建，我感觉没必要，因此注释掉
        // if(numKFsPassedSinceInit < 15)
        //     continue;

        // 修改：原程序中只有在5的倍数帧才开始重建，为了debug，改成每帧都重建
        if ((numKFsPassedSinceInit - 15) % 5 != 0)
            continue;

//        int numKFsPassedSinceLastRecon = int(mpCurrentKeyFrame->mnId) - nLastReconKFID;
//        if (numKFsPassedSinceLastRecon  < 8)
//            continue;
        
        // 1. 统计物体 pMO 上有效（三维）地图点的数量，存储在变量 n_valid_points 中。
        std::vector<MapPoint*> points_on_object = pMO->GetMapPointsOnObject();
        int n_points = points_on_object.size();
        int n_valid_points = 0;
        for (auto pMP : points_on_object)
        {
            if (!pMP)
                continue;
            if (pMP->isBad())
                continue;
            if (pMP->isOutlier())
                continue;
            n_valid_points++;
        }

        // 2. 统计与检测到的（二维）特征点（FeaturePoints）相关联的、属于目标物体的有效射线数量，存储在变量 n_rays 中。
        int n_rays = 0;
        auto map_points_vector = mpCurrentKeyFrame->GetMapPointMatches();
        for (auto idx : det->GetFeaturePoints())
        {
            auto pMP = map_points_vector[idx];
            if (!pMP)
                continue;
            if (pMP->isBad())
                continue;
            if (pMP->object_id != pMO->mnId)
                continue;
            if (pMP->isOutlier())
                continue;
            n_rays++;
        }
        // cout << "Object " << pMO->mnId << ": " << n_points << " points observed, " << "with " << n_valid_points << " valid points, and " << n_rays << " rays" << endl;

        // Surface points
        if (n_valid_points >= 50 && n_rays > 20)
        {
            Eigen::MatrixXf surface_points_cam = Eigen::MatrixXf::Zero(n_valid_points, 3);
            // 3. （三维）sdf表面的点，存储在 surface_points_cam 中。
            int p_i = 0;
            for (auto pMP : points_on_object)
            {
                if (!pMP)
                    continue;
                if (pMP->isBad())
                    continue;
                if (pMP->isOutlier())
                    continue;

                cv::Mat x3Dw = pMP->GetWorldPos();
                cv::Mat x3Dc = Rcw * x3Dw + tcw;
                float xc = x3Dc.at<float>(0);
                float yc = x3Dc.at<float>(1);
                float zc = x3Dc.at<float>(2);
                surface_points_cam(p_i, 0) = xc;
                surface_points_cam(p_i, 1) = yc;
                surface_points_cam(p_i, 2) = zc;
                p_i++;
            }

            // 4. Rays 初始化射线和深度变量
            Eigen::MatrixXf ray_pixels = Eigen::MatrixXf::Zero(n_rays, 2);
            Eigen::VectorXf depth_obs = Eigen::VectorXf::Zero(n_rays);
            
            // 5. 遍历（二维）特征点并筛选有效的地图点
            int k_i = 0;
            for (auto point_idx : det->GetFeaturePoints())
            {
                auto pMP = map_points_vector[point_idx];
                if (!pMP)
                    continue;
                if(pMP->isBad())
                    continue;
                if(pMP->object_id != pMO->mnId)
                    continue;
                if (pMP->isOutlier())
                    continue;

                cv::Mat x3Dw = pMP->GetWorldPos();
                cv::Mat x3Dc = Rcw * x3Dw + tcw;
                depth_obs(k_i) = x3Dc.at<float>(2);
                ray_pixels(k_i, 0) = mpCurrentKeyFrame->mvKeysUn[point_idx].pt.x;
                ray_pixels(k_i, 1 ) = mpCurrentKeyFrame->mvKeysUn[point_idx].pt.y;
                k_i++;
            }

            // 6. 利用（二维）特征点ray_pixels，计算前景射线。将射线转换为相机坐标系下的射线
            Eigen::MatrixXf u_hom(n_rays, 3);
            u_hom << ray_pixels, Eigen::MatrixXf::Ones(n_rays, 1);
            Eigen::MatrixXf fg_rays(n_rays, 3);
            Eigen::Matrix3f invK = Converter::toMatrix3f(mpTracker->GetCameraIntrinsics()).inverse();
            for (int i = 0; i  < n_rays; i++)
            {
                auto x = u_hom.row(i).transpose();
                fg_rays.row(i) = (invK * x).transpose();
            }

            // 7. 背景射线。通过python程序get_detections获得
            
            // 8. 合并前景射线和背景射线。 推测：前景是二维特征点对应的线，背景是物体mask中每个像素对应的线
            Eigen::MatrixXf rays(fg_rays.rows() + det->background_rays.rows(), 3);
            rays << fg_rays, det->background_rays;

            PyThreadStateLock PyThreadLock;

            // 获取dsp优化器
            int class_id = 60; //det->label;  //临时设置为table，用于debug
            py::object* optimizer_ptr;
            if(mmPyOptimizers.count(class_id) > 0) {
                py::object* optimizer_ptr_local = &(mmPyOptimizers[class_id]);
                optimizer_ptr = optimizer_ptr_local;
            }
            else{
                cout << " [ProcessDetectedObjects_byPythonReconstruct] class " << class_id << " is not in yolo_classes" << endl;
                int default_class_id = 60;  //默认物体设置为桌子
                py::object* optimizer_ptr_local = &(mmPyOptimizers[default_class_id]);
                optimizer_ptr = optimizer_ptr_local;
            }

            cout << "Before reconstruct_object" << std::endl;

            auto pyMapObject = optimizer_ptr->attr("reconstruct_object")
                    (SE3Tcw * pMO->Sim3Two, surface_points_cam, rays, depth_obs, pMO->vShapeCode);
            cout << "reconstruct_object 1" << std::endl;

            // If not initialized, duplicate optimization to resolve orientation ambiguity
            if (!pMO->reconstructed)
            {
                auto flipped_Two = pMO->Sim3Two;
                flipped_Two.col(0) *= -1;
                flipped_Two.col(2) *= -1;
                auto pyMapObjectFlipped = optimizer_ptr->attr("reconstruct_object")
                        (SE3Tcw * flipped_Two, surface_points_cam, rays, depth_obs, pMO->vShapeCode);

                if (pyMapObject.attr("loss").cast<float>() > pyMapObjectFlipped.attr("loss").cast<float>())
                    pyMapObject = pyMapObjectFlipped;
            }
            cout << "reconstruct_object 2" << std::endl;

            auto Sim3Tco = pyMapObject.attr("t_cam_obj").cast<Eigen::Matrix4f>();
            det->SetPoseMeasurementSim3(Sim3Tco);
            // Sim3, SE3, Sim3
            Eigen::Matrix4f Sim3Two = SE3Twc * Sim3Tco;
            int code_len = optimizer_ptr->attr("code_len").cast<int>();
            Eigen::Matrix<float, 64, 1> code = Eigen::VectorXf::Zero(64);
            if (code_len == 32)
            {
                auto code_32 = pyMapObject.attr("code").cast<Eigen::Matrix<float, 32, 1>>();
                code.head(32) = code_32;
            }
            else
            {
                code = pyMapObject.attr("code").cast<Eigen::Matrix<float, 64, 1>>();
            }

            
            cout << "Before extract_mesh_from_code" << std::endl;

            // 获取mesh提取器
            py::object* mesh_extracter_ptr;
            if(mmPyOptimizers.count(class_id) > 0) {
                py::object* mesh_extracter_ptr_local = &(mmPyMeshExtractors[class_id]);
                mesh_extracter_ptr = mesh_extracter_ptr_local;
            }
            else{
                cout << " [ProcessDetectedObjects_byPythonReconstruct] class " << class_id << " is not in yolo_classes" << endl;
                int default_class_id = 60;  //默认物体设置为桌子
                py::object* mesh_extracter_ptr_local = &(mmPyMeshExtractors[default_class_id]);
                mesh_extracter_ptr = mesh_extracter_ptr_local;
            }

            pMO->UpdateReconstruction(Sim3Two, code);
            auto pyMesh = mesh_extracter_ptr->attr("extract_mesh_from_code")(code);
            pMO->vertices = pyMesh.attr("vertices").cast<Eigen::MatrixXf>();
            pMO->faces = pyMesh.attr("faces").cast<Eigen::MatrixXi>();
            pMO->reconstructed = true;
            pMO->AddObservation(mpCurrentKeyFrame, det_i);
            mpCurrentKeyFrame->AddMapObject(pMO, det_i);
            mpObjectDrawer->AddObject(pMO);
            mlpRecentAddedMapObjects.push_back(pMO);

            nLastReconKFID = int(mpCurrentKeyFrame->mnId);
        }
    }
}


// 根据已有point的min和max xy，将0~maxz的点都加入pMO->GetMapPointsOnObject()中。
std::vector<MapPoint*> LocalMapping::AddCubePointsToMapObject(std::vector<MapPoint*> points){
    // 计算points的float minX, float maxX, float minY, float maxY, float maxZ
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();
    float maxZ = std::numeric_limits<float>::lowest();

    for (auto pMP : points) {
        if (!pMP)
            continue;
        if (pMP->isBad())
            continue;
        if (pMP->isOutlier())
            continue;

        cv::Mat x3Dw = pMP->GetWorldPos();
        float xc = x3Dw.at<float>(0);
        float yc = x3Dw.at<float>(1);
        float zc = x3Dw.at<float>(2);

        // 更新 minX 和 maxX
        if (xc < minX) minX = xc;
        if (xc > maxX) maxX = xc;

        // 更新 minY 和 maxY
        if (yc < minY) minY = yc;
        if (yc > maxY) maxY = yc;

        // 更新 maxZ
        if (zc > maxZ) maxZ = zc;
    }

    vector<MapPoint*> vpMP = mpMap->GetAllMapPoints();

    // 存储到新的vector中
    std::vector<MapPoint*> cube_points_on_object;
    for (auto pMP : vpMP) {
        if (!pMP)
            continue;
        if (pMP->isBad())
            continue;
        // if (pMP->isOutlier())
        //     continue;

        cv::Mat x3Dw = pMP->GetWorldPos();
        float xc = x3Dw.at<float>(0);
        float yc = x3Dw.at<float>(1);
        float zc = x3Dw.at<float>(2);

        if (xc >= minX && xc <= maxX && yc >= minY && yc <= maxY && zc <= maxZ) {
            cube_points_on_object.push_back(pMP);
        }
    }

    return cube_points_on_object;
}

}
