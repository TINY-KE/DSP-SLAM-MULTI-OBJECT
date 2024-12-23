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

#ifndef OBJECTDETECTION_H
#define OBJECTDETECTION_H

# include <mutex>
# include <Eigen/Dense>

namespace ORB_SLAM2
{

class ObjectDetection {
public:
    // Detction from LiDAR frame, with initial pose, surface LiDAR points, rays and depth measurement
    ObjectDetection(const Eigen::Matrix4f &T, const Eigen::MatrixXf &Pts, const Eigen::MatrixXf &Rays,
                    const Eigen::VectorXf &Depth);

    ObjectDetection();  // Detection from Mono frame
    void SetPoseMeasurementSim3(const Eigen::Matrix4f &T);
    void SetPoseMeasurementSE3(const Eigen::Matrix4f &T);
    std::vector<int> GetFeaturePoints();
    void AddFeaturePoint(const int &i);
    int NumberOfPoints();

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix4f Sim3Tco;
    Eigen::Matrix4f SE3Tco;
    float scale;
    Eigen::Matrix3f Rco;
    Eigen::Vector3f tco;
    Eigen::MatrixXf SurfacePoints;
    Eigen::MatrixXf RayDirections;
    Eigen::VectorXf DepthObs;
    Eigen::MatrixXf background_rays;
    std::vector<int> mvKeysIndices;
    int nRays;
    int nPts;
    bool isNew;  //对于一个观测，在track的数据关联中，如果没有与地图中的现有物体匹配上，就是新的观测，即isNew=true, 否则isNew=false
    bool isGood;
    std::mutex mMutexFeatures;
    std::mutex mMutexDetection;


public:
    Eigen::Vector4d bbox;   //物体检测框，用于椭球体prior
    int label;              //物体类别
    double prob;            //物体检测概率

};
}


#endif //OBJECTDETECTION_H
