
#ifndef PLANEEXTRACTORMANHATTAN_H
#define PLANEEXTRACTORMANHATTAN_H

#include "PlaneExtractor.h"

namespace ORB_SLAM2
{

// 公有继承. 扩展该函数.
class PlaneExtractorManhattan : public PlaneExtractor
{

public:
    PlaneExtractorManhattan();
    PlaneExtractorManhattan(PlaneExtractorParam& param, g2o::plane* gplane);
    
    bool extractManhattanPlanes(const cv::Mat &depth, Eigen::Vector3d& local_gt, g2o::SE3Quat &Twc);
    std::vector<g2o::plane*> GetPotentialMHPlanes();    //  用于物体支撑关系的搜索
    bool GetMHResult();     // 上一次曼哈顿提取结果

    bool GetTotalMHResult();    // 整体曼哈顿提取结果: 一共应该有5个平面
    std::vector<g2o::plane*> GetHomeDominantStructuralMHPlanes();

    void SetGroundPlane(g2o::plane* gplane);

    std::vector<PointCloudPCL> GetPotentialMHPlanesPoints();

private:
    void UpdateHomeDominantStructuralMHPlanes(g2o::SE3Quat &Twc);
    void AddNewDominantMHPlane(g2o::plane* vP);


    std::vector<g2o::plane*> mvpPotentialStructuralMHPlanes_nouse; // 当前提取的潜在曼哈顿结构平面 ("经过大小过滤"), 只用于进一步提取mvpHomeDominantStructuralMHPlanes
    std::vector<g2o::plane*> mvpPotentialMHPlanes; // 当前提取的所有曼哈顿平面 ("仅满足垂直平行约束")
    bool mbResult;

    bool mbDominantResult;
    std::vector<g2o::plane*> mvpHomeDominantStructuralMHPlanes;   // 房间内的Dominant曼哈顿平面, 最多只保留5个，用于在Ellipsoid Extractor中过滤物体内点云

    g2o::plane* mpGroundplane;

    std::vector<PointCloudPCL> mvPotentialMHPlanesPoints;

};


}

#endif // PLANEEXTRACTORMANHATTAN_H