/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>
#include <mutex>

// ellipsoid version
#include "geometry/Ellipsoid.h"
#include "geometry/Geometry.h"
#include "geometry/Plane.h"
// #include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>	


using namespace g2o;

namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;
class MapObject;

enum ADD_POINT_CLOUD_TYPE
{
    REPLACE_POINT_CLOUD = 0,
    ADD_POINT_CLOUD = 1
};

class Map
{
public:
    Map();

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();

    void clear();

    vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

    int mnDynamicObj;

    // Object SLAM
    std::set<MapObject*> mspMapObjects;
    void AddMapObject(MapObject* pMO);
    void EraseMapObject(MapObject* pMO);
    MapObject* GetMapObject(int object_id);
    std::vector<MapObject*> GetAllMapObjects();

protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    std::mutex mMutexMap;




// ellipsoid-version
public:
    bool AddPointCloudList(const string& name, PointCloud* pCloud, int type = 0);   // type 0: replace when exist,  type 1: add when exist
    bool DeletePointCloudList(const string& name, int type = 0);    // type 0: complete matching, 1: partial matching
    bool ClearPointCloudLists();
    std::map<string, PointCloud*> mmPointCloudLists; // name-> pClouds

    // 针对新的接口
    bool AddPointCloudList(const string& name, std::vector<pcl::PointCloud<pcl::PointXYZRGB>>& vCloudPCL, g2o::SE3Quat& Twc, int type = REPLACE_POINT_CLOUD);

    // plane
    void addPlane(plane* pPlane, int visual_group = 0);
    std::vector<plane*> GetAllPlanes();
    void clearPlanes();

    
protected:
    std::vector<ellipsoid*> mspEllipsoidsVisual;
    // std::vector<ellipsoid*> mspEllipsoidsObjects;

public:
    // 用于可视化的椭球体，并没用参与优化
    // those visual ellipsoids are for visualization only and DO NOT join the optimization
    void addEllipsoidVisual(ellipsoid* pObj);
    std::vector<ellipsoid*> GetAllEllipsoidsVisual();
    void ClearEllipsoidsVisual();


};

} //namespace ORB_SLAM

#endif // MAP_H
