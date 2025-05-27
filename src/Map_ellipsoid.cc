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

#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{

// ellipsoid-version

bool Map::AddPointCloudList(const string& name, std::vector<pcl::PointCloud<pcl::PointXYZRGB>>& vCloudPCL, g2o::SE3Quat& Twc, int type)
{
    if(type == REPLACE_POINT_CLOUD){
        DeletePointCloudList(name, COMPLETE_MATCHING);
    }
    srand(time(0));
    for( auto& cloud : vCloudPCL )
    {
        EllipsoidSLAM::PointCloud cloudQuadri = pclToQuadricPointCloud(cloud);
        EllipsoidSLAM::PointCloud* pCloudGlobal = new EllipsoidSLAM::PointCloud(cloudQuadri);
        transformPointCloudSelf(pCloudGlobal, &Twc);
        
        int r = rand()%155;
        int g = rand()%155;
        int b = rand()%155;
        SetPointCloudProperty(pCloudGlobal, r, g, b, 4);
        bool result = AddPointCloudList(name, pCloudGlobal, ADD_POINT_CLOUD);
        if(!result) {
            delete pCloudGlobal;
            pCloudGlobal = NULL;
        }
    }
    return true;
}

bool Map::AddPointCloudList(const string& name, PointCloud* pCloud, int type){
    unique_lock<mutex> lock(mMutexMap);
    if(pCloud == NULL)
    {
        std::cout << "NULL point cloud." << std::endl;
        return false;
    }

    // Check repetition
    if(mmPointCloudLists.find(name) != mmPointCloudLists.end() )
    {
        // Exist
        auto pCloudInMap = mmPointCloudLists[name];
        if(pCloudInMap==NULL){
            std::cout << "Error: the cloud " << name << " has been deleted." << std::endl;
            return false;
        }

        if( type == 0){
            // replace it.
            pCloudInMap->clear(); // release it
            mmPointCloudLists[name] = pCloud;
        }
        else if( type == 1 )
        {
            // add together
            for( auto &p : *pCloud )
                pCloudInMap->push_back(p);
        }
        else 
        {
            std::cout << "Wrong type : " << type << std::endl;
        }

        return false;
    }
    else{
        mmPointCloudLists.insert(make_pair(name, pCloud));
        return true;
    }
        
}

// 删除点云
bool Map::DeletePointCloudList(const string& name, int type){
    unique_lock<mutex> lock(mMutexMap);

    if( type == 0 ) // complete matching: the name must be the same
    {
        auto iter = mmPointCloudLists.find(name);
        if (iter != mmPointCloudLists.end() )
        {
            PointCloud* pCloud = iter->second;
            if(pCloud!=NULL)
            {
                delete pCloud;
                pCloud = NULL;
            }
            mmPointCloudLists.erase(iter);
            return true;
        }
        else{
            std::cerr << "PointCloud name " << name << " doesn't exsit. Can't delete it." << std::endl;
            return false;
        }
    }
    else if ( type == 1 ) // partial matching
    {
        bool deleteSome = false;
        for( auto iter = mmPointCloudLists.begin();iter!=mmPointCloudLists.end();)
        {
            auto strPoints = iter->first;
            if( strPoints.find(name) != strPoints.npos )
            {
                PointCloud* pCloud = iter->second;
                if(pCloud!=NULL)
                {
                    delete pCloud;
                    pCloud = NULL;
                }
                iter = mmPointCloudLists.erase(iter);
                deleteSome = true;
                continue;
            }
            iter++;
        }
        return deleteSome;
    }
    
    return false;
}


bool Map::ClearPointCloudLists(){
    unique_lock<mutex> lock(mMutexMap);

    mmPointCloudLists.clear();
    return true;
}


/**
 * Plane
 */

void Map::addPlane(plane *pPlane, int visual_group) {
    unique_lock<mutex> lock(mMutexMap);
    pPlane->miVisualGroup = visual_group;
    mspPlanes.insert(pPlane);
}

vector<plane *> Map::GetAllPlanes() {
    unique_lock<mutex> lock(mMutexMap);
    return vector<plane *>(mspPlanes.begin(), mspPlanes.end());
}

void Map::clearPlanes() {
    unique_lock<mutex> lock(mMutexMap);
    mspPlanes.clear();
}



// 用于可视化的椭球体，并没用参与优化
// 添加真值/地图实际
void Map::addEllipsoidVisual(ellipsoid *pObj) {
    unique_lock<mutex> lock(mMutexMap);
    mspEllipsoidsVisual.push_back(pObj);
}

vector<ellipsoid *> Map::GetAllEllipsoidsVisual() {
    unique_lock<mutex> lock(mMutexMap);
    return mspEllipsoidsVisual;
}

void Map::ClearEllipsoidsVisual() {
    unique_lock<mutex> lock(mMutexMap);
    // cout << "!!!! Map::ClearEllipsoidsVisual !!!!" << endl;
    mspEllipsoidsVisual.clear();
}

} //namespace ORB_SLAM
