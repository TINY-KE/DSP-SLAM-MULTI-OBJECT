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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h> 
#include <tf/transform_datatypes.h> 
#include "Converter.h"
#include "System.h"


using namespace std;

std::vector<int> yolo_class;

int loop = 0;
//NBV MAM end

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}
    
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;

};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();
    string yamlfile, sensor; bool semanticOnline, rosBagFlag;
    //(1)从ros param中获取参数
    if(argc != 5)
    {
        cerr << endl << "Usage: ./qsp_slam_rgbd path_to_vocabulary path_to_settings path_to_save_color_image path_to_saved_trajectory" << endl;
        return 1;
    }
    /* 
            ./dsp_slam_rgbd     path_to_vocabulary     path_to_settings        path_to_sequence        path_to_association        path_to_saved_trajectory
            0                   1                      2                      3                      4                         5
     */

    cv::FileStorage fSettings(string(argv[2]), cv::FileStorage::READ);

    if(fSettings["System.mode"].isNamed()) {
        string system_mode = fSettings["System.mode"];
        if (system_mode=="RGBD") {
            msensor = ORB_SLAM2::System::RGBD;
        }
    }

    auto msensor = ORB_SLAM2::System::RGBD;

    ORB_SLAM2::System SLAM(argv[1], argv[2], argv[3], msensor);

    string strSettingsFile = argv[2];


    //(3)接受ros topic
    ImageGrabber igb(&SLAM);
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/depth_to_rgb/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // (4)Save camera trajectory
    // SLAM.SaveKeyFrameTrajectoryTUM("/home/zhjd/active_eao/src/active_eao/eval/temp/KeyFrameTrajectory.txt");
    // int SaveLocalObjects = fSettings["Viewer.savelocalobject"];
    // if(SaveLocalObjects){
    //     SLAM.SaveObjects(   "/home/zhjd/active_eao/src/active_eao/eval/temp/Objects.txt",
    //                         "/home/zhjd/active_eao/src/active_eao/eval/temp/Objects_with_points.txt");
    //     SLAM.SaveGlobalNBVPose("/home/zhjd/active_eao/src/active_eao/eval/temp/GlobalNBV.txt");
    // }

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    double current_time = msgRGB->header.stamp.toSec();

    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // 显示 RGB 图像
    if (cv_ptrRGB) {
        cv::imshow("RGB Image", cv_ptrRGB->image); // 使用 OpenCV 显示 RGB 图像
    }
    // 显示深度图
    if (cv_ptrD) {
        // 将深度图像归一化到 8 位（可视化）
        cv::Mat depth_normalized;
        cv::normalize(cv_ptrD->image, depth_normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        cv::imshow("Depth Image", depth_normalized);
    }
    // 刷新窗口 (非阻塞)
    cv::waitKey(1); // 等待 1 毫秒刷新窗口，非阻塞

    // cv::Mat Tcw;
    // Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,  cv_ptrD->image,  cv_ptrRGB->header.stamp.toSec());

}
