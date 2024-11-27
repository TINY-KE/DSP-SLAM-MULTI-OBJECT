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


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>

#include <opencv2/core/core.hpp>

#include "System.h"
#include <boost/filesystem.hpp>
#include <sys/resource.h>
#include <iostream>

using namespace std;

/**
 * @brief 加载图像
 * 
 * @param[in] strAssociationFilename     关联文件的访问路径
 * @param[out] vstrImageFilenamesRGB     彩色图像路径序列
 * @param[out] vstrImageFilenamesD       深度图像路径序列
 * @param[out] vTimestamps               时间戳
 */

// bool CreateDirIfNotExist(const std::string file_path)
// {
//   if (!boost::filesystem::exists(file_path)) {
//     if (!boost::filesystem::create_directories(file_path)) {
//     //   ROS_ERROR("Failed to create directory: %s", file_path.c_str());
//       std::cerr << "Failed to create directory: " << file_path << std::endl;
//       return false;
//     }
//   }
//   return true;
// }

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    // ros::init(argc, argv, "ASLAM_RGBD");
    // ros::start();
    // ros::NodeHandle nh;

    if(argc != 6)
    {
        cerr << endl << "Usage: ./dsp_slam_rgbd path_to_vocabulary path_to_settings path_to_sequence path_to_association path_to_saved_trajectory" << endl;
        return 1;
    }

    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]);

    cv::FileStorage fSettings(string(argv[2]), cv::FileStorage::READ);

    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    int nImages = vstrImageFilenamesRGB.size();

    if(vstrImageFilenamesRGB.empty()){
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size()){
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    auto msensor = ORB_SLAM2::System::RGBD;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], argv[3], msensor);

    string strSettingsFile = argv[2];

    //  path_to_dataset [path_to_map]
    string dataset_path = argv[3];
    string dataset_path_savedir = dataset_path + "/result/";

    std::string save_map_dir = std::string(argv[5]);
    std::string data_source_dir = std::string(argv[3]);

    std::cout << "- settings file: " << strSettingsFile << std::endl;
    std::cout << "- dataset_path: " << dataset_path << std::endl;

    // 读取图片
    // SLAM.SetImageNames(vstrImageFilenamesRGB);

    // 每一帧的track耗时
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);


    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;

    for(int ni = 0; ni < nImages; ni++)
    {
        std::cout << "\n========================================" << std::endl;
        std::cout << "=> Inputting Image " << ni << "/" << nImages << std::endl;

        std::chrono::steady_clock::time_point t1_read = std::chrono::steady_clock::now();

        //! 读取图像
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni], CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni], CV_LOAD_IMAGE_UNCHANGED);

        std::chrono::steady_clock::time_point t2_read = std::chrono::steady_clock::now();
        double t_read = std::chrono::duration_cast<std::chrono::duration<double> >(t2_read - t1_read).count();

        cout << " 读取 Image 耗时: " << t_read << "s" << endl;

        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;/*  */
        }


        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // cout << "imRGB.type() = " << imRGB.type() << endl;

        // cout << "imD.type() = " << imD.type() << endl;

        assert(imRGB.type()==16);
        assert(imD.type()==2);

        SLAM.TrackRGBD(imRGB, imD, tframe);
   
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        cout << " - [ total_frame: " << (double) ttrack  << "s ]" << endl;

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T = 0.0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<size_t>((T- ttrack)*1e6)));
        }

    }

    SLAM.SaveEntireMap(save_map_dir);

    string traj_path = data_source_dir  + "KeyFrameTrajectory.txt";
    
    SLAM.SaveKeyFrameTrajectoryTUM(traj_path);

    

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    cout << "Press y to quit." << endl;
    char key;
    key = getchar();

    while(key!='y' && key!='Y'){
        cout << "Press y to quit." << endl;
        key = getchar();
    }

    cv::destroyAllWindows();

    // Stop all threads
    SLAM.Shutdown();

    cout << "End." << endl;
    return 0;
}

//从关联文件中提取这些需要加载的图像的路径和时间戳
void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    //输入文件流
    ifstream fAssociation;
    //打开关联文件
    fAssociation.open(strAssociationFilename.c_str());
    //一直读取,知道文件结束
    while(!fAssociation.eof())
    {
        string s;
        //读取一行的内容到字符串s中
        getline(fAssociation,s);
        //如果不是空行就可以分析数据了
        if(!s.empty())
        {
            //字符串流
            stringstream ss;
            ss << s;
            //字符串格式:  时间戳 rgb图像路径 时间戳 深度图像路径
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}


