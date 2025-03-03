
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include <filesystem>
#include <iostream>
#include <filesystem>
#include <string>
 #include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include <cstdlib>  // system()
#include <cstdio>   // popen(), pclose()

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#include <Eigen/Dense>

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/core/eigen.hpp>

#include <opencv2/calib3d.hpp>  // Rodrigues() 在这个头文件里

using namespace std;
namespace fs = std::filesystem;

#include <tuple>
ros::Publisher publisher_GT;
ros::Publisher publisher_SdfObject;
ros::Publisher publisher_points;
ros::Publisher publisher_KF;
std::vector<std::tuple<float, float, float>> mvObjectColors;



// 获取目录下的所有文件名（Linux / Mac）
std::vector<std::string> getFilesInDirectory(const std::string& targetpath) {
    std::vector<std::string> files;
    std::string command = "ls " + targetpath;
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) {
        std::cerr << "无法执行 ls 命令" << std::endl;
        return files;
    }

    char buffer[256];
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        std::string filename(buffer);
        filename.erase(filename.find_last_not_of(" \n\r\t") + 1);  // 去掉换行符
        files.push_back(targetpath + "/" + filename);
    }

    pclose(pipe);
    return files;
}




void read_view(const std::string filePath, std::vector<std::vector<double>>& views){
    std::ifstream infile(filePath, std::ios::in);
    if (!infile.is_open())
    {
        std::cout << "open fail: " << filePath << " " << std::endl;
        exit(233);
    }
    else
    {
        std::cout << "read VIEWs.txt" << std::endl;
    }

    std::vector<double> view;
    views.clear();
    std::string line;

    while (getline(infile, line))
    {
        // std::cout<<"读取新的一行:"<<endl;

        std::istringstream istr(line);
        // double time,tx,ty,tz,qx,qy,qz,qw;

        double temp;
        istr >> temp;  view.push_back(temp); 

        
        cv::Vec3d translation;  // 存储 tx, ty, tz
        cv::Vec4d quaternion;   // 存储 qx, qy, qz, qw
        istr >> temp;  view.push_back(temp);  //tx
        istr >> temp;  view.push_back(temp);   //ty
        istr >> temp;  view.push_back(temp);   //tz
        istr >> temp;  view.push_back(temp);   //qx
        istr >> temp;  view.push_back(temp);   //qy
        istr >> temp;  view.push_back(temp);   //qz
        istr >> temp;  view.push_back(temp);   //qw
        std::cout<<"[read_view] view size: "<< view[0] << " " << view[1] << " " << view[2] << " " << view[3] << " " << view[4] << " " << view[5] << " " << view[6] << " " << view[7] << std::endl;

        views.push_back(view);

        view.clear();
        istr.clear();
        line.clear();
    }
}

void PublishCameras(const vector<cv::Mat> &VIEWs, int step = 15)
{
    visualization_msgs::Marker mKeyFrames;
    float fCameraSize=0.04;
    mKeyFrames.header.frame_id = "world";
    mKeyFrames.ns = "KEYFRAMES";
    mKeyFrames.id=1;
    mKeyFrames.type = visualization_msgs::Marker::LINE_LIST;
    mKeyFrames.scale.x=0.005;
    mKeyFrames.pose.orientation.w=1.0;
    mKeyFrames.action=visualization_msgs::Marker::ADD;
    mKeyFrames.color.b=1.0f;
    mKeyFrames.color.a = 1.0;
    visualization_msgs::Marker mMST;
    mMST.header.frame_id = "world";
    mMST.ns = "MST";
    mMST.id=3;
    mMST.type = visualization_msgs::Marker::LINE_LIST;
    mMST.scale.x=0.005;
    mMST.pose.orientation.w=1.0;
    mMST.action=visualization_msgs::Marker::ADD;
    mMST.color.r=0.0f;
    mMST.color.b=0.0f;
    mMST.color.g=1.0f;
    mMST.color.a = 1.0;

    std::cout<<"PublishCameras 1"<<endl;

    float d = 0.05;

    //Camera is a pyramid. Define in camera coordinate system
    cv::Mat o = (cv::Mat_<float>(4, 1) << 0, 0, 0, 1);
    cv::Mat p1 = (cv::Mat_<float>(4, 1) << d, d * 0.8, d * 0.5, 1);
    cv::Mat p2 = (cv::Mat_<float>(4, 1) << d, -d * 0.8, d * 0.5, 1);
    cv::Mat p3 = (cv::Mat_<float>(4, 1) << -d, -d * 0.8, d * 0.5, 1);
    cv::Mat p4 = (cv::Mat_<float>(4, 1) << -d, d * 0.8, d * 0.5, 1);


    for (size_t i = 0, iend = VIEWs.size(); i < iend; i++) {

        if( i%step!=0) continue;

        cv::Mat Twc = VIEWs[i];
        //根据 Ow.copyTo(Twc.rowRange(0,3).col(3));
        cv::Mat ow = VIEWs[i].rowRange(0, 3).col(3).clone();//->GetCameraCenter();
        cv::Mat p1w = Twc * p1;
        cv::Mat p2w = Twc * p2;
        cv::Mat p3w = Twc * p3;
        cv::Mat p4w = Twc * p4;

        geometry_msgs::Point msgs_o, msgs_p1, msgs_p2, msgs_p3, msgs_p4;
        msgs_o.x = ow.at<float>(0);
        msgs_o.y = ow.at<float>(1);
        msgs_o.z = ow.at<float>(2);
        msgs_p1.x = p1w.at<float>(0);
        msgs_p1.y = p1w.at<float>(1);
        msgs_p1.z = p1w.at<float>(2);
        msgs_p2.x = p2w.at<float>(0);
        msgs_p2.y = p2w.at<float>(1);
        msgs_p2.z = p2w.at<float>(2);
        msgs_p3.x = p3w.at<float>(0);
        msgs_p3.y = p3w.at<float>(1);
        msgs_p3.z = p3w.at<float>(2);
        msgs_p4.x = p4w.at<float>(0);
        msgs_p4.y = p4w.at<float>(1);
        msgs_p4.z = p4w.at<float>(2);

        mKeyFrames.points.push_back(msgs_o);
        mKeyFrames.points.push_back(msgs_p1);
        mKeyFrames.points.push_back(msgs_o);
        mKeyFrames.points.push_back(msgs_p2);
        mKeyFrames.points.push_back(msgs_o);
        mKeyFrames.points.push_back(msgs_p3);
        mKeyFrames.points.push_back(msgs_o);
        mKeyFrames.points.push_back(msgs_p4);
        mKeyFrames.points.push_back(msgs_p1);
        mKeyFrames.points.push_back(msgs_p2);
        mKeyFrames.points.push_back(msgs_p2);
        mKeyFrames.points.push_back(msgs_p3);
        mKeyFrames.points.push_back(msgs_p3);
        mKeyFrames.points.push_back(msgs_p4);
        mKeyFrames.points.push_back(msgs_p4);
        mKeyFrames.points.push_back(msgs_p1);

        if (i > step) {
            cv::Mat Owp = VIEWs[i - step].rowRange(0, 3).col(3).clone();//->GetCameraCenter();;
            geometry_msgs::Point msgs_op;
            msgs_op.x = Owp.at<float>(0);
            msgs_op.y = Owp.at<float>(1);
            msgs_op.z = Owp.at<float>(2);
            mMST.points.push_back(msgs_o);
            mMST.points.push_back(msgs_op);
        }
    }

    mKeyFrames.header.stamp = ros::Time::now();
    //mCovisibilityGraph.header.stamp = ros::Time::now();
    mMST.header.stamp = ros::Time::now();

    publisher_KF.publish(mKeyFrames);
    publisher_KF.publish(mMST);
    
}







int main(int argc, char **argv) {

    string filepath = "/home/robotlab/ws_3d_vp/src/QSP-SLAM-my/eval/单个床_my";

    // 读取旧版的轨迹真值
    string cam_traj_file_name = filepath + "/cam_traj.txt";
    std::vector<std::vector<double>> cameras;
    read_view(cam_traj_file_name, cameras);


    cout << endl << "Saving keyframe trajectory in:  " << filepath << " ..." << endl;


    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();
    std::string filename = filepath + "/cam_traj_new.txt"; 
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for (size_t i = 0, iend = cameras.size(); i < iend; i++) 
    {
        auto cam = cameras[i];

        f << setprecision(6) << cam[0] << setprecision(7) 
        << " " << cam[1] 
        << " " << cam[2]
        << " " << cam[3]
        << " " << cam[4]
        << " " << cam[5]
        << " " << cam[6]
        << " " << cam[7] << endl;

    }

    f.close();
    cout << endl << "trajectory translated!" << endl;



    return 0;
}
