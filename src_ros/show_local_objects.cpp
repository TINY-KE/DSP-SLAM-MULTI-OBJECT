
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
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
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
ros::Publisher publisher_CubeGT;
ros::Publisher publisher_SdfObject;
ros::Publisher publisher_points;
ros::Publisher publisher_KF;
ros::Publisher publisher_baselink_trajectory;
ros::Publisher publisher_ellipsoid;
ros::Publisher publisher_MHPlanes;
std::vector<std::tuple<float, float, float>> mvObjectColors;

double my_trajectory_length=0, direct_trajectory_length=0;
string root_path = "/home/robotlab/ws_ellipsoid_dsp/src/DSP-SLAM-MULTI-OBJECT/src_ros/eval/show";

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


void publishEllipsoidWireframe(ros::Publisher& pub,
                                const Eigen::Vector3d& center,
                                const Eigen::Vector3d& radii,
                                const Eigen::Quaterniond& orientation, // ✅ 四元数
                                const std_msgs::ColorRGBA& color,
                                int id,
                                const std::string& frame_id = "world")
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "ellipsoid";
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.01;  // 线宽

    marker.color = color;

    const int segments_per_circle = 14;
    const int num_latitude = 14;
    const int num_longitude = 14;

    const double dtheta = 2 * M_PI / segments_per_circle;
    const double dphi = M_PI / num_latitude;

    // 纬线（水平圈）
    for (int i = 1; i < num_latitude; ++i) {
        double phi = i * dphi;
        double z = cos(phi);
        double r = sin(phi);

        for (int j = 0; j < segments_per_circle; ++j) {
            double theta1 = j * dtheta;
            double theta2 = (j + 1) * dtheta;

            Eigen::Vector3d p1_unit(r * cos(theta1), r * sin(theta1), z);
            Eigen::Vector3d p2_unit(r * cos(theta2), r * sin(theta2), z);

            // 缩放为椭球体
            Eigen::Vector3d p1_scaled = p1_unit.cwiseProduct(radii);
            Eigen::Vector3d p2_scaled = p2_unit.cwiseProduct(radii);

            // 旋转 + 平移
            Eigen::Vector3d p1 = orientation * p1_scaled + center;
            Eigen::Vector3d p2 = orientation * p2_scaled + center;

            geometry_msgs::Point gp1, gp2;
            gp1.x = p1.x(); gp1.y = p1.y(); gp1.z = p1.z();
            gp2.x = p2.x(); gp2.y = p2.y(); gp2.z = p2.z();

            marker.points.push_back(gp1);
            marker.points.push_back(gp2);
        }
    }

    // 经线（垂直圈）
    for (int j = 0; j < segments_per_circle; ++j) {
        double theta = j * dtheta;

        for (int i = 0; i < num_latitude; ++i) {
            double phi1 = i * dphi;
            double phi2 = (i + 1) * dphi;

            Eigen::Vector3d p1_unit(sin(phi1) * cos(theta), sin(phi1) * sin(theta), cos(phi1));
            Eigen::Vector3d p2_unit(sin(phi2) * cos(theta), sin(phi2) * sin(theta), cos(phi2));

            Eigen::Vector3d p1_scaled = p1_unit.cwiseProduct(radii);
            Eigen::Vector3d p2_scaled = p2_unit.cwiseProduct(radii);

            Eigen::Vector3d p1 = orientation * p1_scaled + center;
            Eigen::Vector3d p2 = orientation * p2_scaled + center;

            geometry_msgs::Point gp1, gp2;
            gp1.x = p1.x(); gp1.y = p1.y(); gp1.z = p1.z();
            gp2.x = p2.x(); gp2.y = p2.y(); gp2.z = p2.z();

            marker.points.push_back(gp1);
            marker.points.push_back(gp2);
        }
    }

    pub.publish(marker);
}


void PublishObjectGroundtruth(string object_groundtruth_file_name){
    std::ifstream infile(object_groundtruth_file_name, std::ios::in);
    if (!infile.is_open())
    {
        std::cout << "open fail: " << object_groundtruth_file_name << " " << std::endl;
        exit(233);
    }
    else
    {
        std::cout << "read Object Groundtruth" << std::endl;
    }

    std::vector<double> row;

    std::string line;

    int cubeId =0 ;
    while (getline(infile, line))
    {
        // std::cout<<"读取新的一行:"<<endl;

        std::istringstream istr(line);
        double tx,ty,tz,qr,qp,qy, width,length,height, angle, flag=0;

        double temp;
        cv::Vec3d translation;  // 存储 tx, ty, tz
        cv::Vec4d quaternion;   // 存储 qx, qy, qz, qw
        istr >> temp;  tx = temp;  //tx
        istr >> temp;  ty = temp;  //ty
        istr >> temp;  tz = temp;  //tz
        istr >> temp;  qr = temp;  //qr
        istr >> temp;  qp = temp;  //qp
        istr >> temp;  qy = temp;  //qy
        istr >> temp;  width = temp;  //w
        istr >> temp;  length = temp;  //h
        istr >> temp;  height = temp;  //l

        istr >> temp;  angle = temp;  //旋转角度
        istr >> temp;  flag = temp;  //flag

        

        // 发布Cube物体
        visualization_msgs::Marker CubeMarker;
        CubeMarker.id = cubeId++;
        // CubeMarker.lifetime = ros::Duration(mObject_Duration);
        CubeMarker.header.frame_id= "world";
        CubeMarker.header.stamp=ros::Time::now();
        CubeMarker.type = visualization_msgs::Marker::LINE_LIST; //LINE_STRIP;
        CubeMarker.action = visualization_msgs::Marker::ADD;
        CubeMarker.color.a = 1.0f; // 设置透明度为 1.0（不透明）
        // CubeMarker.color.r =  get<0>(mvObjectColors[(pMO->mnId+5) % 10]);
        // CubeMarker.color.g =  get<1>(mvObjectColors[(pMO->mnId+5) % 10]);
        // CubeMarker.color.b =  get<2>(mvObjectColors[(pMO->mnId+5) % 10]);
        CubeMarker.scale.x = 0.01;
        //     8------7
        //    /|     /|
        //   / |    / |
        //  5------6  |
        //  |  4---|--3
        //  | /    | /
        //  1------2
        // lenth ：2 3
        // width ：1 2
        // height：2 6
        std::vector<geometry_msgs::Point> vertices;
        
        // 角度转弧度
        double angle_rad = angle * M_PI / 180.0;
        double cos_a = cos(angle_rad);
        double sin_a = sin(angle_rad);

        // 计算立方体顶点（局部坐标绕中心旋转）
        std::vector<cv::Point3d> local_points = {
            {-width/2, -length/2, 0},
            { width/2, -length/2, 0},
            { width/2,  length/2, 0},
            {-width/2,  length/2, 0},
            {-width/2, -length/2, height},
            { width/2, -length/2, height},
            { width/2,  length/2, height},
            {-width/2,  length/2, height}
        };

        // 空置位
        geometry_msgs::Point p0;
        p0.x = 0;
        p0.y = 0;
        p0.z = 0+flag;
        vertices.push_back(p0);


        for (const auto& pt : local_points) {
            geometry_msgs::Point p;
            p.x = tx + pt.x * cos_a - pt.y * sin_a;
            p.y = ty + pt.x * sin_a + pt.y * cos_a;
            p.z = pt.z + flag;
            vertices.push_back(p);
        }

        CubeMarker.points.push_back(vertices[1]);
        CubeMarker.points.push_back(vertices[2]);
        CubeMarker.points.push_back(vertices[2]);
        CubeMarker.points.push_back(vertices[3]);
        CubeMarker.points.push_back(vertices[3]);
        CubeMarker.points.push_back(vertices[4]);
        CubeMarker.points.push_back(vertices[4]);
        CubeMarker.points.push_back(vertices[1]);

        CubeMarker.points.push_back(vertices[5]);
        CubeMarker.points.push_back(vertices[1]);
        CubeMarker.points.push_back(vertices[6]);
        CubeMarker.points.push_back(vertices[2]);
        CubeMarker.points.push_back(vertices[7]);
        CubeMarker.points.push_back(vertices[3]);
        CubeMarker.points.push_back(vertices[8]);
        CubeMarker.points.push_back(vertices[4]);

        CubeMarker.points.push_back(vertices[5]);
        CubeMarker.points.push_back(vertices[6]);
        CubeMarker.points.push_back(vertices[6]);
        CubeMarker.points.push_back(vertices[7]);
        CubeMarker.points.push_back(vertices[7]);
        CubeMarker.points.push_back(vertices[8]);
        CubeMarker.points.push_back(vertices[8]);
        CubeMarker.points.push_back(vertices[5]);

        publisher_CubeGT.publish(CubeMarker);

        row.clear();
        istr.clear();
        line.clear();
    }
}

void read_view(const std::string filePath, std::vector<cv::Mat>& views){
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

    std::vector<double> row;

    cv::Mat cam_pose_mat;
    views.clear();
    std::string line;

    while (getline(infile, line))
    {
        // std::cout<<"读取新的一行:"<<endl;

        std::istringstream istr(line);
        double time,tx,ty,tz,qx,qy,qz,qw;


        istr >> time;

        double temp;
        cv::Vec3d translation;  // 存储 tx, ty, tz
        cv::Vec4d quaternion;   // 存储 qx, qy, qz, qw
        istr >> temp;  translation[0] = temp;  //tx
        istr >> temp;  translation[1] = temp;  //ty
        istr >> temp;  translation[2] = temp;  //tz
        istr >> temp;  quaternion[0] = temp;  //qx
        istr >> temp;  quaternion[1] = temp;  //qy
        istr >> temp;  quaternion[2] = temp;  //qz
        istr >> temp;  quaternion[3] = temp;  //qw
        double w = quaternion[3];
        double x = quaternion[0];
        double y = quaternion[1];
        double z = quaternion[2];

        /// 计算旋转角度 theta
        double theta = 2.0 * std::acos(w);

        // 计算旋转轴 u
        double norm = std::sqrt(x*x + y*y + z*z);
        cv::Mat u = (cv::Mat_<double>(3, 1) << x/norm, y/norm, z/norm);

        // 计算旋转向量 r = theta * u
        cv::Mat r = theta * u;

        // 使用cv::Rodrigues将旋转向量转换为旋转矩阵
        cv::Mat rotation_matrix;
        cv::Rodrigues(r, rotation_matrix);

        cv::Mat rotation_matrix_32F;
        rotation_matrix.convertTo(rotation_matrix_32F, CV_32F); // 转换为 float

        // 构造 4x4 变换矩阵 (CV_32F)
        cv::Mat transform = cv::Mat::eye(4, 4, CV_32F); // 4x4 单位矩阵

        // 复制旋转部分
        rotation_matrix_32F.copyTo(transform(cv::Range(0, 3), cv::Range(0, 3))); 

        // 赋值平移向量 (确保数据类型匹配)
        transform.at<float>(0, 3) = static_cast<float>(translation[0]); // tx
        transform.at<float>(1, 3) = static_cast<float>(translation[1]); // ty
        transform.at<float>(2, 3) = static_cast<float>(translation[2]); // tz

        views.push_back(transform);

        row.clear();
        istr.clear();
        line.clear();
    }
}

void PublishCameras(const vector<cv::Mat> &VIEWs, int step = 15, bool only_draw_last_frame = false, int type = 0 /* 相机0  底盘1*/)
{
    if(type==0)
    {
        visualization_msgs::Marker mKeyFrames;
        mKeyFrames.header.frame_id = "world";
        mKeyFrames.ns = "KEYFRAMES";
        mKeyFrames.id=1;
        mKeyFrames.type = visualization_msgs::Marker::LINE_LIST;
        mKeyFrames.scale.x=0.02;
        mKeyFrames.pose.orientation.w=1.0;
        mKeyFrames.action=visualization_msgs::Marker::ADD;
        mKeyFrames.color.b=1.0f;
        mKeyFrames.color.a = 1.0;
        visualization_msgs::Marker mMST;
        mMST.header.frame_id = "world";
        mMST.ns = "MST";
        mMST.id=3;
        mMST.type = visualization_msgs::Marker::LINE_LIST;
        // mMST.scale.x=0.005;
        mMST.scale.x=0.02;
        mMST.pose.orientation.w=1.0;
        mMST.action=visualization_msgs::Marker::ADD;
        mMST.color.r=0.0f;
        mMST.color.b=0.0f;
        mMST.color.g=1.0f;
        mMST.color.a = 1.0;

        float d = 0.15;

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
            // double x_trans = 0-0.5;   // -0.5 用于椭球体观测示意图的paper做图  ;
            double x_trans = 0-0.5;   // -0.5 用于椭球体观测示意图的paper做图  ;
            msgs_o.x = ow.at<float>(0) + x_trans;
            msgs_o.y = ow.at<float>(1);
            msgs_o.z = ow.at<float>(2);
            msgs_p1.x = p1w.at<float>(0) + x_trans;
            msgs_p1.y = p1w.at<float>(1);
            msgs_p1.z = p1w.at<float>(2);
            msgs_p2.x = p2w.at<float>(0) + x_trans;
            msgs_p2.y = p2w.at<float>(1);
            msgs_p2.z = p2w.at<float>(2);
            msgs_p3.x = p3w.at<float>(0) + x_trans;
            msgs_p3.y = p3w.at<float>(1);
            msgs_p3.z = p3w.at<float>(2);
            msgs_p4.x = p4w.at<float>(0) + x_trans;
            msgs_p4.y = p4w.at<float>(1);
            msgs_p4.z = p4w.at<float>(2);

            if(!only_draw_last_frame){
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
            }
            else if(i==iend-step || i==0) {
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
            }
            

            if (i > step) {
                cv::Mat Owp = VIEWs[i - step].rowRange(0, 3).col(3).clone();//->GetCameraCenter();;
                geometry_msgs::Point msgs_op;
                msgs_op.x = Owp.at<float>(0) + x_trans;
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
    else if(type==1)
    {
        visualization_msgs::Marker mKeyFrames;
        float fCameraSize=0.04;
        mKeyFrames.header.frame_id = "world";
        mKeyFrames.ns = "baselink_my";
        mKeyFrames.id=1;
        mKeyFrames.type = visualization_msgs::Marker::LINE_LIST;
        mKeyFrames.scale.x=0.005;
        mKeyFrames.pose.orientation.w=1.0;
        mKeyFrames.action=visualization_msgs::Marker::ADD;
        mKeyFrames.color.b=1.0f;
        mKeyFrames.color.a = 1.0;
        visualization_msgs::Marker mMST;
        mMST.header.frame_id = "world";
        mMST.ns = "baselink_my_MST";
        mMST.id=3;
        mMST.type = visualization_msgs::Marker::LINE_LIST;
        mMST.scale.x=0.01;
        mMST.pose.orientation.w=1.0;
        mMST.action=visualization_msgs::Marker::ADD;
        mMST.color.r=0.0f;
        mMST.color.b=1.0f;
        mMST.color.g=0.0f;
        mMST.color.a = 1.0;

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

                // 计算相邻两个相机中心之间的距离，并累加到 trajectory_length
                double dx = msgs_o.x - msgs_op.x;
                double dy = msgs_o.y - msgs_op.y;
                double dz = msgs_o.z - msgs_op.z;
                my_trajectory_length += std::sqrt(dx * dx + dy * dy + dz * dz);
                
            }
        }

        mKeyFrames.header.stamp = ros::Time::now();
        //mCovisibilityGraph.header.stamp = ros::Time::now();
        mMST.header.stamp = ros::Time::now();

        // publisher_KF.publish(mKeyFrames);
        publisher_baselink_trajectory.publish(mMST);
    }
    else if(type==2)
    {
        visualization_msgs::Marker mKeyFrames;
        float fCameraSize=0.04;
        mKeyFrames.header.frame_id = "world";
        mKeyFrames.ns = "baselink_direct";
        mKeyFrames.id=1;
        mKeyFrames.type = visualization_msgs::Marker::LINE_LIST;
        mKeyFrames.scale.x=0.01;
        mKeyFrames.pose.orientation.w=1.0;
        mKeyFrames.action=visualization_msgs::Marker::ADD;
        mKeyFrames.color.b=1.0f;
        mKeyFrames.color.a = 1.0;
        visualization_msgs::Marker mMST;
        mMST.header.frame_id = "world";
        mMST.ns = "baselink_direct_MST";
        mMST.id=3;
        mMST.type = visualization_msgs::Marker::LINE_LIST;
        mMST.scale.x=0.005;
        mMST.pose.orientation.w=1.0;
        mMST.action=visualization_msgs::Marker::ADD;
        // mMST.color.r=0.0f;
        // mMST.color.b=0.0f;
        // mMST.color.g=1.0f;
        mMST.color.a = 1.0;

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

                // 计算相邻两个相机中心之间的距离，并累加到 trajectory_length
                double dx = msgs_o.x - msgs_op.x;
                double dy = msgs_o.y - msgs_op.y;
                double dz = msgs_o.z - msgs_op.z;
                direct_trajectory_length += std::sqrt(dx * dx + dy * dy + dz * dz);
            }
        }

        mKeyFrames.header.stamp = ros::Time::now();
        //mCovisibilityGraph.header.stamp = ros::Time::now();
        mMST.header.stamp = ros::Time::now();

        // publisher_KF.publish(mKeyFrames);
        publisher_baselink_trajectory.publish(mMST);
    }
    else{
        std::cout<<"[PublishCameras] 无效的Views类型"<<std::endl;
    }
    
}


void publishPlanes(const std::string &file_path, ros::Publisher &marker_pub) {
    // 打开文件
    std::ifstream file(file_path);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open file: %s", file_path.c_str());
        return;
    }

    // MarkerArray 用于存储所有 Marker
    visualization_msgs::MarkerArray marker_array;

    // 读取文件中的每一行
    std::string line;
    int id = 0; // Marker 的唯一 ID
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        if (line.empty() || line[0] == '#') {
            continue; // 跳过空行或注释行
        }

        // 解析数据
        float x, y, z, w, l, h, yaw, alpha;
        int color;
        if (!(iss >> x >> y >> z >> w >> l >> h >> yaw >> color >> alpha)) {
            ROS_WARN("Invalid line format, skipping: %s", line.c_str());
            continue;
        }

        // 创建 Marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world"; // 固定框架
        marker.header.stamp = ros::Time::now();
        marker.ns = "cubes";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        // 设置位置
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;

        // 设置旋转 (yaw 转为四元数)
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = std::sin(yaw/180*M_PI / 2.0);
        marker.pose.orientation.w = std::cos(yaw/180*M_PI / 2.0);

        // 设置尺寸
        marker.scale.x = w;
        marker.scale.y = l;
        marker.scale.z = h;

        // 设置颜色
        if (color == 2) {
            marker.color.r = 1.0f; 
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
        } else if(color == 1) {
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 1.0f; 
        }
        marker.color.a = alpha; // 不透明

        // 将 Marker 添加到 MarkerArray
        marker_array.markers.push_back(marker);
    }
    file.close();

    // 发布 MarkerArray
    publisher_MHPlanes.publish(marker_array);
    ROS_INFO("Published %lu cubes to RViz.", marker_array.markers.size());
}

geometry_msgs::Point transformPointToWorld(const geometry_msgs::Point& point_object, 
                                           double tx, double ty, double tz,
                                           double qx, double qy, double qz, double qw) {
    // 1️⃣ 创建 TF2 变换
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(tx, ty, tz));  // 设置平移
    transform.setRotation(tf2::Quaternion(qx, qy, qz, qw));  // 设置旋转

    // 2️⃣ 物体坐标系下的点
    tf2::Vector3 point_local(point_object.x, point_object.y, point_object.z);

    // 3️⃣ 进行变换
    tf2::Vector3 point_transformed = transform * point_local;

    // 4️⃣ 结果转换回 geometry_msgs::Point
    geometry_msgs::Point point_world;
    point_world.x = point_transformed.x();
    point_world.y = point_transformed.y();
    point_world.z = point_transformed.z();

    return point_world;
}

geometry_msgs::Point transformPointToWorld_scale(const geometry_msgs::Point& point_world, 
                                           double tx, double ty, double tz,
                                           const Eigen::Quaterniond& orientation, // ✅ 四元数
                                           double degree /* 绕z轴的角度 */ ,
                                           double scale_x /* 扩大的尺度 */,
                                           double scale_y /* 扩大的尺度 */,
                                           double scale_z /* 扩大的尺度 */                                
                                           ) {
    Eigen::Vector3d eigen_point(point_world.x, point_world.y, point_world.z);
    // 使用四元数旋转点
    Eigen::Vector3d rotated_point = orientation.inverse() * eigen_point;

    geometry_msgs::Point point_object;
    point_object.x = rotated_point.x();
    point_object.y = rotated_point.y();
    point_object.z = rotated_point.z();
                                            
    // 1️⃣ 计算四元数（绕 Z 轴旋转 degree 角度）
    double radian = degree * M_PI / 180.0;  // 角度转弧度
    tf2::Quaternion manual_tf2_q;
    manual_tf2_q.setRPY(0, radian, 0); 
    Eigen::Quaterniond eigen_q(manual_tf2_q.w(), manual_tf2_q.x(), manual_tf2_q.y(), manual_tf2_q.z());
    Eigen::Quaterniond result = orientation * eigen_q;
    tf2::Quaternion q(result.x(),
                                   result.y(),
                                   result.z(),
                                   result.w());

    // 1️⃣ 创建 TF2 变换
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(tx, ty, tz));  // 设置平移
    transform.setRotation(q);  // 设置旋转

    // 2️⃣ 物体坐标系下的点
    tf2::Vector3 point_local(point_object.x * scale_x, 
                             point_object.y * scale_y, 
                             point_object.z * scale_z);

    // 3️⃣ 进行变换
    tf2::Vector3 point_transformed = transform * point_local;

    // 4️⃣ 结果转换回 geometry_msgs::Point
    geometry_msgs::Point point_world_new;
    point_world_new.x = point_transformed.x();
    point_world_new.y = point_transformed.y();
    point_world_new.z = point_transformed.z();

    return point_world_new;
}



int main(int argc, char **argv) {

    
    ros::init ( argc, argv, "show_local_object" );
    ros::NodeHandle nh;
    publisher_CubeGT = nh.advertise<visualization_msgs::Marker>("/object_cube_groudtruth", 1000);
    publisher_SdfObject = nh.advertise<visualization_msgs::Marker>("/local_objects", 1000);
    publisher_points = nh.advertise<visualization_msgs::Marker>("/Point", 1000);
    publisher_points = nh.advertise<visualization_msgs::Marker>("/Point", 1000);
    publisher_KF = nh.advertise<visualization_msgs::Marker>("/KeyFrame", 1000);
    publisher_baselink_trajectory = nh.advertise<visualization_msgs::Marker>("/baselink_trajectory", 1000);
    publisher_ellipsoid  = nh.advertise<visualization_msgs::Marker>("ellipsoid_marker", 1);
    publisher_MHPlanes = nh.advertise<visualization_msgs::MarkerArray>("MHPlanes", 10);
    ros::start();


    // 提前获得所有物体
    std::string targetpath = root_path + "/objects";  // 目标文件夹
    std::vector<std::string> object_files_list;
    object_files_list = getFilesInDirectory(targetpath);
    mvObjectColors.push_back(std::tuple<float, float, float>({210. / 255., 245. / 255., 60. / 255.}));  //lime  0
    mvObjectColors.push_back(std::tuple<float, float, float>({60. / 255., 180. / 255., 75. / 255.}));   // green  1
    mvObjectColors.push_back(std::tuple<float, float, float>({0., 0., 255. / 255.}));	 // blue  2
    mvObjectColors.push_back(std::tuple<float, float, float>({255. / 255., 0, 255. / 255.}));   // Magenta  3
    mvObjectColors.push_back(std::tuple<float, float, float>({255. / 255., 165. / 255., 0}));   // orange 4
    mvObjectColors.push_back(std::tuple<float, float, float>({128. / 255., 0, 128. / 255.}));   //purple 5
    mvObjectColors.push_back(std::tuple<float, float, float>({0., 255. / 255., 255. / 255.}));   //cyan 6
    mvObjectColors.push_back(std::tuple<float, float, float>({230. / 255., 0., 0.}));	 // red  7
    mvObjectColors.push_back(std::tuple<float, float, float>({250. / 255., 190. / 255., 190. / 255.})); //pink  8
    mvObjectColors.push_back(std::tuple<float, float, float>({0., 128. / 255., 128. / 255.}));   //Teal  9


     
    ros::Rate rate(10);
    while (nh.ok())
    {

        // 发布物体
        int Id = 0;
        for (const auto& filename : object_files_list) {
                std::cout << "读取物体文件: " << filename << std::endl;
                std::ifstream file(filename);
            
            if (!file) {
                std::cerr << "无法打开文件: " << filename << std::endl;
                return 1;
            }

            std::string line;
            bool firstLine = true;

            // 创建一个 Marker 消息
            visualization_msgs::Marker mesh_marker;
            mesh_marker.header.frame_id = "world"; // 设置网格的参考坐标系
            mesh_marker.header.stamp = ros::Time::now();
            mesh_marker.ns = "sdf_mesh";
            mesh_marker.id = Id;
            mesh_marker.type = visualization_msgs::Marker::TRIANGLE_LIST; // 三角形列表类型
            mesh_marker.action = visualization_msgs::Marker::ADD;


            // 设置标记的缩放
            mesh_marker.scale.x = 1.0;  
            mesh_marker.scale.y = 1.0;
            mesh_marker.scale.z = 1.0;

            std::getline(file, line);
            std::stringstream ss(line);
            double mnId, label,    tx,ty,tz,  qx,qy,qz,qw,  w,h,l,   degree=1,   scale_x=1,scale_y=1,scale_z=1;  
            double color = 0;
            ss >> mnId; ss >> label;
            ss >> tx; ss >> ty; ss >> tz;   Eigen::Vector3d center(tx, ty, tz);
            ss >> qx; ss >> qy; ss >> qz; ss >> qw;  Eigen::Quaterniond quaternion(qw, qx, qy, qz); // 顺序：w, x, y, z
            ss >> w; ss >> h; ss >> l;  
            ss >> degree;
            ss >> scale_x; ss >> scale_y; ss >> scale_z;  Eigen::Vector3d radii(w*scale_x/2.0, l*scale_z/2.0, h*scale_y/2.0);
            ss >> color;  //设定颜色的种类

            // 设置标记的颜色和透明度
            mesh_marker.color.a = 1; //0.8f; // 设置透明度为 1.0（不透明）
            if(label==-1)
                mesh_marker.color.a = 0.9f; // 设置透明度为 1.0（不透明）
            mesh_marker.color.r =  std::get<0>(mvObjectColors[int(mnId) % 10]);
            mesh_marker.color.g =  std::get<1>(mvObjectColors[int(mnId) % 10]);
            mesh_marker.color.b =  std::get<2>(mvObjectColors[int(mnId) % 10]);
            std_msgs::ColorRGBA color_ellip;
            if(label==0){
                color_ellip.r = .0f;
                color_ellip.g = 0.0f;
                color_ellip.b = 1.0f;
                color_ellip.a = 1.0f;
            }
            else{
                color_ellip.r = 1.0f;
                color_ellip.g = 0.0f;
                color_ellip.b = 0.0f;
                color_ellip.a = 1.0f;
            }
            

            // 物体内部的点
            while (std::getline(file, line)) {
                
                std::stringstream ss(line);
                
                double temp;
                
                geometry_msgs::Point point_object;
                ss >> temp; point_object.x = temp;
                ss >> temp; point_object.y = temp;
                ss >> temp; point_object.z = temp;
                
                geometry_msgs::Point point_world = transformPointToWorld_scale(point_object, tx, ty, tz, quaternion, degree, scale_x,scale_y,scale_z);
                
                mesh_marker.points.push_back(point_world);

            }
            file.close();

            // 发布网格物体
            publisher_SdfObject.publish(mesh_marker);
            publishEllipsoidWireframe(publisher_ellipsoid, center, radii, quaternion, color_ellip, Id, "world");
            Id++;
        }  

        // 发布点
        string points_file_name = root_path + "/points.txt";
        std::ifstream points_file(points_file_name); 
        if (!points_file) {
            std::cerr << "无法打开文件: " << points_file_name << std::endl;
            return 1;
        }
        visualization_msgs::Marker mPoints;
        float fPointSize=0.018;
        mPoints.header.frame_id =  "world";
        mPoints.ns = "POINTS";
        mPoints.id=0;
        mPoints.type = visualization_msgs::Marker::POINTS;
        mPoints.scale.x=fPointSize;
        mPoints.scale.y=fPointSize;
        mPoints.pose.orientation.w=1.0;
        mPoints.action=visualization_msgs::Marker::ADD;
        mPoints.color.a = 1;
        // mPoints.color.r = 1.0;

        std::string line;
        while (std::getline(points_file, line)) {

            std::stringstream ss(line);
            double temp;
        
            geometry_msgs::Point point;
            ss >> temp; point.x = temp;
            ss >> temp; point.y = temp;
            ss >> temp; point.z = temp;

            mPoints.points.push_back(point);

        }
        points_file.close();
        publisher_points.publish(mPoints);



        // 发布轨迹真值
        string cam_traj_file_name = root_path + "/cam_traj.txt";
        std::vector<cv::Mat> camera_groundTruths;
        read_view(cam_traj_file_name, camera_groundTruths);
        std::cout<<"Publish Camera GroundTruth"<<endl;
        int step = 1;
        if(argc > 1 )
        {
            step = atoi(argv[1]);
        }
        PublishCameras(camera_groundTruths,step, false, 0);

        
        // 发布my底盘轨迹
        my_trajectory_length=0;
        string my_baselink_traj_file_name = root_path + "/baselink_traj/my.txt";
        std::vector<cv::Mat> my_baselink_groundTruths;
        read_view(my_baselink_traj_file_name, my_baselink_groundTruths);
        std::cout<<"Publish my_baselink GroundTruth"<<endl;
        step = 1;
        PublishCameras(my_baselink_groundTruths, step, false, 1);

        // 发布my底盘轨迹
        direct_trajectory_length=0;
        string direct_baselink_traj_file_name = root_path + "/baselink_traj/direct.txt";
        std::vector<cv::Mat> direct_baselink_groundTruths;
        read_view(direct_baselink_traj_file_name, direct_baselink_groundTruths);
        std::cout<<"Publish baselink GroundTruth"<<endl;
        step = 1;
        PublishCameras(direct_baselink_groundTruths, step, false, 2);

        std::cout<<"[轨迹长度]  my_trajectory_length:"<<my_trajectory_length<<",  direct_trajectory_length:"<<direct_trajectory_length
        <<",  增长比例："<< (direct_trajectory_length-my_trajectory_length) / my_trajectory_length
        <<std::endl;


        // 发布物体cube真值
        string object_groundtruth_file_name = root_path + "/object_groundtruth.txt";
        PublishObjectGroundtruth(object_groundtruth_file_name);

        // 曼哈顿平面
        // 文件路径
        std::string file_path = root_path + "/planes.txt"; // 替换为实际的文件路径
        publishPlanes(file_path, publisher_MHPlanes); // 调用整合函数
        rate.sleep();
    }



    ros::shutdown();
    return 0;
}
