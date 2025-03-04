
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

geometry_msgs::Point transformPointToWorld_scale(const geometry_msgs::Point& point_object, 
                                           double tx, double ty, double tz,
                                           double degree /* 绕z轴的角度 */ ,
                                           double scale_x /* 扩大的尺度 */,
                                           double scale_y /* 扩大的尺度 */,
                                           double scale_z /* 扩大的尺度 */                                
                                           ) {
    // 1️⃣ 计算四元数（绕 Z 轴旋转 degree 角度）
    double radian = degree * M_PI / 180.0;  // 角度转弧度
    tf2::Quaternion q;
    q.setRPY(0, 0, radian); // 绕 Z 轴旋转

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
    geometry_msgs::Point point_world;
    point_world.x = point_transformed.x();
    point_world.y = point_transformed.y();
    point_world.z = point_transformed.z();

    return point_world;
}



int main(int argc, char **argv) {

    
    ros::init ( argc, argv, "show_local_object" );
    ros::NodeHandle nh;
    publisher_GT = nh.advertise<visualization_msgs::Marker>("/objectmap_groudtruth", 1000);
    publisher_SdfObject = nh.advertise<visualization_msgs::Marker>("/local_objects", 1000);
    publisher_points = nh.advertise<visualization_msgs::Marker>("/Point", 1000);
    publisher_points = nh.advertise<visualization_msgs::Marker>("/Point", 1000);
    publisher_KF = nh.advertise<visualization_msgs::Marker>("/KeyFrame", 1000);
    ros::start();


    // 提前获得所有物体
    std::string targetpath = "/home/robotlab/ws_3d_vp/src/QSP-SLAM-my/eval/show/objects";  // 目标文件夹
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

            // 设置标记的颜色和透明度
            mesh_marker.color.a = 1.0f; // 设置透明度为 1.0（不透明）
            mesh_marker.color.r =  std::get<0>(mvObjectColors[Id % 10]);
            mesh_marker.color.g =  std::get<1>(mvObjectColors[Id % 10]);
            mesh_marker.color.b =  std::get<2>(mvObjectColors[Id % 10]);

            // 设置标记的缩放
            mesh_marker.scale.x = 1.0;
            mesh_marker.scale.y = 1.0;
            mesh_marker.scale.z = 1.0;

            std::getline(file, line);
            std::stringstream ss(line);
            double mnId, label,    tx,ty,tz,  qx,qy,qz,qw,  w,h,l,   degree=1,   scale_x=1,scale_y=1,scale_z=1;  
            ss >> mnId; ss >> label;
            ss >> tx; ss >> ty; ss >> tz;
            ss >> qx; ss >> qy; ss >> qz; ss >> qw;
            ss >> w; ss >> h; ss >> l;
            ss >> degree;
            ss >> scale_x; ss >> scale_y; ss >> scale_z;


            while (std::getline(file, line)) {
                
                std::stringstream ss(line);
                
                double temp;
                
                geometry_msgs::Point point_object;
                ss >> temp; point_object.x = temp;
                ss >> temp; point_object.y = temp;
                ss >> temp; point_object.z = temp;
                
                geometry_msgs::Point point_world = transformPointToWorld_scale(point_object, tx, ty, tz, degree, scale_x,scale_y,scale_z);
                
                mesh_marker.points.push_back(point_world);

            }
            file.close();

            // 发布网格物体
            publisher_SdfObject.publish(mesh_marker);
            Id++;
        }  

        // 发布点
        string points_file_name = "/home/robotlab/ws_3d_vp/src/QSP-SLAM-my/eval/show/points.txt";
        std::ifstream points_file(points_file_name); 
        if (!points_file) {
            std::cerr << "无法打开文件: " << points_file_name << std::endl;
            return 1;
        }
        visualization_msgs::Marker mPoints;
        float fPointSize=0.015;
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
        string cam_traj_file_name = "/home/robotlab/ws_3d_vp/src/QSP-SLAM-my/eval/show/cam_traj.txt";
        std::vector<cv::Mat> cameras;
        read_view(cam_traj_file_name, cameras);
        std::cout<<"PublishCameras 0"<<endl;
        int step = 15;
        if(argc > 1 )
        {
            step = atoi(argv[1]);
        }
        PublishCameras(cameras,step);

        rate.sleep();
    }



    ros::shutdown();
    return 0;
}
