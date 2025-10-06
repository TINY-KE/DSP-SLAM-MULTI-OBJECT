#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>

// ✅ 你的函数（保持不变）
void publishEllipsoidWireframe(ros::Publisher& pub,
                                const Eigen::Vector3d& center,
                                const Eigen::Vector3d& radii,
                                const Eigen::Quaterniond& orientation,
                                const std_msgs::ColorRGBA& color,
                                const std::string& frame_id = "world",
                                int marker_id = 0)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "ellipsoid";
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.005;  // 线宽
    marker.color = color;

    const int segments_per_circle = 20;
    const int num_latitude = 20;
    const int num_longitude = 20;
    const double dtheta = 2 * M_PI / segments_per_circle;
    const double dphi = M_PI / num_latitude;

    // 纬线
    for (int i = 1; i < num_latitude; ++i) {
        double phi = i * dphi;
        double z = cos(phi);
        double r = sin(phi);
        for (int j = 0; j < segments_per_circle; ++j) {
            double theta1 = j * dtheta;
            double theta2 = (j + 1) * dtheta;
            Eigen::Vector3d p1_unit(r * cos(theta1), r * sin(theta1), z);
            Eigen::Vector3d p2_unit(r * cos(theta2), r * sin(theta2), z);
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

    // 经线
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

// ✅ 主程序：读取 txt 数据并发布多个椭球体
int main(int argc, char** argv) {
    ros::init(argc, argv, "ellipsoid_marker_node");
    ros::NodeHandle nh;

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("soslam_ellipsoid", 100);

    ros::Rate rate(1);  // 每秒刷新一次

    int dateset_type = std::atoi(argv[1]);
    std::string file_path;
    if(dateset_type == 1)
        file_path = "/home/robotlab/dataset/ICL-NUIM/living_room_traj2n_frei_png/ellipsoid-soslam/ellipsoids.txt";
    else if(dateset_type == 2)
        file_path = "/home/robotlab/dataset/Replica-Dataset-results/hotel_0_640/ellipsoid-soslam/ellipsoids.txt";
    else if(dateset_type == 3){
        file_path = "/home/robotlab/dataset/MySimDataset/gazebo_dataset_10/ellipsoid-soslam/ellipsoids.txt";
    }

    while (ros::ok()) {
        std::ifstream infile(file_path);
        if (!infile.is_open()) {
            ROS_ERROR("Failed to open file: %s", file_path.c_str());
            return 1;
        }

        std::string line;
        int marker_id = 0;
        while (std::getline(infile, line)) {
            if (line.empty()) continue;

            std::istringstream iss(line);
            int id, label;
            double x, y, z, roll, pitch, yaw, sx, sy, sz;
            if (!(iss >> id >> label >> x >> y >> z >> roll >> pitch >> yaw >> sx >> sy >> sz)) {
                ROS_WARN("Failed to parse line: %s", line.c_str());
                continue;
            }

            Eigen::Vector3d center(x, y, z);
            Eigen::Vector3d radii(sx, sy, sz);

            // RPY 转四元数
            Eigen::AngleAxisd rollAngle(roll/180*M_PI, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(pitch/180*M_PI, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(yaw/180*M_PI, Eigen::Vector3d::UnitZ());
            Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

            std_msgs::ColorRGBA color;
            color.r = 1;
            color.g = 0;
            color.b = 0;
            color.a = 1.0f;

            publishEllipsoidWireframe(marker_pub, center, radii, q, color, "world", marker_id++);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}