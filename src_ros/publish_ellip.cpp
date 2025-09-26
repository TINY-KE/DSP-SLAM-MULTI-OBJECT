#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <Eigen/Dense>
void publishEllipsoidWireframe(ros::Publisher& pub,
                                const Eigen::Vector3d& center,
                                const Eigen::Vector3d& radii,
                                const Eigen::Quaterniond& orientation, // ✅ 四元数
                                const std_msgs::ColorRGBA& color,
                                const std::string& frame_id = "map")
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "ellipsoid";
    marker.id = 0;
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

int main(int argc, char** argv) {
    ros::init(argc, argv, "ellipsoid_marker_node");
    ros::NodeHandle nh;

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("ellipsoid_marker", 1);

    ros::Rate rate(10);

    // 椭球中心和半轴长度
    Eigen::Vector3d center(0.0, 0.0, 0.0);
    Eigen::Vector3d radii(1.0, 0.5, 0.3);

    std_msgs::ColorRGBA color;
    color.r = 0.0f;
    color.g = 1.0f;
    color.b = 0.0f;
    color.a = 1.0f;

    // 四元数（单位朝向）
    Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitY());

    while (ros::ok()) {
        publishEllipsoidWireframe(marker_pub, center, radii, q, color, "world");
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


