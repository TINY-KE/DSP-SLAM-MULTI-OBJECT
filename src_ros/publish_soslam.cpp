#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

struct Ellipsoid {
    int id;
    double x, y, z;
    double roll, pitch, yaw;
    double scale_x, scale_y, scale_z;
    int color_index;
};

std::vector<std::tuple<float, float, float>> colors = {
    {1.0f, 0.0f, 0.0f}, // red
    {0.0f, 1.0f, 0.0f}, // green
    {0.0f, 0.0f, 1.0f}, // blue
    {1.0f, 1.0f, 0.0f}, // yellow
    {1.0f, 0.0f, 1.0f}, // magenta
    {0.0f, 1.0f, 1.0f}, // cyan
    {1.0f, 0.5f, 0.0f}, // orange
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ellipsoids_marker_publisher");
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Rate r(1);

    std::vector<Ellipsoid> ellipsoids = {
        {10, -1.212478, -0.716595, -0.715631, -1.572374, -0.452684, 0.002137, 0.496801, 0.347907, 0.443791, 56},
        {28, -1.664205, -0.018837,  0.675063, -1.572215, -0.019002, 0.001474, 0.044691, 0.495878, 0.333914, 62},
        {29, -1.422469, -0.549884,  0.660766,  1.572226,  0.125225, -3.139966, 0.192143, 0.135494, 0.176544, 75},
        {50, -1.336246, -1.072182,  2.057645,  1.573014, -0.876649,  3.141335, 0.163280, 0.105688, 0.116244, 75},
        {54,  1.644554, -0.693747, -0.912605,  1.581933, -1.245386,  3.134294, 0.345730, 0.313481, 0.410422, 56}
    };

    while (ros::ok()) {
        int i = 0;
        for (const auto& e : ellipsoids) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.ns = "ellipsoids";
            marker.id = e.id;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = e.x;
            marker.pose.position.y = e.y;
            marker.pose.position.z = e.z;

            tf2::Quaternion q;
            q.setRPY(e.roll, e.pitch, e.yaw);
            marker.pose.orientation.x = q.x();
            marker.pose.orientation.y = q.y();
            marker.pose.orientation.z = q.z();
            marker.pose.orientation.w = q.w();

            marker.scale.x = e.scale_x;
            marker.scale.y = e.scale_y;
            marker.scale.z = e.scale_z;

            auto c = colors[e.color_index % colors.size()];
            marker.color.r = std::get<0>(c);
            marker.color.g = std::get<1>(c);
            marker.color.b = std::get<2>(c);
            marker.color.a = 0.8;

            marker.lifetime = ros::Duration();

            marker_pub.publish(marker);
            ++i;
        }
        r.sleep();
    }

    return 0;
}