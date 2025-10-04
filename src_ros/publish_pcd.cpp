#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

// ./src_ros/publish_pcd /home/robotlab/dataset/ICL-NUIM/living_room_traj2n_frei_png/dataset.pcd
int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "pcd_publisher");
    ros::NodeHandle nh;

    // 创建 ROS 发布器
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_topic", 1);

    // 创建 PCL 点云对象
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // 检查是否提供了 PCD 文件路径
    if (argc < 2) {
        ROS_ERROR("Please provide the path to a PCD file.");
        return -1;
    }

    // 加载 PCD 文件
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) == -1) {
        ROS_ERROR("Couldn't read the PCD file.");
        return -1;
    }

    // Eigen::Matrix4d transform;
    // transform << 0, 0, 1, 0,
    //      -1,  0,  0, 0,
    //       0, -1,  0, -1.17,
    //       0,  0,  0, 1;
    // Eigen::Matrix4d transform_inverse = Eigen::Matrix4d::Identity();
    // transform_inverse.block<3, 3>(0, 0) = transform.block<3, 3>(0, 0).transpose();  // Rᵀ
    // transform_inverse.block<3, 1>(0, 3) = -transform.block<3, 3>(0, 0).transpose() * transform.block<3, 1>(0, 3);  // -Rᵀ * t
    // // Matrix4d transform = Tre.inverse().to_homogeneous_matrix();

    // pcl::transformPointCloud (*cloud, *cloud, transform);

    // // 颠倒 z 轴和 y 轴
    // for (auto& point : cloud->points) {
    //     point.z = -point.z;  // 颠倒 z 轴
    //     // point.y = -point.y;  // 颠倒 y 轴
    //     point.x += 2.2;
    // }

    pcl::PointCloud<pcl::PointXYZRGB>* pCloudFiltered = new pcl::PointCloud<pcl::PointXYZRGB>;
    int num = cloud->size();
    for(int i=0;i<num;i++)
    {
        pcl::PointXYZRGB p = (*cloud)[i];
        Eigen::Vector3d center; center << p.x, p.y, p.z;

        double height = p.z;
        double y_dis = p.y;
        double x_dis = p.x;
        // if(height < dis_thresh && y_dis > -2)  // 过滤掉过高的点
        
        if(height > 1.1 && x_dis > 0.4)  // 过滤掉过高的点
        {
            continue;
        }
        if(y_dis > -2)
            pCloudFiltered->push_back(p);
    }
    *cloud = *pCloudFiltered;
    ROS_INFO("Loaded %d points from %s", (int)cloud->points.size(), argv[1]);

    // std::string output_file = "/home/robotlab/ws_ellipsoid_dsp/src/DSP-SLAM-MULTI-OBJECT/src_ros/modified_cloud.pcd";  // 输出文件名
    // if (pcl::io::savePCDFileASCII(output_file, *cloud) == -1) {
    //     std::cerr << "Failed to save the modified point cloud." << std::endl;
    //     return -1;
    // }

    // std::cout << "Saved modified point cloud to " << output_file << std::endl;

    // 转换到 ROS 的 sensor_msgs::PointCloud2 消息格式
    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = "world";  // 设置坐标系

    // 手动填充 sensor_msgs::PointCloud2 消息
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.height = 1;  // 无组织点云
    cloud_msg.width = cloud->points.size();
    cloud_msg.is_dense = true;

    // 设置字段
    cloud_msg.fields.resize(4);
    cloud_msg.fields[0].name = "x";
    cloud_msg.fields[0].offset = 0;
    cloud_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[0].count = 1;

    cloud_msg.fields[1].name = "y";
    cloud_msg.fields[1].offset = 4;
    cloud_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[1].count = 1;

    cloud_msg.fields[2].name = "z";
    cloud_msg.fields[2].offset = 8;
    cloud_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[2].count = 1;

    cloud_msg.fields[3].name = "rgb";
    cloud_msg.fields[3].offset = 12;
    cloud_msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[3].count = 1;

    // 设置点云步长
    cloud_msg.point_step = 16;  // 每个点占用 16 字节
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;

    // 分配数据存储空间
    cloud_msg.data.resize(cloud_msg.row_step * cloud_msg.height);

    // 填充点云数据
    uint8_t* ptr = cloud_msg.data.data();
    for (const auto& point : cloud->points) {
        memcpy(ptr, &point.x, sizeof(float));  // x
        ptr += 4;
        memcpy(ptr, &point.y, sizeof(float));  // y
        ptr += 4;
        memcpy(ptr, &point.z, sizeof(float));  // z
        ptr += 4;
        memcpy(ptr, &point.rgb, sizeof(float));  // rgb
        ptr += 4;
    }

    // 发布点云消息
    ros::Rate loop_rate(10);  // 10 Hz
    while (ros::ok()) {
        cloud_msg.header.stamp = ros::Time::now();  // 更新时间戳
        cloud_pub.publish(cloud_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}