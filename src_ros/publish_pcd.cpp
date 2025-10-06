#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>

pcl::PointCloud<pcl::PointXYZRGB>*  filterCloudAsHeight_1(pcl::PointCloud<pcl::PointXYZRGB>* pCloud){
    pcl::PointCloud<pcl::PointXYZRGB>* pCloudFiltered = new pcl::PointCloud<pcl::PointXYZRGB>;
    int num = pCloud->size();
    for(int i=0;i<num;i++)
    {
        pcl::PointXYZRGB p = (*pCloud)[i];
        Eigen::Vector3d center; center << p.x, p.y, p.z;

        double height = p.z;
        double y_dis = p.y;
        double x_dis = p.x;
        // if(height < dis_thresh && y_dis > -2)  
        
        // ICL 电视
        // if(height > 1.1 && x_dis > 0.4)  
        // {
        //     continue;
        // }
        // if(y_dis > -2)

        // 实验对比
        if( x_dis < -1.45)  
        {
            continue;
        }

        // ICL 沙发侧面
        if(height < 2.3)  // 过滤掉过高的点
            pCloudFiltered->push_back(p);
    }
    return pCloudFiltered;
}

pcl::PointCloud<pcl::PointXYZRGB>*  filterCloudAsHeight_2(pcl::PointCloud<pcl::PointXYZRGB>* pCloud){
    pcl::PointCloud<pcl::PointXYZRGB>* pCloudFiltered = new pcl::PointCloud<pcl::PointXYZRGB>;
    int num = pCloud->size();
    for(int i=0;i<num;i++)
    {
        pcl::PointXYZRGB p = (*pCloud)[i];
        Eigen::Vector3d center; center << p.x, p.y, p.z;

        double height = p.z;
        double y_dis = p.y;
        double x_dis = p.x;
        // if(height < dis_thresh && y_dis > -2)  // 过滤掉过高的点
        
        if(y_dis < -0.53)  // 过滤掉过高的点
        {
            continue;
        }

        // ICL 沙发侧面
        if(height < 2.6)  {
            // p.z -= 0.2;
            pCloudFiltered->push_back(p);
        } 
    }
    return pCloudFiltered;
}

pcl::PointCloud<pcl::PointXYZRGB>*  filterCloudAsHeight_3(pcl::PointCloud<pcl::PointXYZRGB>* pCloud){
    pcl::PointCloud<pcl::PointXYZRGB>* pCloudFiltered = new pcl::PointCloud<pcl::PointXYZRGB>;
    
    // 地面
    // // 1. 添加地面点云
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    // double x_min = -8.0, x_max = 4.0;  // x 范围
    // double y_min = -2.0, y_max = 4.0;  // y 范围
    // double z_ground = -0.07;             // 地面高度
    // double resolution = 0.01;          // 地面点云分辨率（步长）

    // for (double x = x_min; x <= x_max; x += resolution) {
    //     for (double y = y_min; y <= y_max; y += resolution) {
    //         pcl::PointXYZRGB groundPoint;
    //         groundPoint.x = x;
    //         groundPoint.y = y;
    //         groundPoint.z = z_ground;

    //         // 设置为黑色的 RGB 值
    //         groundPoint.r = 160;
    //         groundPoint.g = 160;
    //         groundPoint.b = 160;

    //         groundCloud->push_back(groundPoint);
    //     }
    // }

    // *pCloud += *groundCloud;

    int num = pCloud->size();
    for(int i=0;i<num;i++)
    {
        pcl::PointXYZRGB p = (*pCloud)[i];
        Eigen::Vector3d center; center << p.x, p.y, p.z;

        double height = p.z;
        double y_dis = p.y;
        double x_dis = p.x;
        // if(height < dis_thresh && y_dis > -2)  // 过滤掉过高的点
        
        if(x_dis < -7.2 && y_dis < -2 )  // 过滤掉过高的点
        {
            continue;
        }
        
        if(y_dis < -2 )  // 过滤掉过高的点
        {
            continue;
        }

        if(x_dis < 2 && x_dis > -2 && y_dis < 1 && y_dis > 0 && height > 1.5) 
        {
            continue;
        }

        if(height < -0.08) 
        {
            continue;
        }
        
        // ICL 沙发侧面
        if(height < 1.8) {
            // p.z -= 0.2;
            pCloudFiltered->push_back(p);
        } 
    }
    return pCloudFiltered;
}

pcl::PointCloud<pcl::PointXYZRGB>*  filterCloudAsHeight_4(pcl::PointCloud<pcl::PointXYZRGB>* pCloud){
    pcl::PointCloud<pcl::PointXYZRGB>* pCloudFiltered = new pcl::PointCloud<pcl::PointXYZRGB>;
    
    


    int num = pCloud->size();
    for(int i=0;i<num;i++)
    {
        pcl::PointXYZRGB p = (*pCloud)[i];
        Eigen::Vector3d center; center << p.x, p.y, p.z;

        double height = p.z;
        double y_dis = p.y;
        double x_dis = p.x;
        // if(height < dis_thresh && y_dis > -2)  // 过滤掉过高的点
        
        // if(y_dis < -0.53)  // 过滤掉靠墙的点
        // {
        //     continue;
        // }
        // if(x_dis < 1.7)  
        // {
        //     continue;
        // }
        // if(x_dis > 5.7)  
        // {
        //     continue;
        // }
        
        // ICL 沙发侧面
        if(height < 2.6)  {
            // p.z -= 0.2;
            pCloudFiltered->push_back(p);
        } 
    }

    // // 地面
    // // 1. 添加地面点云
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    // double x_min = -4.0, x_max = 18.0;  // x 范围
    // double y_min = -4.0, y_max = 18.0;  // y 范围
    // double z_ground = -0.07 + 0.15;             // 地面高度
    // double resolution = 0.01;          // 地面点云分辨率（步长）

    // for (double x = x_min; x <= x_max; x += resolution) {
    //     for (double y = y_min; y <= y_max; y += resolution) {
    //         pcl::PointXYZRGB groundPoint;
    //         groundPoint.x = x;
    //         groundPoint.y = y;
    //         groundPoint.z = z_ground;

    //         // 设置为黑色的 RGB 值
    //         groundPoint.r = 100;
    //         groundPoint.g = 100;
    //         groundPoint.b = 100;
    //         // 灰色
    //         // groundPoint.r = 160;
    //         // groundPoint.g = 160;
    //         // groundPoint.b = 160;

    //         groundCloud->push_back(groundPoint);
    //     }
    // }

    // *pCloudFiltered += *groundCloud;

    return pCloudFiltered;
}

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "pcd_publisher");
    ros::NodeHandle nh;

    // 创建 ROS 发布器
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_topic", 1);

    // 创建 PCL 点云对象
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // 检查是否提供了 PCD 文件路径
    std::string pcd_file_path;
    int dateset_type = 1; // 1: ICL-NUIM, 2: Replica 
    if (argc < 2) {
        ROS_ERROR("Please provide the path to a PCD type.");
        return -1;
    }
    dateset_type = std::atoi(argv[1]);
    if(dateset_type == 1)
        pcd_file_path  = "/home/robotlab/dataset/ICL-NUIM/living_room_traj2n_frei_png/dataset.pcd";
    else if(dateset_type == 2)
        pcd_file_path  = "/home/robotlab/dataset/Replica-Dataset-results/hotel_0_640/map/dataset.pcd";
    else if(dateset_type == 3){
        pcd_file_path = "/home/robotlab/dataset/MySimDataset/gazebo_dataset_10/map/dataset.pcd";
    }
    else if(dateset_type == 4){
        pcd_file_path = "/home/robotlab/dataset/Replica-Dataset-results/hotel_0_640/map/dataset.pcd";
    }

    else
        ROS_ERROR("Please provide the correct dataset type: 1 for ICL-NUIM, 2 for Replica.");

    // 加载 PCD 文件
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_file_path, *cloud) == -1) {
        ROS_ERROR("Couldn't read the PCD file.");
        return -1;
    }


    {
        // （1）根据需要对点云进行变换
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
  
    }
    

    if(dateset_type==1){
        // Replica 数据集需要对点云进行滤波
        pcl::PointCloud<pcl::PointXYZRGB>* filtered_cloud = filterCloudAsHeight_1(cloud.get());
        cloud->clear();
        *cloud = *filtered_cloud;
        delete filtered_cloud;
    }
    else if(dateset_type==2){
        // Replica 数据集需要对点云进行滤波
        pcl::PointCloud<pcl::PointXYZRGB>* filtered_cloud = filterCloudAsHeight_2(cloud.get());
        cloud->clear();
        *cloud = *filtered_cloud;
        delete filtered_cloud;
    }
    else if(dateset_type==3){
        // Replica 数据集需要对点云进行滤波
        pcl::PointCloud<pcl::PointXYZRGB>* filtered_cloud = filterCloudAsHeight_3(cloud.get());
        cloud->clear();
        *cloud = *filtered_cloud;
        delete filtered_cloud;
    }
    else if(dateset_type==4){
        // Replica 数据集需要对点云进行滤波
        pcl::PointCloud<pcl::PointXYZRGB>* filtered_cloud = filterCloudAsHeight_4(cloud.get());
        cloud->clear();
        *cloud = *filtered_cloud;
        delete filtered_cloud;
    }

    {
        // // （3）根据需要保存修改后的点云到新的 PCD 文件
        // std::string output_file = "/home/robotlab/dataset/MySimDataset/gazebo_dataset_10/map/modified_cloud.pcd";  // 输出文件名
        // if (pcl::io::savePCDFileASCII(output_file, *cloud) == -1) {
        //     std::cerr << "Failed to save the modified point cloud." << std::endl;
        //     return -1;
        // }
        // std::cout << "Saved modified point cloud to " << output_file << std::endl;
    }
    
    // {
    //     // 2. 过滤点云
    //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    //     for (const auto& point : cloud->points) {
    //         if (point.z < 2.3) {  // 过滤掉过高的点
    //             filtered_cloud->points.push_back(point);
    //         }
    //     }
    //     filtered_cloud->width = filtered_cloud->points.size();
    //     filtered_cloud->height = 1;
    //     filtered_cloud->is_dense = true;

    //     // 3. 计算法向量
    //     pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    //     pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    //     pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    //     ne.setSearchMethod(tree);
    //     ne.setInputCloud(filtered_cloud);
    //     ne.setKSearch(20);  // 使用 20 个最近点计算法向量
    //     ne.compute(*normals);

    //     // 4. 合并点云和法向量
    //     pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
    //     pcl::concatenateFields(*filtered_cloud, *normals, *cloud_with_normals);

    //     // 5. 泊松表面重建
    //     pcl::Poisson<pcl::PointNormal> poisson;
    //     poisson.setDepth(8);  // 控制重建深度（值越大网格越精细）
    //     pcl::PolygonMesh mesh;
    //     poisson.setInputCloud(cloud_with_normals);
    //     poisson.reconstruct(mesh);
    // }

    // （4）发布rostopic
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