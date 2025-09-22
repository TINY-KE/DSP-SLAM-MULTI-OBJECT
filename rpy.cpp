#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>  // 包含旋转和几何工具

int main() {
    // 定义旋转矩阵 (3x3)
    Eigen::Matrix3d R_mat;
    // R_mat << -0.7766601284137539807, -0.03335248712866088067, -0.6290362919058515301,
    //           0.6299198718347684967, -0.04112197137203246755, -0.7755707179478520397,
    //           1.222929003948120707e-16, -0.9985973137720781656,  0.05294718998388627668;

    // R_mat << -0.1101925653095542218, -0.2638692763987721679, -0.9582434990769768124,
    //           0.9939102567890621964, -0.02925458538549653015, -0.1062382731571963473,
    //           1.180699993719285590e-16, -0.9641147100872962117,  0.2654860180749429310;

    R_mat <<  1, 0., -0.,
              0, 1., -0.,
              0, 0.,  1.;

    // 将角度转换为弧度
    double angle_deg = -75.0;
    double angle_rad = angle_deg * M_PI / 180.0;
    // Eigen::Matrix3d Ron = Eigen::AngleAxisf(M_PI/2, Eigen::Vector3d(1,0,0)).matrix()
    //     * Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3d(0,1,0)).matrix();
    Eigen::Matrix3f Ron = Eigen::AngleAxisf(angle_rad, Eigen::Vector3f(1,0,0)).matrix();

    // // 提取欧拉角 (RPY: Roll, Pitch, Yaw)
    // Eigen::Vector3d euler_angles = R_mat.eulerAngles(2, 1, 0);  // Yaw (Z), Pitch (Y), Roll (X)

    // // 转换为以度为单位表示
    // euler_angles = euler_angles * 180.0 / M_PI;

    // // 输出结果
    // std::cout << "RPY (degrees): \n";
    // std::cout << "Roll  (X): " << euler_angles[2] << " degrees\n";
    // std::cout << "Pitch (Y): " << euler_angles[1] << " degrees\n";
    // std::cout << "Yaw   (Z): " << euler_angles[0] << " degrees\n";

    // 计算新矩阵 new_mat = R_mat * R_z
    Eigen::Matrix3d new_mat = R_mat * Ron.cast<double>();

    // 将旋转矩阵 new_mat 转换为四元数
    Eigen::Quaterniond quat(new_mat);

    // 输出四元数的分量
    std::cout << "Quaternion (w, x, y, z): \n";
    std::cout << "x: " << quat.x() << "\n";
    std::cout << "y: " << quat.y() << "\n";
    std::cout << "z: " << quat.z() << "\n";
    std::cout << "w: " << quat.w() << "\n";



    return 0;
}