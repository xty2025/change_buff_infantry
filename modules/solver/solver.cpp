#include "solver.hpp"
#include <opencv2/core/eigen.hpp>

auto modules::createSolver() -> std::unique_ptr<modules::Solver> {
    return std::make_unique<solver::Solver>();
}

namespace solver {

XYV Solver::world2camera(const PYD& point, const ImuData& imuData) {
    XYV xyv(0,0);
    
    // 构建旋转矩阵 - 按照 roll(Z) -> pitch(Y) -> yaw(X) 的顺序
    Eigen::Matrix3d R = (Eigen::AngleAxisd(imuData.roll, Eigen::Vector3d::UnitZ()).toRotationMatrix() 
                     * Eigen::AngleAxisd(imuData.pitch, Eigen::Vector3d::UnitY()).toRotationMatrix()
                     * Eigen::AngleAxisd(imuData.yaw, Eigen::Vector3d::UnitX()).toRotationMatrix());

    // 世界坐标点
    Eigen::Vector3d P_w(point.distance * cos(point.pitch) * cos(point.yaw),
                        point.distance * cos(point.pitch) * sin(point.yaw),
                        point.distance * sin(point.pitch));
    
    // 转换到相机坐标系
    Eigen::Vector3d P_c = R * (P_w - cameraOffset);
    
    // 应用相机内参矩阵
    Eigen::Vector3d P_i = cameraIntrinsicMatrix * P_c;
    
    // 添加除零保护
    if (std::abs(P_i.z()) > 1e-6) {  // 避免除零
        xyv.x = P_i.x() / P_i.z();
        xyv.y = P_i.y() / P_i.z();
        xyv.visible = P_i.z() > 0;
    } else {
        xyv.visible = false;
    }
    
    
    return xyv;
}

PYDs Solver::camera2world(const ArmorXYVs& trackResults, const ImuData& imuData, bool isLarge) {
    PYDs pyds;
    cv::Mat cameraMatrix, distCoeffs;
    cv::eigen2cv(cameraIntrinsicMatrix, cameraMatrix);
    cv::eigen2cv(distorationCoefficients, distCoeffs);
    Eigen::Matrix3d R = (Eigen::AngleAxisd(imuData.roll, Eigen::Vector3d::UnitZ()).toRotationMatrix() 
                    * Eigen::AngleAxisd(imuData.pitch, Eigen::Vector3d::UnitY()).toRotationMatrix()
                    * Eigen::AngleAxisd(imuData.yaw, Eigen::Vector3d::UnitX()).toRotationMatrix());
    
    for (const auto& trackResult : trackResults) {
        if (trackResult.size() != 4) {
            continue; // Skip invalid track results
        }

        std::vector<cv::Point3f> objectPoints;
        if (isLarge) {
            objectPoints = LargeArmorPoints;
        } else {
            objectPoints = SmallArmorPoints;
        }

        std::vector<cv::Point2f> imagePoints;
        for (const auto& xyv : trackResult) 
        {//temporarily dont consider visibility
            imagePoints.emplace_back(xyv.x, xyv.y);
        }
        double center_x = (imagePoints[0].x + imagePoints[1].x + imagePoints[2].x + imagePoints[3].x) / 4;
        double center_y = (imagePoints[0].y + imagePoints[1].y + imagePoints[2].y + imagePoints[3].y) / 4;
        // 反投影获取相机坐标系下的射线方向
        Eigen::Vector3d ray_c(center_x, center_y, 1.0);  // 归一化平面坐标
        ray_c = cameraIntrinsicMatrix.inverse() * ray_c;  // 转换到相机坐标系
        ray_c.normalize();  // 单位化

        // 计算pitch和yaw
        PYD pyd = PYD::XYZ2PYD(ray_c.x(), ray_c.y(), ray_c.z());
        //pyd.pitch = std::asin(ray_c.z());  // 俯仰角
        //pyd.yaw = std::atan2(ray_c.y(), ray_c.x());  // 偏航角
        
        // 使用 solvePnP 计算距离
        cv::Mat rvec, tvec;
        cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE);
        Eigen::Vector3d P_c(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
        pyd.distance = P_c.norm();  // 距离

        // 考虑相机偏移
        P_c += cameraOffset;

        // 考虑 IMU 数据
        Eigen::Vector3d P_w = R.inverse() * P_c;

        // 转换为 pyd
        pyd.pitch = std::asin(P_w.z() / pyd.distance);
        pyd.yaw = std::atan2(P_w.y(), P_w.x());

        pyds.push_back(pyd);
    }
    return pyds;
}

void Solver::setCameraIntrinsicMatrix(const Eigen::Matrix3d& cameraIntrinsicMatrix) {
    this->cameraIntrinsicMatrix = cameraIntrinsicMatrix;
}

void Solver::setCameraOffset(const Eigen::Vector3d& cameraOffset) {
    this->cameraOffset = cameraOffset;
}

void Solver::setDistorationCoefficients(const Eigen::Vector5d& distorationCoefficients) {
    this->distorationCoefficients = distorationCoefficients;
}

} // namespace solver
