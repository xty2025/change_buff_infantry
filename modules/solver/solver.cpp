#include "solver.hpp"
#include <Log/log.hpp>
#include <opencv2/core/eigen.hpp>

auto solver::createSolver(double X, double Y, double X1, double Y1, double X2, double Y2) -> std::shared_ptr<Solver> {
    return std::make_unique<solver::Solver>(X, Y, X1, Y1, X2, Y2);
}

namespace solver {
    double Solver::X;
    double Solver::Y;
    double Solver::X1;
    double Solver::Y1;
    double Solver::X2;
    double Solver::Y2;

	// 距离修正系数
	const double distCoef = 5.0/3.7;
	inline constexpr auto SmallArmorHalfWidth = 0.0675f;
	inline constexpr auto SmallArmorHalfHeight = 0.0275f;
	inline constexpr auto SmallArmorWidthRatio = 1.0f;
	inline constexpr auto SmallArmorHeightRatio = 1.0f;
	//SAH : Small Armor Half
	inline constexpr auto SAHW = SmallArmorHalfWidth * SmallArmorWidthRatio;
	inline constexpr auto SAHH = SmallArmorHalfHeight * SmallArmorHeightRatio;
	const std::vector SmallArmorPoints =	// 装甲板放在地上
	{
		// cv::Point3f(-SAHW, SAHH, 0.0f),
		// cv::Point3f(-SAHW, -SAHH, 0.0f),
		// cv::Point3f(SAHW, -SAHH, 0.0f),
		// cv::Point3f(SAHW, SAHH, 0.0f)
		cv::Point3f(-SAHW, SAHH, 0.0f),
		cv::Point3f(SAHW, SAHH, 0.0f),
		cv::Point3f(SAHW, -SAHH, 0.0f),
		cv::Point3f(-SAHW, -SAHH, 0.0f)
	};

    inline constexpr auto LargeArmorHalfWidth = 0.116f;
	inline constexpr auto LargeArmorHalfHeight = 0.0275f;
	inline constexpr auto LargeArmorWidthRatio = 0.87f;
	inline constexpr auto LargeArmorHeightRatio = 1.0f;
	inline constexpr auto LAHW = LargeArmorHalfWidth * LargeArmorWidthRatio;
	inline constexpr auto LAHH = LargeArmorHalfHeight * LargeArmorHeightRatio;
	inline constexpr auto LargeArmorXRatio = 0.87f;
	const std::vector LargeArmorPoints =
	{
		cv::Point3f(-LAHW, LAHH, 0.0f),
		cv::Point3f(LAHW, LAHH, 0.0f),
		cv::Point3f(LAHW, -LAHH, 0.0f),
		cv::Point3f(-LAHW, -LAHH, 0.0f)
	};

// The following code is deprecated
// Due to no improvement compared with solvePnP method
// Now i'm not sure if this method is useful
// because the fatal bug of the order of armor points
// was fixed now.
//拟合参数: 2.419558578250169 130.9631449517108 0.4683121472580308 -0.4554121653810246
double A = 2.419558578250169;
double B = 130.9631449517108;
double C = 0.4683121472580308;
double D = -0.4554121653810246;
[[deprecated("Use solvePnP method instead")]]
double calcDistance(const std::vector<cv::Point2f>& points, double yaw, double pitch) 
{
    // 计算边长
    auto dist = [](const cv::Point2f& p1, const cv::Point2f& p2) {
        return std::hypot(p1.x - p2.x, p1.y - p2.y);
    };
    
    // 计算平均边长
    double s1 = (dist(points[0], points[1]) + dist(points[2], points[3])) / 2.0;
    double s2 = (dist(points[1], points[2]) + dist(points[3], points[0])) / 2.0;
    
    // 计算对角线余弦值
    double costheta_1 = (s1 * s1 + s2 * s2 - std::pow(dist(points[0], points[2]), 2)) / (2 * s1 * s2);
    double costheta_2 = (s1 * s1 + s2 * s2 - std::pow(dist(points[1], points[3]), 2)) / (2 * s1 * s2);
    double costheta = (costheta_1 + costheta_2) / 2.0;
    
    // 计算delta
    double delta = 4 * std::pow(A * s1 * s2 * costheta, 2) + 
                  std::pow(s2 * s2 - std::pow(A * s1, 2), 2);
    
    // 计算最终距离
    double distance = B / std::sqrt(std::pow(A * s1, 2) + s2 * s2 + std::sqrt(delta));
    double a1 = std::sqrt(distance * distance + C * C - 2 * distance * C * std::cos(yaw));
    return std::sqrt(a1 * a1 + D * D - 2 * a1 * D * std::cos(pitch));
}

// cv::Point2f exactCenter(const std::vector<cv::Point2f>& points) {
//     cv::Point2f center(0, 0);
//     if(points.size() != 4) {
//         return center; // 返回默认值
//     }
//     // 计算对角线交点
//     // 假设四个顶点按顺时针或逆时针顺序排列
//     // 对角线为：armor[0]连接armor[2]，armor[1]连接armor[3]
//     double x1 = points[0].x, y1 = points[0].y;  // 第一个点
//     double x2 = points[2].x, y2 = points[2].y;  // 对角点
//     double x3 = points[1].x, y3 = points[1].y;  // 第二个点
//     double x4 = points[3].x, y4 = points[3].y;  // 对角点

//     // 求解两线段交点
//     double denominator = (y4-y3)*(x2-x1) - (x4-x3)*(y2-y1);
//     if (std::abs(denominator) < 1e-10) {
//         // 两条线几乎平行，使用四点的平均值
//         center.x = (x1 + x2 + x3 + x4) / 4.0;
//         center.y = (y1 + y2 + y3 + y4) / 4.0;
//     } else {
//         double ua = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / denominator;
//         center.x = x1 + ua * (x2-x1);
//         center.y = y1 + ua * (y2-y1);
//     }
//     return center;
// }、
cv::Point2f exactCenter(const std::vector<cv::Point2f>& points) {
    cv::Point2f center(0, 0);
    if(points.size() != 4) {
        return center; // 返回默认值
    }
    // 计算四个点的平均值
    for (const auto& point : points) {
        center.x += point.x;
        center.y += point.y;
    }
    center.x /= 4.0;
    center.y /= 4.0;
    return center;
}

inline double normalizeAngle(double angle) {
    return std::remainder(angle, 2 * M_PI);
}

//return {pyd, armor_yaw}
std::pair<PYD,double> Solver::camera2world(const ArmorXYV& trackResult, const ImuData& imuData_deg, bool isLarge) {
    PYD pyd(imuData_deg);
    cv::Mat cameraMatrix, distCoeffs;
    cv::eigen2cv(cameraIntrinsicMatrix, cameraMatrix);
    cv::eigen2cv(distorationCoefficients, distCoeffs);
    auto imu_roll = imuData_deg.roll * M_PI / 180, imu_pitch = imuData_deg.pitch * M_PI / 180, imu_yaw = imuData_deg.yaw * M_PI / 180;
    Eigen::Matrix3d R = (Eigen::AngleAxisd(imu_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() 
                    * Eigen::AngleAxisd(-imu_pitch, Eigen::Vector3d::UnitY()).toRotationMatrix()
                    * Eigen::AngleAxisd(imu_roll, Eigen::Vector3d::UnitX()).toRotationMatrix());
    
    if (trackResult.size() != 4) 
        return std::make_pair(pyd, 0.0);

    std::vector<cv::Point3f> objectPoints = isLarge ? LargeArmorPoints : SmallArmorPoints;

    std::vector<cv::Point2f> imagePoints;
    for (const auto& xyv : trackResult) 
    {//temporarily dont consider visibility
        imagePoints.emplace_back(xyv.x, xyv.y);
    }
    // double center_x = (imagePoints[0].x + imagePoints[1].x + imagePoints[2].x + imagePoints[3].x) / 4;
    // double center_y = (imagePoints[0].y + imagePoints[1].y + imagePoints[2].y + imagePoints[3].y) / 4;
    cv::Point2f center = exactCenter(imagePoints);
    pyd = fuseIMU(CXYD2PYD(CXYD(center.x, center.y, 0)), imuData_deg);
    // // 使用 solvePnP 计算距离
    // cv::Mat rvec, tvec;
    // cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE);
    // // // 用rvec计算装甲板相对于相机的yaw角
    // // cv::Mat rotationMatrix;
    // // cv::Rodrigues(rvec, rotationMatrix); // 将旋转向量转换为旋转矩阵
    // // //cv::Mat R_inv = rotationMatrix.t();
    // // //cv::Mat worldXDir = R_inv * cv::Mat(cv::Vec3f(1, 0, 0));
    // // // from rotationMatrix to Euler angles
    
    // // //double armor_yaw = atan2(rotationMatrix.at<double>(1,0), rotationMatrix.at<double>(0,0));
    // // double armor_yaw = atan2(rotationMatrix.at<double>(2, 1), rotationMatrix.at<double>(2, 0));
    // // //double armor_yaw = atan2(worldXDir.at<float>(1), worldXDir.at<float>(0));
    // // //double armor_yaw = std::atan2(rotationMatrix.at<double>(1, 0), rotationMatrix.at<double>(0, 0));
    // // //double armor_pitch = std::atan2(rotationMatrix.at<double>(2, 0), rotationMatrix.at<double>(2, 1));
    // // //INFO("armor_yaw: {}, armor_pitch: {}", armor_yaw, armor_pitch);
    // // INFO("armor_yaw: {}", armor_yaw);
    // cv::Mat rotationMatrix;
    // cv::Rodrigues(rvec, rotationMatrix); // 将旋转向量转换为旋转矩阵

    // // 计算装甲板yaw角
    // double armor_yaw = atan2(rotationMatrix.at<double>(2, 1), rotationMatrix.at<double>(2, 0));

    // // 角度规范化和连续性处理
    // static double prev_armor_yaw = armor_yaw; // 静态变量保存上一帧角度
    // double diff = armor_yaw - prev_armor_yaw;

    // // // 处理角度跳变
    // // if (diff > M_PI) {
    // //     armor_yaw -= 2 * M_PI;
    // // } else if (diff < -M_PI) {
    // //     armor_yaw += 2 * M_PI;
    // // }
    // armor_yaw = normalizeAngle(armor_yaw);

    // // 对于IPPE算法可能的多解问题，可以检查另一个可能的解
    // if (std::abs(diff) > M_PI/2) {
    //     double alternative_yaw = armor_yaw + M_PI; // 另一个可能的解
    //     if (alternative_yaw > M_PI) alternative_yaw -= 2 * M_PI;
    //     if (alternative_yaw < -M_PI) alternative_yaw += 2 * M_PI;
        
    //     // 选择与上一帧更接近的解
    //     if (std::abs(alternative_yaw - prev_armor_yaw) < std::abs(armor_yaw - prev_armor_yaw)) {
    //         armor_yaw = alternative_yaw;
    //     }
    // }

    // // 更新前一帧角度
    // prev_armor_yaw = armor_yaw;

    // INFO("armor_yaw: {}", armor_yaw);

    // 使用 solvePnPGeneric 计算多个可能的解
std::vector<cv::Mat> rvecs, tvecs;
int solutions = cv::solvePnPGeneric(objectPoints, imagePoints, cameraMatrix, distCoeffs, 
                                   rvecs, tvecs, false, cv::SOLVEPNP_IPPE);

// 计算所有解的yaw角并选择最合适的解
static double prev_armor_yaw = 0.0; // 静态变量保存上一帧角度
double armor_yaw = 0.0;
double min_diff = std::numeric_limits<double>::max();
cv::Mat tvec;

// 处理第一帧的情况
if (solutions > 0 && std::abs(prev_armor_yaw) < 1e-6) {
    cv::Mat rotMat;
    cv::Rodrigues(rvecs[0], rotMat);
    prev_armor_yaw = atan2(rotMat.at<double>(2, 0), rotMat.at<double>(0, 0));
    prev_armor_yaw = normalizeAngle(prev_armor_yaw);
}

// 遍历所有解，选择与上一帧最接近的
for (int i = 0; i < solutions; i++) {
    cv::Mat rotMat;
    cv::Rodrigues(rvecs[i], rotMat);
    double current_yaw = atan2(rotMat.at<double>(2, 0), rotMat.at<double>(0, 0));
    current_yaw = normalizeAngle(current_yaw);
    
    // 计算与上一帧的角度差
    double diff = std::abs(normalizeAngle(current_yaw - prev_armor_yaw));
    double dist = tvecs[i].at<double>(0) * tvecs[i].at<double>(0) +
                  tvecs[i].at<double>(1) * tvecs[i].at<double>(1) +
                  tvecs[i].at<double>(2) * tvecs[i].at<double>(2);
    dist = std::sqrt(dist);
    
    // 保存差异最小的解
    if (diff < min_diff) {
        min_diff = diff;
        armor_yaw = current_yaw;
        tvec = tvecs[i];
    }
    
    INFO("Solution {}: armor_yaw = {}, diff = {}, dist = {}", i, current_yaw, diff, dist);
}

// 更新前一帧角度
prev_armor_yaw = armor_yaw;

INFO("Selected armor_yaw: {}", armor_yaw);




    Eigen::Vector3d P_c(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
    P_c += cameraOffset;
    pyd.distance = P_c.norm();  // 距离
    pyd.distance *= distCoef;  // 修正距离
    //pyd.distance = 1;

    //pyd.distance = calcDistance(imagePoints, pyd.yaw, pyd.pitch);
    INFO("pitch:{},yaw:{},x:{},y:{},z:{},dist:{}",pyd.pitch,pyd.yaw
        ,pyd.distance * cos(pyd.pitch) * cos(pyd.yaw),
        pyd.distance * cos(pyd.pitch) * sin(pyd.yaw),
        pyd.distance * sin(pyd.pitch),pyd.distance);
    return std::make_pair(pyd, armor_yaw + imu_yaw);
}

std::pair<PYD,double> Solver::camera2worldWithWholeCar(const ArmorXYV& trackResult, const ImuData& imuData_deg, const cv::Rect& bounding_rect, bool isLarge) 
{
    PYD pyd(imuData_deg);
    cv::Mat cameraMatrix, distCoeffs;
    cv::eigen2cv(cameraIntrinsicMatrix, cameraMatrix);
    cv::eigen2cv(distorationCoefficients, distCoeffs);
    auto imu_roll = imuData_deg.roll * M_PI / 180, imu_pitch = imuData_deg.pitch * M_PI / 180, imu_yaw = imuData_deg.yaw * M_PI / 180;
    Eigen::Matrix3d R = (Eigen::AngleAxisd(imu_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() 
                    * Eigen::AngleAxisd(-imu_pitch, Eigen::Vector3d::UnitY()).toRotationMatrix()
                    * Eigen::AngleAxisd(imu_roll, Eigen::Vector3d::UnitX()).toRotationMatrix());
    
    if (trackResult.size() != 4) 
        return std::make_pair(pyd, 0.0);

    std::vector<cv::Point3f> objectPoints = isLarge ? LargeArmorPoints : SmallArmorPoints;

    std::vector<cv::Point2f> imagePoints;
    for (const auto& xyv : trackResult) 
    {//temporarily dont consider visibility
        imagePoints.emplace_back(xyv.x, xyv.y);
    }
    // double center_x = (imagePoints[0].x + imagePoints[1].x + imagePoints[2].x + imagePoints[3].x) / 4;
    // double center_y = (imagePoints[0].y + imagePoints[1].y + imagePoints[2].y + imagePoints[3].y) / 4;
    cv::Point2f center = exactCenter(imagePoints);
    pyd = fuseIMU(CXYD2PYD(CXYD(center.x, center.y, 0)), imuData_deg);
    // 使用 solvePnPGeneric 计算多个可能的解
std::vector<cv::Mat> rvecs, tvecs;
int solutions = cv::solvePnPGeneric(objectPoints, imagePoints, cameraMatrix, distCoeffs, 
                                   rvecs, tvecs, false, cv::SOLVEPNP_IPPE);

// 计算所有解的yaw角并选择最合适的解
double estimate_armor_yaw = 0.0; // 估计的角度
double armor_yaw = 0.0;
double min_diff = std::numeric_limits<double>::max();
cv::Mat tvec;

//according to the bounding_rect, calculate the estimate_armor_yaw
double rect_center_x = (bounding_rect.x + bounding_rect.width / 2);
double armor_center_x = (imagePoints[0].x + imagePoints[1].x + imagePoints[2].x + imagePoints[3].x) / 4;
double half_width = bounding_rect.width / 2;
estimate_armor_yaw = std::asin((armor_center_x - rect_center_x) / half_width);

// 遍历所有解，选择与上一帧最接近的
for (int i = 0; i < solutions; i++) {
    cv::Mat rotMat;
    cv::Rodrigues(rvecs[i], rotMat);
    double current_yaw = atan2(rotMat.at<double>(2, 0), rotMat.at<double>(0, 0));
    current_yaw = normalizeAngle(current_yaw);
    
    // 计算与上一帧的角度差
    double diff = std::abs(normalizeAngle(current_yaw - estimate_armor_yaw));
    double dist = tvecs[i].at<double>(0) * tvecs[i].at<double>(0) +
                  tvecs[i].at<double>(1) * tvecs[i].at<double>(1) +
                  tvecs[i].at<double>(2) * tvecs[i].at<double>(2);
    dist = std::sqrt(dist);
    
    // 保存差异最小的解
    if (diff < min_diff) {
        min_diff = diff;
        armor_yaw = current_yaw;
        tvec = tvecs[i];
    }
    
    INFO("Solution {}: armor_yaw = {}, diff = {}, dist = {}", i, current_yaw, diff, dist);
}

    INFO("Selected armor_yaw: {}", armor_yaw);




    Eigen::Vector3d P_c(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
    P_c += cameraOffset;
    pyd.distance = P_c.norm();  // 距离
    pyd.distance *= distCoef;  // 修正距离
    // pyd.distance = 1;

    //pyd.distance = calcDistance(imagePoints, pyd.yaw, pyd.pitch);
    INFO("pitch:{},yaw:{},x:{},y:{},z:{},dist:{}",pyd.pitch,pyd.yaw
        ,pyd.distance * cos(pyd.pitch) * cos(pyd.yaw),
        pyd.distance * cos(pyd.pitch) * sin(pyd.yaw),
        pyd.distance * sin(pyd.pitch),pyd.distance);
    return std::make_pair(pyd, armor_yaw + imu_yaw);
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

ImuData::ImuData(const ParsedSerialData& x) : pitch(x.pitch_now), yaw(x.yaw_now), roll(x.roll_now) {};

} // namespace solver
