#include "solver.hpp"
#include <Log/log.hpp>
#include <opencv2/core/eigen.hpp>

auto solver::createSolver(param::Param json_param) -> std::shared_ptr<Solver> {
    return std::make_unique<solver::Solver>(json_param);
}

namespace solver {


//	inline constexpr auto SmallArmorHalfWidth = 0.0675f;
//	inline constexpr auto SmallArmorHalfHeight = 0.0275f;
    inline constexpr auto SmallArmorHalfWidth = 0.067f;
    inline constexpr auto SmallArmorHalfHeight = 0.03f;
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

//  inline constexpr auto LargeArmorHalfWidth = 0.116f;
//	inline constexpr auto LargeArmorHalfHeight = 0.0275f;
    inline constexpr auto LargeArmorHalfWidth = 0.114f;
    inline constexpr auto LargeArmorHalfHeight = 0.03f;
	//inline constexpr auto LargeArmorWidthRatio = 0.87f;
	inline constexpr auto LargeArmorWidthRatio = 1.0f;
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

//计算四个点的中心坐标
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
//重载cameratoworld:- 使用OpenCV的solvePnPGeneric函数计算多个可能的解：
//solvePnP 是 OpenCV 中用于 求解外部参数（外参） 的算法。
// trackResult: 装甲板的四个角点坐标
// 它是 Perspective-n-Point（PnP）问题的解法之一，通常用于从已知的 3D 点和 2D 图像点计算相机的位置和方向（即外参：旋转矩阵和位移向量）。
//- 通过比较与前一帧的角度差选择最优解,在调用正常的camera2world函数转换成世界坐标
//return {pyd, armor_yaw}
std::pair<XYZ,double> Solver::camera2world(const ArmorXYV& trackResult, const ImuData& imuData_deg, bool isLarge) {
    cv::Mat cameraMatrix, distCoeffs;
    cv::eigen2cv(cameraIntrinsicMatrix, cameraMatrix);
    cv::eigen2cv(distorationCoefficients, distCoeffs);
    auto imu_roll = imuData_deg.roll * M_PI / 180, imu_pitch = imuData_deg.pitch * M_PI / 180, imu_yaw = imuData_deg.yaw * M_PI / 180;
    
    if (trackResult.size() != 4) 
        return std::make_pair(XYZ(), 0.0);

    std::vector<cv::Point3f> objectPoints = isLarge ? LargeArmorPoints : SmallArmorPoints;

    std::vector<cv::Point2f> imagePoints;
    for (const auto& xyv : trackResult) 
    {//temporarily dont consider visibility
        imagePoints.emplace_back(xyv.x, xyv.y);
    }
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
        cv::Rodrigues(rvecs[0], rotMat);//旋转向量存储成分旋转矩阵。
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
        //计算距离，欧几里得范数
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


    //把opencv的坐标系转换成我们的世界坐标系。
    XYZ camera(tvec.at<double>(2), -tvec.at<double>(0), -tvec.at<double>(1));
    XYZ result = camera2world(camera, imuData_deg);
    //pyd.distance = 1;

    //pyd.distance = calcDistance(imagePoints, pyd.yaw, pyd.pitch);
    INFO("x:{},y:{},z:{},dist:{}",result.x,result.y,result.z, sqrt(result.x*result.x + result.y*result.y + result.z*result.z));
    return std::make_pair(result, armor_yaw + imu_yaw);
    //返回世界坐标和装甲板总角度（装甲板角度+IMU yaw角度）
}

//重载cameratoworldWithWholeCar:
//功能：- 基于整个车辆的包围矩形来优化装甲板的世界坐标计算
//通过包围矩形估计装甲板的角度，然后选择最接近估计角度的解
std::pair<XYZ,double> Solver::camera2worldWithWholeCar(const ArmorXYV& trackResult, const ImuData& imuData_deg, const cv::Rect& bounding_rect, bool isLarge)
{
    cv::Mat cameraMatrix, distCoeffs;
    cv::eigen2cv(cameraIntrinsicMatrix, cameraMatrix);
    cv::eigen2cv(distorationCoefficients, distCoeffs);
    auto imu_roll = imuData_deg.roll * M_PI / 180, imu_pitch = imuData_deg.pitch * M_PI / 180, imu_yaw = imuData_deg.yaw * M_PI / 180;
    
    if (trackResult.size() != 4)
        return std::make_pair(XYZ(), 0.0);

    std::vector<cv::Point3f> objectPoints = isLarge ? LargeArmorPoints : SmallArmorPoints;

    std::vector<cv::Point2f> imagePoints;
    for (const auto& xyv : trackResult) 
    {//temporarily dont consider visibility
        imagePoints.emplace_back(xyv.x, xyv.y);
    }

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
    // 根据装甲板相对于矩形中心的位置估算角度
    //左右的极限角度。
    if((armor_center_x - rect_center_x) / half_width > 1)
        estimate_armor_yaw = M_PI / 2.0;
    else if ((armor_center_x - rect_center_x) / half_width < -1)
        estimate_armor_yaw = -M_PI / 2.0;
    else
        estimate_armor_yaw = std::asin((armor_center_x - rect_center_x) / half_width);
    // 加一个处理第一帧的情况？
    /*
    if (solutions > 0 && std::abs(prev_armor_yaw) < 1e-6) {
        cv::Mat rotMat;
        cv::Rodrigues(rvecs[0], rotMat);//旋转向量存储成分旋转矩阵。
        prev_armor_yaw = atan2(rotMat.at<double>(2, 0), rotMat.at<double>(0, 0));
        prev_armor_yaw = normalizeAngle(prev_armor_yaw);
    }*/

    // 遍历所有解，选择与上一帧最接近的（估计角度）。
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

    if((estimate_armor_yaw>0) && (armor_yaw<0)) {
        armor_yaw = -armor_yaw;
    }
    else if((estimate_armor_yaw<0) && (armor_yaw>0))
    {
        armor_yaw = -armor_yaw;
    }

    INFO("Selected armor_yaw: {}", armor_yaw);


    XYZ camera(tvec.at<double>(2), -tvec.at<double>(0), -tvec.at<double>(1));
    INFO("before transform: x:{},y:{},z:{}",camera.x,camera.y,camera.z);
    // 调用另一个重载的camera2world函数将相机坐标转换为世界坐标
    XYZ result = camera2world(camera, imuData_deg);
    //pyd.distance = 1;
    
    //pyd.distance = calcDistance(imagePoints, pyd.yaw, pyd.pitch);
    INFO("x:{},y:{},z:{},dist:{}",result.x,result.y,result.z, sqrt(result.x*result.x + result.y*result.y + result.z*result.z));
    return std::make_pair(result, armor_yaw + imu_yaw);
}
//setfunction.
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
