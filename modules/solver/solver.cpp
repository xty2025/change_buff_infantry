#include "solver.hpp"
#include <Log/log.hpp>
#include <opencv2/core/eigen.hpp>

auto solver::createSolver() -> std::shared_ptr<Solver> {
    return std::make_unique<solver::Solver>();
}

namespace solver {

	inline constexpr auto SmallArmorHalfWidth = 0.0675f;
	inline constexpr auto SmallArmorHalfHeight = 0.0275f;
	inline constexpr auto SmallArmorWidthRatio = 1.0f;
	inline constexpr auto SmallArmorHeightRatio = 1.0f;
	inline constexpr auto SAHW = SmallArmorHalfWidth * SmallArmorWidthRatio;
	inline constexpr auto SAHH = SmallArmorHalfHeight * SmallArmorHeightRatio;
	const std::vector SmallArmorPoints =
	{
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


PYD Solver::solveArmorPoses(ArmorXYV armor,int car_id,ImuData imuData_deg) {
	PYD pyd(imuData_deg);

	cv::Mat m_R;
	cv::Mat m_T;
	Eigen::Matrix3d e_R;
	Eigen::Vector3d e_T;
	std::vector<cv::Point2f> corners;

	if (armor.size() != 4) 
	return pyd;

	for (int i = 0; i < 4; i++) {
		corners.push_back(cv::Point2f(armor[i].x, armor[i].y));
	}

	if (car_id != 1 || car_id != 0) { // 英雄和基地除外
		solvePnP(SmallArmorPoints, corners, cameraIntrinsicMatrix, distorationCoefficients,
			rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
	}else{
		solvePnP(LargeArmorPoints, corners, cameraIntrinsicMatrix, distorationCoefficients,
			rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
	}

	cv::Rodrigues(rvec, m_R);
	cv::cv2eigen(m_R, e_R);
	cv::cv2eigen(tvec,e_T);

	Eigen::Matrix3d R_conversion;
	
	// 原：x向右，y向下，z向前
	// x向前、y向左，z向上
	R_conversion << 0, 0, 1,
					-1, 0, 0,
					0, -1, 0;

	
	Sophus::SE3<double> transRight(R_conversion, Eigen::Vector3d(0, 0, 0));

	Sophus::SO3<double> armor_rotate(e_R);
	
	// TODO: 使用 旋转向量的yaw 角度
	// double yaw = armor_rotate.matrix().eulerAngles(2,1,0)[0];//yaw
	// double pitch = armor_rotate.matrix().eulerAngles(2,1,0)[1];//pitch
	// double roll = armor_rotate.matrix().eulerAngles(2,1,0)[2];//roll

	armor_to_camera = transRight *Sophus::SE3<double>(e_R, e_T);
	auto armor_to_gimbal = camera_to_gimbal * armor_to_camera;
	XYZ pose;
	pose.x = armor_to_gimbal.translation()[0];
	pose.y = armor_to_gimbal.translation()[1];
	pose.z = armor_to_gimbal.translation()[2];
	return fuseIMU(XYZ2PYD(pose),imuData_deg);

}

CXYD Solver::solvesubArmorPoses(ArmorXYV armor,int car_id,ImuData imuData_deg) {
	cv::Mat m_R;
	cv::Mat m_T;
	Eigen::Matrix3d e_R;
	Eigen::Vector3d e_T;
	std::vector<cv::Point2f> corners;

	for (int i = 0; i < 4; i++) {
		corners.push_back(cv::Point2f(armor[i].x, armor[i].y));
	}

	if (car_id != 1 || car_id != 0) { // 英雄和基地除外
		solvePnP(SmallArmorPoints, corners, cameraIntrinsicMatrix, distorationCoefficients,
			rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
	}else{
		solvePnP(LargeArmorPoints, corners, cameraIntrinsicMatrix, distorationCoefficients,
			rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
	}

	cv::Rodrigues(rvec, m_R);
	cv::cv2eigen(m_R, e_R);
	cv::cv2eigen(tvec,e_T);

	Eigen::Matrix3d R_conversion;
	
	// 原：x向右，y向下，z向前
	// x向前、y向左，z向上
	R_conversion << 0, 0, 1,
					-1, 0, 0,
					0, -1, 0;

	
	Sophus::SE3<double> transRight(R_conversion, Eigen::Vector3d(0, 0, 0));

	Sophus::SO3<double> armor_rotate(e_R);
	
	// TODO: 使用 旋转向量的yaw 角度
	// double yaw = armor_rotate.matrix().eulerAngles(2,1,0)[0];//yaw
	// double pitch = armor_rotate.matrix().eulerAngles(2,1,0)[1];//pitch
	// double roll = armor_rotate.matrix().eulerAngles(2,1,0)[2];//roll

	armor_to_camera = transRight *Sophus::SE3<double>(e_R, e_T);
	auto armor_to_gimbal = camera_to_gimbal * armor_to_camera;
	XYZ pose;
	pose.x = armor_to_gimbal.translation()[0];
	pose.y = armor_to_gimbal.translation()[1];
	pose.z = armor_to_gimbal.translation()[2];
	return PYD2CXYD(XYZ2PYD(pose));

}
void Solver::setCameraIntrinsicMatrix(const cv::Mat& cameraIntrinsicMatrix) {
    this->cameraIntrinsicMatrix = cameraIntrinsicMatrix;
	fx = cameraIntrinsicMatrix.at<double>(0, 0);
	fy = cameraIntrinsicMatrix.at<double>(1, 1);
	cx = cameraIntrinsicMatrix.at<double>(0, 2);
	cy = cameraIntrinsicMatrix.at<double>(1, 2);
}

void Solver::setCameraOffset(const Eigen::Vector3d& cameraOffset) {
    this->cameraOffset = cameraOffset;
}

void Solver::setDistorationCoefficients(const cv::Mat& distorationCoefficients) {
    this->distorationCoefficients = distorationCoefficients;
}
void Solver::setCameraExternalMatrix(const Eigen::Vector3d cameraTrans, const double cameraPitchAngle) {
    Eigen::Vector3d euler_angle(cameraPitchAngle * degree2rad, 0, 0);
    Eigen::Matrix3d rotation_matrix;

    rotation_matrix = Eigen::AngleAxisd(euler_angle[2], Eigen::Vector3d::UnitZ()) * // roll
                      Eigen::AngleAxisd(euler_angle[0], Eigen::Vector3d::UnitX()) * // pitch
                      Eigen::AngleAxisd(euler_angle[1], Eigen::Vector3d::UnitY());  // yaw
	
    this->camera_to_gimbal = Sophus::SE3(rotation_matrix, cameraTrans);

}

ImuData::ImuData(const ParsedSerialData& x) : pitch(x.pitch_now), yaw(x.yaw_now), roll(x.roll_now) {};

} // namespace solver
