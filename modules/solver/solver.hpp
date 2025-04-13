#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "type.hpp"
#include "driver/type.hpp"
#include "Location/location.hpp"
#include "tracker/type.hpp"


namespace solver {
	using driver::ParsedSerialData;
	using tracker::ArmorXYV;

class Solver : public BaseSolver
{
private:
	cv::Mat cameraIntrinsicMatrix;
    cv::Mat distorationCoefficients;// k1,k2,p1,p2,k3
	double fx,fy,cx,cy;

	Eigen::Vector3d cameraOffset;

	Sophus::SE3<double> armor_to_camera;  
	Sophus::SE3<double> camera_to_gimbal;
	Sophus::SE3<double> gimbal_to_world;
	
	cv::Mat rvec; // 旋转向量 OpenCV
	cv::Mat tvec; // 平移向量 OpenCV

	double degree2rad = M_PI / 180;


public:
	explicit Solver() = default;

	PYD solveArmorPoses(ArmorXYV armor,int car_id,ImuData imuData_deg);
	CXYD solvesubArmorPoses(ArmorXYV armor,int car_id,ImuData imuData_deg);
	inline PYD XYZ2PYD(const XYZ& in) const override
	{
		double distance = sqrt(in.x*in.x + in.y*in.y + in.z*in.z);
		double pitch = asin(in.z/distance);
		double yaw = atan2(in.y, in.x);
		return PYD(pitch, yaw, distance);
	}
	inline XYZ PYD2XYZ(const PYD& in) const override
	{
		XYZ out;
		out.x = in.distance * cos(in.pitch) * cos(in.yaw);
		out.y = in.distance * cos(in.pitch) * sin(in.yaw);
		out.z = in.distance * sin(in.pitch);
		return out;
	}


	inline PYD CXYD2PYD(const CXYD& in) const override
	{
		double kx = (in.cx-cx)/fx;
		double ky = (in.cy-cy)/fy;

		// x向前，y向左，z向上
		double X2C = in.distance/sqrt(1 + kx*kx + ky*ky); // 沿着光心方向的深度
		double Y2C = -kx * X2C;
		double Z2C = -ky * X2C; // 相对于相机的三维坐标（真实）

		Eigen::Vector3d objectTrans(X2C, Y2C, Z2C);

		Eigen::Matrix3d rotation_matrix;
		rotation_matrix = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
						  Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
						  Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()); 
								
		Sophus::SE3<double> object_to_camera = Sophus::SE3(rotation_matrix, objectTrans);
		Sophus::SE3<double> object_to_gimbal = camera_to_gimbal  * object_to_camera;

		// 提取其中的平移向量
		Eigen::Vector3d object_to_gimbal_trans = object_to_gimbal.translation();
		XYZ sub_out = XYZ(object_to_gimbal_trans[0], object_to_gimbal_trans[1], object_to_gimbal_trans[2]);

		PYD out = XYZ2PYD(sub_out);

		return out;
	}
	inline CXYD PYD2CXYD(const PYD& in) const override
	{
		XYZ sub_in = PYD2XYZ(in);
		Eigen::Vector3d object_to_gimbal_trans(sub_in.x, sub_in.y, sub_in.z);
		Eigen::Matrix3d rotation_matrix;
		rotation_matrix = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
						  Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
						  Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()); 
		Sophus::SE3<double> object_to_gimbal =  Sophus::SE3(rotation_matrix, object_to_gimbal_trans);

		// 转换到相机坐标系
		Sophus::SE3<double> object_to_camera = camera_to_gimbal.inverse() * object_to_gimbal;

		double X2C = object_to_camera.translation()[0];
		double Y2C = object_to_camera.translation()[1];
		double Z2C = object_to_camera.translation()[2];

		//相机到像素
		double distance = sqrt(X2C*X2C + Y2C*Y2C + Z2C*Z2C);
		double kx = -Y2C/X2C;
		double ky = -Z2C/X2C;

		double X2P = kx * fx + cx;
		double Y2P = ky * fy + cy;

		CXYD out = CXYD(X2P, Y2P, distance);
		return out;
	}


	inline PYD fuseIMU(const PYD& in, const PYD& imuData) const override
	{
		PYD out;
		out.pitch = in.pitch + imuData.pitch;
		out.yaw = in.yaw + imuData.yaw;
		out.distance = in.distance;
		return out;
	}
	inline PYD separateIMU(const PYD& in, const PYD& imuData) const override
	{
		PYD out;
		out.pitch = in.pitch - imuData.pitch;
		out.yaw = in.yaw - imuData.yaw;
		out.distance = in.distance;
		return out;
	}
	
	PYD camera2world(const ArmorXYV& trackResult, const ImuData& imuData, bool isLarge);

	void setCameraIntrinsicMatrix(const cv::Mat& cameraIntrinsicMatrix);
    void setCameraOffset(const Eigen::Vector3d& cameraOffset);
    void setDistorationCoefficients(const cv::Mat& distorationCoefficients);
	void setCameraExternalMatrix(const Eigen::Vector3d cameraTrans, const double cameraPitchAngle);
};
std::shared_ptr<Solver> createSolver();

} // namespace solver
