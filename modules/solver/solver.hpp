#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
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
    Eigen::Matrix3d cameraIntrinsicMatrix;
    Eigen::Vector3d cameraOffset;
    Eigen::Vector5d distorationCoefficients;// k1,k2,p1,p2,k3
	// static double X = 637.5, Y = 496.5;  // center of the image
	// static double X1 = -3.78e-6, Y1 = 7.394e-4, X2 = 7.266e-4, Y2 = 3e-6;
public:
	static double X ,Y;  // center of the image
	static double X1 ,Y1 ,X2 ,Y2;
	explicit Solver(double _X, double _Y, double _X1, double _Y1, double _X2, double _Y2)
	{
		X = _X;
		Y = _Y;
		X1 = _X1;
		Y1 = _Y1;
		X2 = _X2;
		Y2 = _Y2;
	};
    
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
		PYD out;
		out.distance = in.distance;
		out.pitch = - ((in.cx - X) * X1 + (in.cy - Y) * Y1);
		out.yaw = - ((in.cx - X) * X2 + (in.cy - Y) * Y2);
		return out;
	}
	inline CXYD PYD2CXYD(const PYD& in) const override
	{
		CXYD out;
		double det = X1 * Y2 - Y1 * X2;
		out.cx = X - (Y2 * in.pitch - Y1 * in.yaw) / det;
		out.cy = Y - (-X2 * in.pitch + X1 * in.yaw) / det;
		out.distance = in.distance;
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
    
	void setCameraIntrinsicMatrix(const Eigen::Matrix3d& cameraIntrinsicMatrix);
    void setCameraOffset(const Eigen::Vector3d& cameraOffset);
    void setDistorationCoefficients(const Eigen::Vector5d& distorationCoefficients);
};
std::shared_ptr<Solver> createSolver(double X, double Y, double X1, double Y1, double X2, double Y2);

} // namespace solver
