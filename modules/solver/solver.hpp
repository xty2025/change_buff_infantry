#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include "type.hpp"
#include "driver/type.hpp"
#include "Location/location.hpp"
#include "tracker/type.hpp"
#include "Param/param.hpp"


namespace solver {
	using driver::ParsedSerialData;
	using tracker::ArmorXYV;
    extern double solver_pitch_offset;
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
    //static double yaw_offset;
//	explicit Solver(double _X, double _Y, double _X1, double _Y1, double _X2, double _Y2)
//	{
//		X = _X;
//		Y = _Y;
//		X1 = _X1;
//		Y1 = _Y1;
//		X2 = _X2;
//		Y2 = _Y2;
//	};
    explicit Solver(param::Param& json_param)
    {
        X = json_param["X"].Double();
        Y = json_param["Y"].Double();
        X1 = json_param["X1"].Double();
        Y1 = json_param["Y1"].Double();
        X2 = json_param["X2"].Double();
        Y2 = json_param["Y2"].Double();
        solver_pitch_offset = json_param["pitch_offset"].Double();
        // Load camera intrinsic matrix from parameter file
        auto intrinsicArray = json_param["camera_intrinsic_matrix"].to<std::vector<std::vector<double>>>();
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                cameraIntrinsicMatrix(i, j) = intrinsicArray[i][j];
        auto distortionCoefficientsArray = json_param["camera_distortion_matrix"].to<std::vector<double>>();
        for (int i = 0; i < 5; ++i)
            distorationCoefficients(i) = distortionCoefficientsArray[i];
    }
    
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
		double pitch = std::remainder(in.pitch, 2 * M_PI);
		double yaw = std::remainder(in.yaw, 2 * M_PI);
		CXYD out;
		double det = X1 * Y2 - Y1 * X2;
		out.cx = X - (Y2 * pitch - Y1 * yaw) / det;
		out.cy = Y - (-X2 * pitch + X1 * yaw) / det;
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
	
	std::pair<PYD,double> camera2world(const ArmorXYV& trackResult, const ImuData& imuData, bool isLarge);
    std::pair<PYD,double> camera2worldWithWholeCar(const ArmorXYV& trackResult, const ImuData& imuData, const cv::Rect& bounding_rect, bool isLarge);

	void setCameraIntrinsicMatrix(const Eigen::Matrix3d& cameraIntrinsicMatrix);
    void setCameraOffset(const Eigen::Vector3d& cameraOffset);
    void setDistorationCoefficients(const Eigen::Vector5d& distorationCoefficients);
};
std::shared_ptr<Solver> createSolver(param::Param json_param);

} // namespace solver
