#pragma once
#include "interfaceType.hpp"
#include "modules.hpp"
#include <Eigen/Core>
#include <opencv2/opencv.hpp>


namespace solver {

	inline constexpr auto SmallArmorHalfWidth = 0.0675f;
	inline constexpr auto SmallArmorHalfHeight = 0.0275f;
	inline constexpr auto SmallArmorWidthRatio = 1.0f;
	inline constexpr auto SmallArmorHeightRatio = 1.0f;
	//SAH : Small Armor Half
	inline constexpr auto SAHW = SmallArmorHalfWidth * SmallArmorWidthRatio;
	inline constexpr auto SAHH = SmallArmorHalfHeight * SmallArmorHeightRatio;
	const std::vector SmallArmorPoints =	// 装甲板放在地上
	{
		cv::Point3f(-SAHW, SAHH, 0.0f),
		cv::Point3f(-SAHW, -SAHH, 0.0f),
		cv::Point3f(SAHW, -SAHH, 0.0f),
		cv::Point3f(SAHW, SAHH, 0.0f)
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
		cv::Point3f(-LAHW, -LAHH, 0.0f),
		cv::Point3f(LAHW, -LAHH, 0.0f),
		cv::Point3f(LAHW, LAHH, 0.0f)
	};

class Solver : public modules::Solver {
private:
    Eigen::Matrix3d cameraIntrinsicMatrix;
    Eigen::Vector3d cameraOffset;
    Eigen::Vector5d distorationCoefficients;// k1,k2,p1,p2,k3
public:
    XYV world2camera(const PYD& point, const ImuData& imuData) override;
    PYDs camera2world(const ArmorXYVs& trackResults, const ImuData& imuData, bool isLarge) override;
    void setCameraIntrinsicMatrix(const Eigen::Matrix3d& cameraIntrinsicMatrix) override;
    void setCameraOffset(const Eigen::Vector3d& cameraOffset) override;
    void setDistorationCoefficients(const Eigen::Vector5d& distorationCoefficients) override;
};

//A noneed statement only for reminding you.
using modules::createSolver;

} // namespace solver
