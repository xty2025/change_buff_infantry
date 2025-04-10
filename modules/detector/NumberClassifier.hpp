#pragma once

#include <opencv2/opencv.hpp>

namespace detector
{
    class NumberClassifier
    {
    public:
        explicit NumberClassifier(std::string mode);
        ~NumberClassifier();
        std::pair<int, double> predict(const cv::Mat &frame, const std::vector<cv::Point2f> &corners);
    private:
        cv::HOGDescriptor *hog_;
        void initHog();
        bool affineNumber(const cv::Mat &frame, const std::vector<cv::Point2f> &corners);
        float getDistance(const cv::Point2f &point_1, const cv::Point2f &point_2);
        void calcGammaTable(float gamma);
        uchar gamma_table[256];
        cv::Mat lut_table;

        void AutoGamma(const cv::Mat &img, cv::Mat &out);
        cv::Mat AutoGammaCorrect(const cv::Mat image);
        cv::Mat pixel(const cv::Mat image, double gamma, double mu, double sigma, int flag);
        bool loadModel(const std::string &model_path);
        cv::Ptr<cv::ml::SVM> number_svm_model;
        bool is_model_set;
        bool is_get_roi;
        cv::Mat number_roi;

        int mode_;
        cv::Mat class_;
    };
}