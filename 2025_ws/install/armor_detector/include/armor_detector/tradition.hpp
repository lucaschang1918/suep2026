//
// Created by gx on 24-4-28.
//
/*
 * 此文件编写方向：
 *      在神经网络推理的矩形框内得到灯条的四个角点
 *      采用覆盖原神经网络四个角点的过程
 */
#ifndef V8_INFERENCE_H_

#include "armor_detector/detector.hpp"

//const int binary_thres=120;

const int RED = 0;
const int BLUE = 1;
//LightParams



// width / height
const double min_ratio=0.1;
const double max_ratio=0.4;
// vertical angle
const double max_angle=45.0;

struct Light : public cv::RotatedRect
{
    Light() = default;
    explicit Light(cv::RotatedRect box) : cv::RotatedRect(box)
    {
        cv::Point2f p[4];
        box.points(p);
        std::sort(p, p + 4, [](const cv::Point2f & a, const cv::Point2f & b) { return a.y < b.y; });
        top = (p[0] + p[1]) / 2;
        bottom = (p[2] + p[3]) / 2;

        length = cv::norm(top - bottom);
        width = cv::norm(p[0] - p[1]);

        tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
        tilt_angle = tilt_angle / CV_PI * 180;
    }

    int color;
    cv::Point2f top, bottom;
    double length;
    double width;
    float tilt_angle;
};

//functions
cv::Mat preprocessROI(const cv::Mat& img,cv::Rect armor_box,int binary_threshold_light);
bool isLight(const Light &light);
std::vector<Light> findLight(const cv::Mat& rgb_img, const cv::Mat &binary_img);
std::vector<Light> sortLight(std::vector<Light> &lights);
std::vector<OneArmor> tradition(cv::Mat input_img,std::vector<OneArmor>& armors_data,int binary_threshold_light);

#define V8_INFERENCE_H_

#endif //V8_INFERENCE_H























