// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__DETECTOR_HPP_
#define ARMOR_DETECTOR__DETECTOR_HPP_

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

// STD
#include <cmath>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/number_classifier.hpp"
#include "auto_aim_interfaces/msg/debug_armors.hpp"
#include "auto_aim_interfaces/msg/debug_lights.hpp"


// std::call_once所需的头文件
#include <algorithm>
#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>     //opencv header file
#include <openvino/openvino.hpp>  //openvino header file
#include <thread>

//#define MODEL_PATH "/home/gx/桌面/pplc_416train_last_int/last.xml"
#define SCORE_THRESHOLD 0.72
#define NMS_THRESHOLD 0.3
using namespace cv;
using namespace dnn;
/*
 * 整体重写的方向为：
 * 1、成为一个独立头文件
 * 2、方便其他文件调用此文件各种函数（安全就不考虑了）
 * 3、分为推理+输出+可视化 ，三部分
 * 4、提供更多的宏定义，便于调整参数（此处就无所谓滥用宏了）
 */

//******************推理**************
// -------- Step 1. Initialize OpenVINO Runtime Core --------
static ov::Core core;
static ov::CompiledModel compiled_model;
static ov::InferRequest infer_request;
static ov::Output<const ov::Node> input_port;

static std::once_flag flag;  // 只运行一次的标志

struct dataImg
{
  float scale;
  cv::Mat blob;
  cv::Mat input;
};
struct OneClassArmor
{
  std::vector<float> class_scores;
  std::vector<Rect> boxes;
  std::vector<std::vector<float>> objects_keypoints;
  int class_ids;
};
struct OneArmor
{
  float class_scores;
  cv::Rect box;
  cv::Point objects_keypoints[4];
  int class_ids;
};

dataImg processImage(const cv::Mat & input, cv::Size new_shape, cv::Scalar color);  //图片预处理
std::vector<OneArmor> startInferAndNMS(dataImg data, int detect_color,std::string MODEL_PATH);  //开始推理并返回结果

void init();  //初始化

//********调整结果输出************
//在rv中，一快armor由两条灯带和板子写的type（单独算）组成，在这里，直接定义一块板子的一对灯条
//不在此处写，在rv的matchLights做处理
//**************可视化***********（先不写）

namespace rm_auto_aim
{
class Detector
{
public:
  struct ArmorParams
  {
    double min_light_ratio;

    // 小目标中心点间的最小和最大距离
    double min_small_center_distance;
    double max_small_center_distance;
    // 大目标中心点间的最小和最大距离
    double min_large_center_distance;
    double max_large_center_distance;
    // 水平角度的最大值，用于限制检测范围
    double max_angle;
    long int binary_threshold_light;
    
  };

  Detector(std::string model_path_yolo,const int & color, const ArmorParams & a);

  std::vector<Armor> detect(const cv::Mat & input);
  std::vector<Armor> getArmors(const std::vector<OneArmor> & armors_data);
  cv::Mat outputImage;

  // For debug usage
  cv::Mat getAllNumbersImage();
  void drawResults(cv::Mat & img);

  std::string MODEL_PATH;
  int detect_color;
  
  ArmorParams a;

  std::unique_ptr<NumberClassifier> classifier;

private:
  ArmorType isArmor(const OneArmor & armor_data) const;

  std::vector<Armor> armors_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_HPP_
