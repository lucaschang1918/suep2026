// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__ARMOR_HPP_
#define ARMOR_DETECTOR__ARMOR_HPP_

#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>  
// STL
#include <algorithm>
#include <string>

namespace rm_auto_aim
{
  const int RED = 1;
  const int BLUE = 0;

  enum class ArmorType { SMALL, LARGE, INVALID };
  const std::string ARMOR_TYPE_STR[3] = {"small", "large", "invalid"};

struct Armor
{
  Armor() = default;
  ~Armor() = default;

  cv::Point tl,tr,bl,br;
  float modelconf;
  cv::Point center;
  ArmorType type;
  int color;
  // Number part
  cv::Mat number_img;
  std::string number;
  float confidence;
  std::string classfication_result;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__ARMOR_HPP_
