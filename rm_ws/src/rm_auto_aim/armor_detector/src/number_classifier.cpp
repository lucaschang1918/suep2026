//
// Created by rm_autoaim on 2025/9/14.
//

#include "number_classifier.hpp"
#include "armor.hpp"
//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn.hpp>
//stl
#include <algorithm>
#include <cstddef>
#include <fstream>
#include <map>
#include <string>
#include <vector>
#include <openvino/runtime/properties.hpp>

namespace rm_auto_aim {
    NumberClassifier::NumberClassifier(const std::string &model_path,
                                       const std::string &label_path,
                                       const double threshold,
                                       const std::vector<std::string> &ignore_classes) : threshold(threshold),
        ignore_classes_(ignore_classes) {
        net_ = cv::dnn::readNetFromONNX(model_path);

        std::ifstream label_file(label_path);
        std::string line;
        while (std::getline(label_file, line)) {
            class_names_.push_back(line);
        }
    }

    void NumberClassifier::extractNumbers(const cv::Mat &src, std::vector<Armor> &armors) {
        // static int num = 0;
        // Light length in image
        const int light_length = 12;
        // Image size after warp
        const int warp_height = 28;
        const int small_armor_width = 32;
        const int large_armor_width = 54;
        // Number ROI size
        const cv::Size roi_size(20, 28);

        for (auto &armor: armors) {
            // Warp perspective transform
            cv::Point2f lights_vertices[4] = {
                armor.objects_keypoints[0], // 左上 (top-left)
                armor.objects_keypoints[3], // 右上 (top-right)
                armor.objects_keypoints[2], // 右下 (bottom-right)
                armor.objects_keypoints[1] // 左下 (bottom-left)
            };

            const int top_light_y = (warp_height - light_length) / 2 - 1;
            const int bottom_light_y = top_light_y + light_length;
            const int warp_width = small_armor_width;

            cv::Point2f target_vertices[4] = {
                cv::Point(0, top_light_y), // 左上
                cv::Point(warp_width - 1, top_light_y), // 右上
                cv::Point(warp_width - 1, bottom_light_y), // 右下
                cv::Point(0, bottom_light_y) // 左下
            };

            cv::Mat number_image;
            auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
            cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));

            // Get ROI
            number_image =
                    number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

            // Binarize
            std::vector<cv::Mat> channels(3);
            cv::split(number_image, channels);

            cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
            //保存数字图案20*28
            // cv::imwrite("/home/gx/rm_classifier_training-main/补_/"+std::to_string(num++)+".jpg", number_image);
            cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
            // cv::Mat number_img_ = perform_opening(number_image,2);

            armor.number_img = number_image;
        }
    }

    void NumberClassifier::classify(std::vector<Armor> &armors) {
        for (auto &armor: armors) {
            cv::Mat image = armor.number_img.clone();
            image = image / 255.0;

            cv::Mat blob;
            cv::dnn::blobFromImage(image, blob);
            net_.setInput(blob);
            cv::Mat outputs = net_.forward();

            float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
            cv::Mat softmax_prob;
            cv::exp(outputs - max_prob, softmax_prob);
            float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
            //gui yi hua
            softmax_prob /= sum;

            double confidence;
            cv::Point class_id_point;
            cv::minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
            int label_id = class_id_point.x;

            armor.confidence = confidence;
            armor.number = class_names_[label_id];

            std::stringstream result_ss;
            result_ss << armor.number << ":" << std::fixed << std::setprecision(1)
                    << armor.confidence * 100.0 << "%";
            armor.classfication_result = result_ss.str();
        }

        armors.erase(
            std::remove_if(
                armors.begin(), armors.end(),
                [this](const Armor &armor) {
                    if (armor.confidence < threshold) {
                        return true;
                    }
                    for (const auto &ignore_classs: ignore_classes_) {
                        if (armor.number == ignore_classs) {
                            return true;
                        }
                    }
                }
            ),
            armors.end());
    }
}

