//
// Created by rm_autoaim on 2025/9/14.
//

#ifndef RM_AUTO_AIM_NUMBER_CLASSIFIER_HPP
#define RM_AUTO_AIM_NUMBER_CLASSIFIER_HPP
//opencv
#include <opencv2/opencv.hpp>
//stl
#include <cstddef>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "armor.hpp"

namespace rm_auto_aim {
    class NumberClassifier {
    public:
        NumberClassifier(
            const std::string & model_path, const std::string & label_path, const double threshold,
            const std::vector<std::string> & ignore_classes = {});

        void extractNumbers(const cv::Mat & src, std::vector<Armor> & armors);

        void classify(std::vector<Armor> & armors);


        double threshold;

    private:
        cv::dnn::Net net_;
        std::vector<std::string> class_names_;
        std::vector<std::string> ignore_classes_;


    };


}
#endif //RM_AUTO_AIM_NUMBER_CLASSIFIER_HPP