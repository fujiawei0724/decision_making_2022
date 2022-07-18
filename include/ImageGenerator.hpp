/*
 * @Author: fujiawei0724
 * @Date: 2022-07-10 16:22:38
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-07-18 08:29:26
 * @Description: Generate image from the percepted obstacles.
 */

#pragma once

#include "Const.hpp"
#include "VehicleState.hpp"

namespace Utils {

class ImageGenerator {
public:

    static constexpr int height_{300};
    static constexpr int width_{50};
    static constexpr double scale_{3.0};
    static constexpr double lane_width_{3.5};

    /**
     * @description: generate image from provided surrounding vehicle in frenet frame.
     * @return matrix with the specific size.
     */    
    static cv::Mat generateSingleImage(const std::vector<double>& lane_info, const std::vector<Common::FsImageVehicle>& surrounding_vehicles);


    /**
     * @description: transform a frenet position to the corresponding position in image.
     * @return corresponding position in image.
     */
    static cv::Point positionTransform(const Eigen::Matrix<double, 2, 1>& frenet_pos);

};

} // End of namespace Utils
