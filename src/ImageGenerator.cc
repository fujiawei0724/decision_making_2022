/*
 * @Author: fujiawei0724
 * @Date: 2022-07-10 21:55:14
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-07-27 12:47:40
 * @Description: Generate image from the percepted obstacles.
 */

#include "Common.hpp"

namespace Utils {

constexpr int ImageGenerator::height_;
constexpr int ImageGenerator::width_;
constexpr double ImageGenerator::scale_;
constexpr double ImageGenerator::lane_width_;

cv::Mat ImageGenerator::generateSingleImage(const std::vector<double>& lane_info, const std::vector<Common::FsImageVehicle>& surrounding_vehicles) {
    // Initialize canvas 
    cv::Mat img = cv::Mat(ImageGenerator::height_, ImageGenerator::width_, CV_8UC1, cv::Scalar(0));

    // supply lanes
    cv::line(img, cv::Point(std::round(ImageGenerator::width_ / 2), 0), cv::Point(std::round(ImageGenerator::width_ / 2), ImageGenerator::height_), cv::Scalar(255), std::round(ImageGenerator::lane_width_ * ImageGenerator::scale_));
    if (lane_info[0] == 1) {
        cv::line(img, cv::Point(std::round(ImageGenerator::width_ / 2 + lane_info[2] * ImageGenerator::scale_ / 2), 0), cv::Point(std::round(ImageGenerator::width_ / 2 + lane_info[2] * ImageGenerator::scale_ / 2), ImageGenerator::height_), cv::Scalar(255), std::round(ImageGenerator::lane_width_ * ImageGenerator::scale_));
    }
    if (lane_info[1] == 1) {
        cv::line(img, cv::Point(std::round(ImageGenerator::width_ / 2 - lane_info[3] * ImageGenerator::scale_ / 2), 0), cv::Point(std::round(ImageGenerator::width_ / 2 - lane_info[3] * ImageGenerator::scale_ / 2), ImageGenerator::height_), cv::Scalar(255), std::round(ImageGenerator::lane_width_ * ImageGenerator::scale_));
    }

    // supply surrounding vehicles
    std::vector<std::vector<cv::Point>> contours;
    for (auto& sur_image_veh : surrounding_vehicles) {
        FsImageVehicle cur_sur_image_veh = sur_image_veh;
        cv::Point2f vertex[4];
        cv::RotatedRect box(ImageGenerator::positionTransform(cur_sur_image_veh.position_), cv::Size(std::round(cur_sur_image_veh.length_ * ImageGenerator::scale_), std::round(cur_sur_image_veh.width_ * ImageGenerator::scale_)), -cur_sur_image_veh.theta_ * 180.0 / M_PI + 90.0);
        box.points(vertex);
        std::vector<cv::Point> cur_contour;
        for (int i = 0; i < 4; i++) {
            cur_contour.emplace_back(cv::Point(std::round(vertex[i].x), std::round(vertex[i].y)));
        }
        contours.emplace_back(cur_contour);
    }
    cv::fillPoly(img, contours, cv::Scalar(0), 8);

    // // DEBUG
    // cv::imshow("Observation", img);
    // cv::waitKey(0);
    // // END DEBUG

    return img / 255.0;

}

cv::Point ImageGenerator::positionTransform(const Eigen::Matrix<double, 2, 1>& frenet_pos) {
    Eigen::Matrix<double, 2, 1> deformed_frenet_pos = frenet_pos * ImageGenerator::scale_;
    return cv::Point(std::round(ImageGenerator::width_ / 2) - deformed_frenet_pos(1, 0), ImageGenerator::height_ - deformed_frenet_pos(0, 0));
}




} // End of namespace Utils
