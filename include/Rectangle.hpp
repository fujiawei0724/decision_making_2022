/*
    Copyright [2019] Jian ZhiQiang
*/

#ifndef RECTANGLE_INCLUDE_COMMON_HPP_
#define RECTANGLE_INCLUDE_COMMON_HPP_

#include "Point.hpp"
#include "Path.hpp"
#include "LineSegment.hpp"
#include "Const.hpp"
#include <vector>

// 占用区域基本单位——矩形
class Rectangle {
 public:
    double center_x_;  // 矩形中心点x
    double center_y_;  // 矩形中心点y
    double width_;  // 矩形宽度
    double length_;  // 矩形长度
    double rotation_;  // 矩形朝向
    std::vector<PathPlanningUtilities::Point2f> points_;  // 矩形角点
    std::vector<PathPlanningUtilities::LineSegment> lines_;  // 矩形的边
    std::vector<Eigen::Matrix<double, 2, 1>> axes_; // The axes of rectangle

    // 构造函数
    Rectangle(){};

    // 析构函数
    ~Rectangle(){};

    // 构造函数
    Rectangle(double center_x, double center_y, double rotation, double width, double length) {
        this->center_x_ = center_x;
        this->center_y_ = center_y;
        this->rotation_ = rotation;
        this->width_ = width;
        this->length_ = length;
        this->generatePoints();
        this->generateLines();
        generateAxes();
    }

    // 求解矩形四个角点
    std::vector<PathPlanningUtilities::Point2f> getRectanglePoints(double center_x, double center_y, double rotation, double width, double length) {
        std::vector<PathPlanningUtilities::Point2f> points;
        points.resize(4);
        PathPlanningUtilities::Point2f point_1, point_2, point_3, point_4;
        point_1.x_ = center_x + length*0.5*cos(rotation) - width*0.5*sin(rotation);
        point_1.y_ = center_y + length*0.5*sin(rotation) + width*0.5*cos(rotation);
        points[0] = point_1;
        point_2.x_ = center_x + length*0.5*cos(rotation) + width*0.5*sin(rotation);
        point_2.y_ = center_y + length*0.5*sin(rotation) - width*0.5*cos(rotation);
        points[1] = point_2;
        point_3.x_ = center_x - length*0.5*cos(rotation) + width*0.5*sin(rotation);
        point_3.y_ = center_y - length*0.5*sin(rotation) - width*0.5*cos(rotation);
        points[2] = point_3;
        point_4.x_ = center_x - length*0.5*cos(rotation) - width*0.5*sin(rotation);
        point_4.y_ = center_y - length*0.5*sin(rotation) + width*0.5*cos(rotation);
        points[3] = point_4;
        return points;
    }


 private:
    // 计算矩形角点
    void generatePoints() {
        this->points_ = this->getRectanglePoints(this->center_x_, this->center_y_, this->rotation_, this->width_, this->length_);
    }
    
    // 计算矩形的边
    void generateLines() {
        for (size_t i = 0; i < 4; i++) {
            PathPlanningUtilities::LineSegment line = PathPlanningUtilities::LineSegment(this->points_[i % 4], this->points_[(i + 1) % 4]);
            this->lines_.push_back(line);
        }
    }

    // Calculate the axes of rectangle
    void generateAxes() {
        // For the first two vertex
        Eigen::Matrix<double, 2, 1> vec_1{points_[1].x_ - points_[0].x_, points_[1].y_ - points_[0].y_};
        double length_1 = vec_1.norm();
        Eigen::Matrix<double, 2, 1> normalized_vec_1 = vec_1 / length_1;
        Eigen::Matrix<double, 2, 1> axis_0{-normalized_vec_1(1), normalized_vec_1(0)};
        axes_.emplace_back(axis_0);

        // For the second and third vertex
        Eigen::Matrix<double, 2, 1> vec_2{points_[2].x_ - points_[1].x_, points_[2].y_ - points_[1].y_};
        double length_2 = vec_2.norm();
        Eigen::Matrix<double, 2, 1> normalized_vec_2 = vec_2 / length_2;
        Eigen::Matrix<double, 2, 1> axis_1{-normalized_vec_2(1), normalized_vec_2(0)};
        axes_.emplace_back(axis_1);
    }
};

#endif