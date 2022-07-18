/*
 * @Author: fujiawei0724
 * @Date: 2022-07-16 18:44:05
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-07-18 14:50:16
 * @Description: the storage of observations sequence.
 */

#include "Common.hpp"

namespace Utils {

ObservationBuffer::ObservationBuffer(int full_size) {
    full_size_ = full_size;
}
ObservationBuffer::~ObservationBuffer() = default;

/**
 * @description: update the information in the buffer from the current percepted information and filter these observations whose timestamp is not suitable
 * @return {*}
 */    
void ObservationBuffer::update(const std::vector<Common::FsImageVehicle>& current_surround_vehicles, const std::chrono::steady_clock::time_point& cur_time_stamp) {
    if (size() == 0) {
        // Update data if buffer is empty
        data_.emplace_back(current_surround_vehicles);
        last_update_time_stamp_ = cur_time_stamp;
    }
    
    // Get the time stamp difference between two continuous update
    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(cur_time_stamp - last_update_time_stamp_);
    double time_diff = static_cast<double>(time_span.count());

    // Update stored data
    if (time_diff >= 0.2) {
        if (time_diff > 0.25) {
            printf("[ObservationBuffer] observations time gap is too large: %lf.\n", time_diff);
        }
        data_.emplace_back(current_surround_vehicles);
        last_update_time_stamp_ = cur_time_stamp;
        if (size() > full_size_) {
            data_.erase(data_.begin());
        }
        // Update data
    }
}

/**
 * @description: fill the buffer when the number of the observations is not enough
 * @return {*}
 */    
void ObservationBuffer::selfFill() {
    while (size() < full_size_) {
        std::vector<Common::FsImageVehicle> previous_surround_vehicles = data_[size() - 1];
        data_.emplace_back(previous_surround_vehicles);
    }
}


/**
 * @description: transform the formation of stored observations and output 
 * @param {lane_info} lane existence and width information, which are used to construct the image
 * @return a sequence of observations
 */
std::vector<cv::Mat> ObservationBuffer::output(const std::vector<double>& lane_info) const {
    assert(size() == full_size_);
    std::vector<cv::Mat> observations;
    for (int i = 0; i < full_size_; i++) {
        observations.emplace_back(ImageGenerator::generateSingleImage(lane_info, data_[i]));
    }
    return observations;
}



} // End of namespace Utils
