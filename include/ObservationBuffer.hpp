/*
 * @Author: fujiawei0724
 * @Date: 2022-07-16 18:40:30
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-07-16 19:30:10
 * @Description: the storage of observations sequence.
 */

#pragma once
#include <chrono>
#include "Const.hpp"
#include "VehicleState.hpp"

namespace Utils {

class ObservationBuffer {
 public:
    ObservationBuffer(int full_size = 10);
    ~ObservationBuffer();

    inline int size() {return static_cast<int>(data_.size());}

    /**
     * @description: update the information in the buffer from the current percepted information and filter these observations whose timestamp is not suitable
     * @return {*}
     */    
    void update(const std::unordered_map<int, Common::FsImageVehicle>& current_surround_vehicles, const std::chrono::steady_clock::time_point& cur_time_stamp);

    /**
     * @description: transform the formation of stored observations and output 
     * @param {lane_info} lane existence and width information, which are used to construct the image
     * @return a sequence of observations
     */
    std::vector<cv::Mat> output(const std::vector<double>& lane_info);




    int full_size_{0};
    std::chrono::steady_clock::time_point last_update_time_stamp_;
    std::vector<std::unordered_map<int, Common::FsImageVehicle>> data_;
    

};

} // End of name space Utils
