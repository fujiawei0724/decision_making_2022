/*
    Copyright [2019] Jian ZhiQiang
*/

#include "Common.hpp"

// 更新地图信息，ros服务(TODO)
void DecisionMaking::SubVehicle::updateMapInformation() {
    do {
        // 初始化道路
        this->left_lane_ = Lane();
        this->right_lane_ = Lane();
        this->center_lane_ = Lane();
        // 调用地图服务，提供服务所需参数
        vec_map_cpp_msgs::GetGuidedCurves map_service;
        geometry_msgs::PoseStamped current_pose;
        current_pose.header.frame_id = "world";
        current_pose.header.stamp = ros::Time::now();
        this->current_vehicle_world_position_mutex_.lock();
        // // 以车头的中心点作为中心道路锚点
        // double vehicle_head_x, vehicle_head_y, vehicle_rear_axis_center_scale;
        // this->nh_.getParam("vehicle_rear_axis_center_scale", vehicle_rear_axis_center_scale);
        // vehicle_head_x = this->current_vehicle_world_position_.position_.x_ + vehicle_rear_axis_center_scale * this->vehicle_length_ * cos(this->current_vehicle_world_position_.theta_/2.0);
        // vehicle_head_y = this->current_vehicle_world_position_.position_.y_ + vehicle_rear_axis_center_scale * this->vehicle_length_ * sin(this->current_vehicle_world_position_.theta_/2.0);
        // current_pose.pose.position.x = vehicle_head_x;
        // current_pose.pose.position.y = vehicle_head_y;
        current_pose.pose.position.x = this->current_vehicle_world_position_.position_.x_;
        current_pose.pose.position.y = this->current_vehicle_world_position_.position_.y_;
        current_pose.pose.orientation.x = 0.0;
        current_pose.pose.orientation.y = 0.0;
        current_pose.pose.orientation.z = sin(this->current_vehicle_world_position_.theta_/2.0);
        current_pose.pose.orientation.w = cos(this->current_vehicle_world_position_.theta_/2.0);
        this->current_vehicle_world_position_mutex_.unlock();
        current_pose.pose.position.z = 0;
        LOG(INFO) << "向地图请求的当前位置为" << std::setprecision(14) << current_pose.pose.position.x << "||" << std::setprecision(14) << current_pose.pose.position.y << "||" << std::setprecision(14) << std::atan2(current_pose.pose.orientation.z, current_pose.pose.orientation.w) * 2.0;
        std::cout << "向地图请求的当前位置为" << std::setprecision(14) << current_pose.pose.position.x << "||" << std::setprecision(14) << current_pose.pose.position.y << "||" << std::setprecision(14) << std::atan2(current_pose.pose.orientation.z, current_pose.pose.orientation.w) * 2.0 << std::endl;
        map_service.request.current_pose = current_pose;
        bool current_pose_ignore_orientation;
        this->nh_.getParam("current_pose_ignore_orientation", current_pose_ignore_orientation);
        map_service.request.current_pose_orientation_unknown = current_pose_ignore_orientation;
        // this->destination_mutex_.lock();
        // map_service.request.goal_pose = this->destination_pose_;
        // this->destination_mutex_.unlock();
        // bool goal_pose_ignore_orientation;
        // this->nh_.getParam("goal_pose_ignore_orientation", goal_pose_ignore_orientation);
        // map_service.request.goal_pose_orientation_unknown = goal_pose_ignore_orientation;
        map_service.request.point_margin = LANE_GAP_DISTANCE;
        std::vector<size_t> ignore_traffic_obstacle_ids;
        for (size_t i = 0; i < this->traffic_rule_obstacles_raw_.size(); i++) {
            if (this->traffic_rule_obstacles_raw_[i].mode == vec_map_cpp_msgs::VirtualObstacle::INVALID) {
                ignore_traffic_obstacle_ids.push_back(this->traffic_rule_obstacles_raw_[i].id);
                LOG(INFO) << "删除的id为" << this->traffic_rule_obstacles_raw_[i].id;
            }
        }

        map_service.request.ignored_ids = ignore_traffic_obstacle_ids;
        
        int failed_call_map_service_num = 0;
        while (true) {
            bool is_success = this->map_service_client_.call(map_service);
            if (is_success) {
                break;
            }
            failed_call_map_service_num += 1;
            if (failed_call_map_service_num >= 100) {
                LOG(INFO) << "Called the map service failed 100 times, process has exited.";
                std::cout << "Called the map service failed 100 times, process has exited." <<std::endl;
                exit(0);
            }
        }
        // Judge whether the goal point is unreachable
        if (map_service.response.status == vec_map_cpp_msgs::GetGuidedCurvesResponse::GOAL_UNREACHABLE) {
            LOG(INFO) << "The goal is unreachable";
            continue;
        }
        // 获取服务返回值，对于起点和终点的判断
        if (map_service.response.current_pose_state == vec_map_cpp_msgs::GetGuidedCurvesResponse::WRONG_ORIENTATION) {
            LOG(INFO) << "地图服务得到的起点方向错误";
            continue;
        } else if (map_service.response.current_pose_state == vec_map_cpp_msgs::GetGuidedCurvesResponse::OUT_MAP) {
            LOG(INFO) << "地图服务得到的起点在地图外";
            continue;
        }

        // 判断车道长度
        if (map_service.response.center_lane.geometry.points.size() < static_cast<size_t>(15.0 / LANE_GAP_DISTANCE)) {
            LOG(INFO) << "给出中间道路过短" << map_service.response.center_lane.geometry.points.size();
            continue;
        }

        // 判断是否在停车位内
        this->ROTATE_AND_REVERSE_ENABLE_FLAG_ = false;
        if (map_service.response.current_pose_state == vec_map_cpp_msgs::GetGuidedCurvesResponse::PARKING_ISLAND) {
            LOG(INFO) << "在停车位内,允许转向和倒车";
            this->ROTATE_AND_REVERSE_ENABLE_FLAG_ = true;
        }

        // if (map_service.response.goal_pose_state == vec_map_cpp_msgs::GetGuidedCurvesResponse::WRONG_ORIENTATION) {
        //     LOG(INFO) << "地图服务得到的终点方向错误";
        // } else if (map_service.response.goal_pose_state == vec_map_cpp_msgs::GetGuidedCurvesResponse::OUT_MAP) {
        //     LOG(INFO) << "地图服务得到的终点在地图外";
        // }

        // if (map_service.response.goal_reachable == false) {
        //     LOG(INFO) << "地图服务得到的终点不可达";
        // }

        // 获取服务的返回值,首先是中间车道
        this->center_lane_.enable();
        // 判断是否需要连续换道
        if (map_service.response.multiple_lane_changes) {
            // 需要
            std::vector<double> max_speeds, min_speeds;
            for (auto max_speed: map_service.response.center_lane.max_speeds) {
                max_speeds.push_back(0.5 * max_speed);
            }
            this->center_lane_.setLaneVelocityLimitation(max_speeds);

            for (auto min_speed: map_service.response.center_lane.min_speeds) {
                min_speeds.push_back(0.5 * min_speed);
            }
            this->center_lane_.setLowVelocity(min_speeds);
        } else {
            // 不需要
            this->center_lane_.setLaneVelocityLimitation(map_service.response.center_lane.max_speeds);
            this->center_lane_.setLowVelocity(map_service.response.center_lane.min_speeds);
        }
        
        // 附加打灯情况
        this->center_lane_.setTurn(map_service.response.center_lane.turn);
        // for (size_t i = 0; i < map_service.response.center_lane.max_speeds.size(); i++) {
        //     std::cout << "asfwgax " << map_service.response.center_lane.min_speeds[i] << std::endl;
        // }
        this->center_lane_.generateLaneCenter(map_service.response.center_lane.geometry);
        this->center_lane_.generateLaneTransMatrix();
        // 判断左侧车道是否存在
        if (map_service.response.left_lane_exist) {
            // 判断左道长度
            if (map_service.response.left_lane.geometry.points.size() < static_cast<size_t>(15.0 / LANE_GAP_DISTANCE)) {
                LOG(INFO) << "给出左侧道路过短" << map_service.response.left_lane.geometry.points.size();
                continue;
            }
            this->left_lane_.enable();
            // 判断是否需要连续换道
            if (map_service.response.multiple_lane_changes) {
                // 需要
                std::vector<double> max_speeds, min_speeds;
                for (auto max_speed: map_service.response.left_lane.max_speeds) {
                    max_speeds.push_back(0.5 * max_speed);
                }
                this->left_lane_.setLaneVelocityLimitation(max_speeds);

                for (auto min_speed: map_service.response.left_lane.min_speeds) {
                    min_speeds.push_back(0.5 * min_speed);
                }
                this->left_lane_.setLowVelocity(min_speeds);
            } else {
                // 不需要
                this->left_lane_.setLaneVelocityLimitation(map_service.response.left_lane.max_speeds);
                this->left_lane_.setLowVelocity(map_service.response.left_lane.min_speeds);
            }
            this->left_lane_.generateLaneCenter(map_service.response.left_lane.geometry);
            this->left_lane_.generateLaneTransMatrix();
            // 附加打灯情况
            this->left_lane_.setTurn(map_service.response.left_lane.turn);
        } else {
            this->left_lane_.disable();
        }
        // 判断右侧车道是否存在
        if (map_service.response.right_lane_exist) {
            // 判断右道长度
            if (map_service.response.right_lane.geometry.points.size() < static_cast<size_t>(15.0 / LANE_GAP_DISTANCE)) {
                LOG(INFO) << "给出右侧道路过短" << map_service.response.right_lane.geometry.points.size();
                continue;
            }
            this->right_lane_.enable();
            // 判断是否需要连续换道
            if (map_service.response.multiple_lane_changes) {
                // 需要
                std::vector<double> max_speeds, min_speeds;
                for (auto max_speed: map_service.response.right_lane.max_speeds) {
                    max_speeds.push_back(0.5 * max_speed);
                }
                this->right_lane_.setLaneVelocityLimitation(max_speeds);

                for (auto min_speed: map_service.response.right_lane.min_speeds) {
                    min_speeds.push_back(0.5 * min_speed);
                }
                this->right_lane_.setLowVelocity(min_speeds);
            } else {
                // 不需要
                this->right_lane_.setLaneVelocityLimitation(map_service.response.right_lane.max_speeds);
                this->right_lane_.setLowVelocity(map_service.response.right_lane.min_speeds);
            }
            this->right_lane_.generateLaneCenter(map_service.response.right_lane.geometry);
            this->right_lane_.generateLaneTransMatrix();
            // 附加打灯情况
            this->right_lane_.setTurn(map_service.response.right_lane.turn);
        } else {
            this->right_lane_.disable();
        }
        // 确定引导道路类型
        this->guidance_type_ = map_service.response.guidance;
        LOG(INFO) << "guided type: "<< this->guidance_type_;
        // std::cout << "guided type raw: "<< Lane::GuidanceType::ALL_AVAILABLE << std::endl;
        // if (this->guidance_type_ == Lane::GuidanceType::CHANGE_LEFT) {
        //     this->right_lane_.disable();
        // } else if (this->guidance_type_ == Lane::GuidanceType::CHANGE_RIGHT) {
        //     this->left_lane_.disable();
        // }

        // 设置道路优先级
        // 确定中间道的优先级
        switch (this->guidance_type_) {
            case Lane::GuidanceType::CHANGE_LEFT:
                this->center_lane_.setLanePriority(PRIORITY_INIT_VALUE_MEDIAN);
                break;
            case Lane::GuidanceType::KEEP_CENTER:
                this->center_lane_.setLanePriority(PRIORITY_INIT_VALUE_MAX);
                break;
            case Lane::GuidanceType::CHANGE_RIGHT:
                this->center_lane_.setLanePriority(PRIORITY_INIT_VALUE_MEDIAN);
                break;
            case Lane::GuidanceType::CENTER_LEFT:
                this->center_lane_.setLanePriority(PRIORITY_INIT_VALUE_MAX);
                break;
            case Lane::GuidanceType::CENTER_RIGHT:
                this->center_lane_.setLanePriority(PRIORITY_INIT_VALUE_MAX);
                break;
            case Lane::GuidanceType::ALL_AVAILABLE:
                this->center_lane_.setLanePriority(PRIORITY_INIT_VALUE_MAX);
                break;
            default:
                this->center_lane_.setLanePriority(PRIORITY_INIT_VALUE_MEDIAN);
                break;
        }
        LOG(INFO) << "中间车道优先级" << this->center_lane_.getLanePriority();
        // 设置左道优先级
        if (this->left_lane_.getLaneExistance()) {
            // 如果左道存在
            switch (this->guidance_type_) {
                case Lane::GuidanceType::CHANGE_LEFT:
                    this->left_lane_.setLanePriority(PRIORITY_INIT_VALUE_MAX);
                    break;
                case Lane::GuidanceType::KEEP_CENTER:
                    this->left_lane_.setLanePriority(PRIORITY_INIT_VALUE_MEDIAN);
                    break;
                case Lane::GuidanceType::CHANGE_RIGHT:
                    this->left_lane_.setLanePriority(PRIORITY_INIT_VALUE_MIN);
                    break;
                case Lane::GuidanceType::CENTER_LEFT:
                    this->left_lane_.setLanePriority(PRIORITY_INIT_VALUE_HIGH);
                    break;
                case Lane::GuidanceType::CENTER_RIGHT:
                    this->left_lane_.setLanePriority(PRIORITY_INIT_VALUE_LOW);
                    break;
                case Lane::GuidanceType::ALL_AVAILABLE:
                    this->left_lane_.setLanePriority(PRIORITY_INIT_VALUE_HIGH);
                    break;
                default:
                    this->left_lane_.setLanePriority(PRIORITY_INIT_VALUE_MEDIAN);
                    break;
            }
            LOG(INFO) << "左边车道优先级" << this->left_lane_.getLanePriority();
        }
        // 设置右道优先级
        if (this->right_lane_.getLaneExistance()) {
            switch (this->guidance_type_) {
                case Lane::GuidanceType::CHANGE_LEFT:
                    this->right_lane_.setLanePriority(PRIORITY_INIT_VALUE_MIN);
                    break;
                case Lane::GuidanceType::KEEP_CENTER:
                    this->right_lane_.setLanePriority(PRIORITY_INIT_VALUE_MEDIAN);
                    break;
                case Lane::GuidanceType::CHANGE_RIGHT:
                    this->right_lane_.setLanePriority(PRIORITY_INIT_VALUE_MAX);
                    break;
                case Lane::GuidanceType::CENTER_LEFT:
                    this->right_lane_.setLanePriority(PRIORITY_INIT_VALUE_LOW);
                    break;
                case Lane::GuidanceType::CENTER_RIGHT:
                    this->right_lane_.setLanePriority(PRIORITY_INIT_VALUE_HIGH);
                    break;
                case Lane::GuidanceType::ALL_AVAILABLE:
                    this->right_lane_.setLanePriority(PRIORITY_INIT_VALUE_HIGH);
                    break;
                default:
                    this->right_lane_.setLanePriority(PRIORITY_INIT_VALUE_MEDIAN);
                    break;
            }
            LOG(INFO) << "右边车道优先级" << this->right_lane_.getLanePriority();
        }

        // 当前位置最大限速
        this->expected_velocity_upper_bound_ = this->center_lane_.getLaneVelocityLimitation()[0];

        // 获取交通规则生成的障碍物
        this->traffic_rule_obstacles_raw_ = map_service.response.virtual_obstacles;
        VisualizationMethods::visualizeTrafficRules(this->traffic_rule_obstacles_raw_, this->visualization_pub_);

        // for (size_t i = 0; i < this->traffic_rule_obstacles_raw_.size(); i++) {
        //     if (this->traffic_rule_obstacles_raw_[i].points.size() > 0) {
        //         LOG(INFO) << " traffic obstacle is " << this->traffic_rule_obstacles_raw_[i].points[0].x << "||" << this->traffic_rule_obstacles_raw_[i].points[0].y;
        //         std::cout << " traffic obstacle is " << this->traffic_rule_obstacles_raw_[i].points[0].x << "||" << this->traffic_rule_obstacles_raw_[i].points[0].y << std::endl;
        //     } else {
        //         LOG(INFO) << "traffic obstacle size error";
        //         std::cout << "traffic obstacle size error" << std::endl;
        //     }

        // }

        // 判断是否为单车道
        if (!map_service.response.right_lane_exist && !map_service.response.left_lane_exist) {
            this->is_single_lane_ = true;
        } else {
            this->is_single_lane_ = false;
        }
        // std::cout << "hgugugu" << std::endl;

        // 获取是否允许避障标志位
        this->is_avoidance_capable_ = map_service.response.obstacle_avoidance_allowed;
        // this->is_avoidance_capable_ = true;
        // DEBUG
        // if (this->is_single_lane_) {
        //     this->is_avoidance_capable_ = false;
        // } else {
        //     this->is_avoidance_capable_ = true;
        // }
        // std::cout << "sgwbafsha" << std::endl;
        // 离终点的距离
        this->distance_to_goal_ = map_service.response.distance_to_goal;
        // TOFIX判断长度是否足够
        double shortest_distance = std::min(map_service.response.distance_to_stop, map_service.response.distance_to_goal);
        if (Tools::isLarge(shortest_distance, std::max(NAVIGATION_LENGTH_ENOUGH_MIN_VALUE, this->expected_velocity_upper_bound_ * NAVIGATION_LENGTH_ENOUGH_MIN_COEF))) {
            this->is_length_enough_ = true;
        } else {
            this->is_length_enough_ = false;
        }
        // this->is_length_enough_ = true;
        this->remain_distance_ = shortest_distance;
        LOG(INFO) << "可以自由行驶的距离还剩" << shortest_distance << "，是否足够" << this->is_length_enough_;
        std::cout << "可以自由行驶的距离还剩" << shortest_distance << "，是否足够" << this->is_length_enough_ << std::endl;

        // if (map_service.response.extra_flags != "") {
        //     if (map_service.response.extra_flags == "HIGHWAY_DOWN_MIDDLE") {
        //         GLOBAL_IS_IN_CHECK_ = true;
        //         // std::cout << "在临检区" << std::endl;
        //     } else if (map_service.response.extra_flags == "BLIND_ZONE"){
        //         GLOBAL_IS_IN_SLOW_ = true;
        //         // std::cout << "不在临检区" << std::endl;
        //     } else if (map_service.response.extra_flags == "OVERTAKE") {
        //         GLOBAL_IS_IN_OVERTAKE_ = true;
        //     }
        // } else {
        //     GLOBAL_IS_IN_CHECK_ = false;
        //     GLOBAL_IS_IN_SLOW_ = false;
        //     GLOBAL_IS_IN_OVERTAKE_ = false;
        //     GLOBAL_IS_IN_CHECK_COUNT_FLAG_ = 0;
        // }
        break;
    } while (true);

}
