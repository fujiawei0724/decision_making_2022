/*
    Copyright [2019] Jian ZhiQiang
*/

#include "Common.hpp"

// // 特殊处理
// bool GLOBAL_IS_IN_GROUND_ = false;
// std::mutex GLOBAL_IN_GROUND_MUTEX_;
// Rectangle GLOBAL_GROUND_AREA = {-11337.9472656, -2507.84375, 133.0, 142.0, 2.0476333396251585};
// bool GLOBAL_IS_IN_JIFEI_ = false;
// std::mutex GLOBAL_IN_JIFEI_MUTEX_;
// Rectangle GLOBAL_JIFEI_AREA = {-11356.0869141, -2458.27685547, 101.0, 20.0, 2.0231007985729406};
// bool GLOBAL_IS_IN_CHECK_ = false;
// bool GLOBAL_IS_IN_SLOW_ = false;
// bool GLOBAL_IS_IN_OVERTAKE_ = false;
// int GLOBAL_IS_IN_CHECK_COUNT_FLAG_ = 0;

// 构造函数和析构函数
DecisionMaking::SubVehicle::SubVehicle(const ros::NodeHandle &nh) {
    // 获取ros句柄
    this->nh_ = nh;
    // 初始化车辆的状态机
    // this->initVehicleStates();
    // // 初始化当前状态为停车状态
    // this->current_state_ = this->states_set_[StateNames::STOP];
    // 初始化车辆信息
    double vehicle_width, vehicle_length, vehicle_rear_axis_center_scale;
    this->nh_.getParam("vehicle_width", vehicle_width);
    this->nh_.getParam("vehicle_length", vehicle_length);
    this->nh_.getParam("vehicle_rear_axis_center_scale", vehicle_rear_axis_center_scale);
    this->vehicle_width_ = vehicle_width;
    this->vehicle_length_ = vehicle_length;
    this->vehicle_rear_axis_center_scale_ = vehicle_rear_axis_center_scale;

    // 初始化全局变量
    // 获取是否允许超车标志位
    this->nh_.getParam("is_overtake_enable", this->IS_OVERTAKE_ENABLE_FLAG_);
    // 获取环绕雷达是否使用标志位
    this->nh_.getParam("is_surround_radar_enable", this->IS_SURROUND_RADAR_ENABLE_FLAG_);
    // 获取交通灯使用标志位
    this->nh_.getParam("traffic_light_usage_flag", this->TRAFFIC_LIGHT_USAGE_FLAG_);
    // 获取临时空气墙是否使用标志位
    this->nh_.getParam("not_permanent_traffic_rule_usage_flag", this->NOT_PERMANENT_TRAFFIC_RULE_USAGE_FLAG_);
    // 获取是否为全自动模式标志位
    this->nh_.getParam("is_total_autonomous", this->IS_TOTAL_AUTONOMOUS_FLAG_);
    // 获取是否允许倒车和原地转向
    this->nh_.getParam("rotate_and_reverse_enable", this->ROTATE_AND_REVERSE_ENABLE_FLAG_);

    // 初始化ros相关节点和服务
    this->rosInit();

    // Construct
    hpdm_planner_ = new HpdmPlanner::HpdmPlannerCore();

    // // Load model
    module_ = torch::jit::load("/home/fjw/Desktop/model0.pt");
    module_.to(torch::kCUDA);

    ROS_INFO("INITAL SUCCESS");
    std::cout << "IS_OVERTAKE_ENABLE_FLAG: " << this->IS_OVERTAKE_ENABLE_FLAG_ << std::endl;
    std::cout << "IS_SURROUND_RADAR_ENABLE_FLAG: " << this->IS_SURROUND_RADAR_ENABLE_FLAG_ << std::endl;
    std::cout << "IS_TRAFFIC_LIGHT_USAGE_FLAG: " << this->TRAFFIC_LIGHT_USAGE_FLAG_ << std::endl;
    std::cout << "NOT_PERMANENT_TRAFFIC_RULE_USAGE_FLAG: " << this->NOT_PERMANENT_TRAFFIC_RULE_USAGE_FLAG_ << std::endl;
    std::cout << "IS_TOTAL_AUTONOMOUS_FLAG: " << this->IS_TOTAL_AUTONOMOUS_FLAG_ << std::endl;
    std::cout << "ROTATE_AND_REVERSE_ENABLE_FLAG: " << this->ROTATE_AND_REVERSE_ENABLE_FLAG_ << std::endl;
}

DecisionMaking::SubVehicle::~SubVehicle() {}


// 初始化并启动线程
void DecisionMaking::SubVehicle::runMotionPlanning() {
    // 初始化自检测对象
    // this->self_test_.add("Program checking", this, &DecisionMaking::SubVehicle::selfTestForProgress);
    // 启动各线程,线程一为订阅ros节点和服务，获取车辆信息，线程二为motionplanning和decisionmaking，计算并保持轨迹
    std::thread ros_msgs_receiver_thread(&DecisionMaking::SubVehicle::listenRosMSGThread, this);
    std::thread motion_planning_thread(&DecisionMaking::SubVehicle::motionPlanningThread, this);
    std::thread replanning_trigger_thread(&DecisionMaking::SubVehicle::triggerThread, this);
    ros_msgs_receiver_thread.join();
    motion_planning_thread.join();
    replanning_trigger_thread.join();
}


// 初始化ros节点
void DecisionMaking::SubVehicle::rosInit() {
    // 初始化可视化节点
    std::string visualization_topic, vis_vehicle_topic, vis_obstacle_topic, vis_collision_topic, vis_multi_curve_topic, vis_influence_obstacle_topic, vis_occupation_topic;
    this->nh_.getParam("visualization_topic", visualization_topic);
    this->nh_.getParam("vis_vehicle_topic", vis_vehicle_topic);
    this->nh_.getParam("vis_obstacle_topic", vis_obstacle_topic);
    this->nh_.getParam("vis_collision_topic", vis_collision_topic);
    this->nh_.getParam("vis_multi_curve_topic", vis_multi_curve_topic);
    this->nh_.getParam("vis_influence_obstacle_topic", vis_influence_obstacle_topic);
    this->nh_.getParam("vis_occupation_topic", vis_occupation_topic);
    this->visualization_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(visualization_topic, 10);
    this->vis_vehicle_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(vis_vehicle_topic, 10);
    this->vis_obstacle_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(vis_obstacle_topic, 10);
    this->vis_occupation_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(vis_occupation_topic, 10);
    // debug
    this->vis_collision_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(vis_collision_topic, 10);
    this->vis_multi_curve_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(vis_multi_curve_topic, 10);
    this->vis_influence_obstacle_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(vis_influence_obstacle_topic, 10);

    // Visualization behavior planner and trajectory planner
    std::string vis_behavior_planner_ego_states_topic;
    nh_.getParam("vis_behavior_planner_ego_states_topic", vis_behavior_planner_ego_states_topic);
    vis_behavior_planner_ego_states_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(vis_behavior_planner_ego_states_topic, 10);

    std::string vis_behavior_planner_candidates_states_topic;
    nh_.getParam("vis_behavior_planner_candidates_states_topic", vis_behavior_planner_candidates_states_topic);
    vis_behavior_planner_candidates_states_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(vis_behavior_planner_candidates_states_topic, 10);
    
    std::string vis_trajectory_planner_topic;
    nh_.getParam("vis_trajectory_planner_topic", vis_trajectory_planner_topic);
    vis_trajectory_planner_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(vis_trajectory_planner_topic, 10);

    // 获取tf
    this->tf_listener_ptr_ = new tf::TransformListener();

    // 获取位置topic
    std::string odometry_topic;
    this->nh_.getParam("odometry_topic", odometry_topic);
    this->odom_sub_ = this->nh_.subscribe(odometry_topic, 1, &DecisionMaking::SubVehicle::updateVehiclePosition, this);

    // 获取移动状态topic
    std::string movement_topic;
    this->nh_.getParam("movement_topic", movement_topic);
    this->movement_sub_ = this->nh_.subscribe(movement_topic, 1, &DecisionMaking::SubVehicle::updateVehicleMovement, this);

    // 获取曲率topic
    std::string curvature_topic;
    this->nh_.getParam("curvature_topic", curvature_topic);
    this->curvature_sub_ = this->nh_.subscribe(curvature_topic, 1, &DecisionMaking::SubVehicle::updateVehicleCurvature, this);

    // Get steer topic 
    std::string steer_topic;
    this->nh_.getParam("steer_topic", steer_topic);
    steer_sub_ = nh_.subscribe(steer_topic, 1, &DecisionMaking::SubVehicle::updateVehicleSteer, this);

    // Get acceleration topic
    std::string acceleration_topic;
    this->nh_.getParam("acceleration_topic", acceleration_topic);
    acceleration_sub_ = nh_.subscribe(acceleration_topic, 1, &DecisionMaking::SubVehicle::updateVehicleAcceleration, this);

    // 获取控制状态topic
    std::string control_report_topic;
    this->nh_.getParam("control_report_topic", control_report_topic);
    this->control_report_sub_ = this->nh_.subscribe(control_report_topic, 1, &DecisionMaking::SubVehicle::updateControlReport, this);

    // 开始任务服务
    std::string mission_start_service_name;
    this->nh_.getParam("mission_start_service", mission_start_service_name);
    this->mission_start_service_server_ = this->nh_.advertiseService(mission_start_service_name, &DecisionMaking::SubVehicle::startMission, this);


    // 障碍物topic
    std::string obstacle_topic;
    this->nh_.getParam("obstacle_topic", obstacle_topic);
    this->obstacle_sub_ = this->nh_.subscribe(obstacle_topic, 1, &DecisionMaking::SubVehicle::getObstacles, this);

    // 初始化地图服务
    std::string map_service_name;
    this->nh_.getParam("map_service", map_service_name);
    ros::service::waitForService(map_service_name);
    this->map_service_client_ = this->nh_.serviceClient<vec_map_cpp_msgs::GetGuidedCurves>(map_service_name);

    // 初始化障碍物轨迹预测服务
    std::string obstacle_trajectory_prediction_service_name;
    this->nh_.getParam("obstacle_trajectory_prediction_service", obstacle_trajectory_prediction_service_name);
    ros::service::waitForService(obstacle_trajectory_prediction_service_name);
    this->obstacle_trajectory_prediction_service_client_ = this->nh_.serviceClient<vec_map_cpp_msgs::GetPredictedTrajectory>(obstacle_trajectory_prediction_service_name);

    // 初始化到达目的地服务节点
    std::string destination_reached_service_name;
    this->nh_.getParam("destination_reached_service", destination_reached_service_name);
    ros::service::waitForService(destination_reached_service_name);
    this->destination_reached_service_client_ = this->nh_.serviceClient<std_srvs::Trigger>(destination_reached_service_name);

    // 初始化调用A星掉头服务节点
    std::string motion_planning_uncapable_service_name;
    this->nh_.getParam("motion_planning_uncapable_service", motion_planning_uncapable_service_name);
    this->motion_planning_uncapable_client_ = this->nh_.serviceClient<mission_msgs::RequireTurnAround>(motion_planning_uncapable_service_name, 10);

    // 初始化报告无法进行服务节点
    std::string motion_planning_failed_service_name;
    this->nh_.getParam("motion_planning_failed_service", motion_planning_failed_service_name);
    this->motion_planning_failed_client_ = this->nh_.serviceClient<std_srvs::Trigger>(motion_planning_failed_service_name, 10);

    // 初始化ros publish节点
    std::string motion_planning_curve_publish_topic;
    this->nh_.getParam("motion_planning_curve_publish_topic", motion_planning_curve_publish_topic);
    this->motion_planning_curve_pub_ = this->nh_.advertise<path_planning_msgs::MotionPlanningCurve>(motion_planning_curve_publish_topic, 10);

    this->history_curve_sub_ = this->nh_.subscribe(motion_planning_curve_publish_topic, 1, &DecisionMaking::SubVehicle::getHistoryCurve, this);

    // 初始化转向灯发布节点
    std::string turn_signal_publish_topic;
    this->nh_.getParam("turn_signal_publish_topic", turn_signal_publish_topic);
    this->turn_signal_pub_ = this->nh_.advertise<dbw_mkz_msgs::TurnSignalCmd>(turn_signal_publish_topic, 10);

    // 初始化紧急刹车发布节点
    std::string emergency_break_publish_topic;
    this->nh_.getParam("emergency_break_publish_topic", emergency_break_publish_topic);
    this->emergency_break_pub_ = this->nh_.advertise<std_msgs::Empty>(emergency_break_publish_topic, 10);

    if (this->TRAFFIC_LIGHT_USAGE_FLAG_) {
        // 初始化交通灯服务
        std::string traffic_light_service_name;
        this->nh_.getParam("traffic_light_service", traffic_light_service_name);
        ros::service::waitForService(traffic_light_service_name);
        this->traffic_light_service_client_ = this->nh_.serviceClient<traffic_light_msgs::traffic_lights>(traffic_light_service_name);
    }

    if (this->IS_SURROUND_RADAR_ENABLE_FLAG_) {
        // 初始化预警毫米波
        std::string surround_radar_topic;
        this->nh_.getParam("surround_radar_topic", surround_radar_topic);
        this->surround_radar_sub_ = this->nh_.subscribe(surround_radar_topic, 1, &DecisionMaking::SubVehicle::updateSurroundRadarInfo, this);
    }
}

// 启动ros订阅线程,50hz
void DecisionMaking::SubVehicle::listenRosMSGThread() {
    ros::Rate loop_rate(ROS_UPDATE_FREQUENCY);
    while (ros::ok()) {
        ros::spinOnce();
        this->self_test_.checkTest();
        loop_rate.sleep();
    }
}

// 更新车辆位置信息，ros节点
void DecisionMaking::SubVehicle::updateVehiclePosition(const nav_msgs::Odometry::ConstPtr odometry_msg) {
    // 获取当前位置信息(世界坐标系下)
    this->current_vehicle_world_position_mutex_.lock();
    this->current_vehicle_world_position_.position_.x_ = odometry_msg->pose.pose.position.x;
    this->current_vehicle_world_position_.position_.y_ = odometry_msg->pose.pose.position.y;
    tf::Quaternion quaternion;
    double raw, pitch, theta;
    tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, quaternion);
    tf::Matrix3x3(quaternion).getRPY(raw, pitch, theta);
    this->current_vehicle_world_position_.theta_ = theta;
    this->current_vehicle_world_position_mutex_.unlock();
    // 确定车辆位置信息加载成功
    this->vehicle_position_ready_flag_mutex_.lock();
    if (!this->VEHICLE_POSITION_READY_FLAG_) {
        this->VEHICLE_POSITION_READY_FLAG_ = true;
        ROS_INFO("VEHICLE POSITION GOT");
        LOG(INFO) << "VEHICLE POSITION GOT";
    }
    this->vehicle_position_ready_flag_mutex_.unlock();

    // // 判断当前位置是否处于园区内
    // GLOBAL_IN_GROUND_MUTEX_.lock();
    // if (Tools::isRectangleOverlap(current_pose, GLOBAL_GROUND_AREA, 1.0, 1.0)) {
    //     // 在园区内
    //     GLOBAL_IS_IN_GROUND_ = true;
    //     // std::cout << "在园区内" << std::endl;
    // } else {
    //     GLOBAL_IS_IN_GROUND_ = false;
    //     // std::cout << "不在园区内" << std::endl;
    // }
    // GLOBAL_IN_GROUND_MUTEX_.unlock();
    // // 判断当前位置是否处于机非混行内
    // GLOBAL_IN_JIFEI_MUTEX_.lock();
    // if (Tools::isRectangleOverlap(current_pose, GLOBAL_JIFEI_AREA, 1.0, 1.0)) {
    //     // 在园区内
    //     GLOBAL_IS_IN_JIFEI_ = true;
    //     // std::cout << "在机非混行内" << std::endl;
    // } else {
    //     GLOBAL_IS_IN_JIFEI_ = false;
    //     // std::cout << "不在机非混行内" << std::endl;
    // }
    // GLOBAL_IN_JIFEI_MUTEX_.unlock();

    // 判断是否可以可视化车辆当前位置
    this->vehicle_curvature_ready_flag_mutex_.lock();
    this->vehicle_position_ready_flag_mutex_.lock();
    double pose_defined = this->VEHICLE_CURVATURE_READY_FLAG_ && this->VEHICLE_POSITION_READY_FLAG_;
    this->vehicle_position_ready_flag_mutex_.unlock();
    this->vehicle_curvature_ready_flag_mutex_.unlock();
    if (!pose_defined) {
        return;
    }
    // 车辆位置进行可视化
    this->current_vehicle_world_position_mutex_.lock();
    // 1.清除之前的位置
    visualization_msgs::MarkerArray delete_marker_array;
    delete_marker_array.markers.push_back(VisualizationMethods::visualizedeleteMarker(VisualizationMethods::VisualizationID::VEHICLE_ODOM));
    this->vis_vehicle_pub_.publish(delete_marker_array);
    // 2.加载新的位置信息
    visualization_msgs::MarkerArray marker_array;
    std_msgs::ColorRGBA color;
    color.r = 0;
    color.g = 1;
    color.b = 0;
    color.a = 1;
    marker_array.markers.push_back(VisualizationMethods::visualizeRectToMarker(this->current_vehicle_world_position_.position_.x_, this->current_vehicle_world_position_.position_.y_, Tools::centerYawToRearYaw(this->current_vehicle_world_position_.theta_, this->current_vehicle_world_position_.kappa_, DISTANCE_FROM_REAR_TO_CENTER), this->vehicle_width_, this->vehicle_length_, this->vehicle_rear_axis_center_scale_, color, VisualizationMethods::VisualizationID::VEHICLE_ODOM));
    
    // Get velocity information and print
    std_msgs::ColorRGBA str_color;
    str_color.r = 0;
    str_color.g = 0;
    str_color.b = 0;
    str_color.a = 1;
    PathPlanningUtilities::VehicleMovementState start_point_movement;
    this->current_vehicle_movement_mutex_.lock();
    start_point_movement = this->current_vehicle_movement_;
    this->current_vehicle_movement_mutex_.unlock();
    std::string velocity_str = "velocity: " + std::to_string(start_point_movement.velocity_);
    marker_array.markers.push_back(VisualizationMethods::visualizeStringToMarker(velocity_str, current_vehicle_world_position_.position_.x_ + 3.0, current_vehicle_world_position_.position_.y_ + 3.0, str_color, 9000000));
    
    this->vis_vehicle_pub_.publish(marker_array);
    this->current_vehicle_world_position_mutex_.unlock();
}

// 更新车辆速度和速度朝向，ros节点
void DecisionMaking::SubVehicle::updateVehicleMovement(const std_msgs::Float64::ConstPtr velocity_msg) {
    // 更新车辆速度信息
    this->current_vehicle_movement_mutex_.lock();
    // Add compensation to simulate specific situation
    this->current_vehicle_movement_.velocity_ = velocity_msg->data;
    this->current_vehicle_movement_mutex_.unlock();
    // 确定车辆运动信息加载成功
    this->vehicle_movement_ready_flag_mutex_.lock();
    if (!this->VEHICLE_MOVEMENT_READY_FLAG_) {
        this->VEHICLE_MOVEMENT_READY_FLAG_ = true;
        ROS_INFO("VEHICLE MOVEMENT GOT");
        LOG(INFO) << "VEHICLE MOVEMENT GOT";
    }
    this->vehicle_movement_ready_flag_mutex_.unlock();
    // 特殊处理
    if (Tools::isLarge(this->current_vehicle_movement_.velocity_, 0.0)) {
        this->stop_count_recorder_ = 0;
        this->low_frequency_stop_count_recorder_ = 0;
    }
    
}

// 更新车辆曲率
void DecisionMaking::SubVehicle::updateVehicleCurvature(const std_msgs::Float64::ConstPtr curvature_msg) {
    // 记录车辆当前曲率
    this->current_vehicle_world_position_mutex_.lock();
    this->current_vehicle_world_position_.kappa_ = curvature_msg->data;
    this->current_vehicle_world_position_mutex_.unlock();
    this->current_vehicle_kappa_mutex_.lock();
    this->current_vehicle_kappa_ = curvature_msg->data;
    this->current_vehicle_kappa_mutex_.unlock();
    // 确定车辆曲率信息加载成功
    this->vehicle_curvature_ready_flag_mutex_.lock();
    if (!this->VEHICLE_CURVATURE_READY_FLAG_) {
        this->VEHICLE_CURVATURE_READY_FLAG_ = true;
        ROS_INFO("VEHICLE CURVATURE GOT");
        LOG(INFO) << "VEHICLE CURVATURE GOT";
    }
    this->vehicle_curvature_ready_flag_mutex_.unlock();
}

// Update vehicle steer information
void DecisionMaking::SubVehicle::updateVehicleSteer(const std_msgs::Float64::ConstPtr steer_msg) {
    current_vehicle_steer_metex_.lock();
    current_vehicle_steer_ = steer_msg->data;
    current_vehicle_steer_metex_.unlock();
}

void DecisionMaking::SubVehicle::updateVehicleAcceleration(const std_msgs::Float64::ConstPtr acceleration_msg) {
    // Update acceleration information
    this->current_vehicle_movement_mutex_.lock();
    this->current_vehicle_movement_.acceleration_ = acceleration_msg->data;
    this->current_vehicle_movement_mutex_.unlock();
}

// 更新控制报告信息
void DecisionMaking::SubVehicle::updateControlReport(const control_msgs::CoreReport::ConstPtr control_report_msg) {
    if (control_report_msg->status == control_msgs::CoreReport::GOAL_REACHED) {
        // 控制报告完成
        this->control_finished_flag_mutex_.lock();
        this->CONTROL_FINISHED_FLAG_ = true;
        this->control_finished_flag_mutex_.unlock();
    }
}

// 更新毫米波雷达信息，ros节点
void DecisionMaking::SubVehicle::updateSurroundRadarInfo(const dbw_mkz_msgs::SurroundReport::ConstPtr radar_msgs) {
    this->right_alert_mutex_.lock();
    this->right_alert_ = radar_msgs->blis_right_alert;
    this->right_alert_mutex_.unlock();
    this->left_alert_mutex_.lock();
    this->left_alert_ = radar_msgs->blis_left_alert;
    this->left_alert_mutex_.unlock();
    this->vehicle_surround_radar_ready_flag_mutex_.lock();
    if (!this->VEHICLE_SURROUND_RADAR_READY_FLAG_) {
        this->VEHICLE_SURROUND_RADAR_READY_FLAG_ = true;
    }
    this->vehicle_surround_radar_ready_flag_mutex_.unlock();
}

// 开始任务节点
bool DecisionMaking::SubVehicle::startMission(mission_msgs::StartMainRequest &request ,mission_msgs::StartMainResponse &response) {
    this->mission_start_mutex_.lock();
    this->MISSION_START_FLAG_ = true;
    ROS_INFO("MISSION START");
    LOG(INFO) << "MISSION START";
    this->mission_start_mutex_.unlock();
    // 获取终点的位置
    this->destination_mutex_.lock();
    this->destination_pose_ = request.goal_pose;
    LOG(INFO) << "收到的目标点为" << std::setprecision(14) << this->destination_pose_.pose.position.x << "||" << std::setprecision(14) << this->destination_pose_.pose.position.y << "||" << std::setprecision(14) << std::atan2(this->destination_pose_.pose.orientation.z, this->destination_pose_.pose.orientation.w) * 2.0; 
    this->destination_mutex_.unlock();
    // // 更新紧急停车标志
    // this->emergency_break_flag_mutex_.lock();
    // // this->IS_EMERGENCY_BREAK_FLAG_ = false;
    // this->emergency_break_flag_mutex_.unlock();
    return true;
}


// 障碍物信息callback函数，ros节点
void DecisionMaking::SubVehicle::getObstacles(const ibeo_lidar_msgs::object_filter_data::ConstPtr &msg) {
    this->perception_object_mutex_.lock();
    // 更新障碍物信息
    this->perception_objects_ = msg->objects;
    // 障碍物可视化(TOFIX)
    this->perception_object_mutex_.unlock();
}

// 历史路径callback函数,ros节点
void DecisionMaking::SubVehicle::getHistoryCurve(const path_planning_msgs::MotionPlanningCurve &msg) {
    this->history_curve_mutex_.lock();
    PathPlanningUtilities::Curve().swap(this->history_curve_);
    for (auto point: msg.points) {
        PathPlanningUtilities::CurvePoint curve_point;
        curve_point.position_.x_ = point.x;
        curve_point.position_.y_ = point.y;
        curve_point.theta_ = point.theta;
        curve_point.kappa_ = point.kappa;
        this->history_curve_.push_back(curve_point);
    }
    this->history_curve_mutex_.unlock();
}

// Replanning thread
void DecisionMaking::SubVehicle::triggerThread() {
    // Frequency 50hz
    ros::Rate loop_rate(50);
    // Prepare 
    Utils::Trigger* trigger = new Utils::Trigger();

    while (ros::ok()) {
        if (executed_trajectory_.empty()) {
            // printf("[Trigger] no executed trajectory, waiting.\n");
            replanning_from_previous_trajectory_state_ = false;
            need_replanning_ = true;
            continue;
        }

        // Transform ego vehicle information and surround vehice information
        // Update vehicle information
        PathPlanningUtilities::VehicleState start_point_in_world;
        this->current_vehicle_world_position_mutex_.lock();
        start_point_in_world = this->current_vehicle_world_position_;
        this->current_vehicle_world_position_mutex_.unlock();
        PathPlanningUtilities::VehicleMovementState start_point_movement;
        this->current_vehicle_movement_mutex_.lock();
        start_point_movement = this->current_vehicle_movement_;
        this->current_vehicle_movement_mutex_.unlock();
        this->current_vehicle_kappa_mutex_.lock();
        double start_point_kappa = this->current_vehicle_kappa_;
        start_point_in_world.kappa_ = start_point_kappa;
        this->current_vehicle_kappa_mutex_.unlock();
        current_vehicle_steer_metex_.lock();
        double current_vehicle_steer = current_vehicle_steer_;
        current_vehicle_steer_metex_.unlock();

        trigger->load(trajectory_update_time_stamp_, executed_trajectory_, executed_traj_thetas_, executed_traj_curvatures_, executed_traj_velocities_, executed_traj_accelerations_, start_point_in_world, start_point_movement);
        bool replanning_from_previous_trajectory = trigger->runOnce(&prev_traj_corres_veh_state_, &prev_traj_corres_veh_movement_state_);
        bool need_replanning = trigger->needReplanning();
        replanning_from_previous_trajectory_state_ = replanning_from_previous_trajectory;
        need_replanning_ = need_replanning;
    }
}


// 规划和决策线程,20hz
void DecisionMaking::SubVehicle::motionPlanningThread() {
    ros::Rate loop_rate(MOTION_PLANNING_FREQUENCY);

    // // DEBUG
    // // Decrease frequency to record data
    // ros::Rate loop_rate(2);
    // // END DEBUG

    // 进程等待直到数据准备完成
    while (ros::ok()) {
        this->vehicle_surround_radar_ready_flag_mutex_.lock();
        bool surround_radar_ready_flag = !this->IS_SURROUND_RADAR_ENABLE_FLAG_ || 
        this->VEHICLE_SURROUND_RADAR_READY_FLAG_;
        vehicle_surround_radar_ready_flag_mutex_.unlock();
        this->vehicle_position_ready_flag_mutex_.lock();
        this->vehicle_movement_ready_flag_mutex_.lock();
        this->vehicle_curvature_ready_flag_mutex_.lock();
        bool data_read_flag = this->VEHICLE_POSITION_READY_FLAG_ && this->VEHICLE_MOVEMENT_READY_FLAG_ && this->VEHICLE_CURVATURE_READY_FLAG_;
        this->vehicle_curvature_ready_flag_mutex_.unlock();
        this->vehicle_movement_ready_flag_mutex_.unlock();
        this->vehicle_position_ready_flag_mutex_.unlock();
        if (data_read_flag && surround_radar_ready_flag) {
            break;
        }
        loop_rate.sleep();
    }
    std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++ data prepare finished +++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    LOG(INFO) << "++++++++++++++++++++++++++++++++++++++++++++++++++ data prepare finished +++++++++++++++++++++++++++++++++++++++++++++++++++++";
    // 主程序入口
    while (ros::ok()) {
        // 判断任务是否开始
        this->mission_start_mutex_.lock();
        bool is_mission_start = this->MISSION_START_FLAG_;
        this->mission_start_mutex_.unlock();
        if (!is_mission_start) {
            loop_rate.sleep();
            continue;
        }

        // Run HPDM
        bool is_hpdm_planning_success = false;
        double time_cons = 0.0;
        hpdmPlanning(&is_hpdm_planning_success, &time_cons);
        if (!is_hpdm_planning_success) {
            printf("[MainPineline] hpdm planning failed.\n");
            continue;
        }

        // Run trajectory planning
        bool is_trajectory_planning_success = false;
        sscPlanning(&is_trajectory_planning_success);
        if (!is_trajectory_planning_success) {
            printf("[MainPineline] ssc planning failed.\n");
            continue;
        }

        // Check trajectory
        std::vector<double> thetas, curvatures, velocities, accelerations;
        trajectoryCheck(&thetas, &curvatures, &velocities, &accelerations);

        // // DEBUG
        // for (int i = 0; i < static_cast<int>(thetas.size()); i++) {
        //     printf("Index: %d, theta: %lf, curvature: %lf, velocity: %lf, acceleration: %lf.\n", i, thetas[i], curvatures[i], velocities[i], accelerations[i]);
        // }
        // // END DEBUG


        // Publish trajectory
        if (need_replanning_) {
            trajectoryPublish(thetas, curvatures, velocities, accelerations, motion_planning_curve_pub_);
            // Visualization executed trajectory
            // VisualizationMethods::visualizeTrajectory(executed_trajectory_, vis_trajectory_planner_pub_, true);
            printf("[MainPineline] execute replanning.\n");

            // Calculate the minimum distance to obstacles
            double min_distance_to_obstacles = MAX_VALUE;
            int predicted_states_num = static_cast<int>(ego_trajectory_.size());
            for (int i = 0; i < predicted_states_num; i++) {
                for (const auto& sur_veh_info : surround_trajectories_) {
                    double cur_veh_dis = (ego_trajectory_[i].state_.position_ - sur_veh_info.second[i].state_.position_).norm();
                    min_distance_to_obstacles = std::min(min_distance_to_obstacles, cur_veh_dis);
                }
            }

            // Store trajectory information
            std::string root_path = ros::package::getPath("motion_planning");
            std::string log_file_path = "/trajectory_record/" + Tools::returnCurrentTimeAndDate() + ".csv";
            log_file_path = root_path + log_file_path;
            std::ofstream file(log_file_path);
            if (file) {
                file << std::setprecision(14) << min_distance_to_obstacles << "," << time_cons << "\n";
                for (int i = 0; i < static_cast<int>(executed_trajectory_.size()); i++) {
                    file << std::setprecision(14) << executed_trajectory_[i].x_ << "," << executed_trajectory_[i].y_ << "," << executed_traj_thetas_[i] << "," << executed_traj_curvatures_[i] << "," << executed_traj_velocities_[i] << "," << executed_traj_accelerations_[i] << "\n";
                }
            }
            file.close();
        }


        // // DEBUG 
        // // Test control deviance
        // sleep(1);
        // // END DEBUG

        loop_rate.sleep();

    }
}


