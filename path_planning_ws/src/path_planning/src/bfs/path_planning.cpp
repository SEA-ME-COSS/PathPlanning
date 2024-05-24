#include "path_planning.hpp"

PathPlanning::PathPlanning() : rclcpp::Node("bfs") {
    // Load Waypoints Info
    std::string nodes_file_path = "src/path_planning/include/path_planning/bfs/globalmap/parsinginfo.txt";
    std::string waypoints_file_path = "src/path_planning/include/path_planning/bfs/globalmap/waypoints.txt";
    std::vector<std::vector<double>> waypoints = load_waypoints(nodes_file_path, waypoints_file_path);

    // Make Map Instruction
    std::string map_file_path = "src/path_planning/include/path_planning/bfs/globalmap/flipped-track.txt";
    Map map = Map(map_file_path);
    // Path Planner
    this->planner = std::make_unique<BFS>(map);
    planner->plan_with_waypoints(waypoints);
    this->path = planner->get_waypoints_path();

    // Decision Making
    VehicleState current_state = VehicleState::Driving;
    this->normal_throttle = 1.0;

    decision_making = DecisionMaking(current_state, this->normal_throttle,
                                &(this->signs), &(this->lights), &(this->pose));

    // Activate ROS2 Message
    this->use_sign = false;
    this->use_light = false;
    this->use_pose = false;

     // Initialization
    this->throttle = 0;
    this->state = 0;
    this->factor_x = 0.768;
    this->factor_y = 0.776;

    // ROS Subscription
    sign_subscription_ = this->create_subscription<vision_msgs::msg::Classification2D>(
        "/perception/sign", 10, std::bind(&PathPlanning::sign_callback, this,  std::placeholders::_1));
    light_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/perception/light", 10, std::bind(&PathPlanning::light_callback, this,  std::placeholders::_1));
    pose_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/piracer/odom", 10, std::bind(&PathPlanning::pose_callback, this,  std::placeholders::_1));

    // ROS Publisher
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
        "/pathplanner/path", 10);
    throttle_publisher_ = this->create_publisher<example_interfaces::msg::Float64>(
        "/pathplanner/throttle", 10);

    publisher_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&PathPlanning::publisher_timer_callback, this)
    );

    std::cout << "Initialized OK" << std::endl;
}

void PathPlanning::sign_callback(const vision_msgs::msg::Classification2D::SharedPtr sign_msg) {
    this->sign_msg = sign_msg;
}

void PathPlanning::light_callback(const std_msgs::msg::String::SharedPtr light_msg) {
    this->light_msg = light_msg;
}

void PathPlanning::pose_callback(const nav_msgs::msg::Odometry::SharedPtr pose_msg) {
    this->pose_msg = pose_msg;
}

void PathPlanning::publisher_timer_callback() {
   // Use Message Validation
    if (!this->isUseMessageValid()) {return;}

    // Update Using Messages to Decision Making
    this->updateUseMessages();
    // Decision Making with Using Messages
    this->decision_making.decide();

    this->throttle = static_cast<int>(this->decision_making.getThrottle());
    
    this->publish_path();
    this->publish_throttle();
}

bool PathPlanning::isUseMessageValid() {
    if (this->use_sign) {if (!this->sign_msg) {return false;}}
    if (this->use_light) {if (!this->light_msg) {return false;}}
    if (this->use_pose) {if (!this->pose_msg) {return false;}}
    return true;
}

void PathPlanning::updateUseMessages() {
    if (this->use_sign) {this->update_sign();}
    if (this->use_light) {this->update_light();}
    if (this->use_pose) {this->update_pose();}
}

void PathPlanning::publish_path() {
    nav_msgs::msg::Path path_msg;
    path_msg.header = std_msgs::msg::Header();
    path_msg.header.stamp = rclcpp::Clock().now();
    path_msg.header.frame_id = "map";
    for (size_t i = 0; i < this->path.size(); ++i) {
        this->addPose(path_msg, this->path[i]);
    }
    this->path_publisher_->publish(path_msg);
}

void PathPlanning::publish_throttle() {
    example_interfaces::msg::Float64 throttle_msg;
    throttle_msg.data = this->throttle;
    // std::cout << "Throttle : " << throttle_msg.data << std::endl;
    this->throttle_publisher_->publish(throttle_msg);
}

void PathPlanning::update_sign() {
    this->signs.clear();
    Sign sign;

    for (size_t i = 0; i < sign_msg->results.size(); ++i) {
        // CHECK 1 (Distance)
        sign.id = sign_msg->results[i].id;
        sign.distance = sign_msg->results[i].score;
        this->signs.push_back(sign);
    }
}

void PathPlanning::update_light() {
    this->lights.clear();
    Light light;

    if(!light_msg->data.empty()) {
        light.id = light_msg->data;
        this->lights.push_back(light);
    }
}

void PathPlanning::update_pose() {
    this->pose.x = pose_msg->pose.pose.position.x * 100;  // [cm]
    this->pose.y = pose_msg->pose.pose.position.y * 100;  // [cm]
    
    tf2::Quaternion q(
      pose_msg->pose.pose.orientation.x,
      pose_msg->pose.pose.orientation.y,
      pose_msg->pose.pose.orientation.z,
      pose_msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);  // [rad]
    this->pose.yaw = yaw;

    this->pose.v = sqrt(pow(pose_msg->twist.twist.linear.x, 2)
                        + pow(pose_msg->twist.twist.linear.y, 2)) * 100;  // [cm/s]

    std::cout << "X pose : " << this->pose.x << " Y pose : " << this->pose.y << " Yaw : " << this->pose.yaw  << " Speed : " << this->pose.v << std::endl;
}

void PathPlanning::addPose(nav_msgs::msg::Path& path_msg, std::vector<double> pose) {
    geometry_msgs::msg::PoseStamped pose_stamped;

    pose_stamped.pose.position.x = pose[0]*this->factor_x;
    pose_stamped.pose.position.y = pose[1]*this->factor_y;
    
    tf2::Quaternion quat;
    quat.setRPY(0, 0, pose[2]);
    pose_stamped.pose.orientation.x = quat.x();
    pose_stamped.pose.orientation.y = quat.y();
    pose_stamped.pose.orientation.z = quat.z();
    pose_stamped.pose.orientation.w = quat.w();

    pose_stamped.header.frame_id = "map";
    pose_stamped.header.stamp = rclcpp::Clock().now();

    path_msg.poses.push_back(pose_stamped);
}
