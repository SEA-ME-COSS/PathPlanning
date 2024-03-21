#include "path_planning.hpp"

PathPlanning::PathPlanning() : rclcpp::Node("path_planning") {
    // Load Waypoints Info
    std::string nodes_file_path = "../globalmap/parsinginfo.txt";
    std::string waypoints_file_path = "../globalmap/waypoints.txt";
    std::vector<std::vector<double>> waypoints = load_waypoints(nodes_file_path, waypoints_file_path);

    // Make Map Instruction
    std::string map_file_path = "../globalmap/flipped-track.txt";
    Map map = Map(map_file_path);

    // Path Planner
    this->planner = A_Star(map);
    // this->planner = BFS(map);
    planner.plan_with_waypoints(waypoints);
    this->path = planner.get_waypoints_path();

    // Decision Making
    VehicleState current_state = VehicleState::Driving;
    this->normal_throttle = 25.0;

    decision_making = DecisionMaking(current_state, normal_throttle,
                                &(this->stopline), &(this->signs), &(this->lights), &(this->objects), &(this->pose));

    // Activate ROS2 Message
    this->use_sign = false;
    this->use_object = false;
    this->use_stopline = false;
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
    object_subscription_ = this->create_subscription<vision_msgs::msg::Detection2D>(
        "/perception/object", 10, std::bind(&PathPlanning::object_callback, this,  std::placeholders::_1));
    stopline_subscription_ = this->create_subscription<vision_msgs::msg::Classification2D>(
        "/perception/stopline", 10, std::bind(&PathPlanning::stopline_callback, this,  std::placeholders::_1));
    light_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/perception/light", 10, std::bind(&PathPlanning::light_callback, this,  std::placeholders::_1));
    pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/localization/pose", 10, std::bind(&PathPlanning::pose_callback, this,  std::placeholders::_1));

    // ROS Publisher
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
        "/planner/path", 10);
    throttle_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
        "/planner/throttle", 10);

    publisher_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&PathPlanning::publisher_timer_callback, this)
    );

    std::cout << "Initialized OK" << std::endl;
}

void PathPlanning::sign_callback(const vision_msgs::msg::Classification2D::SharedPtr sign_msg) {
    this->sign_msg = sign_msg;
}

void PathPlanning::object_callback(const vision_msgs::msg::Detection2D::SharedPtr object_msg) {
    this->object_msg = object_msg;
}

void PathPlanning::stopline_callback(const vision_msgs::msg::Classification2D::SharedPtr stopline_msg) {
    this->stopline_msg = stopline_msg;
}

void PathPlanning::light_callback(const std_msgs::msg::String::SharedPtr light_msg) {
    this->light_msg = light_msg;
}

void PathPlanning::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) {
    this->pose_msg = pose_msg;
}

void PathPlanning::publisher_timer_callback() {
   // Use Message Validation
    if (!this->isUseMessageValid()) {return;}

    // Update Using Messages to Decision Making
    this->updateUseMessages();
    // Decision Making with Using Messages
    this->decision_making.decide();

    this->throttle = static_cast<int>(this->velocity_factor * this->decision_making.getThrottle());
    
    this->publish_path();
    this->publish_throttle();
}

bool PathPlanning::isUseMessageValid() {
    if (this->use_sign) {if (!this->sign_msg) {return false;}}
    if (this->use_light) {if (!this->light_msg) {return false;}}
    if (this->use_object) {if (!this->object_msg) {return false;}}
    if (this->use_stopline) {if (!this->stopline_msg) {return false;}}
    return true;
}

void PathPlanning::updateUseMessages() {
    if (this->use_sign) {this->update_sign();}
    if (this->use_object) {this->update_object();}
    if (this->use_stopline) {this->update_stopline();}
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
    std_msgs::msg::Float64 throttle_msg;
    throttle_msg.data = this->throttle;
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

void PathPlanning::update_object() {
    this->objects.clear();
    Object object;

    for (size_t i = 0; i < object_msg->results.size(); ++i) {
        // CHECK 3 (Is information enough)
        object.id = object_msg->results[i].id;
        object.x = object_msg->results[i].pose.pose.position.x;
        object.y = object_msg->results[i].pose.pose.position.y;
        object.isDynamic = static_cast<bool>(static_cast<int>(object_msg->results[i].score + 0.5));
        this->objects.push_back(object);
    }
}

void PathPlanning::update_stopline() {
    this->stopline.clear();
    StopLine stopline;

    if(!stopline_msg->results.empty()) {
        stopline.distance = stopline_msg->results[0].score;
        this->stopline.push_back(stopline);
    }
}

void PathPlanning::update_pose(){
    this->pose.x = pose_msg->pose.position.x;
    this->pose.y = pose_msg->pose.position.y;
    // this->pose.heading = this->quat_to_yaw(pose_msg->pose.orientation);
}

float PathPlanning::quat_to_yaw(const geometry_msgs::msg::Quaternion quat) {
    tf2::Quaternion tf2_quat;
    tf2::fromMsg(quat, tf2_quat);
    double roll;
    double pitch;
    double yaw;
    tf2::Matrix3x3 matrix(tf2_quat);
    matrix.getRPY(roll, pitch, yaw);
    return roll;
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
