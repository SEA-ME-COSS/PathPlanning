#pragma once

#include "utils/loader.hpp"
#include "utils/map.hpp"
#include "planner/planner.hpp"
#include "planner/bfs.hpp"
#include "decision_making/decision_making.hpp"
#include "msg_struct.h"

#include "rclcpp/rclcpp.hpp"

#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/classification2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "example_interfaces/msg/float64.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <string>
#include <cmath>
#include <vector>
#include <deque>
#include <utility>
#include <memory>

#include <std_msgs/msg/string.hpp>

class PathPlanning : public rclcpp::Node {
public:
    PathPlanning();

private:
    Map map;
    std::unique_ptr<Planner> planner;
    DecisionMaking decision_making;

    std::vector<Sign> signs;
    std::vector<Light> lights;
    Pose pose;

    vision_msgs::msg::Classification2D::SharedPtr sign_msg;
    std_msgs::msg::String::SharedPtr light_msg;
    nav_msgs::msg::Odometry::SharedPtr pose_msg;

    bool use_sign;
    bool use_light;
    bool use_pose;

    std::vector<std::vector<double>> path;
    float throttle;
    int state;
    float normal_throttle;

    double factor_x;
    double factor_y;

    rclcpp::Subscription<vision_msgs::msg::Classification2D>::SharedPtr sign_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr light_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscription_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr throttle_publisher_;
    
    rclcpp::TimerBase::SharedPtr publisher_timer_;

    void sign_callback(const vision_msgs::msg::Classification2D::SharedPtr sign_msg);
    void light_callback(const std_msgs::msg::String::SharedPtr light_msg);
    void pose_callback(const nav_msgs::msg::Odometry::SharedPtr pose_msg);

    void publisher_timer_callback();

    bool isUseMessageValid();
    void updateUseMessages();
    void publish_path();
    void publish_throttle();
    void publish_state();

    void update_sign();
    void update_light();
    void update_pose();
    
    void addPose(nav_msgs::msg::Path& path_msg, std::vector<double> pose);
};