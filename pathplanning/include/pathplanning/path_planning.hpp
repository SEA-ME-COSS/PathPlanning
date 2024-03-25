#pragma once

#include "utils/loader.hpp"
#include "utils/map.hpp"
#include "planner/planner.hpp"
#include "planner/a_star.hpp"
#include "planner/bfs.hpp"
#include "decision_making/decision_making.hpp"
#include "msg_struct.h"

#include "rclcpp/rclcpp.hpp"

#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/classification2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
// #include "std_msgs/msg/int8.hpp"
// #include "std_msgs/msg/float64.hpp"
#include "example_interfaces/msg/float64.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>
#include <cmath>
#include <vector>
#include <deque>
#include <utility>
#include <memory>

class PathPlanning : public rclcpp::Node {
public:
    PathPlanning();

private:
    Map map;
    std::unique_ptr<Planner> planner;
    DecisionMaking decision_making;

    std::vector<Sign> signs;
    std::vector<Light> lights;
    std::vector<Object> objects;
    std::vector<StopLine> stopline;
    Pose pose;

    vision_msgs::msg::Classification2D::SharedPtr sign_msg;
    // vision_msgs::msg::Classification2D::SharedPtr light_msg;
    // vision_msgs::msg::Detection2D::SharedPtr object_msg;
    geometry_msgs::msg::PoseStamped::SharedPtr pose_msg;
    // vision_msgs::msg::Classification2D::SharedPtr stopline_msg;

    bool use_sign;
    // bool use_light;
    // bool use_object;
    bool use_pose;
    // bool use_stopline;

    std::vector<std::vector<double>> path;
    int throttle;
    int state;
    bool normal_throttle;

    double factor_x;
    double factor_y;

    rclcpp::Subscription<vision_msgs::msg::Classification2D>::SharedPtr sign_subscription_;
    // rclcpp::Subscription<vision_msgs::msg::Classification2D>::SharedPtr light_subscription_;
    // rclcpp::Subscription<vision_msgs::msg::Detection2D>::SharedPtr object_subscription_;
    // rclcpp::Subscription<vision_msgs::msg::Classification2D>::SharedPtr stopline_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr throttle_publisher_;
    
    rclcpp::TimerBase::SharedPtr publisher_timer_;

    void sign_callback(const vision_msgs::msg::Classification2D::SharedPtr sign_msg);
    // void light_callback(const vision_msgs::msg::Classification2D::SharedPtr light_msg);
    // void object_callback(const vision_msgs::msg::Detection2D::SharedPtr object_msg);
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg);
    // void stopline_callback(const vision_msgs::msg::Classification2D::SharedPtr stopline_msg);

    void publisher_timer_callback();

    bool isUseMessageValid();
    void updateUseMessages();
    void publish_path();
    void publish_throttle();
    void publish_state();

    void update_sign();
    // void update_light();
    // void update_object();
    void update_pose();
    // void update_stopline();

    float quat_to_yaw(const geometry_msgs::msg::Quaternion quat_msg);
    void addPose(nav_msgs::msg::Path& path_msg, std::vector<double> pose);
};