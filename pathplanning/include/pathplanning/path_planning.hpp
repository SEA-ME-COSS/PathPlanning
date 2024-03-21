#pragma once

#include "msg_struct.h"

#include <rclcpp/rclcpp.hpp>

#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/classification2_d.hpp>
#include <std_msgs/msg/string.hpp>

#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>
#include <cmath>

class PathPlanning : public rclcpp::Node {
public:
    PathPlanning();

private:
    std::vector<Sign> signs;
    std::vector<Light> lights;
    std::vector<Object> objects;
    std::vector<StopLine> stopline;

    vision_msgs::msg::Classification2D::SharedPtr sign_msg;
    vision_msgs::msg::Detection2D::SharedPtr object_msg;
    vision_msgs::msg::Classification2D::SharedPtr stopline_msg;
    std_msgs::msg::String::SharedPtr light_msg;

    bool use_sign;
    bool use_object;
    bool use_stopline;
    bool use_light;

    rclcpp::Subscription<vision_msgs::msg::Classification2D>::SharedPtr sign_subscription_;
    rclcpp::Subscription<vision_msgs::msg::Detection2D>::SharedPtr object_subscription_;
    rclcpp::Subscription<vision_msgs::msg::Classification2D>::SharedPtr stopline_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr light_subscription_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr throttle_publisher_;
    rclcpp::TimerBase::SharedPtr publisher_timer_;

    void sign_callback(const vision_msgs::msg::Classification2D::SharedPtr sign_msg);
    void object_callback(const vision_msgs::msg::Detection2D::SharedPtr object_msg);
    void stopline_callback(const vision_msgs::msg::Classification2D::SharedPtr stopline_msg);
    void light_callback(const std_msgs::msg::String::SharedPtr light_msg);

    void publisher_timer_callback();

    bool isUseMessageValid();
    void updateUseMessages();
    void publish_path();
    void publish_throttle();

    void update_sign();
    void update_object();
    void update_stopline();
    void update_light();

    // void update_pose();
    // float quat_to_yaw(const geometry_msgs::msg::Quaternion quat_msg);
    // void addPose(nav_msgs::msg::Path& path_msg, std::vector<double> pose);
};