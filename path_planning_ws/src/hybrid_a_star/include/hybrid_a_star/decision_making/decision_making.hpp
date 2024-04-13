#pragma once

#include "data_structure/ros2_msg_struct.h"
#include "data_structure/decision_making_struct.h"

#include <vector>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <cmath>

class DecisionMaking {
public:
    DecisionMaking(VehicleState current_state, float normal_throttle,
                   std::vector<Sign> *signs, std::vector<Light> *lights, Pose *pose);
    DecisionMaking();
    ~DecisionMaking();

    void decide();
    float getThrottle();
    int getState();

private:
    std::vector<Sign> *signs;
    std::vector<Light> *lights;
    Pose *pose;

    float throttle;

    VehicleState current_state;
    float normal_throttle;

    bool crosswalknow_timecheck;
    std::chrono::steady_clock::time_point crosswalknow_stoptime;

    bool crosswalkworthy_timecheck;
    std::chrono::steady_clock::time_point crosswalkworthy_time;

    bool rotarynow_timecheck;
    std::chrono::steady_clock::time_point rotarynow_stoptime;

    bool rotaryworthy_timecheck;
    std::chrono::steady_clock::time_point rotaryworthy_time;

    float crosswalksign_ignore;
    float rotarysign_ignore;
    float sign_mindistance;
    float status_break_time;

    bool trafficlight_status;

    void TrafficLightStatusDecision();
    void SignStatusDecision();

    void DefaultState();
    void TrafficLightRedBeforeState();
    void TrafficLightRedNowState();
    void TrafficLightYellowBeforeState();
    void TrafficLightYellowNowState();
    void TrafficLightGreenBeforeState();
    void TrafficLightGreenNowState();

    void CrosswalkBeforeState();
    void CrosswalkNowState();
    void RoundAboutBeforeState();
    void RoundAboutNowState();

    void RacingState();

    bool isSignWorthy(const std::string state, bool& sign_check, std::chrono::steady_clock::time_point& sign_miss_time);
};
