#pragma once

#include "utils/ros2_msg_struct.h"
#include "utils/decision_making_struct.h"

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

    float crosswalksign_ignore;
    float sign_mindistance;
    float status_break_time;

    void StatusDecision();

    void DefaultState();

    void CrosswalkBeforeState();
    void CrosswalkNowState();

    void PriorityState();
    void RacingState();

    bool isSignWorthy(const std::string state, bool& sign_check, std::chrono::steady_clock::time_point& sign_miss_time);
};
