#pragma once

#include "msg_struct.h"

#include <vector>
#include <iostream>
#include <algorithm>
#include <chrono>

class DecisionMaking {
public:
    DecisionMaking(VehicleState current_state, float normal_throttle,
                    std::vector<StopLine> *stopline, std::vector<Sign> *signs, std::vector<Light> *lights, std::vector<Object> *objects, Pose *pose);
    DecisionMaking();
    ~DecisionMaking();

    void decide();
    float getThrottle();

private:
    std::vector<Sign> *signs;
    std::vector<Light> *lights;
    std::vector<Object> *objects;
    Pose *pose;
    std::vector<StopLine> *stopline;

    float throttle;

    VehicleState current_state;
    float normal_throttle;

    bool stopnow_timecheck;
    std::chrono::steady_clock::time_point stopnow_stoptime;

    bool stopafter_timecheck;
    std::chrono::steady_clock::time_point stopafter_stoptime;

    bool crosswalknow_timecheck;
    std::chrono::steady_clock::time_point crosswalknow_stoptime;

    float stopline_mindistance;
    float stopsign_ignore;
    float crosswalksign_ignore;

    void DefaultState();

    void StopBeforeState();
    void StopNowState();
    void StopAfterState();

    void CrosswalkBeforeState();
    void CrosswalkNowState();
    void CrosswalkAfterState();

    void PriorityState();
};