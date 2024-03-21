#pragma once

#include <string>

enum class VehicleState {
    Driving,
    CrosswalkBefore,
    CrosswalkAfter,
    CrosswalkNow,
    Priority,
};

struct Sign {
    std::string id;
    float distance;
};

struct Light {
    std::string id;
};

struct Object {
    std::string id;
    float x;
    float y;
    bool isDynamic = false;
};

struct StopLine {
    float distance;
};

struct Pose {
    float x;
    float y;
    float heading;
};