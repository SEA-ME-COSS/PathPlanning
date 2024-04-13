#pragma once

#include "utils/map.hpp"
#include "nav/a_star.hpp"
#include "nav/smoother.hpp"
#include "utils/ros2_msg_struct.h"
#include "utils/hybridastar_struct.h"
#include "utils/car_struct.h"

#include <iostream>
#include <array>
#include <chrono>

class Planner {
public:
    Planner(Map* map, double resolution, std::vector<std::array<int, 3>> waypoints);
    Planner();
    ~Planner();

    void plan_route();
    std::vector<std::vector<double>> get_route();

private:
    Map* map;
    Pose *pose;
    Car car;

    double resolution;

    fcc::FootprintCollisionChecker<Map, Point>::Footprint footprint;
    nav2_smac_planner::SearchInfo info;

    unsigned int theta_resolution;

	int max_iterations;
	float tolerance;
	int it_on_approach;

    nav2_smac_planner::Smoother<Map>* smoother;
    nav2_smac_planner::OptimizerParams optimizer_params;
    nav2_smac_planner::SmootherParams smoother_params;

    std::vector<std::vector<double>> waypoints_route;
    std::vector<std::array<int, 3>> waypoints;

    double pi_2_degree(double angle);
};