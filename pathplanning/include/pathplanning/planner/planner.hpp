#pragma once

#include "utils/map.hpp"

#include <deque>
#include <array>
#include <utility>
#include <map>
#include <cstdint>

class Planner {
public:
    Planner(Map& map);
    ~Planner();

    void plan_with_waypoints(std::vector<std::vector<double>> waypoints);
    std::vector<std::vector<int>> get_waypoints_path();

protected:
    Map& map;

    // Deque(to Visit) and Map(Current & Previous) for Graph Search
    std::map<std::vector<int>, std::vector<int>> visit_map;

    void clear_storage();
    void set_visit_and_prev(std::vector<int> old_point, std::vector<int> new_point);
    bool is_visited(std::vector<int> point);
    std::vector<int> get_prev_point(std::vector<int> point);

    virtual bool path_planning(std::vector<int> start_point, std::vector<int> end_point) = 0;
    virtual void save_planned_path(std::vector<int> start_point, std::vector<int> end_point) = 0;

    std::deque<std::vector<int>> iteration_path;
    std::vector<std::vector<int>> waypoints_route;
};