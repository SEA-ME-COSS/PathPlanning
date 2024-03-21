#pragma once

#include "planner/planner.h"

#include <queue>
#include <functional>
#include <cmath>

class A_Star : public Planner {
public:
    A_Star(Map& map);
    ~A_Star();

    virtual bool path_planning(std::vector<int> start_point, std::vector<int> end_point);
    virtual void save_planned_path(std::vector<int> start_point, std::vector<int> end_point);

private:
    int distance(std::vector<int> point1, std::vector<int> point2);
    int heuristic(std::vector<int> point, std::vector<int> end_point);
};