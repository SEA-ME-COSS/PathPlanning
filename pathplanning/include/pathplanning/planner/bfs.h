#pragma once

#include "planner/planner.h"

class BFS : public Planner {
public:
    BFS(Map& map);
    ~BFS();

    virtual bool path_planning(std::vector<int> start_point, std::vector<int> end_point);
    virtual void save_planned_path(std::vector<int> start_point, std::vector<int> end_point);
};