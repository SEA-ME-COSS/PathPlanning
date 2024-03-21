#include "planner/planner.h"

Planner::Planner(Map& map) : map(map) {}

Planner::~Planner() {}

void Planner::clear_storage() {
    this->visit_map.clear();
    this->iteration_path.clear();
}

void Planner::plan_with_waypoints(std::vector<std::vector<double>> waypoints) {
    int waypoints_size = waypoints.size();

    std::cout<<"start_x : "<<waypoints[0][0]<<'\t'<<"start_y : "<<waypoints[0][1]<<std::endl;
    std::cout<<"end_x : "<<waypoints[waypoints_size-1][0]<<'\t'<<"end_y : "<<waypoints[waypoints_size-1][1]<<std::endl;

    for(int k = 0; k < (waypoints_size-1); ++k) {
        this->clear_storage();
            
        std::vector<int> start_point = {static_cast<int>(round(waypoints[k][0])), static_cast<int>(round(waypoints[k][1]))};
        std::vector<int> end_point = {static_cast<int>(round(waypoints[k+1][0])), static_cast<int>(round(waypoints[k+1][1]))};

        if (this->path_planning(start_point, end_point)) {
            this->save_planned_path(start_point, end_point);
            for (size_t i = 0; i < iteration_path.size(); ++i) {
                waypoints_route.push_back(iteration_path[i]);
            }
        }
        else {
            std::cout<<"No Way"<<std::endl;
            break;    
        }
    }
    // std::cout << "Plan Finish" << std::endl;
}

std::vector<std::vector<int>> Planner::get_waypoints_path() {
    return waypoints_route;
}

void Planner::set_visit_and_prev(std::vector<int> old_point, std::vector<int> new_point) {
    visit_map[new_point] = old_point;
}

bool Planner::is_visited(std::vector<int> point) {
    return visit_map.contains(point);
}

std::vector<int> Planner::get_prev_point(std::vector<int> point) {
    return visit_map[point];
}