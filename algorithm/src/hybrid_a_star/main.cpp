#include "map/hybrid_a_star_map.hpp"
#include "hybrid-a-star/a_star.hpp"
#include "planner/hybrid_a_star_planner.hpp"

#include "utils/matplotlibcpp.h"
#include "utils/point.h"
#include "utils/loader.h"

#include <iostream>
#include <memory>
#include <cmath>
#include <vector>
#include <array>
#include <deque>
#include <fstream>
#include <sstream>

namespace plt = matplotlibcpp;

int main() {
    // Load Graph and Waypoints
    std::string nodes_file_path = "../globalmap/parsinginfo.txt";
    std::string points_file_path = "../globalmap/waypoints.txt";
    std::string points_set_file_path = "../globalmap/waypoints-set.txt";
    std::vector<std::array<int, 3>> waypoints = load_waypoints(nodes_file_path, points_file_path);
    std::vector<std::array<int, 3>> waypoints_set = load_waypoints(nodes_file_path, points_set_file_path);

    // Load and Configure Map
    std::string mapdata_file_path = "../globalmap/flipped-track.txt";
    double resolution = 0.77;
    Map *map = new Map(mapdata_file_path, resolution, waypoints_set);

    double vehicle_width = 15.0;
    double vehicle_length = 30.0;
    float change_penalty = 3.0;
	float non_straight_penalty = 2.0;
	float reverse_penalty = 2.0;
	float minimum_turning_radius = 60.0;
	int theta_resolution = 5;

	int max_iterations = 10000;
	float tolerance = 10.0;
	int it_on_approach = 10;
    
    Planner planner = Planner(map, vehicle_width, vehicle_length,
                            change_penalty, non_straight_penalty, reverse_penalty, minimum_turning_radius, theta_resolution,
                            max_iterations, tolerance, it_on_approach);

    std::array<int, 3> startpos = {35, 65, 0};

    planner.plan_route(startpos, waypoints);
    std::vector<std::vector<double>> route = planner.get_route();

    // Only for drawing
    std::cout << "Draw Map" << std::endl;
    std::map<int, float> structure_color_map = {
        {0, 1.0},
        {255, 0.0},
        {254, 0.1},
        {253, 0.7},
        {252, 0.8},
    };

    // Draw map info
    std::vector<std::vector<unsigned int>> draw_grid_map = map->get_cost_map();
    std::vector<double> draw_map_info = map->get_map_info();

    int size_x = static_cast<int>(draw_map_info[0]);
    int size_y = static_cast<int>(draw_map_info[1]);

    std::vector<float> image_data(size_x * size_y);

    for(int x = 0; x < size_x; ++x) {
        for(int y = 0; y < size_y; ++y) {
            int structure = draw_grid_map[y][x];
            int index = y * size_x + x;
            image_data[index] = structure_color_map[structure];
        }
    }

    plt::imshow(&image_data[0], size_y, size_x, 1);

    // Draw global path
    double draw_x;
    double draw_y;
    double draw_theta;
    std::vector<double> path_x;
    std::vector<double> path_y;

    std::array<double, 2> heading_x;
    std::array<double, 2> heading_y;

    std::array<double, 5> car_x;
    std::array<double, 5> car_y;
    double car_w = 15;
    double car_h = 30;
    
    for (size_t i = 0; i < route.size(); i+=20) {
        draw_x = route[i][0];
        draw_y = route[i][1];
        draw_theta = route[i][2];

        path_x.push_back(draw_x);
        path_y.push_back(draw_y);

        heading_x[0] = draw_x;
        heading_x[1] = draw_x + 1.5*car_h * cos(draw_theta);
        heading_y[0] = draw_y;
        heading_y[1] = draw_y - 1.5*car_h * -sin(draw_theta);

        car_x[0] = draw_x + car_h * cos(draw_theta) - car_w * -sin(draw_theta);
        car_x[1] = draw_x + car_h * cos(draw_theta) + car_w * -sin(draw_theta);
        car_x[2] = draw_x - car_h * cos(draw_theta) + car_w * -sin(draw_theta);
        car_x[3] = draw_x - car_h * cos(draw_theta) - car_w * -sin(draw_theta);
        car_x[4] = car_x[0];
        car_y[0] = draw_y - (car_h * -sin(draw_theta) + car_w * cos(draw_theta));
        car_y[1] = draw_y - (car_h * -sin(draw_theta) - car_w * cos(draw_theta));
        car_y[2] = draw_y - (- car_h * -sin(draw_theta) - car_w * cos(draw_theta));
        car_y[3] = draw_y - (-car_h * -sin(draw_theta) + car_w * cos(draw_theta));
        car_y[4] = car_y[0];

        // std::cout<<route[i][0]<<" "<<route[i][1]<<std::endl;
        plt::plot(path_x, path_y, "bo-");
        plt::plot(heading_x, heading_y, "r-");
        plt::plot(car_x, car_y, "r-");
        plt::pause(0.01);
        plt::clf();

        plt::imshow(&image_data[0], size_y, size_x, 1);
    }

    plt::plot(path_x, path_y, "bo-");
    plt::plot(heading_x, heading_y, "r-");
    plt::plot(car_x, car_y, "r-");
    plt::pause(10);
    
	delete map;
     
    return 0;
}