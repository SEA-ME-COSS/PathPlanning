#include <utils/map.h>
#include <utils/loader.h>
#include <visual/matplotlibcpp.h>

#include <planner/bfs.h>
#include <planner/a_star.h>

#include <iostream>
#include <cmath>
#include <fstream>
#include <sstream>

namespace plt = matplotlibcpp;

int main() {
    // Load Waypoints Info
    std::string nodes_file_path = "../globalmap/parsinginfo.txt";
    std::string waypoints_file_path = "../globalmap/test-waypoints.txt";
    std::vector<std::vector<double>> waypoints = load_waypoints(nodes_file_path, waypoints_file_path);

    // Make Map instruction
    std::string map_file_path = "../globalmap/flipped-track.txt";
    Map map = Map(map_file_path);

    // Path Planner (Using same map)
    auto planner = A_Star(map);
    // auto planner = A_Star(map);
    planner.plan_with_waypoints(waypoints);
    std::vector<std::vector<int>> route = planner.get_waypoints_path();

    // Only for drawing
    std::cout << "Draw Map" << std::endl;
    std::map<int, float> structure_color_map = {
        {0, 1.0}, // solidline (실선)
        {1, 0.5}, // stopline (정지선)
        {2, 0.3}, // crosswalk (횡단보도)
        {3, 0.4}, // dottedline (점선)
        {4, 0.5}, // parkinglot (주차장)
        {5, 0.6}, // roundabout (로터리)
        {6, 0.7}, // startline (시작구역)
        {7, 0.0}, // floor (차도)
        {8, 0.8}, // busline (버스전용차선)
        {9, 0.9} // sidewalk (인도 or 차 못가는 모든 곳)
    };

    // Draw map info
    std::vector<std::vector<Point>> draw_grid_map = map.get_map();
    std::vector<int> draw_map_info = map.get_map_info();

    int map_width = draw_map_info[0];
    int map_height = draw_map_info[1];

    std::vector<float> image_data(map_width * map_height);

    for(int x=0; x<map_width; ++x) {
        for(int y=0; y<map_height; ++y) {
            int structure = draw_grid_map[x][y].structure;
            int index = y * map_width + x;
            image_data[index] = structure_color_map[structure];
        }
    }

    plt::imshow(&image_data[0], map_height, map_width, 1);

    // Draw global path : reverse mode
    for (size_t step = 0; step < route.size(); step += 5) {
        std::vector<int> path_x;
        std::vector<int> path_y;

        for (size_t i = 0; i <= step; ++i) {
            path_x.push_back(route[i][0]);
            path_y.push_back(route[i][1]);
        }

        plt::plot(path_x, path_y, "r-");
        plt::pause(0.01);
        plt::clf();

        plt::imshow(&image_data[0], map_height, map_width, 1);
    }

    plt::show();

    return 0;
}