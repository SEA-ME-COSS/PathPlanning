#pragma once

#include "utils/loader.hpp"

#include <vector>
#include <cmath>
#include <iostream>

class Map {
public:
    Map(const std::string& map_path);
    ~Map();

    std::vector<std::vector<int>> get_grid_map();
    std::vector<std::vector<bool>> get_binary_map();
    std::vector<int> get_map_info();

    bool is_blocked(std::vector<int> point);

private:
    int rows;
    int cols;

    bool is_outside(std::vector<int> point);
    void set_grid_map(std::vector<std::vector<int>> *map_data);
    void set_binary_map(std::vector<std::vector<int>> *map_data);

    std::vector<std::vector<int>> grid_map;
    std::vector<std::vector<bool>> booleanMapData;
};