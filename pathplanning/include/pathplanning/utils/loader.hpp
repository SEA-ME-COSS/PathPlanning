#pragma once

#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>
#include <stdexcept>

std::vector<std::vector<int>> load_map_data(const std::string& file_path);
std::map<int, std::vector<double>> load_nodes(const std::string& file_path);
std::vector<int> load_points(const std::string& file_path);
std::vector<std::vector<double>> load_waypoints(const std::string& nodes_file_path, const std::string& points_file_path);
