#pragma once

#include <utils/point.h>
#include <utils/loader.h>

#include <vector>
#include <iostream>

class Map {
public:
    Map(const std::string& map_path);
    ~Map();

    std::vector<std::vector<Point>> get_map();
    std::vector<int> get_map_info();

    bool is_blocked(std::vector<int> point);

private:
    int rows;
    int cols;

    bool is_outside(std::vector<int> point);

    std::vector<std::vector<Point>> grid_map;
    std::vector<std::vector<bool>> booleanMapData;
};

Map::Map(const std::string& map_path) {
    std::vector<std::vector<int>> map_data = load_map_data(map_path);
    this->rows = map_data[0].size();
    this->cols = map_data.size();

    std::cout << rows << " , " << cols << std::endl;

    grid_map.resize(rows, std::vector<Point>(cols, Point()));

    for (int x = 0; x < rows; ++x) {
        for (int y = 0; y < cols; ++y) {
            uint8_t map_value = map_data[y][x];
            grid_map[x][y].structure = map_value;
            switch (map_value) {
                case 1: // solidline
                case 6: // roundabout
                    grid_map[x][y].blocked = true;
                    break;
                case 0: // stopline
                case 2: // crosswalk
                case 3: // dottedline
                case 4: // parkinglot
                case 5: // startline
                    grid_map[x][y].blocked = false;
                    break;
                default:
                    std::cerr << "Invalid map value: " << static_cast<int>(map_value) << " at (" << x << ", " << y << ")\n";
                    exit(EXIT_FAILURE);
                    break;
            }
        }
    }


    booleanMapData.resize(rows, std::vector<bool>(cols, false));

    for (int y = 0; y < cols; ++y) {
        for (int x = 0; x < rows; ++x) {
            uint8_t map_value = map_data[y][x];
            // Assuming 1 and 6 represent blocked areas
            booleanMapData[x][y] = map_value == 1 || map_value == 6;
        }
    }
    std::cout << "Map Size: " << grid_map.size() << " x " << grid_map[0].size() << std::endl;    
}

Map::~Map() {}

std::vector<std::vector<Point>> Map::get_map() {
    return grid_map;
}

std::vector<int> Map::get_map_info() {
    std::vector<int> map_info = {rows, cols};
    return map_info;
}

// bool Map::is_blocked(std::vector<int> point) {
//     if (is_outside(point)) {return true;}

//     if (grid_map[point[0]][point[1]].blocked) {return true;}
//     return false;
// }

// bool Map::is_outside(std::vector<int> point) {
//     if (point[0]<0 || rows<=point[0]) {return true;}
//     else if (point[1]<0 || cols<=point[1]) {return true;}
//     else {return false;}
// }

bool Map::is_blocked(std::vector<int> point) {
    // 지도 바깥에 있는지 확인
    if (point[0] < 0 || point[0] >= rows || point[1] < 0 || point[1] >= cols) {
        return true; // 지도 바깥에 있는 경우, 자동으로 차단된 것으로 처리
    }

    // 지도 내부의 지점이 차단되었는지 확인
    return booleanMapData[point[0]][point[1]];
}
