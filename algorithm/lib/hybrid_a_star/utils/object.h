struct ObstacleInfo {
    std::vector<double> center;
    int radius;
    std::vector<std::tuple<unsigned int, unsigned int, unsigned int>> occupiedTrace;
};

struct Object {
    std::string id;
    float x;
    float y;
    bool isDynamic = false;
};