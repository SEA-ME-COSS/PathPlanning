#pragma once

#include <cstdint>

class Point {
public:
    Point();
    ~Point();

    uint8_t structure;
    bool blocked;
};

Point::Point() {
    blocked = false;
}

Point::~Point() {
}
