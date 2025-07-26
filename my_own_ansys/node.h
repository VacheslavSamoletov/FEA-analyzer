#pragma once
#include <iostream>
#include <cmath>

class node {
    double _x, _y;
    size_t _number;
public:
    node(size_t number, double x = 0, double y = 0) : _number(number), _x(x), _y(y) {}

    double x() const { return _x; }
    double y() const { return _y; }
    size_t number() const { return _number; }

    double distance_to(const node& another) const {
        return sqrt(pow(_x - another._x, 2) + pow(_y - another._y, 2));
    }

    bool operator<(const node& other) const {
        return _number < other._number;
    }

    bool operator==(const node& other) const {
        return _number == other._number;
    }
};

std::ostream& operator<<(std::ostream& os, const node& n) {
    return os << "x = " << n.x() << ", y = " << n.y();
}