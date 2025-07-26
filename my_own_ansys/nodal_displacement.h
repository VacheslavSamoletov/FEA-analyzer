#pragma once
#include "node.h"
#include "displacement.h"

class nodal_displacement {
    node _node;
    displacement _displacement;
public:
    nodal_displacement(const node& n, displacement d) : _node(n), _displacement(d) {}

    node get_node() const { return _node; }
    displacement get_displacement() const { return _displacement; }

    bool operator==(const nodal_displacement& that) const {
        return _node == that._node && _displacement == that._displacement;
    }

    bool operator<(const nodal_displacement& that) const {
        if (_node == that._node)
            return _displacement < that._displacement;
        return _node < that._node;
    }
};

std::ostream& operator<<(std::ostream& os, const nodal_displacement& nd) {
    os << nd.get_node();
    switch (nd.get_displacement()) {
    case displacement::ux: os << ", ux"; break;
    case displacement::uy: os << ", uy"; break;
    case displacement::rotz: os << ", rotz"; break;
    }
    return os;
}