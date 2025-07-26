#pragma once
#include <vector>
#include "nodal_displacement.h"

class element {
public:
    virtual ~element() = default;
    virtual std::vector<nodal_displacement> nodal_displacements() const = 0;
    virtual std::vector<std::vector<double>> stiffness_matrix() const = 0;
    virtual std::vector<double> get_the_element_solution(const std::vector<double>&) const = 0;
};