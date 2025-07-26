#pragma once
#include "element.h"
#include "node.h"
#include <vector>
#include <cmath>

class linear_element : public element {
    node _node1, _node2;
    double _young_modulus, _area;
    std::vector<std::vector<double>> _stiffness_matrix;

    std::vector<std::vector<double>> rotation_matrix() const {
        double dx = _node2.x() - _node1.x();
        double dy = _node2.y() - _node1.y();
        double L = sqrt(dx * dx + dy * dy);
        double c = dx / L;
        double s = dy / L;

        return {
            {c, -s, 0, 0},
            {s, c, 0, 0},
            {0, 0, c, -s},
            {0, 0, s, c}
        };
    }

public:
    linear_element(const node& node1, const node& node2, double young_modulus, double area)
        : _node1(node1), _node2(node2), _young_modulus(young_modulus), _area(area) {

        double L = _node1.distance_to(_node2);
        double k = _young_modulus * _area / L;

        // Локальная матрица жесткости
        std::vector<std::vector<double>> K_local = {
            {k, 0, -k, 0},
            {0, 0, 0, 0},
            {-k, 0, k, 0},
            {0, 0, 0, 0}
        };

        auto Q = rotation_matrix();

        // Умножение матриц: Q^T * K_local * Q
        _stiffness_matrix = multiply_matrices(transpose_matrix(Q),
            multiply_matrices(K_local, Q));
    }

    std::vector<nodal_displacement> nodal_displacements() const override {
        return {
            nodal_displacement(_node1, displacement::ux),
            nodal_displacement(_node1, displacement::uy),
            nodal_displacement(_node2, displacement::ux),
            nodal_displacement(_node2, displacement::uy)
        };
    }

    std::vector<std::vector<double>> stiffness_matrix() const override {
        return _stiffness_matrix;
    }

    std::vector<double> get_the_element_solution(const std::vector<double>& displacements) const override {
        auto Q = rotation_matrix();
        auto local_disp = multiply_matrix_vector(Q, displacements);

        double L = _node1.distance_to(_node2);
        double k = _young_modulus * _area / L;

        std::vector<std::vector<double>> K_local = {
            {k, 0, -k, 0},
            {0, 0, 0, 0},
            {-k, 0, k, 0},
            {0, 0, 0, 0}
        };

        std::vector<double> forces(4, 0.0);
        for (size_t i = 0; i < 4; ++i) {
            for (size_t j = 0; j < 4; ++j) {
                forces[i] += K_local[i][j] * local_disp[j];
            }
        }
        return forces;
    }

private:
    static std::vector<std::vector<double>> transpose_matrix(const std::vector<std::vector<double>>& m) {
        std::vector<std::vector<double>> result(m[0].size(), std::vector<double>(m.size()));
        for (size_t i = 0; i < m.size(); ++i)
            for (size_t j = 0; j < m[0].size(); ++j)
                result[j][i] = m[i][j];
        return result;
    }

    static std::vector<std::vector<double>> multiply_matrices(
        const std::vector<std::vector<double>>& a,
        const std::vector<std::vector<double>>& b) {

        std::vector<std::vector<double>> result(a.size(), std::vector<double>(b[0].size(), 0));
        for (size_t i = 0; i < a.size(); ++i)
            for (size_t j = 0; j < b[0].size(); ++j)
                for (size_t k = 0; k < b.size(); ++k)
                    result[i][j] += a[i][k] * b[k][j];
        return result;
    }

    static std::vector<double> multiply_matrix_vector(
        const std::vector<std::vector<double>>& m,
        const std::vector<double>& v) {

        std::vector<double> result(m.size(), 0);
        for (size_t i = 0; i < m.size(); ++i)
            for (size_t j = 0; j < v.size(); ++j)
                result[i] += m[i][j] * v[j];
        return result;
    }
};