#pragma once
#include "element.h"
#include "nodal_displacement.h"
#include <vector>
#include <map>
#include <algorithm>

class model {
    std::vector<element*> _elements;
    std::map<nodal_displacement, size_t> _nodes;
    std::vector<std::vector<double>> _stiffness_matrix;

    void create_dictionary() {
        size_t counter = 0;
        for (const auto& el : _elements) {
            auto nds = el->nodal_displacements();
            for (const auto& nd : nds) {
                if (_nodes.find(nd) == _nodes.end()) {
                    _nodes[nd] = counter++;
                }
            }
        }
    }

    void create_the_stiffness_matrix() {
        size_t n = _nodes.size();
        _stiffness_matrix = std::vector<std::vector<double>>(n, std::vector<double>(n, 0.0));

        for (const auto& el : _elements) {
            auto el_matrix = el->stiffness_matrix();
            auto el_nds = el->nodal_displacements();

            for (size_t i = 0; i < el_nds.size(); ++i) {
                size_t global_i = _nodes.at(el_nds[i]);
                for (size_t j = 0; j < el_nds.size(); ++j) {
                    size_t global_j = _nodes.at(el_nds[j]);
                    _stiffness_matrix[global_i][global_j] += el_matrix[i][j];
                }
            }
        }
    }

public:
    model(const std::vector<element*>& elements) : _elements(elements) {
        create_dictionary();
        create_the_stiffness_matrix();
    }

    ~model() {
        for (auto el : _elements) delete el;
    }

    model(const model&) = delete;
    model& operator=(const model&) = delete;

    std::vector<double> solve(
        const std::map<nodal_displacement, double>& constraints,
        const std::map<nodal_displacement, double>& loads) const {

        // Формирование правой части
        std::vector<double> rhs(_nodes.size(), 0.0);
        for (const auto& load_pair : loads) {
            const nodal_displacement& nd = load_pair.first;
            double load = load_pair.second;
            if (_nodes.count(nd)) {
                rhs[_nodes.at(nd)] = load;
            }
        }

        // Учет граничных условий
        std::vector<bool> is_fixed(_nodes.size(), false);
        for (const auto& [nd, value] : constraints) {
            if (_nodes.count(nd)) {
                size_t j = _nodes.at(nd);
                is_fixed[j] = true;
                for (size_t i = 0; i < _stiffness_matrix.size(); ++i) {
                    if (!is_fixed[i]) {
                        rhs[i] -= _stiffness_matrix[i][j] * value;
                    }
                }
            }
        }

        // Решение СЛАУ (упрощенный вариант - на практике используйте библиотеки)
        std::vector<double> solution(_nodes.size(), 0.0);
        for (size_t i = 0; i < _stiffness_matrix.size(); ++i) {
            if (!is_fixed[i]) {
                solution[i] = rhs[i] / _stiffness_matrix[i][i]; // Для диагональной матрицы
            }
        }

        // Восстановление заданных значений
        for (const auto& [nd, value] : constraints) {
            if (_nodes.count(nd)) {
                solution[_nodes.at(nd)] = value;
            }
        }

        return solution;
    }

    std::map<nodal_displacement, double> get_the_nodal_solution(
        const std::map<nodal_displacement, double>& constraints,
        const std::map<nodal_displacement, double>& loads) const {

        auto solution = solve(constraints, loads);
        std::map<nodal_displacement, double> result;

        for (const auto& pair : _nodes) {
            result[pair.first] = solution[pair.second];
        }

        for (const auto& pair: constraints) {
            result[pair.first] = solution[pair.second];
        }

        return result;
    }

    std::map<nodal_displacement, double> nodal_reactions(
        const std::map<nodal_displacement, double>& constraints,
        const std::map<nodal_displacement, double>& loads,
        const std::vector<double>& solution) const {

        std::vector<double> forces(_stiffness_matrix.size(), 0.0);
        for (size_t i = 0; i < _stiffness_matrix.size(); ++i) {
            for (size_t j = 0; j < _stiffness_matrix[i].size(); ++j) {
                forces[i] += _stiffness_matrix[i][j] * solution[j];
            }
        }

        std::map<nodal_displacement, double> reactions;
        for (const auto& [nd, value] : constraints) {
            if (_nodes.count(nd)) {
                reactions[nd] = forces[_nodes.at(nd)];
            }
        }

        for (const auto& [nd, load] : loads) {
            if (reactions.count(nd)) {
                reactions[nd] -= load;
            }
        }

        return reactions;
    }

    std::vector<element*> get_elements() const {
        return _elements;
    }

    std::vector<double> get_the_element_solution(
        const std::map<nodal_displacement, double>& nodal_solution,
        const element* el) const {

        auto element_disp = el->nodal_displacements();
        std::vector<double> displacements;

        for (const auto& nd : element_disp) {
            displacements.push_back(nodal_solution.at(nd));
        }

        return el->get_the_element_solution(displacements);
    }
};