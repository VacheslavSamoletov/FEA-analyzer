#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include "node.h"
#include "displacement.h"
#include "nodal_displacement.h"
#include "element.h"
#include "linear_element.h"
#include "model.h"
#include <clocale>
using namespace std;

int main() {
    setlocale(LC_ALL, "Russian");
    // Пример 1: Простая ферма
    {
        cout << "Пример 1: Простая ферма\n";
        node n1(1, 0.0, 3.0);
        node n2(2, 0.0, 0.0);
        node n3(3, 0.0, -3.0);
        node n4(4, 4.0, 0.0);

        vector<element*> elements = {
            new linear_element(n1, n4, 2e11, 0.1e-3),
            new linear_element(n2, n4, 2e11, 0.1e-3),
            new linear_element(n3, n4, 2e11, 0.1e-3)
        };

        model m(elements);

        map<nodal_displacement, double> constraints = {
            {nodal_displacement(n1, displacement::ux), 0},
            {nodal_displacement(n1, displacement::uy), 0},
            {nodal_displacement(n2, displacement::ux), 0},
            {nodal_displacement(n2, displacement::uy), 0},
            {nodal_displacement(n3, displacement::ux), 0},
            {nodal_displacement(n3, displacement::uy), 0}
        };

        map<nodal_displacement, double> loads = {
            {nodal_displacement(n4, displacement::uy), -1000}
        };

        auto solution = m.solve(constraints, loads);
        cout << "Перемещения:\n";
        auto nodal_solution = m.get_the_nodal_solution(constraints, loads);
        for (const auto& pair : nodal_solution) {
            cout << pair.first << ": " << pair.second << endl;
        }

        cout << "\nРеакции:\n";
        auto reactions = m.nodal_reactions(constraints, loads, solution);
        for (const auto& pair : reactions) {
            cout << pair.first << ": " << pair.second << endl;
        }

        cout << "\nУсилия в элементах:\n";
        auto els = m.get_elements();
        for (size_t i = 0; i < els.size(); ++i) {
            auto forces = m.get_the_element_solution(nodal_solution, els[i]);
            cout << "Элемент " << i + 1 << ": " << forces.front() << endl;
        }
    }

    // Пример 2: Ферма Наседкина
    {
        cout << "\nПример 2: Ферма Наседкина\n";
        node n1(1, 0.0, 0.0);
        node n2(2, 2.0, 0.0);
        node n3(3, 4.0, 0.0);
        node n4(4, 2.5, sqrt(3.0) / 2);
        node n5(5, 1.0, sqrt(3.0));

        vector<element*> elements = {
            new linear_element(n1, n2, 2e11, 0.1e-3),
            new linear_element(n2, n3, 2e11, 0.1e-3),
            new linear_element(n3, n4, 2e11, 0.1e-3),
            new linear_element(n4, n5, 2e11, 0.1e-3),
            new linear_element(n5, n1, 2e11, 0.1e-3),
            new linear_element(n5, n2, 2e11, 0.1e-3),
            new linear_element(n2, n4, 2e11, 0.1e-3)
        };

        model m(elements);

        map<nodal_displacement, double> constraints = {
            {nodal_displacement(n1, displacement::uy), 0},
            {nodal_displacement(n3, displacement::uy), 0},
            {nodal_displacement(n3, displacement::ux), 0}
        };

        map<nodal_displacement, double> loads = {
            {nodal_displacement(n1, displacement::uy), -1000},
            {nodal_displacement(n3, displacement::uy), -1000},
            {nodal_displacement(n4, displacement::uy), -2000},
            {nodal_displacement(n5, displacement::uy), -2000}
        };

        auto solution = m.solve(constraints, loads);

        cout << "Реакции опор:\n";
        auto reactions = m.nodal_reactions(constraints, loads, solution);
        for (const auto& [nd, val] : reactions) {
            cout << nd << ": " << val << endl;
        }

        cout << "\nУсилия в стержнях:\n";
        auto nodal_solution = m.get_the_nodal_solution(constraints, loads);
        auto els = m.get_elements();
        for (size_t i = 0; i < els.size(); ++i) {
            auto forces = m.get_the_element_solution(nodal_solution, els[i]);
            cout << "Стержень " << i + 1 << ": " << forces.front() << endl;
        }
    }

    return 0;
}