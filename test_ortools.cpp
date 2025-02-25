#include "stdafx.h"
#include "ortools/linear_solver/linear_solver.h"
#include <iostream>

int main() {
    using namespace operations_research;

    // Crear el solver con GLOP (para programación lineal)
    std::unique_ptr<MPSolver> solver(MPSolver::CreateSolver("GLOP"));
    if (!solver) {
        std::cout << "Error: Solver no disponible." << std::endl;
        return 1;
    }

    // Variables
    MPVariable* x = solver->MakeNumVar(0.0, 1.0, "x");
    MPVariable* y = solver->MakeNumVar(0.0, 2.0, "y");

    // Restricciones
    MPConstraint* c1 = solver->MakeRowConstraint(0.0, 2.0);
    c1->SetCoefficient(x, 1);
    c1->SetCoefficient(y, 1);

    // Función objetivo
    MPObjective* objective = solver->MutableObjective();
    objective->SetCoefficient(x, 3);
    objective->SetCoefficient(y, 2);
    objective->SetMaximization();

    // Resolver
    solver->Solve();
    std::cout << "Solución: x = " << x->solution_value() << ", y = " << y->solution_value() << std::endl;
    std::cout << "Valor óptimo: " << objective->Value() << std::endl;

    return 0;
}
