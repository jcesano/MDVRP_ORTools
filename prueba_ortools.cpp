#include "stdafx.h"
#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_parameters.h"
#include <iostream>
#include <vector>

using namespace operations_research;

// Definición de datos del problema
const int num_vehiculos = 3;
const int deposito = 0;
const int capacidad_vehiculo = 15;

// Matriz de distancias
const int num_nodos = 6;
const int distancia[num_nodos][num_nodos] = {
    {0, 2, 9, 10, 7, 6},
    {2, 0, 4, 3, 2, 5},
    {9, 4, 0, 8, 6, 7},
    {10, 3, 8, 0, 9, 4},
    {7, 2, 6, 9, 0, 3},
    {6, 5, 7, 4, 3, 0}
};

// Demandas
const int demanda[num_nodos] = { 0, 4, 3, 5, 7, 6 };

void ResolverVRP() {
    // 🔹 Opción 1: Si usas un solo depósito
    //RoutingIndexManager manager(num_nodos, num_vehiculos, deposito);
    operations_research::RoutingNodeIndex depot_index = static_cast<operations_research::RoutingNodeIndex>(deposito);
    RoutingIndexManager manager(num_nodos, num_vehiculos, depot_index);

    // 🔹 Opción 2: Si necesitas definir diferentes inicios y finales
    // std::vector<int> starts(num_vehiculos, deposito);
    // std::vector<int> ends(num_vehiculos, deposito);
    // RoutingIndexManager manager(num_nodos, num_vehiculos, starts, ends);

    // Crear el modelo de ruteo
    RoutingModel routing(manager);

    // Función de costo basada en la matriz de distancias
    const int transit_callback_index = routing.RegisterTransitCallback(
        [](int64_t from_index, int64_t to_index) -> int64_t {
            int from_node = static_cast<int>(from_index);
            int to_node = static_cast<int>(to_index);
            return distancia[from_node][to_node];
        });

    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

    // Restricción de capacidad
    const int demand_callback_index = routing.RegisterUnaryTransitCallback(
        [](int64_t from_index) -> int64_t {
            int from_node = static_cast<int>(from_index);
            return demanda[from_node];
        });

    std::vector<int64_t> capacidades(num_vehiculos, capacidad_vehiculo);
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,
        capacidades,
        true,
        "Capacity");

    // Configuración del solucionador
    RoutingSearchParameters search_parameters = DefaultRoutingSearchParameters();
    search_parameters.set_first_solution_strategy(FirstSolutionStrategy::PATH_CHEAPEST_ARC);
    search_parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);

    // Resolver el problema
    const Assignment* solution = routing.SolveWithParameters(search_parameters);

    // Mostrar la solución
    if (solution != nullptr) {
        for (int veh = 0; veh < num_vehiculos; ++veh) {
            std::cout << "Ruta para el vehículo " << veh << ": ";
            int64_t index = routing.Start(veh);
            while (!routing.IsEnd(index)) {
                std::cout << manager.IndexToNode(index) << " -> ";
                index = solution->Value(routing.NextVar(index));
            }
            std::cout << manager.IndexToNode(index) << "\n";
        }
    }
    else {
        std::cout << "No se encontró solución.\n";
    }
}

int main() {
    ResolverVRP();
    return 0;
}
