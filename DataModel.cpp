#include "stdafx.h"
#include "DataModel.h"
#include "DistanceTable.h"  // Ahora sí lo incluimos en el .cpp

// Implementación de la función de conversión
std::vector<std::vector<int>> DataModel::ConvertDistanceTableToMatrix(DistanceTable* dt) {
    int numNodes = dt->getNumberOfVertices();
    std::vector<std::vector<int>> distance_matrix(numNodes, std::vector<int>(numNodes, 0));

    for (int i = 0; i < numNodes; i++) {
        for (int j = 0; j < numNodes; j++) {
            float distance = dt->getEdge(i, j);
            if (distance == dt->NO_ADJ) {
                distance_matrix[i][j] = 999999;  // Valor alto para representar "sin conexión"
            }
            else {
                distance_matrix[i][j] = static_cast<int>(distance); // OR-Tools usa enteros
            }
        }
    }

    return distance_matrix;
}

// Implementación del constructor de DataModel
DataModel::DataModel(DistanceTable* dt, int vehicles, int depotIndex)
    : num_vehicles(vehicles), depot(depotIndex) {
    distance_matrix = ConvertDistanceTableToMatrix(dt);
}
