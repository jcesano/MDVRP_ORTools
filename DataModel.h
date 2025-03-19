#ifndef DATAMODEL_H_
#define DATAMODEL_H_

#include <vector>

class DistanceTable;

class DataModel {
public:
    std::vector<std::vector<int>> distance_matrix;
    int num_vehicles;
    int depot;

    // Constructor que toma una DistanceTable y la convierte
    DataModel(DistanceTable* dt, int vehicles, int depotIndex);

    // Función para convertir DistanceTable en una matriz compatible con OR-Tools
    static std::vector<std::vector<int>> ConvertDistanceTableToMatrix(DistanceTable* dt);
};

#endif  // DATAMODEL_H_
