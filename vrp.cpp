// Copyright 2010-2024 Google LLC
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// [START program]
// [START import]
#include "stdafx.h"
#include <cstdint>
#include <sstream>
#include <vector>
#include <iostream>
#include <time.h>
#include <chrono>
#include <string>
#include <random>
#include <utility>

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

// Imported from original MDVRP
// MDVRP_JC.cpp : Defines the entry point for the console application.
//


#include "FrogLeapSolution.h"
#include "FrogLeapController.h"
#include "DecodedFrogLeapSolution.h"
#include "FrogObjectCol.h"
#include "IndexList.h"
#include "FeasibleSolution.h"
#include "DistanceTable.h"
#include "Graph.h"
#include "FeasibleSolCol.h"
#include "DistanceTable.h"

//#include "MDVRP_JC.h"
// [END import]

using std::string;

using namespace std;

void setDepot(FrogLeapController* controller, Graph* g, int vertIdLabel, int capacity)
{
    int vertexIndex = g->getPositionVertexById(vertIdLabel);

    if (vertexIndex != -1)
    {
        controller->setAsDepot(vertexIndex, capacity);
    }
}

void setCustomer(FrogLeapController* controller, Graph* g, int vertIdLabel, int capacity)
{
    int vertexIndex = g->getPositionVertexById(vertIdLabel);

    if (vertexIndex != -1)
    {
        controller->setAsCustomer(vertexIndex, capacity);
    }
}

namespace operations_research {
// [START data_model]
struct DataModel {
  const std::vector<std::vector<int64_t>> distance_matrix{
      {0, 548, 776, 696, 582, 274, 502, 194, 308, 194, 536, 502, 388, 354, 468,
       776, 662},
      {548, 0, 684, 308, 194, 502, 730, 354, 696, 742, 1084, 594, 480, 674,
       1016, 868, 1210},
      {776, 684, 0, 992, 878, 502, 274, 810, 468, 742, 400, 1278, 1164, 1130,
       788, 1552, 754},
      {696, 308, 992, 0, 114, 650, 878, 502, 844, 890, 1232, 514, 628, 822,
       1164, 560, 1358},
      {582, 194, 878, 114, 0, 536, 764, 388, 730, 776, 1118, 400, 514, 708,
       1050, 674, 1244},
      {274, 502, 502, 650, 536, 0, 228, 308, 194, 240, 582, 776, 662, 628, 514,
       1050, 708},
      {502, 730, 274, 878, 764, 228, 0, 536, 194, 468, 354, 1004, 890, 856, 514,
       1278, 480},
      {194, 354, 810, 502, 388, 308, 536, 0, 342, 388, 730, 468, 354, 320, 662,
       742, 856},
      {308, 696, 468, 844, 730, 194, 194, 342, 0, 274, 388, 810, 696, 662, 320,
       1084, 514},
      {194, 742, 742, 890, 776, 240, 468, 388, 274, 0, 342, 536, 422, 388, 274,
       810, 468},
      {536, 1084, 400, 1232, 1118, 582, 354, 730, 388, 342, 0, 878, 764, 730,
       388, 1152, 354},
      {502, 594, 1278, 514, 400, 776, 1004, 468, 810, 536, 878, 0, 114, 308,
       650, 274, 844},
      {388, 480, 1164, 628, 514, 662, 890, 354, 696, 422, 764, 114, 0, 194, 536,
       388, 730},
      {354, 674, 1130, 822, 708, 628, 856, 320, 662, 388, 730, 308, 194, 0, 342,
       422, 536},
      {468, 1016, 788, 1164, 1050, 514, 514, 662, 320, 274, 388, 650, 536, 342,
       0, 764, 194},
      {776, 868, 1552, 560, 674, 1050, 1278, 742, 1084, 810, 1152, 274, 388,
       422, 764, 0, 798},
      {662, 1210, 754, 1358, 1244, 708, 480, 856, 514, 468, 354, 844, 730, 536,
       194, 798, 0},
  };
  const int num_vehicles = 4;
  const RoutingIndexManager::NodeIndex depot{0};
};
// [END data_model]

// [START solution_printer]
//! @brief Print the solution.
//! @param[in] data Data of the problem.
//! @param[in] manager Index manager used.
//! @param[in] routing Routing solver used.
//! @param[in] solution Solution found by the solver.
void PrintSolution(const DataModel& data, const RoutingIndexManager& manager,
                   const RoutingModel& routing, const Assignment& solution) {
  LOG(INFO) << "Objective: " << solution.ObjectiveValue();
  int64_t total_distance{0};
  for (int vehicle_id = 0; vehicle_id < data.num_vehicles; ++vehicle_id) {
    int64_t index = routing.Start(vehicle_id);
    LOG(INFO) << "Route for Vehicle " << vehicle_id << ":";
    int64_t distance{0};
    std::stringstream route;
    while (!routing.IsEnd(index)) {
      route << manager.IndexToNode(index).value() << " -> ";
      const int64_t previous_index = index;
      index = solution.Value(routing.NextVar(index));
      distance += routing.GetArcCostForVehicle(previous_index, index,
                                               int64_t{vehicle_id});
    }
    LOG(INFO) << route.str() << manager.IndexToNode(index).value();
    LOG(INFO) << "Distance of the route: " << distance << "m";
    total_distance += distance;
  }
  LOG(INFO) << "Total distance of all routes: " << total_distance << "m";
  LOG(INFO) << "";
  LOG(INFO) << "Advanced usage:";
  LOG(INFO) << "Problem solved in " << routing.solver()->wall_time() << "ms";
}
// [END solution_printer]

void Vrp() {
  // Instantiate the data problem.
  // [START data]
  DataModel data;
  // [END data]

  // Create Routing Index Manager
  // [START index_manager]
  RoutingIndexManager manager(data.distance_matrix.size(), data.num_vehicles,
                              data.depot);
  // [END index_manager]

  // Create Routing Model.
  // [START routing_model]
  RoutingModel routing(manager);
  // [END routing_model]

  // Create and register a transit callback.
  // [START transit_callback]
  const int transit_callback_index = routing.RegisterTransitCallback(
      [&data, &manager](const int64_t from_index,
                        const int64_t to_index) -> int64_t {
        // Convert from routing variable Index to distance matrix NodeIndex.
        const int from_node = manager.IndexToNode(from_index).value();
        const int to_node = manager.IndexToNode(to_index).value();
        return data.distance_matrix[from_node][to_node];
      });
  // [END transit_callback]

  // Define cost of each arc.
  // [START arc_cost]
  routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);
  // [END arc_cost]

  // Setting first solution heuristic.
  // [START parameters]
  RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
  searchParameters.set_first_solution_strategy(
      FirstSolutionStrategy::PATH_CHEAPEST_ARC);
  // [END parameters]

  // Solve the problem.
  // [START solve]
  const Assignment* solution = routing.SolveWithParameters(searchParameters);
  // [END solve]

  // Print solution on console.
  // [START print_solution]
  PrintSolution(data, manager, routing, *solution);
  // [END print_solution]
}
}  // namespace operations_research

int main(int /*argc*/, char* /*argv*/[]) {
  //operations_research::Vrp();
	auto start_time = std::chrono::high_resolution_clock::now();

	// create the graph given in above fugure 

	int V = 9;

	FrogLeapController* controller = new FrogLeapController();

	const char* fileName = "casog01.vrp";
	//char * fileName = "casog09.vrp";
	//char* fileName = "ClusterId_0_Input.vrp";
	//char* fileName = "ClusterId_1_copy.vrp";
	//char* fileName = "casoch01.vrp";
	//char* fileName = "casoch02swp.txt";
	const char* testCaseFileName = "TestCase01Casog01 - copia.txt";


	DecodedFrogLeapSolution* dfls_1 = NULL, * dfls_2 = NULL;

	controller->setSourceType(SourceType::Tsp2DEuc);
	//controller->setSourceType(SourceType::ClarkWright);
	//controller->setSourceType(SourceType::Sweep);

	switch (controller->getSourceType())
	{
	case SourceType::Graph:
	{
		Graph* g = new Graph(V);

		g->addEdge(0, 1, 4);
		g->addEdge(0, 7, 8);
		g->addEdge(1, 2, 8);
		g->addEdge(1, 7, 11);
		g->addEdge(2, 3, 7);
		g->addEdge(2, 8, 2);
		g->addEdge(2, 5, 4);
		g->addEdge(3, 4, 9);
		g->addEdge(3, 5, 14);
		g->addEdge(4, 5, 10);
		g->addEdge(5, 6, 2);
		g->addEdge(6, 7, 1);
		g->addEdge(6, 8, 6);
		g->addEdge(7, 8, 7);

		int vertexIndex = g->getPositionVertexById(0);
		DistVect* dv = NULL;

		if (vertexIndex != -1)
		{
			dv = g->dijkstra(vertexIndex);
		}

		/* main to test Dijkstra algorithm */

		setDepot(controller, g, 0, 12);
		setDepot(controller, g, 6, 14);

		setCustomer(controller, g, 1, 3);
		setCustomer(controller, g, 2, 3);
		setCustomer(controller, g, 3, 3);
		setCustomer(controller, g, 4, 3);
		setCustomer(controller, g, 7, 3);
		setCustomer(controller, g, 8, 3);

		/* main to fill the Distance Table */
		DistanceTable* dt = new DistanceTable(V);

		dt->fillDistanceTable(dv, g);

		dt->printDistanceTable();

		g->setDistanceTable(dt);

		controller->setGraph(g);

		controller->setDistanceTable(dt);

		dt = NULL;

		controller->setUpCustomerAndDepotLists();

	}
	break;
	case SourceType::Tsp2DEuc:
	{
		controller->loadTSPEUC2D_Data(fileName);
		controller->setUpCustomerAndDepotLists();
		controller->setUpVehicleCapacity();
		controller->loadDistanceTable();
	}
	break;
	case SourceType::ClarkWright:
	{
		controller->cw_loadDistanceTable();
		controller->cw_loadCustomersAndDepots();
		controller->cw_loadVehicleCapacity();
	}
	break;
	default:
		break;
	}

	//controller->setUpVehiclesPerDepot();	

	/* Main to test all permutations of distance = 1 and distance = 2

	FeasibleSolution * fs;
	FeasibleSolCol * fscol;

	fs = new FeasibleSolution(V);
	*/

	/* setting indexes as values in the FeasibleSolution
	for(int i = 0; i<V;i++)
	{
	fs->setSolFactValue(i, i);
	}

	int distance;
	distance = 1;
	fscol = fs->genPermutations(distance,NULL, controller);
	fscol->printFeasSolCol();
	*/

	/* main to test random vector (FeasibleSolution)
	fs = new FeasibleSolution(V);

	fs->setRandomSolution();

	fs->printFeasibleSolution();
	*/

	/* main to test the random solution generator
	//FeasibleSolution * fs = new FeasibleSolution(V);
	fs->setRandomSolution();
	fscol = fs->genRandomFeasibleSolCol();

	fscol->printFeasSolCol();
	*/


	/* main test frogSolution */

	DistanceTable* dt = controller->getDistanceTable();

	//printf("Testing distance table: dt[%d, %d] = %f", 260, 54, dt[260, 54]);

	int nDepots = controller->getNumberOfDepots();
	int nCustomers = controller->getNumberOfCustomers();

	dfls_1 = NULL;
	float evalSol, evalSol2;
	const long long int itNumber = controller->getNumberOfIterations();
	long long int i = 0, min_i = 0;
	long long int timeBound, execTime;
	timeBound = 10800000;

	auto end_time = std::chrono::high_resolution_clock::now();
	auto time = end_time - start_time;
	execTime = std::chrono::duration_cast<std::chrono::milliseconds>(time).count();
	bool isFeasibleFLS = false;

	printf("Program execution started ... \n");

	controller->openOutPutFile();

	controller->writeSeed();
	int counterAux = 0;
	bool testMode = false;
	FrogLeapSolution* fls = NULL;

	//while(execTime <= timeBound)
	while (i < 4000)//(i < 4000)	//while(true)
	{
		//controller->writeFrogLeapSolution(fls);

		//fls->printFrogObj();

		//FrogLeapSolution* fls = new FrogLeapSolution(SolutionGenerationType::FrogLeaping, controller->getSourceType(), nCustomers, nDepots, 0);

		if (i == 6)
		{
			printf("parar aca \n");
		}

		if (testMode == false)
		{
			fls = new FrogLeapSolution(SolutionGenerationType::FrogLeaping, controller->getSourceType(), nCustomers, nDepots, 0);

			// reset all values of the frogleap solution with -1
			fls->setAllFLSWithValue(-1);
			isFeasibleFLS = fls->genRandomSolution14_0(controller);

			if (isFeasibleFLS == true)
			{
				//fls->writeFrogLeapSolution(controller);

				//dfls_1 = fls->decodeWholeSolutionWithSimplifiedClosestNextCriteria(controller);
				//dfls_1 = fls->decodeWholeSolutionWithClosestNextCriteria(controller);
				//dfls_1 = fls->decodeWholeSolutionWithMixedCriteria(controller);
				//dfls_1 = fls->decodeWholeSolutionWithAngularCriteria(controller);
				//dfls_1 = fls->decodeWholeSolutionWithClarkWrightCriteria(controller);
				dfls_1 = fls->decodeWholeSolutionWithClarkWrightCriteria2(controller);
				//bool isFeasibleEval2 = dfls_2->getIsFeasibleSolution();
				bool isFeasibleEval1 = dfls_1->getIsFeasibleSolution();

				//if(isFeasibleEval2 == true && isFeasibleEval1 == true)
				//{
					//evalSol = dfls_1->cw_evalSolution();
					//evalSol = dfls_1->evalSolution();
					//evalSol2 = dfls_2->evalSolution();
				//}

				if (dfls_1->getIsFeasibleSolution() == true)
				{
					controller->incSuccessAttempts();
					//evalSol = dfls_1->cw_evalSolution();
					evalSol = dfls_1->cw2_evalSolution();
					//printf("Showing CURRENT SOLUTION VALUE: %.3f\n", evalSol);
					//controller->printCtrl();

					//dfls_1->writeDecodedFrogLeapSolution(controller);
					//dfls_1->writeDecodedFrogLeapSolutionWithCoordinates(controller);

					if (evalSol < controller->getMinCostValue())
					{
						printf("New solution found \n");
						controller->incGlobalSearchImprovements();
						controller->setBestDecodedFrogLeapSolution(dfls_1);
						controller->setMinCostValue(evalSol);
						min_i = i;
						controller->setMinValueIteration(min_i);
						//apply local search
						//controller->applyLocalSearch();
						fls->exportClusterColToVRP(controller);
						fls->printFrogLeapSolutionArray(controller);
						fls->printFrogLeapSolutionWithSolutionData(controller);
						//controller->printCtrlSolutionData(controller);
						controller->cw_printCtrlSolutionData(controller);

						//controller->writeCtrl();

						//controller->getBestDecodedFrogLeapSolution();
						//dfls_1->writeDecodedFrogLeapSolutionWithCoordinates(controller);
					}
					else
					{
						//dfls_1->printFrogObj();
						delete dfls_1;
					}
				}
				else
				{
					evalSol = -1;
					//dfls_1->printFrogObj();
					controller->incFailAttempts();
					delete dfls_1;
				}
			}

			printf("Iteration Number i = %lld MinCostValue = %.3f CurrentValue = %.3f min_i = %lld\n", i, controller->getMinCostValue(), evalSol, min_i);
			delete fls;
		}
		else
		{
			dfls_1 = controller->loadTestCaseData(testCaseFileName);

			if (dfls_1->getIsFeasibleSolution() == true)
			{
				evalSol = dfls_1->evalSolution();
				//dfls_1->writeDecodedFrogLeapSolution(controller);
				//dfls_1->writeDecodedFrogLeapSolutionWithCoordinates(controller);
				if (evalSol < controller->getMinCostValue())
				{
					//printf("New solution found \n");
					controller->incGlobalSearchImprovements();
					controller->setBestDecodedFrogLeapSolution(dfls_1);
					controller->setMinCostValue(evalSol);

					//apply local search
					//controller->applyLocalSearch();					
					controller->printCtrl();
				}
				//dfls_1->printFrogObj();
				delete dfls_1;
			}
		}

		//delete fls;

		end_time = std::chrono::high_resolution_clock::now();
		time = end_time - start_time;
		execTime = std::chrono::duration_cast<std::chrono::milliseconds>(time).count();

		//controller->writeIterationInfo(i, evalSol);		
		i++;

		counterAux++;
	}

	printf("TOTAL ITERATION NUMBER %lld", i);

	DecodedFrogLeapSolution* bestSolution = controller->getBestDecodedFrogLeapSolution();
	//bestSolution->writeDecodedFrogLeapSolution(controller);
	//bestSolution->writeDecodedFrogLeapSolutionWithCoordinates(controller);

	controller->printCtrl();
	controller->closeOutPutFile();

	//delete g;
	//delete fls;
	delete controller;

	end_time = std::chrono::high_resolution_clock::now();
	time = end_time - start_time;

	std::cout << "FrogLeaping took " <<
		std::chrono::duration_cast<std::chrono::milliseconds>(time).count() << " to run.\n";

    return EXIT_SUCCESS;
}
// [END program]
