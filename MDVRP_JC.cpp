// MDVRP_JC.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "FrogLeapSolution.h"
#include "FrogLeapController.h"
#include "DecodedFrogLeapSolution.h"
//#include "FrogObjectCol.h"
//#include "IndexList.h"
#include "FeasibleSolution.h"
//#include "DistanceTable.h"
#include "Graph.h"
#include "FeasibleSolCol.h"
#include "DistanceTable.h"

//#include <cstdint>
//#define ABSL_NO_HAVE_INTRINSIC_INT128
//#include "ortools/constraint_solver/routing.h"
//#include "ortools/constraint_solver/routing_parameters.h"

#include <iostream>
#include <time.h>
#include <chrono>
#include <string>
#include <random>
#include <utility>
#include "MDVRP_JC.h"

using std::string;

using namespace std;

void setDepot(FrogLeapController *controller, Graph * g, int vertIdLabel, int capacity)
{
	int vertexIndex = g->getPositionVertexById(vertIdLabel);

	if (vertexIndex != -1)
	{
		controller->setAsDepot(vertexIndex, capacity);
	}
}

void setCustomer(FrogLeapController *controller, Graph * g, int vertIdLabel, int capacity)
{
	int vertexIndex = g->getPositionVertexById(vertIdLabel);

	if (vertexIndex != -1)
	{
		controller->setAsCustomer(vertexIndex, capacity);
	}
}


int main()
{

	auto start_time = std::chrono::high_resolution_clock::now();

	// create the graph given in above fugure 

	int V = 9;

	FrogLeapController * controller = new FrogLeapController();

	const char* fileName = "casog01.vrp";
	//char * fileName = "casog09.vrp";
	//char* fileName = "ClusterId_0_Input.vrp";
 	//char* fileName = "ClusterId_1_copy.vrp";
	//char* fileName = "casoch01.vrp";
	//char* fileName = "casoch02swp.txt";
	const char *testCaseFileName = "TestCase01Casog01 - copia.txt";

	
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

	DistanceTable * dt = controller->getDistanceTable();

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

		if( i == 6)
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

	DecodedFrogLeapSolution * bestSolution = controller->getBestDecodedFrogLeapSolution();
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
	return 0;
}


