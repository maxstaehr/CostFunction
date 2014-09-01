#include "SimulatedAnnealing.h"
#include "NearestNeighbour.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <float.h>

  


SimulatedAnnealing::SimulatedAnnealing(int NumerOfParrlelCalculation, double T, double alpha, int Npcl, int Nangle)
{

	NofE = (int)NumerOfParrlelCalculation/12;
	minEnergy = new float[NofE];
	isCurrentlyEvaluatingAnn = new bool[NofE];
	this->Nangle = Nangle;
	this->Npcl = Npcl;

	for(unsigned int i=0; i<NofE; i++)
	{
		isCurrentlyEvaluatingAnn[i] = true;
		minEnergy[i] = FLT_MAX;
	}

	this->T = T;
	this->alpha = alpha;
	
    time_t t;
    time(&t);
    srand((unsigned int)t);              /* Zufallsgenerator initialisieren */
}


SimulatedAnnealing::~SimulatedAnnealing(void)
{
	delete minEnergy;
	delete isCurrentlyEvaluatingAnn;
}

void SimulatedAnnealing::initializeFirstRun(int* pclIndex, int* angleIndex)
{
	for(unsigned int i=0; i<NofE; i++)
	{
		pclIndex[i] = rand() % Npcl;
		angleIndex[i] = rand() % Nangle;
	}
}

void SimulatedAnnealing::chooseRandomConfiguration(int* pclIndex, int* angleIndex)
{
	for(unsigned int i=0; i<12; i++)
	{
		pclIndex[i] = rand() % Npcl;
		angleIndex[i] = rand() % Nangle;
	}
}


bool SimulatedAnnealing::iterate(const int* const nn_indices, int* pclIndex, int* angleIndex, float* costs)
{
	const double e = 2.718281828;
	//iterating through all possible configuration
	for(unsigned int i=0; i<NofE; i++)
	{
		//find the lowest engergy for all 12 permutations
		float localMinE = FLT_MAX;
		unsigned int minIndex = 0;
		for(unsigned int j=0; j<12; j++)
		{
			if(costs[i*12+j] < localMinE)
			{
				localMinE = costs[i*12+j];
				minIndex = j;
			}
		}

		if(localMinE < minEnergy[i])
		{
			//no local minima found
			//do hill climbing
			minEnergy[i] = localMinE;
			//setting nearest neighbour for next hill climbing iteration

			NearestNeighbour::setNearestNeighbour(nn_indices, pclIndex[i*12+minIndex], angleIndex[i*12+minIndex], pclIndex+i*12, angleIndex+i*12);
			isCurrentlyEvaluatingAnn[i] = false;
		}else
		{
			//in simulated annealing
			//checking whether initialating of interpreting annealing
			if(isCurrentlyEvaluatingAnn[i])
			{
				//check if solution is acceppted
				if ((rand() / (double)RAND_MAX) <= pow(e, -(localMinE - minEnergy[i]) / T))
				{
					localMinE = localMinE;
					NearestNeighbour::setNearestNeighbour(nn_indices, pclIndex[i*12+minIndex], angleIndex[i*12+minIndex], pclIndex+i*12, angleIndex+i*12);
					isCurrentlyEvaluatingAnn[i] = false;
				}else
				{
					//choose new candidates
					chooseRandomConfiguration(pclIndex+i*12, angleIndex+i*12);
					isCurrentlyEvaluatingAnn[i] = true;
				}

			}else
			{
				//choose new candidates
				chooseRandomConfiguration(pclIndex+i*12, angleIndex+i*12);
				isCurrentlyEvaluatingAnn[i] = true;
			}
		}
	}
	if(T > 0.000008)
	{
		T *= alpha;
		return true;
	}else
	{
		return false;
	}
}