#pragma once
#include "NearestNeighbour.h"


enum STATE{
	HC,
	NS
};

struct SOLUTION{
	int pcl;
	int angle;
	double costs;
	double currProp;
	double curT;
	int ite;
	double minEnergy;
	enum STATE state;	
	double globalMin;
};

struct COST_POINT{
	int pcl;
	int angle;
	double c;
};



struct COST
{
	struct COST_POINT m;
	struct COST_POINT p;
	struct COST_POINT c;
	struct COST_POINT c_t1;
	struct COST_POINT min;
	struct COST_POINT nm;
	struct COST_POINT np;
	NearestNeighbour::GRADIENT_DIM dir[6];
	unsigned char dim;
};


class SimulatedAnnealing
{
public:
	SimulatedAnnealing(int N, double T, double alpha, int Npcl, int Nangle);
	~SimulatedAnnealing(void);

	void initializeFirstRun(int* pclIndex, int* angleIndex);
	bool iterate(const int* const nn_indices, int* pclIndex, int* angleIndex, double* costs);
	void chooseRandomConfiguration(int* pclIndex, int* angleIndex);
	void printCurrentStatus();
	void findGlobalMinimum();
	void resetHasChanged(int i);
	bool hasHillClimbingFinished(int i);
	void writeResultsToFile(std::string pre, struct PCL *pcl, struct POSITIONS *pos);
	void writeEnergyResultsToFile(std::string pre);
	void setNewVector(int i);
	void addResult(void);
	void addSingleResult(int i);
	void writeAllResultToFile(std::string pre);

	double iterateSingle(const int* const nn_indices, int* pclIndex, int* angleIndex, double* costs, int i);

private:
	int NofE;
	int ite;
	double T;
	int Npcl;
	int Nangle;	
	double alpha;
	
	double minCostOverAll;

	double* minEnergy;
	bool* noChange;
	unsigned char* cDim;

	
	struct SOLUTION * solu;	
	struct SOLUTION globalMin;
	enum STATE* curStates;

	std::vector<SOLUTION> recordSol;
	
	std::vector<double*> recordedSolution;
	
	

	
	

};

