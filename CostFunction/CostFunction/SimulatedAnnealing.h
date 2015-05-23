#pragma once
#include "NearestNeighbour.h"
#include <time.h>
#include "SearchClass.h"
#include "AngleGenerator.h"
#include "struct_definitions.h"

//struct COST
//{
//	struct COST_POINT m;
//	struct COST_POINT p;
//	struct COST_POINT c;
//	struct COST_POINT c_t1;
//	struct COST_POINT min;
//	struct COST_POINT nm;
//	struct COST_POINT np;
//	NearestNeighbour::GRADIENT_DIM dir[6];
//	unsigned char dim;
//};

class SimulatedAnnealing : public SearchClass
{
public:
	//SimulatedAnnealing(int N, double T, double alpha, int Npcl, int Nangle, int nOfCams, int* NN, struct PCL* robot);


	SimulatedAnnealing(SAMPLE_PCL* sp, SAMPLE_ROTATIONS* sr, VALID_POS* vp, int nC, int nI, int* nn_indices, AngleGenerator* ag);
	~SimulatedAnnealing(void);

	
	//bool iterate2(const int* const nn_indices, int* pclIndex, int* angleIndex, double* costs);
	void chooseRandomConfiguration(int* pclIndex, int* angleIndex, int i);
	void printCurrentStatus();
	void findGlobalMinimum(int i);
	void resetHasChanged(int i);
	bool hasHillClimbingFinished(int i);
	void writeResultsToFile(std::string pre, struct PCL *pcl, struct POSITIONS *pos);
	void writeEnergyResultsToFile(std::string pre);
	void setNewVector(int i);
	void addResult(void);
	void addSingleResult(int i);
	void writeAllResultToFile(std::string pre);

	void setXYZ(int* const pclIndex, int*const angleIndex, int enegeryIndex, int dim, int cam);
	void setAngle(int* const pclIndex, int*const angleIndex, int enegeryIndex, int dim, int cam);
	
	
	
	double iterateSingle(const int* const nn_indices, int * const pclIndex, int * const angleIndex, double const *const costs, int i);
	void setCoolingPlan(double const *const costs);

	bool iterate(int* const pI, int*const aI, double const* const prob, float const *const d, int const *const weights);
	void writeResultsToFile(unsigned long long* vec, int nOfCams, struct SAMPLE_POINTS_BUFFER* samplePoints);
	void printMinPositionToFile(std::string pre, struct SAMPLE_POINTS_BUFFER* samplePoints);
	
	
	void writeMinCostToFile(unsigned long long* vec);	
	bool areAllPositionsValid(int* pI, int* aI);
	bool areAllPositionsValidAssert(int* pI, int* aI);
	bool isConfigurationValid(int* pI, int i, int sign);
	bool isConfigurationValidAssert(int* pI, int i, int sign);
	int getIndex(int enegeryIndex, int sign, int cam);

private:
	int NofE;
	int ite;
	double T;
	int Npcl;
	int Nangle;	
	int nOfCams;
	double alpha;
	int DOF;
	int* hasImproved;
	int* nsRuns;


	
	
	struct SAMPLE_PCL* robot;
	int angleIndexRandomRange_roll;
	int angleIndexRandomRange_pitch;
	int angleIndexRandomRange_yaw;
	float minCostOverAll;

	int* pclIndex_t1;
	int* angleIndex_t1;
	double* minEnergy_t1;


	double* minEnergy;
	bool* noChange;
	unsigned char* cDim;
	static const double e;

	bool firstEvaluation;
	bool firstIteration;
	time_t start;
	double loopTime;
	double maxTime;
	int NofIteration;
	int maxIteration;
	float min_threshold;

	
	struct SOLUTION * solu;	
	struct SOLUTION globalMin;
	enum STATE* curStates;

	std::vector<SOLUTION> recordSol;
	
	std::vector<float*> recordedSolution;
	

	double neighbourRadius;
	double neighbourAngle;


	

	
	

};

