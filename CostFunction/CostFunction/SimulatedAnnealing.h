#pragma once
class SimulatedAnnealing
{
public:
	SimulatedAnnealing(int N, double T, double alpha, int Npcl, int Nangle);
	~SimulatedAnnealing(void);

	void initializeFirstRun(int* pclIndex, int* angleIndex);
	bool iterate(const int* const nn_indices, int* pclIndex, int* angleIndex, float* costs);
	void chooseRandomConfiguration(int* pclIndex, int* angleIndex);

private:
	int NofE;
	
	double T;
	int Npcl;
	int Nangle;
	double alpha;
	float* minEnergy;
	bool* isCurrentlyEvaluatingAnn;
	

};

