#pragma once

#include "global.h"
#include "HomogeneTransformation.h"
#include "SampleCameraConfiguration.h"
#include <vector>

class CFOBJECT_EXPORT Search
{
public:

	struct LOG_DATA{
		HomogeneTransformation* current;
		HomogeneTransformation* minus;
		HomogeneTransformation* plus;
		double costm;
		double costp;
		double cost;
		int* dim;
		int state;
	};

	enum EVAL_DIM
	{
		X,
		Y,
		Z,
		ROLL,
		PITCH,
		YAW
	};

	enum STATE
	{
		HC,
		RJ,
		FI
	};

	Search(SampleCameraConfiguration* sampleConfig, int n, bool lo);
	~Search(void);

	virtual bool nextIteration(double cost_m, double cost_p) = 0;
	virtual void setCurrentTransformation(HomogeneTransformation h, int i) = 0;

	HomogeneTransformation getCurrentTransformation(int i){return currentTrans[i];}
	HomogeneTransformation getNextEvalMinus(int i){return nextEvalMinus[i];}
	HomogeneTransformation getNextEvalPlus(int i){return nextEvalPlus[i];}
	double getCurrenCosts(){return currentCosts;}
	int getN(){return n;}
	void saveLog(const char* fileName);

protected:
	double currentCosts;

	SampleCameraConfiguration* sampleConfig;
	int n;
	bool log;
	std::vector<struct LOG_DATA> dataLog;

	void logging(double m, double p);

	HomogeneTransformation* currentTrans;
	HomogeneTransformation* nextEvalMinus;
	HomogeneTransformation* nextEvalPlus;

	STATE state;
	EVAL_DIM* currentDim;

};

