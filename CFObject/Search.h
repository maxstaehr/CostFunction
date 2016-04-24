#pragma once

#include "global.h"
#include "HomogeneTransformation.h"
#include "SampleCameraConfiguration.h"

class CFOBJECT_EXPORT Search
{
public:
	Search(SampleCameraConfiguration* sampleConfig, int n);
	~Search(void);

	virtual bool nextIteration(double cost_m, double cost_p) = 0;
	virtual void setCurrentTransformation(HomogeneTransformation h, int i) = 0;

	HomogeneTransformation getCurrentTransformation(int i){return currentTrans[i];}
	HomogeneTransformation getNextEvalMinus(){return nextEvalMinus;}
	HomogeneTransformation getNextEvalPlus(){return nextEvalPlus;}
	double getCurrenCosts(){return currentCosts;}
	int getN(){return n;}

protected:
	double currentCosts;

	SampleCameraConfiguration* sampleConfig;
	int n;

	HomogeneTransformation* currentTrans;
	HomogeneTransformation nextEvalMinus;
	HomogeneTransformation nextEvalPlus;

};

