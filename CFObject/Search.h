#pragma once

#include "global.h"
#include "HomogeneTransformation.h"
#include "SampleCameraConfiguration.h"

class CFOBJECT_EXPORT Search
{
public:
	Search(const SampleCameraConfiguration& sampleConfig);
	~Search(void);

	virtual bool nextIteration(double cost_m, double cost_p) = 0;

	void setCurrentTransformation(HomogeneTransformation h){currentTrans = h;}
	HomogeneTransformation getCurrentTransformation(){return currentTrans;}
	HomogeneTransformation getNextEvalMinus(){return nextEvalMinus;}
	HomogeneTransformation getNextEvalPlus(){return nextEvalPlus;}


	double getCurrenCosts(){return currentCosts;}

private:
	double currentCosts;
	const SampleCameraConfiguration& sampleConfig;
	HomogeneTransformation currentTrans;
	HomogeneTransformation nextEvalMinus;
	HomogeneTransformation nextEvalPlus;

};

