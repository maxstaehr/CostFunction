#pragma once

#include "global.h"
#include "HomogeneTransformation.h"
#include "SampleCameraConfiguration.h"

class CFOBJECT_EXPORT Search
{
public:
	Search(SampleCameraConfiguration& sampleConfig);
	~Search(void);

	virtual bool nextIteration(double cost_m, double cost_p) = 0;

	virtual void setCurrentTransformation(HomogeneTransformation h) = 0;
	HomogeneTransformation getCurrentTransformation(){return currentTrans;}
	HomogeneTransformation getNextEvalMinus(){return nextEvalMinus;}
	HomogeneTransformation getNextEvalPlus(){return nextEvalPlus;}


	double getCurrenCosts(){return currentCosts;}

protected:
	double currentCosts;
	SampleCameraConfiguration& sampleConfig;
	HomogeneTransformation currentTrans;
	HomogeneTransformation nextEvalMinus;
	HomogeneTransformation nextEvalPlus;

};

