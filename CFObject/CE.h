#pragma once

#include "global.h"
#include "Search.h"



class CFOBJECT_EXPORT CE:
	public Search
{
public:
	CE(SampleCameraConfiguration* sampleConfig, int n);
	~CE(void);

	virtual bool nextIteration(double cost_m, double cost_p) = 0;
	virtual void setCurrentTransformation(HomogeneTransformation h, int i) = 0;

private:
	int currentCompleteIndex;
	int maxIndex;

};

