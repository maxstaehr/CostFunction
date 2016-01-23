#pragma once
#include "search.h"
#include "global.h"

class CFOBJECT_EXPORT SA :
	public Search
{
public:
	SA(const SampleCameraConfiguration& sampleConfig);
	~SA(void);
	bool nextIteration(double cost_m, double cost_p);



private:

	
	
};

