#pragma once

#include "global.h"
#include "Search.h"
#include <vector>


class CFOBJECT_EXPORT CE:
	public Search
{
public:
	CE(SampleCameraConfiguration* sampleConfig, int n, bool log);
	~CE(void);

	bool nextIteration(double cost_m, double cost_p);
	void setCurrentTransformation(HomogeneTransformation h, int i);

	int getIndices(int m, int n);
	int getM() {return indices.size();}

private:
	int currentCompleteIndex;
	int maxIndex;
	
	std::vector<std::vector<int>*> indices;



};

