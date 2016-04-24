#pragma once
#include "search.h"
#include "global.h"

class CFOBJECT_EXPORT SA :
	public Search
{


public:



	SA(SampleCameraConfiguration* sampleConfig, int n, bool log);
	~SA(void);

	bool nextIteration(double cost_m, double cost_p);
	void setCurrentTransformation(HomogeneTransformation h, int i);
	enum STATE getState(){return state;}



private:

	void nextDim();
	bool* localMinima;


	double T;
	double alpha;
	double minThres;

	void resetLocalMinima();
	bool isLocalMinimaReached();
	void setLocalMinima();
	void performRandomJump();
	void setNextTransformations();
	void resetDim();
	int currentSampleIndex;
	
	
};

