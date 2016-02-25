#pragma once
#include "search.h"
#include "global.h"

class CFOBJECT_EXPORT SA :
	public Search
{


public:

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

	SA(SampleCameraConfiguration& sampleConfig);
	~SA(void);
	bool nextIteration(double cost_m, double cost_p);
	void setCurrentTransformation(HomogeneTransformation h);
	enum STATE getState(){return state;}



private:

	void nextDim();
	bool localMinima[6];

	STATE state;
	EVAL_DIM currentDim;
	double T;
	double alpha;
	double minThres;

	void resetLocalMinima();
	bool isLocalMinimaReached();
	void setLocalMinima();
	void performRandomJump();
	void setNextTransformations();
	
	
};

