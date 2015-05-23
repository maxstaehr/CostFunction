#pragma once
#include "struct_definitions.h"
#include "AngleGenerator.h"
class SearchClass
{

public:
	SearchClass(SAMPLE_PCL* sp, SAMPLE_ROTATIONS* sr, VALID_POS* pos, int nC, int nI, int* const nn_indices, AngleGenerator* ag);
	virtual ~SearchClass(void);

	virtual bool iterate(int* const pI, int*const aI, double const* const prob, float const *const d, int const *const weights) = 0;
	bool isPositionValid(int pI, int aI);
	virtual void writeResultsToFile(unsigned long long* vec, int nOfCams, struct SAMPLE_POINTS_BUFFER* samplePoints) = 0;
	double* prop;
	float* dist;
	int* weights;
	double maxProb;

	int bestPindex;
	int bestAngleIndex;
	int numberOfCalculations;

private:

protected:
	SAMPLE_PCL* sp;
	SAMPLE_ROTATIONS* sr;
	VALID_POS* vp;
	int nC;
	int nI; 
	int* nn_indices;
	AngleGenerator* aG;
	
};

