#pragma once
#include "struct_definitions.h"
#include "AngleGenerator.h"
class SearchClass
{

public:
	SearchClass(SAMPLE_PCL* sp, SAMPLE_ROTATIONS* sr, int nC, int nI, int* const nn_indices, AngleGenerator* ag);
	virtual ~SearchClass(void);

	virtual bool iterate(int* pI, int* aI, float* prob, float* d, int* weights) = 0;
	virtual void writeResultsToFile(unsigned long long* vec, int nOfCams) = 0;
	float* prop;
	float* dist;
	int* weights;

private:

protected:
	SAMPLE_PCL* sp;
	SAMPLE_ROTATIONS* sr;
	int nC;
	int nI; 
	int* nn_indices;
	AngleGenerator* aG;
	
};

