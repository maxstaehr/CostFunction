#pragma once
#include "struct_definitions.h"
class SearchClass
{

public:
	SearchClass(SAMPLE_PCL* sp, SAMPLE_ROTATIONS* sr, int nC, int nI);
	virtual ~SearchClass(void);

	virtual bool iterate(int* pI, int* aI, float* prob) = 0;
	float* prop;

private:

protected:
	SAMPLE_PCL* sp;
	SAMPLE_ROTATIONS* sr;
	int nC;
	int nI;
	
};

