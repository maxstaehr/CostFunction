#pragma once
#include "searchclass.h"
class CompleteEnumeration :
	public SearchClass
{
public:
	CompleteEnumeration(SAMPLE_PCL* sp, SAMPLE_ROTATIONS* sr, int nC, int nI);

	bool iterate(int* pI, int* aI, float* prob);

	~CompleteEnumeration(void);

private:

	int c_i;
	int c_vi;
	int l_vi;
	
};


