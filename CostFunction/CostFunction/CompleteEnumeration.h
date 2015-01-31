#pragma once
#include "searchclass.h"
class CompleteEnumeration :
	public SearchClass
{
public:
	CompleteEnumeration(SAMPLE_PCL* sp, SAMPLE_ROTATIONS* sr, int nC, int nI,  int* nn_indices);

	bool iterate(int* pI, int* aI, float* prob, float* d, int* weights);
	void writeResultsToFile(unsigned long long* vec, int nOfCams, struct SAMPLE_POINTS_BUFFER* samplePoints);


	~CompleteEnumeration(void);

private:

	int c_i;
	int c_vi;
	int l_vi;


	
};


