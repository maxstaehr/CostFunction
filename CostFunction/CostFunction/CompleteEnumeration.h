#pragma once
#include "searchclass.h"
class CompleteEnumeration :
	public SearchClass
{
public:
	CompleteEnumeration(SAMPLE_PCL* sp, SAMPLE_ROTATIONS* sr, VALID_POS* vp, int nC, int nI,  int* nn_indices);

	bool iterate(int* const pI, int*const aI, double const* const prob, float const *const d, int const *const weights);
	void writeResultsToFile(unsigned long long* vec, int nOfCams, struct SAMPLE_POINTS_BUFFER* samplePoints);


	~CompleteEnumeration(void);
	

private:

	int c_i;
	int c_vi;
	int l_vi;




	
};


