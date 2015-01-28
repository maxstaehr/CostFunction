#pragma once
#include "SearchClass.h"

#define SEARCH_DOF 4

class InversionSearch :
	public SearchClass
{
public:
	


	InversionSearch(SAMPLE_PCL* sp, SAMPLE_ROTATIONS* sr, int nC, int nI, int* const nn_indices);
	bool iterate(int* pI, int* aI, float* prob, float* d, int* weights);

	void setInversionParamters(SAMPLE_POINTS_BUFFER* buffer);
	~InversionSearch(void);

	void buildProbabilityList();

	void writeResultsToFile(unsigned long long* vec, int nOfCams);

private:
	int maxPCLIndices[8]; 
	float distPCLIndices[8];
	float x_d[8];
	float y_d[8];
	float z_d[8];

	int c_i;
	int c_vi;
	int l_vi;

	int bufferPi;



	
};

