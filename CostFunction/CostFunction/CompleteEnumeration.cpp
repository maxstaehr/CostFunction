#include "CompleteEnumeration.h"
#include <iostream>
#include <fstream>
#include <assert.h>

using namespace std;
CompleteEnumeration::CompleteEnumeration(SAMPLE_PCL* sp, SAMPLE_ROTATIONS* sr, int nC, int nI,  int* nn_indices):SearchClass(sp,sr,nC,nI, nn_indices, NULL)
{
	//just a single camera for complete enumeration
	assert(nC == 1);
	c_i = 0;	
	c_vi = 0;
	l_vi = 0;
	this->prop = new float[sp->n*sr->nRotations];
	this->dist = new float[sp->n*sr->nRotations];
	this->weights = new int[sp->n*sr->nRotations];

	printf("staring complete enumearion for %d positions and %d rotation and %d samples", sp->n, sr->nRotations, sp->n*sr->nRotations);
}

void CompleteEnumeration::writeResultsToFile(unsigned long long* vec, int nOfCams, struct SAMPLE_POINTS_BUFFER* samplePoints)
{
}

bool CompleteEnumeration::iterate(int* pI, int* aI, float* p, float* d, int* w)
{
	//saving results of the last iteration when it is not the first	
	if(c_vi > 0)
	{
		memcpy(this->prop+l_vi, p, c_vi*sizeof(float));
		memcpy(this->dist+l_vi, d, c_vi*sizeof(float));		
		memcpy(this->weights+l_vi, w, c_vi*sizeof(int));		
		l_vi += c_vi;
	}
	

	if(!(c_i < sp->n*sr->nRotations))
		return false;

	unsigned int ite = 0;	
	while(c_i < sp->n*sr->nRotations && ite < nI)
	{

	

		pI[ite] = c_i/sr->nRotations;
		aI[ite] = c_i - pI[ite]*sr->nRotations;			
		

		ite++;
		c_i++;
		
	}
	c_vi = ite;
	return true;

}

CompleteEnumeration::~CompleteEnumeration(void)
{
	
	//ofstream outbin("completeEnumerationCosts.bin", ofstream::binary );
	//if (!outbin) std::cerr << "error";

	//outbin.write((char*)&sp->n,sizeof(int));
	//if (!outbin) std::cerr << "error";

	//outbin.write((char*)&sr->nRotations,sizeof(int));
	//if (!outbin) std::cerr << "error";

	//outbin.write((char*)prop, sp->n*sr->nRotations*sizeof(double));
	//if (!outbin) std::cerr << "error";

	//outbin.close();

	delete prop;
	delete weights;
	delete dist;

}
