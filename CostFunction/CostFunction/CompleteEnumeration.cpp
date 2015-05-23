#include "CompleteEnumeration.h"
#include <iostream>
#include <fstream>
#include <assert.h>

using namespace std;
CompleteEnumeration::CompleteEnumeration(SAMPLE_PCL* sp, SAMPLE_ROTATIONS* sr, VALID_POS* vp,  int nC, int nI,  int* nn_indices):SearchClass(sp,sr,vp,nC,nI, nn_indices, NULL)
{
	//just a single camera for complete enumeration
	assert(nC == 1);
	c_i = 0;	
	c_vi = 0;
	l_vi = 0;
	this->prop = new double[sp->n*sr->nRotations];
	this->dist = new float[sp->n*sr->nRotations];
	this->weights = new int[sp->n*sr->nRotations];

	printf("staring complete enumearion for %d positions and %d rotation and %d samples", sp->n, sr->nRotations, sp->n*sr->nRotations);
}

void CompleteEnumeration::writeResultsToFile(unsigned long long* vec, int nOfCams, struct SAMPLE_POINTS_BUFFER* samplePoints)
{
}

bool CompleteEnumeration::iterate(int* const pI, int*const aI, double const* const p, float const *const d, int const *const weights)
{
	//saving results of the last iteration when it is not the first	
	//if(c_vi > 0)
	//{
	//	memcpy(this->prop+l_vi, p, c_vi*sizeof(double));
	//	memcpy(this->dist+l_vi, d, c_vi*sizeof(float));		
	//	memcpy(this->weights+l_vi, w, c_vi*sizeof(int));	
	//	for(int i=0; i<nI; i++)
	//	{
	//		if(!isPositionValid(pI[i], aI[i]))
	//		{
	//			this->prop[l_vi+i] = 0.0;

	//		
	//		}
	//	}

	//	l_vi += c_vi;
	//}

	//checking for best result
	for(int i=0; i<nI; i++)
	{
		if(p[i] > maxProb)
		{
			maxProb = p[i];
			bestPindex = pI[i];
			bestAngleIndex = aI[i];
			printf("%.5lf\n", maxProb);
		}
	}
	

	if(!(c_i < sp->n*sr->nRotations))	
		return false;
	

	unsigned int ite = 0;	
	int pIndex,aIndex;
	while(c_i < sp->n*sr->nRotations && ite < nI)
	{

		////if position is not valid just continue
		pIndex = c_i/sr->nRotations;
		aIndex = c_i - pIndex*sr->nRotations;
		//check last position was valid or not
		if(isPositionValid(pIndex,aIndex))
		{
			
			pI[ite] = pIndex;
			aI[ite] = aIndex;			
			ite++;
			numberOfCalculations++;
		}
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
