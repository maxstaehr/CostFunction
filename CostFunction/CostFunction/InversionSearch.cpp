#include "InversionSearch.h"
#include "CudaMem.h"
#include <assert.h>

#define MAX_SPACE (10.f)


InversionSearch::InversionSearch(SAMPLE_PCL* sp, SAMPLE_ROTATIONS* sr, int nC, int nI, int* const nn_indices):SearchClass(sp,sr,nC,nI,nn_indices, NULL)
{


	x_d[0] = 1.0f;
	y_d[0] = 1.0f;
	z_d[0] = -1.0f;

	//x_d[0] = 1.0f;
	//y_d[0] = -1.0f;
	//z_d[0] = -1.0f;

	x_d[1] = -1.0f;
	y_d[1] = 1.0f;
	z_d[1] = -1.0f;

	x_d[2] = -1.0f;
	y_d[2] = -1.0f;
	z_d[2] = -1.0f;

	x_d[3] = 1.0f;
	y_d[3] = -1.0f;
	z_d[3] = -1.0f;


	x_d[4] = 1.0f;
	y_d[4] = 1.0f;
	z_d[4] = 1.0f;

	x_d[5] = -1.0f;
	y_d[5] = 1.0f;
	z_d[5] = 1.0f;

	x_d[6] = -1.0f;
	y_d[6] = -1.0f;
	z_d[6] = 1.0f;

	x_d[7] = 1.0f;
	y_d[7] = -1.0f;
	z_d[7] = 1.0f;

	for(int i=0; i<SEARCH_DOF; i++)
	{
		maxPCLIndices[i] = 0;
		distPCLIndices[i] = FLT_MAX;
	}

	assert(nC == 1);
	c_i = 0;	
	c_vi = 0;
	l_vi = 0;
	this->prop = new float[SEARCH_DOF*sr->nRotations];
	this->dist = new float[SEARCH_DOF*sr->nRotations];
	this->weights = new int[SEARCH_DOF*sr->nRotations];

	printf("staring inversion search %d pos %d rot %d samples", sp->n, sr->nRotations, sp->n*sr->nRotations);

}

void InversionSearch::setInversionParamters(SAMPLE_POINTS_BUFFER* buffer)
{
	float* buf = new float[NUMELEM_H*buffer->n];
	CudaMem::cudaMemCpyReport(buf, buffer->d_H, buffer->n*NUMELEM_H*sizeof(float), cudaMemcpyDeviceToHost);

	float x_t, y_t, z_t, diffx, diffy, diffz, di;

	for(int pcli = 0; pcli<buffer->n; pcli++)
	{
		x_t = buf[pcli*NUMELEM_H+3];
		y_t = buf[pcli*NUMELEM_H+7];
		z_t = buf[pcli*NUMELEM_H+11];

		for(int i=0; i<SEARCH_DOF; i++)
		{
			diffx = x_d[i]*MAX_SPACE-x_t;
			diffy = y_d[i]*MAX_SPACE-y_t;
			diffz = z_d[i]*MAX_SPACE-z_t;
			di = sqrt(powf(diffx,2.0)+powf(diffz,2.0)+powf(diffy,2.0));

			if(di  < distPCLIndices[i])
			{
				distPCLIndices[i] = di;
				maxPCLIndices[i] = pcli;
			}
		}

	}
	delete buf;
}

	
bool InversionSearch::iterate(int* pI, int* aI, float* p, float* d, int* w)
{
	if(c_vi > 0)
	{
		memcpy(this->prop+l_vi, p, c_vi*sizeof(float));
		memcpy(this->dist+l_vi, d, c_vi*sizeof(float));		
		memcpy(this->weights+l_vi, w, c_vi*sizeof(int));		
		l_vi += c_vi;
	}
	

	if(!(c_i < SEARCH_DOF*sr->nRotations))
		return false;

	unsigned int ite = 0;	
	while(c_i < SEARCH_DOF*sr->nRotations && ite < nI)
	{

	
		bufferPi = c_i/sr->nRotations;
		pI[ite] = maxPCLIndices[bufferPi];
		aI[ite] = c_i - bufferPi*sr->nRotations;			
		

		ite++;
		c_i++;
		
	}
	c_vi = ite;

	

	return true;
}

void InversionSearch::buildProbabilityList()
{


}

void InversionSearch::writeResultsToFile(unsigned long long* vec, int nOfCams, struct SAMPLE_POINTS_BUFFER* samplePoints)
{
}


InversionSearch::~InversionSearch(void)
{
	delete this->prop;
	delete this->dist;
	delete this->weights;
}
