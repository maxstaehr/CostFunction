#include "SampleCameraConfiguration.h"

#include <iostream>
#include <string>
#include <assert.h>
SampleCameraConfiguration::SampleCameraConfiguration( HomogeneTransformation * kinChainH,  HomogeneTransformation * relativeH, const int const *in, int n):initialH(NULL), currentH(NULL), index(NULL), relativH(NULL)
{
	initialH = new HomogeneTransformation[NELEM_H*n];
	currentH = new HomogeneTransformation[NELEM_H*n];
	relativH = new HomogeneTransformation[NELEM_H*n];
	index = new int[n];
	memcpy(index, in, sizeof(int)*n);
	this->n = n;

	for(int i=0; i<n; i++)
	{
		relativH[i] = relativeH[i];
		initialH[i] = kinChainH[index[i]].mul(relativH[i]);
		currentH[i] = initialH[i];
	}
	
}

HomogeneTransformation SampleCameraConfiguration::getInitialH(int i)
{ 
	assert(i>= 0 && i<n);
	return initialH[i];
}
HomogeneTransformation SampleCameraConfiguration::getCurrentH(int i)
{
	assert(i>= 0 && i<n);
	return currentH[i];
}
HomogeneTransformation SampleCameraConfiguration::getRelativeH(int i)
{
	assert(i>= 0 && i<n);
	return relativH[i];
}


SampleCameraConfiguration::~SampleCameraConfiguration(void)
{
	delete[] initialH;
	delete[] currentH;
	delete[] relativH;
	delete[] index;
}


void SampleCameraConfiguration::updateConfigurations(const float const* H)
{

}

HomogeneTransformation SampleCameraConfiguration::findNN(HomogeneTransformation h, HomogeneTransformation::DIM_DIR dim)
{
	assert(n>0);
	HomogeneTransformation ret = h;
	float minDistDim = FLT_MAX;
	float minDistComplete = FLT_MAX;
	float distDim;
	float distComplete;
	for(int i=0; i<n; i++)
	{
		distDim = h.getDist(initialH[i],dim);
		if(distDim < minDistDim && ~h.isEqual(initialH[i]))
		{
			//new shortest Trans
			ret = initialH[i];
			minDistDim = distDim;
			minDistComplete = h.getDist(initialH[i]);
		}else if(distDim < FLT_MAX && distDim == minDistDim && ~h.isEqual(initialH[i])){
			distComplete =  h.getDist(initialH[i]);
			if(distComplete < minDistComplete)
			{
				minDistComplete = distComplete;
				ret = initialH[i];
			}
		}
	}
	return ret;
}