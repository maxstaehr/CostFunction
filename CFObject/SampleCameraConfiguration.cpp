#include "SampleCameraConfiguration.h"

#include <iostream>
#include <string>
#include <assert.h>
SampleCameraConfiguration::SampleCameraConfiguration( HomogeneTransformation * kinChainH,  HomogeneTransformation * relativeH, const int const *in, int nLink, int nRelative):initialH(NULL),
	currentH(NULL), index(NULL), relativH(NULL), nRelative(nRelative), nLink(nLink)
{
	initialH = new HomogeneTransformation[nRelative];
	currentH = new HomogeneTransformation[nRelative];
	relativH = new HomogeneTransformation[nRelative];
	index = new int[nRelative];
	memcpy(index, in, sizeof(int)*nRelative);
	

	for(int i=0; i<nRelative; i++)
	{
		relativH[i] = relativeH[i];
		initialH[i] = kinChainH[index[i]].mul(relativH[i]);
		currentH[i] = initialH[i];
	}
	
}

HomogeneTransformation SampleCameraConfiguration::getInitialH(int i)
{ 
	assert(i>= 0 && i<nRelative);
	return initialH[i];
}
HomogeneTransformation SampleCameraConfiguration::getCurrentH(int i)
{
	assert(i>= 0 && i<nRelative);
	return currentH[i];
}
HomogeneTransformation SampleCameraConfiguration::getRelativeH(int i)
{
	assert(i>= 0 && i<nRelative);
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
	assert(nRelative>0);
	HomogeneTransformation ret =	h;
	float minDistDim =				FLT_MAX;
	float minDistComplete =			FLT_MAX;
	float distDim;
	float distComplete;
	for(int i=0; i<nRelative; i++)
	{
		distDim = h.getDist(initialH[i],dim);
		distComplete =  h.getDist(initialH[i]);

		if(h.isEqual(initialH[i]) || distDim == FLT_MAX)
		{
			continue;
		}

		if(	distDim < minDistDim ||
			(distDim == minDistDim && distComplete < minDistComplete))
		{
			//new shortest Trans
			ret = initialH[i];
			minDistDim = distDim;
			minDistComplete = distComplete;
			
		}
	}
	return ret;
}