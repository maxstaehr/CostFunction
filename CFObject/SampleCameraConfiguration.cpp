#include "SampleCameraConfiguration.h"


#include <iostream>
#include <fstream>
#include <string>
#include <assert.h>

SampleCameraConfiguration::SampleCameraConfiguration( Link * kinChainH,  HomogeneTransformation * relativeH, const int const *in, int nLink, int nRelative):initialH(NULL),
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
		initialH[i] = kinChainH[index[i]].getH().mul(relativH[i]);
		currentH[i] = initialH[i];
	}
	
}
SampleCameraConfiguration::SampleCameraConfiguration(Link* kinChainH, CFIO::SAMPLE_CAMERA_POSITIONS* samplePos, int nLink):
initialH(NULL),	currentH(NULL), index(NULL), relativH(NULL), nRelative(samplePos->n), nLink(nLink)
{
	initialH = new HomogeneTransformation[nRelative];
	currentH = new HomogeneTransformation[nRelative];
	relativH = new HomogeneTransformation[nRelative];
	index = new int[nRelative];
	memcpy(index, samplePos->i, sizeof(int)*nRelative);
	
	for(int i=0; i<nRelative; i++)
	{
		relativH[i].setH(samplePos->h+i*NELEM_H);
		initialH[i] = kinChainH[index[i]].getH().mul(relativH[i]);
		currentH[i] = initialH[i];
	}
}

void SampleCameraConfiguration::saveCurrentSzene(const char* fileName)
{
	float* buffer = new float[nRelative*NELEM_H];
	for(int i=0; i<nRelative; i++)
	{
		memcpy(buffer+i*NELEM_H, currentH[i].getH(), NELEM_H*sizeof(float));
	}

	std::ofstream outbin(fileName, std::ofstream::binary );
	if (!outbin) std::cerr << "error";

	outbin.write((char*)&nRelative,sizeof(int));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)buffer,sizeof(float)*NELEM_H*nRelative);
	if (!outbin) std::cerr << "error";
	
	outbin.close();
	delete buffer;
}

SampleCameraConfiguration::SampleCameraConfiguration():initialH(NULL),currentH(NULL), index(NULL), relativH(NULL), nRelative(0), nLink(0)
{
}

SampleCameraConfiguration::SampleCameraConfiguration(const SampleCameraConfiguration& inst)
{


	nRelative = inst.nRelative;
	nLink = inst.nLink;
	initialH = new HomogeneTransformation[nRelative];
	currentH = new HomogeneTransformation[nRelative];
	relativH = new HomogeneTransformation[nRelative];
	index = new int[nRelative];


	
	for(int i=0; i<nRelative; i++)
	{
		index[i] = inst.index[i];
		relativH[i] = inst.relativH[i];
		initialH[i] = inst.initialH[i];
		currentH[i] = inst.currentH[i];
	}

}
void SampleCameraConfiguration::operator=(SampleCameraConfiguration& inst )
{
	if(initialH != NULL)
	{
		delete [] initialH;
		delete [] currentH;
		delete [] relativH;
		delete [] index;
	}

	nRelative = inst.nRelative;
	nLink = inst.nLink;
	initialH = new HomogeneTransformation[nRelative];
	currentH = new HomogeneTransformation[nRelative];
	relativH = new HomogeneTransformation[nRelative];
	index = new int[nRelative];


	
	for(int i=0; i<nRelative; i++)
	{
		index[i] = inst.index[i];
		relativH[i] = inst.relativH[i];
		initialH[i] = inst.initialH[i];
		currentH[i] = inst.currentH[i];
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

//bool SampleCameraConfiguration::findRelativeTransformationAndLink(HomogeneTransformation input,  HomogeneTransformation * kinChain, HomogeneTransformation& absolute, HomogeneTransformation& relative, int& linkIndex)
//{
//	
//	bool ret = false;
//	for(int i=0; i<nRelative; i++)
//	{		
//		if(input.isEqual(initialH[i]))
//		{
//			linkIndex = index[i];
//			absolute = kinChain[index[i]];			
//			relative = relativH[index[i]];
//			ret = true;
//			return ret;
//		}
//	}
//	return ret;
//}

HomogeneTransformation SampleCameraConfiguration::findNewCameraPos(Link * kinChainH, HomogeneTransformation input)
{
	HomogeneTransformation ret;
	for(int i=0; i<nRelative; i++)
	{		
		if(input.isEqual(initialH[i]))
		{
			ret = kinChainH[index[i]].getH().mul(relativH[i]);
			return ret;
		}
	}
	return ret;
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