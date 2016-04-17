#pragma once

#include "global.h"
#include "HomogeneTransformation.h"
#include "Link.h"
#include "CFIO.h"

class CFOBJECT_EXPORT SampleCameraConfiguration
{
public:



	SampleCameraConfiguration( Link * kinChainH,  HomogeneTransformation * relativeH, const int const *i, int nLink, int nRelative);
	SampleCameraConfiguration(Link* kinChainH, CFIO::SAMPLE_CAMERA_POSITIONS* samplePos, int nLink);
	SampleCameraConfiguration();
	SampleCameraConfiguration(const SampleCameraConfiguration& inst);
	void operator=(SampleCameraConfiguration& rhs );

	HomogeneTransformation findNN(HomogeneTransformation h, HomogeneTransformation::DIM_DIR dim);
	//bool findRelativeTransformationAndLink(HomogeneTransformation input,  HomogeneTransformation * kinChain, HomogeneTransformation& absolute, HomogeneTransformation& relative, int& linkIndex);
	HomogeneTransformation findNewCameraPos(Link * kinChain, HomogeneTransformation initialAbs);

	HomogeneTransformation getInitialH(int i);
	HomogeneTransformation getCurrentH(int i);
	HomogeneTransformation getRelativeH(int i);
	int getNLink(){return nLink;}
	int getNRelative(){return nRelative;}
	const int const* getIndex(){return index;}

	~SampleCameraConfiguration(void);
	void saveCurrentSzene(const char* fileName);

private:
	HomogeneTransformation* initialH;
	HomogeneTransformation* currentH;
	HomogeneTransformation* relativH;
	int* index;
	int nLink;
	int nRelative;


};

