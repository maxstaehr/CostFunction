#pragma once

#include "global.h"
#include "HomogeneTransformation.h"

class CFOBJECT_EXPORT SampleCameraConfiguration
{
public:



	SampleCameraConfiguration( HomogeneTransformation * kinChainH,  HomogeneTransformation * relativeH, const int const *i, int n);
	void updateConfigurations(const float const* H);
	HomogeneTransformation findNN(HomogeneTransformation h, HomogeneTransformation::DIM_DIR dim);

	HomogeneTransformation getInitialH(int i);
	HomogeneTransformation getCurrentH(int i);
	HomogeneTransformation getRelativeH(int i);
	int getN(){return n;}
	const int const* getIndex(){return index;}

	~SampleCameraConfiguration(void);

private:
	HomogeneTransformation* initialH;
	HomogeneTransformation* currentH;
	HomogeneTransformation* relativH;
	int* index;
	int n;


};

