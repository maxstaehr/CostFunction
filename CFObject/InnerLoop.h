#pragma once

#include "global.h"
#include "CameraType.h"
#include "Camera.h"
#include "KinChain.h"
#include "HomogeneTransformation.h"
#include "SampleCameraConfiguration.h"
#include "EC.h"
#include "PossibleConfigurations.h"



class CFOBJECT_EXPORT InnerLoop
{
public: 
	InnerLoop(int nCameras, CameraType* const types, KinChain kinchain,  SampleCameraConfiguration sampleconfiguration, PossibleConfigurations possibleConfigs);
	

	double evaluatePositions(HomogeneTransformation* const cameraConfigurations);

	~InnerLoop(void);

	

private:

	Camera** cameraSetup;
	int nCameras;


	KinChain kinChain;
	SampleCameraConfiguration sampleConfig;
	PossibleConfigurations possibleConfig;

	EC* ec;
	

};

