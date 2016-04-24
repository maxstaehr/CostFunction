#pragma once

#include "global.h"
#include "CameraType.h"
#include "Camera.h"
#include "KinChain.h"
#include "HomogeneTransformation.h"
#include "SampleCameraConfiguration.h"
#include "EC.h"
#include "PossibleConfigurations.h"
#include "CameraSetup.h"



class CFOBJECT_EXPORT InnerLoop
{
public: 
	InnerLoop(CameraSetup* pCameraSetup,  KinChain kinchain,  SampleCameraConfiguration sampleconfiguration,SampleCameraConfiguration sampleConfigEnv,  PossibleConfigurations possibleConfigs,
		bool dbg, const char* path);
	

	double evaluatePositions(HomogeneTransformation* const cameraConfigurations);

	~InnerLoop(void);

	

private:

	CameraSetup* cameraSetup;	
	EC* ec;

	KinChain kinChain;
	SampleCameraConfiguration sampleConfig;
	SampleCameraConfiguration sampleConfigEnv;
	PossibleConfigurations possibleConfig;
	bool debug;
	std::string path;
};

