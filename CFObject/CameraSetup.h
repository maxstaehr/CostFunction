#pragma once

#include "global.h"

#include "CameraType.h"
#include "Camera.h"
#include "KinChain.h"



class CFOBJECT_EXPORT CameraSetup
{
public:
	CameraSetup(CameraType** types, int nTypes);

	~CameraSetup(void);

	void init(int nCamera, bool* isEnv);
	bool iterateCamera();
	int getNCamera(){return nCamera;}
	Camera* getCamera(int i);
	void saveCameraSetup(const char* fN);
	void raytrace(KinChain& kinChain);
	int getNRays();
	const bool* getIsEnv(){return isEnv;}

private:
	int nCamera;
	bool* isEnv;
	Camera** pCamera;

	int nCameraTypes;
	CameraType** pCameraTypes;
};

