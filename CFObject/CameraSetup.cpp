#include "CameraSetup.h"
#include <assert.h>
#include <fstream>


CameraSetup::CameraSetup(CameraType** types, int nTypes):pCameraTypes(types), nCameraTypes(nTypes), pCamera(NULL), nCamera(0)
{

}


CameraSetup::~CameraSetup(void)
{
	if(pCamera != NULL)
	{
		for(int i=0; i< nCamera; i++)
		{
			delete pCamera[i];
		}
		delete pCamera;
	}
}


void CameraSetup::saveCameraSetup(const char* fN)
{
	if(nCamera > 0)
	{
		std::ofstream outbin(fN, std::ofstream::binary );
		if (!outbin) std::cerr << "error";

		outbin.write((char*)&nCamera,sizeof(int));
		if (!outbin) std::cerr << "error";

		for(int i=0; i<nCamera; i++)
		{
			pCamera[i]->saveCurrentState(&outbin);
		}
		outbin.close();
	}
}

void CameraSetup::raytrace(KinChain& kinChain)
{
		//raytracing first all sensor/ except the own sensor
		for(int camIndex1 = 0; camIndex1<nCamera; camIndex1++)
		{
			for(int camIndex2 = 0; camIndex2<nCamera; camIndex2++)
			{
				if (camIndex1 != camIndex2)
				{
					pCamera[camIndex1]->raytrace(pCamera[camIndex2]->getLink());
				}
			}
			
		}

		//raytracing the robot
		for(int camIndex = 0; camIndex<nCamera; camIndex++)
		{
			for(int robotIndex = 0; robotIndex<kinChain.getNRobotLinks(); robotIndex++)
			{
				pCamera[camIndex]->raytrace(kinChain.getRobotLinks()[robotIndex]);			
			}
		}
		
		//raytracing the environment
		for(int camIndex = 0; camIndex<nCamera; camIndex++)
		{
			for(int envIndex = 0; envIndex<kinChain.getNEnvLinks(); envIndex++)
			{
				pCamera[camIndex]->raytrace(kinChain.getEnvLinks()[envIndex]);			
			}
		}

		//raytracing the human
		for(int camIndex = 0; camIndex<nCamera; camIndex++)
		{
			for(int humanIndex = 0; humanIndex<kinChain.getNHumanLinks(); humanIndex++)
			{
				pCamera[camIndex]->raytrace(kinChain.getHumanLinks()[humanIndex]);			
			}
		}


		//removing points from the robot
		for(int camIndex = 0; camIndex<nCamera; camIndex++)
		{
			for(int robotIndex = 0; robotIndex<kinChain.getNRobotLinks(); robotIndex++)
			{
				pCamera[camIndex]->checkinBB(kinChain.getRobotLinks()[robotIndex]);			
			}
		}
		
		//removing points from the environment
		for(int camIndex = 0; camIndex<nCamera; camIndex++)
		{
			for(int envIndex = 0; envIndex<kinChain.getNEnvLinks(); envIndex++)
			{
				pCamera[camIndex]->checkinBB(kinChain.getEnvLinks()[envIndex]);			
			}
		}
}

void CameraSetup::init(int nCamera)
{
	this->nCamera = nCamera;
	pCamera= new Camera*[nCamera];
	for(int i=0; i<nCamera; i++)
	{
		pCamera[i] = new Camera(*(pCameraTypes[0]));
	}
}
int CameraSetup::getNRays()
{
	int nRays = 0;
	for(int i=0; i<nCamera; i++)
	{
		nRays += pCamera[i]->getNRays();
	}
	return nRays;
}
bool CameraSetup::iterateCamera()
{
	return true;
}
	
Camera* CameraSetup::getCamera(int i)
{
	assert(i>= 0 && i < nCamera);
	return pCamera[i];
}
