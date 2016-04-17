#include "InnerLoop.h"




InnerLoop::InnerLoop(int nCameras, CameraType* const types, KinChain kinchain,  SampleCameraConfiguration sampleconfiguration, PossibleConfigurations possibleConfigs)
{
	this->nCameras = nCameras;
	this->cameraSetup = new Camera*[this->nCameras];
	
	int nRays(0);
	for(int i=0; i<this->nCameras; i++)
	{
		this->cameraSetup[i] = new Camera(types[i]);
		nRays += cameraSetup[i]->getNRays();
	}

	this->ec = new EC(nRays);
	kinChain = kinchain;
	sampleConfig = sampleconfiguration;
	this->possibleConfig = possibleConfigs;
}



double InnerLoop::evaluatePositions(HomogeneTransformation* const cameraConfigurations)
{
	//setting kinChain by fiding the closest configuraton, that fits the actual configuraton
	//finding the relative 
	double averageProb = 0;
	for(int i=0; i<kinChain.getNPos(); i++)
	{
		//Updating the KinChain
		kinChain.setPosIndex(i);

		//Updating the CameraPos
		for(int camIndex = 0; camIndex<nCameras; camIndex++)
		{
			cameraSetup[camIndex]->updateCameraPos(sampleConfig.findNewCameraPos(kinChain.getRobotLinks(), cameraConfigurations[camIndex]));
		}
		//raytracing first all sensor/ except the own sensor
		for(int camIndex1 = 0; camIndex1<nCameras; camIndex1++)
		{
			for(int camIndex2 = 0; camIndex2<nCameras; camIndex2++)
			{
				if (camIndex1 != camIndex2)
				{
					cameraSetup[camIndex1]->raytrace(cameraSetup[camIndex2]->getLink());
				}
			}
			
		}

		//raytracing the robot
		for(int camIndex = 0; camIndex<nCameras; camIndex++)
		{
			for(int robotIndex = 0; robotIndex<kinChain.getNRobotLinks(); robotIndex++)
			{
				cameraSetup[camIndex]->raytrace(kinChain.getRobotLinks()[robotIndex]);			
			}
		}
		
		//raytracing the environment
		for(int camIndex = 0; camIndex<nCameras; camIndex++)
		{
			for(int envIndex = 0; envIndex<kinChain.getNEnvLinks(); envIndex++)
			{
				cameraSetup[camIndex]->raytrace(kinChain.getEnvLinks()[envIndex]);			
			}
		}

		//raytracing the human
		for(int camIndex = 0; camIndex<nCameras; camIndex++)
		{
			for(int humanIndex = 0; humanIndex<kinChain.getNHumanLinks(); humanIndex++)
			{
				cameraSetup[camIndex]->raytrace(kinChain.getHumanLinks()[humanIndex]);			
			}
		}

		ec->reset();
		for(int camIndex = 0; camIndex<nCameras; camIndex++)
		{
			ec->addDepthData(*cameraSetup[camIndex]);
		}
		ec->cluster();

		//take largest cluster, if one cluster exists
		if(ec->getNumOfClusters() > 0)
		{
			possibleConfig.findBestFit(ec->getLargestCluster());
			averageProb += possibleConfig.getMaxProb();
		}
				
	}	
	return averageProb/kinChain.getNPos();
}


InnerLoop::~InnerLoop(void)
{
		
	for(int i=0; i<this->nCameras; i++)
	{
		delete this->cameraSetup[i];
	}
	delete [] this->cameraSetup;
	delete ec;
	
}
