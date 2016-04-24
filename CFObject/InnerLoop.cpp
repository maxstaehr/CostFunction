#include "InnerLoop.h"
#include <sstream>
#include <string>




InnerLoop::InnerLoop(CameraSetup* pCameraSetup, KinChain kinchain,  SampleCameraConfiguration sampleconfiguration, SampleCameraConfiguration sampleConfigEnv, PossibleConfigurations possibleConfigs, bool dbg, const char * path):
debug(dbg) 
{	
	this->path = path;
	this->cameraSetup = pCameraSetup;
	this->kinChain = kinchain;
	this->sampleConfig = sampleconfiguration;
	this->sampleConfigEnv = sampleConfigEnv;
	this->possibleConfig = possibleConfigs;


	this->ec = new EC(cameraSetup->getNRays());	
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
		for(int camIndex = 0; camIndex<cameraSetup->getNCamera(); camIndex++)
		{
			if(cameraSetup->getIsEnv()[camIndex])
			{
				cameraSetup->getCamera(camIndex)->updateCameraPos(sampleConfigEnv.findNewCameraPos(kinChain.getEnvLinks(), cameraConfigurations[camIndex]));				
			}else{
				cameraSetup->getCamera(camIndex)->updateCameraPos(sampleConfig.findNewCameraPos(kinChain.getRobotLinks(), cameraConfigurations[camIndex]));
			}
		}
		//raytracing first all sensor/ except the own sensor
		for(int camIndex1 = 0; camIndex1<cameraSetup->getNCamera(); camIndex1++)
		{
			for(int camIndex2 = 0; camIndex2<cameraSetup->getNCamera(); camIndex2++)
			{
				if (camIndex1 != camIndex2)
				{
					cameraSetup->getCamera(camIndex1)->raytrace(cameraSetup->getCamera(camIndex2)->getLink());
				}
			}
			
		}

		cameraSetup->raytrace(kinChain);

		ec->reset();
		for(int camIndex = 0; camIndex<cameraSetup->getNCamera(); camIndex++)
		{
			ec->addDepthData(*(cameraSetup->getCamera(camIndex)));
		}
		ec->cluster();

		//take largest cluster, if one cluster exists
		Cluster cluster;
		if(ec->getNumOfClusters() > 0)
		{
			cluster = ec->getLargestCluster();			
		}
		possibleConfig.findBestFit(cluster);
		averageProb += possibleConfig.getMaxProb();
		//check if debug enabled
		if(debug)
		{

			std::string num = static_cast<std::ostringstream*>( &(std::ostringstream() << i) )->str();
			std::string postfix = ".bin";
			std::string fN;
			fN = path + "sampleHumanState_scene" + num + postfix;
			possibleConfig.saveState(fN.c_str());
			
			fN = path + "cluster_scene" + num + postfix;
			cluster.saveCluster(fN.c_str());

			fN = path + "cameraSetup_scene" + num + postfix;
			cameraSetup->saveCameraSetup(fN.c_str());

			fN = path + "currentSzene_scene" + num + postfix;
			kinChain.saveCurrentSzene(fN.c_str());

			fN = path + "currentSzeneSamplePoints_scene" + num + postfix;
			sampleConfig.saveCurrentSzene(fN.c_str());

			fN = path + "currentSzeneSamplePointsEnv_scene" + num + postfix;
			sampleConfigEnv.saveCurrentSzene(fN.c_str());

		}
				
	}	
	return averageProb/kinChain.getNPos();
	
}


InnerLoop::~InnerLoop(void)
{
		
	delete ec;
	
}
