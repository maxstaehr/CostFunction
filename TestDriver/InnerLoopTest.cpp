#include "stdafx.h"

using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace	Microsoft::VisualStudio::TestTools::UnitTesting;

#include "../CFObject/InnerLoop.h"

namespace TestDriver
{
	[TestClass]
	public ref class InnerLoopTest
	{
	public: 

	

		[TestMethod]
		void TestConstructor()
		{
			CFIO::LINK humanPCL;
			const char* fileNameHuman = "C:\\ra\\GitHub\\CostFunction\\TestData\\human.bin";
			CFIO::loadLink(&humanPCL, fileNameHuman);

			CFIO::LINK envPCL;
			const char* fileNameEnv = "C:\\ra\\GitHub\\CostFunction\\TestData\\environment_scene1.bin";
			CFIO::loadLink(&envPCL, fileNameEnv);
			
			CFIO::LINK robotPCL;
			const char* fileNameRobot = "C:\\ra\\GitHub\\CostFunction\\TestData\\robot_short.bin2";			
			CFIO::loadLink(&robotPCL, fileNameRobot);

			CFIO::SAMPLE_POSITIONS samplePositions;
			const char* fileSamplePositions = "C:\\ra\\GitHub\\CostFunction\\TestData\\samplePositionsS4.bin";			
			CFIO::loadSamplePositions(&samplePositions, fileSamplePositions);

			CFIO::POSSIBLE_CAMERA_TYPES possibleTypes;
			const char* fileSampleCameraTypes = "C:\\ra\\GitHub\\CostFunction\\TestData\\sampleCamera.bin";
			CFIO::loadSampleCamera(&possibleTypes, fileSampleCameraTypes);

			CameraType** pType = new CameraType*[possibleTypes.nCameraTypes];
			CFIO::SAMPLE_CAMERA* pC;
			for(int i=0; i<possibleTypes.nCameraTypes; i++)
			{
				pType[i] = new CameraType(&(possibleTypes.possibleCameraTypes[i]));
			}

			CFIO::SAMPLE_HUMAN_POSITIONS sampleHumanPos;
			CFIO::loadSampleHumanPositions(&sampleHumanPos, "C:\\ra\\GitHub\\CostFunction\\TestData\\sampleFitting.bin");
			PossibleConfigurations possibleHumanConfigs(&sampleHumanPos);

			KinChain test(samplePositions.nP, samplePositions.nS);
			test.addHumanPCL(&humanPCL);
			test.addEnvPCL(&envPCL);
			test.addRobotPCL(&robotPCL);

			test.setRobotPos(samplePositions.qr);
			test.setHumanPos(samplePositions.qh);
			test.setEnvPos(samplePositions.qe);

			test.setPosIndex(0);

			CFIO::SAMPLE_CAMERA_POSITIONS cameraPos;
			const char* fileSampleCameraPositions = "C:\\ra\\GitHub\\CostFunction\\TestData\\samplePoints.bin";
			CFIO::loadCameraPos(&cameraPos, fileSampleCameraPositions);
			SampleCameraConfiguration sampleConfig(test.getRobotLinks(), &cameraPos, test.getNRobotLinks());

			CFIO::SAMPLE_CAMERA_POSITIONS cameraPosEnv;
			const char* fileSampleCameraPositionsEnv = "C:\\ra\\GitHub\\CostFunction\\TestData\\samplePointsEnv.bin";
			CFIO::loadCameraPos(&cameraPosEnv, fileSampleCameraPositionsEnv);
			SampleCameraConfiguration sampleConfigEnv(test.getEnvLinks(), &cameraPosEnv, test.getNEnvLinks());


			HomogeneTransformation initCameraPos[2];
			initCameraPos[0] = sampleConfig.getCurrentH(0);
			initCameraPos[1] = sampleConfigEnv.getCurrentH(1);
			double maxProb[2];
			CameraSetup cameraSetup(pType, possibleTypes.nCameraTypes);
			bool isEnv[2];
			isEnv[0] = true;
			isEnv[1] = false;
			cameraSetup.init(2, isEnv);
			
			InnerLoop innerloop(&cameraSetup,  test,  sampleConfig, sampleConfigEnv, possibleHumanConfigs,	true, "C:\\ra\\GitHub\\CostFunction\\TestData\\");
			innerloop.evaluatePositions(initCameraPos);
			
		}

	};
}
