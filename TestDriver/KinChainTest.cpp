#include "stdafx.h"

using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace	Microsoft::VisualStudio::TestTools::UnitTesting;

#include "../CFObject/KinChain.h"
#include "../CFObject/CFIO.h"
#include "../CFObject/SampleCameraConfiguration.h"
#include "../CFObject/CameraSetup.h"
#include "../CFObject/EC.h"
#include "../CFObject/Cluster.h"
#include "../CFObject/PossibleConfigurations.h"
#include <cstring>


namespace TestDriver
{
	[TestClass]
	public ref class KinChainTest
	{
	public: 


		[TestMethod]
		void TestConstructor()
		{
			int nPos = 100;
			KinChain test(nPos, 1);

			Assert::AreEqual(nPos, test.getNPos());

			for(int i=0; i<nPos; i++)
			{
				for(int h=0; h<NELEM_H*DOF_H; h++)
				{
					Assert::AreEqual(0.0f, test.getHumanPos()[i*DOF_H+h]);
				}

				for(int r=0; r<NELEM_H*DOF_R; r++)
				{
					Assert::AreEqual(0.0f, test.getRobotPos()[i*DOF_R+r]);
				}

				for(int e=0; e<NELEM_H*DOF_E; e++)
				{
					Assert::AreEqual(0.0f, test.getRobotPos()[i*DOF_E+e]);
				}
			}

		}

		[TestMethod]
		void TestSetHumanPos()
		{
			int nPos = 100;
			KinChain test(nPos, 1);

			float *temp1 = new float[NELEM_H*DOF_H*nPos];
			float pattern[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
			for(int pos = 0; pos<nPos; pos++)
			{
				for(int h=0; h<DOF_H; h++)
				{
					for(int patteri=0; patteri<NELEM_H; patteri++)
					{
						temp1[pos*DOF_H*NELEM_H + h*NELEM_H + patteri] = pattern[patteri];
					}
				}
			}

			test.setHumanPos(temp1);
			for(int pos = 0; pos<nPos; pos++)
			{
				for(int h=0; h<DOF_H; h++)
				{
					for(int patteri=0; patteri<NELEM_H; patteri++)
					{						
						Assert::AreEqual(test.getHumanPos()[pos*DOF_H*NELEM_H + h*NELEM_H + patteri], pattern[patteri]);
					}
				}
			}
			delete temp1;
		}


		[TestMethod]
		void TestSetEnvPos()
		{
			int nPos = 100;
			KinChain test(nPos, 1);

			float *temp1 = new float[NELEM_H*DOF_E*nPos];
			float pattern[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
			for(int pos = 0; pos<nPos; pos++)
			{
				for(int h=0; h<DOF_E; h++)
				{
					for(int patteri=0; patteri<NELEM_H; patteri++)
					{
						temp1[pos*DOF_E*NELEM_H + h*NELEM_H + patteri] = pattern[patteri];
					}
				}
			}

			test.setEnvPos(temp1);
			for(int pos = 0; pos<nPos; pos++)
			{
				for(int h=0; h<DOF_E; h++)
				{
					for(int patteri=0; patteri<NELEM_H; patteri++)
					{						
						Assert::AreEqual(test.getEnvPos()[pos*DOF_E*NELEM_H + h*NELEM_H + patteri], pattern[patteri]);
					}
				}
			}
			delete temp1;
		}


		[TestMethod]
		void TestSetRobotPos()
		{
			int nPos = 100;
			KinChain test(nPos, 1);

			float *temp1 = new float[NELEM_H*DOF_R*nPos];
			float pattern[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
			for(int pos = 0; pos<nPos; pos++)
			{
				for(int h=0; h<DOF_R; h++)
				{
					for(int patteri=0; patteri<NELEM_H; patteri++)
					{
						temp1[pos*DOF_R*NELEM_H + h*NELEM_H + patteri] = pattern[patteri];
					}
				}
			}

			test.setRobotPos(temp1);
			for(int pos = 0; pos<nPos; pos++)
			{
				for(int h=0; h<DOF_R; h++)
				{
					for(int patteri=0; patteri<NELEM_H; patteri++)
					{						
						Assert::AreEqual(test.getRobotPos()[pos*DOF_R*NELEM_H + h*NELEM_H + patteri], pattern[patteri]);
					}
				}
			}
			delete temp1;
		}


		[TestMethod]
		void TestSetRobotPosIndex()
		{
			int nPos = 100;
			KinChain test(nPos, 1);

			float *temp1 = new float[NELEM_H*DOF_R*nPos];			
			for(int pos = 0; pos<nPos; pos++)
			{
				for(int h=0; h<DOF_R; h++)
				{
					for(int patteri=0; patteri<NELEM_H; patteri++)
					{
						temp1[pos*DOF_R*NELEM_H + h*NELEM_H + patteri] = pos*DOF_R*NELEM_H + h*NELEM_H + patteri;
					}
				}
			}

			test.setRobotPos(temp1);
			int pos = 5;
			test.setPosIndex(pos);			
			Link link;

			for(int h=0; h<DOF_R; h++)
			{
				link = test.getRobotLinks()[h];
				for(int patteri=0; patteri<NELEM_H; patteri++)
				{						
					Assert::AreEqual((float)pos*DOF_R*NELEM_H + h*NELEM_H + patteri,
						link.getH().getH()[patteri]);
				}
			}
			delete temp1;
		}


		[TestMethod]
		void TestSetHumanPosIndex()
		{
			int nPos = 100;
			KinChain test(nPos, 1);

			float *temp1 = new float[NELEM_H*DOF_H*nPos];			
			for(int pos = 0; pos<nPos; pos++)
			{
				for(int h=0; h<DOF_H; h++)
				{
					for(int patteri=0; patteri<NELEM_H; patteri++)
					{
						temp1[pos*DOF_H*NELEM_H + h*NELEM_H + patteri] = pos*DOF_H*NELEM_H + h*NELEM_H + patteri;
					}
				}
			}

			test.setHumanPos(temp1);
			int pos = 5;
			test.setPosIndex(pos);			
			Link link;

			for(int h=0; h<DOF_H; h++)
			{
				link = test.getHumanLinks()[h];
				for(int patteri=0; patteri<NELEM_H; patteri++)
				{						
					Assert::AreEqual((float)pos*DOF_H*NELEM_H + h*NELEM_H + patteri,
						link.getH().getH()[patteri]);
				}
			}
			delete temp1;
		}

		[TestMethod]
		void TestSetEnvPosIndex()
		{
			int nPos = 100;
			KinChain test(nPos, 1);

			float *temp1 = new float[NELEM_H*DOF_E*nPos];			
			for(int pos = 0; pos<nPos; pos++)
			{
				for(int h=0; h<DOF_E; h++)
				{
					for(int patteri=0; patteri<NELEM_H; patteri++)
					{
						temp1[pos*DOF_E*NELEM_H + h*NELEM_H + patteri] = pos*DOF_E*NELEM_H + h*NELEM_H + patteri;
					}
				}
			}

			test.setEnvPos(temp1);
			int pos = 5;
			test.setPosIndex(pos);			
			Link link;

			for(int h=0; h<DOF_E; h++)
			{
				link = test.getEnvLinks()[h];
				for(int patteri=0; patteri<NELEM_H; patteri++)
				{						
					Assert::AreEqual((float)pos*DOF_E*NELEM_H + h*NELEM_H + patteri,
						link.getH().getH()[patteri]);
				}
			}
			delete temp1;
		}


		[TestMethod]
		void TestCopy()
		{
			int nPos = 100;
			KinChain test1(nPos, 1);

			float *temp1 = new float[NELEM_H*DOF_E*nPos];
			float *temp2 = new float[NELEM_H*DOF_R*nPos];
			float *temp3 = new float[NELEM_H*DOF_H*nPos];
			for(int pos = 0; pos<nPos; pos++)
			{
				for(int h=0; h<DOF_E; h++)
				{
					for(int patteri=0; patteri<NELEM_H; patteri++)
					{
						temp1[pos*DOF_E*NELEM_H + h*NELEM_H + patteri] = pos*DOF_E*NELEM_H + h*NELEM_H + patteri;
					}
				}
			}

			for(int pos = 0; pos<nPos; pos++)
			{
				for(int h=0; h<DOF_R; h++)
				{
					for(int patteri=0; patteri<NELEM_H; patteri++)
					{
						temp2[pos*DOF_R*NELEM_H + h*NELEM_H + patteri] = pos*DOF_R*NELEM_H + h*NELEM_H + patteri;
					}
				}
			}

			for(int pos = 0; pos<nPos; pos++)
			{
				for(int h=0; h<DOF_H; h++)
				{
					for(int patteri=0; patteri<NELEM_H; patteri++)
					{
						temp3[pos*DOF_H*NELEM_H + h*NELEM_H + patteri] = pos*DOF_H*NELEM_H + h*NELEM_H + patteri;
					}
				}
			}

			test1.setEnvPos(temp1);
			test1.setRobotPos(temp2);
			test1.setHumanPos(temp3);
			KinChain test2 = test1;


			
			test1.setPosIndex(0);			
			

			for(int h=0; h<NELEM_H*DOF_E*nPos; h++)
			{
				Assert::AreEqual(test1.getEnvPos()[h], temp1[h], 1e-5f);
				Assert::AreEqual(test2.getEnvPos()[h], temp1[h], 1e-5f);
			}

			for(int h=0; h<NELEM_H*DOF_R*nPos; h++)
			{
				Assert::AreEqual(test1.getRobotPos()[h], temp2[h], 1e-5f);
				Assert::AreEqual(test2.getRobotPos()[h], temp2[h], 1e-5f);
			}
			for(int h=0; h<NELEM_H*DOF_H*nPos; h++)
			{
				Assert::AreEqual(test1.getHumanPos()[h], temp3[h], 1e-5f);
				Assert::AreEqual(test2.getHumanPos()[h], temp3[h], 1e-5f);
			}

			delete temp1;
			delete temp2;
			delete temp3;
		}

		[TestMethod]
		void TestEqual()
		{
			int nPos = 100;
			KinChain test1(nPos, 1);

			float *temp1 = new float[NELEM_H*DOF_E*nPos];
			float *temp2 = new float[NELEM_H*DOF_R*nPos];
			float *temp3 = new float[NELEM_H*DOF_H*nPos];
			for(int pos = 0; pos<nPos; pos++)
			{
				for(int h=0; h<DOF_E; h++)
				{
					for(int patteri=0; patteri<NELEM_H; patteri++)
					{
						temp1[pos*DOF_E*NELEM_H + h*NELEM_H + patteri] = pos*DOF_E*NELEM_H + h*NELEM_H + patteri;
					}
				}
			}

			for(int pos = 0; pos<nPos; pos++)
			{
				for(int h=0; h<DOF_R; h++)
				{
					for(int patteri=0; patteri<NELEM_H; patteri++)
					{
						temp2[pos*DOF_R*NELEM_H + h*NELEM_H + patteri] = pos*DOF_R*NELEM_H + h*NELEM_H + patteri;
					}
				}
			}

			for(int pos = 0; pos<nPos; pos++)
			{
				for(int h=0; h<DOF_H; h++)
				{
					for(int patteri=0; patteri<NELEM_H; patteri++)
					{
						temp3[pos*DOF_H*NELEM_H + h*NELEM_H + patteri] = pos*DOF_H*NELEM_H + h*NELEM_H + patteri;
					}
				}
			}

			test1.setEnvPos(temp1);
			test1.setRobotPos(temp2);
			test1.setHumanPos(temp3);
			KinChain test2(nPos, 1);
			test2 = test1;
			Link link;


			
			test1.setPosIndex(0);			
			

			for(int h=0; h<NELEM_H*DOF_E*nPos; h++)
			{
				Assert::AreEqual(test1.getEnvPos()[h], temp1[h], 1e-5f);
				Assert::AreEqual(test2.getEnvPos()[h], temp1[h], 1e-5f);

			}


			for(int pos=0; pos < nPos; pos++)
			{
				test1.setPosIndex(pos);
				test2 = test1;
				for(int h=0; h<DOF_E; h++)
				{
					link = test1.getEnvLinks()[h];
					for(int patteri=0; patteri<NELEM_H; patteri++)
					{						
						Assert::AreEqual((float)pos*DOF_E*NELEM_H + h*NELEM_H + patteri,
							link.getH().getH()[patteri]);
					}

					link = test2.getEnvLinks()[h];
					for(int patteri=0; patteri<NELEM_H; patteri++)
					{						
						Assert::AreEqual((float)pos*DOF_E*NELEM_H + h*NELEM_H + patteri,
							link.getH().getH()[patteri]);
					}
				}
			}

			for(int h=0; h<NELEM_H*DOF_R*nPos; h++)
			{
				Assert::AreEqual(test1.getRobotPos()[h], temp2[h], 1e-5f);
				Assert::AreEqual(test2.getRobotPos()[h], temp2[h], 1e-5f);
			}

			for(int pos=0; pos < nPos; pos++)
			{
				test1.setPosIndex(pos);
				test2 = test1;
				for(int h=0; h<DOF_R; h++)
				{
					link = test1.getRobotLinks()[h];
					for(int patteri=0; patteri<NELEM_H; patteri++)
					{						
						Assert::AreEqual((float)pos*DOF_R*NELEM_H + h*NELEM_H + patteri,
							link.getH().getH()[patteri]);
					}

					link = test2.getRobotLinks()[h];
					for(int patteri=0; patteri<NELEM_H; patteri++)
					{						
						Assert::AreEqual((float)pos*DOF_R*NELEM_H + h*NELEM_H + patteri,
							link.getH().getH()[patteri]);
					}
				}
			}

			for(int h=0; h<NELEM_H*DOF_H*nPos; h++)
			{
				Assert::AreEqual(test1.getHumanPos()[h], temp3[h], 1e-5f);
				Assert::AreEqual(test2.getHumanPos()[h], temp3[h], 1e-5f);
			}

			for(int pos=0; pos < nPos; pos++)
			{
				test1.setPosIndex(pos);
				test2 = test1;
				for(int h=0; h<DOF_H; h++)
				{
					link = test1.getHumanLinks()[h];
					for(int patteri=0; patteri<NELEM_H; patteri++)
					{						
						Assert::AreEqual((float)pos*DOF_H*NELEM_H + h*NELEM_H + patteri,
							link.getH().getH()[patteri]);
					}

					link = test2.getHumanLinks()[h];
					for(int patteri=0; patteri<NELEM_H; patteri++)
					{						
						Assert::AreEqual((float)pos*DOF_H*NELEM_H + h*NELEM_H + patteri,
							link.getH().getH()[patteri]);
					}
				}
			}

			delete temp1;
			delete temp2;
			delete temp3;
		}

		[TestMethod]
		void TestSaveSzene()
		{
			CFIO::LINK humanPCL;
			const char* fileNameHuman = "C:\\ra\\GitHub\\CostFunction\\TestData\\human.bin";
			CFIO::loadLink(&humanPCL, fileNameHuman);

			CFIO::LINK envPCL;
			const char* fileNameEnv = "C:\\ra\\GitHub\\CostFunction\\TestData\\environment.bin";
			CFIO::loadLink(&envPCL, fileNameEnv);


			
			CFIO::LINK robotPCL;
			const char* fileNameRobot = "C:\\ra\\GitHub\\CostFunction\\TestData\\robot_short.bin2";
			//const char* fileNameRobot = "C:\\ra\\GitHub\\CostFunction\\TestData\\robotpcl_q0.bin";			
			CFIO::loadLink(&robotPCL, fileNameRobot);

			CFIO::SAMPLE_POSITIONS samplePositions;
			const char* fileSamplePositions = "C:\\ra\\GitHub\\CostFunction\\TestData\\samplePositionsS1.bin";			
			CFIO::loadSamplePositions(&samplePositions, fileSamplePositions);



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

			CFIO::POSSIBLE_CAMERA_TYPES possibleTypes;
			const char* fileSampleCameraTypes = "C:\\ra\\GitHub\\CostFunction\\TestData\\sampleCamera.bin";
			CFIO::loadSampleCamera(&possibleTypes, fileSampleCameraTypes);

			CFIO::SAMPLE_HUMAN_POSITIONS sampleHumanPos;
			CFIO::loadSampleHumanPositions(&sampleHumanPos, "C:\\ra\\GitHub\\CostFunction\\TestData\\sampleFitting.bin");
			PossibleConfigurations possibleHumanConfigs(&sampleHumanPos);

			CameraType** pType = new CameraType*[possibleTypes.nCameraTypes];
			CFIO::SAMPLE_CAMERA* pC;
			for(int i=0; i<possibleTypes.nCameraTypes; i++)
			{
				pType[i] = new CameraType(&(possibleTypes.possibleCameraTypes[i]));
			}


			CameraSetup cameraSetup(pType, possibleTypes.nCameraTypes);
			cameraSetup.init(2);
			cameraSetup.getCamera(0)->updateCameraPos(sampleConfig.getCurrentH(0));
			cameraSetup.getCamera(1)->updateCameraPos(sampleConfig.getCurrentH(1));
			cameraSetup.raytrace(test);

			EC ec(cameraSetup.getNRays());
			ec.reset();
			for(int camIndex = 0; camIndex<cameraSetup.getNCamera(); camIndex++)
			{
				ec.addDepthData(*cameraSetup.getCamera(camIndex));
			}
			ec.cluster();
			Cluster	cluster = ec.getLargestCluster();
			cluster.calculateCentroid();



			possibleHumanConfigs.findBestFit(cluster);
			double maxProb = possibleHumanConfigs.getMaxProb();
			

			

			possibleHumanConfigs.saveState("C:\\ra\\GitHub\\CostFunction\\TestData\\sampleHumanState_scene1.bin");
			cluster.saveCluster("C:\\ra\\GitHub\\CostFunction\\TestData\\cluster_scene1.bin");
			cameraSetup.saveCameraSetup("C:\\ra\\GitHub\\CostFunction\\TestData\\cameraSetup_scene1.bin");
			test.saveCurrentSzene("C:\\ra\\GitHub\\CostFunction\\TestData\\currentSzene_scene1.bin");
			sampleConfig.saveCurrentSzene("C:\\ra\\GitHub\\CostFunction\\TestData\\currentSzeneSamplePoints_scene1.bin");

			CFIO::clearPCL(&humanPCL);
			CFIO::clearCameraPos(&cameraPos);


		}

		[TestMethod]
		void TestSaveSzene2()
		{
			CFIO::LINK humanPCL;
			const char* fileNameHuman = "C:\\ra\\GitHub\\CostFunction\\TestData\\human.bin";
			CFIO::loadLink(&humanPCL, fileNameHuman);

			CFIO::LINK envPCL;
			const char* fileNameEnv = "C:\\ra\\GitHub\\CostFunction\\TestData\\environment_scene2.bin";
			CFIO::loadLink(&envPCL, fileNameEnv);


			
			CFIO::LINK robotPCL;
			const char* fileNameRobot = "C:\\ra\\GitHub\\CostFunction\\TestData\\robot_short.bin2";
			//const char* fileNameRobot = "C:\\ra\\GitHub\\CostFunction\\TestData\\robotpcl_q0.bin";			
			CFIO::loadLink(&robotPCL, fileNameRobot);
			CFIO::SAMPLE_POSITIONS samplePositions;
			const char* fileSamplePositions = "C:\\ra\\GitHub\\CostFunction\\TestData\\samplePositionsS1.bin";			
			CFIO::loadSamplePositions(&samplePositions, fileSamplePositions);



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

			CFIO::POSSIBLE_CAMERA_TYPES possibleTypes;
			const char* fileSampleCameraTypes = "C:\\ra\\GitHub\\CostFunction\\TestData\\sampleCamera.bin";
			CFIO::loadSampleCamera(&possibleTypes, fileSampleCameraTypes);

			CFIO::SAMPLE_HUMAN_POSITIONS sampleHumanPos;
			CFIO::loadSampleHumanPositions(&sampleHumanPos, "C:\\ra\\GitHub\\CostFunction\\TestData\\sampleFitting.bin");
			PossibleConfigurations possibleHumanConfigs(&sampleHumanPos);

			CameraType** pType = new CameraType*[possibleTypes.nCameraTypes];
			CFIO::SAMPLE_CAMERA* pC;
			for(int i=0; i<possibleTypes.nCameraTypes; i++)
			{
				pType[i] = new CameraType(&(possibleTypes.possibleCameraTypes[i]));
			}


			CameraSetup cameraSetup(pType, possibleTypes.nCameraTypes);
			cameraSetup.init(2);
			cameraSetup.getCamera(0)->updateCameraPos(sampleConfig.getCurrentH(0));
			cameraSetup.getCamera(1)->updateCameraPos(sampleConfig.getCurrentH(1));
			cameraSetup.raytrace(test);

			EC ec(cameraSetup.getNRays());
			ec.reset();
			for(int camIndex = 0; camIndex<cameraSetup.getNCamera(); camIndex++)
			{
				ec.addDepthData(*cameraSetup.getCamera(camIndex));
			}
			ec.cluster();
			Cluster	cluster;
			if(ec.getNumOfClusters() > 0)
			{
				cluster = ec.getLargestCluster();
				cluster.calculateCentroid();

				possibleHumanConfigs.findBestFit(cluster);
				double maxProb = possibleHumanConfigs.getMaxProb();
			}
			

			

			possibleHumanConfigs.saveState("C:\\ra\\GitHub\\CostFunction\\TestData\\sampleHumanState_scene2.bin");
			cluster.saveCluster("C:\\ra\\GitHub\\CostFunction\\TestData\\cluster_scene2.bin");
			cameraSetup.saveCameraSetup("C:\\ra\\GitHub\\CostFunction\\TestData\\cameraSetup_scene2.bin");
			test.saveCurrentSzene("C:\\ra\\GitHub\\CostFunction\\TestData\\currentSzene_scene2.bin");
			sampleConfig.saveCurrentSzene("C:\\ra\\GitHub\\CostFunction\\TestData\\currentSzeneSamplePoints_scene2.bin");

			CFIO::clearPCL(&humanPCL);
			CFIO::clearCameraPos(&cameraPos);


		}


		[TestMethod]
		void TestSaveSzene3()
		{
			CFIO::LINK humanPCL;
			const char* fileNameHuman = "C:\\ra\\GitHub\\CostFunction\\TestData\\human.bin";
			CFIO::loadLink(&humanPCL, fileNameHuman);

			CFIO::LINK envPCL;
			const char* fileNameEnv = "C:\\ra\\GitHub\\CostFunction\\TestData\\environment_scene3.bin";
			CFIO::loadLink(&envPCL, fileNameEnv);


			
			CFIO::LINK robotPCL;
			const char* fileNameRobot = "C:\\ra\\GitHub\\CostFunction\\TestData\\robot_short.bin2";
			//const char* fileNameRobot = "C:\\ra\\GitHub\\CostFunction\\TestData\\robotpcl_q0.bin";			
			CFIO::loadLink(&robotPCL, fileNameRobot);
			CFIO::SAMPLE_POSITIONS samplePositions;
			const char* fileSamplePositions = "C:\\ra\\GitHub\\CostFunction\\TestData\\samplePositionsS1.bin";			
			CFIO::loadSamplePositions(&samplePositions, fileSamplePositions);



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

			CFIO::POSSIBLE_CAMERA_TYPES possibleTypes;
			const char* fileSampleCameraTypes = "C:\\ra\\GitHub\\CostFunction\\TestData\\sampleCamera.bin";
			CFIO::loadSampleCamera(&possibleTypes, fileSampleCameraTypes);

			CFIO::SAMPLE_HUMAN_POSITIONS sampleHumanPos;
			CFIO::loadSampleHumanPositions(&sampleHumanPos, "C:\\ra\\GitHub\\CostFunction\\TestData\\sampleFitting.bin");
			PossibleConfigurations possibleHumanConfigs(&sampleHumanPos);

			CameraType** pType = new CameraType*[possibleTypes.nCameraTypes];
			CFIO::SAMPLE_CAMERA* pC;
			for(int i=0; i<possibleTypes.nCameraTypes; i++)
			{
				pType[i] = new CameraType(&(possibleTypes.possibleCameraTypes[i]));
			}


			CameraSetup cameraSetup(pType, possibleTypes.nCameraTypes);
			cameraSetup.init(2);
			cameraSetup.getCamera(0)->updateCameraPos(sampleConfig.getCurrentH(0));
			cameraSetup.getCamera(1)->updateCameraPos(sampleConfig.getCurrentH(1));
			cameraSetup.raytrace(test);

			EC ec(cameraSetup.getNRays());
			ec.reset();
			for(int camIndex = 0; camIndex<cameraSetup.getNCamera(); camIndex++)
			{
				ec.addDepthData(*cameraSetup.getCamera(camIndex));
			}
			ec.cluster();
			Cluster	cluster;
			if(ec.getNumOfClusters() > 0)
			{
				cluster = ec.getLargestCluster();
				cluster.calculateCentroid();

				possibleHumanConfigs.findBestFit(cluster);
				double maxProb = possibleHumanConfigs.getMaxProb();
			}
			

			

			possibleHumanConfigs.saveState("C:\\ra\\GitHub\\CostFunction\\TestData\\sampleHumanState_scene3.bin");
			cluster.saveCluster("C:\\ra\\GitHub\\CostFunction\\TestData\\cluster_scene3.bin");
			cameraSetup.saveCameraSetup("C:\\ra\\GitHub\\CostFunction\\TestData\\cameraSetup_scene3.bin");
			test.saveCurrentSzene("C:\\ra\\GitHub\\CostFunction\\TestData\\currentSzene_scene3.bin");
			sampleConfig.saveCurrentSzene("C:\\ra\\GitHub\\CostFunction\\TestData\\currentSzeneSamplePoints_scene3.bin");

			CFIO::clearPCL(&humanPCL);
			CFIO::clearCameraPos(&cameraPos);


		}


		[TestMethod]
		void TestSaveSzene4()
		{
			CFIO::LINK humanPCL;
			const char* fileNameHuman = "C:\\ra\\GitHub\\CostFunction\\TestData\\human.bin";
			CFIO::loadLink(&humanPCL, fileNameHuman);

			CFIO::LINK envPCL;
			const char* fileNameEnv = "C:\\ra\\GitHub\\CostFunction\\TestData\\environment_scene3.bin";
			CFIO::loadLink(&envPCL, fileNameEnv);


			
			CFIO::LINK robotPCL;
			const char* fileNameRobot = "C:\\ra\\GitHub\\CostFunction\\TestData\\robot_short.bin2";
			//const char* fileNameRobot = "C:\\ra\\GitHub\\CostFunction\\TestData\\robotpcl_q0.bin";			
			CFIO::loadLink(&robotPCL, fileNameRobot);
			CFIO::SAMPLE_POSITIONS samplePositions;
			const char* fileSamplePositions = "C:\\ra\\GitHub\\CostFunction\\TestData\\samplePositionsS1.bin";			
			CFIO::loadSamplePositions(&samplePositions, fileSamplePositions);



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

			CFIO::POSSIBLE_CAMERA_TYPES possibleTypes;
			const char* fileSampleCameraTypes = "C:\\ra\\GitHub\\CostFunction\\TestData\\sampleCamera.bin";
			CFIO::loadSampleCamera(&possibleTypes, fileSampleCameraTypes);

			CFIO::SAMPLE_HUMAN_POSITIONS sampleHumanPos;
			CFIO::loadSampleHumanPositions(&sampleHumanPos, "C:\\ra\\GitHub\\CostFunction\\TestData\\sampleFitting.bin");
			PossibleConfigurations possibleHumanConfigs(&sampleHumanPos);

			CameraType** pType = new CameraType*[possibleTypes.nCameraTypes];
			CFIO::SAMPLE_CAMERA* pC;
			for(int i=0; i<possibleTypes.nCameraTypes; i++)
			{
				pType[i] = new CameraType(&(possibleTypes.possibleCameraTypes[i]));
			}


			CameraSetup cameraSetup(pType, possibleTypes.nCameraTypes);
			cameraSetup.init(2);
			cameraSetup.getCamera(0)->updateCameraPos(sampleConfig.getCurrentH(0));
			cameraSetup.getCamera(1)->updateCameraPos(sampleConfig.getCurrentH(1));
			cameraSetup.raytrace(test);

			EC ec(cameraSetup.getNRays());
			ec.reset();
			for(int camIndex = 0; camIndex<cameraSetup.getNCamera(); camIndex++)
			{
				ec.addDepthData(*cameraSetup.getCamera(camIndex));
			}
			ec.cluster();
			Cluster	cluster;
			if(ec.getNumOfClusters() > 0)
			{
				cluster = ec.getLargestCluster();
				cluster.calculateCentroid();

				possibleHumanConfigs.findBestFit(cluster);
				double maxProb = possibleHumanConfigs.getMaxProb();
			}
			

			

			possibleHumanConfigs.saveState("C:\\ra\\GitHub\\CostFunction\\TestData\\sampleHumanState_scene3.bin");
			cluster.saveCluster("C:\\ra\\GitHub\\CostFunction\\TestData\\cluster_scene3.bin");
			cameraSetup.saveCameraSetup("C:\\ra\\GitHub\\CostFunction\\TestData\\cameraSetup_scene3.bin");
			test.saveCurrentSzene("C:\\ra\\GitHub\\CostFunction\\TestData\\currentSzene_scene3.bin");
			sampleConfig.saveCurrentSzene("C:\\ra\\GitHub\\CostFunction\\TestData\\currentSzeneSamplePoints_scene3.bin");

			CFIO::clearPCL(&humanPCL);
			CFIO::clearCameraPos(&cameraPos);


		}
	};


}
