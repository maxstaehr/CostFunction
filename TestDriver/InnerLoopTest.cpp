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
//CameraType
			int nx = 1; 
			int ny = 1;
			float x[] = {1};
			float y[] = {0};
			float z[] = {0};

			int ssnx = 1;
			int ssny = 1;
			float ssx[] = {0};
			float ssy[] = {0};
			float ssz[] = {0};				
			
			CameraType* camTypes[1];
			camTypes[0] = new CameraType(nx, ny, x, y, z, ssnx, ssny, ssx, ssy,ssz, Link());

//Kinchain test
			float eye[] = EYE_16;			
			int nPos = 1;
			KinChain kinChain(nPos, 1);

			float *temp1 = new float[NELEM_H*DOF_E*nPos];
			float *temp2 = new float[NELEM_H*DOF_R*nPos];
			float *temp3 = new float[NELEM_H*DOF_H*nPos];
			for(int pos = 0; pos<nPos; pos++)
			{
				for(int h=0; h<DOF_E; h++)
				{
					for(int patteri=0; patteri<NELEM_H; patteri++)
					{
						temp1[pos*DOF_E*NELEM_H + h*NELEM_H + patteri] = eye[patteri];
					}
				}
			}

			for(int pos = 0; pos<nPos; pos++)
			{
				for(int h=0; h<DOF_R; h++)
				{
					for(int patteri=0; patteri<NELEM_H; patteri++)
					{
						temp2[pos*DOF_R*NELEM_H + h*NELEM_H + patteri] = eye[patteri];
					}
				}
			}

			for(int pos = 0; pos<nPos; pos++)
			{
				for(int h=0; h<DOF_H; h++)
				{
					for(int patteri=0; patteri<NELEM_H; patteri++)
					{
						temp3[pos*DOF_H*NELEM_H + h*NELEM_H + patteri] = eye[patteri];
					}
				}
			}

			kinChain.setEnvPos(temp1);
			kinChain.setRobotPos(temp2);
			kinChain.setHumanPos(temp3);
			kinChain.setPosIndex(0);

//Sample Configuration
			int index[] = {0};			
			HomogeneTransformation relative[1];		
			SampleCameraConfiguration sampleConfiguration(kinChain.getRobotLinks(), relative, index, DOF_R, 1);
//PossibleConfigs
			Transformation2D transform2D[1];
			PossibleConfigurations possibleConfigs(transform2D, 1);

//InnerLoop
			InnerLoop innerLoop(1, camTypes[0], kinChain, sampleConfiguration, possibleConfigs);

//free all the memory
			delete camTypes[0];
			
		}

		[TestMethod]
		void TestRun1()
		{
//CameraType
			int nx = 1; 
			int ny = 1;
			float x[] = {1};
			float y[] = {0};
			float z[] = {0};

			int ssnx = 1;
			int ssny = 1;
			float ssx[] = {0};
			float ssy[] = {0};
			float ssz[] = {0};				
			
			CameraType* camTypes[1];
			HomogeneTransformation trans[1];
			camTypes[0] = new CameraType(nx, ny, x, y, z, ssnx, ssny, ssx, ssy,ssz, Link());

//Kinchain test
			float eye[] = EYE_16;			
			int nPos = 1;
			KinChain kinChain(nPos, 1);

			float *temp1 = new float[NELEM_H*DOF_E*nPos];
			float *temp2 = new float[NELEM_H*DOF_R*nPos];
			float *temp3 = new float[NELEM_H*DOF_H*nPos];
			for(int pos = 0; pos<nPos; pos++)
			{
				for(int h=0; h<DOF_E; h++)
				{
					for(int patteri=0; patteri<NELEM_H; patteri++)
					{
						temp1[pos*DOF_E*NELEM_H + h*NELEM_H + patteri] = eye[patteri];
					}
				}
			}

			for(int pos = 0; pos<nPos; pos++)
			{
				for(int h=0; h<DOF_R; h++)
				{
					for(int patteri=0; patteri<NELEM_H; patteri++)
					{
						temp2[pos*DOF_R*NELEM_H + h*NELEM_H + patteri] = eye[patteri];
					}
				}
			}

			for(int pos = 0; pos<nPos; pos++)
			{
				for(int h=0; h<DOF_H; h++)
				{
					for(int patteri=0; patteri<NELEM_H; patteri++)
					{
						temp3[pos*DOF_H*NELEM_H + h*NELEM_H + patteri] = eye[patteri];
					}
				}
			}

			kinChain.setEnvPos(temp1);
			kinChain.setRobotPos(temp2);
			kinChain.setHumanPos(temp3);
			kinChain.setPosIndex(0);

//Sample Configuration
			int index[] = {0};			
			HomogeneTransformation relative[1];		
			SampleCameraConfiguration sampleConfiguration(kinChain.getRobotLinks(), relative, index, DOF_R, 1);
//PossibleConfigs
			Transformation2D transform2D[1];
			PossibleConfigurations possibleConfigs(transform2D, 1);

//InnerLoop
			InnerLoop innerLoop(1, camTypes[0], kinChain, sampleConfiguration, possibleConfigs);
			double prob = innerLoop.evaluatePositions(trans);

//free all the memory
			delete camTypes[0];
			
		}


		[TestMethod]
		void TestRun2()
		{
//CameraType
			int nx = 1; 
			int ny = 1;
			float x[] = {1};
			float y[] = {0};
			float z[] = {0};

			int ssnx = 1;
			int ssny = 1;
			float ssx[] = {0};
			float ssy[] = {0};
			float ssz[] = {0};				
			
			CameraType* camTypes[1];
			HomogeneTransformation trans[1];
			camTypes[0] = new CameraType(nx, ny, x, y, z, ssnx, ssny, ssx, ssy,ssz, Link());

//Kinchain test
			float eye[] = EYE_16;			
			int nPos = 1;
			KinChain kinChain(nPos, 1);

			float *temp1 = new float[NELEM_H*DOF_E*nPos];
			float *temp2 = new float[NELEM_H*DOF_R*nPos];
			float *temp3 = new float[NELEM_H*DOF_H*nPos];
			for(int pos = 0; pos<nPos; pos++)
			{
				for(int h=0; h<DOF_E; h++)
				{
					for(int patteri=0; patteri<NELEM_H; patteri++)
					{
						temp1[pos*DOF_E*NELEM_H + h*NELEM_H + patteri] = eye[patteri];
					}
				}
			}

			for(int pos = 0; pos<nPos; pos++)
			{
				for(int h=0; h<DOF_R; h++)
				{
					for(int patteri=0; patteri<NELEM_H; patteri++)
					{
						temp2[pos*DOF_R*NELEM_H + h*NELEM_H + patteri] = eye[patteri];
					}
				}
			}

			for(int pos = 0; pos<nPos; pos++)
			{
				for(int h=0; h<DOF_H; h++)
				{
					for(int patteri=0; patteri<NELEM_H; patteri++)
					{
						temp3[pos*DOF_H*NELEM_H + h*NELEM_H + patteri] = eye[patteri];
					}
				}
			}

			kinChain.setEnvPos(temp1);
			kinChain.setRobotPos(temp2);
			kinChain.setHumanPos(temp3);
			kinChain.setPosIndex(0);

//Sample Configuration
			int index[] = {0};			
			HomogeneTransformation relative[1];		
			SampleCameraConfiguration sampleConfiguration(kinChain.getRobotLinks(), relative, index, DOF_R, 1);
//PossibleConfigs
			Transformation2D transform2D[1];
			PossibleConfigurations possibleConfigs(transform2D, 1);

//InnerLoop
			InnerLoop innerLoop(1, camTypes[0], kinChain, sampleConfiguration, possibleConfigs);
			double prob = innerLoop.evaluatePositions(trans);

//free all the memory
			delete camTypes[0];
			
		}

	};
}
