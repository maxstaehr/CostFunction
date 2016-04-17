#include "stdafx.h"

using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace	Microsoft::VisualStudio::TestTools::UnitTesting;

#include "../CFObject/SampleCameraConfiguration.h"
#include <iostream>
#include <stdlib.h>
#include <time.h>

namespace TestDriver
{
	[TestClass]
	public ref class SampleCameraConfigurationTest
	{
	public: 

		[TestMethod]
		void TestConstructor1()
		{
			float initialH[] = EYE_16;
			float currentH[] = EYE_16;
			float relativH[] = EYE_16;
			int index[] = {0};
			int n = 1;

			HomogeneTransformation H1(initialH);
			HomogeneTransformation H2(currentH);
			HomogeneTransformation H3(relativH);
			
			Link a1[1];
			a1[0].setH(H1);
			HomogeneTransformation a2[1];
			a2[0] = H2;
			HomogeneTransformation a3[1];
			a3[0] = H3;

			SampleCameraConfiguration test(a1, a2, index, 1, 1);
			
			Assert::IsTrue(test.getIndex()		!= NULL);
			Assert::IsTrue(test.getNLink()			!= 0);
			
			for(int i=0; i<16; i++)
			{					
				Assert::AreEqual(test.getInitialH(0).getH()[i],		initialH[i]);				
				Assert::AreEqual(test.getCurrentH(0).getH()[i],		currentH[i]);				
				Assert::AreEqual(test.getRelativeH(0).getH()[i],	relativH[i]);				
			}
			Assert::AreEqual(test.getIndex()[0], index[0]);

		}


		[TestMethod]
		void TestConstructor2()
		{
			

			float currentH[] = EYE_16;
			currentH[3] = 2;
			currentH[7] = 2;
			currentH[11] = 2;
			HomogeneTransformation H1(currentH);
			Link a1[1];
			a1[0].setH(H1);

			float relativH[] = EYE_16;
			HomogeneTransformation H2(relativH);
			HomogeneTransformation a2[1];
			a2[0] = H2;

			float resCurrent[] = EYE_16;
			resCurrent[3] = 2;
			resCurrent[7] = 2;
			resCurrent[11] = 2;
			float resinitialH[] = EYE_16;
			resinitialH[3] = 2;
			resinitialH[7] = 2;
			resinitialH[11] = 2;

			int index[] = {0};
			int n = 1;


			SampleCameraConfiguration test(a1, a2, index, 1, 1);
			
			Assert::IsTrue(test.getIndex()		!= NULL);
			Assert::IsTrue(test.getNLink()			!= 0);
			
			for(int i=0; i<16; i++)
			{					
				Assert::AreEqual(test.getInitialH(0).getH()[i], resinitialH[i]);				
				Assert::AreEqual(test.getCurrentH(0).getH()[i], resCurrent[i]);				
				Assert::AreEqual(test.getRelativeH(0).getH()[i], relativH[i]);				
			}
			Assert::AreEqual(test.getIndex()[0], index[0]);

		}

		[TestMethod]
		void TestFindNN1()
		{
			float roll, pitch, yaw, x, y, z;
			HomogeneTransformation kinChainH1; 
			roll = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			pitch = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			yaw = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			kinChainH1.init(roll, pitch, yaw, x, y, z);

			HomogeneTransformation relativeH1;
			roll = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			pitch = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			yaw = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			relativeH1.init(roll, pitch, yaw, x, y, z);

			Link kinChain[1];
			kinChain[0].setH(kinChainH1);

			HomogeneTransformation relative[1];
			relative[0] = relativeH1;

			int index[] = {0};
			int n = 1;

			int nLink = 1;
			int nRelative = 1;
			SampleCameraConfiguration test(kinChain, relative, index, nLink, nRelative);

			HomogeneTransformation S = kinChainH1.mul(relativeH1);
			HomogeneTransformation result = test.findNN(S, HomogeneTransformation::DIM_DIR::XP);

			for(int i=0; i<16; i++)
			{
				Assert::AreEqual(S.getH()[i], result.getH()[i], 1e-5f);
			}
		}

		[TestMethod]
		void TestFindNN2()
		{
			float roll, pitch, yaw, x, y, z;
			HomogeneTransformation kinChainH1; 
			roll = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			pitch = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			yaw = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			kinChainH1.init(roll, pitch, yaw, x, y, z);
			Link kinChain[1];
			kinChain[0].setH(kinChainH1);

			HomogeneTransformation relative[3];
			
			HomogeneTransformation relativeH1;
			relativeH1.init(0, 0, 0, 0.1, 0, 0);
			HomogeneTransformation relativeH2;
			relativeH2.init(0, 0, 0, 0.2, 0, 0);
			HomogeneTransformation relativeH3;
			relativeH3.init(0, 0, 0, 0.3, 0, 0);
			HomogeneTransformation relativeH4;
			relativeH4.init(0, 0, 0, 0.0, 0, 0);

			relative[0] = relativeH1;
			relative[1] = relativeH2;
			relative[2] = relativeH3;

			int index[] = {0, 0, 0};
			int n = 3;

			int nLink = 1;
			int nRelative = 3;
			SampleCameraConfiguration test(kinChain, relative, index, nLink, nRelative);

			HomogeneTransformation S = kinChainH1.mul(relativeH4);
			HomogeneTransformation R = kinChainH1.mul(relativeH1);

			HomogeneTransformation result = test.findNN(S, HomogeneTransformation::DIM_DIR::XP);

			for(int i=0; i<16; i++)
			{
				Assert::AreEqual(R.getH()[i], result.getH()[i], 1e-5f);
			}
		}

		[TestMethod]
		void TestFindNN3()
		{
			float roll, pitch, yaw, x, y, z;
			HomogeneTransformation kinChainH1; 
			//roll = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			//pitch = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			//yaw = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			//x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			//y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			//z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			kinChainH1.init(roll, pitch, yaw, x, y, z);
			Link kinChain[1];
			kinChain[0].setH(kinChainH1);

			HomogeneTransformation relative[3];
			
			HomogeneTransformation relativeH1;
			relativeH1.init(0, 0, 0, 0.1, 0, 0);
			HomogeneTransformation relativeH2;
			relativeH2.init(0, 0, 0, 0.2, 0, 0);
			HomogeneTransformation relativeH3;
			relativeH3.init(0, 0, 0, 0.3, 0, 0);
			HomogeneTransformation relativeH4;
			relativeH4.init(0, 0, 0, 0.0, 0, 0);

			relative[0] = relativeH1;
			relative[1] = relativeH2;
			relative[2] = relativeH3;




			int index[] = {0, 0, 0};
			int n = 1;

			int nLink = 1;
			int nRelative = 3;
			SampleCameraConfiguration test(kinChain, relative, index, nLink, nRelative);

			HomogeneTransformation S = kinChainH1.mul(relativeH4);
			HomogeneTransformation R = kinChainH1.mul(relativeH4);

			HomogeneTransformation result = test.findNN(S, HomogeneTransformation::DIM_DIR::XM);

			for(int i=0; i<16; i++)
			{
				Assert::AreEqual(R.getH()[i], result.getH()[i], 1e-5f);
			}
		}

		[TestMethod]
		void TestFindNN4()
		{
			float roll, pitch, yaw, x, y, z;
			HomogeneTransformation kinChainH1; 
			roll = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			pitch = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			yaw = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			kinChainH1.init(roll, pitch, yaw, x, y, z);
			Link kinChain[1];
			kinChain[0].setH(kinChainH1);

			HomogeneTransformation relative[3];
			
			HomogeneTransformation relativeH1;
			relativeH1.init(0, 0, 0, 0.1, 0, 0);
			HomogeneTransformation relativeH2;
			relativeH2.init(0, 0, 0, 0.2, 0, 0);
			HomogeneTransformation relativeH3;
			relativeH3.init(0, 0, 0, 0.3, 0, 0);
			HomogeneTransformation relativeH4;
			relativeH4.init(0, 0, 0, 0.11, 0, 0);

			relative[0] = relativeH1;
			relative[1] = relativeH2;
			relative[2] = relativeH3;

			int index[] = {0, 0, 0};
			int n = 3;

			int nLink = 1;
			int nRelative = 3;
			SampleCameraConfiguration test(kinChain, relative, index, nLink, nRelative);

			HomogeneTransformation S = kinChainH1.mul(relativeH4);
			HomogeneTransformation R = kinChainH1.mul(relativeH1);

			HomogeneTransformation result = test.findNN(S, HomogeneTransformation::DIM_DIR::XM);

			for(int i=0; i<16; i++)
			{
				Assert::AreEqual(R.getH()[i], result.getH()[i], 1e-5f);
			}
		}

		[TestMethod]
		void TestFindNN5()
		{
			float roll, pitch, yaw, x, y, z;
			HomogeneTransformation kinChainH1; 
			roll = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			pitch = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			yaw = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			kinChainH1.init(roll, pitch, yaw, x, y, z);
			Link kinChain[1];
			kinChain[0].setH(kinChainH1);

			HomogeneTransformation relative[3];
			
			HomogeneTransformation relativeH1;
			relativeH1.init(0, 0, 0, 0.1, 0.1, 0);
			HomogeneTransformation relativeH2;
			relativeH2.init(0, 0, 0, 0.1, 0.1, 0);
			HomogeneTransformation relativeH3;
			relativeH3.init(0, 0, 0, 0.1, 0.2, 0);
			HomogeneTransformation relativeH4;
			relativeH4.init(0, 0, 0, 0.1, 0.1, 0);

			relative[0] = relativeH1;
			relative[1] = relativeH2;
			relative[2] = relativeH3;

			int index[] = {0, 0, 0};
			int n = 3;

			int nLink = 1;
			int nRelative = 3;
			SampleCameraConfiguration test(kinChain, relative, index, nLink, nRelative);

			HomogeneTransformation S = kinChainH1.mul(relativeH4);
			HomogeneTransformation R = kinChainH1.mul(relativeH3);

			HomogeneTransformation result = test.findNN(S, HomogeneTransformation::DIM_DIR::XM);

			for(int i=0; i<16; i++)
			{
				Assert::AreEqual(R.getH()[i], result.getH()[i], 1e-5f);
			}
		}

		[TestMethod]
		void TestFindNN6()
		{
			float roll, pitch, yaw, x, y, z;
			HomogeneTransformation kinChainH1; 
			roll = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			pitch = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			yaw = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			kinChainH1.init(roll, pitch, yaw, x, y, z);
			Link kinChain[1];
			kinChain[0].setH(kinChainH1);

			HomogeneTransformation relative[3];
			
			HomogeneTransformation relativeH1;
			relativeH1.init(0, 0, 0, 0.1, 0.1, 0);
			HomogeneTransformation relativeH2;
			relativeH2.init(0, 0, 0, 0.1, 0.1, 0);
			HomogeneTransformation relativeH3;
			relativeH3.init(0, 0, 0, 0.1, 0.2, 0);
			HomogeneTransformation relativeH4;
			relativeH4.init(0, 0, 0, 0.0, 0.1, 0);

			relative[0] = relativeH1;
			relative[1] = relativeH2;
			relative[2] = relativeH3;

			int index[] = {0, 0, 0};
			int n = 3;

			int nLink = 1;
			int nRelative = 3;
			SampleCameraConfiguration test(kinChain, relative, index, nLink, nRelative);

			HomogeneTransformation S = kinChainH1.mul(relativeH4);
			HomogeneTransformation R = kinChainH1.mul(relativeH4);

			HomogeneTransformation result = test.findNN(S, HomogeneTransformation::DIM_DIR::XM);

			for(int i=0; i<16; i++)
			{
				Assert::AreEqual(R.getH()[i], result.getH()[i], 1e-5f);
			}
		}

		[TestMethod]
		void TestFindNN7()
		{
			float roll, pitch, yaw, x, y, z;
			HomogeneTransformation kinChainH1; 
			roll = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			pitch = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			yaw = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			kinChainH1.init(roll, pitch, yaw, x, y, z);
			Link kinChain[1];
			kinChain[0].setH(kinChainH1);

			HomogeneTransformation relative[3];
			
			HomogeneTransformation relativeH1;
			relativeH1.init(0, 0, 0, 0,0.1, 0);
			HomogeneTransformation relativeH2;
			relativeH2.init(0, 0, 0, 0,0.2, 0);
			HomogeneTransformation relativeH3;
			relativeH3.init(0, 0, 0,0, 0.3, 0);
			HomogeneTransformation relativeH4;
			relativeH4.init(0, 0, 0,0, 0.11, 0);

			relative[0] = relativeH1;
			relative[1] = relativeH2;
			relative[2] = relativeH3;

			int index[] = {0, 0, 0};
			int n = 3;

			int nLink = 1;
			int nRelative = 3;
			SampleCameraConfiguration test(kinChain, relative, index, nLink, nRelative);

			HomogeneTransformation S = kinChainH1.mul(relativeH4);
			HomogeneTransformation R = kinChainH1.mul(relativeH1);

			HomogeneTransformation result = test.findNN(S, HomogeneTransformation::DIM_DIR::YM);

			for(int i=0; i<16; i++)
			{
				Assert::AreEqual(R.getH()[i], result.getH()[i], 1e-5f);
			}
		}

		[TestMethod]
		void TestFindNN8()
		{
			float roll, pitch, yaw, x, y, z;
			HomogeneTransformation kinChainH1; 
			roll = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			pitch = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			yaw = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			kinChainH1.init(roll, pitch, yaw, x, y, z);
			Link kinChain[1];
			kinChain[0].setH(kinChainH1);

			HomogeneTransformation relative[3];
			
			HomogeneTransformation relativeH1;
			relativeH1.init(0, 0, 0,0, 0.1, 0.1);
			HomogeneTransformation relativeH2;
			relativeH2.init(0, 0, 0,0, 0.1, 0.1);
			HomogeneTransformation relativeH3;
			relativeH3.init(0, 0, 0,0, 0.1, 0.2);
			HomogeneTransformation relativeH4;
			relativeH4.init(0, 0, 0,0, 0.1, 0.1);

			relative[0] = relativeH1;
			relative[1] = relativeH2;
			relative[2] = relativeH3;

			int index[] = {0, 0, 0};
			int n = 3;

			int nLink = 1;
			int nRelative = 3;
			SampleCameraConfiguration test(kinChain, relative, index, nLink, nRelative);

			HomogeneTransformation S = kinChainH1.mul(relativeH4);
			HomogeneTransformation R = kinChainH1.mul(relativeH3);

			HomogeneTransformation result = test.findNN(S, HomogeneTransformation::DIM_DIR::YM);

			for(int i=0; i<16; i++)
			{
				Assert::AreEqual(R.getH()[i], result.getH()[i], 1e-5f);
			}
		}

		[TestMethod]
		void TestFindNN9()
		{
			float roll, pitch, yaw, x, y, z;
			HomogeneTransformation kinChainH1; 
			roll = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			pitch = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			yaw = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			kinChainH1.init(roll, pitch, yaw, x, y, z);
			Link kinChain[1];
			kinChain[0].setH(kinChainH1);

			HomogeneTransformation relative[3];
			
			HomogeneTransformation relativeH1;
			relativeH1.init(0, 0, 0,0, 0.1, 0.1);
			HomogeneTransformation relativeH2;
			relativeH2.init(0, 0, 0,0, 0.1, 0.1);
			HomogeneTransformation relativeH3;
			relativeH3.init(0, 0, 0,0, 0.1, 0.2);
			HomogeneTransformation relativeH4;
			relativeH4.init(0, 0, 0,0, 0.0, 0.1);

			relative[0] = relativeH1;
			relative[1] = relativeH2;
			relative[2] = relativeH3;

			int index[] = {0, 0, 0};
			int n = 3;

			int nLink = 1;
			int nRelative = 3;
			SampleCameraConfiguration test(kinChain, relative, index, nLink, nRelative);

			HomogeneTransformation S = kinChainH1.mul(relativeH4);
			HomogeneTransformation R = kinChainH1.mul(relativeH4);

			HomogeneTransformation result = test.findNN(S, HomogeneTransformation::DIM_DIR::YM);

			for(int i=0; i<16; i++)
			{
				Assert::AreEqual(R.getH()[i], result.getH()[i], 1e-5f);
			}
		}

		[TestMethod]
		void TestFindNN10()
		{
			float roll, pitch, yaw, x, y, z;
			HomogeneTransformation kinChainH1; 
			roll = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			pitch = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			yaw = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			kinChainH1.init(roll, pitch, yaw, x, y, z);
			Link kinChain[1];
			kinChain[0].setH(kinChainH1);

			HomogeneTransformation relative[3];
			
			HomogeneTransformation relativeH1;
			relativeH1.init(0, 0, 0, 0, 0, 0.1);
			HomogeneTransformation relativeH2;
			relativeH2.init(0, 0, 0, 0, 0, 0.2);
			HomogeneTransformation relativeH3;
			relativeH3.init(0, 0, 0, 0, 0, 0.3);
			HomogeneTransformation relativeH4;
			relativeH4.init(0, 0, 0, 0, 0, 0.11);

			relative[0] = relativeH1;
			relative[1] = relativeH2;
			relative[2] = relativeH3;

			int index[] = {0, 0, 0};
			int n = 3;

			int nLink = 1;
			int nRelative = 3;
			SampleCameraConfiguration test(kinChain, relative, index, nLink, nRelative);

			HomogeneTransformation S = kinChainH1.mul(relativeH4);
			HomogeneTransformation R = kinChainH1.mul(relativeH1);

			HomogeneTransformation result = test.findNN(S, HomogeneTransformation::DIM_DIR::ZM);

			for(int i=0; i<16; i++)
			{
				Assert::AreEqual(R.getH()[i], result.getH()[i], 1e-5f);
			}
		}

		[TestMethod]
		void TestFindNN11()
		{
			float roll, pitch, yaw, x, y, z;
			HomogeneTransformation kinChainH1; 
			roll = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			pitch = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			yaw = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			kinChainH1.init(roll, pitch, yaw, x, y, z);
			Link kinChain[1];
			kinChain[0].setH(kinChainH1);

			HomogeneTransformation relative[3];
			
			HomogeneTransformation relativeH1;
			relativeH1.init(0, 0, 0,0, 0.1, 0.1);
			HomogeneTransformation relativeH2;
			relativeH2.init(0, 0, 0,0, 0.1, 0.1);
			HomogeneTransformation relativeH3;
			relativeH3.init(0, 0, 0,0, 0.2, 0.1);
			HomogeneTransformation relativeH4;
			relativeH4.init(0, 0, 0,0, 0.1, 0.1);

			relative[0] = relativeH1;
			relative[1] = relativeH2;
			relative[2] = relativeH3;

			int index[] = {0, 0, 0};
			int n = 3;

			int nLink = 1;
			int nRelative = 3;
			SampleCameraConfiguration test(kinChain, relative, index, nLink, nRelative);

			HomogeneTransformation S = kinChainH1.mul(relativeH4);
			HomogeneTransformation R = kinChainH1.mul(relativeH2);

			HomogeneTransformation result = test.findNN(S, HomogeneTransformation::DIM_DIR::ZM);

			for(int i=0; i<16; i++)
			{
				Assert::AreEqual(R.getH()[i], result.getH()[i], 1e-5f);
			}
		}

		[TestMethod]
		void TestFindNN12()
		{
			float roll, pitch, yaw, x, y, z;
			HomogeneTransformation kinChainH1; 
			roll = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			pitch = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			yaw = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			kinChainH1.init(roll, pitch, yaw, x, y, z);
			Link kinChain[1];
			kinChain[0].setH(kinChainH1);

			HomogeneTransformation relative[3];
			
			HomogeneTransformation relativeH1;
			relativeH1.init(0, 0, 0,0, 0.1, 0.1);
			HomogeneTransformation relativeH2;
			relativeH2.init(0, 0, 0,0, 0.1, 0.1);
			HomogeneTransformation relativeH3;
			relativeH3.init(0, 0, 0,0, 0.2, 0.1);
			HomogeneTransformation relativeH4;
			relativeH4.init(0, 0, 0,0, 0.1, 0.0);

			relative[0] = relativeH1;
			relative[1] = relativeH2;
			relative[2] = relativeH3;

			int index[] = {0, 0, 0};
			int n = 3;

			int nLink = 1;
			int nRelative = 3;
			SampleCameraConfiguration test(kinChain, relative, index, nLink, nRelative);

			HomogeneTransformation S = kinChainH1.mul(relativeH4);
			HomogeneTransformation R = kinChainH1.mul(relativeH4);

			HomogeneTransformation result = test.findNN(S, HomogeneTransformation::DIM_DIR::ZM);

			for(int i=0; i<16; i++)
			{
				Assert::AreEqual(R.getH()[i], result.getH()[i], 1e-5f);
			}
		}

		[TestMethod]
		void TestFindNN13()
		{
			float roll, pitch, yaw, x, y, z;
			HomogeneTransformation kinChainH1; 
			roll = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			pitch = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			yaw = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			kinChainH1.init(roll, pitch, yaw, x, y, z);
			Link kinChain[1];
			kinChain[0].setH(kinChainH1);

			HomogeneTransformation relative[3];
			
			HomogeneTransformation relativeH1;
			relativeH1.init(0.1, 0, 0, 0, 0, 0);
			HomogeneTransformation relativeH2;
			relativeH2.init(0.2, 0, 0, 0, 0, 0);
			HomogeneTransformation relativeH3;
			relativeH3.init(0.3, 0, 0, 0, 0, 0);
			HomogeneTransformation relativeH4;
			relativeH4.init(0.11, 0, 0, 0, 0, 0);

			relative[0] = relativeH1;
			relative[1] = relativeH2;
			relative[2] = relativeH3;




			int index[] = {0, 0, 0};
			int n = 3;
			int nLink = 1;
			int nRelative = 3;
			SampleCameraConfiguration test(kinChain, relative, index, nLink, nRelative);

			HomogeneTransformation S = kinChainH1.mul(relativeH4);
			HomogeneTransformation R = kinChainH1.mul(relativeH1);

			HomogeneTransformation result = test.findNN(S, HomogeneTransformation::DIM_DIR::ROLLM);

			for(int i=0; i<16; i++)
			{
				Assert::AreEqual(R.getH()[i], result.getH()[i], 1e-5f);
			}
		}

		[TestMethod]
		void TestFindNN14()
		{
			float roll, pitch, yaw, x, y, z;
			HomogeneTransformation kinChainH1; 
			//roll = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			//pitch = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			//yaw = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			//x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			//y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			//z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			kinChainH1.init(roll, pitch, yaw, x, y, z);
			Link kinChain[1];
			kinChain[0].setH(kinChainH1);

			HomogeneTransformation relative[3];
			
			HomogeneTransformation relativeH1;
			relativeH1.init(0.1, 0, 0, 0, 0, 0.2);
			HomogeneTransformation relativeH2;
			relativeH2.init(0.1, 0, 0, 0, 0, 0.11);
			HomogeneTransformation relativeH3;
			relativeH3.init(0.1, 0, 0, 0, 0, 0.2);

			HomogeneTransformation relativeH4;
			relativeH4.init(0.1, 0, 0, 0, 0, 0.1);

			relative[0] = relativeH1;
			relative[1] = relativeH2;
			relative[2] = relativeH3;

			int index[] = {0, 0, 0};
			int n = 3;

			int nLink = 1;
			int nRelative = 3;
			SampleCameraConfiguration test(kinChain, relative, index, nLink, nRelative);

			HomogeneTransformation S = kinChainH1.mul(relativeH4);
			HomogeneTransformation R = kinChainH1.mul(relativeH4);

			HomogeneTransformation result = test.findNN(S, HomogeneTransformation::DIM_DIR::ROLLM);

			for(int i=0; i<16; i++)
			{
				Assert::AreEqual(R.getH()[i], result.getH()[i], 1e-5f);
			}
		}

		[TestMethod]
		void TestFindNN15()
		{
			float roll, pitch, yaw, x, y, z;
			HomogeneTransformation kinChainH1; 
			roll = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			pitch = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			yaw = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			kinChainH1.init(roll, pitch, yaw, x, y, z);
			Link kinChain[1];
			kinChain[0].setH(kinChainH1);

			HomogeneTransformation relative[3];
			
			HomogeneTransformation relativeH1;
			relativeH1.init(0.25, 0, 0, 0, 0, 0.3);
			HomogeneTransformation relativeH2;
			relativeH2.init(0.2, 0, 0, 0, 0, 0.1);
			HomogeneTransformation relativeH3;
			relativeH3.init(0.2, 0, 0, 0, 0, 0.2);
			HomogeneTransformation relativeH4;
			relativeH4.init(0.3, 0, 0, 0, 0, 0.0);

			relative[0] = relativeH1;
			relative[1] = relativeH2;
			relative[2] = relativeH3;

			int index[] = {0, 0, 0};

			int nLink = 1;
			int nRelative = 3;
			SampleCameraConfiguration test(kinChain, relative, index, nLink, nRelative);

			HomogeneTransformation S = kinChainH1.mul(relativeH4);
			HomogeneTransformation R = kinChainH1.mul(relativeH1);

			HomogeneTransformation result = test.findNN(S, HomogeneTransformation::DIM_DIR::ROLLM);

			for(int i=0; i<16; i++)
			{
				Assert::AreEqual(R.getH()[i], result.getH()[i], 1e-5f);
			}
		}

	/*	[TestMethod]
		void TestFindNN16()
		{
			float roll, pitch, yaw, x, y, z;
			HomogeneTransformation kinChainH1; 
			roll = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			pitch = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			yaw = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			kinChainH1.init(roll, pitch, yaw, x, y, z);
			HomogeneTransformation kinChain[1];
			kinChain[0] = kinChainH1;

			HomogeneTransformation relative[3];
			
			HomogeneTransformation relativeH1;
			relativeH1.init(0.25, 0, 0, 0, 0, 0.3);
			HomogeneTransformation relativeH2;
			relativeH2.init(0.2, 0, 0, 0, 0, 0.1);
			HomogeneTransformation relativeH3;
			relativeH3.init(0.2, 0, 0, 0, 0, 0.2);
			HomogeneTransformation relativeH4;
			relativeH4.init(0.3, 0, 0, 0, 0, 0.0);

			relative[0] = relativeH1;
			relative[1] = relativeH2;
			relative[2] = relativeH3;

			int index[] = {0, 0, 0};

			int nLink = 1;
			int nRelative = 3;
			SampleCameraConfiguration test(kinChain, relative, index, nLink, nRelative);

			HomogeneTransformation S = kinChainH1.mul(relativeH4);
			HomogeneTransformation R = kinChainH1.mul(relativeH1);

			HomogeneTransformation result = test.findNN(S, HomogeneTransformation::DIM_DIR::ROLLM);

			for(int i=0; i<16; i++)
			{
				Assert::AreEqual(R.getH()[i], result.getH()[i], 1e-5f);
			}


			HomogeneTransformation absolute1;
			HomogeneTransformation relative1;
			int linkIndex;
			bool ret =  test.findRelativeTransformationAndLink(R,  kinChain, absolute1, relative1, linkIndex);

			Assert::IsTrue(ret);
			Assert::AreEqual(0, linkIndex);
			for(int i=0; i<16; i++)
			{
				Assert::AreEqual(absolute1.getH()[i], kinChainH1.getH()[i], 1e-5f);
				Assert::AreEqual(relative1.getH()[i], relativeH1.getH()[i], 1e-5f);
			}
		}*/


		[TestMethod]
		void TestFindNewCameraPosTest()
		{
			float roll, pitch, yaw, x, y, z;
			HomogeneTransformation kinChainH1; 
			roll = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			pitch = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			yaw = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			kinChainH1.init(roll, pitch, yaw, x, y, z);
			Link kinChain[1];
			kinChain[0].setH(kinChainH1);

			HomogeneTransformation relative[3];
			
			HomogeneTransformation relativeH1;
			relativeH1.init(0.25, 0, 0, 0, 0, 0.3);
			HomogeneTransformation relativeH2;
			relativeH2.init(0.2, 0, 0, 0, 0, 0.1);
			HomogeneTransformation relativeH3;
			relativeH3.init(0.2, 0, 0, 0, 0, 0.2);
			HomogeneTransformation relativeH4;
			relativeH4.init(0.3, 0, 0, 0, 0, 0.0);

			relative[0] = relativeH1;
			relative[1] = relativeH2;
			relative[2] = relativeH3;

			int index[] = {0, 0, 0};

			int nLink = 1;
			int nRelative = 3;
			SampleCameraConfiguration test(kinChain, relative, index, nLink, nRelative);

			HomogeneTransformation S = kinChainH1.mul(relativeH4);
			HomogeneTransformation R = kinChainH1.mul(relativeH1);

			HomogeneTransformation result = test.findNN(S, HomogeneTransformation::DIM_DIR::ROLLM);

			for(int i=0; i<16; i++)
			{
				Assert::AreEqual(R.getH()[i], result.getH()[i], 1e-5f);
			}


			HomogeneTransformation absolute1;
			HomogeneTransformation relative1;


			HomogeneTransformation ret = test.findNewCameraPos(kinChain, R);


			for(int i=0; i<16; i++)
			{
				Assert::AreEqual(ret.getH()[i], R.getH()[i], 1e-5f);
				
			}
		}


		[TestMethod]
		void TestFindNewCameraPosTest2()
		{
			float roll, pitch, yaw, x, y, z;
			HomogeneTransformation kinChainH1; 
			roll = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			pitch = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			yaw = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			kinChainH1.init(roll, pitch, yaw, x, y, z);
			Link kinChain1[1];
			kinChain1[0].setH(kinChainH1);


						
			HomogeneTransformation kinChainH2; 
			roll = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			pitch = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			yaw = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			kinChainH2.init(roll, pitch, yaw, x, y, z);
			Link kinChain2[1];
			kinChain2[0].setH(kinChainH2);

			HomogeneTransformation relative[3];
			
			HomogeneTransformation relativeH1;
			relativeH1.init(0.25, 0, 0, 0, 0, 0.3);
			HomogeneTransformation relativeH2;
			relativeH2.init(0.2, 0, 0, 0, 0, 0.1);
			HomogeneTransformation relativeH3;
			relativeH3.init(0.2, 0, 0, 0, 0, 0.2);
			HomogeneTransformation relativeH4;
			relativeH4.init(0.3, 0, 0, 0, 0, 0.0);

			relative[0] = relativeH1;
			relative[1] = relativeH2;
			relative[2] = relativeH3;

			int index[] = {0, 0, 0};

			int nLink = 1;
			int nRelative = 3;
			SampleCameraConfiguration test(kinChain1, relative, index, nLink, nRelative);
			
			HomogeneTransformation R1 = kinChainH1.mul(relativeH1);
			HomogeneTransformation R2 = kinChainH2.mul(relativeH1);


			HomogeneTransformation ret = test.findNewCameraPos(kinChain2, R1);
			for(int i=0; i<16; i++)
			{
				Assert::AreEqual(ret.getH()[i], R2.getH()[i], 1e-5f);
				
			}
		}


		[TestMethod]
		void TestFindNewCameraPosTest3()
		{
			float roll, pitch, yaw, x, y, z;
			HomogeneTransformation kinChainH1; 
			roll = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			pitch = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			yaw = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			kinChainH1.init(roll, pitch, yaw, x, y, z);
			Link kinChain1[1];
			kinChain1[0].setH(kinChainH1);


						
			HomogeneTransformation kinChainH2; 
			roll = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			pitch = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			yaw = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			kinChainH2.init(roll, pitch, yaw, x, y, z);
			Link kinChain2[1];
			kinChain2[0].setH(kinChainH2);

			HomogeneTransformation kinChainH3; 
			roll = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			pitch = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			yaw = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			kinChainH3.init(roll, pitch, yaw, x, y, z);
			Link kinChain3[1];
			kinChain3[0].setH(kinChainH3);

			HomogeneTransformation relative[3];
			
			HomogeneTransformation relativeH1;
			relativeH1.init(0.25, 0, 0, 0, 0, 0.3);
			HomogeneTransformation relativeH2;
			relativeH2.init(0.2, 0, 0, 0, 0, 0.1);
			HomogeneTransformation relativeH3;
			relativeH3.init(0.2, 0, 0, 0, 0, 0.2);
			HomogeneTransformation relativeH4;
			relativeH4.init(0.3, 0, 0, 0, 0, 0.0);

			relative[0] = relativeH1;
			relative[1] = relativeH2;
			relative[2] = relativeH3;

			int index[] = {0, 0, 0};

			int nLink = 1;
			int nRelative = 3;
			SampleCameraConfiguration test(kinChain1, relative, index, nLink, nRelative);
			
			HomogeneTransformation R1 = kinChainH1.mul(relativeH1);
			HomogeneTransformation R2 = kinChainH2.mul(relativeH1);
			HomogeneTransformation R3 = kinChainH3.mul(relativeH1);


			HomogeneTransformation ret1 = test.findNewCameraPos(kinChain2, R1);
			HomogeneTransformation ret2 = test.findNewCameraPos(kinChain3, R1);
			for(int i=0; i<16; i++)
			{
				Assert::AreEqual(ret1.getH()[i], R2.getH()[i], 1e-5f);				
				Assert::AreEqual(ret2.getH()[i], R3.getH()[i], 1e-5f);	
			}
		}


		[TestMethod]
		void TestCopyConstructor()
		{
			float roll, pitch, yaw, x, y, z;
			HomogeneTransformation kinChainH1; 
			roll = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			pitch = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			yaw = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			kinChainH1.init(roll, pitch, yaw, x, y, z);
			Link kinChain1[1];
			kinChain1[0].setH(kinChainH1);


				

			HomogeneTransformation relative[3];
			
			HomogeneTransformation relativeH1;
			relativeH1.init(0.25, 0, 0, 0, 0, 0.3);
			HomogeneTransformation relativeH2;
			relativeH2.init(0.2, 0, 0, 0, 0, 0.1);
			HomogeneTransformation relativeH3;
			relativeH3.init(0.2, 0, 0, 0, 0, 0.2);

			relative[0] = relativeH1;
			relative[1] = relativeH2;
			relative[2] = relativeH3;
			int index[] = {0, 0, 0};
			int nLink = 1;
			int nRelative = 3;
			SampleCameraConfiguration test1;
			SampleCameraConfiguration test2(kinChain1, relative, index, nLink, nRelative);
			test1 = test2;

			for(int i=0; i<16; i++)
			{

				
				for(int j=0; j<3; j++)
				{
					Assert::AreEqual(test1.getInitialH(j).getH()[i], test2.getInitialH(j).getH()[i], 1e-5f);
					Assert::AreEqual(test1.getRelativeH(j).getH()[i], test2.getRelativeH(j).getH()[i], 1e-5f);
					Assert::AreEqual(test1.getCurrentH(j).getH()[i], test2.getCurrentH(j).getH()[i], 1e-5f);
					
				}
				Assert::AreEqual(test1.getNLink(), test2.getNLink());
				Assert::AreEqual(test1.getNRelative(), test2.getNRelative());
			}

		}

		[TestMethod]
		void TestEqualOperator()
		{
			float roll, pitch, yaw, x, y, z;
			HomogeneTransformation kinChainH1; 
			roll = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			pitch = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			yaw = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			kinChainH1.init(roll, pitch, yaw, x, y, z);
			Link kinChain1[1];
			kinChain1[0].setH(kinChainH1);


				

			HomogeneTransformation relative[3];
			
			HomogeneTransformation relativeH1;
			relativeH1.init(0.25, 0, 0, 0, 0, 0.3);
			HomogeneTransformation relativeH2;
			relativeH2.init(0.2, 0, 0, 0, 0, 0.1);
			HomogeneTransformation relativeH3;
			relativeH3.init(0.2, 0, 0, 0, 0, 0.2);

			relative[0] = relativeH1;
			relative[1] = relativeH2;
			relative[2] = relativeH3;
			int index[] = {0, 0, 0};
			int nLink = 1;
			int nRelative = 3;
			
			SampleCameraConfiguration test2(kinChain1, relative, index, nLink, nRelative);
			SampleCameraConfiguration test1 = test2;

			for(int i=0; i<16; i++)
			{				
				for(int j=0; j<3; j++)
				{
					Assert::AreEqual(test1.getInitialH(j).getH()[i], test2.getInitialH(j).getH()[i], 1e-5f);
					Assert::AreEqual(test1.getRelativeH(j).getH()[i], test2.getRelativeH(j).getH()[i], 1e-5f);
					Assert::AreEqual(test1.getCurrentH(j).getH()[i], test2.getCurrentH(j).getH()[i], 1e-5f);
					
				}
				Assert::AreEqual(test1.getNLink(), test2.getNLink());
				Assert::AreEqual(test1.getNRelative(), test2.getNRelative());
			}

		}


	};
}
