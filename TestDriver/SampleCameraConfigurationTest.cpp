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
			
			HomogeneTransformation a1[1];
			a1[0] = H1;
			HomogeneTransformation a2[1];
			a2[0] = H2;
			HomogeneTransformation a3[1];
			a3[0] = H3;

			SampleCameraConfiguration test(a1, a2, index, n);
			
			Assert::IsTrue(test.getIndex()		!= NULL);
			Assert::IsTrue(test.getN()			!= 0);
			
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
			HomogeneTransformation a1[1];
			a1[0] = H1;

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


			SampleCameraConfiguration test(a1, a2, index, n);
			
			Assert::IsTrue(test.getIndex()		!= NULL);
			Assert::IsTrue(test.getN()			!= 0);
			
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

			HomogeneTransformation kinChain[1];
			kinChain[0] = kinChainH1;

			HomogeneTransformation relative[1];
			relative[0] = relativeH1;

			int index[] = {0};
			int n = 1;

			SampleCameraConfiguration test(kinChain, relative, index, n);

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
			HomogeneTransformation kinChain[1];
			kinChain[0] = kinChainH1;

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

			SampleCameraConfiguration test(kinChain, relative, index, n);

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




			int index[] = {0};
			int n = 1;

			SampleCameraConfiguration test(kinChain, relative, index, n);

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
			HomogeneTransformation kinChain[1];
			kinChain[0] = kinChainH1;

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

			SampleCameraConfiguration test(kinChain, relative, index, n);

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
			HomogeneTransformation kinChain[1];
			kinChain[0] = kinChainH1;

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

			SampleCameraConfiguration test(kinChain, relative, index, n);

			HomogeneTransformation S = kinChainH1.mul(relativeH4);
			HomogeneTransformation R = kinChainH1.mul(relativeH2);

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
			HomogeneTransformation kinChain[1];
			kinChain[0] = kinChainH1;

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

			SampleCameraConfiguration test(kinChain, relative, index, n);

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
			HomogeneTransformation kinChain[1];
			kinChain[0] = kinChainH1;

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

			SampleCameraConfiguration test(kinChain, relative, index, n);

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
			HomogeneTransformation kinChain[1];
			kinChain[0] = kinChainH1;

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

			SampleCameraConfiguration test(kinChain, relative, index, n);

			HomogeneTransformation S = kinChainH1.mul(relativeH4);
			HomogeneTransformation R = kinChainH1.mul(relativeH2);

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
			HomogeneTransformation kinChain[1];
			kinChain[0] = kinChainH1;

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

			SampleCameraConfiguration test(kinChain, relative, index, n);

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
			HomogeneTransformation kinChain[1];
			kinChain[0] = kinChainH1;

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

			SampleCameraConfiguration test(kinChain, relative, index, n);

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
			HomogeneTransformation kinChain[1];
			kinChain[0] = kinChainH1;

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

			SampleCameraConfiguration test(kinChain, relative, index, n);

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
			HomogeneTransformation kinChain[1];
			kinChain[0] = kinChainH1;

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

			SampleCameraConfiguration test(kinChain, relative, index, n);

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
			HomogeneTransformation kinChain[1];
			kinChain[0] = kinChainH1;

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

			SampleCameraConfiguration test(kinChain, relative, index, n);

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
			relativeH1.init(0.1, 0, 0, 0, 0, 0.2);
			HomogeneTransformation relativeH2;
			relativeH2.init(0.1, 0, 0, 0, 0, 0.1);
			HomogeneTransformation relativeH3;
			relativeH3.init(0.1, 0, 0, 0, 0, 0.2);
			HomogeneTransformation relativeH4;
			relativeH4.init(0.1, 0, 0, 0, 0, 0.1);

			relative[0] = relativeH1;
			relative[1] = relativeH2;
			relative[2] = relativeH3;

			int index[] = {0, 0, 0};
			int n = 3;

			SampleCameraConfiguration test(kinChain, relative, index, n);

			HomogeneTransformation S = kinChainH1.mul(relativeH4);
			HomogeneTransformation R = kinChainH1.mul(relativeH2);

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
			//roll = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			//pitch = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			//yaw = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			//x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			//y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			//z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
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
			int n = 3;

			SampleCameraConfiguration test(kinChain, relative, index, n);

			HomogeneTransformation S = kinChainH1.mul(relativeH4);
			HomogeneTransformation R = kinChainH1.mul(relativeH1);

			HomogeneTransformation result = test.findNN(S, HomogeneTransformation::DIM_DIR::ROLLM);

			for(int i=0; i<16; i++)
			{
				Assert::AreEqual(R.getH()[i], result.getH()[i], 1e-5f);
			}
		}

	};
}
