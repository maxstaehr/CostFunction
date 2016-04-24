#include "stdafx.h"

#define _USE_MATH_DEFINES 
#include <cmath>

#include <float.h>
using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace	Microsoft::VisualStudio::TestTools::UnitTesting;

#include "../CFObject/SA.h"

namespace TestDriver
{
	[TestClass]
	public ref class SATest
	{
	public: 
		[TestMethod]
		void TestMethod1()
		{
			int base = 3;
			int exp = 6;


			int value = (int)pow(3.0,6.0);
			
			Link kinChain[1];
			HomogeneTransformation *relative = new HomogeneTransformation[value];
			int *index = new int[value];

			float da = M_PI/4;
			float xr, yr, zr, rollr, pitchr, yawr;
			int ii =0;
			for(int x=-1;x<2; x++)
				for(int y=-1; y<2; y++)
					for(int z=-1; z<2; z++)
						for(int roll=-1; roll<2; roll++)
							for(int pitch=-1; pitch<2; pitch++)
								for(int yaw=-1; yaw<2; yaw++)
								{
									xr = x;
									yr = y;
									zr = z;
									rollr = roll*da;
									pitchr = pitch*da;
									yawr = yaw*da;
									relative[ii].init(rollr, pitchr, yawr, xr, yr, zr);
									index[ii] = 0;
									ii++;
									
								}
						
			int n = 1;
			SampleCameraConfiguration sC(kinChain, relative, index, n, value);

			SA test(&sC, 1, false);

			
			HomogeneTransformation start, current, minus, plus;		
			HomogeneTransformation currentExpected, minusExpected, plusExpected;
			bool ret;
			SA::STATE state;

			test.setCurrentTransformation(start,0);
			
			current = test.getCurrentTransformation(0);
			minus = test.getNextEvalMinus(0);
			plus = test.getNextEvalPlus(0);

			currentExpected = sC.getInitialH(364);
			minusExpected = sC.getInitialH(121);
			plusExpected = sC.getInitialH(607);
						
			Assert::IsTrue(current.isEqual(currentExpected));
			Assert::IsTrue(minus.isEqual(minusExpected));
			Assert::IsTrue(plus.isEqual(plusExpected));

/***************************************************************/						
			ret = test.nextIteration(0.99, 0.98);

			state = test.getState();
			current = test.getCurrentTransformation(0);
			minus = test.getNextEvalMinus(0);
			plus = test.getNextEvalPlus(0);

			currentExpected = sC.getInitialH(607);
			minusExpected = sC.getInitialH(526);
			plusExpected = sC.getInitialH(688);						

			Assert::IsTrue(state==SA::STATE::HC);
			Assert::IsTrue(ret);
			Assert::IsTrue(current.isEqual(currentExpected));
			Assert::IsTrue(minus.isEqual(minusExpected));
			Assert::IsTrue(plus.isEqual(plusExpected));
/***************************************************************/
			ret = test.nextIteration(0.97, 0.96);
			state = test.getState();
			current = test.getCurrentTransformation(0);
			minus = test.getNextEvalMinus(0);
			plus = test.getNextEvalPlus(0);

			currentExpected = sC.getInitialH(688);
			minusExpected = sC.getInitialH(661);
			plusExpected = sC.getInitialH(715);						

			Assert::IsTrue(state==SA::STATE::HC);
			Assert::IsTrue(ret);
			Assert::IsTrue(current.isEqual(currentExpected));
			Assert::IsTrue(minus.isEqual(minusExpected));
			Assert::IsTrue(plus.isEqual(plusExpected));

/***************************************************************/
			ret = test.nextIteration(0.95, 0.94);
			state = test.getState();
			current = test.getCurrentTransformation(0);
			minus = test.getNextEvalMinus(0);
			plus = test.getNextEvalPlus(0);

			currentExpected = sC.getInitialH(715);
			minusExpected = sC.getInitialH(706);
			plusExpected = sC.getInitialH(724);						

			Assert::IsTrue(state==SA::STATE::HC);
			Assert::IsTrue(ret);
			Assert::IsTrue(current.isEqual(currentExpected));
			Assert::IsTrue(minus.isEqual(minusExpected));
			Assert::IsTrue(plus.isEqual(plusExpected));
/***************************************************************/
			ret = test.nextIteration(0.93, 0.92);
			state = test.getState();
			current = test.getCurrentTransformation(0);
			minus = test.getNextEvalMinus(0);
			plus = test.getNextEvalPlus(0);

			currentExpected = sC.getInitialH(724);
			minusExpected = sC.getInitialH(721);
			plusExpected = sC.getInitialH(727);						

			Assert::IsTrue(state==SA::STATE::HC);
			Assert::IsTrue(ret);
			Assert::IsTrue(current.isEqual(currentExpected));
			Assert::IsTrue(minus.isEqual(minusExpected));
			Assert::IsTrue(plus.isEqual(plusExpected));
/***************************************************************/
			ret = test.nextIteration(0.91, 0.90);
			state = test.getState();	
			current = test.getCurrentTransformation(0);
			minus = test.getNextEvalMinus(0);
			plus = test.getNextEvalPlus(0);

			currentExpected = sC.getInitialH(727);
			minusExpected = sC.getInitialH(726);
			plusExpected = sC.getInitialH(728);						

			Assert::IsTrue(state==SA::STATE::HC);
			Assert::IsTrue(ret);
			Assert::IsTrue(current.isEqual(currentExpected));
			Assert::IsTrue(minus.isEqual(minusExpected));
			Assert::IsTrue(plus.isEqual(plusExpected));
/***************************************************************/
			ret = test.nextIteration(0.89, 0.88);
			state = test.getState();
			current = test.getCurrentTransformation(0);
			minus = test.getNextEvalMinus(0);
			plus = test.getNextEvalPlus(0);

			currentExpected = sC.getInitialH(728);
			minusExpected = sC.getInitialH(485);
			plusExpected = sC.getInitialH(728);	

			Assert::IsTrue(state==SA::STATE::HC);
			Assert::IsTrue(ret);
			Assert::IsTrue(current.isEqual(currentExpected));
			Assert::IsTrue(minus.isEqual(minusExpected));
			Assert::IsTrue(plus.isEqual(plusExpected));

		}

		[TestMethod]
		void TestMethod2()
		{

			int base = 3;
			int exp = 6;


			int value = (int)pow(3.0,6.0);
			
			Link kinChain[1];
			HomogeneTransformation *relative = new HomogeneTransformation[value];
			int *index = new int[value];

			float da = M_PI/4;
			float xr, yr, zr, rollr, pitchr, yawr;
			int ii =0;
			for(int x=-1;x<2; x++)
				for(int y=-1; y<2; y++)
					for(int z=-1; z<2; z++)
						for(int roll=-1; roll<2; roll++)
							for(int pitch=-1; pitch<2; pitch++)
								for(int yaw=-1; yaw<2; yaw++)
								{
									xr = x;
									yr = y;
									zr = z;
									rollr = roll*da;
									pitchr = pitch*da;
									yawr = yaw*da;
									relative[ii].init(rollr, pitchr, yawr, xr, yr, zr);
									index[ii] = 0;
									ii++;
									
								}
						
			int n = 1;
			HomogeneTransformation start, current, minus, plus;		
			HomogeneTransformation currentExpected, minusExpected, plusExpected;
			SA::STATE state;
			bool ret;

			SampleCameraConfiguration sC(kinChain, relative, index, n, value);
			SA test(&sC, 1, false);

			


			test.setCurrentTransformation(start,0);

			ret = test.nextIteration(1.0, 1.0);
			state = test.getState();
			Assert::IsTrue(state==SA::STATE::HC);
			Assert::IsTrue(ret);
			
			ret = test.nextIteration(1.0, 1.0);
			state = test.getState();
			Assert::IsTrue(state==SA::STATE::HC);
			Assert::IsTrue(ret);

			ret = test.nextIteration(1.0, 1.0);
			state = test.getState();
			Assert::IsTrue(state==SA::STATE::HC);
			Assert::IsTrue(ret);

			ret = test.nextIteration(1.0, 1.0);
			state = test.getState();
			Assert::IsTrue(state==SA::STATE::HC);
			Assert::IsTrue(ret);

			ret = test.nextIteration(1.0, 1.0);
			state = test.getState();
			Assert::IsTrue(state==SA::STATE::HC);
			Assert::IsTrue(ret);

			ret = test.nextIteration(1.0, 1.0);
			state = test.getState();
			Assert::IsTrue(state==SA::STATE::RJ);
			Assert::IsTrue(ret);
		}


		[TestMethod]
		void TestMethod3()
		{

			int base = 3;
			int exp = 6;


			int value = (int)pow(3.0,6.0);
			
			Link kinChain[1];
			HomogeneTransformation *relative = new HomogeneTransformation[value];
			int *index = new int[value];

			float da = M_PI/4;
			float xr, yr, zr, rollr, pitchr, yawr;
			int ii =0;
			for(int x=-1;x<2; x++)
				for(int y=-1; y<2; y++)
					for(int z=-1; z<2; z++)
						for(int roll=-1; roll<2; roll++)
							for(int pitch=-1; pitch<2; pitch++)
								for(int yaw=-1; yaw<2; yaw++)
								{
									xr = x;
									yr = y;
									zr = z;
									rollr = roll*da;
									pitchr = pitch*da;
									yawr = yaw*da;
									relative[ii].init(rollr, pitchr, yawr, xr, yr, zr);
									index[ii] = 0;
									ii++;
									
								}
						
			int n = 1;
			HomogeneTransformation start, current, minus, plus;		
			HomogeneTransformation currentExpected, minusExpected, plusExpected;
			SA::STATE state;
			bool ret;

			SampleCameraConfiguration sC(kinChain, relative, index, n, value);
			SA test(&sC, 1, false);

			


			test.setCurrentTransformation(start,0);

			ret = test.nextIteration(1.0, 1.0);
			state = test.getState();
			Assert::IsTrue(state==SA::STATE::HC);
			Assert::IsTrue(ret);
			
			ret = test.nextIteration(1.0, 1.0);
			state = test.getState();
			Assert::IsTrue(state==SA::STATE::HC);
			Assert::IsTrue(ret);

			ret = test.nextIteration(1.0, 1.0);
			state = test.getState();
			Assert::IsTrue(state==SA::STATE::HC);
			Assert::IsTrue(ret);

			ret = test.nextIteration(1.0, 1.0);
			state = test.getState();
			Assert::IsTrue(state==SA::STATE::HC);
			Assert::IsTrue(ret);

			ret = test.nextIteration(1.0, 1.0);
			state = test.getState();
			Assert::IsTrue(state==SA::STATE::HC);
			Assert::IsTrue(ret);

			ret = test.nextIteration(1.0, 1.0);
			state = test.getState();
			Assert::IsTrue(state==SA::STATE::RJ);
			Assert::IsTrue(ret);

			ret = test.nextIteration(1.0, 1.0);
			state = test.getState();
			Assert::IsTrue(state==SA::STATE::HC);
			Assert::IsTrue(ret);
			
			ret = test.nextIteration(1.0, 1.0);
			state = test.getState();
			Assert::IsTrue(state==SA::STATE::HC);
			Assert::IsTrue(ret);

			ret = test.nextIteration(1.0, 1.0);
			state = test.getState();
			Assert::IsTrue(state==SA::STATE::HC);
			Assert::IsTrue(ret);

			ret = test.nextIteration(1.0, 1.0);
			state = test.getState();
			Assert::IsTrue(state==SA::STATE::HC);
			Assert::IsTrue(ret);

			ret = test.nextIteration(1.0, 1.0);
			state = test.getState();
			Assert::IsTrue(state==SA::STATE::HC);
			Assert::IsTrue(ret);

			ret = test.nextIteration(1.0, 1.0);
			state = test.getState();
			Assert::IsTrue(state==SA::STATE::HC);
			Assert::IsTrue(ret);

			ret = test.nextIteration(1.0, 1.0);
			state = test.getState();
			Assert::IsTrue(state==SA::STATE::RJ);
			Assert::IsTrue(ret);
		}

		[TestMethod]
		void TestMethod4()
		{

			int base = 3;
			int exp = 6;


			int value = (int)pow(3.0,6.0);
			
			Link kinChain[1];
			HomogeneTransformation *relative = new HomogeneTransformation[value];
			int *index = new int[value];

			float da = M_PI/4;
			float xr, yr, zr, rollr, pitchr, yawr;
			int ii =0;
			for(int x=-1;x<2; x++)
				for(int y=-1; y<2; y++)
					for(int z=-1; z<2; z++)
						for(int roll=-1; roll<2; roll++)
							for(int pitch=-1; pitch<2; pitch++)
								for(int yaw=-1; yaw<2; yaw++)
								{
									xr = x;
									yr = y;
									zr = z;
									rollr = roll*da;
									pitchr = pitch*da;
									yawr = yaw*da;
									relative[ii].init(rollr, pitchr, yawr, xr, yr, zr);
									index[ii] = 0;
									ii++;
									
								}
						
			int n = 1;
			HomogeneTransformation start, current, minus, plus;		
			HomogeneTransformation currentExpected, minusExpected, plusExpected;
			SA::STATE state;
			bool ret;

			SampleCameraConfiguration sC(kinChain, relative, index, n, value);
			SA test(&sC, 1, false);

			


			test.setCurrentTransformation(start,0);
			
	

			ret = true;
			int ite = 1;
			do
			{
				ret = test.nextIteration(1.0, 1.0);
				state = test.getState();
				if(!ret)
				{
					break;
				}


				if((ite+1)%7==0)
				{
					Assert::IsTrue(state==SA::STATE::RJ);
				}else
				{
					Assert::IsTrue(state==SA::STATE::HC);
				}				
				ite++;
			}
			while(true);
			
			Assert::IsTrue(state==SA::STATE::FI);
		}


		[TestMethod]
		void TestMethod5()
		{

			int base = 3;
			int exp = 6;


			int value = (int)pow(3.0,6.0);
			
			Link kinChain[1];
			HomogeneTransformation *relative = new HomogeneTransformation[value];
			int *index = new int[value];

			float da = M_PI/4;
			float xr, yr, zr, rollr, pitchr, yawr;
			int ii =0;
			for(int x=-1;x<2; x++)
				for(int y=-1; y<2; y++)
					for(int z=-1; z<2; z++)
						for(int roll=-1; roll<2; roll++)
							for(int pitch=-1; pitch<2; pitch++)
								for(int yaw=-1; yaw<2; yaw++)
								{
									xr = x;
									yr = y;
									zr = z;
									rollr = roll*da;
									pitchr = pitch*da;
									yawr = yaw*da;
									relative[ii].init(rollr, pitchr, yawr, xr, yr, zr);
									index[ii] = 0;
									ii++;
									
								}
						
			int n = 1;
			HomogeneTransformation start, current, minus, plus;		
			HomogeneTransformation currentExpected, minusExpected, plusExpected;
			SA::STATE state;
			bool ret;

			SampleCameraConfiguration pSC[2];
			SampleCameraConfiguration sC0(kinChain, relative, index, n, value);
			SampleCameraConfiguration sC1(kinChain, relative, index, n, value);
			pSC[0] = sC0;
			pSC[1] = sC1;
			int nCameras = 2;
			SA test(pSC, nCameras, false);

			


			test.setCurrentTransformation(start,0);
			
	

			ret = true;
			int ite = 1;
			do
			{
				ret = test.nextIteration(1.0, 1.0);
				state = test.getState();
				if(!ret)
				{
					break;
				}


				if((ite+1)%(nCameras*6+1)==0)
				{
					Assert::IsTrue(state==SA::STATE::RJ);
				}else
				{
					Assert::IsTrue(state==SA::STATE::HC);
				}				
				ite++;
			}
			while(true);
			
			Assert::IsTrue(state==SA::STATE::FI);
		}

		[TestMethod]
		void TestMethod6()
		{

			int base = 3;
			int exp = 6;
			int nCameras = 6;

			int value = (int)pow(3.0,6.0);
			
			Link kinChain[1];
			HomogeneTransformation *relative = new HomogeneTransformation[value];
			int *index = new int[value];

			float da = M_PI/4;
			float xr, yr, zr, rollr, pitchr, yawr;
			int ii =0;
			for(int x=-1;x<2; x++)
				for(int y=-1; y<2; y++)
					for(int z=-1; z<2; z++)
						for(int roll=-1; roll<2; roll++)
							for(int pitch=-1; pitch<2; pitch++)
								for(int yaw=-1; yaw<2; yaw++)
								{
									xr = x;
									yr = y;
									zr = z;
									rollr = roll*da;
									pitchr = pitch*da;
									yawr = yaw*da;
									relative[ii].init(rollr, pitchr, yawr, xr, yr, zr);
									index[ii] = 0;
									ii++;
									
								}
						
			int n = 1;
			HomogeneTransformation start, current, minus, plus;		
			HomogeneTransformation currentExpected, minusExpected, plusExpected;
			SA::STATE state;
			bool ret;

			SampleCameraConfiguration* pSC = new SampleCameraConfiguration[nCameras];
			for(int i=0; i< nCameras; i++)
			{
				pSC[i] = SampleCameraConfiguration(kinChain, relative, index, n, value);
			}		
			SA test(pSC, nCameras, false);

			


			test.setCurrentTransformation(start,0);
			
	

			ret = true;
			int ite = 1;
			do
			{
				ret = test.nextIteration(1.0, 1.0);
				state = test.getState();
				if(!ret)
				{
					break;
				}


				if((ite+1)%(nCameras*6+1)==0)
				{
					Assert::IsTrue(state==SA::STATE::RJ);
				}else
				{
					Assert::IsTrue(state==SA::STATE::HC);
				}				
				ite++;
			}
			while(true);
			
			Assert::IsTrue(state==SA::STATE::FI);
			delete pSC;
		}


	};
}
