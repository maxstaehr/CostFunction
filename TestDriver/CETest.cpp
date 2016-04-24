#include "stdafx.h"

#define _USE_MATH_DEFINES 
#include <cmath>

using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace	Microsoft::VisualStudio::TestTools::UnitTesting;

#include "../CFObject/CE.h"

namespace TestDriver
{
	[TestClass]
	public ref class CETest
	{
	public: 
		[TestMethod]
		void TestMethod1()
		{						
			Link kinChain[1];
			HomogeneTransformation relative[2];
			int index[] = {0, 0};	
			int n = 1;
			int value = 2;

			SampleCameraConfiguration sC(kinChain, relative, index, n, value);

			CE test(&sC, 1, false);
			Assert::AreEqual(test.getM(), 2);
			Assert::AreEqual(test.getN(), 1);
			Assert::AreEqual(0, test.getIndices(0,0));
			Assert::AreEqual(1, test.getIndices(1,0));		
		}

		[TestMethod]
		void TestMethod2()
		{						
			Link kinChain[1];
			HomogeneTransformation relative[2];
			int index[] = {0, 0};	
			int n = 1;
			int value = 2;

			SampleCameraConfiguration sC0(kinChain, relative, index, n, value);
			SampleCameraConfiguration sC1(kinChain, relative, index, n, value);
			SampleCameraConfiguration aS[2];
			aS[0] = sC0;
			aS[1] = sC1;

			CE test(aS, 2, false);
			Assert::AreEqual(test.getM(), 4);
			Assert::AreEqual(test.getN(), 2);

			Assert::AreEqual(0, test.getIndices(0,0));
			Assert::AreEqual(1, test.getIndices(1,0));		
			Assert::AreEqual(0, test.getIndices(2,0));
			Assert::AreEqual(1, test.getIndices(3,0));	

			Assert::AreEqual(0, test.getIndices(0,1));
			Assert::AreEqual(0, test.getIndices(1,1));		
			Assert::AreEqual(1, test.getIndices(2,1));
			Assert::AreEqual(1, test.getIndices(3,1));	
		}

		[TestMethod]
		void TestMethod3()
		{						
			Link kinChain[1];
			HomogeneTransformation relative[2];
			int index[] = {0, 0};	
			int n = 1;
			int value = 2;

			SampleCameraConfiguration sC0(kinChain, relative, index, n, value);
			SampleCameraConfiguration sC1(kinChain, relative, index, n, value);
			SampleCameraConfiguration sC2(kinChain, relative, index, n, value);
			SampleCameraConfiguration aS[3];
			aS[0] = sC0;
			aS[1] = sC1;
			aS[2] = sC2;

			CE test(aS, 3, false);
			Assert::AreEqual(test.getM(), 8);
			Assert::AreEqual(test.getN(), 3);

			Assert::AreEqual(0, test.getIndices(0,0));
			Assert::AreEqual(1, test.getIndices(1,0));		
			Assert::AreEqual(0, test.getIndices(2,0));
			Assert::AreEqual(1, test.getIndices(3,0));	
			Assert::AreEqual(0, test.getIndices(4,0));
			Assert::AreEqual(1, test.getIndices(5,0));		
			Assert::AreEqual(0, test.getIndices(6,0));
			Assert::AreEqual(1, test.getIndices(7,0));	

			Assert::AreEqual(0, test.getIndices(0,1));
			Assert::AreEqual(0, test.getIndices(1,1));		
			Assert::AreEqual(1, test.getIndices(2,1));
			Assert::AreEqual(1, test.getIndices(3,1));	
			Assert::AreEqual(0, test.getIndices(4,1));
			Assert::AreEqual(0, test.getIndices(5,1));		
			Assert::AreEqual(1, test.getIndices(6,1));
			Assert::AreEqual(1, test.getIndices(7,1));

			Assert::AreEqual(0, test.getIndices(0,2));
			Assert::AreEqual(0, test.getIndices(1,2));		
			Assert::AreEqual(0, test.getIndices(2,2));
			Assert::AreEqual(0, test.getIndices(3,2));	
			Assert::AreEqual(1, test.getIndices(4,2));
			Assert::AreEqual(1, test.getIndices(5,2));		
			Assert::AreEqual(1, test.getIndices(6,2));
			Assert::AreEqual(1, test.getIndices(7,2));	
		}


		[TestMethod]
		void TestMethod4()
		{						
			Link kinChain[1];
			HomogeneTransformation relative[2];
			relative[0].init(0, 0, 0, 1.0, 0, 0);
			relative[1].init(0, 0, 0, 2.0, 0, 0);
			int index[] = {0, 0};	
			int n = 1;
			int value = 2;

			SampleCameraConfiguration sC(kinChain, relative, index, n, value);

			CE test(&sC, 1, false);
			Assert::AreEqual(test.getM(), 2);
			Assert::AreEqual(test.getN(), 1);
			Assert::AreEqual(0, test.getIndices(0,0));
			Assert::AreEqual(1, test.getIndices(1,0));		

			Assert::IsTrue(test.nextIteration(0, 0));
			Assert::AreEqual(1.0f, test.getNextEvalMinus(0).getH()[3]);
			Assert::AreEqual(2.0f, test.getNextEvalPlus(0).getH()[3]);
			Assert::IsFalse(test.nextIteration(0, 0));
		}

		[TestMethod]
		void TestMethod5()
		{						
			Link kinChain[1];
			HomogeneTransformation relative[4];
			relative[0].init(0, 0, 0, 1.0, 0, 0);
			relative[1].init(0, 0, 0, 2.0, 0, 0);
			relative[2].init(0, 0, 0, 3.0, 0, 0);
			relative[3].init(0, 0, 0, 4.0, 0, 0);
			int index[] = {0, 0, 0, 0};	
			int n = 1;
			int value = 4;

			SampleCameraConfiguration sC(kinChain, relative, index, n, value);

			CE test(&sC, 1, false);
			Assert::AreEqual(test.getM(), 4);
			Assert::AreEqual(test.getN(), 1);
			Assert::AreEqual(0, test.getIndices(0,0));
			Assert::AreEqual(1, test.getIndices(1,0));		

			Assert::IsTrue(test.nextIteration(0, 0));
			Assert::AreEqual(1.0f, test.getNextEvalMinus(0).getH()[3]);
			Assert::AreEqual(2.0f, test.getNextEvalPlus(0).getH()[3]);
			Assert::IsTrue(test.nextIteration(0, 0));
			Assert::AreEqual(3.0f, test.getNextEvalMinus(0).getH()[3]);
			Assert::AreEqual(4.0f, test.getNextEvalPlus(0).getH()[3]);
			Assert::IsFalse(test.nextIteration(0, 0));
		}

		[TestMethod]
		void TestMethod6()
		{						
			Link kinChain[1];
			HomogeneTransformation relative0[2];
			relative0[0].init(0, 0, 0, 1.0, 0, 0);
			relative0[1].init(0, 0, 0, 2.0, 0, 0);

			HomogeneTransformation relative1[2];
			relative1[0].init(0, 0, 0, 3.0, 0, 0);
			relative1[1].init(0, 0, 0, 4.0, 0, 0);

			int index[] = {0, 0, 0, 0};	
			int n = 1;
			int value = 2;

			SampleCameraConfiguration sC0(kinChain, relative0, index, n, value);
			SampleCameraConfiguration sC1(kinChain, relative1, index, n, value);
			SampleCameraConfiguration aS[2];
			aS[0] = sC0;
			aS[1] = sC1;

			CE test(aS, 2, false);

			Assert::AreEqual(test.getM(), 4);
			Assert::AreEqual(test.getN(), 2);
	

			Assert::IsTrue(test.nextIteration(0, 0));
			Assert::AreEqual(1.0f, test.getNextEvalMinus(0).getH()[3]);
			Assert::AreEqual(2.0f, test.getNextEvalPlus(0).getH()[3]);
			Assert::AreEqual(3.0f, test.getNextEvalMinus(1).getH()[3]);
			Assert::AreEqual(3.0f, test.getNextEvalPlus(1).getH()[3]);

			Assert::IsTrue(test.nextIteration(0, 0));
			Assert::AreEqual(1.0f, test.getNextEvalMinus(0).getH()[3]);
			Assert::AreEqual(2.0f, test.getNextEvalPlus(0).getH()[3]);
			Assert::AreEqual(4.0f, test.getNextEvalMinus(1).getH()[3]);
			Assert::AreEqual(4.0f, test.getNextEvalPlus(1).getH()[3]);

			Assert::IsFalse(test.nextIteration(0, 0));
		}
	};
}
