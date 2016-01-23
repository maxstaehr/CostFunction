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

			int value = base;
			for(int i=0; i<exp; i++)
			{
				value *= base;
			}
			
			HomogeneTransformation kinChain[1];
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
									ii++;
									index[ii] = 0;
								}
						
			int n = 1;
			SampleCameraConfiguration sC(kinChain, relative, index, n);

			SA test(sC);

			
			HomogeneTransformation start, current, minus, plus;			
			bool ret;
			test.setCurrentTransformation(start);

			ret = test.nextIteration(DBL_MAX-2,DBL_MAX-1);
			current = test.getCurrentTransformation();
			minus = test.getNextEvalMinus();
			plus = test.getNextEvalPlus();
			Assert::IsFalse(ret);	
			Assert::IsTrue(current.isEqual(relative[364]);
			Assert::IsTrue(minus.isEqual(relative[364]);
			Assert::IsTrue(plus.isEqual(relative[607]);



			//ret = test.nextIteration(DBL_MAX-1,DBL_MAX-1, finish_m, finish_p);
			//Assert::IsFalse(ret);	
			////Assert::IsTrue(finish.isEqual(relative[607]);
		}
	};
}
