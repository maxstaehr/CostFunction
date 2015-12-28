#include "stdafx.h"

using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace	Microsoft::VisualStudio::TestTools::UnitTesting;

#include "../CFObject/KinChain.h"

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
			KinChain test(nPos);

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
			KinChain test(nPos);

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
			KinChain test(nPos);

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
			KinChain test(nPos);

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

	};
}
