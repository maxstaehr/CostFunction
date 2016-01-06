#include "stdafx.h"

using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace	Microsoft::VisualStudio::TestTools::UnitTesting;

#include "../CFObject/Link.h"
#include <iostream>

namespace TestDriver
{
	[TestClass]
	public ref class LinkTest
	{
	public: 
		[TestMethod]
		void TestConstructor()
		{
			Link link;
			HomogeneTransformation test;
			float temp1[16];
			float temp2[16] ={	0.0f, 1.0f, 2.0f, 3.0f, 
								4.0f, 5.0f, 6.0f, 7.0f, 
								8.0f, 9.0f, 10.0f, 11.0f,
								12.0f, 13.0f, 14.0f, 15.0f};


			

			Assert::AreEqual(test.getH()[0], 1.0f);
			Assert::AreEqual(test.getH()[1], 0.0f);
			Assert::AreEqual(test.getH()[2], 0.0f);
			Assert::AreEqual(test.getH()[3], 0.0f);

			Assert::AreEqual(test.getH()[4], 0.0f);
			Assert::AreEqual(test.getH()[5], 1.0f);
			Assert::AreEqual(test.getH()[6], 0.0f);
			Assert::AreEqual(test.getH()[7], 0.0f);

			Assert::AreEqual(test.getH()[8], 0.0f);
			Assert::AreEqual(test.getH()[9], 0.0f);
			Assert::AreEqual(test.getH()[10], 1.0f);
			Assert::AreEqual(test.getH()[11], 0.0f);

			Assert::AreEqual(test.getH()[12], 0.0f);
			Assert::AreEqual(test.getH()[13], 0.0f);
			Assert::AreEqual(test.getH()[14], 0.0f);
			Assert::AreEqual(test.getH()[15], 1.0f);

						
			test.setH(temp2);
			link.setH(test);

			

			Assert::AreEqual(link.getH().getH()[0], temp2[0]);
			Assert::AreEqual(link.getH().getH()[1], temp2[1]);
			Assert::AreEqual(link.getH().getH()[2], temp2[2]);
			Assert::AreEqual(link.getH().getH()[3], temp2[3]);

			Assert::AreEqual(link.getH().getH()[4], temp2[4]);
			Assert::AreEqual(link.getH().getH()[5], temp2[5]);
			Assert::AreEqual(link.getH().getH()[6], temp2[6]);
			Assert::AreEqual(link.getH().getH()[7], temp2[7]);

			Assert::AreEqual(link.getH().getH()[8], temp2[8]);
			Assert::AreEqual(link.getH().getH()[9], temp2[9]);
			Assert::AreEqual(link.getH().getH()[10],temp2[10]);
			Assert::AreEqual(link.getH().getH()[11],temp2[11]);

			Assert::AreEqual(link.getH().getH()[12], temp2[12]);
			Assert::AreEqual(link.getH().getH()[13], temp2[13]);
			Assert::AreEqual(link.getH().getH()[14], temp2[14]);
			Assert::AreEqual(link.getH().getH()[15], temp2[15]);

		}

		[TestMethod]
		void TestAddPCL0()
		{
			Link link;

			Assert::AreEqual(link.getnPCL(), 0);
			Assert::IsTrue(link.getPCL()==NULL);
		}

		[TestMethod]
		void TestAddPCL1()
		{
			Link link;
			int nV = 3;
			int nF = 1;
			float x[] = {0, 1, 2};
			float y[] = {3, 4, 5};
			float z[] = {6, 7, 8};
			int fx[] = {9};
			int fy[] = {10};
			int fz[] = {11};

			PCL test1(nV, nF, x, y, z, fx, fy, fz);
			link.addPCL(test1);

			Assert::AreEqual(link.getnPCL(), 1);
			Assert::IsTrue(link.getPCL() != NULL);

			PCL test = link.getPCL()[0];

			Assert::AreEqual(test.getx()[0], x[0], 1e-3f);
			Assert::AreEqual(test.getx()[1], x[1], 1e-3f);
			Assert::AreEqual(test.getx()[2], x[2], 1e-3f);

			Assert::AreEqual(test.gety()[0], y[0], 1e-3f);
			Assert::AreEqual(test.gety()[1], y[1], 1e-3f);
			Assert::AreEqual(test.gety()[2], y[2], 1e-3f);

			Assert::AreEqual(test.getz()[0], z[0], 1e-3f);
			Assert::AreEqual(test.getz()[1], z[1], 1e-3f);
			Assert::AreEqual(test.getz()[2], z[2], 1e-3f);

			Assert::AreEqual(test.getfx()[0], fx[0]);

			Assert::AreEqual(test.getfy()[0], fy[0]);

			Assert::AreEqual(test.getfz()[0], fz[0]);


		}


		[TestMethod]
		void TestAddBoundingBox1()
		{
			float ta[] = EYE_16;
			Link link;
			HomogeneTransformation trans;
			BoundingBox test1(trans, 1, 2, 3);
			link.addBB(test1);

			Assert::AreEqual(link.getnBB(), 1);
			Assert::IsTrue(link.getBB() != NULL);

			BoundingBox test = link.getBB()[0];

			for(int i=0; i<16; i++)
			{
				Assert::AreEqual(test.getIH().getH()[i], ta[i], 1e-3f);
				Assert::AreEqual(test.getH().getH()[i], ta[i], 1e-3f);
				Assert::AreEqual(test.getInvH().getH()[i], ta[i], 1e-3f);
			}
			Assert::AreEqual(test.getXDim(), 1.0f);
			Assert::AreEqual(test.getYDim(), 2.0f);
			Assert::AreEqual(test.getZDim(), 3.0f);
		}


		[TestMethod]
		void TestAddBoundingBox2()
		{
			float ta1[] = EYE_16;
			float ta2[] = EYE_16;
			float ta2inv[] = {	0.8000, -0.2000, -0.2000, -0.2000,
								-0.2000, 0.8000, -0.2000, -0.2000,
								-0.2000, -0.2000, 0.8000, -0.2000,
								-0.2000, -0.2000, -0.2000, 0.8000};
			for(int i=0; i<16; i++)
			{
				ta2[i] = ta2[i] + 1.0f;
			}

			Link link;

			HomogeneTransformation trans1;
			trans1.setH(ta1);

			HomogeneTransformation trans2;
			trans2.setH(ta2);

			BoundingBox in1(trans1, 1, 2, 3);
			BoundingBox in2(trans2, 4, 5, 6);

			link.addBB(in1);
			link.addBB(in2);

			Assert::AreEqual(link.getnBB(), 2);
			Assert::IsTrue(link.getBB() != NULL);

			BoundingBox test1 = link.getBB()[0];
			BoundingBox test2 = link.getBB()[1];

			for(int i=0; i<16; i++)
			{
				Assert::AreEqual(test1.getIH().getH()[i],	ta1[i], 1e-3f);
				Assert::AreEqual(test1.getH().getH()[i],	ta1[i], 1e-3f);
				Assert::AreEqual(test1.getInvH().getH()[i],	ta1[i], 1e-3f);

				Assert::AreEqual(test2.getIH().getH()[i],	ta2[i], 1e-3f);
				Assert::AreEqual(test2.getH().getH()[i],	ta2[i], 1e-3f);
				Assert::AreEqual(test2.getInvH().getH()[i],	ta2inv[i], 1e-3f);

			}
			Assert::AreEqual(test1.getXDim(), 1.0f);
			Assert::AreEqual(test1.getYDim(), 2.0f);
			Assert::AreEqual(test1.getZDim(), 3.0f);

			Assert::AreEqual(test2.getXDim(), 4.0f);
			Assert::AreEqual(test2.getYDim(), 5.0f);
			Assert::AreEqual(test2.getZDim(), 6.0f);
		}

		[TestMethod]
		void TestAddPCL2()
		{
			Link link;
			int nV = 3;
			int nF = 1;
			float x1[] = {0, 1, 2};
			float y1[] = {3, 4, 5};
			float z1[] = {6, 7, 8};
			int fx1[] = {9};
			int fy1[] = {10};
			int fz1[] = {11};

			float x2[] = {10, 11, 12};
			float y2[] = {13, 14, 15};
			float z2[] = {16, 17, 18};
			int fx2[] = {19};
			int fy2[] = {110};
			int fz2[] = {111};

			PCL test1(nV, nF, x1, y1, z1, fx1, fy1, fz1);
			PCL test2(nV, nF, x2, y2, z2, fx2, fy2, fz2);

			link.addPCL(test1);
			link.addPCL(test2);

			Assert::AreEqual(link.getnPCL(), 2);
			Assert::IsTrue(link.getPCL() != NULL);

			PCL test = link.getPCL()[0];

			Assert::AreEqual(test.getx()[0], x1[0], 1e-3f);
			Assert::AreEqual(test.getx()[1], x1[1], 1e-3f);
			Assert::AreEqual(test.getx()[2], x1[2], 1e-3f);

			Assert::AreEqual(test.gety()[0], y1[0], 1e-3f);
			Assert::AreEqual(test.gety()[1], y1[1], 1e-3f);
			Assert::AreEqual(test.gety()[2], y1[2], 1e-3f);

			Assert::AreEqual(test.getz()[0], z1[0], 1e-3f);
			Assert::AreEqual(test.getz()[1], z1[1], 1e-3f);
			Assert::AreEqual(test.getz()[2], z1[2], 1e-3f);

			Assert::AreEqual(test.getfx()[0], fx1[0]);

			Assert::AreEqual(test.getfy()[0], fy1[0]);

			Assert::AreEqual(test.getfz()[0], fz1[0]);

			test = link.getPCL()[1];

			Assert::AreEqual(test.getx()[0], x2[0], 1e-3f);
			Assert::AreEqual(test.getx()[1], x2[1], 1e-3f);
			Assert::AreEqual(test.getx()[2], x2[2], 1e-3f);

			Assert::AreEqual(test.gety()[0], y2[0], 1e-3f);
			Assert::AreEqual(test.gety()[1], y2[1], 1e-3f);
			Assert::AreEqual(test.gety()[2], y2[2], 1e-3f);

			Assert::AreEqual(test.getz()[0], z2[0], 1e-3f);
			Assert::AreEqual(test.getz()[1], z2[1], 1e-3f);
			Assert::AreEqual(test.getz()[2], z2[2], 1e-3f);

			Assert::AreEqual(test.getfx()[0], fx2[0]);

			Assert::AreEqual(test.getfy()[0], fy2[0]);

			Assert::AreEqual(test.getfz()[0], fz2[0]);

		}


		[TestMethod]
		void TestTransformPCL1()
		{
			Link link;
			int nV = 3;
			int nF = 1;
			float x1[] = {0, 1, 2};
			float y1[] = {3, 4, 5};
			float z1[] = {6, 7, 8};

			float x2[] = {10, 11, 12};
			float y2[] = {13, 14, 15};
			float z2[] = {16, 17, 18};

			int fx[] = {9};
			int fy[] = {10};
			int fz[] = {11};

			PCL test1(nV, nF, x1, y1, z1, fx, fy, fz);
			link.addPCL(test1);

			Assert::AreEqual(link.getnPCL(), 1);
			Assert::IsTrue(link.getPCL() != NULL);

			PCL test = link.getPCL()[0];

			Assert::AreEqual(test.getx()[0], x1[0], 1e-3f);
			Assert::AreEqual(test.getx()[1], x1[1], 1e-3f);
			Assert::AreEqual(test.getx()[2], x1[2], 1e-3f);

			Assert::AreEqual(test.gety()[0], y1[0], 1e-3f);
			Assert::AreEqual(test.gety()[1], y1[1], 1e-3f);
			Assert::AreEqual(test.gety()[2], y1[2], 1e-3f);

			Assert::AreEqual(test.getz()[0], z1[0], 1e-3f);
			Assert::AreEqual(test.getz()[1], z1[1], 1e-3f);
			Assert::AreEqual(test.getz()[2], z1[2], 1e-3f);

			Assert::AreEqual(test.getfx()[0], fx[0]);
			Assert::AreEqual(test.getfy()[0], fy[0]);
			Assert::AreEqual(test.getfz()[0], fz[0]);

			HomogeneTransformation trans;
			trans.init(0,0,0,10,10,10);
			link.setH(trans);

			PCL test2 = link.getPCL()[0];

			Assert::AreEqual(test2.getx()[0], x2[0], 1e-3f);
			Assert::AreEqual(test2.getx()[1], x2[1], 1e-3f);
			Assert::AreEqual(test2.getx()[2], x2[2], 1e-3f);

			Assert::AreEqual(test2.gety()[0], y2[0], 1e-3f);
			Assert::AreEqual(test2.gety()[1], y2[1], 1e-3f);
			Assert::AreEqual(test2.gety()[2], y2[2], 1e-3f);

			Assert::AreEqual(test2.getz()[0], z2[0], 1e-3f);
			Assert::AreEqual(test2.getz()[1], z2[1], 1e-3f);
			Assert::AreEqual(test2.getz()[2], z2[2], 1e-3f);

			Assert::AreEqual(test2.getfx()[0], fx[0]);
			Assert::AreEqual(test2.getfy()[0], fy[0]);
			Assert::AreEqual(test2.getfz()[0], fz[0]);
		}

	};
}
