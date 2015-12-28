#include "stdafx.h"
#define _USE_MATH_DEFINES 
#include <cmath>

using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace	Microsoft::VisualStudio::TestTools::UnitTesting;

#include "../CFObject/PCL.h"
#include "../CFObject/Rotation.h"
#include "../CFObject/HomogeneTransformation.h"


namespace TestDriver
{
	[TestClass]
	public ref class PCLTest
	{
	public: 

		[TestMethod]
		void TestConstructor()
		{
			int nV = 3;
			int nF = 1;

			float x[] = {0, 1, 2};
			float y[] = {3, 4, 5};
			float z[] = {6, 7, 8};

			int fx[] = {9};
			int fy[] = {10};
			int fz[] = {11};

			PCL test(nV, nF, x, y, z, fx, fy, fz);

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
		void TestRotatePCL()
		{
			int nV = 3;
			int nF = 1;

			float x[] = {0, 0, 1};
			float y[] = {0, 1, 0};
			float z[] = {0, 0, 0};

			float xr[] = {0, -1, 0};
			float yr[] = {0, 0, 1};
			float zr[] = {0, 0, 0};

			int fx[] = {0};
			int fy[] = {0};
			int fz[] = {0};

			PCL test(nV, nF, x, y, z, fx, fy, fz);
			Rotation rot;
			rot.initYaw(M_PI/2.0);
			test.rotate(rot);


			Assert::AreEqual(test.getx()[0], xr[0], 1e-3f);
			Assert::AreEqual(test.getx()[1], xr[1], 1e-3f);
			Assert::AreEqual(test.getx()[2], xr[2], 1e-3f);

			Assert::AreEqual(test.gety()[0], yr[0], 1e-3f);
			Assert::AreEqual(test.gety()[1], yr[1], 1e-3f);
			Assert::AreEqual(test.gety()[2], yr[2], 1e-3f);

			Assert::AreEqual(test.getz()[0], zr[0], 1e-3f);
			Assert::AreEqual(test.getz()[1], zr[1], 1e-3f);
			Assert::AreEqual(test.getz()[2], zr[2], 1e-3f);

			Assert::AreEqual(test.getfx()[0], fx[0]);

			Assert::AreEqual(test.getfy()[0], fy[0]);

			Assert::AreEqual(test.getfz()[0], fz[0]);


		}


		[TestMethod]
		void TestTransformPCL()
		{
			int nV = 3;
			int nF = 1;

			float x[] = {0, 0, 1};
			float y[] = {0, 1, 0};
			float z[] = {0, 0, 0};

			float xr[] = {2, 1, 2};
			float yr[] = {2, 2, 3};
			float zr[] = {2, 2, 2};



			int fx[] = {0};
			int fy[] = {0};
			int fz[] = {0};

			PCL test(nV, nF, x, y, z, fx, fy, fz);

			HomogeneTransformation trans;
			trans.init(0, 0, M_PI/2.0, 2, 2, 2);			
			test.transform(trans);

			Assert::AreEqual(test.getx()[0], xr[0], 1e-3f);
			Assert::AreEqual(test.getx()[1], xr[1], 1e-3f);
			Assert::AreEqual(test.getx()[2], xr[2], 1e-3f);

			Assert::AreEqual(test.gety()[0], yr[0], 1e-3f);
			Assert::AreEqual(test.gety()[1], yr[1], 1e-3f);
			Assert::AreEqual(test.gety()[2], yr[2], 1e-3f);

			Assert::AreEqual(test.getz()[0], zr[0], 1e-3f);
			Assert::AreEqual(test.getz()[1], zr[1], 1e-3f);
			Assert::AreEqual(test.getz()[2], zr[2], 1e-3f);

			Assert::AreEqual(test.getfx()[0], fx[0]);

			Assert::AreEqual(test.getfy()[0], fy[0]);

			Assert::AreEqual(test.getfz()[0], fz[0]);
		}
	};
}
