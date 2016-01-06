#include "stdafx.h"

using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace	Microsoft::VisualStudio::TestTools::UnitTesting;

#include "../CFObject/BoundingBox.h"

#define _USE_MATH_DEFINES 
#include <cmath>

namespace TestDriver
{
	[TestClass]
	public ref class BoundingBoxTest
	{
	public: 
		[TestMethod]
		void TestConstructor()
		{
			HomogeneTransformation trans;
			BoundingBox bb(trans, 1, 2, 3);

			Assert::AreEqual(bb.getH().getH()[0], 1.0f);
			Assert::AreEqual(bb.getH().getH()[1], 0.0f);
			Assert::AreEqual(bb.getH().getH()[2], 0.0f);
			Assert::AreEqual(bb.getH().getH()[3], 0.0f);

			Assert::AreEqual(bb.getH().getH()[4], 0.0f);
			Assert::AreEqual(bb.getH().getH()[5], 1.0f);
			Assert::AreEqual(bb.getH().getH()[6], 0.0f);
			Assert::AreEqual(bb.getH().getH()[7], 0.0f);

			Assert::AreEqual(bb.getH().getH()[8], 0.0f);
			Assert::AreEqual(bb.getH().getH()[9], 0.0f);
			Assert::AreEqual(bb.getH().getH()[10], 1.0f);
			Assert::AreEqual(bb.getH().getH()[11], 0.0f);

			Assert::AreEqual(bb.getH().getH()[12], 0.0f);
			Assert::AreEqual(bb.getH().getH()[13], 0.0f);
			Assert::AreEqual(bb.getH().getH()[14], 0.0f);
			Assert::AreEqual(bb.getH().getH()[15], 1.0f);

			Assert::AreEqual(bb.getInvH().getH()[0], 1.0f);
			Assert::AreEqual(bb.getInvH().getH()[1], 0.0f);
			Assert::AreEqual(bb.getInvH().getH()[2], 0.0f);
			Assert::AreEqual(bb.getInvH().getH()[3], 0.0f);

			Assert::AreEqual(bb.getInvH().getH()[4], 0.0f);
			Assert::AreEqual(bb.getInvH().getH()[5], 1.0f);
			Assert::AreEqual(bb.getInvH().getH()[6], 0.0f);
			Assert::AreEqual(bb.getInvH().getH()[7], 0.0f);

			Assert::AreEqual(bb.getInvH().getH()[8], 0.0f);
			Assert::AreEqual(bb.getInvH().getH()[9], 0.0f);
			Assert::AreEqual(bb.getInvH().getH()[10], 1.0f);
			Assert::AreEqual(bb.getInvH().getH()[11], 0.0f);

			Assert::AreEqual(bb.getInvH().getH()[12], 0.0f);
			Assert::AreEqual(bb.getInvH().getH()[13], 0.0f);
			Assert::AreEqual(bb.getInvH().getH()[14], 0.0f);
			Assert::AreEqual(bb.getInvH().getH()[15], 1.0f);

		}

		[TestMethod]
		void TestSetH()
		{
			HomogeneTransformation trans;
			BoundingBox bb(trans, 1, 2, 3);

			float p[3];
			p[0] = 0;
			p[1] = 0;
			p[2] = 0;
			Assert::IsFalse(bb.isInBoundingBox(p));

			p[0] = 1.0;
			p[1] = 2.0;
			p[2] = 3.0;
			Assert::IsFalse(bb.isInBoundingBox(p));

		}

		[TestMethod]
		void TestIsInBoundingBox1()
		{
			HomogeneTransformation trans;
			BoundingBox bb(trans, 1, 2, 3);

			float EPS = 1e-5f;
			float p[3];
			p[0] = EPS;
			p[1] = EPS;
			p[2] = EPS;
			Assert::IsTrue(bb.isInBoundingBox(p));

			p[0] = 1.0-EPS;
			p[1] = 2.0-EPS;
			p[2] = 3.0-EPS;
			Assert::IsTrue(bb.isInBoundingBox(p));
		}


		[TestMethod]
		void TestIsInBoundingBox2()
		{
			HomogeneTransformation trans;
			BoundingBox bb(trans, 1, 2, 3);
			trans.init(M_PI/2.0, 0 , 0, 0, 0, 0);
			bb.setH(trans);

			float EPS = 1e-5f;
			float p[3];
			p[0] = EPS;
			p[1] = EPS;
			p[2] = -EPS;
			Assert::IsTrue(bb.isInBoundingBox(p));

			p[0] = 1.0-EPS;
			p[1] = 3.0-EPS;
			p[2] = -(2.0-EPS);
			Assert::IsTrue(bb.isInBoundingBox(p));
		}
	};
}
