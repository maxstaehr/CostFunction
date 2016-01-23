#include "stdafx.h"
#define _USE_MATH_DEFINES 
#include <cmath>

using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace	Microsoft::VisualStudio::TestTools::UnitTesting;

#include "../CFObject/Cluster.h"

namespace TestDriver
{
	[TestClass]
	public ref class ClusterTest
	{
	public: 
		[TestMethod]
		void TestConstructor()
		{
			Cluster c(5);
			Assert::AreEqual(5, c.getMaxSize());
		}


		[TestMethod]
		void TestCopyConstructor()
		{
			Cluster c1(1);
			c1.setX(0,1.0f);
			c1.setY(0,2.0f);
			c1.setZ(0,3.0f);
			Cluster c2(c1);

			Assert::AreEqual(1, c1.getMaxSize());
			Assert::AreEqual(1.0f, c1.getX()[0]);
			Assert::AreEqual(2.0f, c1.getY()[0]);
			Assert::AreEqual(3.0f, c1.getZ()[0]);

			Assert::AreEqual(1, c2.getMaxSize());
			Assert::AreEqual(1.0f, c2.getX()[0]);
			Assert::AreEqual(2.0f, c2.getY()[0]);
			Assert::AreEqual(3.0f, c2.getZ()[0]);

			Assert::IsFalse(c1.getX() == c2.getX());
			Assert::IsFalse(c1.getY() == c2.getY());
			Assert::IsFalse(c1.getZ() == c2.getZ());
			Assert::IsFalse(c1.getCentroid() == c2.getCentroid());
		}

		[TestMethod]
		void TestEqualOperator()
		{
			Cluster c1(1);
			c1.setX(0,1.0f);
			c1.setY(0,2.0f);
			c1.setZ(0,3.0f);
			Cluster c2 =c1 ;

			Assert::AreEqual(1, c1.getMaxSize());
			Assert::AreEqual(1.0f, c1.getX()[0]);
			Assert::AreEqual(2.0f, c1.getY()[0]);
			Assert::AreEqual(3.0f, c1.getZ()[0]);

			Assert::AreEqual(1, c2.getMaxSize());
			Assert::AreEqual(1.0f, c2.getX()[0]);
			Assert::AreEqual(2.0f, c2.getY()[0]);
			Assert::AreEqual(3.0f, c2.getZ()[0]);

			Assert::IsFalse(c1.getX() == c2.getX());
			Assert::IsFalse(c1.getY() == c2.getY());
			Assert::IsFalse(c1.getZ() == c2.getZ());
			Assert::IsFalse(c1.getCentroid() == c2.getCentroid());
		}

		[TestMethod]
		void TestCalcCentroid1()
		{
			Cluster c1(1);
			c1.setX(0,1.0f);
			c1.setY(0,2.0f);
			c1.setZ(0,3.0f);
			c1.calculate();

			Assert::AreEqual(1, c1.getMaxSize());
			Assert::AreEqual(1.0f, c1.getX()[0]);
			Assert::AreEqual(2.0f, c1.getY()[0]);
			Assert::AreEqual(3.0f, c1.getZ()[0]);
			Assert::AreEqual(1.0f, c1.getCentroid()[0]);
			Assert::AreEqual(2.0f, c1.getCentroid()[1]);
			Assert::AreEqual(3.0f, c1.getCentroid()[2]);
		}

		[TestMethod]
		void TestCalcCentroid2()
		{
			Cluster c1(2);
			c1.setX(0,1.0f);
			c1.setY(0,2.0f);
			c1.setZ(0,3.0f);

			c1.setX(1,	0.5f);
			c1.setY(1,	0.5f);
			c1.setZ(1,	0.5f);

			c1.calculate();

			Assert::AreEqual(2, c1.getMaxSize());
			Assert::AreEqual(1.0f, c1.getX()[0]);
			Assert::AreEqual(2.0f, c1.getY()[0]);
			Assert::AreEqual(3.0f, c1.getZ()[0]);

			Assert::AreEqual(0.5f, c1.getX()[1]);
			Assert::AreEqual(0.5f, c1.getY()[1]);
			Assert::AreEqual(0.5f, c1.getZ()[1]);

			Assert::AreEqual(0.75f, c1.getCentroid()[0]);
			Assert::AreEqual(1.25f, c1.getCentroid()[1]);
			Assert::AreEqual(1.75f, c1.getCentroid()[2]);
		}




		[TestMethod]
		void TestCalcDist2Model()
		{
			Cluster c1(1);
			c1.setX(0,-0.168624513405465f);
			c1.setY(0,-0.767965926432605f);
			c1.setZ(0,1.474260804595051f);

			

			c1.calculate();
			Assert::AreEqual(0.728605772778698f, c1.getMeanDistance2Model(), 1e-5f);
			Assert::AreEqual(0.0, c1.getHumanProb(), 1e-3);

		}

		[TestMethod]
		void TestCalcDist2Model2()
		{
			Cluster c1(1);
			c1.setX(0, 0.0f);
			c1.setY(0, 1.207813763297230f);
			c1.setZ(0, 2.077142003943222f);
			
			c1.calculate();
			Assert::AreEqual(1.413773951490104f, c1.getMeanDistance2Model(), 1e-5f);
			Assert::AreEqual(0.0, c1.getHumanProb(), 1e-3);

		}

		[TestMethod]
		void TestCalcDist2Model3()
		{
			Cluster c1(1);
			c1.setX(0, -0.100000000000000f);
			c1.setY(0, 0.168733222621294f);
			c1.setZ(0, 1.100000000000000f);

			  
   
   
			
			c1.calculate();
			Assert::AreEqual(0.0f, c1.getMeanDistance2Model(), 1e-5f);
			Assert::AreEqual(1.0, c1.getHumanProb(), 1e-5);
			

		}

		[TestMethod]
		void TestTransformation()
		{
			Cluster c1(1);
			c1.setX(0, 1.0f);
			c1.setY(0, 2.0f);
			c1.setZ(0, 3.0f);

			  
			Transformation2D trans(M_PI/2.0f, 1.0f, 1.0f);
			c1.transform(trans);

			Assert::AreEqual(-1.0f, c1.getX()[0], 1e-5f);
			Assert::AreEqual(2.0f,	c1.getY()[0], 1e-5f);
			Assert::AreEqual(3.0f,	c1.getZ()[0], 1e-5f);
   
			

			

		}
	};
}
