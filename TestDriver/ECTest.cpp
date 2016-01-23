#include "stdafx.h"

using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace	Microsoft::VisualStudio::TestTools::UnitTesting;


#include "../CFObject/ECTestAdapter.h"
#include "../CFObject/EC.h"

namespace TestDriver
{
	[TestClass]
	public ref class ECTest
	{
	public: 

		[TestMethod]
		void Test1()
		{
			ECTestAdapter adapter;			
			Assert::IsTrue(adapter.test1());
		}


		[TestMethod]
		void Test2()
		{
			ECTestAdapter adapter;			
			Assert::IsTrue(adapter.test2());
		}

		[TestMethod]
		void Test3()
		{
			ECTestAdapter adapter;			
			Assert::IsTrue(adapter.test3());
		}


		[TestMethod]
		void Test4()
		{
			ECTestAdapter adapter;			
			Assert::IsTrue(adapter.test4());
		}

		[TestMethod]
		void Test5()
		{
			ECTestAdapter adapter;			
			Assert::IsTrue(adapter.test5());
		}


		[TestMethod]
		void Test6()
		{
			ECTestAdapter adapter;			
			Assert::IsTrue(adapter.test5());

			float x[] = {0, 0, FLT_MAX};
			float y[] = {0, 0, FLT_MAX};
			float z[] = {0, 0, FLT_MAX};
			int nx = 3;
			int ny = 1;
			CameraType type(nx, ny, x, y, z, nx, ny, x, y, z);
			Camera cam(type);
			for(int i=0; i<nx*ny; i++)
			{
				cam.setdx()[i] = x[i];
				cam.setdy()[i] = y[i];
				cam.setdz()[i] = z[i];
			}


			EC ec(6);	
			ec.addDepthData(cam);
			ec.addDepthData(cam);
			
			Assert::AreEqual(ec.getMaxSize(), 6);

			Assert::IsTrue(ec.getX() != NULL);
			Assert::IsTrue(ec.getY() != NULL);
			Assert::IsTrue(ec.getZ() != NULL);
			Assert::IsTrue(ec.getHasProcessed() != NULL);

			Assert::AreEqual(ec.getX()[0], x[0]);
			Assert::AreEqual(ec.getY()[0], y[0]);
			Assert::AreEqual(ec.getZ()[0], z[0]);

			Assert::AreEqual(ec.getX()[1], x[1]);
			Assert::AreEqual(ec.getY()[1], y[1]);
			Assert::AreEqual(ec.getZ()[1], z[1]);

			Assert::AreEqual(ec.getX()[2], x[0]);
			Assert::AreEqual(ec.getY()[2], y[0]);
			Assert::AreEqual(ec.getZ()[2], z[0]);

			Assert::AreEqual(ec.getX()[3], x[1]);
			Assert::AreEqual(ec.getY()[3], y[1]);
			Assert::AreEqual(ec.getZ()[3], z[1]);

			ec.cluster();

			Assert::AreEqual(ec.getNumOfClusters(), 1);
			Cluster C1 = ec.getCluster(0);

			Assert::AreEqual(C1.getMaxSize(), 4);

			Assert::AreEqual(C1.getX()[0], x[0]);
			Assert::AreEqual(C1.getY()[0], y[0]);
			Assert::AreEqual(C1.getZ()[0], z[0]);

			Assert::AreEqual(C1.getX()[1], x[1]);
			Assert::AreEqual(C1.getY()[1], y[1]);
			Assert::AreEqual(C1.getZ()[1], z[1]);

			Assert::AreEqual(C1.getX()[2], x[0]);
			Assert::AreEqual(C1.getY()[2], y[0]);
			Assert::AreEqual(C1.getZ()[2], z[0]);

			Assert::AreEqual(C1.getX()[3], x[1]);
			Assert::AreEqual(C1.getY()[3], y[1]);
			Assert::AreEqual(C1.getZ()[3], z[1]);

		}


		[TestMethod]
		void Test7()
		{
			ECTestAdapter adapter;			
			Assert::IsTrue(adapter.test5());

			float x[] = {0, 2.0, FLT_MAX};
			float y[] = {0, 2.0, FLT_MAX};
			float z[] = {0, 2.0, FLT_MAX};
			int nx = 3;
			int ny = 1;
			CameraType type(nx, ny, x, y, z, nx, ny, x, y, z);
			Camera cam(type);
			for(int i=0; i<nx*ny; i++)
			{
				cam.setdx()[i] = x[i];
				cam.setdy()[i] = y[i];
				cam.setdz()[i] = z[i];
			}


			EC ec(6);	
			ec.addDepthData(cam);
			ec.addDepthData(cam);
			
			Assert::AreEqual(ec.getMaxSize(), 6);

			Assert::IsTrue(ec.getX() != NULL);
			Assert::IsTrue(ec.getY() != NULL);
			Assert::IsTrue(ec.getZ() != NULL);
			Assert::IsTrue(ec.getHasProcessed() != NULL);

			Assert::AreEqual(ec.getX()[0], x[0]);
			Assert::AreEqual(ec.getY()[0], y[0]);
			Assert::AreEqual(ec.getZ()[0], z[0]);

			Assert::AreEqual(ec.getX()[1], x[1]);
			Assert::AreEqual(ec.getY()[1], y[1]);
			Assert::AreEqual(ec.getZ()[1], z[1]);

			Assert::AreEqual(ec.getX()[2], x[0]);
			Assert::AreEqual(ec.getY()[2], y[0]);
			Assert::AreEqual(ec.getZ()[2], z[0]);

			Assert::AreEqual(ec.getX()[3], x[1]);
			Assert::AreEqual(ec.getY()[3], y[1]);
			Assert::AreEqual(ec.getZ()[3], z[1]);

			ec.cluster();

			Assert::AreEqual(ec.getNumOfClusters(), 2);
			Cluster C1 = ec.getCluster(0);
			Cluster C2 = ec.getCluster(1);

			Assert::AreEqual(C1.getMaxSize(), 2);

			Assert::AreEqual(C1.getX()[0], x[0]);
			Assert::AreEqual(C1.getY()[0], y[0]);
			Assert::AreEqual(C1.getZ()[0], z[0]);

			Assert::AreEqual(C1.getX()[1], x[0]);
			Assert::AreEqual(C1.getY()[1], y[0]);
			Assert::AreEqual(C1.getZ()[1], z[0]);

			Assert::AreEqual(C2.getX()[0], x[1]);
			Assert::AreEqual(C2.getY()[0], y[1]);
			Assert::AreEqual(C2.getZ()[0], z[1]);

			Assert::AreEqual(C2.getX()[1], x[1]);
			Assert::AreEqual(C2.getY()[1], y[1]);
			Assert::AreEqual(C2.getZ()[1], z[1]);

		}

		[TestMethod]
		void Test8()
		{
			ECTestAdapter adapter;			
			Assert::IsTrue(adapter.test5());

			float x[] = {0, 2.0, 0.1, 2.1, FLT_MAX};
			float y[] = {0, 2.0, 0.1, 2.1, FLT_MAX};
			float z[] = {0, 2.0, 0.1, 2.1, FLT_MAX};
			int nx = 5;
			int ny = 1;
			CameraType type(nx, ny, x, y, z, nx, ny, x, y, z);
			Camera cam(type);
			for(int i=0; i<nx*ny; i++)
			{
				cam.setdx()[i] = x[i];
				cam.setdy()[i] = y[i];
				cam.setdz()[i] = z[i];
			}


			EC ec(10);	
			ec.addDepthData(cam);
			ec.addDepthData(cam);
			
			Assert::AreEqual(ec.getMaxSize(), 10);

			Assert::IsTrue(ec.getX() != NULL);
			Assert::IsTrue(ec.getY() != NULL);
			Assert::IsTrue(ec.getZ() != NULL);
			Assert::IsTrue(ec.getHasProcessed() != NULL);

			Assert::AreEqual(ec.getX()[0], x[0]);
			Assert::AreEqual(ec.getY()[0], y[0]);
			Assert::AreEqual(ec.getZ()[0], z[0]);

			Assert::AreEqual(ec.getX()[1], x[1]);
			Assert::AreEqual(ec.getY()[1], y[1]);
			Assert::AreEqual(ec.getZ()[1], z[1]);

			Assert::AreEqual(ec.getX()[2], x[2]);
			Assert::AreEqual(ec.getY()[2], y[2]);
			Assert::AreEqual(ec.getZ()[2], z[2]);

			Assert::AreEqual(ec.getX()[3], x[3]);
			Assert::AreEqual(ec.getY()[3], y[3]);
			Assert::AreEqual(ec.getZ()[3], z[3]);

			ec.cluster();

			Assert::AreEqual(ec.getNumOfClusters(), 2);
			Cluster C1 = ec.getCluster(0);
			Cluster C2 = ec.getCluster(1);

			Assert::AreEqual(C1.getMaxSize(), 4);

			Assert::AreEqual(C1.getX()[0], x[0]);
			Assert::AreEqual(C1.getY()[0], y[0]);
			Assert::AreEqual(C1.getZ()[0], z[0]);

			Assert::AreEqual(C1.getX()[1], x[2]);
			Assert::AreEqual(C1.getY()[1], y[2]);
			Assert::AreEqual(C1.getZ()[1], z[2]);

			Assert::AreEqual(C1.getX()[2], x[0]);
			Assert::AreEqual(C1.getY()[2], y[0]);
			Assert::AreEqual(C1.getZ()[2], z[0]);

			Assert::AreEqual(C1.getX()[3], x[2]);
			Assert::AreEqual(C1.getY()[3], y[2]);
			Assert::AreEqual(C1.getZ()[3], z[2]);

			Assert::AreEqual(C2.getX()[0], x[1]);
			Assert::AreEqual(C2.getY()[0], y[1]);
			Assert::AreEqual(C2.getZ()[0], z[1]);

			Assert::AreEqual(C2.getX()[1], x[3]);
			Assert::AreEqual(C2.getY()[1], y[3]);
			Assert::AreEqual(C2.getZ()[1], z[3]);

			Assert::AreEqual(C2.getX()[2], x[1]);
			Assert::AreEqual(C2.getY()[2], y[1]);
			Assert::AreEqual(C2.getZ()[2], z[1]);

			Assert::AreEqual(C2.getX()[3], x[3]);
			Assert::AreEqual(C2.getY()[3], y[3]);
			Assert::AreEqual(C2.getZ()[3], z[3]);

		}
	};
}
