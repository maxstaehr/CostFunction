#include "stdafx.h"
#define _USE_MATH_DEFINES 
#include <cmath>
#include <float.h>

using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace	Microsoft::VisualStudio::TestTools::UnitTesting;

#include "../CFObject/Camera.h"
#include <iostream>



namespace TestDriver
{
	[TestClass]
	public ref class CameraTest
	{
	public: 
		[TestMethod]
		void TestConstructor()
		{

			int nx = 1; 
			int ny = 1;
			float x[] = {1};
			float y[] = {2};
			float z[] = {3};
			int ssnx = 1;
			int ssny = 1;
			float ssx[] = {4};
			float ssy[] = {5};			
			float ssz[] = {6};			
			CameraType camType(nx, ny, x, y, z, ssnx, ssny, ssx, ssy,ssz);
			Camera cam(camType);

			Assert::AreEqual(nx, cam.getCameraType().getnx());
			Assert::AreEqual(ny, cam.getCameraType().getny());
			Assert::AreEqual(x[0], cam.getx()[0]);
			Assert::AreEqual(y[0], cam.gety()[0]);
			Assert::AreEqual(z[0], cam.getz()[0]);
			
			Assert::AreEqual(ssnx, cam.getCameraType().getssnx());
			Assert::AreEqual(ssny, cam.getCameraType().getssny());
			Assert::AreEqual(ssx[0], cam.getssx()[0]);
			Assert::AreEqual(ssy[0], cam.getssy()[0]);
			Assert::AreEqual(ssz[0], cam.getssz()[0]);

			Assert::IsTrue(cam.getd() != NULL);
			Assert::IsTrue(cam.getssd() != NULL);
			
		}

		[TestMethod]
		void TestSetUpTransformation()
		{

			int nx = 3; 
			int ny = 1;
			int ssnx = 3;
			int ssny = 1;

			float x[] = {0, 0, 1};
			float y[] = {0, 1, 0};
			float z[] = {0, 0, 0};

			float ssx[] = {0, 0, 1};
			float ssy[] = {0, 1, 0};
			float ssz[] = {0, 0, 0};

			float xr[] = {0, -1, 0};
			float yr[] = {0, 0, 1};
			float zr[] = {0, 0, 0};



			
			HomogeneTransformation trans;
			trans.init(0, 0, M_PI/2.0, 2, 2, 2);			
			

			CameraType camType(nx, ny, x, y, z, ssnx, ssny, ssx, ssy,ssz);
			Camera cam(camType);

			Assert::AreEqual(nx, cam.getCameraType().getnx());
			Assert::AreEqual(ny, cam.getCameraType().getny());
			Assert::AreEqual(x[0], cam.getx()[0]);
			Assert::AreEqual(y[0], cam.gety()[0]);
			Assert::AreEqual(z[0], cam.getz()[0]);
			Assert::AreEqual(x[1], cam.getx()[1]);
			Assert::AreEqual(y[1], cam.gety()[1]);
			Assert::AreEqual(z[1], cam.getz()[1]);
			Assert::AreEqual(x[2], cam.getx()[2]);
			Assert::AreEqual(y[2], cam.gety()[2]);
			Assert::AreEqual(z[2], cam.getz()[2]);
			
			Assert::AreEqual(ssnx, cam.getCameraType().getssnx());
			Assert::AreEqual(ssny, cam.getCameraType().getssny());
			Assert::AreEqual(ssx[0], cam.getssx()[0]);
			Assert::AreEqual(ssy[0], cam.getssy()[0]);
			Assert::AreEqual(ssz[0], cam.getssz()[0]);
			Assert::AreEqual(ssx[1], cam.getssx()[1]);
			Assert::AreEqual(ssy[1], cam.getssy()[1]);
			Assert::AreEqual(ssz[1], cam.getssz()[1]);
			Assert::AreEqual(ssx[2], cam.getssx()[2]);
			Assert::AreEqual(ssy[2], cam.getssy()[2]);
			Assert::AreEqual(ssz[2], cam.getssz()[2]);

			Assert::IsTrue(cam.getd() != NULL);
			Assert::IsTrue(cam.getssd() != NULL);

			cam.updateCameraPos(trans);

			Assert::AreEqual(nx, cam.getCameraType().getnx());
			Assert::AreEqual(ny, cam.getCameraType().getny());
			Assert::AreEqual(xr[0], cam.getx()[0], 1e-3f);
			Assert::AreEqual(yr[0], cam.gety()[0], 1e-3f);
			Assert::AreEqual(zr[0], cam.getz()[0], 1e-3f);
			Assert::AreEqual(xr[1], cam.getx()[1], 1e-3f);
			Assert::AreEqual(yr[1], cam.gety()[1], 1e-3f);
			Assert::AreEqual(zr[1], cam.getz()[1], 1e-3f);
			Assert::AreEqual(xr[2], cam.getx()[2], 1e-3f);
			Assert::AreEqual(yr[2], cam.gety()[2], 1e-3f);
			Assert::AreEqual(zr[2], cam.getz()[2], 1e-3f);

			Assert::AreEqual(ssnx, cam.getCameraType().getssnx());
			Assert::AreEqual(ssny, cam.getCameraType().getssny());
			Assert::AreEqual(xr[0], cam.getssx()[0], 1e-3f);
			Assert::AreEqual(yr[0], cam.getssy()[0], 1e-3f);
			Assert::AreEqual(zr[0], cam.getssz()[0], 1e-3f);
			Assert::AreEqual(xr[1], cam.getssx()[1], 1e-3f);
			Assert::AreEqual(yr[1], cam.getssy()[1], 1e-3f);
			Assert::AreEqual(zr[1], cam.getssz()[1], 1e-3f);
			Assert::AreEqual(xr[2], cam.getssx()[2], 1e-3f);
			Assert::AreEqual(yr[2], cam.getssy()[2], 1e-3f);
			Assert::AreEqual(zr[2], cam.getssz()[2], 1e-3f);

			Assert::IsTrue(cam.getd() != NULL);
			Assert::IsTrue(cam.getssd() != NULL);
			
		}


		[TestMethod]
		void TestRaytrace1()
		{

			int nx = 1; 
			int ny = 1;
			int ssnx = 1;
			int ssny = 1;

			float x[] = {1};
			float y[] = {0};
			float z[] = {0};

			float ssx[] = {1};
			float ssy[] = {0};
			float ssz[] = {0};

			int nV = 3;
			int nF = 1;
			float xpcl[] = {1, 1, 1};
			float ypcl[] = {0, 1, 0};
			float zpcl[] = {0, 0, 1};
			int fx[] = {0};
			int fy[] = {1};
			int fz[] = {2};
			PCL pcl(nV, nF, xpcl, ypcl, zpcl, fx, fy, fz);
			
			CameraType camType(nx, ny, x, y, z, ssnx, ssny, ssx, ssy,ssz);
			Camera cam(camType);
			cam.raytrace(pcl);

			Assert::AreEqual(nx, cam.getCameraType().getnx());
			Assert::AreEqual(ny, cam.getCameraType().getny());
			Assert::AreEqual(x[0], cam.getx()[0]);
			Assert::AreEqual(y[0], cam.gety()[0]);
			Assert::AreEqual(z[0], cam.getz()[0]);
			
			Assert::AreEqual(ssnx, cam.getCameraType().getssnx());
			Assert::AreEqual(ssny, cam.getCameraType().getssny());
			Assert::AreEqual(ssx[0], cam.getssx()[0]);
			Assert::AreEqual(ssy[0], cam.getssy()[0]);
			Assert::AreEqual(ssz[0], cam.getssz()[0]);

			Assert::IsTrue(cam.getd() != NULL);
			Assert::IsTrue(cam.getssd() != NULL);



			Assert::AreEqual(1.0f, cam.getd()[0], 1e-3f);
			Assert::AreEqual(1.0f, cam.getssd()[0], 1e-3f);		

			Assert::IsTrue(cam.getdx() != NULL);
			Assert::IsTrue(cam.getdy() != NULL);
			Assert::IsTrue(cam.getdz() != NULL);
			Assert::IsTrue(cam.getssdx() != NULL);
			Assert::IsTrue(cam.getssdy() != NULL);
			Assert::IsTrue(cam.getssdz() != NULL);
			Assert::AreEqual(1.0f, cam.getdx()[0], 1e-3f);
			Assert::AreEqual(0.0f, cam.getdy()[0], 1e-3f);		
			Assert::AreEqual(0.0f, cam.getdz()[0], 1e-3f);
			Assert::AreEqual(1.0f, cam.getssdx()[0], 1e-3f);		
			Assert::AreEqual(0.0f, cam.getssdy()[0], 1e-3f);
			Assert::AreEqual(0.0f, cam.getssdz()[0], 1e-3f);		


		}

		[TestMethod]
		void TestRaytrace2()
		{

			int nx = 1; 
			int ny = 1;
			int ssnx = 1;
			int ssny = 1;

			float x[] = {1};
			float y[] = {0};
			float z[] = {0};

			float ssx[] = {1};
			float ssy[] = {0};
			float ssz[] = {0};

			int nV = 3;
			int nF = 1;
			float xpcl[] = {1, 1, 1};
			float ypcl[] = {0, 1, 0};
			float zpcl[] = {0, 0, 1};
			int fx[] = {0};
			int fy[] = {1};
			int fz[] = {2};
			PCL pcl(nV, nF, xpcl, ypcl, zpcl, fx, fy, fz);
			
			CameraType camType(nx, ny, x, y, z, ssnx, ssny, ssx, ssy,ssz);
			Camera cam(camType);
			

			HomogeneTransformation trans;
			trans.init(0, 0, 0, 0, 1, 1);	

			cam.updateCameraPos(trans);
			cam.raytrace(pcl);

			Assert::IsTrue(cam.getd() != NULL);
			Assert::IsTrue(cam.getssd() != NULL);

			Assert::AreEqual(FLT_MAX, cam.getd()[0], 1e-3f);
			Assert::AreEqual(FLT_MAX, cam.getssd()[0], 1e-3f);		

			Assert::IsTrue(cam.getdx() != NULL);
			Assert::IsTrue(cam.getdy() != NULL);
			Assert::IsTrue(cam.getdz() != NULL);
			Assert::IsTrue(cam.getssdx() != NULL);
			Assert::IsTrue(cam.getssdy() != NULL);
			Assert::IsTrue(cam.getssdz() != NULL);
			Assert::IsTrue(FLT_MAX==(cam.getdx()[0]));
			Assert::IsTrue(FLT_MAX==(cam.getdy()[0]));
			Assert::IsTrue(FLT_MAX==(cam.getdz()[0]));
			Assert::IsTrue(FLT_MAX==(cam.getssdx()[0]));
			Assert::IsTrue(FLT_MAX==(cam.getssdy()[0]));
			Assert::IsTrue(FLT_MAX==(cam.getssdz()[0]));
		}

		[TestMethod]
		void TestRaytrace3()
		{

			int nx = 1; 
			int ny = 1;
			int ssnx = 1;
			int ssny = 1;

			float x[] = {1};
			float y[] = {0};
			float z[] = {0};

			float ssx[] = {1};
			float ssy[] = {0};
			float ssz[] = {0};

			int nV = 3;
			int nF = 1;
			float xpcl[] = {1, 1, 2};
			float ypcl[] = {0, 1, 0};
			float zpcl[] = {0, 0, 0};
			int fx[] = {0};
			int fy[] = {1};
			int fz[] = {2};
			PCL pcl(nV, nF, xpcl, ypcl, zpcl, fx, fy, fz);
			
			CameraType camType(nx, ny, x, y, z, ssnx, ssny, ssx, ssy,ssz);
			Camera cam(camType);
			

			HomogeneTransformation trans;
			trans.init(0, 0, 0, 0, 0, 0);	

			cam.updateCameraPos(trans);
			cam.raytrace(pcl);

			Assert::IsTrue(cam.getd() != NULL);
			Assert::IsTrue(cam.getssd() != NULL);

			Assert::AreEqual(FLT_MAX, cam.getd()[0]);
			Assert::AreEqual(FLT_MAX, cam.getssd()[0]);		


			Assert::IsTrue(cam.getdx() != NULL);
			Assert::IsTrue(cam.getdy() != NULL);
			Assert::IsTrue(cam.getdz() != NULL);
			Assert::IsTrue(cam.getssdx() != NULL);
			Assert::IsTrue(cam.getssdy() != NULL);
			Assert::IsTrue(cam.getssdz() != NULL);
			Assert::IsTrue(FLT_MAX==(cam.getdx()[0]));
			Assert::IsTrue(FLT_MAX==(cam.getdy()[0]));
			Assert::IsTrue(FLT_MAX==(cam.getdz()[0]));
			Assert::IsTrue(FLT_MAX==(cam.getssdx()[0]));
			Assert::IsTrue(FLT_MAX==(cam.getssdy()[0]));
			Assert::IsTrue(FLT_MAX==(cam.getssdz()[0]));

		}
		
		[TestMethod]
		void TestRaytrace4()
		{

			int nx = 1; 
			int ny = 1;
			int ssnx = 1;
			int ssny = 1;

			float x[] = {1};
			float y[] = {0};
			float z[] = {0};

			float ssx[] = {1};
			float ssy[] = {0};
			float ssz[] = {0};

			int nV = 3;
			int nF = 1;
			float xpcl[] = {1, 1, 2};
			float ypcl[] = {0, 1, 0};
			float zpcl[] = {0, 0, 1};
			int fx[] = {0};
			int fy[] = {1};
			int fz[] = {2};
			PCL pcl(nV, nF, xpcl, ypcl, zpcl, fx, fy, fz);
			
			CameraType camType(nx, ny, x, y, z, ssnx, ssny, ssx, ssy,ssz);
			Camera cam(camType);
			

			HomogeneTransformation trans;
			trans.init(0, 0, 0, 0, 0.25, 0.5);	

			cam.updateCameraPos(trans);
			cam.raytrace(pcl);

			Assert::IsTrue(cam.getd() != NULL);
			Assert::IsTrue(cam.getssd() != NULL);

			Assert::AreEqual(1.5f, cam.getd()[0], 1e-3f);
			Assert::AreEqual(1.5f, cam.getssd()[0], 1e-3f);			


			Assert::IsTrue(cam.getdx() != NULL);
			Assert::IsTrue(cam.getdy() != NULL);
			Assert::IsTrue(cam.getdz() != NULL);
			Assert::IsTrue(cam.getssdx() != NULL);
			Assert::IsTrue(cam.getssdy() != NULL);
			Assert::IsTrue(cam.getssdz() != NULL);
			Assert::AreEqual(1.5f, cam.getdx()[0], 1e-3f);
			Assert::AreEqual(0.25f, cam.getdy()[0], 1e-3f);		
			Assert::AreEqual(0.5f, cam.getdz()[0], 1e-3f);
			Assert::AreEqual(1.5f, cam.getssdx()[0], 1e-3f);		
			Assert::AreEqual(0.25f, cam.getssdy()[0], 1e-3f);
			Assert::AreEqual(0.5f, cam.getssdz()[0], 1e-3f);	
		}

		[TestMethod]
		void TestRaytrace5()
		{

			int nx = 1; 
			int ny = 1;
			int ssnx = 1;
			int ssny = 1;

			float x[] = {1};
			float y[] = {0};
			float z[] = {0};

			float ssx[] = {1};
			float ssy[] = {0};
			float ssz[] = {0};

			int nV = 3;
			int nF = 1;
			float xpcl[] = {1, 1, 2};
			float ypcl[] = {0, 1, 0};
			float zpcl[] = {0, 0, 1};

			int fx[] = {0};
			int fy[] = {1};
			int fz[] = {2};
			PCL pcl(nV, nF, xpcl, ypcl, zpcl, fx, fy, fz);
			
			CameraType camType(nx, ny, x, y, z, ssnx, ssny, ssx, ssy,ssz);
			Camera cam(camType);
			

			HomogeneTransformation trans;
			trans.init(0, 0, 0, 2.5, 0.25, 0.5);	

			cam.updateCameraPos(trans);
			cam.raytrace(pcl);

			Assert::IsTrue(cam.getd() != NULL);
			Assert::IsTrue(cam.getssd() != NULL);

			Assert::AreEqual(FLT_MAX, cam.getd()[0], 1e-3f);
			Assert::AreEqual(FLT_MAX, cam.getssd()[0], 1e-3f);			

			Assert::IsTrue(cam.getdx() != NULL);
			Assert::IsTrue(cam.getdy() != NULL);
			Assert::IsTrue(cam.getdz() != NULL);
			Assert::IsTrue(cam.getssdx() != NULL);
			Assert::IsTrue(cam.getssdy() != NULL);
			Assert::IsTrue(cam.getssdz() != NULL);
			Assert::IsTrue(FLT_MAX==cam.getdx()[0]);
			Assert::IsTrue(FLT_MAX==(cam.getdy()[0]));
			Assert::IsTrue(FLT_MAX==(cam.getdz()[0]));
			Assert::IsTrue(FLT_MAX==(cam.getssdx()[0]));
			Assert::IsTrue(FLT_MAX==(cam.getssdy()[0]));
			Assert::IsTrue(FLT_MAX==(cam.getssdz()[0]));
		}

		[TestMethod]
		void TestRaytrace6()
		{

			int nx = 1; 
			int ny = 1;
			int ssnx = 1;
			int ssny = 1;

			float x[] = {1};
			float y[] = {0};
			float z[] = {0};

			float ssx[] = {1};
			float ssy[] = {0};
			float ssz[] = {0};

			int nV = 3;
			int nF = 1;

			float xpcl1[] = {1, 1, 2};
			float ypcl1[] = {0, 1, 0};
			float zpcl1[] = {0, 0, 1};

			float xpcl2[] = {2, 2, 3};
			float ypcl2[] = {0, 1, 0};
			float zpcl2[] = {0, 0, 1};

			int fx[] = {0};
			int fy[] = {1};
			int fz[] = {2};

			PCL pcl1(nV, nF, xpcl1, ypcl1, zpcl1, fx, fy, fz);
			PCL pcl2(nV, nF, xpcl2, ypcl2, zpcl2, fx, fy, fz);
			
			CameraType camType(nx, ny, x, y, z, ssnx, ssny, ssx, ssy,ssz);
			Camera cam(camType);
			

			HomogeneTransformation trans;
			trans.init(0, 0, 0, 0, 0.25, 0.5);	

			cam.updateCameraPos(trans);
			cam.raytrace(pcl1);
			cam.raytrace(pcl2);

			Assert::IsTrue(cam.getd() != NULL);
			Assert::IsTrue(cam.getssd() != NULL);

			Assert::AreEqual(1.5f, cam.getd()[0], 1e-3f);
			Assert::AreEqual(1.5f, cam.getssd()[0], 1e-3f);		

			Assert::IsTrue(cam.getdx() != NULL);
			Assert::IsTrue(cam.getdy() != NULL);
			Assert::IsTrue(cam.getdz() != NULL);
			Assert::IsTrue(cam.getssdx() != NULL);
			Assert::IsTrue(cam.getssdy() != NULL);
			Assert::IsTrue(cam.getssdz() != NULL);
			Assert::AreEqual(1.5f, cam.getdx()[0], 1e-3f);
			Assert::AreEqual(0.25f, cam.getdy()[0], 1e-3f);		
			Assert::AreEqual(0.5f, cam.getdz()[0], 1e-3f);
			Assert::AreEqual(1.5f, cam.getssdx()[0], 1e-3f);		
			Assert::AreEqual(0.25f, cam.getssdy()[0], 1e-3f);
			Assert::AreEqual(0.5f, cam.getssdz()[0], 1e-3f);	
		}

		[TestMethod]
		void TestRaytrace7()
		{

			int nx = 1; 
			int ny = 1;
			int ssnx = 1;
			int ssny = 1;

			float x[] = {1};
			float y[] = {0};
			float z[] = {0};

			float ssx[] = {1};
			float ssy[] = {0};
			float ssz[] = {0};

			int nV = 3;
			int nF = 1;

			float xpcl1[] = {1, 1, 2};
			float ypcl1[] = {0, 1, 0};
			float zpcl1[] = {0, 0, 1};

			float xpcl2[] = {2, 2, 3};
			float ypcl2[] = {0, 1, 0};
			float zpcl2[] = {0, 0, 1};

			int fx[] = {0};
			int fy[] = {1};
			int fz[] = {2};

			PCL pcl1(nV, nF, xpcl1, ypcl1, zpcl1, fx, fy, fz);
			PCL pcl2(nV, nF, xpcl2, ypcl2, zpcl2, fx, fy, fz);
			
			CameraType camType(nx, ny, x, y, z, ssnx, ssny, ssx, ssy,ssz);
			Camera cam(camType);
			

			HomogeneTransformation trans;
			trans.init(0, 0, 0, 0, 0.25, 0.5);	

			cam.updateCameraPos(trans);
			cam.raytrace(pcl2);
			cam.raytrace(pcl1);

			Assert::IsTrue(cam.getd() != NULL);
			Assert::IsTrue(cam.getssd() != NULL);

			Assert::AreEqual(1.5f, cam.getd()[0], 1e-3f);
			Assert::AreEqual(1.5f, cam.getssd()[0], 1e-3f);			

			Assert::IsTrue(cam.getdx() != NULL);
			Assert::IsTrue(cam.getdy() != NULL);
			Assert::IsTrue(cam.getdz() != NULL);
			Assert::IsTrue(cam.getssdx() != NULL);
			Assert::IsTrue(cam.getssdy() != NULL);
			Assert::IsTrue(cam.getssdz() != NULL);
			Assert::AreEqual(1.5f, cam.getdx()[0], 1e-3f);
			Assert::AreEqual(0.25f, cam.getdy()[0], 1e-3f);		
			Assert::AreEqual(0.5f, cam.getdz()[0], 1e-3f);
			Assert::AreEqual(1.5f, cam.getssdx()[0], 1e-3f);		
			Assert::AreEqual(0.25f, cam.getssdy()[0], 1e-3f);
			Assert::AreEqual(0.5f, cam.getssdz()[0], 1e-3f);	
		}
	};
}
