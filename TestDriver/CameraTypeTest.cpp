#include "stdafx.h"

using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace	Microsoft::VisualStudio::TestTools::UnitTesting;

#include "../CFObject/CameraType.h"
#include <iostream>

namespace TestDriver
{
	[TestClass]
	public ref class CameraTypeTest
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

			CameraType cam(nx, ny, x, y, z, ssnx, ssny, ssx, ssy,ssz, Link());

			Assert::AreEqual(nx, cam.getnx());
			Assert::AreEqual(ny, cam.getny());
			Assert::AreEqual(x[0], cam.getx()[0]);
			Assert::AreEqual(y[0], cam.gety()[0]);
			Assert::AreEqual(z[0], cam.getz()[0]);
			
			Assert::AreEqual(ssnx, cam.getssnx());
			Assert::AreEqual(ssny, cam.getssny());
			Assert::AreEqual(ssx[0], cam.getssx()[0]);
			Assert::AreEqual(ssy[0], cam.getssy()[0]);
			Assert::AreEqual(ssz[0], cam.getssz()[0]);


			
		}
	};
}
