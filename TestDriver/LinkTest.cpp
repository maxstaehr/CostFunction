#include "stdafx.h"

using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace	Microsoft::VisualStudio::TestTools::UnitTesting;

#include "../CFObject/Link.h"

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


			test.getH(temp1);

			Assert::AreEqual(temp1[0], 1.0f);
			Assert::AreEqual(temp1[1], 0.0f);
			Assert::AreEqual(temp1[2], 0.0f);
			Assert::AreEqual(temp1[3], 0.0f);

			Assert::AreEqual(temp1[4], 0.0f);
			Assert::AreEqual(temp1[5], 1.0f);
			Assert::AreEqual(temp1[6], 0.0f);
			Assert::AreEqual(temp1[7], 0.0f);

			Assert::AreEqual(temp1[8], 0.0f);
			Assert::AreEqual(temp1[9], 0.0f);
			Assert::AreEqual(temp1[10], 1.0f);
			Assert::AreEqual(temp1[11], 0.0f);

			Assert::AreEqual(temp1[12], 0.0f);
			Assert::AreEqual(temp1[13], 0.0f);
			Assert::AreEqual(temp1[14], 0.0f);
			Assert::AreEqual(temp1[15], 1.0f);

						
			test.setH(temp2);
			link.setH(test);

			link.getH().getH(temp1);

			Assert::AreEqual(temp1[0], temp2[0]);
			Assert::AreEqual(temp1[1], temp2[1]);
			Assert::AreEqual(temp1[2], temp2[2]);
			Assert::AreEqual(temp1[3], temp2[3]);

			Assert::AreEqual(temp1[4], temp2[4]);
			Assert::AreEqual(temp1[5], temp2[5]);
			Assert::AreEqual(temp1[6], temp2[6]);
			Assert::AreEqual(temp1[7], temp2[7]);

			Assert::AreEqual(temp1[8], temp2[8]);
			Assert::AreEqual(temp1[9], temp2[9]);
			Assert::AreEqual(temp1[10],temp2[10]);
			Assert::AreEqual(temp1[11],temp2[11]);

			Assert::AreEqual(temp1[12], temp2[12]);
			Assert::AreEqual(temp1[13], temp2[13]);
			Assert::AreEqual(temp1[14], temp2[14]);
			Assert::AreEqual(temp1[15], temp2[15]);

		}
	};
}
