#include "stdafx.h"

using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace	Microsoft::VisualStudio::TestTools::UnitTesting;

#include "../CFObject/Rotation.h"

namespace TestDriver
{
	[TestClass]
	public ref class RotationTest
	{
	public: 
		[TestMethod]
		void TestConstructor()
		{
			Rotation test1;			
			float temp[9];

			test1.getH(temp);

			Assert::AreEqual(temp[0], 1.0f);
			Assert::AreEqual(temp[1], 0.0f);
			Assert::AreEqual(temp[2], 0.0f);			

			Assert::AreEqual(temp[3], 0.0f);
			Assert::AreEqual(temp[4], 1.0f);
			Assert::AreEqual(temp[5], 0.0f);			

			Assert::AreEqual(temp[6], 0.0f);
			Assert::AreEqual(temp[7], 0.0f);
			Assert::AreEqual(temp[8], 1.0f);			

		}

		[TestMethod]
		void TestSetter()
		{
			Rotation test;
			float temp1[9];
			float temp2[9] ={	0.0f, 1.0f, 2.0f, 
								3.0f, 4.0f, 5.0f, 
								6.0f, 7.0f, 8.0f
							};								
			test.getH(temp1);

			Assert::AreEqual(temp1[0], 1.0f);
			Assert::AreEqual(temp1[1], 0.0f);
			Assert::AreEqual(temp1[2], 0.0f);
			
			Assert::AreEqual(temp1[3], 0.0f);
			Assert::AreEqual(temp1[4], 1.0f);
			Assert::AreEqual(temp1[5], 0.0f);			

			Assert::AreEqual(temp1[6], 0.0f);
			Assert::AreEqual(temp1[7], 0.0f);
			Assert::AreEqual(temp1[8], 1.0f);
			
						
			test.setH(temp2);
			test.getH(temp1);

			Assert::AreEqual(temp1[0], temp2[0]);
			Assert::AreEqual(temp1[1], temp2[1]);
			Assert::AreEqual(temp1[2], temp2[2]);			

			Assert::AreEqual(temp1[3], temp2[3]);
			Assert::AreEqual(temp1[4], temp2[4]);
			Assert::AreEqual(temp1[5], temp2[5]);
			
			Assert::AreEqual(temp1[6], temp2[6]);
			Assert::AreEqual(temp1[7], temp2[7]);
			Assert::AreEqual(temp1[8], temp2[8]);			

		}


		[TestMethod]
		void testCopyConstructor(void)
		{

						
			float temp1[9] = 
			{
				0.915735525189067, 0.035711678574190, 0.757740130578333, 
				0.792207329559554, 0.849129305868777, 0.743132468124916, 
				0.959492426392903, 0.933993247757551, 0.392227019534168, 				
			};
			float temp2[9];
			float temp3[9] = 
			{
				0.915735525189067, 0.035711678574190, 0.757740130578333, 
				0.792207329559554, 0.849129305868777, 0.743132468124916, 
				0.959492426392903, 0.933993247757551, 0.392227019534168, 
			};

			Rotation test1;
			test1.setH(temp1);
			Rotation test2(test1);
			test2.getH(temp2);



			Assert::AreEqual(temp3[0], temp2[0], 1e-3f);
			Assert::AreEqual(temp3[1], temp2[1], 1e-3f);
			Assert::AreEqual(temp3[2], temp2[2], 1e-3f);

			Assert::AreEqual(temp3[3], temp2[3], 1e-3f);
			Assert::AreEqual(temp3[4], temp2[4], 1e-3f);
			Assert::AreEqual(temp3[5], temp2[5], 1e-3f);

			Assert::AreEqual(temp3[6], temp2[6], 1e-3f);
			Assert::AreEqual(temp3[7], temp2[7], 1e-3f);
			Assert::AreEqual(temp3[8], temp2[8], 1e-3f);
		}


		[TestMethod]
		void testMultiply(void)
		{

						
			float temp1[9] = 
			{
			   0.814723686393179, 0.913375856139019, 0.278498218867048,
			   0.905791937075619, 0.632359246225410, 0.546881519204984,
			   0.126986816293506, 0.097540404999410, 0.957506835434298
			};
			float temp2[9] = 
			{
			   0.964888535199277, 0.957166948242946, 0.141886338627215,
			   0.157613081677548, 0.485375648722841, 0.421761282626275,
			   0.970592781760616, 0.800280468888800, 0.915735525189067
			};

			float temp3[9] = 
			{
			   1.200385888737554, 1.446033668447076, 0.755855446178697,
			   1.504455599903725, 1.611584482132265, 0.896022983483697,
			   1.067250989942073, 0.935165339997611, 0.935979485556798
			};
			float temp4[9];

			Rotation test1;
			test1.setH(temp1);

			Rotation test2(test1);
			test2.setH(temp2);

			Rotation test3 = test1.mul(test2);
			test3.getH(temp4);


			Assert::AreEqual(temp3[0], temp4[0], 1e-3f);
			Assert::AreEqual(temp3[1], temp4[1], 1e-3f);
			Assert::AreEqual(temp3[2], temp4[2], 1e-3f);

			Assert::AreEqual(temp3[3], temp4[3], 1e-3f);
			Assert::AreEqual(temp3[4], temp4[4], 1e-3f);
			Assert::AreEqual(temp3[5], temp4[5], 1e-3f);

			Assert::AreEqual(temp3[6], temp4[6], 1e-3f);
			Assert::AreEqual(temp3[7], temp4[7], 1e-3f);
			Assert::AreEqual(temp3[8], temp4[8], 1e-3f);
		}

		[TestMethod]
		void testRoll(void)
		{
			float temp3[9] = 
			{

			   1.0, 0, 0,
				0, 0.686069447756349, -0.727536056051725, 
				0, 0.727536056051725, 0.686069447756349 				
			};
			float roll = 0.814723686393179;
			float pitch = 0.0f;
			float yaw = 0.0f;
			float temp2[9];

			Rotation test1;
			test1.initRoll(roll);
			test1.getH(temp2);

			Assert::AreEqual(temp3[0], temp2[0], 1e-3f);
			Assert::AreEqual(temp3[1], temp2[1], 1e-3f);
			Assert::AreEqual(temp3[2], temp2[2], 1e-3f);

			Assert::AreEqual(temp3[3], temp2[3], 1e-3f);
			Assert::AreEqual(temp3[4], temp2[4], 1e-3f);
			Assert::AreEqual(temp3[5], temp2[5], 1e-3f);

			Assert::AreEqual(temp3[6], temp2[6], 1e-3f);
			Assert::AreEqual(temp3[7], temp2[7], 1e-3f);
			Assert::AreEqual(temp3[8], temp2[8], 1e-3f);
		}

		[TestMethod]
		void testPitch(void)
		{
			float temp3[16] = 
			{

				0.686069447756349, 0, 0.727536056051725,
                0, 1.0, 0, 
				-0.727536056051725, 0, 0.686069447756349,                  
			};
			float roll = 0.0f;
			float pitch = 0.814723686393179f;
			float yaw = 0.0f;
			float temp2[9];

			Rotation test1;
			test1.initPitch(pitch);
			test1.getH(temp2);

			Assert::AreEqual(temp3[0], temp2[0], 1e-3f);
			Assert::AreEqual(temp3[1], temp2[1], 1e-3f);
			Assert::AreEqual(temp3[2], temp2[2], 1e-3f);

			Assert::AreEqual(temp3[3], temp2[3], 1e-3f);
			Assert::AreEqual(temp3[4], temp2[4], 1e-3f);
			Assert::AreEqual(temp3[5], temp2[5], 1e-3f);

			Assert::AreEqual(temp3[6], temp2[6], 1e-3f);
			Assert::AreEqual(temp3[7], temp2[7], 1e-3f);
			Assert::AreEqual(temp3[8], temp2[8], 1e-3f);
		}

		[TestMethod]
		void testYaw(void)
		{
			float temp3[9] = 
			{

				0.686069447756349, -0.727536056051725, 0,
				0.727536056051725, 0.686069447756349, 0, 
                0, 0, 1.0, 
                
			};
			float roll = 0.0f;
			float pitch = 0.0f;
			float yaw = 0.814723686393179f;
			float temp2[9];

			Rotation test1;
			test1.initYaw(yaw);
			test1.getH(temp2);

			Assert::AreEqual(temp3[0], temp2[0], 1e-3f);
			Assert::AreEqual(temp3[1], temp2[1], 1e-3f);
			Assert::AreEqual(temp3[2], temp2[2], 1e-3f);

			Assert::AreEqual(temp3[3], temp2[3], 1e-3f);
			Assert::AreEqual(temp3[4], temp2[4], 1e-3f);
			Assert::AreEqual(temp3[5], temp2[5], 1e-3f);

			Assert::AreEqual(temp3[6], temp2[6], 1e-3f);
			Assert::AreEqual(temp3[7], temp2[7], 1e-3f);
			Assert::AreEqual(temp3[8], temp2[8], 1e-3f);

		}

		[TestMethod]
		void testRollPitchYaw(void)
		{
			float temp3[9] = 
			{

				0.470691287144702,  -0.499140260198239,   0.727536056051725,
				0.862282796519497,   0.085600113760143,  -0.499140260198239,
				0.186863830188132,   0.862282796519497,   0.470691287144702,                
			};
			float roll = 0.814723686393179f;
			float pitch = 0.814723686393179f;
			float yaw = 0.814723686393179f;
			float temp2[9];


			Rotation test1;
			test1.initRoll(roll);
			Rotation test2;
			test2.initPitch(pitch);
			Rotation test3;
			test3.initYaw(yaw);
			Rotation test4 = test1.mul(test2);
			Rotation test5 = test4.mul(test3);				
			test5.getH(temp2);

			Assert::AreEqual(temp3[0], temp2[0], 1e-3f);
			Assert::AreEqual(temp3[1], temp2[1], 1e-3f);
			Assert::AreEqual(temp3[2], temp2[2], 1e-3f);

			Assert::AreEqual(temp3[3], temp2[3], 1e-3f);
			Assert::AreEqual(temp3[4], temp2[4], 1e-3f);
			Assert::AreEqual(temp3[5], temp2[5], 1e-3f);

			Assert::AreEqual(temp3[6], temp2[6], 1e-3f);
			Assert::AreEqual(temp3[7], temp2[7], 1e-3f);
			Assert::AreEqual(temp3[8], temp2[8], 1e-3f);

		}


	};
}
