#include "stdafx.h"

using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace	Microsoft::VisualStudio::TestTools::UnitTesting;


#include "../CFObject/HomogeneTransformation.h"
#include <float.h>
#include <stdlib.h>
#include <time.h>
namespace TestDriver
{
	[TestClass]
	public ref class HomogeneTransformationTest
	{
	public: 

		[TestMethod]
		void TestConstructor()
		{
			HomogeneTransformation test1;


			Assert::AreEqual(test1.getH()[0], 1.0f);
			Assert::AreEqual(test1.getH()[1], 0.0f);
			Assert::AreEqual(test1.getH()[2], 0.0f);
			Assert::AreEqual(test1.getH()[3], 0.0f);

			Assert::AreEqual(test1.getH()[4], 0.0f);
			Assert::AreEqual(test1.getH()[5], 1.0f);
			Assert::AreEqual(test1.getH()[6], 0.0f);
			Assert::AreEqual(test1.getH()[7], 0.0f);

			Assert::AreEqual(test1.getH()[8], 0.0f);
			Assert::AreEqual(test1.getH()[9], 0.0f);
			Assert::AreEqual(test1.getH()[10], 1.0f);
			Assert::AreEqual(test1.getH()[11], 0.0f);

			Assert::AreEqual(test1.getH()[12], 0.0f);
			Assert::AreEqual(test1.getH()[13], 0.0f);
			Assert::AreEqual(test1.getH()[14], 0.0f);
			Assert::AreEqual(test1.getH()[15], 1.0f);
		}

		[TestMethod]
		void TestGetter()
		{
			HomogeneTransformation test;

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
		}

		[TestMethod]
		void TestSetter()
		{
			HomogeneTransformation test;			
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
			
			Assert::AreEqual(test.getH()[0], temp2[0]);
			Assert::AreEqual(test.getH()[1], temp2[1]);
			Assert::AreEqual(test.getH()[2], temp2[2]);
			Assert::AreEqual(test.getH()[3], temp2[3]);

			Assert::AreEqual(test.getH()[4], temp2[4]);
			Assert::AreEqual(test.getH()[5], temp2[5]);
			Assert::AreEqual(test.getH()[6], temp2[6]);
			Assert::AreEqual(test.getH()[7], temp2[7]);

			Assert::AreEqual(test.getH()[8], temp2[8]);
			Assert::AreEqual(test.getH()[9], temp2[9]);
			Assert::AreEqual(test.getH()[10],temp2[10]);
			Assert::AreEqual(test.getH()[11],temp2[11]);

			Assert::AreEqual(test.getH()[12], temp2[12]);
			Assert::AreEqual(test.getH()[13], temp2[13]);
			Assert::AreEqual(test.getH()[14], temp2[14]);
			Assert::AreEqual(test.getH()[15], temp2[15]);
		}

		[TestMethod]
		void TestIsqual1()
		{
			HomogeneTransformation test1;
			HomogeneTransformation test2;
			Assert::IsTrue(test1.isEqual(test2));


		}

		[TestMethod]
		void TestIsqual2()
		{
			float tmp[] = EYE_16;
			tmp[0] = 2;
			HomogeneTransformation test1(tmp);
			HomogeneTransformation test2;
			Assert::IsFalse(test1.isEqual(test2));


		}



		[TestMethod]
		void TestInverse()
		{
			HomogeneTransformation test;
			float temp1[16] = 
			{
				0.915735525189067, 0.035711678574190, 0.757740130578333, 0.171186687811562,
				0.792207329559554, 0.849129305868777, 0.743132468124916, 0.706046088019609,
				0.959492426392903, 0.933993247757551, 0.392227019534168, 0.031832846377421,
				0.655740699156587, 0.678735154857773, 0.655477890177557, 0.276922984960890
			};			
			float temp3[16] = 
			{
			   1.072870643907527, 0.881846074808848, 1.389159384428151, -3.071272833251428,
			  -1.128394343681584, -0.312329315993337, 0.340627711626420, 1.454707099924857,
			   0.054700766484468, -1.616730820282032, -1.640194500065132, 4.276764847535353,
			   0.095697438969413,   2.504154114094748,  -0.241991896087627,  -2.804848622157487			
			};

			test.setH(temp1);
			HomogeneTransformation test2 = test.inv();
			



			Assert::AreEqual(temp3[0], test2.getH()[0], 1e-3f);
			Assert::AreEqual(temp3[1], test2.getH()[1], 1e-3f);
			Assert::AreEqual(temp3[2], test2.getH()[2], 1e-3f);
			Assert::AreEqual(temp3[3], test2.getH()[3], 1e-3f);

			Assert::AreEqual(temp3[4], test2.getH()[4], 1e-3f);
			Assert::AreEqual(temp3[5], test2.getH()[5], 1e-3f);
			Assert::AreEqual(temp3[6], test2.getH()[6], 1e-3f);
			Assert::AreEqual(temp3[7], test2.getH()[7], 1e-3f);

			Assert::AreEqual(temp3[8], test2.getH()[8], 1e-3f);
			Assert::AreEqual(temp3[9], test2.getH()[9], 1e-3f);
			Assert::AreEqual(temp3[10],test2.getH()[10], 1e-3f);
			Assert::AreEqual(temp3[11],test2.getH()[11], 1e-3f);

			Assert::AreEqual(temp3[12], test2.getH()[12], 1e-3f);
			Assert::AreEqual(temp3[13], test2.getH()[13], 1e-3f);
			Assert::AreEqual(temp3[14], test2.getH()[14], 1e-3f);
			Assert::AreEqual(temp3[15], test2.getH()[15], 1e-3f);

		}

		[TestMethod]
		void TestInverseAlgo()
		{
			HomogeneTransformation test;
			float mat[16] = 
			{
				0.915735525189067, 0.035711678574190, 0.757740130578333, 0.171186687811562,
				0.792207329559554, 0.849129305868777, 0.743132468124916, 0.706046088019609,
				0.959492426392903, 0.933993247757551, 0.392227019534168, 0.031832846377421,
				0.655740699156587, 0.678735154857773, 0.655477890177557, 0.276922984960890
			};
			float dst[16];
			float temp3[16] = 
			{
			   1.072870643907527, 0.881846074808848, 1.389159384428151, -3.071272833251428,
			  -1.128394343681584, -0.312329315993337, 0.340627711626420, 1.454707099924857,
			   0.054700766484468, -1.616730820282032, -1.640194500065132, 4.276764847535353,
			   0.095697438969413,   2.504154114094748,  -0.241991896087627,  -2.804848622157487
			};


			float tmp[12]; /* temp array for pairs */
			float src[16]; /* array of transpose source matrix */
			float det; /* determinant */
			/* transpose matrix */
			for (int i = 0; i < 4; i++) {
				src[i] = mat[i*4];
				src[i + 4] = mat[i*4 + 1];
				src[i + 8] = mat[i*4 + 2];
				src[i + 12] = mat[i*4 + 3];
			}
			/* calculate pairs for first 8 elements (cofactors) */
			tmp[0] = src[10] * src[15];
			tmp[1] = src[11] * src[14];
			tmp[2] = src[9] * src[15];
			tmp[3] = src[11] * src[13];
			tmp[4] = src[9] * src[14];
			tmp[5] = src[10] * src[13];
			tmp[6] = src[8] * src[15];
			tmp[7] = src[11] * src[12];
			tmp[8] = src[8] * src[14];
			tmp[9] = src[10] * src[12];
			tmp[10] = src[8] * src[13];
			tmp[11] = src[9] * src[12];
			/* calculate first 8 elements (cofactors) */
			dst[0] = tmp[0]*src[5] + tmp[3]*src[6] + tmp[4]*src[7];
			dst[0] -= tmp[1]*src[5] + tmp[2]*src[6] + tmp[5]*src[7];
			dst[1] = tmp[1]*src[4] + tmp[6]*src[6] + tmp[9]*src[7];
			dst[1] -= tmp[0]*src[4] + tmp[7]*src[6] + tmp[8]*src[7];
			dst[2] = tmp[2]*src[4] + tmp[7]*src[5] + tmp[10]*src[7];
			dst[2] -= tmp[3]*src[4] + tmp[6]*src[5] + tmp[11]*src[7];
			dst[3] = tmp[5]*src[4] + tmp[8]*src[5] + tmp[11]*src[6];
			dst[3] -= tmp[4]*src[4] + tmp[9]*src[5] + tmp[10]*src[6];
			dst[4] = tmp[1]*src[1] + tmp[2]*src[2] + tmp[5]*src[3];
			dst[4] -= tmp[0]*src[1] + tmp[3]*src[2] + tmp[4]*src[3];
			dst[5] = tmp[0]*src[0] + tmp[7]*src[2] + tmp[8]*src[3];
			dst[5] -= tmp[1]*src[0] + tmp[6]*src[2] + tmp[9]*src[3];
			dst[6] = tmp[3]*src[0] + tmp[6]*src[1] + tmp[11]*src[3];
			dst[6] -= tmp[2]*src[0] + tmp[7]*src[1] + tmp[10]*src[3];
			dst[7] = tmp[4]*src[0] + tmp[9]*src[1] + tmp[10]*src[2];
			dst[7] -= tmp[5]*src[0] + tmp[8]*src[1] + tmp[11]*src[2];
			/* calculate pairs for second 8 elements (cofactors) */
			tmp[0] = src[2]*src[7];
			tmp[1] = src[3]*src[6];
			tmp[2] = src[1]*src[7];
			tmp[3] = src[3]*src[5];
			tmp[4] = src[1]*src[6];
			tmp[5] = src[2]*src[5];

			tmp[6] = src[0]*src[7];
			tmp[7] = src[3]*src[4];
			tmp[8] = src[0]*src[6];
			tmp[9] = src[2]*src[4];
			tmp[10] = src[0]*src[5];
			tmp[11] = src[1]*src[4];
			/* calculate second 8 elements (cofactors) */
			dst[8] = tmp[0]*src[13] + tmp[3]*src[14] + tmp[4]*src[15];
			dst[8] -= tmp[1]*src[13] + tmp[2]*src[14] + tmp[5]*src[15];
			dst[9] = tmp[1]*src[12] + tmp[6]*src[14] + tmp[9]*src[15];
			dst[9] -= tmp[0]*src[12] + tmp[7]*src[14] + tmp[8]*src[15];
			dst[10] = tmp[2]*src[12] + tmp[7]*src[13] + tmp[10]*src[15];
			dst[10]-= tmp[3]*src[12] + tmp[6]*src[13] + tmp[11]*src[15];
			dst[11] = tmp[5]*src[12] + tmp[8]*src[13] + tmp[11]*src[14];
			dst[11]-= tmp[4]*src[12] + tmp[9]*src[13] + tmp[10]*src[14];
			dst[12] = tmp[2]*src[10] + tmp[5]*src[11] + tmp[1]*src[9];
			dst[12]-= tmp[4]*src[11] + tmp[0]*src[9] + tmp[3]*src[10];
			dst[13] = tmp[8]*src[11] + tmp[0]*src[8] + tmp[7]*src[10];
			dst[13]-= tmp[6]*src[10] + tmp[9]*src[11] + tmp[1]*src[8];
			dst[14] = tmp[6]*src[9] + tmp[11]*src[11] + tmp[3]*src[8];
			dst[14]-= tmp[10]*src[11] + tmp[2]*src[8] + tmp[7]*src[9];
			dst[15] = tmp[10]*src[10] + tmp[4]*src[8] + tmp[9]*src[9];
			dst[15]-= tmp[8]*src[9] + tmp[11]*src[10] + tmp[5]*src[8];
			/* calculate determinant */
			det=src[0]*dst[0]+src[1]*dst[1]+src[2]*dst[2]+src[3]*dst[3];
			/* calculate matrix inverse */
			det = 1/det;
			for (int j = 0; j < 16; j++)
				dst[j] *= det;




			Assert::AreEqual(temp3[0], dst[0], 1e-3f);
			Assert::AreEqual(temp3[1], dst[1], 1e-3f);
			Assert::AreEqual(temp3[2], dst[2], 1e-3f);
			Assert::AreEqual(temp3[3], dst[3], 1e-3f);

			Assert::AreEqual(temp3[4], dst[4], 1e-3f);
			Assert::AreEqual(temp3[5], dst[5], 1e-3f);
			Assert::AreEqual(temp3[6], dst[6], 1e-3f);
			Assert::AreEqual(temp3[7], dst[7], 1e-3f);

			Assert::AreEqual(temp3[8], dst[8], 1e-3f);
			Assert::AreEqual(temp3[9], dst[9], 1e-3f);
			Assert::AreEqual(temp3[10],dst[10], 1e-3f);
			Assert::AreEqual(temp3[11],dst[11], 1e-3f);

			Assert::AreEqual(temp3[12], dst[12], 1e-3f);
			Assert::AreEqual(temp3[13], dst[13], 1e-3f);
			Assert::AreEqual(temp3[14], dst[14], 1e-3f);
			Assert::AreEqual(temp3[15], dst[15], 1e-3f);

		}

		[TestMethod]
		void testCopyConstructor(void)
		{

						
			float temp1[16] = 
			{
				0.915735525189067, 0.035711678574190, 0.757740130578333, 0.171186687811562,
				0.792207329559554, 0.849129305868777, 0.743132468124916, 0.706046088019609,
				0.959492426392903, 0.933993247757551, 0.392227019534168, 0.031832846377421,
				0.655740699156587, 0.678735154857773, 0.655477890177557, 0.276922984960890
			};			
			float temp3[16] = 
			{
				0.915735525189067, 0.035711678574190, 0.757740130578333, 0.171186687811562,
				0.792207329559554, 0.849129305868777, 0.743132468124916, 0.706046088019609,
				0.959492426392903, 0.933993247757551, 0.392227019534168, 0.031832846377421,
				0.655740699156587, 0.678735154857773, 0.655477890177557, 0.276922984960890
			};

			HomogeneTransformation test1;
			test1.setH(temp1);
			HomogeneTransformation test2(test1);
	



			Assert::AreEqual(temp3[0], test2.getH()[0], 1e-3f);
			Assert::AreEqual(temp3[1], test2.getH()[1], 1e-3f);
			Assert::AreEqual(temp3[2], test2.getH()[2], 1e-3f);
			Assert::AreEqual(temp3[3], test2.getH()[3], 1e-3f);

			Assert::AreEqual(temp3[4], test2.getH()[4], 1e-3f);
			Assert::AreEqual(temp3[5], test2.getH()[5], 1e-3f);
			Assert::AreEqual(temp3[6], test2.getH()[6], 1e-3f);
			Assert::AreEqual(temp3[7], test2.getH()[7], 1e-3f);

			Assert::AreEqual(temp3[8], test2.getH()[8], 1e-3f);
			Assert::AreEqual(temp3[9], test2.getH()[9], 1e-3f);
			Assert::AreEqual(temp3[10],test2.getH()[10], 1e-3f);
			Assert::AreEqual(temp3[11],test2.getH()[11], 1e-3f);

			Assert::AreEqual(temp3[12], test2.getH()[12], 1e-3f);
			Assert::AreEqual(temp3[13], test2.getH()[13], 1e-3f);
			Assert::AreEqual(temp3[14], test2.getH()[14], 1e-3f);
			Assert::AreEqual(temp3[15], test2.getH()[15], 1e-3f);

		}


		[TestMethod]
		void testMultiply(void)
		{

						
			float temp1[16] = 
			{
			   0.814723686393179, 0.632359246225410, 0.957506835434298, 0.957166948242946,
			   0.905791937075619, 0.097540404999410, 0.964888535199277, 0.485375648722841,
			   0.126986816293506, 0.278498218867048, 0.157613081677548, 0.800280468888800,
			   0.913375856139019, 0.546881519204984, 0.970592781760616, 0.141886338627215,
			};
			float temp2[16] = 
			{
			   0.421761282626275, 0.655740699156587, 0.678735154857773, 0.655477890177557,
			   0.915735525189067, 0.035711678574190, 0.757740130578333, 0.171186687811562,
			   0.792207329559554, 0.849129305868777, 0.743132468124916, 0.706046088019609,
			   0.959492426392903, 0.933993247757551, 0.392227019534168, 0.031832846377421,
			};

			float temp3[16] = 
			{
			   2.599631104176721, 2.263864671052060, 2.119126742426342, 1.348798151734409,
			   1.701455211870797, 1.870100680367278, 1.596120552581956, 1.307130870827371,
			   1.201314122663400, 0.974506503470718, 0.728238714651490, 0.267669543037837,
			   1.791074990860224, 1.575147436813857, 1.811265042099493, 1.382116397661908
			};
			
			

			HomogeneTransformation test1;
			test1.setH(temp1);

			HomogeneTransformation test2(test1);
			test2.setH(temp2);

			HomogeneTransformation test3 = test1.mul(test2);
			



			Assert::AreEqual(temp3[0], test3.getH()[0], 1e-3f);
			Assert::AreEqual(temp3[1], test3.getH()[1], 1e-3f);
			Assert::AreEqual(temp3[2], test3.getH()[2], 1e-3f);
			Assert::AreEqual(temp3[3], test3.getH()[3], 1e-3f);

			Assert::AreEqual(temp3[4], test3.getH()[4], 1e-3f);
			Assert::AreEqual(temp3[5], test3.getH()[5], 1e-3f);
			Assert::AreEqual(temp3[6], test3.getH()[6], 1e-3f);
			Assert::AreEqual(temp3[7], test3.getH()[7], 1e-3f);

			Assert::AreEqual(temp3[8], test3.getH()[8], 1e-3f);
			Assert::AreEqual(temp3[9], test3.getH()[9], 1e-3f);
			Assert::AreEqual(temp3[10],test3.getH()[10], 1e-3f);
			Assert::AreEqual(temp3[11],test3.getH()[11], 1e-3f);

			Assert::AreEqual(temp3[12], test3.getH()[12], 1e-3f);
			Assert::AreEqual(temp3[13], test3.getH()[13], 1e-3f);
			Assert::AreEqual(temp3[14], test3.getH()[14], 1e-3f);
			Assert::AreEqual(temp3[15], test3.getH()[15], 1e-3f);

		}

		[TestMethod]
		void testRoll(void)
		{
			float temp3[16] = 
			{

			   1.0, 0, 0, 0,
				0, 0.686069447756349, -0.727536056051725, 0,
				0, 0.727536056051725, 0.686069447756349, 0,
				0, 0, 0, 1.000000000000000
			};
			float roll = 0.814723686393179;
			float pitch = 0.0f;
			float yaw = 0.0f;
			float x = 0.0f;
			float y = 0.0f;
			float z = 0.0f;
			

			HomogeneTransformation test1;
			test1.init(roll, pitch, yaw, x, y, z);


			Assert::AreEqual(temp3[0], test1.getH()[0], 1e-3f);
			Assert::AreEqual(temp3[1], test1.getH()[1], 1e-3f);
			Assert::AreEqual(temp3[2], test1.getH()[2], 1e-3f);
			Assert::AreEqual(temp3[3], test1.getH()[3], 1e-3f);

			Assert::AreEqual(temp3[4], test1.getH()[4], 1e-3f);
			Assert::AreEqual(temp3[5], test1.getH()[5], 1e-3f);
			Assert::AreEqual(temp3[6], test1.getH()[6], 1e-3f);
			Assert::AreEqual(temp3[7], test1.getH()[7], 1e-3f);

			Assert::AreEqual(temp3[8], test1.getH()[8], 1e-3f);
			Assert::AreEqual(temp3[9], test1.getH()[9], 1e-3f);
			Assert::AreEqual(temp3[10],test1.getH()[10], 1e-3f);
			Assert::AreEqual(temp3[11],test1.getH()[11], 1e-3f);

			Assert::AreEqual(temp3[12], test1.getH()[12], 1e-3f);
			Assert::AreEqual(temp3[13], test1.getH()[13], 1e-3f);
			Assert::AreEqual(temp3[14], test1.getH()[14], 1e-3f);
			Assert::AreEqual(temp3[15], test1.getH()[15], 1e-3f);
		}

		[TestMethod]
		void testPitch(void)
		{
			float temp3[16] = 
			{

				0.686069447756349, 0, 0.727536056051725, 0,
                0, 1.0, 0, 0,
				-0.727536056051725, 0, 0.686069447756349, 0,
                  0, 0, 0, 1.0
			};
			float roll = 0.0f;
			float pitch = 0.814723686393179f;
			float yaw = 0.0f;
			float x = 0.0f;
			float y = 0.0f;
			float z = 0.0f;
			

			HomogeneTransformation test1;
			test1.init(roll, pitch, yaw, x, y, z);


			Assert::AreEqual(temp3[0], test1.getH()[0], 1e-3f);
			Assert::AreEqual(temp3[1], test1.getH()[1], 1e-3f);
			Assert::AreEqual(temp3[2], test1.getH()[2], 1e-3f);
			Assert::AreEqual(temp3[3], test1.getH()[3], 1e-3f);

			Assert::AreEqual(temp3[4], test1.getH()[4], 1e-3f);
			Assert::AreEqual(temp3[5], test1.getH()[5], 1e-3f);
			Assert::AreEqual(temp3[6], test1.getH()[6], 1e-3f);
			Assert::AreEqual(temp3[7], test1.getH()[7], 1e-3f);

			Assert::AreEqual(temp3[8], test1.getH()[8], 1e-3f);
			Assert::AreEqual(temp3[9], test1.getH()[9], 1e-3f);
			Assert::AreEqual(temp3[10],test1.getH()[10], 1e-3f);
			Assert::AreEqual(temp3[11],test1.getH()[11], 1e-3f);

			Assert::AreEqual(temp3[12], test1.getH()[12], 1e-3f);
			Assert::AreEqual(temp3[13], test1.getH()[13], 1e-3f);
			Assert::AreEqual(temp3[14], test1.getH()[14], 1e-3f);
			Assert::AreEqual(temp3[15], test1.getH()[15], 1e-3f);
		}

		[TestMethod]
		void testYaw(void)
		{
			float temp3[16] = 
			{

				0.686069447756349, -0.727536056051725, 0, 0,
				0.727536056051725, 0.686069447756349, 0, 0,
                0, 0, 1.0, 0,
                0, 0, 0, 1.0
			};
			float roll = 0.0f;
			float pitch = 0.0f;
			float yaw = 0.814723686393179f;
			float x = 0.0f;
			float y = 0.0f;
			float z = 0.0f;
			

			HomogeneTransformation test1;
			test1.init(roll, pitch, yaw, x, y, z);
			
			Assert::AreEqual(temp3[0], test1.getH()[0], 1e-3f);
			Assert::AreEqual(temp3[1], test1.getH()[1], 1e-3f);
			Assert::AreEqual(temp3[2], test1.getH()[2], 1e-3f);
			Assert::AreEqual(temp3[3], test1.getH()[3], 1e-3f);

			Assert::AreEqual(temp3[4], test1.getH()[4], 1e-3f);
			Assert::AreEqual(temp3[5], test1.getH()[5], 1e-3f);
			Assert::AreEqual(temp3[6], test1.getH()[6], 1e-3f);
			Assert::AreEqual(temp3[7], test1.getH()[7], 1e-3f);

			Assert::AreEqual(temp3[8], test1.getH()[8], 1e-3f);
			Assert::AreEqual(temp3[9], test1.getH()[9], 1e-3f);
			Assert::AreEqual(temp3[10],test1.getH()[10], 1e-3f);
			Assert::AreEqual(temp3[11],test1.getH()[11], 1e-3f);

			Assert::AreEqual(temp3[12], test1.getH()[12], 1e-3f);
			Assert::AreEqual(temp3[13], test1.getH()[13], 1e-3f);
			Assert::AreEqual(temp3[14], test1.getH()[14], 1e-3f);
			Assert::AreEqual(temp3[15], test1.getH()[15], 1e-3f);
		}

		[TestMethod]
		void testRollPitchYaw(void)
		{
			float temp3[16] = 
			{

				0.470691287144702,  -0.499140260198239,   0.727536056051725, 0,
				0.862282796519497,   0.085600113760143,  -0.499140260198239, 0,
				0.186863830188132,   0.862282796519497,   0.470691287144702, 0,
                0,                   0,               0,   1.000000000000000
			};
			float roll = 0.814723686393179f;
			float pitch = 0.814723686393179f;
			float yaw = 0.814723686393179f;
			float x = 0.0f;
			float y = 0.0f;
			float z = 0.0f;
			

			HomogeneTransformation test1;
			test1.init(roll, pitch, yaw, x, y, z);
			

			Assert::AreEqual(temp3[0], test1.getH()[0], 1e-3f);
			Assert::AreEqual(temp3[1], test1.getH()[1], 1e-3f);
			Assert::AreEqual(temp3[2], test1.getH()[2], 1e-3f);
			Assert::AreEqual(temp3[3], test1.getH()[3], 1e-3f);

			Assert::AreEqual(temp3[4], test1.getH()[4], 1e-3f);
			Assert::AreEqual(temp3[5], test1.getH()[5], 1e-3f);
			Assert::AreEqual(temp3[6], test1.getH()[6], 1e-3f);
			Assert::AreEqual(temp3[7], test1.getH()[7], 1e-3f);

			Assert::AreEqual(temp3[8], test1.getH()[8], 1e-3f);
			Assert::AreEqual(temp3[9], test1.getH()[9], 1e-3f);
			Assert::AreEqual(temp3[10],test1.getH()[10], 1e-3f);
			Assert::AreEqual(temp3[11],test1.getH()[11], 1e-3f);

			Assert::AreEqual(temp3[12], test1.getH()[12], 1e-3f);
			Assert::AreEqual(temp3[13], test1.getH()[13], 1e-3f);
			Assert::AreEqual(temp3[14], test1.getH()[14], 1e-3f);
			Assert::AreEqual(temp3[15], test1.getH()[15], 1e-3f);
		}

		[TestMethod]
		void testRollPitchYawXYZ(void)
		{
			float temp3[16] = 
			{

				0.470691287144702,  -0.499140260198239,   0.727536056051725, 0.814723686393179,
				0.862282796519497,   0.085600113760143,  -0.499140260198239, 0.814723686393179,
				0.186863830188132,   0.862282796519497,   0.470691287144702, 0.814723686393179,
                0,                   0,               0,   1.000000000000000
			};
			float roll = 0.814723686393179;
			float pitch = 0.814723686393179f;
			float yaw = 0.814723686393179f;
			float x = 0.814723686393179f;
			float y = 0.814723686393179f;
			float z = 0.814723686393179f;

			

			HomogeneTransformation test1;
			test1.init(roll, pitch, yaw, x, y, z);
			

			Assert::AreEqual(temp3[0], test1.getH()[0], 1e-3f);
			Assert::AreEqual(temp3[1], test1.getH()[1], 1e-3f);
			Assert::AreEqual(temp3[2], test1.getH()[2], 1e-3f);
			Assert::AreEqual(temp3[3], test1.getH()[3], 1e-3f);

			Assert::AreEqual(temp3[4], test1.getH()[4], 1e-3f);
			Assert::AreEqual(temp3[5], test1.getH()[5], 1e-3f);
			Assert::AreEqual(temp3[6], test1.getH()[6], 1e-3f);
			Assert::AreEqual(temp3[7], test1.getH()[7], 1e-3f);

			Assert::AreEqual(temp3[8], test1.getH()[8], 1e-3f);
			Assert::AreEqual(temp3[9], test1.getH()[9], 1e-3f);
			Assert::AreEqual(temp3[10],test1.getH()[10], 1e-3f);
			Assert::AreEqual(temp3[11],test1.getH()[11], 1e-3f);

			Assert::AreEqual(temp3[12], test1.getH()[12], 1e-3f);
			Assert::AreEqual(temp3[13], test1.getH()[13], 1e-3f);
			Assert::AreEqual(temp3[14], test1.getH()[14], 1e-3f);
			Assert::AreEqual(temp3[15], test1.getH()[15], 1e-3f);
		}

		[TestMethod]
		void testDistDimXDiff(void)
		{

			HomogeneTransformation a;
			a.init(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

			HomogeneTransformation b;
			b.init(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);

			Assert::AreEqual(1.0f,		a.getDist(b, HomogeneTransformation::DIM_DIR::XP));
			Assert::AreEqual(FLT_MAX,	b.getDist(a, HomogeneTransformation::DIM_DIR::XP));

			Assert::AreEqual(FLT_MAX,	a.getDist(b, HomogeneTransformation::DIM_DIR::XM));
			Assert::AreEqual(1.0f,		b.getDist(a, HomogeneTransformation::DIM_DIR::XM));

		}

		[TestMethod]
		void testDistDimXEqual(void)
		{

			HomogeneTransformation a;
			a.init(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

			HomogeneTransformation b;
			b.init(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

			Assert::AreEqual(0.0f,	a.getDist(b, HomogeneTransformation::DIM_DIR::XP));
			Assert::AreEqual(0.0f,	b.getDist(a, HomogeneTransformation::DIM_DIR::XP));

			Assert::AreEqual(0.0f,	a.getDist(b, HomogeneTransformation::DIM_DIR::XM));
			Assert::AreEqual(0.0f,	b.getDist(a, HomogeneTransformation::DIM_DIR::XM));

		}

		[TestMethod]
		void testDistDimYDiff(void)
		{

			HomogeneTransformation a;
			a.init(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

			HomogeneTransformation b;
			b.init(0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

			Assert::AreEqual(1.0f,		a.getDist(b, HomogeneTransformation::DIM_DIR::YP));
			Assert::AreEqual(FLT_MAX,	b.getDist(a, HomogeneTransformation::DIM_DIR::YP));

			Assert::AreEqual(FLT_MAX,	a.getDist(b, HomogeneTransformation::DIM_DIR::YM));
			Assert::AreEqual(1.0f,		b.getDist(a, HomogeneTransformation::DIM_DIR::YM));

		}

		[TestMethod]
		void testDistDimYEqual(void)
		{

			HomogeneTransformation a;
			a.init(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

			HomogeneTransformation b;
			b.init(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

			Assert::AreEqual(0.0f,	a.getDist(b, HomogeneTransformation::DIM_DIR::YP));
			Assert::AreEqual(0.0f,	b.getDist(a, HomogeneTransformation::DIM_DIR::YP));

			Assert::AreEqual(0.0f,	a.getDist(b, HomogeneTransformation::DIM_DIR::YM));
			Assert::AreEqual(0.0f,	b.getDist(a, HomogeneTransformation::DIM_DIR::YM));

		}

		[TestMethod]
		void testDistDimZDiff(void)
		{

			HomogeneTransformation a;
			a.init(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

			HomogeneTransformation b;
			b.init(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

			Assert::AreEqual(1.0f,		a.getDist(b, HomogeneTransformation::DIM_DIR::ZP));
			Assert::AreEqual(FLT_MAX,	b.getDist(a, HomogeneTransformation::DIM_DIR::ZP));

			Assert::AreEqual(FLT_MAX,	a.getDist(b, HomogeneTransformation::DIM_DIR::ZM));
			Assert::AreEqual(1.0f,		b.getDist(a, HomogeneTransformation::DIM_DIR::ZM));

		}

		[TestMethod]
		void testDistDimZEqual(void)
		{

			HomogeneTransformation a;
			a.init(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

			HomogeneTransformation b;
			b.init(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

			Assert::AreEqual(0.0f,	a.getDist(b, HomogeneTransformation::DIM_DIR::ZP));
			Assert::AreEqual(0.0f,	b.getDist(a, HomogeneTransformation::DIM_DIR::ZP));

			Assert::AreEqual(0.0f,	a.getDist(b, HomogeneTransformation::DIM_DIR::ZM));
			Assert::AreEqual(0.0f,	b.getDist(a, HomogeneTransformation::DIM_DIR::ZM));

		}

		[TestMethod]
		void testDistDimRollDiff(void)
		{

			HomogeneTransformation a;
			a.init(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

			HomogeneTransformation b;
			b.init(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);

			Assert::AreEqual(1.0f,		a.getDist(b, HomogeneTransformation::DIM_DIR::ROLLP));
			Assert::AreEqual(FLT_MAX,	b.getDist(a, HomogeneTransformation::DIM_DIR::ROLLP));

			Assert::AreEqual(FLT_MAX,	a.getDist(b, HomogeneTransformation::DIM_DIR::ROLLM));
			Assert::AreEqual(1.0f,		b.getDist(a, HomogeneTransformation::DIM_DIR::ROLLM));

		}

		[TestMethod]
		void testDistDimRollEqual(void)
		{

			HomogeneTransformation a;
			a.init(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

			HomogeneTransformation b;
			b.init(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

			Assert::AreEqual(0.0f,	a.getDist(b, HomogeneTransformation::DIM_DIR::ROLLP));
			Assert::AreEqual(0.0f,	b.getDist(a, HomogeneTransformation::DIM_DIR::ROLLP));

			Assert::AreEqual(0.0f,	a.getDist(b, HomogeneTransformation::DIM_DIR::ROLLM));
			Assert::AreEqual(0.0f,	b.getDist(a, HomogeneTransformation::DIM_DIR::ROLLM));

		}

		[TestMethod]
		void testDistDimPitchDiff(void)
		{

			HomogeneTransformation a;
			a.init(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

			HomogeneTransformation b;
			b.init(0.0, 1.0, 0.0, 0.0, 0.0, 0.0);

			Assert::AreEqual(1.0f,		a.getDist(b, HomogeneTransformation::DIM_DIR::PITCHP));
			Assert::AreEqual(FLT_MAX,	b.getDist(a, HomogeneTransformation::DIM_DIR::PITCHP));

			Assert::AreEqual(FLT_MAX,	a.getDist(b, HomogeneTransformation::DIM_DIR::PITCHM));
			Assert::AreEqual(1.0f,		b.getDist(a, HomogeneTransformation::DIM_DIR::PITCHM));

		}

		[TestMethod]
		void testDistDimPitchEqual(void)
		{

			HomogeneTransformation a;
			a.init(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

			HomogeneTransformation b;
			b.init(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

			Assert::AreEqual(0.0f,	a.getDist(b, HomogeneTransformation::DIM_DIR::PITCHP));
			Assert::AreEqual(0.0f,	b.getDist(a, HomogeneTransformation::DIM_DIR::PITCHP));

			Assert::AreEqual(0.0f,	a.getDist(b, HomogeneTransformation::DIM_DIR::PITCHM));
			Assert::AreEqual(0.0f,	b.getDist(a, HomogeneTransformation::DIM_DIR::PITCHM));

		}

		[TestMethod]
		void testDistDimYawDiff(void)
		{

			HomogeneTransformation a;
			a.init(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

			HomogeneTransformation b;
			b.init(0.0, 0.0, 1.0, 0.0, 0.0, 0.0);

			Assert::AreEqual(1.0f,		a.getDist(b, HomogeneTransformation::DIM_DIR::YAWP));
			Assert::AreEqual(FLT_MAX,	b.getDist(a, HomogeneTransformation::DIM_DIR::YAWP));

			Assert::AreEqual(FLT_MAX,	a.getDist(b, HomogeneTransformation::DIM_DIR::YAWM));
			Assert::AreEqual(1.0f,		b.getDist(a, HomogeneTransformation::DIM_DIR::YAWM));

		}

		[TestMethod]
		void testDistDimYawEqual(void)
		{

			HomogeneTransformation a;
			a.init(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

			HomogeneTransformation b;
			b.init(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

			Assert::AreEqual(0.0f,	a.getDist(b, HomogeneTransformation::DIM_DIR::YAWP));
			Assert::AreEqual(0.0f,	b.getDist(a, HomogeneTransformation::DIM_DIR::YAWP));

			Assert::AreEqual(0.0f,	a.getDist(b, HomogeneTransformation::DIM_DIR::YAWM));
			Assert::AreEqual(0.0f,	b.getDist(a, HomogeneTransformation::DIM_DIR::YAWM));

		}


		[TestMethod]
		void testDistComplete(void)
		{

			HomogeneTransformation a;
			a.init(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

			HomogeneTransformation b;
			b.init(0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f);

			Assert::AreEqual(0.6f,	a.getDist(b));
			Assert::AreEqual(0.6f,	b.getDist(a));

		}

		[TestMethod]
		void testTR2RPY(void)
		{
			srand (time(NULL));
			HomogeneTransformation a;
			float roll = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			float pitch = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			float yaw = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			a.init(roll, pitch, yaw, 0.0, 0.0, 0.0);

			float rpy[3];
			a.tr2rpy(rpy);
			Assert::AreEqual(roll, rpy[0], 1e-5f);
			Assert::AreEqual(pitch, rpy[1], 1e-5f);
			Assert::AreEqual(yaw, rpy[2], 1e-5f);
		}

	};
}
