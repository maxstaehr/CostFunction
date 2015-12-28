#include "stdafx.h"

using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace	Microsoft::VisualStudio::TestTools::UnitTesting;


#include "../CFObject/HomogeneTransformation.h"
namespace TestDriver
{
	[TestClass]
	public ref class HomogeneTransformationTest
	{
	public: 

		[TestMethod]
		void TestGetter()
		{
			HomogeneTransformation test;
			float temp[16];
			test.getH(temp);

			Assert::AreEqual(temp[0], 1.0f);
			Assert::AreEqual(temp[1], 0.0f);
			Assert::AreEqual(temp[2], 0.0f);
			Assert::AreEqual(temp[3], 0.0f);

			Assert::AreEqual(temp[4], 0.0f);
			Assert::AreEqual(temp[5], 1.0f);
			Assert::AreEqual(temp[6], 0.0f);
			Assert::AreEqual(temp[7], 0.0f);

			Assert::AreEqual(temp[8], 0.0f);
			Assert::AreEqual(temp[9], 0.0f);
			Assert::AreEqual(temp[10], 1.0f);
			Assert::AreEqual(temp[11], 0.0f);

			Assert::AreEqual(temp[12], 0.0f);
			Assert::AreEqual(temp[13], 0.0f);
			Assert::AreEqual(temp[14], 0.0f);
			Assert::AreEqual(temp[15], 1.0f);
		}

		[TestMethod]
		void TestSetter()
		{
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
			Assert::AreEqual(temp1[9], temp2[9]);
			Assert::AreEqual(temp1[10],temp2[10]);
			Assert::AreEqual(temp1[11],temp2[11]);

			Assert::AreEqual(temp1[12], temp2[12]);
			Assert::AreEqual(temp1[13], temp2[13]);
			Assert::AreEqual(temp1[14], temp2[14]);
			Assert::AreEqual(temp1[15], temp2[15]);
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
			float temp2[16];
			float temp3[16] = 
			{
			   1.072870643907527, 0.881846074808848, 1.389159384428151, -3.071272833251428,
			  -1.128394343681584, -0.312329315993337, 0.340627711626420, 1.454707099924857,
			   0.054700766484468, -1.616730820282032, -1.640194500065132, 4.276764847535353,
			   0.095697438969413,   2.504154114094748,  -0.241991896087627,  -2.804848622157487			
			};

			test.setH(temp1);
			HomogeneTransformation test2 = test.inv();
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
			Assert::AreEqual(temp3[9], temp2[9], 1e-3f);
			Assert::AreEqual(temp3[10],temp2[10], 1e-3f);
			Assert::AreEqual(temp3[11],temp2[11], 1e-3f);

			Assert::AreEqual(temp3[12], temp2[12], 1e-3f);
			Assert::AreEqual(temp3[13], temp2[13], 1e-3f);
			Assert::AreEqual(temp3[14], temp2[14], 1e-3f);
			Assert::AreEqual(temp3[15], temp2[15], 1e-3f);

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
			float temp2[16];
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
			Assert::AreEqual(temp3[9], temp2[9], 1e-3f);
			Assert::AreEqual(temp3[10],temp2[10], 1e-3f);
			Assert::AreEqual(temp3[11],temp2[11], 1e-3f);

			Assert::AreEqual(temp3[12], temp2[12], 1e-3f);
			Assert::AreEqual(temp3[13], temp2[13], 1e-3f);
			Assert::AreEqual(temp3[14], temp2[14], 1e-3f);
			Assert::AreEqual(temp3[15], temp2[15], 1e-3f);

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
			float temp4[16];
			

			HomogeneTransformation test1;
			test1.setH(temp1);

			HomogeneTransformation test2(test1);
			test2.setH(temp2);

			HomogeneTransformation test3 = test1.mul(test2);
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
			Assert::AreEqual(temp3[9], temp4[9], 1e-3f);
			Assert::AreEqual(temp3[10],temp4[10], 1e-3f);
			Assert::AreEqual(temp3[11],temp4[11], 1e-3f);

			Assert::AreEqual(temp3[12], temp4[12], 1e-3f);
			Assert::AreEqual(temp3[13], temp4[13], 1e-3f);
			Assert::AreEqual(temp3[14], temp4[14], 1e-3f);
			Assert::AreEqual(temp3[15], temp4[15], 1e-3f);

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
			float temp2[16];

			HomogeneTransformation test1;
			test1.init(roll, pitch, yaw, x, y, z);
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
			Assert::AreEqual(temp3[9], temp2[9], 1e-3f);
			Assert::AreEqual(temp3[10],temp2[10], 1e-3f);
			Assert::AreEqual(temp3[11],temp2[11], 1e-3f);

			Assert::AreEqual(temp3[12], temp2[12], 1e-3f);
			Assert::AreEqual(temp3[13], temp2[13], 1e-3f);
			Assert::AreEqual(temp3[14], temp2[14], 1e-3f);
			Assert::AreEqual(temp3[15], temp2[15], 1e-3f);
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
			float temp2[16];

			HomogeneTransformation test1;
			test1.init(roll, pitch, yaw, x, y, z);
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
			Assert::AreEqual(temp3[9], temp2[9], 1e-3f);
			Assert::AreEqual(temp3[10],temp2[10], 1e-3f);
			Assert::AreEqual(temp3[11],temp2[11], 1e-3f);

			Assert::AreEqual(temp3[12], temp2[12], 1e-3f);
			Assert::AreEqual(temp3[13], temp2[13], 1e-3f);
			Assert::AreEqual(temp3[14], temp2[14], 1e-3f);
			Assert::AreEqual(temp3[15], temp2[15], 1e-3f);
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
			float temp2[16];

			HomogeneTransformation test1;
			test1.init(roll, pitch, yaw, x, y, z);
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
			Assert::AreEqual(temp3[9], temp2[9], 1e-3f);
			Assert::AreEqual(temp3[10],temp2[10], 1e-3f);
			Assert::AreEqual(temp3[11],temp2[11], 1e-3f);

			Assert::AreEqual(temp3[12], temp2[12], 1e-3f);
			Assert::AreEqual(temp3[13], temp2[13], 1e-3f);
			Assert::AreEqual(temp3[14], temp2[14], 1e-3f);
			Assert::AreEqual(temp3[15], temp2[15], 1e-3f);
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
			float temp2[16];

			HomogeneTransformation test1;
			test1.init(roll, pitch, yaw, x, y, z);
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
			Assert::AreEqual(temp3[9], temp2[9], 1e-3f);
			Assert::AreEqual(temp3[10],temp2[10], 1e-3f);
			Assert::AreEqual(temp3[11],temp2[11], 1e-3f);

			Assert::AreEqual(temp3[12], temp2[12], 1e-3f);
			Assert::AreEqual(temp3[13], temp2[13], 1e-3f);
			Assert::AreEqual(temp3[14], temp2[14], 1e-3f);
			Assert::AreEqual(temp3[15], temp2[15], 1e-3f);
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

			float temp2[16];

			HomogeneTransformation test1;
			test1.init(roll, pitch, yaw, x, y, z);
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
			Assert::AreEqual(temp3[9], temp2[9], 1e-3f);
			Assert::AreEqual(temp3[10],temp2[10], 1e-3f);
			Assert::AreEqual(temp3[11],temp2[11], 1e-3f);

			Assert::AreEqual(temp3[12], temp2[12], 1e-3f);
			Assert::AreEqual(temp3[13], temp2[13], 1e-3f);
			Assert::AreEqual(temp3[14], temp2[14], 1e-3f);
			Assert::AreEqual(temp3[15], temp2[15], 1e-3f);
		}



	};
}
