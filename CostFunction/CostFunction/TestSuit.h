
#ifndef TESTSUIT_H_
#define TESTSUIT_H_

#include "struct_definitions.h"

class TestSuit{

	public:
		TestSuit();
		virtual ~TestSuit();
		static void testCudaFunctions();
		static void testCameraParameters(struct CAM* cam);
};

#endif