
#include "CostFunctionClass.h"

int main(int argc, char** argv)
{

	//CostFunctionClass::testUpdateCameraParameters();
	CostFunctionClass cf;

	


	cf.allocOptimisationMemory();
	cf.optimize_all_memory();
	//cf.optimize();
	return 0;
}
