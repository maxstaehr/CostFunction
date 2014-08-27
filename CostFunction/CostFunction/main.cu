
#include "CostFunctionClass.h"

int main(int argc, char** argv)
{

	CostFunctionClass cf;


	cf.allocOptimisationMemory();
	cf.optimize_all_memory();
	//cf.optimize();
	return 0;
}
