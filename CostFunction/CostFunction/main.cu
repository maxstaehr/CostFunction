
#include "CostFunctionClass.h"

#include <iostream>
#include <ctime>
#include <stdlib.h>
#include <cmath>
int main(int argc, char** argv)
{

	//CostFunctionClass::testUpdateCameraParameters();
	CostFunctionClass cf;
	

	

	//cf.init_costfunction(true);
	cf.allocOptimisationMemory();
	cf.testSelectNN();
	cf.optimize_all_memory();
	//cf.optimize();
	return 0;
}


using namespace std;
double f(double x)
{
	return x = pow(x,4)+(4/3)*pow(x,3)-4*pow(x,2)+5;
}

//int main(void)
//{
//
//
//
//
//
//	double first_run, second_run, third_run;
//	time_t systime;
//	time(&systime);
//	srand((unsigned int)systime);
//	double alpha = 0.9;
//	const double e = 2.718281828;
//	double x = 10;
//	cout << "Initial state = " << x << "\t and F(x)= " << f(x) << endl;
//	double L = f(x);
//
//	for(double T = 80; T > 0.00008; T*=alpha)
//	{
//		for(int i=0; i<200; i++)
//		{
//			double xNew = x + ((rand() / (double)RAND_MAX) * 2 - 1);
//            double LNew = f(xNew);
// 
//            if (LNew < L || (rand() / (double)RAND_MAX) <= pow(e, -(LNew - L) / T))
//            {
//                L = LNew;
//                x = xNew;
//            }
//		}
//	}
//
//	cout << "Final state = " << x << "\t, total of F(x) = " << f(x) << endl << endl;
// 
// 
// 
//    cin.get();
//    return 0;
//}