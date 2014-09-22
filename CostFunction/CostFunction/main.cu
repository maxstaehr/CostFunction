
#include "CostFunctionClass.h"

#include <iostream>
#include <ctime>
#include <stdlib.h>
#include <cmath>
#include "_generate.h"
struct B{
	int pcl;
	int angle;
	double costs;
	double currProp;
	double curT;
	int ite;
	double minEnergy;
	enum STATE state;	
	double globalMin;
};
int main(int argc, char** argv)
{
	//struct B b;
	//b.pcl = 0.0;
	//std::vector<std::vector<struct B>> a;
	//a.push_back(std::vector<struct B>(0));
	//a.back().push_back(b);
	//a.back().push_back(b);
	//a.push_back(std::vector<struct B>(0));
	//a.back().push_back(b);
	//a.back().push_back(b);

	//for(unsigned int i=0; i<a.size(); i++)
	//{
	//	for(unsigned int j=0; j<a[i].size(); j++)
	//	{
	//		std::cout << a[i][j].pcl << std::endl;
	//	}

	//}

	//unsigned int long a = 999;
	//unsigned int long p = 34;
	//unsigned int long i = 34*1290+999;

	//unsigned int long p2 = i/1290;
	//unsigned int long ai = i-p2*1290;

	//unsigned long long n = 5;
	//unsigned long long k = 2;
	//unsigned long long* vector = new unsigned long long[k];

	//int gen_result = gen_comb_norep_lex_init(vector, n, k);
	//int set_counter = 0; 
	//while(gen_result == GEN_NEXT)
	// {
	//	 set_counter++;

	//	 for(int x = 0; x < k; x++)
	//	  printf("%u ", vector[x]);

	//	 printf("(%u)\n", set_counter);

	//	 gen_result = gen_comb_norep_lex_next(vector, n, k);
	// }


	//CostFunctionClass::testUpdateCameraParameters();
	CostFunctionClass cf;
	
	

	

	//cf.init_costfunction(true);
	cf.allocOptimisationMemory();
	//cf.testSelectNN();
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
//			double propNew = (rand() / (double)RAND_MAX);
//			double oldProp  = pow(e, ( L-LNew ) / T);
//			printf("new propNew: %.5f\t oldProp: %.5f\n", propNew, oldProp);
//            if (LNew < L || propNew <= oldProp)
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