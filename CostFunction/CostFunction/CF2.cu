#include "CF2.h"

#include "CudaMem.h"


#include <cuda_profiler_api.h>
#include "device_launch_parameters.h"
#include "cuda_runtime.h"
#include "mathcuda.h"
#include "NearestNeighbour.h"


#include <cstdio>
#include <string.h>
#include <math.h>
#include <time.h>
#include <fstream>
#include <iostream> 
#include <assert.h>
#include "IO.h"

#define DOF_ROBOT_Q 9
#define MAX_DEPTH 7.0f
#define THREAD_SIZE 24
#define MAX_EQUAL_SOLUTION 100
#define MIN_PERCENT_OF_ELEMENT 0.5
#define MAX_NUMER_OF_CANDITATE 100


#define MATH_PI 3.14159265359f
#define EYE {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f}


//#include "global.h"
#include "allKernelFct.cuh"
CF2::CF2()
{

}