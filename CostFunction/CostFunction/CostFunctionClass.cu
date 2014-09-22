/*
 * CostFunctionClass.cpp
 *
 *  Created on: 13.06.2014
 *      Author: tsdf
 */

#include "CostFunctionClass.h"
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

#define TESTING



#define MATH_PI 3.14159265359f
#define EYE {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f}


//#include "global.h"
#include "allKernelFct.cuh"


CostFunctionClass::CostFunctionClass():cudaDevice(0), nx(64), ny(64), nz(32),N(nx*ny*nz), N_char(N/8), N_int(N_char/4),
		xmin(-3.0f), xmax(3.0f), ymin(-3.0f), ymax(3.0f), zmin(0.0f),zmax(3.0),nOfCams(1)
{




	opt_data.h_pcl_index = NULL;
	opt_data.h_angle_index = NULL;
	comp_vec = NULL;

	cudaStatus = cudaDeviceReset();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "could not reset cuda device");

	}

	//*********************************************
	//init of the robot cloud at q = [0...0] for camer pos
	init_mmkr16();
	init_mmkr16_q0();

	float h[] = EYE;
	CudaMem::cudaMemAllocReport((void**)&d_eye, NUMELEM_H*sizeof(float));
	CudaMem::cudaMemCpyReport(d_eye,h, NUMELEM_H*sizeof(float), cudaMemcpyHostToDevice);



	IO::loadPCL(&robotpcl_0, "robotpcl_q0.bin");
	CudaMem::copyPCLHostToDevice(&robotpcl_0);

	IO::loadPCL(&humanpcl_0, "humanpcl_q0.bin");
	CudaMem::copyPCLHostToDevice(&humanpcl_0);

	float qv[9];
	for(int i=0;i<9; i++)
		qv[i] = 0.0f;
	adjustRobotTransformation(qv);
	robotPCL_qa0.n = robotpcl_0.n;
	CudaMem::allocPCLN(&robotPCL_qa0);
	CudaMem::cpyPCL(&robotPCL_qa0, &robotpcl_0);
	CudaMem::copyPCLHostToDevice(&robotPCL_qa0);
	adjustrobotpcl(&robotPCL_qa0);
	CudaMem::copyPCLDeviceToHost(&robotPCL_qa0);

	robotpcl.n = robotpcl_0.n;
	CudaMem::allocPCLN(&robotpcl);

	humanpcl.n = humanpcl_0.n;
	CudaMem::allocPCLN(&humanpcl);



	IO::loadRobotPos(&robotPos, N);
	IO::loadHumanPos(&humanPos);


#ifdef DEBUG_POSITIONS
	robotPos.n = 1;
	humanPos.n = 1;
	humanPos.positions[0] = -2.5f;
	humanPos.positions[1] = 0.0f;
	humanPos.positions[2] = MATH_PI/2.0f;
	//humanPos.positions[3] = -2.5f;
	//humanPos.positions[4] = 1.0f;
	//humanPos.positions[5] = MATH_PI/2.0f;
	for(int i=0;i<9;i++)
	{
		robotPos.positions[i] = 0.0f;
		robotPos.velocities[i] = 0.0f;
	}
#endif
	initCameraPositions(&pos);
	initCam(&cam);

	isValidPosition = new bool[robotPos.n * humanPos.n];
	CudaMem::allocRobotPositions(&robotPos);
	CudaMem::allocHumanPositions(&humanPos);


	initDHTransformations();
	initHTransformations();

	initWSCoordinates();


	cudaFuncSetCacheConfig(cuda_calc::raytrace_shared, cudaFuncCachePreferShared);
	cudaFuncSetCacheConfig(cuda_calc::project_voxel_into_ws, cudaFuncCachePreferShared);
	cudaFuncSetCacheConfig(cuda_calc::fuseGrids_shared, cudaFuncCachePreferShared);

	sA = new SimulatedAnnealing(MAX_ITE, 2500, 0.995, robotPCL_qa0.n, pos.nOfAngles);

}

CostFunctionClass::~CostFunctionClass() {
	// TODO Auto-generated destructor stub
}

bool CostFunctionClass::doesHumanCollideWithRobot(void)
{
	cuda_calc::checkCollisionEnvironmentHuman<<<1,1024>>>(opt_data.allData.d_humanoccupancy, opt_data.allData.d_robotoccupancy, opt_data.d_hasCollision);
	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "addKernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching addKernel!\n", cudaStatus);

	}

	unsigned int ret;
	CudaMem::cudaMemCpyReport(&ret, opt_data.d_hasCollision, sizeof(int), cudaMemcpyDeviceToHost);
	if (ret > 0)
	{
		return false;
	}else
	{
		return true;
	}
}

void CostFunctionClass::initDHTransformations(void)
{

	CudaMem::allocDHTransformations(&opt_data.s_mmkr16_q0, 1);
	CudaMem::cudaMemCpyReport(opt_data.s_mmkr16_q0.d_T, s_mmkr16_q0.T, NUMELEM_H * (N_ELEMENT_T) * sizeof(float), cudaMemcpyHostToDevice);


	CudaMem::allocDHTransformations(&opt_data.s_mmkr16, robotPos.n);
	for(unsigned int r=0; r<robotPos.n; r++)
	{
		adjustRobotTransformation(&robotPos.positions[9*r]);
		calculateVelocity(&(robotPos.velocities[9*r]));
		int offset_t = r*NUMELEM_H * (N_ELEMENT_T);
		int offset_v = r*(N_ELEMENT_T);
		CudaMem::cudaMemCpyReport(opt_data.s_mmkr16.d_T+offset_t, s_mmkr16.T, NUMELEM_H * (N_ELEMENT_T) * sizeof(float), cudaMemcpyHostToDevice);
		CudaMem::cudaMemCpyReport(opt_data.s_mmkr16.d_v_x+offset_v,s_mmkr16.v_x,(N_ELEMENT_T) * sizeof(float), cudaMemcpyHostToDevice);
		CudaMem::cudaMemCpyReport(opt_data.s_mmkr16.d_v_y+offset_v,s_mmkr16.v_y,(N_ELEMENT_T) * sizeof(float), cudaMemcpyHostToDevice);
		CudaMem::cudaMemCpyReport(opt_data.s_mmkr16.d_v_z+offset_v,s_mmkr16.v_z,(N_ELEMENT_T) * sizeof(float), cudaMemcpyHostToDevice);
		CudaMem::cudaMemCpyReport(opt_data.s_mmkr16.d_w_x+offset_v,s_mmkr16.w_x,(N_ELEMENT_T) * sizeof(float), cudaMemcpyHostToDevice);
		CudaMem::cudaMemCpyReport(opt_data.s_mmkr16.d_w_y+offset_v,s_mmkr16.w_y,(N_ELEMENT_T) * sizeof(float), cudaMemcpyHostToDevice);
		CudaMem::cudaMemCpyReport(opt_data.s_mmkr16.d_w_z+offset_v,s_mmkr16.w_z,(N_ELEMENT_T) * sizeof(float), cudaMemcpyHostToDevice);
	}
	//CudaMem

}

void CostFunctionClass::initHTransformations(void)
{
	CudaMem::allocHTransformations(&opt_data.s_human, robotPos.n*humanPos.n);
	float H_temp[16];
	for(unsigned int r=0; r<robotPos.n; r++)
	{
		for(unsigned int h=0; h<humanPos.n; h++)
		{
			transformHumanIntoRobotSystem(robotPos.positions+9*r, humanPos.positions+3*h, H_temp);
			if( abs(H_temp[3]) - WS_BORDER > WS_X_MAX || abs(H_temp[7]) - WS_BORDER > WS_Y_MAX)
			{
				isValidPosition[r*humanPos.n+h] = false;
			}else
			{
				isValidPosition[r*humanPos.n+h] = true;
			}
			int offset = (r*humanPos.n+h)*NUMELEM_H;
			CudaMem::cudaMemCpyReport(opt_data.s_human.d_h+offset, H_temp, NUMELEM_H*sizeof(float), cudaMemcpyHostToDevice);

		}
	}

}

void CostFunctionClass::calcNextCandidates(void)
{

}


void CostFunctionClass::init_mmkr16(void)
{

	s_mmkr16.theta = new float[DOF_ROBOT_Q];
	s_mmkr16.d = new float[DOF_ROBOT_Q];
	s_mmkr16.a = new float[DOF_ROBOT_Q];
	s_mmkr16.alpha = new float[DOF_ROBOT_Q];
	s_mmkr16.sigma = new float[DOF_ROBOT_Q];
	s_mmkr16.offset = new float[DOF_ROBOT_Q];
	s_mmkr16.base = new float[NUMELEM_H];

	s_mmkr16.T = new float[NUMELEM_H * (N_ELEMENT_T)];
	s_mmkr16.v_x = new float[N_ELEMENT_T];
	s_mmkr16.v_y = new float[N_ELEMENT_T];
	s_mmkr16.v_z = new float[N_ELEMENT_T];
	s_mmkr16.w_x = new float[N_ELEMENT_T];
	s_mmkr16.w_y = new float[N_ELEMENT_T];
	s_mmkr16.w_z = new float[N_ELEMENT_T];


	CudaMem::cudaMemAllocReport((void**)&s_mmkr16.d_T, NUMELEM_H * (N_ELEMENT_T) * sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&s_mmkr16.d_v_x, (N_ELEMENT_T) * sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&s_mmkr16.d_v_y, (N_ELEMENT_T) * sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&s_mmkr16.d_v_z, (N_ELEMENT_T) * sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&s_mmkr16.d_w_x, (N_ELEMENT_T) * sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&s_mmkr16.d_w_y, (N_ELEMENT_T) * sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&s_mmkr16.d_w_z, (N_ELEMENT_T) * sizeof(float));


	s_mmkr16.theta[0] = -MATH_PI/2.0;
	s_mmkr16.theta[1] = MATH_PI/2.0;
	s_mmkr16.theta[2] = 0.0;
	s_mmkr16.theta[3] = 0.0;
	s_mmkr16.theta[4] = 0.0;
	s_mmkr16.theta[5] = 0.0;
	s_mmkr16.theta[6] = 0.0;
	s_mmkr16.theta[7] = 0.0;
	s_mmkr16.theta[8] = 0.0;

	s_mmkr16.d[0] = 0.0;
	s_mmkr16.d[1] = 0.0;
	s_mmkr16.d[2] = 0.870f;
	s_mmkr16.d[3] = -0.235f;
	s_mmkr16.d[4] = 0.0;
	s_mmkr16.d[5] = 0.0;
	s_mmkr16.d[6] =  -0.670f;
	s_mmkr16.d[7] = 0.0;
	s_mmkr16.d[8] = -0.158f;

	s_mmkr16.a[0] = 0.0;
	s_mmkr16.a[1] = 0.0;
	s_mmkr16.a[2] = 0.0;
	s_mmkr16.a[3] = 0.450f;
	s_mmkr16.a[4] = 0.680f;
	s_mmkr16.a[5] = -0.035f;
	s_mmkr16.a[6] = 0.0;
	s_mmkr16.a[7] = 0.0;
	s_mmkr16.a[8] = 0.0;

	s_mmkr16.alpha[0] = -MATH_PI/2.0;;
	s_mmkr16.alpha[1] = MATH_PI/2.0;;
	s_mmkr16.alpha[2] = MATH_PI;
	s_mmkr16.alpha[3] = MATH_PI/2.0;
	s_mmkr16.alpha[4] = 0.0;
	s_mmkr16.alpha[5] = MATH_PI/2.0;
	s_mmkr16.alpha[6] = -MATH_PI/2.0;
	s_mmkr16.alpha[7] = MATH_PI/2.0;
	s_mmkr16.alpha[8] = MATH_PI;


	s_mmkr16.sigma[0] = 1.0;
	s_mmkr16.sigma[1] = 1.0;
	s_mmkr16.sigma[2] = 0.0;
	s_mmkr16.sigma[3] = 0.0;
	s_mmkr16.sigma[4] = 0.0;
	s_mmkr16.sigma[5] = 0.0;
	s_mmkr16.sigma[6] = 0.0;
	s_mmkr16.sigma[7] = 0.0;
	s_mmkr16.sigma[8] = 0.0;

	s_mmkr16.offset[0] = 0.0;
	s_mmkr16.offset[1] = 0.0;
	s_mmkr16.offset[2] = MATH_PI/2.0;
	s_mmkr16.offset[3] = 0.0;
	s_mmkr16.offset[4] = -MATH_PI/2.0;
	s_mmkr16.offset[5] = 0.0;
	s_mmkr16.offset[6] = 0.0;
	s_mmkr16.offset[7] = 0.0;
	s_mmkr16.offset[8] = 0.0;

	s_mmkr16.base[0] = 1.0;
	s_mmkr16.base[1] = 0.0;
	s_mmkr16.base[2] = 0.0;
	s_mmkr16.base[3] = 0.0;

	s_mmkr16.base[4] = 0.0;
	s_mmkr16.base[5] = 0.0;
	s_mmkr16.base[6] = 1.0;
	s_mmkr16.base[7] = 0.0;

	s_mmkr16.base[8] = 0.0;
	s_mmkr16.base[9] = -1.0;
	s_mmkr16.base[10] = 0.0;
	s_mmkr16.base[11] = 0.0;

	s_mmkr16.base[12] = 0.0;
	s_mmkr16.base[13] = 0.0;
	s_mmkr16.base[14] = 0.0;
	s_mmkr16.base[15] = 1.0;
}

void CostFunctionClass::init_mmkr16_q0(void)
{

	s_mmkr16_q0.theta = new float[DOF_ROBOT_Q];
	s_mmkr16_q0.d = new float[DOF_ROBOT_Q];
	s_mmkr16_q0.a = new float[DOF_ROBOT_Q];
	s_mmkr16_q0.alpha = new float[DOF_ROBOT_Q];
	s_mmkr16_q0.sigma = new float[DOF_ROBOT_Q];
	s_mmkr16_q0.offset = new float[DOF_ROBOT_Q];
	s_mmkr16_q0.base = new float[NUMELEM_H];
	s_mmkr16_q0.T = new float[NUMELEM_H * (N_ELEMENT_T)];

	s_mmkr16_q0.v_x = new float[N_ELEMENT_T];
	s_mmkr16_q0.v_y = new float[N_ELEMENT_T];
	s_mmkr16_q0.v_z = new float[N_ELEMENT_T];
	s_mmkr16_q0.w_x = new float[N_ELEMENT_T];
	s_mmkr16_q0.w_y = new float[N_ELEMENT_T];
	s_mmkr16_q0.w_z = new float[N_ELEMENT_T];



	CudaMem::cudaMemAllocReport((void**)&s_mmkr16_q0.d_T, NUMELEM_H * (N_ELEMENT_T) * sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&s_mmkr16_q0.d_v_x, (N_ELEMENT_T) * sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&s_mmkr16_q0.d_v_y, (N_ELEMENT_T) * sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&s_mmkr16_q0.d_v_z, (N_ELEMENT_T) * sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&s_mmkr16_q0.d_w_x, (N_ELEMENT_T) * sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&s_mmkr16_q0.d_w_y, (N_ELEMENT_T) * sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&s_mmkr16_q0.d_w_z, (N_ELEMENT_T) * sizeof(float));







	s_mmkr16_q0.theta[0] = -MATH_PI/2.0;
	s_mmkr16_q0.theta[1] = MATH_PI/2.0;
	s_mmkr16_q0.theta[2] = 0.0;
	s_mmkr16_q0.theta[3] = 0.0;
	s_mmkr16_q0.theta[4] = 0.0;
	s_mmkr16_q0.theta[5] = 0.0;
	s_mmkr16_q0.theta[6] = 0.0;
	s_mmkr16_q0.theta[7] = 0.0;
	s_mmkr16_q0.theta[8] = 0.0;

	s_mmkr16_q0.d[0] = 0.0;
	s_mmkr16_q0.d[1] = 0.0;
	s_mmkr16_q0.d[2] = 0.870f;
	s_mmkr16_q0.d[3] = -0.235f;
	s_mmkr16_q0.d[4] = 0.0;
	s_mmkr16_q0.d[5] = 0.0;
	s_mmkr16_q0.d[6] =  -0.670f;
	s_mmkr16_q0.d[7] = 0.0;
	s_mmkr16_q0.d[8] = -0.158f;

	s_mmkr16_q0.a[0] = 0.0;
	s_mmkr16_q0.a[1] = 0.0;
	s_mmkr16_q0.a[2] = 0.0;
	s_mmkr16_q0.a[3] = 0.450f;
	s_mmkr16_q0.a[4] = 0.680f;
	s_mmkr16_q0.a[5] = -0.035f;
	s_mmkr16_q0.a[6] = 0.0;
	s_mmkr16_q0.a[7] = 0.0;
	s_mmkr16_q0.a[8] = 0.0;

	s_mmkr16_q0.alpha[0] = -MATH_PI/2.0;;
	s_mmkr16_q0.alpha[1] = MATH_PI/2.0;;
	s_mmkr16_q0.alpha[2] = MATH_PI;
	s_mmkr16_q0.alpha[3] = MATH_PI/2.0;
	s_mmkr16_q0.alpha[4] = 0.0;
	s_mmkr16_q0.alpha[5] = MATH_PI/2.0;
	s_mmkr16_q0.alpha[6] = -MATH_PI/2.0;
	s_mmkr16_q0.alpha[7] = MATH_PI/2.0;
	s_mmkr16_q0.alpha[8] = MATH_PI;


	s_mmkr16_q0.sigma[0] = 1.0;
	s_mmkr16_q0.sigma[1] = 1.0;
	s_mmkr16_q0.sigma[2] = 0.0;
	s_mmkr16_q0.sigma[3] = 0.0;
	s_mmkr16_q0.sigma[4] = 0.0;
	s_mmkr16_q0.sigma[5] = 0.0;
	s_mmkr16_q0.sigma[6] = 0.0;
	s_mmkr16_q0.sigma[7] = 0.0;
	s_mmkr16_q0.sigma[8] = 0.0;

	s_mmkr16_q0.offset[0] = 0.0;
	s_mmkr16_q0.offset[1] = 0.0;
	s_mmkr16_q0.offset[2] = MATH_PI/2.0;
	s_mmkr16_q0.offset[3] = 0.0;
	s_mmkr16_q0.offset[4] = -MATH_PI/2.0;
	s_mmkr16_q0.offset[5] = 0.0;
	s_mmkr16_q0.offset[6] = 0.0;
	s_mmkr16_q0.offset[7] = 0.0;
	s_mmkr16_q0.offset[8] = 0.0;

	s_mmkr16_q0.base[0] = 1.0;
	s_mmkr16_q0.base[1] = 0.0;
	s_mmkr16_q0.base[2] = 0.0;
	s_mmkr16_q0.base[3] = 0.0;

	s_mmkr16_q0.base[4] = 0.0;
	s_mmkr16_q0.base[5] = 0.0;
	s_mmkr16_q0.base[6] = 1.0;
	s_mmkr16_q0.base[7] = 0.0;

	s_mmkr16_q0.base[8] = 0.0;
	s_mmkr16_q0.base[9] = -1.0;
	s_mmkr16_q0.base[10] = 0.0;
	s_mmkr16_q0.base[11] = 0.0;

	s_mmkr16_q0.base[12] = 0.0;
	s_mmkr16_q0.base[13] = 0.0;
	s_mmkr16_q0.base[14] = 0.0;
	s_mmkr16_q0.base[15] = 1.0;


	float qv[] = {0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	float* T1 = new float[16];
	float* T2 = new float[16];
	float* T3 = new float[16];

	memcpy(s_mmkr16_q0.T, s_mmkr16_q0.base, NUMELEM_H*sizeof(float));
	memcpy(T1, s_mmkr16_q0.base, NUMELEM_H*sizeof(float));
	float sa, ca, q, st, ct,d;
	for(unsigned int j=0; j<DOF_ROBOT_Q; j++){

		sa = sin(s_mmkr16_q0.alpha[j]);
		ca = cos(s_mmkr16_q0.alpha[j]);
		q = qv[j] + s_mmkr16_q0.offset[j];
		if (s_mmkr16_q0.sigma[j] == 0.0){
			st = sin(q);
			ct = cos(q);
			d = s_mmkr16_q0.d[j];
		}
		else{
			st = sin(s_mmkr16_q0.theta[j]);
			ct = cos(s_mmkr16_q0.theta[j]);
			d = q;
		}
		T2[0] = ct;
		T2[1] = -st*ca;
		T2[2] = st*sa;
		T2[3] = s_mmkr16_q0.a[j]*ct;

		T2[4] = st;
		T2[5] = ct*ca;
		T2[6] = -ct*sa;
		T2[7] = s_mmkr16_q0.a[j]*st;

		T2[8] = 0;
		T2[9] = sa;
		T2[10] = ca;
		T2[11] = d;

		T2[12] = 0;
		T2[13] = 0;
		T2[14] = 0;
		T2[15] = 1;

		mm16(T1,T2,T3);
		memcpy(&s_mmkr16_q0.T[(j+1)*NUMELEM_H], T3,NUMELEM_H*sizeof(float));
		memcpy(T1,T3,NUMELEM_H * sizeof(float));

	}


}

void CostFunctionClass::initCam(struct CAM* cam)
{
	cam->f =  0.0085f;
	cam->pp[0] = 25.0f;
	cam->pp[1] = 32.0f;
	cam->rho[0] = 0.0001f;
	cam->rho[1] = 0.0001f;

	cam->nu = 50;
	cam->nv = 64;

	cam->C0[0] = cam->f/cam->rho[0];
	cam->C0[1] = 0.0f;
	cam->C0[2] = cam->pp[0];
	cam->C0[3] = 0.0f;

	cam->C0[4] = 0.0f;
	cam->C0[5] = cam->f/cam->rho[1];
	cam->C0[6] = cam->pp[1];
	cam->C0[7] = 0.0f;

	cam->C0[8] = 0.0f;
	cam->C0[9] = 0.0f;
	cam->C0[10] = 1.0;
	cam->C0[11] = 0.0f;

	CudaMem::cudaMemAllocReport((void**)&cam->d_C0, NUMELEM_C*sizeof(float));
	CudaMem::cudaMemCpyReport(cam->d_C0, cam->C0, NUMELEM_C*sizeof(float), cudaMemcpyHostToDevice);



}

void CostFunctionClass::init_costfunction(bool savetofile)
{

	float h_camera[16];
	float h[16];


	unsigned int CostFunctionClassrobot_human_index;
	time_t start;





	CudaMem::cpyPCL(&robotpcl, &robotpcl_0);
	CudaMem::copyPCLHostToDevice(&robotpcl);




	CudaMem::cpyPCL(&humanpcl, &humanpcl_0);
	CudaMem::copyPCLHostToDevice(&humanpcl);








	unsigned int nofHumanPos = robotPos.n * humanPos.n;
	unsigned int robot_human_index = 0;
	unsigned int nOfCameraPos = robotpcl.n * pos.nOfAngles* robotPos.n;

	//******************************************************************
	//Alloc memory
	//
	unsigned int** robotOccupancyGrids = new unsigned int*[robotPos.n];
	unsigned int** humanOccupancyGridsH = new unsigned int*[nofHumanPos];
	float* C_all = new float[nOfCameraPos*12];
	float* Mi_all = new float[nOfCameraPos*9];
	float* CP_all = new float[nOfCameraPos*3];
	float* H_all = new float[nOfCameraPos*16];
	//************kernel*******************************************************

	cudaError cudaStatus;
	time(&start);
	for(unsigned int robotPosIndex = 0; robotPosIndex < robotPos.n; robotPosIndex++)
	{

		adjustRobotTransformation(&robotPos.positions[9*robotPosIndex]);
		CudaMem::cpyPCL(&robotpcl, &robotpcl_0);
		CudaMem::copyPCLHostToDevice(&robotpcl);
		adjustrobotpclandvelocity(&robotpcl,robotPos.positions[9*robotPosIndex], robotPos.positions[9*robotPosIndex+1]);
		CudaMem::copyPCLDeviceToHost(&robotpcl);




		//saving intermediate results
		CudaMem::cudaMemAllocReport((void**)&robotOccupancyGrids[robotPosIndex], N_char * sizeof(unsigned char));
		assignRobotPclIntoWS_shared(&robotpcl,robotOccupancyGrids[robotPosIndex]);

		for(unsigned int humanPosIndex  = 0; humanPosIndex < humanPos.n; humanPosIndex++)
		{
			robot_human_index = robotPosIndex * humanPos.n + humanPosIndex;
			CudaMem::cpyPCL(&humanpcl, &humanpcl_0);
			CudaMem::copyPCLHostToDevice(&humanpcl);
			transformHumanIntoRobotSystem(&robotPos.positions[9*robotPosIndex], &humanPos.positions[3*humanPosIndex], h);

			CudaMem::cudaMemAllocReport((void**)&humanOccupancyGridsH[robot_human_index], N_char * sizeof(unsigned char));
			assignHumanPclIntoWS_shared(&humanpcl, humanOccupancyGridsH[robot_human_index], h);

		}
		printProgress((double)robotPosIndex,(double)robotPos.n, start, "init grids");
	}


	time(&start);
	for(unsigned int robotPointIndex = 0; robotPointIndex < robotpcl.n; robotPointIndex++)
	{
		float roll, pitch, yaw, *current_C, *current_MI, *current_CP;

		unsigned int C_pitch = 12;
		unsigned int MI_pitch = 9;
		unsigned int CP_pitch = 3;
		unsigned int H_pitch = 16;


		for(unsigned int angleIndex = 0; angleIndex < pos.nOfAngles; angleIndex++)
		{

			roll = pos.roll[angleIndex];
			pitch = pos.pitch[angleIndex];
			yaw = pos.yaw[angleIndex];

			unsigned int angleOffSet = robotPointIndex*pos.nOfAngles+angleIndex;

			for(unsigned int robotPosIndex = 0; robotPosIndex < robotPos.n; robotPosIndex++)
			{
				adjustRobotTransformation(&robotPos.positions[9*robotPosIndex]);
				getCameraPosition(robotPointIndex, roll, pitch, yaw, &robotpcl_0, h_camera);
				memcpy(&H_all[(angleOffSet*robotPos.n +robotPosIndex)*H_pitch], h_camera,  16*sizeof(float));
				updateCamera(&cam, h_camera);

				current_C = &C_all[(angleOffSet*robotPos.n +robotPosIndex)*C_pitch];
				current_MI = &Mi_all[(angleOffSet*robotPos.n +robotPosIndex)*MI_pitch];
				current_CP = &CP_all[(angleOffSet*robotPos.n +robotPosIndex)*CP_pitch];
				memcpy(current_C, cam.C, sizeof(float) * C_pitch);
				memcpy(current_MI, cam.Mi, sizeof(float) * MI_pitch);
				current_CP[0] = cam.x;
				current_CP[1] = cam.y;
				current_CP[2] = cam.z;


			}
		}
		printProgress((double)robotPointIndex,(double) robotpcl.n, start, "calc camera pos");
	}

	using namespace std;
	if(savetofile)
	{
		//saving C
		std::string filename = "C.bin";
		ofstream outbin( filename.c_str(), ofstream::binary );
		outbin.write((char*)C_all, nOfCameraPos*12*sizeof(float));
		outbin.close();
		delete C_all;

		//saving Mi
		filename = "Mi.bin";
		outbin.open( filename.c_str(), ofstream::binary );
		outbin.write((char*)Mi_all, nOfCameraPos*9*sizeof(float));
		outbin.close();
		delete Mi_all;

		//saving Mi
		filename = "CP.bin";
		outbin.open( filename.c_str(), ofstream::binary );
		outbin.write((char*)CP_all, nOfCameraPos*3*sizeof(float));
		outbin.close();
		delete CP_all;

		//saving H
		filename = "H.bin";
		outbin.open( filename.c_str(), ofstream::binary );
		outbin.write((char*)H_all, nOfCameraPos*16*sizeof(float));
		outbin.close();
		delete H_all;

		//saving robot occupancy
		char *buffer = new char[robotPos.n*N_char];
		for(unsigned int i=0; i<robotPos.n; i++)
		{
			CudaMem::cudaMemCpyReport(&buffer[N_char*i], robotOccupancyGrids[i], N_char*sizeof(unsigned char), cudaMemcpyDeviceToHost);
		}
		filename = "robotOccupancy.bin";
		outbin.open( filename.c_str(), ofstream::binary );
		outbin.write(buffer, robotPos.n*N_char*sizeof(unsigned char));
		outbin.close();
		delete buffer;

		//saving human occupancy
		buffer = new char[robotPos.n * humanPos.n * N_char];
		for(unsigned int rp =0; rp<robotPos.n; rp++)
		{
			for(unsigned int hp = 0; hp<humanPos.n; hp++)
			{
				unsigned int robot_human_index = rp * humanPos.n + hp;
				CudaMem::cudaMemCpyReport(&buffer[N_char*robot_human_index], humanOccupancyGridsH[robot_human_index], N_char*sizeof(unsigned char), cudaMemcpyDeviceToHost);
			}
		}
		filename = "humanOccupancy.bin";
		outbin.open( filename.c_str(), ofstream::binary );
		outbin.write(buffer, robotPos.n*humanPos.n*N_char*sizeof(unsigned char));
		outbin.close();
		delete buffer;

	}

	//******************************************************************
	//Delete memory
	//

	for(int i=0; i<robotPos.n; i++)
		cudaFree(robotOccupancyGrids[i]);
	delete[] robotOccupancyGrids;

	for(int i=0; i<nofHumanPos; i++)
		cudaFree(humanOccupancyGridsH[i]);
	delete [] humanOccupancyGridsH;


	CudaMem::deletePCL(&humanpcl);
	CudaMem::deletePCL(&robotpcl);
	//*******************************************************************
}

void CostFunctionClass::setRobotOccupancyGrid(int posIndex)
{
	CudaMem::copyPCLDeviceToDevice(&robotpcl_0, &robotpcl);
	adjustrobotpclandvelocity_memory(&robotpcl,posIndex, robotPos.positions[9*posIndex], robotPos.positions[9*posIndex+1]);
	assignRobotPclIntoWS_shared(&robotpcl,opt_data.allData.d_robotoccupancy);
}

void CostFunctionClass::setHumanOccupancyGrid(int humanPosIndex, int robotPosIndex)
{
	CudaMem::copyPCLDeviceToDevice(&humanpcl_0, &humanpcl);
	int index = robotPosIndex*humanPos.n+humanPosIndex;
	assignHumanPclIntoWS_memory(&humanpcl, opt_data.allData.d_humanoccupancy, index);
}

void CostFunctionClass::adjustCameraParameters(int index, int i)
{
	int offset_T = index*NUMELEM_H*N_ELEMENT_T;
	int offset_H = index*NUMELEM_H;
	cuda_calc::updateCameraPositions<<<PAR_KERNEL_LAUNCHS,CAM_ITE>>>(
			opt_data.d_pcl_index+i*MAX_ITE,
			opt_data.d_angle_index+i*MAX_ITE,
			robotpcl.d_x,
			robotpcl.d_y,
			robotpcl.d_z,
			robotpcl.d_i,
			opt_data.s_mmkr16_q0.d_T,
			opt_data.s_mmkr16.d_T+offset_T,
			opt_data.d_h_camera);

	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "project_voxel_into_ws launch failed: %s\n", cudaGetErrorString(cudaStatus));
		return;
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching updateCameraPositions!\n", cudaStatus);
		return;
	}

	cuda_calc::updateCameraParameters<<<PAR_KERNEL_LAUNCHS,CAM_ITE>>>(
			cam.d_C0,
			opt_data.d_h_camera,
			opt_data.allData.d_c,
			opt_data.allData.d_cp,
			opt_data.allData.d_mi);

	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "updateCameraParameters launch failed: %s\n", cudaGetErrorString(cudaStatus));
		return;
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching updateCameraParameters!\n", cudaStatus);
		return;
	}

}

void CostFunctionClass::assignNewCamera(void)
{
	//deleting old memory if exists
	if(opt_data.h_pcl_index != NULL)
		delete opt_data.h_pcl_index;
	opt_data.h_pcl_index = new int [MAX_ITE*nOfCams];			

	if (opt_data.h_angle_index != NULL)
		delete opt_data.h_angle_index;
	opt_data.h_angle_index =	new int [MAX_ITE*nOfCams];

	if(comp_vec != NULL)
		delete comp_vec;
	comp_vec = new unsigned long long[nOfCams];
	gen_result = gen_comb_norep_lex_init(comp_vec, robotPCL_qa0.n*pos.nOfAngles, nOfCams);
	opt_data.maxIteration = maxNumberOfIteration(robotPCL_qa0.n*pos.nOfAngles, nOfCams);

	
}

unsigned long long CostFunctionClass::maxNumberOfIteration(unsigned int long long n, unsigned int long long k)
{
	if(k>1)
	{
		unsigned long long nf = exp(n*log((double)n)-n+0.5*log(2*MATH_PI*n));
		unsigned long long kf = exp(k*log((double)k)-k+0.5*log(2*MATH_PI*k));
		unsigned long long nkf = exp((n-k)*log((double)(n-k))-(n-k)+0.5*log(2*MATH_PI*(n-k)));
		return nf/(kf*nkf);
	}else
	{
		return n;
	}
}

void CostFunctionClass::allocOptimisationMemory(void)
{
	unsigned int nOfCameraPos = robotpcl_0.n * pos.nOfAngles;
	unsigned int human_robot_pos = humanPos.n * robotPos.n;
	unsigned int depthBufferSize = IMG_SIZE *CAM_ITE;

	cudaError		cudaStatus;
//	cudaStatus = cudaDeviceReset();
//	if (cudaStatus != cudaSuccess) {
//		fprintf(stderr, "could not reset cuda device");
//
//	}
	


	CudaMem::cudaMemAllocReport((void**)&opt_data.allData.d_humanoccupancy, 	N_int*sizeof(int));
	CudaMem::cudaMemAllocReport((void**)&opt_data.allData.d_robotoccupancy, 	N_int*sizeof(int));
	CudaMem::cudaMemAllocReport((void**)&opt_data.allData.d_ksdf, 				N*sizeof(float));

	CudaMem::cudaMemAllocReport((void**)&opt_data.allData.d_hs, 				PAR_KERNEL_LAUNCHS*CAM_ITE*N_int*sizeof(int));
	CudaMem::cudaMemAllocReport((void**)&opt_data.allData.d_fa, 				PAR_KERNEL_LAUNCHS*CAM_ITE*N_int*sizeof(int));

	CudaMem::cudaMemAllocReport((void**)&opt_data.allData.d_hs_final, 				PAR_KERNEL_LAUNCHS*CAM_ITE*N_int*sizeof(int));
	CudaMem::cudaMemAllocReport((void**)&opt_data.allData.d_fa_final, 				PAR_KERNEL_LAUNCHS*CAM_ITE*N_int*sizeof(int));



	CudaMem::cudaMemAllocReport((void**)&opt_data.allData.d_fa_costs, 		PAR_KERNEL_LAUNCHS*CAM_ITE*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&opt_data.allData.d_hs_costs, 		PAR_KERNEL_LAUNCHS*CAM_ITE*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&opt_data.allData.d_costs, 			PAR_KERNEL_LAUNCHS*CAM_ITE*sizeof(double));
	CudaMem::cudaMemAllocReport((void**)&opt_data.allData.d_hdm, 			PAR_KERNEL_LAUNCHS*depthBufferSize*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&opt_data.allData.d_edm, 			PAR_KERNEL_LAUNCHS*depthBufferSize*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&opt_data.allData.d_cp, 				PAR_KERNEL_LAUNCHS*CAM_ITE*NUMELEM_Cp*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&opt_data.allData.d_mi, 				PAR_KERNEL_LAUNCHS*CAM_ITE*NUMELEM_Mi*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&opt_data.allData.d_c, 				PAR_KERNEL_LAUNCHS*CAM_ITE*NUMELEM_C*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&opt_data.d_pcl_index, 				PAR_KERNEL_LAUNCHS*CAM_ITE*sizeof(int));
	CudaMem::cudaMemAllocReport((void**)&opt_data.d_angle_index, 			PAR_KERNEL_LAUNCHS*CAM_ITE*sizeof(int));
	CudaMem::cudaMemAllocReport((void**)&opt_data.d_h_camera, 				PAR_KERNEL_LAUNCHS*CAM_ITE*NUMELEM_H*sizeof(float));

	

	CudaMem::cudaMemAllocReport((void**)&opt_data.d_hasCollision, 				sizeof(unsigned int));
	


	opt_data.h_costs = 			new double[PAR_KERNEL_LAUNCHS*CAM_ITE];
	opt_data.h_costs_buffer =	new double[PAR_KERNEL_LAUNCHS*CAM_ITE];
#ifdef DEBUG_RECORD
	opt_data.h_costs_result = 	new double[nOfCameraPos];
#endif






	opt_data.nearesNeighbourIndex = new int[robotpcl.n*6];
	NearestNeighbour::calcNearestNeighbourIndices(opt_data.nearesNeighbourIndex, &robotPCL_qa0);




	//setting pointer for launch configuration
	for(unsigned int i=0; i<PAR_KERNEL_LAUNCHS; i++)
	{
		opt_data.launchConfigurations[i].d_hs = &opt_data.allData.d_hs[i*CAM_ITE*N_int];
		opt_data.launchConfigurations[i].d_fa = &opt_data.allData.d_fa[i*CAM_ITE*N_int];
		opt_data.launchConfigurations[i].d_hs_final = &opt_data.allData.d_hs_final[i*CAM_ITE*N_int];
		opt_data.launchConfigurations[i].d_fa_final = &opt_data.allData.d_fa_final[i*CAM_ITE*N_int];
		opt_data.launchConfigurations[i].d_fa_costs = &opt_data.allData.d_fa_costs[i*CAM_ITE];
		opt_data.launchConfigurations[i].d_hs_costs = &opt_data.allData.d_hs_costs[i*CAM_ITE];
		opt_data.launchConfigurations[i].d_costs = &opt_data.allData.d_costs[i*CAM_ITE];
		opt_data.launchConfigurations[i].d_hdm = &opt_data.allData.d_hdm[i*depthBufferSize];
		opt_data.launchConfigurations[i].d_edm = &opt_data.allData.d_edm[i*depthBufferSize];
		opt_data.launchConfigurations[i].d_cp = &opt_data.allData.d_cp[i*CAM_ITE*NUMELEM_Cp];
		opt_data.launchConfigurations[i].d_mi = &opt_data.allData.d_mi[i*CAM_ITE*NUMELEM_Mi];
		opt_data.launchConfigurations[i].d_c = &opt_data.allData.d_c[i*CAM_ITE*NUMELEM_C];
		opt_data.launchConfigurations[i].d_robotoccupancy = opt_data.allData.d_robotoccupancy;
		opt_data.launchConfigurations[i].d_humanoccupancy = opt_data.allData.d_humanoccupancy;
		opt_data.launchConfigurations[i].d_ksdf = opt_data.allData.d_ksdf;



		cudaStatus = cudaStreamCreate (&opt_data.launchConfigurations[i].cudaStreams);
		if(cudaStatus != cudaSuccess)
		{
			fprintf(stderr, "stream creation failed: %s\n", cudaGetErrorString(cudaStatus));
		}

	}

	//load data into alldata memory
//	for(unsigned int i=0; i<robotPos.n; i++)
//		CudaMem::cudaMemCpyReport(&opt_data.allData.d_ksdf[i*N], robotPos.ksdf[i], N*sizeof(float), cudaMemcpyHostToDevice);
//
//	IO::loadFileOntoCuda("robotOccupancy.bin", opt_data.allData.d_robotoccupancy,robotPos.n*N_int*sizeof(int));
//	IO::loadFileOntoCuda("humanOccupancy.bin", opt_data.allData.d_humanoccupancy,robotPos.n*humanPos.n*N_int*sizeof(int));
//	IO::loadFileOntoHost("CP.bin", opt_data.h_cp,nOfCameraPos*3*sizeof(float));
//	IO::loadFileOntoHost("Mi.bin", opt_data.h_mi,nOfCameraPos*9*sizeof(float));
//	IO::loadFileOntoHost("C.bin", opt_data.h_c,nOfCameraPos*12*sizeof(float));

	checkMemoryUsage();

#ifdef DEBUG_RECORD
	//Init results
	for(unsigned int i=0; i<nOfCameraPos; i++)
		opt_data.h_costs_result[i] = 0.0;
#endif
}



bool CostFunctionClass::generatePCLandAngleIndex(void)
{
	unsigned int ite = 0;
	while(gen_result == GEN_NEXT && ite < MAX_ITE)
	{
		for(unsigned int i=0; i<nOfCams; i++)
		{
			opt_data.h_pcl_index[ite+i*MAX_ITE] = comp_vec[i]/pos.nOfAngles;
			opt_data.h_angle_index[ite+i*MAX_ITE] = comp_vec[i]- opt_data.h_pcl_index[ite]*pos.nOfAngles;
		}
		ite++;
		gen_result = gen_comb_norep_lex_next(comp_vec, robotPCL_qa0.n*pos.nOfAngles, nOfCams);
	}
	opt_data.currentNofValidIterations = ite;
	return true;
}

void CostFunctionClass::initCostArray(void)
{
	for(unsigned int i=0; i< MAX_ITE; i++)
		opt_data.h_costs_buffer[i] = 0.0;
}

void CostFunctionClass::initHSandFAFinal(void)
{
	CudaMem::cudaMemsetReport(opt_data.allData.d_hs_final,  0xFFFFFFFF, MAX_ITE*N_int*sizeof(int));	
	CudaMem::cudaMemsetReport(opt_data.allData.d_fa_final, 0x0, MAX_ITE*N_int*sizeof(int));
}

void CostFunctionClass::calculateCosts(void)
{
		//////fusing multiple grids
	for(unsigned int i=0; i<PAR_KERNEL_LAUNCHS; i++)
	{		
		cuda_calc::fuseGrids_shared<<<CAM_ITE, 1024, 0,opt_data.launchConfigurations[i].cudaStreams>>>(
				opt_data.launchConfigurations[i].d_hs_final,
				opt_data.launchConfigurations[i].d_fa_final,
				opt_data.launchConfigurations[i].d_ksdf,
				opt_data.launchConfigurations[i].d_hs_costs,
				opt_data.launchConfigurations[i].d_fa_costs);
		cudaStatus = cudaGetLastError();
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "fuseGrids_shared launch failed: %s\n", cudaGetErrorString(cudaStatus));
			return;
		}
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching fuseGrids_shared!\n", cudaStatus);
		return;
	}

	for(unsigned int i=0; i<PAR_KERNEL_LAUNCHS; i++)
	{
		cuda_calc::calcCalcCosts<<<1, CAM_ITE,0, opt_data.launchConfigurations[i].cudaStreams>>>(
				opt_data.launchConfigurations[i].d_hs_costs,
				opt_data.launchConfigurations[i].d_fa_costs,
				opt_data.launchConfigurations[i].d_costs);
		cudaStatus = cudaGetLastError();
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "fuseGrids_shared launch failed: %s\n", cudaGetErrorString(cudaStatus));
			return;
		}
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching fuseGrids_shared!\n", cudaStatus);
		return;
	}


}

void CostFunctionClass::optimize_all_memory(void)
{
	unsigned int cameraIndice =0;
	unsigned int nOfCameraPos = robotpcl_0.n * pos.nOfAngles;
	time_t 			start;
	assignNewCamera();



#ifndef DEBUG_RECORD
	sA->initializeFirstRun(opt_data.h_pcl_index, opt_data.h_angle_index);
#endif
	



	time(&start);
	bool isRunning = true;
	opt_data.currentNofValidIterations = MAX_ITE;
	while(isRunning)
	{
#ifdef	DEBUG_RECORD
		generatePCLandAngleIndex();
#endif
		CudaMem::cudaMemCpyReport(opt_data.d_pcl_index, opt_data.h_pcl_index, MAX_ITE*sizeof(int), cudaMemcpyHostToDevice);
		CudaMem::cudaMemCpyReport(opt_data.d_angle_index, opt_data.h_angle_index, MAX_ITE*sizeof(int), cudaMemcpyHostToDevice);

		initCostArray();
		int div = 0;
		for(unsigned int r=0; r<robotPos.n; r++)
		{
			setRobotOccupancyGrid(r);
			calculateKSDF_memory();
			//adjustCameraParameters(r,0);

			for(unsigned int h=0; h<humanPos.n; h++)
			{
				//is outside of every border
				if(!isValidPosition[r*humanPos.n + h])
					continue;
				
				setHumanOccupancyGrid(h,r);
				if(!doesHumanCollideWithRobot())
					continue;
				
				div++;
				initHSandFAFinal();
				for(unsigned int i=0; i < nOfCams; i++)
				{
					adjustCameraParameters(r,i);
					optimize_single();
				}
				calculateCosts();
				CudaMem::cudaMemCpyReport(opt_data.h_costs, opt_data.allData.d_costs, MAX_ITE*sizeof(double), cudaMemcpyDeviceToHost);

#ifdef DEBUG_RECORD
				for(unsigned int i=0; i<opt_data.currentNofValidIterations; i++)
				{
					opt_data.h_costs_result[cameraIndice+i] += opt_data.h_costs[i];
				}
				//if(r%10 == 0 && r > 0)
				//{
					//printProgress((double)(r*humanPos.n+h+1)* opt_data.currentNofValidIterations,(double)nOfCameraPos*robotPos.n*humanPos.n, start, "optimize");
					//printProgress((double)cameraIndice,(double)nOfCameraPos, start, "optimize");
					printf("robot pos: %i\thuman pos: %i\n", r,h);
				//}
#else
				for(unsigned int i=0; i<MAX_ITE; i++)
				{
					opt_data.h_costs_buffer[i] += opt_data.h_costs[i];
				}
#endif
				
			}
			
		}
		printProgress((double)cameraIndice,(double)nOfCameraPos, start, "optimize");

#ifdef DEBUG_RECORD
		for(unsigned int i=0; i<opt_data.currentNofValidIterations; i++)
		{
			opt_data.h_costs_result[cameraIndice+i] /= div;
		}
		cameraIndice += opt_data.currentNofValidIterations;
		isRunning = gen_result == GEN_NEXT;
		
#else
		for(unsigned int i=0; i<MAX_ITE; i++)
		{
			opt_data.h_costs_buffer[i] /= div;
		}
		isRunning = sA->iterate(opt_data.nearesNeighbourIndex,opt_data.h_pcl_index,  opt_data.h_angle_index, opt_data.h_costs_buffer); 
		
#endif

	}
#ifdef DEBUG_RECORD
	IO::printMinCostSingleCameraToFile(opt_data.h_costs_result, &pos, &robotPCL_qa0);
#else
	//sA->writeResultsToFile("sa_path", &robotPCL_qa0, &pos);
	sA->writeEnergyResultsToFile("cost_behave");
	sA->writeAllResultToFile("cost_energy");
#endif
	std::cout << "global minimum found" << std::endl;

	std::string mystring;
	getline (std::cin, mystring);
}


void CostFunctionClass::optimize_single(void)
{
	CudaMem::cudaMemsetReport(opt_data.allData.d_hs, 0x0, MAX_ITE*N_int*sizeof(int));	
	CudaMem::cudaMemsetReport(opt_data.allData.d_fa, 0x0, MAX_ITE*N_int*sizeof(int));

	for(unsigned int i=0; i<PAR_KERNEL_LAUNCHS; i++)
	{
		cuda_calc::raytrace_shared<<<10,320,0,opt_data.launchConfigurations[i].cudaStreams>>>(
				opt_data.launchConfigurations[i].d_robotoccupancy,
				opt_data.launchConfigurations[i].d_humanoccupancy,
				opt_data.launchConfigurations[i].d_mi,
				opt_data.launchConfigurations[i].d_cp,
				opt_data.launchConfigurations[i].d_edm,
				opt_data.launchConfigurations[i].d_hdm);
		cudaStatus = cudaGetLastError();
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "raytrace_shared launch failed: %s\n", cudaGetErrorString(cudaStatus));
			return;
		}
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching raytrace_shared!\n", cudaStatus);
		return;
	}
	cudaProfilerStop();



	dim3 dimBlock(8, 8, 8);
	dim3 dimGrid(8, 8, 4);
	for(unsigned int i=0; i<PAR_KERNEL_LAUNCHS; i++)
	{
		cuda_calc::project_voxel_into_ws<<<dimGrid,dimBlock,0, opt_data.launchConfigurations[i].cudaStreams>>>(
				opt_data.launchConfigurations[i].d_humanoccupancy,
				opt_data.launchConfigurations[i].d_edm,
				opt_data.launchConfigurations[i].d_hdm,
				opt_data.launchConfigurations[i].d_c,
				opt_data.launchConfigurations[i].d_cp,
				robotPos.n, humanPos.n,
				opt_data.launchConfigurations[i].d_fa,
				opt_data.launchConfigurations[i].d_hs);
		cudaStatus = cudaGetLastError();
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "project_voxel_into_ws launch failed: %s\n", cudaGetErrorString(cudaStatus));
			return;
		}
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching project_voxel_into_ws!\n", cudaStatus);
		return;
	}







	for(unsigned int i=0; i<PAR_KERNEL_LAUNCHS; i++)
	{
		cuda_calc::fuse2HSGrids<<<CAM_ITE*4,1024>>>(opt_data.launchConfigurations[i].d_hs_final,
													opt_data.launchConfigurations[i].d_hs,
													opt_data.launchConfigurations[i].d_fa_final,
													opt_data.launchConfigurations[i].d_fa);
		cudaStatus = cudaGetLastError();
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "fuse2HSGrids launch failed: %s\n", cudaGetErrorString(cudaStatus));
			return;
		}


	}
	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching fuse2HSGrids!\n", cudaStatus);
		return;
	}

}





void CostFunctionClass::checkMemoryUsage(void){
	size_t avail;
	size_t total;
	cudaMemGetInfo( &avail, &total );
	size_t used = total - avail;
	double used_d = (double)used;
	double total_d = (double)total;
	double percentage_used = (used_d / total_d)*100.0;
	std::cout.precision(4);
	std::cout << "Device memory used: " << percentage_used << std::endl;
}


void CostFunctionClass::initCameraPositions(struct POSITIONS* pos)
{

	unsigned int id = 0;
	float roll;
	float pitch;
	float yaw = 0.0f;
	float da = 2*MATH_PI/(float)N_OF_A;

	pos->nOfAngles = N_OF_A * N_OF_A* N_OF_A;
	pos->roll = new float[pos->nOfAngles];
	pos->pitch = new float[pos->nOfAngles];
	pos->yaw = new float[pos->nOfAngles];





	for(unsigned int rolli=0; rolli<N_OF_A; rolli++){
		for(unsigned int pitchi=0; pitchi<N_OF_A; pitchi++){
			for(unsigned int yawi=0;yawi<N_OF_A; yawi++)
			{
				id = rolli*N_OF_A*N_OF_A + pitchi*N_OF_A + yawi;

				roll = da * rolli -MATH_PI;
				pitch = da * pitchi -MATH_PI;
				yaw =  da * yawi -MATH_PI;

				pos->roll[id] = roll;
				pos->pitch[id] = pitch;
				pos->yaw[id] = yaw;
			}

		}
	}


	//CudaMem::cudaMemAllocReport((void**)&pos->d_roll, pos->nOfAngles*sizeof(float));
	//CudaMem::cudaMemAllocReport((void**)&pos->d_pitch, pos->nOfAngles*sizeof(float));
	//CudaMem::cudaMemAllocReport((void**)&pos->d_yaw, pos->nOfAngles*sizeof(float));

	//CudaMem::cudaMemCpyReport(pos->d_roll, pos->roll, pos->nOfAngles*sizeof(float), cudaMemcpyHostToDevice);
	//CudaMem::cudaMemCpyReport(pos->d_pitch, pos->pitch, pos->nOfAngles*sizeof(float), cudaMemcpyHostToDevice);
	//CudaMem::cudaMemCpyReport(pos->d_yaw, pos->yaw, pos->nOfAngles*sizeof(float), cudaMemcpyHostToDevice);






}

void CostFunctionClass::transformHumanIntoRobotSystem(float* qr, float* qh, float* robotToHuman)
{
	float r[16];
	float h[16];
	float t[16];

	rpy2r(0.0,0.0, qr[2], r);
	r[3] = qr[0];
	r[7] = qr[1];

	rpy2r(0.0,0.0, qh[2]+1.57, h);
	h[3] = qh[0];
	h[7] = qh[1];

	invert4(r, t);
	mm16(t,h,robotToHuman);
}


void CostFunctionClass::adjustRobotTransformation(float* qvin)
{
	//position hack for transforming everything into robot system
	//the first 3 degress of freedom are set to zero xg,yg,theta
	float qv[DOF_ROBOT_Q];
	memcpy(qv, qvin, DOF_ROBOT_Q*sizeof(float));
	qv[0] = 0.0f;
	qv[1] = 0.0f;
	qv[2] = 0.0f;
	float T1[16];
	float T2[16];
	float T3[16];

	memcpy(s_mmkr16.T, s_mmkr16.base, NUMELEM_H*sizeof(float));
	memcpy(T1, s_mmkr16.base, NUMELEM_H*sizeof(float));
	float sa, ca, q, st, ct,d;
	for(unsigned int j=0; j<DOF_ROBOT_Q; j++){

		sa = sin(s_mmkr16.alpha[j]);
		ca = cos(s_mmkr16.alpha[j]);
		q = qv[j] + s_mmkr16.offset[j];
		if (s_mmkr16.sigma[j] == 0.0){
			st = sin(q);
			ct = cos(q);
			d = s_mmkr16.d[j];
		}
		else{
			st = sin(s_mmkr16.theta[j]);
			ct = cos(s_mmkr16.theta[j]);
			d = q;
		}
		T2[0] = ct;
		T2[1] = -st*ca;
		T2[2] = st*sa;
		T2[3] = s_mmkr16.a[j]*ct;

		T2[4] = st;
		T2[5] = ct*ca;
		T2[6] = -ct*sa;
		T2[7] = s_mmkr16.a[j]*st;

		T2[8] = 0;
		T2[9] = sa;
		T2[10] = ca;
		T2[11] = d;

		T2[12] = 0;
		T2[13] = 0;
		T2[14] = 0;
		T2[15] = 1;

		mm16(T1,T2,T3);
		memcpy(&s_mmkr16.T[(j+1)*NUMELEM_H], T3,NUMELEM_H*sizeof(float));
		memcpy(T1,T3,NUMELEM_H * sizeof(float));

	}





}




void CostFunctionClass::initAllKSDF(bool writeToFile)
{
	time_t start;

	OCCUPANCY_GRIDS grid;
	CudaMem::allocGrid(&grid, xmin, xmax, ymin, ymax, zmin, zmax, nx, ny, nz);
	CudaMem::copyGridHostToDevice(&grid);

	PCL robotpcl;
	robotpcl.n = robotpcl_0.n;
	CudaMem::allocPCLN(&robotpcl);

	time(&start);
	for(unsigned int loop = 0; loop<robotPos.n; loop++)
	{
		CudaMem::cpyPCL(&robotpcl, &robotpcl_0);
		CudaMem::copyPCLHostToDevice(&robotpcl);

		adjustRobotTransformation(&robotPos.positions[9*loop]);
		calculateVelocity(&(robotPos.velocities[9*loop]));
		adjustrobotpclandvelocity(&robotpcl,robotPos.velocities[9*loop +0], robotPos.velocities[9*loop +1]);
		calculateKSDF(&robotpcl, &grid);
		CudaMem::cudaMemCpyReport(robotPos.ksdf[loop],grid.d_ksdf, grid.N * sizeof(float), cudaMemcpyDeviceToHost);
		if(loop % 10 == 0)
			printProgress((double)loop, (double)robotPos.n, start, "ksdf");

	}

	CudaMem::deleteGrid(&grid);
	CudaMem::deletePCL(&robotpcl);
	printf("saving ksdf file.....\n");
	if(writeToFile)
	{
		IO::writeKSDF(&robotPos, N);
	}


}


void CostFunctionClass::calculateVelocity(float* qdin)
{
	//updating velocity vector;
	float v[3], v_1[3], w[3], w_1[3], z_i[3], t_i[3], t_n[3], temp[3];
	float qd[DOF_ROBOT_Q];
	memcpy(qd,qdin,DOF_ROBOT_Q*sizeof(float));
	qd[0] = 0.0f;
	qd[1] = 0.0f;


	for(unsigned int j=0; j<DOF_ROBOT_Q; j++)
	{
		v_1[0] = 0.0f;
		v_1[1] = 0.0f;
		v_1[2] = 0.0f;

		w_1[0] = 0.0f;
		w_1[1] = 0.0f;
		w_1[2] = 0.0f;

		for (unsigned int i=0; i<=j; i++)
		{
			z_i[0] = s_mmkr16.T[i*NUMELEM_H+2];
			z_i[1] = s_mmkr16.T[i*NUMELEM_H+6];
			z_i[2] = s_mmkr16.T[i*NUMELEM_H+10];

			t_i[0] = s_mmkr16.T[i*NUMELEM_H+3];
			t_i[1] = s_mmkr16.T[i*NUMELEM_H+7];
			t_i[2] = s_mmkr16.T[i*NUMELEM_H+11];

			t_n[0] = s_mmkr16.T[(j+1)*NUMELEM_H+3];
			t_n[1] = s_mmkr16.T[(j+1)*NUMELEM_H+7];
			t_n[2] = s_mmkr16.T[(j+1)*NUMELEM_H+11];

			if (s_mmkr16.sigma[i] == 0.0){
				temp[0] = t_n[0] - t_i[0];
				temp[1] = t_n[1] - t_i[1];
				temp[2] = t_n[2] - t_i[2];
				cross(z_i, temp, v);
				w[0] = z_i[0];
				w[1] = z_i[1];
				w[2] = z_i[2];

			}
			else{
				v[0] = z_i[0];
				v[1] = z_i[1];
				v[2] = z_i[2];
				w[0] = 0.0;
				w[1] = 0.0;
				w[2] = 0.0;
			}

			v_1[0] = v_1[0] + v[0]*qd[i];
			v_1[1] = v_1[1] + v[1]*qd[i];
			v_1[2] = v_1[2] + v[2]*qd[i];

			//velocity bug
			w_1[0] = w_1[0] + w[0]*qd[i];
			w_1[1] = w_1[1] + w[1]*qd[i];
			w_1[2] = w_1[2] + w[2]*qd[i];

		}

		s_mmkr16.v_x[j] = v_1[0];
		s_mmkr16.v_y[j] = v_1[1];
		s_mmkr16.v_z[j] = v_1[2];

		s_mmkr16.w_x[j] = w_1[0];
		s_mmkr16.w_y[j] = w_1[1];
		s_mmkr16.w_z[j] = w_1[2];

	}
}

void CostFunctionClass::adjustrobotpclandvelocity_memory(struct PCL* dst,int i,  float vxp, float vyp)
{
	cudaError_t cudaStatus;

	dim3 dimBlock(1, 1, 1);
	dim3 dimGrid(1, 1, dst->n); //KernelFunc

	int offset_t = i*NUMELEM_H * (N_ELEMENT_T);
	int offset_v = i*(N_ELEMENT_T);


	cuda_calc::transformPCLandVelocity<<< dst->n,1>>>(dst->d_x, dst->d_y, dst->d_z, dst->d_v_x, dst->d_v_y, dst->d_v_z, dst->d_i,
			opt_data.s_mmkr16.d_T+offset_t,
			opt_data.s_mmkr16.d_v_x+offset_v,
			opt_data.s_mmkr16.d_v_y+offset_v,
			opt_data.s_mmkr16.d_v_z+offset_v,
			opt_data.s_mmkr16.d_w_x+offset_v,
			opt_data.s_mmkr16.d_w_y+offset_v,
			opt_data.s_mmkr16.d_w_z+offset_v,
			vxp, vyp);


	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "transformPCLandVelocity launch failed: %s\n", cudaGetErrorString(cudaStatus));
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "transformPCLandVelocity snychronize failed: %s\n", cudaGetErrorString(cudaStatus));
	}
}


void CostFunctionClass::adjustrobotpclandvelocity(struct PCL* dst, float vxp, float vyp)
{
	cudaError_t cudaStatus;
	// Choose which GPU to run on, change this on a multi-GPU system.


	dim3 dimBlock(1, 1, 1);
	dim3 dimGrid(1, 1, dst->n); //KernelFunc

	CudaMem::cudaMemCpyReport(s_mmkr16.d_T, s_mmkr16.T, NUMELEM_H * (N_ELEMENT_T) * sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(s_mmkr16.d_v_x, s_mmkr16.v_x, (N_ELEMENT_T) * sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(s_mmkr16.d_v_y, s_mmkr16.v_y, (N_ELEMENT_T) * sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(s_mmkr16.d_v_z, s_mmkr16.v_z, (N_ELEMENT_T) * sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(s_mmkr16.d_w_x, s_mmkr16.w_x, (N_ELEMENT_T) * sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(s_mmkr16.d_w_y, s_mmkr16.w_y, (N_ELEMENT_T) * sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(s_mmkr16.d_w_z, s_mmkr16.w_z, (N_ELEMENT_T) * sizeof(float), cudaMemcpyHostToDevice);

	cuda_calc::transformPCLandVelocity<<< dst->n,1>>>(dst->d_x, dst->d_y, dst->d_z, dst->d_v_x, dst->d_v_y, dst->d_v_z, dst->d_i,s_mmkr16.d_T,
		s_mmkr16.d_v_x, s_mmkr16.d_v_y, s_mmkr16.d_v_z, s_mmkr16.d_w_x, s_mmkr16.d_w_y, s_mmkr16.d_w_z, vxp, vyp);


	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "addKernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "addKernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
	}
}

void CostFunctionClass::assignRobotPclIntoWS_shared(struct PCL* pcl, unsigned int* const R)
{
	cudaError_t cudaStatus;
	assert(pcl->n == robotpcl_0.n);

	CudaMem::cudaMemsetReport(R, 0, N_char*sizeof(unsigned char));
	cuda_calc::pclintows_shared<<< 256,512>>>(R,
		(const float*)pcl->d_x, (const float*)pcl->d_y, (const float*)pcl->d_z, pcl->n,
		(const float*)d_eye);

	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "addKernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching addKernel!\n", cudaStatus);

	}
}

void CostFunctionClass::assignHumanPclIntoWS_memory(struct PCL* pcl, unsigned int* const H, int i)
{

	cudaError_t cudaStatus;
	int offset = NUMELEM_H*i;
	CudaMem::cudaMemsetReport(H, 0, N_char*sizeof(unsigned char));
	cuda_calc::pclintows_shared<<<256,512>>>(H,
		(const float*)pcl->d_x, (const float*)pcl->d_y, (const float*)pcl->d_z, pcl->n,
		(const float*)opt_data.s_human.d_h+offset);

	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "addKernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching addKernel!\n", cudaStatus);

	}
}

void CostFunctionClass::assignHumanPclIntoWS_shared(struct PCL* pcl, unsigned int* const H, float* h)
{

	cudaError_t cudaStatus;


	memcpy(pcl->h, h, 16*sizeof(float));
	CudaMem::cudaMemCpyReport(pcl->d_h,pcl->h, 16*sizeof(float), cudaMemcpyHostToDevice);


	cudaStatus = cudaMemset(H, 0, N_char*sizeof(unsigned char));
	if(cudaStatus != cudaSuccess)
	{
		printf("error copying symbols\n");
	}

	cuda_calc::pclintows_shared<<<256,512>>>(H,
		(const float*)pcl->d_x, (const float*)pcl->d_y, (const float*)pcl->d_z, pcl->n,
		(const float*)pcl->d_h);

	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "addKernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching addKernel!\n", cudaStatus);

	}
}


void CostFunctionClass::printProgress(double ci, double ei, time_t start, const char* task)
{
	double loopTime, timePerLoop, remainingTime, currPro,maxTime;
	time_t end;
	time(&end);
	loopTime = difftime(end, start);
	timePerLoop =  (loopTime /(double)ci);
	remainingTime = timePerLoop * (double) (ei - ci);
	maxTime = timePerLoop * ei;
	currPro = (double)ci/ (double)(ei);
	printf("%s max : %.1fmin\trem : %.2fmin\tpro: %.6f%%\n",task, maxTime/60.0, remainingTime/60.0, currPro*100.0);

}

void CostFunctionClass::getCameraPosition(int id, float roll, float pitch, float yaw, struct PCL* pcl, float* h)
{
	//hack

//	 1.2000    0.0075    1.7173         0    1.7708         0
	int index = robotpcl_0.i[id];
	float r[16];
	rpy2r(roll,pitch, yaw, r);
	r[3] = robotPCL_qa0.x[id];
	r[7] = robotPCL_qa0.y[id];
	r[11] = robotPCL_qa0.z[id];

	//rpy2r(0.0f,1.7708f, 0.0f, r);
	//r[3] = 1.2000f;
	//r[7] = 0.0075f;
	//r[11] = 1.7173f;

	float cii[16];
	float cil[16];


	invert4(&(s_mmkr16_q0.T[16*index]), cii);
	mm16(cii,r,cil);
	mm16(&(s_mmkr16.T[16*index]), cil, h);

}

void CostFunctionClass::updateCamera(struct CAM* cam, float *h)
{
	float i[16];
	float iMi[9];
	invert4(h,i);
	mm12x16(cam->C0, i,cam->C);

	iMi[0] = cam->C[0];
	iMi[1] = cam->C[1];
	iMi[2] = cam->C[2];

	iMi[3] = cam->C[4];
	iMi[4] = cam->C[5];
	iMi[5] = cam->C[6];

	iMi[6] = cam->C[8];
	iMi[7] = cam->C[9];
	iMi[8] = cam->C[10];

	invert3(iMi, cam->Mi);

	cam->x = h[3];
	cam->y = h[7];
	cam->z = h[11];
}

void CostFunctionClass::calculateKSDF_memory(void)
{


	cudaError_t cudaStatus;
	cuda_calc::ksdf<<< 512,256>>>(opt_data.allData.d_ksdf,
		(const float*)robotpcl.d_x, (const float*)robotpcl.d_y, (const float*)robotpcl.d_z, robotpcl.n,
		(const float*)robotpcl.d_v_x, (const float*)robotpcl.d_v_y, (const float*)robotpcl.d_v_z,
		(const float*) opt_data.d_ws_x, (const float*) opt_data.d_ws_y, (const float*) opt_data.d_ws_z);

	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		std::cout << cudaGetErrorString(cudaStatus) << std::endl;
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		std::cout << cudaGetErrorString(cudaStatus) << std::endl;

	}

}
void CostFunctionClass::initWSCoordinates(void)
{
	CudaMem::cudaMemAllocReport((void**)&opt_data.d_ws_x, N*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&opt_data.d_ws_y, N*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&opt_data.d_ws_z, N*sizeof(float));

	float* ws_x = new float[N];
	float* ws_y = new float[N];
	float* ws_z = new float[N];

	unsigned int id = 0;
	float dx = (xmax-xmin)/(nx-1);
	float dy = (ymax-ymin)/(ny-1);
	float dz = (zmax-zmin)/(nz-1);
	float x = 0.0;
	float y = 0.0;
	float z = 0.0;
	for(unsigned int zi=0; zi<nz; zi++){
		for(unsigned int xi=0; xi<nx; xi++){
			for(unsigned int yi=0; yi<ny; yi++){
				id = zi*nx*ny + xi*ny + yi;
				x = dx * xi + xmin;
				y = dy * yi + ymin;
				z = dz * zi + zmin;
				ws_x[id] = x;
				ws_y[id] = y;
				ws_z[id] = z;
			}
		}
	}
	CudaMem::cudaMemCpyReport(opt_data.d_ws_x, ws_x, N*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(opt_data.d_ws_y, ws_y, N*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(opt_data.d_ws_z, ws_z, N*sizeof(float), cudaMemcpyHostToDevice);


}

void CostFunctionClass::calculateKSDF(struct PCL* dst, struct OCCUPANCY_GRIDS* grid)
{


	cudaError_t cudaStatus;
	cuda_calc::ksdf<<< 512,256>>>(grid->d_ksdf,
		(const float*)dst->d_x, (const float*)dst->d_y, (const float*)dst->d_z, dst->n,
		(const float*)dst->d_v_x, (const float*)dst->d_v_y, (const float*)dst->d_v_z,
		(const float*)grid->d_x, (const float*)grid->d_y, (const float*)grid->d_z);

	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "ksdf kernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching ksdf kernel!\n", cudaStatus);

	}

}

void CostFunctionClass::adjustrobotpcl(struct PCL* dst)
{
	cudaError_t cudaStatus;


	dim3 dimBlock(1, 1, 1);
	dim3 dimGrid(1, 1, dst->n); //KernelFunc
	//tion<<<dimGrid, dimBlock>>>(�);

	CudaMem::cudaMemCpyReport(s_mmkr16.d_T, s_mmkr16.T, NUMELEM_H * (N_ELEMENT_T) * sizeof(float), cudaMemcpyHostToDevice);

	cuda_calc::transformPCL<<< dst->n,1>>>(dst->d_x, dst->d_y, dst->d_z, dst->d_i,s_mmkr16.d_T);
	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "addKernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching addKernel!\n", cudaStatus);

	}

}


void CostFunctionClass::testCudaFunctions()
{
	float h1[16], h2[16], r1[16], r2[16];
	for(unsigned int i=0; i<16; i++)
	{
		h1[i] = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		h2[i] = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	}

	mm16(h1, h2, r1);

	float *d_h1, *d_h2, *d_r2;
	CudaMem::cudaMemAllocReport((void**)&d_h1, 16*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&d_h2, 16*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&d_r2, 16*sizeof(float));

	CudaMem::cudaMemCpyReport(d_h1, h1, 16*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(d_h2, h2, 16*sizeof(float), cudaMemcpyHostToDevice);

	cudaError_t cudaStatus;
	cuda_calc::testMM16<<<1, 16>>>(d_h1, d_h2, d_r2);
	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "test kernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching test kernel!\n", cudaStatus);

	}

	CudaMem::cudaMemCpyReport(r2, d_r2, 16*sizeof(float), cudaMemcpyDeviceToHost);

	float eps = 1e-10;
	for(int i=0; i<16; i++)
	{
		if(abs(r2[i]-r1[i]) > eps)
		{
			fprintf(stderr, "error");
		}
	}
	

}


void CostFunctionClass::testCudaInverse()
{
	float h1[16], h2[16], r1[16], r2[16];
	for(unsigned int i=0; i<16; i++)
	{
		h1[i] = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	}

	invert4(h1, r1);

	float *d_h1, *d_r2;
	CudaMem::cudaMemAllocReport((void**)&d_h1, 16*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&d_r2, 16*sizeof(float));

	CudaMem::cudaMemCpyReport(d_h1, h1, 16*sizeof(float), cudaMemcpyHostToDevice);

	cudaError_t cudaStatus;
	cuda_calc::testInvert4<<<1, 1>>>(d_h1, d_r2);
	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "test kernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching test kernel!\n", cudaStatus);

	}

	CudaMem::cudaMemCpyReport(r2, d_r2, 16*sizeof(float), cudaMemcpyDeviceToHost);

	float eps = 1e-10;
	for(int i=0; i<16; i++)
	{
		if(abs(r2[i]-r1[i]) > eps)
		{
			fprintf(stderr, "error");
		}
	}
	

}

void CostFunctionClass::testUpdateCameraParameters()
{

	struct CAM testCam;

	float h[] = EYE;
	float roll = -1.0f;
	float pitch = -1.0f;
	float yaw = -1.0f;
	rpy2r(roll,pitch, yaw, h);
	
	

	CostFunctionClass::initCam(&testCam);
	CostFunctionClass::updateCamera(&testCam, h);


#define nOfSamples 256
	float *d_C, *d_Cp, *d_Mi, *d_H, *d_C0;
	float C[nOfSamples*NUMELEM_C], Cp[nOfSamples*NUMELEM_Cp], Mi[nOfSamples*NUMELEM_Mi];



	CudaMem::cudaMemAllocReport((void**)&d_C, nOfSamples*NUMELEM_C*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&d_Cp, nOfSamples*NUMELEM_Cp*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&d_Mi, nOfSamples*NUMELEM_Mi*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&d_H, nOfSamples*NUMELEM_H*sizeof(float));

	for(unsigned int i=0; i<nOfSamples; i++)
	{
		CudaMem::cudaMemCpyReport(d_H+i*NUMELEM_H, h, NUMELEM_H*sizeof(float), cudaMemcpyHostToDevice);
	}

	cudaError_t cudaStatus;
	cuda_calc::updateCameraParameters<<<1, nOfSamples>>>(testCam.d_C0,d_H, d_C, d_Cp, d_Mi);
	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "test kernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching test kernel!\n", cudaStatus);

	}

	CudaMem::cudaMemCpyReport(C, d_C, nOfSamples*NUMELEM_C*sizeof(float), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(Cp, d_Cp, nOfSamples*NUMELEM_Cp*sizeof(float), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(Mi, d_Mi, nOfSamples*NUMELEM_Mi*sizeof(float), cudaMemcpyDeviceToHost);

	float eps = 1e-5;
	for(int i=0; i<nOfSamples*NUMELEM_C; i++)
	{
		if(abs(testCam.C[i%NUMELEM_C]-C[i]) > eps)
		{
			fprintf(stderr, "error");
		}
	}


	if(abs(testCam.x-Cp[0]) > eps)
	{
		fprintf(stderr, "error");
	}


	if(abs(testCam.y-Cp[1]) > eps)
	{
		fprintf(stderr, "error");
	}

	if(abs(testCam.z-Cp[2]) > eps)
	{
		fprintf(stderr, "error");
	}
	

	for(int i=0; i<nOfSamples*NUMELEM_Mi; i++)
	{
		if(abs(testCam.Mi[i%NUMELEM_Mi]-Mi[i]) > eps)
		{
			fprintf(stderr, "error");
		}
	}
	

}

void CostFunctionClass::testSelectNN(void)
{
	//int startPCL=0, startAngle = 0;
	//printf("starting point: [%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f]\n", robotPCL_qa0.x[startPCL], robotPCL_qa0.y[startPCL], robotPCL_qa0.z[startPCL], pos.roll[startAngle], pos.pitch[startAngle], pos.yaw[startAngle]);
	//NearestNeighbour::setNearestNeighbour((const int*)opt_data.nearesNeighbourIndex, 0, 0, opt_data.h_pcl_index, opt_data.h_angle_index);
	//for(unsigned int i=0; i<12; i++)
	//{
	//	startPCL = opt_data.h_pcl_index[i];
	//	startAngle = opt_data.h_angle_index[i];
	//	printf("point %i: [%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f]\n", i,
	//																	robotPCL_qa0.x[startPCL],
	//																	robotPCL_qa0.y[startPCL],
	//																	robotPCL_qa0.z[startPCL],
	//																	pos.roll[startAngle],
	//																	pos.pitch[startAngle], 
	//																	pos.yaw[startAngle]);
	//}

	int p = 0, int a = 0, rp, ra;
	double cm = 0.0, cp = 1.0, c_t1= 1.0;
	unsigned char dim = 4;
	NearestNeighbour::GRADIENT_DIM dir;
	NearestNeighbour::setNextIteration(cm, cp, c_t1,dim,(const int*)opt_data.nearesNeighbourIndex,p,a,&rp,&ra,&dir);


}