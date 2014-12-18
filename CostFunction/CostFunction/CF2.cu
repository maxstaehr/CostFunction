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
#include "allKernelFct2.cuh"
CF2::CF2()
{
	cudaFuncSetCacheConfig(cuda_calc2::raytraceVertices, cudaFuncCachePreferShared);
	IO::loadRobotPCL(&robot,"robot.bin");
	IO::loadHumanPCL(&human,"human.bin");
	IO::loadEnvironmentPCL(&environment, "environment.bin");

	IO::loadSamplePositions(&samplePositions, "samplePositions.bin");
	IO::loadSamplePCL(&samplePoints, "samplePoints.bin");
	IO::loadSampleRotations(&sampleRotations, "sampleRotations.bin");
	IO::loadSampleCamera(&sampleCamera, "sampleCamera.bin");

	initVertexBuffer();
	initBoundingBoxBuffer();
	initDepthBuffer(3200);
	initSamplePointsBuffer();

	transformVertexBuffer(0);
	transformBoundingBoxBuffer(0);
	transformSamplePointBuffer(0);

	IO::saveVerticeBufferToFile(&vertexBufferRobot, "vertexRobot.bin");
	IO::saveVerticeBufferToFile(&vertexBufferHuman, "vertexHuman.bin");
	IO::saveVerticeBufferToFile(&vertexBufferEnvironment, "vertexEnvironment.bin");
	IO::saveBoundingBoxBufferToFile(&boundingBoxBuffer, "boundingBox.bin");
	

	rayTrace();
	IO::saveDepthBufferToFile(&depthBuffer, "depthBuffer.bin");
}

void CF2::initBoundingBoxBuffer()
{
	boundingBoxBuffer.nBB = robot.nBB+environment.nBB;

	//allocating memory
	CudaMem::cudaMemAllocReport((void**)&boundingBoxBuffer.d_BB, boundingBoxBuffer.nBB*NUMELEM_H*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&boundingBoxBuffer.d_D, boundingBoxBuffer.nBB*3*sizeof(float));

	//setting bounding box buffer
	boundingBoxBufferRobot.nBB = robot.nBB;
	boundingBoxBufferRobot.d_BB = boundingBoxBuffer.d_BB;
	boundingBoxBufferRobot.d_D = boundingBoxBuffer.d_D;

	boundingBoxBufferEnvironment.nBB = environment.nBB;
	boundingBoxBufferEnvironment.d_BB = boundingBoxBuffer.d_BB+boundingBoxBufferRobot.nBB*NUMELEM_H;
	boundingBoxBufferEnvironment.d_D = boundingBoxBuffer.d_D+boundingBoxBufferRobot.nBB*3;

	CudaMem::cudaMemCpyReport(boundingBoxBufferRobot.d_BB, robot.d_bb_H, boundingBoxBufferRobot.nBB*NUMELEM_H*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(boundingBoxBufferRobot.d_D, robot.d_bb_D, boundingBoxBufferRobot.nBB*3*sizeof(float), cudaMemcpyHostToDevice);

	CudaMem::cudaMemCpyReport(boundingBoxBufferEnvironment.d_BB, environment.d_bb_H, boundingBoxBufferEnvironment.nBB*NUMELEM_H*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(boundingBoxBufferEnvironment.d_D, environment.d_bb_D, boundingBoxBufferEnvironment.nBB*3*sizeof(float), cudaMemcpyHostToDevice);


}

void CF2::initVertexBuffer()
{
	//calculating vertexBufferSize
	int vertexSize = robot.nV + human.nV + environment.nV;
	int faceSize = robot.nF + human.nF + environment.nF;
	vertexBuffer.nF = faceSize;
	//allocating memory
	CudaMem::cudaMemAllocReport((void**)&vertexBuffer.d_vx, vertexSize*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&vertexBuffer.d_vy, vertexSize*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&vertexBuffer.d_vz, vertexSize*sizeof(float));

	CudaMem::cudaMemAllocReport((void**)&vertexBuffer.d_fx, faceSize*sizeof(int));
	CudaMem::cudaMemAllocReport((void**)&vertexBuffer.d_fy, faceSize*sizeof(int));
	CudaMem::cudaMemAllocReport((void**)&vertexBuffer.d_fz, faceSize*sizeof(int));

	//setting up face buffer for complete mesh
	int* fb_x = new int[faceSize];
	int* fb_y = new int[faceSize];
	int* fb_z = new int[faceSize];
	
	//setting vertexBufferPointer
	vertexBufferRobot.d_vx = vertexBuffer.d_vx;
	vertexBufferRobot.d_vy = vertexBuffer.d_vy;
	vertexBufferRobot.d_vz = vertexBuffer.d_vz;

	vertexBufferRobot.d_fx = vertexBuffer.d_fx;
	vertexBufferRobot.d_fy = vertexBuffer.d_fy;
	vertexBufferRobot.d_fz = vertexBuffer.d_fz;

	vertexBufferRobot.nF = robot.nF;
	vertexBufferRobot.nV = robot.nV;

	vertexBufferHuman.d_vx = vertexBuffer.d_vx+robot.nV;
	vertexBufferHuman.d_vy = vertexBuffer.d_vy+robot.nV;
	vertexBufferHuman.d_vz = vertexBuffer.d_vz+robot.nV;

	vertexBufferHuman.d_fx = vertexBuffer.d_fx+robot.nF;
	vertexBufferHuman.d_fy = vertexBuffer.d_fy+robot.nF;
	vertexBufferHuman.d_fz = vertexBuffer.d_fz+robot.nF;

	vertexBufferHuman.nF = human.nF;
	vertexBufferHuman.nV = human.nV;

	vertexBufferEnvironment.d_vx = vertexBuffer.d_vx+robot.nV+human.nV;
	vertexBufferEnvironment.d_vy = vertexBuffer.d_vy+robot.nV+human.nV;
	vertexBufferEnvironment.d_vz = vertexBuffer.d_vz+robot.nV+human.nV;

	vertexBufferEnvironment.d_fx = vertexBuffer.d_fx+robot.nF+human.nF;
	vertexBufferEnvironment.d_fy = vertexBuffer.d_fy+robot.nF+human.nF;
	vertexBufferEnvironment.d_fz = vertexBuffer.d_fz+robot.nF+human.nF;

	vertexBufferEnvironment.nF = environment.nF;
	vertexBufferEnvironment.nV = environment.nV;

	memcpy(fb_x, robot.fx, robot.nF*sizeof(int));
	memcpy(fb_y, robot.fy, robot.nF*sizeof(int));
	memcpy(fb_z, robot.fz, robot.nF*sizeof(int));

	for(int i=0; i<human.nF; i++)
	{
		fb_x[i+robot.nF] = human.fx[i]+robot.nF;
		fb_y[i+robot.nF] = human.fy[i]+robot.nF;
		fb_z[i+robot.nF] = human.fz[i]+robot.nF;
	}

	for(int i=0; i<environment.nF; i++)
	{
		fb_x[i+robot.nF+human.nF] = environment.fx[i]+robot.nF+human.nF;
		fb_y[i+robot.nF+human.nF] = environment.fy[i]+robot.nF+human.nF;
		fb_z[i+robot.nF+human.nF] = environment.fz[i]+robot.nF+human.nF;
	}

	CudaMem::cudaMemCpyReport(vertexBuffer.d_fx, fb_x, faceSize*sizeof(int), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(vertexBuffer.d_fy, fb_y, faceSize*sizeof(int), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(vertexBuffer.d_fz, fb_z, faceSize*sizeof(int), cudaMemcpyHostToDevice);
}

void CF2::transformVertexBuffer(int i)
{

	cudaError_t cudaStatus;
	float* t_offset = samplePositions.d_qr+i*N_ELEMENT_T *NUMELEM_H;
	float* h_offset = samplePositions.d_qh+i*N_ELEMENT_HU *NUMELEM_H;
	float* e_offset = samplePositions.d_qe+i*N_ELEMENT_EV *NUMELEM_H;
	
	cuda_calc2::transformVertexRobot<<<robot.nV,1>>>(robot.d_x,
													robot.d_y,
													robot.d_z,
													robot.d_vi,
													t_offset,
													vertexBufferRobot.d_vx,
													vertexBufferRobot.d_vy,
													vertexBufferRobot.d_vz);
					

	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "transformVertexRobot launch failed: %s\n", cudaGetErrorString(cudaStatus));
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching transformVertexRobot!\n", cudaStatus);

	}

	
	cuda_calc2::transformVertexHumanOrEnvironment<<<human.nV,1>>>(
													human.d_x,
													human.d_y,
													human.d_z,													
													h_offset,
													vertexBufferHuman.d_vx,
													vertexBufferHuman.d_vy,
													vertexBufferHuman.d_vz);
					

	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "transformVertexHumanOrEnvironment launch failed: %s\n", cudaGetErrorString(cudaStatus));
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching transformVertexHumanOrEnvironment!\n", cudaStatus);

	}

	cuda_calc2::transformVertexHumanOrEnvironment<<<environment.nV,1>>>(
													environment.d_x,
													environment.d_y,
													environment.d_z,													
													e_offset,
													vertexBufferEnvironment.d_vx,
													vertexBufferEnvironment.d_vy,
													vertexBufferEnvironment.d_vz);
	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "transformVertexHumanOrEnvironment launch failed: %s\n", cudaGetErrorString(cudaStatus));
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching transformVertexHumanOrEnvironment!\n", cudaStatus);

	}
}

void CF2::transformBoundingBoxBuffer(int i)
{
	cudaError_t cudaStatus;
	float* t_offset = samplePositions.d_qr+i*N_ELEMENT_T *NUMELEM_H;
	float* h_offset = samplePositions.d_qh+i*N_ELEMENT_HU *NUMELEM_H;
	float* e_offset = samplePositions.d_qe+i*N_ELEMENT_EV *NUMELEM_H;
	
	//(float* t, int* ii, float* h_bb, float* h_inv)
	cuda_calc2::transformBoundingBoxRobot<<<robot.nBB,1>>>(
													t_offset,													
													robot.d_bbi,
													robot.d_bb_H,
													boundingBoxBufferRobot.d_BB);

					

	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "transformVertexRobot launch failed: %s\n", cudaGetErrorString(cudaStatus));
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching transformVertexRobot!\n", cudaStatus);

	}

	//(float* hi_robot, float* h_bb, float* h_inv)
	cuda_calc2::transformBoundingBoxEnvironment<<<environment.nBB,1>>>(
													e_offset,													
													environment.d_bb_H,
													boundingBoxBufferEnvironment.d_BB);

					

	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "transformVertexRobot launch failed: %s\n", cudaGetErrorString(cudaStatus));
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching transformVertexRobot!\n", cudaStatus);

	}

}

void CF2::initDepthBuffer(int size)
{
	depthBuffer.size = size;
	depthBuffer.dx = new float[size];
	depthBuffer.dy = new float[size];
	depthBuffer.dz = new float[size];
	CudaMem::cudaMemAllocReport((void**)&depthBuffer.d_dx, size*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&depthBuffer.d_dy, size*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&depthBuffer.d_dz, size*sizeof(float));
}

void CF2::rayTrace()
{
	//raytraceAndCalcDistanceToModel(	float* xi, float* yi, float* zi,
	//													int* fx, int* fy, int* fz, int nF,
	//													float* bb_H, float* bb_D, int nBB, 
	//													float* D)
	//float* camPos_H, float* camRot_H, int cami
	//float*	d_bb_H;
	//float*	d_bb_D;
	//int*	d_bbi;
	int indexPos = 50;
	int indexRot = 10;

	float* samplePosOffet = samplePointsBuffer.d_H	+indexPos*NUMELEM_H;
	float* sampleRotOffet = sampleRotations.d_R		+indexRot*NUMELEM_H;

		//__global__ void raytraceVertices(					float* xi, float* yi, float* zi,
		//												int* fx, int* fy, int* fz, int nF,
		//												float* bb_H, float* bb_D, int nBB, 
		//												float* camPos_H, float* camRot_H,
		//												float* camRayX, float* camRayY, float* camRayZ,
		//												float* Dx, float* Dy, float* Dz

	cudaError_t cudaStatus;
	cuda_calc2::raytraceVertices<<<10,320>>>(		vertexBuffer.d_vx,
													vertexBuffer.d_vy,
													vertexBuffer.d_vz,																										
													vertexBuffer.d_fx,
													vertexBuffer.d_fy,
													vertexBuffer.d_fz,													
													vertexBuffer.nF, 
													robot.d_bb_H,
													robot.d_bb_D,
													robot.nBB,
													samplePosOffet,
													sampleRotOffet,
													sampleCamera.d_x,
													sampleCamera.d_y,
													sampleCamera.d_z,
													depthBuffer.d_dx,
													depthBuffer.d_dy,
													depthBuffer.d_dz);

	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "raytraceVertices launch failed: %s\n", cudaGetErrorString(cudaStatus));
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching raytraceVertices!\n", cudaStatus);

	}
}

void CF2::initSamplePointsBuffer()
{
	samplePointsBuffer.n = samplePoints.n;
	CudaMem::cudaMemAllocReport((void**)&samplePointsBuffer.d_H, samplePointsBuffer.n*NUMELEM_H*sizeof(float));
}

void CF2::transformSamplePointBuffer(int i)
{
	
	cudaError_t cudaStatus;
	float* t_offset = samplePositions.d_qr+i*N_ELEMENT_T *NUMELEM_H;
	
	//(float* t, float* ii, float* spi, float* spo)
	cuda_calc2::transformSamplePoint<<<samplePoints.n,1>>>(
													t_offset,													
													samplePoints.d_i,
													samplePoints.d_h,
													samplePointsBuffer.d_H);

					

	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "transformSamplePointBuffer launch failed: %s\n", cudaGetErrorString(cudaStatus));
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching transformSamplePointBuffer!\n", cudaStatus);

	}
}