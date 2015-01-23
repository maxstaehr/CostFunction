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
#include <sstream> 
#include <string> 
#include "IO.h"
#include <stdlib.h>     /* srand, rand */
#include "_generate.h"
#include "CompleteEnumeration.h"
#include "SimulatedAnnealing.h"
#include "Progress.h"
#include "cudaProfiler.h"


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
CF2::CF2():currentNumberOfCams(1)
{

	cudaError_t	cudaStatus = cudaDeviceReset();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "could not reset cuda device");

	}

	cudaFuncSetCacheConfig(cuda_calc2::raytraceVertices, cudaFuncCachePreferShared);
	cudaFuncSetCacheConfig(cuda_calc2::calcMiddlePoint, cudaFuncCachePreferShared);
	cudaFuncSetCacheConfig(cuda_calc2::distanceToEllipseModel, cudaFuncCachePreferShared);

	//init random number generator
	srand (time(NULL));
	cameraCombination.vector = NULL;



	
	IO::loadRobotPCL(&robot,"robot.bin");
	IO::loadHumanPCL(&human,"human.bin");
	IO::loadEnvironmentPCL(&environment, "environment.bin");

	IO::loadSamplePositions(&samplePositions, "samplePositions.bin");
	IO::loadSamplePCL(&samplePoints, "samplePoints.bin");
	IO::loadSampleRotations(&sampleRotations, "sampleRotations.bin");
	IO::loadSampleCamera(&sampleCameraTypes, "sampleCamera.bin");
	IO::loadSampleFitting(&sampleFitting, &launchConfigModelFitting, "sampleFitting.bin");
	IO::loadDistanceMatrix(&distMatrix, "distMatrix.bin");
	//setting launch Configuration

	initVertexBuffer();
	initBoundingBoxBuffer();
	initSamplePointsBuffer();






	setCurrentTans(0);
	transformVertexBuffer();
	transformBoundingBoxBuffer();
	transformSamplePointBuffer();

	

	IO::saveVerticeBufferToFile(&vertexBufferRobot, "vertexRobot.bin");
	IO::saveVerticeBufferToFile(&vertexBufferHuman, "vertexHuman.bin");
	IO::saveVerticeBufferToFile(&vertexBufferEnvironment, "vertexEnvironment.bin");
	IO::saveVerticeBufferToFile(&vertexBuffer, "vertexScene.bin");
	IO::saveBoundingBoxBufferToFile(&boundingBoxBuffer, "boundingBox.bin");
	IO::saveBoundingBoxBuffer(&boundingBoxBuffer, "boundingBoxBuffer.bin");

	nn = new NearestNeighbour(&samplePoints, &sampleRotations);

	
	


}

void CF2::saveAllVertices()
{
	IO::saveVerticeBufferToFile(&vertexBufferRobot, "vertexRobot.bin");
	IO::saveVerticeBufferToFile(&vertexBufferHuman, "vertexHuman.bin");
	IO::saveVerticeBufferToFile(&vertexBufferEnvironment, "vertexEnvironment.bin");
	IO::saveVerticeBufferToFile(&vertexBuffer, "vertexScene.bin");
	//IO::saveBoundingBoxBufferToFile(&boundingBoxBuffer, "boundingBox.bin");
	IO::saveBoundingBoxBuffer(&boundingBoxBuffer, "boundingBoxBuffer.bin");
}

void CF2::iterateCameraCombination()
{	
	for(int x = 0; x < currentNumberOfCams; x++)
		printf("%u ", cameraCombination.vector[x]);
	printf("\n"); 
	cameraCombination.gen_result = gen_comb_norep_lex_next(cameraCombination.vector, sampleCameraTypes.nCameraTypes, currentNumberOfCams);
}

void CF2::initCameraCombination()
{
	if(cameraCombination.vector != NULL) delete cameraCombination.vector;

	cameraCombination.vector = new unsigned long long[currentNumberOfCams];
	cameraCombination.gen_result = gen_comb_norep_lex_init(cameraCombination.vector, sampleCameraTypes.nCameraTypes, currentNumberOfCams);
}

void CF2::run()
{
	//outer loop for combination of multiple cameras
	initCameraCombination();
	//do
	//{
			

		//camera specific allocation
		initParallelOptiRuns();

		//inner loop for possible positions		
	//	sC = new CompleteEnumeration(&samplePoints, &sampleRotations, currentNumberOfCams, MAX_ITE, NULL);
		sC = new SimulatedAnnealing(&samplePoints, &sampleRotations, currentNumberOfCams, MAX_ITE, nn->getNN());
		time_t start;
		time(&start);
		int debugIte = 0;
		cudaProfilerStart();
		while(sC->iterate(optiSession.pI, optiSession.aI, probResult.maxp) )
		{
			//zeroProb();

			//samplePositions.nP = 2;
			for(int i=0; i< samplePositions.nP; i++)
			{				
				setCurrentTans(i);
				transformVertexBuffer();
				transformBoundingBoxBuffer();
				transformSamplePointBuffer();

				rayTrace();
				calculateCentroid();
				calculateProbOfHumanDetection();
				calculateMaxProb();
				//Progress::printProgress((double)i, (double)samplePositions.nP, start, "raytracing rp ");	
			}
			//normalizeProb();
			//for(int i=0; i<optiSession.n; i++)
			//{
			//	RAYTRACING_LAUNCH* p_rtl;
			//	p_rtl = &optiSession.launchs[0];	
			//	std::ostringstream s;
			//	s << i;
			//	std::string depth =   "depthBuffer"+ s.str()+".bin";     
			//	std::string prob =   "probResult"+ s.str()+".bin";     
			//
			//	IO::saveDepthBufferToFile(&p_rtl->depthBuffer,depth.c_str());
			//	IO::saveProbResult2File(&p_rtl->probResult, prob.c_str());
			//	IO::printCentroid(&p_rtl->centroid);
			//}
			//RAYTRACING_LAUNCH* p_rtl;
			//p_rtl = &optiSession.launchs[0];
			//IO::plotIntermediateResults(&p_rtl->probResult, &p_rtl->centroid);
			Progress::printProgress((double)optiSession.pI[0], (double)samplePoints.n, start, "raytracing cp ");
			debugIte++;
		}
		cudaProfilerStop();
	
		//saving results
		saveAllVertices();
		setCurrentTans(0);
		transformSamplePointBuffer();
		IO::saveOptimisationResults(&samplePointsBuffer, &samplePoints, &sampleRotations, sC->prop, "completeEnumeration.bin");

		delete sC;
		freeParallelOptiRuns();


		iterateCameraCombination();
	//}while(cameraCombination.gen_result == GEN_NEXT);
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

	//CudaMem::cudaMemCpyReport(boundingBoxBufferRobot.d_BB, robot.d_bb_H, boundingBoxBufferRobot.nBB*NUMELEM_H*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(boundingBoxBufferRobot.d_D, robot.d_bb_D, boundingBoxBufferRobot.nBB*3*sizeof(float), cudaMemcpyDeviceToDevice);

	//CudaMem::cudaMemCpyReport(boundingBoxBufferEnvironment.d_BB, environment.d_bb_H, boundingBoxBufferEnvironment.nBB*NUMELEM_H*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(boundingBoxBufferEnvironment.d_D, environment.d_bb_D, boundingBoxBufferEnvironment.nBB*3*sizeof(float), cudaMemcpyDeviceToDevice);


}

void CF2::createCudaStream(cudaStream_t** streams, int n)
{
		cudaError_t cudaStatus;

		*streams = new cudaStream_t[n];
		for(int i=0;i<n;i++)
		{
			cudaStatus = cudaStreamCreate (&(*streams)[i]);
			if(cudaStatus != cudaSuccess)
			{
				fprintf(stderr, "stream creation failed: %s\n", cudaGetErrorString(cudaStatus));
			}
		}
}

void CF2::freeCudaStream(cudaStream_t* streams,int n)
{
		cudaError_t cudaStatus;

		for(int i=0;i<n;i++)
		{
			cudaStatus = cudaStreamDestroy(streams[i]);			
			if(cudaStatus != cudaSuccess)
			{
				fprintf(stderr, "stream creation failed: %s\n", cudaGetErrorString(cudaStatus));
			}
		}
		delete streams;
}
void CF2::zeroProb()
{
	CudaMem::cudaMemsetReport(depthBuffer.d_dx,0, depthBuffer.size*sizeof(float));
	CudaMem::cudaMemsetReport(depthBuffer.d_dy,0, depthBuffer.size*sizeof(float));
	CudaMem::cudaMemsetReport(depthBuffer.d_dz,0, depthBuffer.size*sizeof(float));
	CudaMem::cudaMemsetReport(probResult.d_p, 0, probResult.n*sizeof(float));
	CudaMem::cudaMemsetReport(probResult.d_maxp, 0, probResult.nmax*sizeof(float));

}
//void CF2::normalizeProb()
//{
//	cudaError_t cudaStatus;
//	cuda_calc2::normalizeProb<<<PAR_KERNEL_LAUNCHS,CAM_ITE>>>(probResult.d_maxp, samplePositions.nP);
//
//	cudaStatus = cudaGetLastError();
//	if (cudaStatus != cudaSuccess) {
//		fprintf(stderr, "normalizeProb launch failed: %s\n", cudaGetErrorString(cudaStatus));
//	}
//
//	cudaStatus = cudaDeviceSynchronize();
//	if (cudaStatus != cudaSuccess) {
//		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching normalizeProb!\n", cudaStatus);
//
//	}
//
//	CudaMem::cudaMemCpyReport(probResult.maxp, probResult.d_maxp, probResult.nmax*sizeof(float), cudaMemcpyDeviceToHost);
//}

void CF2::initParallelOptiRuns()
{
	
	size_t avail3;
	size_t total3;
	

	cudaMemGetInfo( &avail3, &total3 );
	double div = (double)avail3/(double)total3;
	double usage =  (1.0-div)*100.0;		
	printf("total memory usage is: %.2f\n",usage);


	int numberOfSessions = MAX_ITE;

	//starting the real number of sessions
	optiSession.n = numberOfSessions;
	optiSession.launchs = new struct RAYTRACING_LAUNCH[numberOfSessions];

	optiSession.pI = new int[numberOfSessions*currentNumberOfCams];
	optiSession.aI = new int[numberOfSessions*currentNumberOfCams];
	





	//starting to initilize the rest of the model	
	int nOfRays = 0;
	for(int ite=0; ite<numberOfSessions; ite++)
	{		
		optiSession.launchs[ite].cams = new SAMPLE_CAMERA *[currentNumberOfCams];
		optiSession.launchs[ite].depthBuffers = new DEPTH_BUFFER[currentNumberOfCams];
		optiSession.launchs[ite].cudaStream = new cudaStream_t*[currentNumberOfCams];	
		optiSession.launchs[ite].aI = new int*[currentNumberOfCams];
		optiSession.launchs[ite].pI = new int*[currentNumberOfCams];
		optiSession.launchs[ite].n = currentNumberOfCams;
		for(int i=0; i<currentNumberOfCams; i++)
		{			
			optiSession.launchs[ite].cams[i] = &sampleCameraTypes.possibleCameraTypes[cameraCombination.vector[i]];
			nOfRays += sampleCameraTypes.possibleCameraTypes[cameraCombination.vector[i]].nRays;
		}
	}


	////init the rest of the launch
	initDepthBuffer(&depthBuffer, nOfRays);	
	initCentroidBuffer(&centroid, numberOfSessions);
	initPropBuffer(&probResult ,sampleFitting.n, numberOfSessions);	

	clearPropBuffer(&probResult,sampleFitting.n*numberOfSessions);
	createCudaStream(&cudaStream, currentNumberOfCams*numberOfSessions);

	//setting all the pointer
	nOfRays = 0;
	int nofProps = 0;

	for(int ite=0; ite<numberOfSessions; ite++)
	{	
		//setting pointer for depth
		optiSession.launchs[ite].depthBuffer.devStates = depthBuffer.devStates + nOfRays;

		optiSession.launchs[ite].depthBuffer.d_dx = depthBuffer.d_dx + nOfRays;
		optiSession.launchs[ite].depthBuffer.d_dy = depthBuffer.d_dy + nOfRays;
		optiSession.launchs[ite].depthBuffer.d_dz = depthBuffer.d_dz + nOfRays;

		optiSession.launchs[ite].centroid.d_cx = centroid.d_cx + ite;
		optiSession.launchs[ite].centroid.d_cy = centroid.d_cy + ite;
		optiSession.launchs[ite].centroid.d_cz = centroid.d_cz + ite;

		optiSession.launchs[ite].probResult.d_p = probResult.d_p + nofProps;
		optiSession.launchs[ite].probResult.d_maxp = probResult.d_maxp+ite;
		optiSession.launchs[ite].probResult.maxp = probResult.maxp+ite;
		optiSession.launchs[ite].probResult.n = sampleFitting.n;

		//init random states for rays
		//
		int raysPerLaunch = 0;
		for(int i=0; i<currentNumberOfCams; i++)
		{
			optiSession.launchs[ite].depthBuffers[i].size = optiSession.launchs[ite].cams[i]->nRays;

			optiSession.launchs[ite].depthBuffers[i].devStates = depthBuffer.devStates + nOfRays;
			optiSession.launchs[ite].depthBuffers[i].d_dx = depthBuffer.d_dx + nOfRays;
			optiSession.launchs[ite].depthBuffers[i].d_dy = depthBuffer.d_dy + nOfRays;
			optiSession.launchs[ite].depthBuffers[i].d_dz = depthBuffer.d_dz + nOfRays;

			optiSession.launchs[ite].aI[i] = optiSession.aI+ite*currentNumberOfCams+i;
			optiSession.launchs[ite].pI[i] = optiSession.pI+ite*currentNumberOfCams+i;

			optiSession.launchs[ite].cudaStream[i] = cudaStream+ite*currentNumberOfCams+i;
			
			initRadomNumberGenerator(optiSession.launchs[ite].depthBuffers[i].devStates, optiSession.launchs[ite].cams[i]);

			nOfRays += optiSession.launchs[ite].cams[i]->nRays;
			raysPerLaunch += optiSession.launchs[ite].cams[i]->nRays;
		}
		optiSession.launchs[ite].depthBuffer.size = raysPerLaunch;
		//setting pointer for launch configuration of multiple sensors
		nofProps += sampleFitting.n;
	}


	

	cudaMemGetInfo( &avail3, &total3 );
	div = (double)avail3/(double)total3;
	usage =  (1.0-div)*100.0;		
	printf("total memory usage is: %.2f\n",usage);

}

void CF2::freeParallelOptiRuns()
{
	size_t avail3;
	size_t total3;

	int numberOfSessions = MAX_ITE;

	for(int ite=0; ite<numberOfSessions; ite++)
	{		
		delete optiSession.launchs[ite].cams;				
		delete optiSession.launchs[ite].depthBuffers;
		delete optiSession.launchs[ite].cudaStream;		
		delete optiSession.launchs[ite].aI;
		delete optiSession.launchs[ite].pI;
	}
	delete optiSession.launchs;
	
	

	////init the rest of the launch
	freeDepthBuffer(&depthBuffer);	
	freeCentroidBuffer(&centroid);
	freePropBuffer(&probResult);	
	freeCudaStream(cudaStream, currentNumberOfCams*numberOfSessions);


	delete optiSession.pI;
	delete optiSession.aI;
	


	cudaMemGetInfo( &avail3, &total3 );
	double div = (double)avail3/(double)total3;
	double usage =  (1.0-div)*100.0;		
	printf("total memory usage is: %.2f\n",usage);


}

void CF2::initVertexBuffer()
{
	//calculating vertexBufferSize
	int vertexSize = robot.nV + human.nV + environment.nV;
	int faceSize = robot.nF + human.nF + environment.nF;
	vertexBuffer.nF = faceSize;
	vertexBuffer.nV = vertexSize;
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
		fb_x[i+robot.nF] = human.fx[i]+robot.nV;
		fb_y[i+robot.nF] = human.fy[i]+robot.nV;
		fb_z[i+robot.nF] = human.fz[i]+robot.nV;
	}

	for(int i=0; i<environment.nF; i++)
	{
		fb_x[i+robot.nF+human.nF] = environment.fx[i]+robot.nV+human.nV;
		fb_y[i+robot.nF+human.nF] = environment.fy[i]+robot.nV+human.nV;
		fb_z[i+robot.nF+human.nF] = environment.fz[i]+robot.nV+human.nV;
	}

	CudaMem::cudaMemCpyReport(vertexBuffer.d_fx, fb_x, faceSize*sizeof(int), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(vertexBuffer.d_fy, fb_y, faceSize*sizeof(int), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(vertexBuffer.d_fz, fb_z, faceSize*sizeof(int), cudaMemcpyHostToDevice);
}

void CF2::transformVertexBuffer()
{

	cudaError_t cudaStatus;
	cuda_calc2::transformVertexRobot<<<robot.nV,1>>>(robot.d_x,
													robot.d_y,
													robot.d_z,
													robot.d_vi,
													currentTrans.d_r,
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
													currentTrans.d_h,
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
													currentTrans.d_e,
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

void CF2::setCurrentTans(int i)
{	
	int transIndex = 0;//rand() % N_SAMPLE_TRANSFORMATION;
	currentTrans.d_r =	samplePositions.d_qr +
						i*N_SAMPLE_TRANSFORMATION*N_ELEMENT_T *NUMELEM_H +
						transIndex*N_ELEMENT_T *NUMELEM_H;

	currentTrans.d_h =  samplePositions.d_qh+
						i*N_ELEMENT_HU *NUMELEM_H;

	currentTrans.d_e =  samplePositions.d_qe+
						i*N_SAMPLE_TRANSFORMATION*N_ELEMENT_EV *NUMELEM_H+
						transIndex*N_ELEMENT_EV *NUMELEM_H;

	currentTrans.d_pr = samplePositions.d_pr+i;
}

void CF2::transformBoundingBoxBuffer()
{
	cudaError_t cudaStatus;		
	cuda_calc2::transformBoundingBoxRobot<<<robot.nBB,1>>>(
													currentTrans.d_r,													
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
													currentTrans.d_e,													
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

void CF2::initDepthBuffer(DEPTH_BUFFER* depthBuffer, int size)
{
	depthBuffer->size = size;
	depthBuffer->dx = new float[size];
	depthBuffer->dy = new float[size];
	depthBuffer->dz = new float[size];
	CudaMem::cudaMemAllocReport((void**)&depthBuffer->d_dx, size*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&depthBuffer->d_dy, size*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&depthBuffer->d_dz, size*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&depthBuffer->devStates, sizeof(curandState) * size);	

}

void CF2::freeDepthBuffer(DEPTH_BUFFER* depthBuffer)
{
	
	delete depthBuffer->dx;
	delete depthBuffer->dy;
	delete depthBuffer->dz;
	CudaMem::cudaFreeReport(depthBuffer->d_dx);
	CudaMem::cudaFreeReport(depthBuffer->d_dy);
	CudaMem::cudaFreeReport(depthBuffer->d_dz);
	CudaMem::cudaFreeReport(depthBuffer->devStates);

}






void CF2::initRadomNumberGenerator(curandState *devStates, SAMPLE_CAMERA* sampleCamera )
{
	cudaError_t cudaStatus;
	cuda_calc2::setup_kernel<<<sampleCamera->nBlocks,sampleCamera->nThreads>>>(devStates);

	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "raytraceVertices launch failed: %s\n", cudaGetErrorString(cudaStatus));
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching raytraceVertices!\n", cudaStatus);

	}
	
}

void CF2::initRaytracingLaunch()
{

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

	int indexPos = 50-1;
	int indexRot = 10-1;

	indexPos = 58;
	indexRot = 370 -1;

	float* samplePosOffet;
	float* sampleRotOffet;

		//__global__ void raytraceVertices(					float* xi, float* yi, float* zi,
		//												int* fx, int* fy, int* fz, int nF,
		//												float* bb_H, float* bb_D, int nBB, 
		//												float* camPos_H, float* camRot_H,
		//												float* camRayX, float* camRayY, float* camRayZ,
		//												float* Dx, float* Dy, float* Dz
	cudaError_t cudaStatus;
	//starting all parallel kernels and wait for finishing

	RAYTRACING_LAUNCH* p_rtl;
	SAMPLE_CAMERA* p_camera;
	for(int i=0; i<optiSession.n; i++)
	{
		p_rtl = &optiSession.launchs[i];
		for(int j=0; j<p_rtl->n; j++)
		{
			//setting position and orientation offset
			//indexPos = optiSession.pI[



			//

			indexPos = *(p_rtl->pI[j]);
			indexRot = *(p_rtl->aI[j]);
			//printf("%d\t%d\n", indexPos, indexRot);
			samplePosOffet = samplePointsBuffer.d_H	+indexPos*NUMELEM_H;
			sampleRotOffet = sampleRotations.d_R	+indexRot*NUMELEM_H;

			p_camera = p_rtl->cams[j];		
			cuda_calc2::raytraceVertices<<<p_camera->nBlocks,p_camera->nThreads, 0, *(p_rtl->cudaStream[j])>>>(
															vertexBuffer.d_vx,
															vertexBuffer.d_vy,
															vertexBuffer.d_vz,																										
															vertexBuffer.d_fx,
															vertexBuffer.d_fy,
															vertexBuffer.d_fz,													
															vertexBuffer.nF,
															boundingBoxBuffer.d_BB,
															boundingBoxBuffer.d_D,
															boundingBoxBuffer.nBB,
															samplePosOffet,
															sampleRotOffet,
															p_camera->d_x,
															p_camera->d_y,
															p_camera->d_z,
															p_rtl->depthBuffers[j].devStates,
															p_camera->d_c,
															p_rtl->depthBuffers[j].d_dx,
															p_rtl->depthBuffers[j].d_dy,
															p_rtl->depthBuffers[j].d_dz);


			cudaStatus = cudaGetLastError();
			if (cudaStatus != cudaSuccess) {
				fprintf(stderr, "raytraceVertices stream %d camera %d launch failed: %s\n", i,j,cudaGetErrorString(cudaStatus));
			}
		}
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

void CF2::transformSamplePointBuffer()
{
	
	cudaError_t cudaStatus;	
	cuda_calc2::transformSamplePoint<<<samplePoints.n,1>>>(
													currentTrans.d_r,													
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

void CF2::initCentroidBuffer(CENTROID* centroid, int n)
{

	centroid->cx = new float[n];
	centroid->cy = new float[n];
	centroid->cz = new float[n];
			

	CudaMem::cudaMemAllocReport((void**)&centroid->d_cx, n*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&centroid->d_cy, n*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&centroid->d_cz, n*sizeof(float));
}

void CF2::freeCentroidBuffer(CENTROID* centroid)
{
	delete centroid->cx;
	delete centroid->cy;
	delete centroid->cz;
			
	CudaMem::cudaFreeReport(centroid->d_cx);
	CudaMem::cudaFreeReport(centroid->d_cy);
	CudaMem::cudaFreeReport(centroid->d_cz);

}

void CF2::calculateCentroid()
{
	cudaError_t cudaStatus;


	RAYTRACING_LAUNCH* p_rtl;
	SAMPLE_CAMERA* p_camera;
	for(int i=0; i<optiSession.n; i++)
	{
		p_rtl = &optiSession.launchs[i];
		for(int j=0; j<p_rtl->n; j++)
		{
			p_camera = p_rtl->cams[j];
			
				//calcMiddlePoint(float* Dx, float* Dy, float* Dz, int nP, float* a_x, float* a_y, float* a_z)			
			cuda_calc2::calcMiddlePoint<<<1,AVG_BUFFER_SIZE, 0, *(p_rtl->cudaStream[j])>>>(
															p_rtl->depthBuffer.d_dx,													
															p_rtl->depthBuffer.d_dy,
															p_rtl->depthBuffer.d_dz,
															p_rtl->depthBuffer.size,
															p_rtl->centroid.d_cx,
															p_rtl->centroid.d_cy,
															p_rtl->centroid.d_cz);





			cudaStatus = cudaGetLastError();
			if (cudaStatus != cudaSuccess) {
				fprintf(stderr, "calcMiddlePoint stream %d camera %d launch failed: %s\n", i,j,cudaGetErrorString(cudaStatus));
			}
		}
	}

			
	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching calcMiddlePoint!\n", cudaStatus);

	}
}

void CF2::calculateProbOfHumanDetection()
{
	cudaError_t cudaStatus;

	
	RAYTRACING_LAUNCH* p_rtl;
	
	for(int i=0; i<optiSession.n; i++)
	{
		p_rtl = &optiSession.launchs[i];								
		cuda_calc2::distanceToEllipseModel<<<launchConfigModelFitting.nblocks,launchConfigModelFitting.nthreads, 0, *(p_rtl->cudaStream[0])>>>(
														p_rtl->depthBuffer.d_dx,													
														p_rtl->depthBuffer.d_dy,
														p_rtl->depthBuffer.d_dz,
														p_rtl->depthBuffer.size,
														sampleFitting.d_R,
														sampleFitting.d_Fx,
														sampleFitting.d_Fy,
														p_rtl->centroid.d_cx,
														p_rtl->centroid.d_cy,
														p_rtl->centroid.d_cz,														
														distMatrix.d_ws_l_params,
														distMatrix.d_ws_n_params,
														distMatrix.d_d,
														p_rtl->probResult.d_p);



		cudaStatus = cudaGetLastError();
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "distanceToEllipseModel stream %d launch failed: %s\n", i,cudaGetErrorString(cudaStatus));
		}
		
	}

	
	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching distanceToEllipseModel!\n", cudaStatus);

	}
}

void CF2::initPropBuffer(PROB_RESULT* probResult, int n, int session)
{
	probResult->n = n*session;
	probResult->nmax = session;

	probResult->p = new float[n*session];
	probResult->maxp = new float[session];

	CudaMem::cudaMemAllocReport((void**)&probResult->d_p, n*session*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&probResult->d_maxp, session*sizeof(float));
}

void CF2::freePropBuffer(PROB_RESULT* probResult)
{
	
	delete probResult->p;
	delete probResult->maxp;
	CudaMem::cudaFreeReport(probResult->d_p);
	CudaMem::cudaFreeReport(probResult->d_maxp);
}

void CF2::clearPropBuffer(PROB_RESULT* probResult, int n)
{
	memset(probResult->p, 0, n*sizeof(float));
	memset(probResult->maxp, 0, sizeof(float));

	CudaMem::cudaMemsetReport(probResult->d_p, 0, n*sizeof(float));
	CudaMem::cudaMemsetReport(probResult->d_maxp, 0, sizeof(float));
}

void CF2::calculateMaxProb()
{

	//calculateMaxProb(float* prob, int n, float* maxp)

	cudaError_t cudaStatus;	

	RAYTRACING_LAUNCH* p_rtl;
	
	for(int i=0; i<optiSession.n; i++)
	{
		p_rtl = &optiSession.launchs[i];
		cuda_calc2::calculateMaxProb<<<1,MAX_BUFFER_SIZE,0, *(p_rtl->cudaStream[0])>>>(
														p_rtl->probResult.d_p,
														p_rtl->probResult.n,
														p_rtl->probResult.d_maxp,
														currentTrans.d_pr);
		

					

		cudaStatus = cudaGetLastError();
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "calculateMaxProb stream %d launch failed: %s\n", i,cudaGetErrorString(cudaStatus));
		}
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching calcMiddlePoint!\n", cudaStatus);

	}

	for(int i=0; i<optiSession.n; i++)
	{
		p_rtl = &optiSession.launchs[i];
		CudaMem::cudaMemCpyReport(p_rtl->probResult.maxp, p_rtl->probResult.d_maxp, sizeof(float), cudaMemcpyDeviceToHost);
	}
	
}
