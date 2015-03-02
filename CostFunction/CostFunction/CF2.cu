#include "CF2.h"

#include "CudaMem.h"


#include <cuda_profiler_api.h>
#include "device_launch_parameters.h"
#include "cuda_runtime.h"
#include "mathcuda.h"
#include "NearestNeighbour.h"
#include "InversionSearch.h"
#include "EC.h"


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
#include "AngleGenerator.h"


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



	////calcDistanceMatrix(float* x, float* y, float* z, int n, int nsample, int nx, int ny, float* dist)
	//float *x = new float[8];
	//float *y = new float[8];
	//float *z = new float[8];
	//float *d = new float[64];
	//int *ci = new int[8];
	//int *cn = new int[MAX_CLUSTER_N_SIZE];
	//int clusterSize = 0;
	//int maxClusterIndice = 0;
	//for(int i=0; i<4; i++)
	//{
	//	if(i==0 || i==1 /*|| i==2*/)
	//	{
	//		x[i] = (float)0;
	//		y[i] = (float)0;
	//		z[i] = (float)0;

	//		x[4+i] = (float)0;
	//		y[4+i] = (float)0;
	//		z[4+i] = (float)0;
	//	}else{
	//		x[i] = (float)i*i;
	//		y[i] = (float)i*i;
	//		z[i] = (float)i*i;

	//		x[4+i] = (float)i*i;
	//		y[4+i] = (float)i*i;
	//		z[4+i] = (float)i*i;
	//	}



	//	


	//}
	//memset(ci, -1, 8*sizeof(int));
	//


	//float *dx, *dy, *dz, *dd;
	//int* dci, *dcs, *dcn, *dmci;
	//CudaMem::cudaMemAllocReport((void**)&dx, 8*sizeof(float));
	//CudaMem::cudaMemAllocReport((void**)&dy, 8*sizeof(float));
	//CudaMem::cudaMemAllocReport((void**)&dz, 8*sizeof(float));
	//CudaMem::cudaMemAllocReport((void**)&dd, 64*sizeof(float));
	//CudaMem::cudaMemAllocReport((void**)&dci, 8*sizeof(int));
	//CudaMem::cudaMemAllocReport((void**)&dcs, 1*sizeof(int));
	//CudaMem::cudaMemAllocReport((void**)&dmci, 1*sizeof(int));
	//CudaMem::cudaMemAllocReport((void**)&dcn, MAX_CLUSTER_N_SIZE*sizeof(int));


	//CudaMem::cudaMemCpyReport(dx,x,8*sizeof(float), cudaMemcpyHostToDevice);
	//CudaMem::cudaMemCpyReport(dy,y,8*sizeof(float), cudaMemcpyHostToDevice);
	//CudaMem::cudaMemCpyReport(dz,z,8*sizeof(float), cudaMemcpyHostToDevice);
	//CudaMem::cudaMemCpyReport(dci,ci,8*sizeof(int), cudaMemcpyHostToDevice);
	//
	//

	//
	//cuda_calc2::calcDistanceMatrix<<<dim3(4,4,1), dim3(1,1,1)>>>(dx, dy, dz, 2, 8, dd);
	//cudaStatus = cudaGetLastError();
	//if (cudaStatus != cudaSuccess) {		
	//	IO::waitForEnter();
	//}


	//cudaStatus = cudaDeviceSynchronize();
	//if (cudaStatus != cudaSuccess) {
	//	IO::waitForEnter();
	//}

	////calcCluster(int* ci, float* dist,  int n, int nsample, int nx, int ny)

	//cuda_calc2::calcCluster<<<1, MAX_CLUSTER_BUFFER_SIZE>>>(dci,dd,4, 8, dcs);
	//cudaStatus = cudaGetLastError();
	//if (cudaStatus != cudaSuccess) {		
	//	IO::waitForEnter();
	//}


	//cudaStatus = cudaDeviceSynchronize();
	//if (cudaStatus != cudaSuccess) {
	//	IO::waitForEnter();
	//}




	//CudaMem::cudaMemCpyReport(d,dd,64*sizeof(float), cudaMemcpyDeviceToHost);
	//CudaMem::cudaMemCpyReport(ci,dci,8*sizeof(int), cudaMemcpyDeviceToHost);
	//CudaMem::cudaMemCpyReport(&clusterSize,dcs,1*sizeof(int), cudaMemcpyDeviceToHost);
	//

	//int i2;
	//for(int xi=0; xi<8; xi++)
	//{
	//	printf("%.2f\t%.2f\t%.2f\n", x[xi], y[xi], z[xi]);
	//}
	//printf("\n");
	//printf("\n");

	//for(int yi=0; yi<8; yi++)
	//{
	//	for(int xi=0; xi<8; xi++)
	//	{
	//		i2 = 8*yi+xi;
	//		printf("%.2f\t", d[i2]);
	//	}
	//	printf("\n");
	//}
	//printf("\n\n\n");
	//
	//printf("cluster size: %d\n\n", clusterSize);
	//for(int xi=0; xi<8; xi++)
	//{
	//	printf("%d\t", ci[xi]);
	//}
	//printf("\n");

	////calcNmembersOfCluster(int* ci, int n, int nsample, int* NmembersOfCluster)
	//cuda_calc2::calcNmembersOfCluster<<<1, MAX_CLUSTER_BUFFER_SIZE>>>(dci, 8, 4, dcn, dmci);
	//cudaStatus = cudaGetLastError();
	//if (cudaStatus != cudaSuccess) {		
	//	IO::waitForEnter();
	//}


	//cudaStatus = cudaDeviceSynchronize();
	//if (cudaStatus != cudaSuccess) {
	//	IO::waitForEnter();
	//}

	//CudaMem::cudaMemCpyReport(cn,dcn,MAX_CLUSTER_N_SIZE*sizeof(int), cudaMemcpyDeviceToHost);
	//CudaMem::cudaMemCpyReport(&maxClusterIndice, dmci, 1*sizeof(int), cudaMemcpyDeviceToHost);
	//for(int xi=0; xi<MAX_CLUSTER_N_SIZE; xi++)
	//{
	//	printf("%d\t", cn[xi]);
	//}
	//printf("max cluster indices %d\n", maxClusterIndice);
	//


	cudaFuncSetCacheConfig(cuda_calc2::raytraceVertices, cudaFuncCachePreferShared);
	cudaFuncSetCacheConfig(cuda_calc2::calcMiddlePoint, cudaFuncCachePreferShared);
	cudaFuncSetCacheConfig(cuda_calc2::distanceToEllipseModel, cudaFuncCachePreferShared);

	//init random number generator
	srand (time(NULL));
	cameraCombination.vector = NULL;



	
	IO::loadPCL(&robot,"robot_short.bin");
	IO::loadPCL(&human,"human.bin");
	IO::loadPCL(&environment, "environment.bin");

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

	
#ifndef NDEBUG
	IO::saveVerticeBufferToFile(&vertexBufferRobot, "vertexRobot.bin");
	IO::saveVerticeBufferToFile(&vertexBufferHuman, "vertexHuman.bin");
	IO::saveVerticeBufferToFile(&vertexBufferEnvironment, "vertexEnvironment.bin");
	IO::saveVerticeBufferToFile(&vertexBuffer, "vertexScene.bin");
	IO::saveBoundingBoxBufferToFile(&boundingBoxBuffer, "boundingBox.bin");
	IO::saveBoundingBoxBuffer(&boundingBoxBuffer, "boundingBoxBuffer.bin");
#endif

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
	cameraCombination.gen_result = gen_comb_rep_lex_next(cameraCombination.vector, sampleCameraTypes.nCameraTypes, currentNumberOfCams);
}

void CF2::initCameraCombination()
{
	if(cameraCombination.vector != NULL) delete cameraCombination.vector;

	cameraCombination.vector = new unsigned long long[currentNumberOfCams];
	cameraCombination.gen_result = gen_comb_rep_lex_init(cameraCombination.vector, sampleCameraTypes.nCameraTypes, currentNumberOfCams);
}

void CF2::run()
{
	//outer loop for combination of multiple cameras
	initCameraCombination();

	//camera specific allocation
	initParallelOptiRuns();
	IO::waitForEnter();
	//////////finding first the probability density funtion of the angles to reduces the amount of raytracing angles
	sC = new InversionSearch(&samplePoints, &sampleRotations, 1, MAX_ITE, nn->getNN());
	((InversionSearch*)sC)->setInversionParamters(&samplePointsBuffer);

	while(sC->iterate(optiSession.pI, optiSession.aI, probResult.maxp, probResult.maxd, probResult.maxw) )
	{	
		zeroProb();
		for(int i=0; i< samplePositions.nP; i++)
		{				

			setCurrentTans(i);
			transformVertexBuffer();
			transformBoundingBoxBuffer();
			transformSamplePointBuffer();

			rayTrace();
			IO::saveDepthBufferToFile(&depthBuffer, "depthBuffer.bin");
			calculateCluster();
			//printCentroid(&depthBuffer);

			calculateCentroid();
			calculateProbOfHumanDetection();
			calculateMaxProb();	
			//checkIntermediateResults();
		

		}
		printf("angle initializing....\n");
	}	
	IO::saveInversionSearch(sC->prop, sC->dist, sC->weights, SEARCH_DOF*sampleRotations.nRotations, "inversionSearch.bin");
	//IO::waitForEnter();

	int n;
	IO::loadInversionSearch(sC->prop, sC->dist, sC->weights, &n, "inversionSearch.bin");
	AngleGenerator aG(sC->prop, sampleRotations.nRotations, SEARCH_DOF);
	delete sC;
	freeParallelOptiRuns();
	printf("starting optimisation...\n");
	//IO::waitForEnter();
	
	while(currentNumberOfCams < 4)
	{
	
		do
		{
			//sC = new CompleteEnumeration(&samplePoints, &sampleRotations, currentNumberOfCams, MAX_ITE, NULL);
			sC = new SimulatedAnnealing(&samplePoints, &sampleRotations, currentNumberOfCams, MAX_ITE, nn->getNN(), &aG);

			initParallelOptiRuns();			
			time_t start;			
			time(&start);
			//if (currentNumberOfCams == 2)
			//{
				while(sC->iterate(optiSession.pI, optiSession.aI, probResult.maxp, probResult.maxd, probResult.maxw) )
				{
					zeroProb();
					for(int i=0; i< samplePositions.nP; i++)
					{				
						setCurrentTans(i);
						transformVertexBuffer();
						transformBoundingBoxBuffer();
						transformSamplePointBuffer();

						rayTrace();
						//IO::saveDepthBufferToFile(&depthBuffer, "depthBuffer.bin");
						calculateCluster();
						calculateCentroid();
						//printCentroid(&depthBuffer);
						calculateProbOfHumanDetection();
						calculateMaxProb();
						//Progress::printProgress((double)i, (double)samplePositions.nP, start, "raytracing rp ");	
					}
					//IO::saveDepthBufferToFile(&depthBuffer, "depthBuffer.bin");
					//IO::saveDepthBufferToFileSuperSamples(&depthBuffer, "depthBuffer.bin");
					//Progress::printProgress((double)optiSession.pI[0], (double)samplePoints.n, start, "raytracing cp ");
					//IO::waitForEnter();

				}
			//}

	
			//saving results
			//saveAllVertices();
			//setCurrentTans(0);
			//transformSamplePointBuffer();
			//IO::saveOptimisationResults(&samplePointsBuffer, &samplePoints, &sampleRotations, sC->prop, sC->dist,sC->weights,  "completeEnumeration.bin");
			sC->writeResultsToFile(cameraCombination.vector, currentNumberOfCams, &samplePointsBuffer);
			delete sC;
			freeParallelOptiRuns();

			printf("finished current camera combination\n");

			//IO::waitForEnter();
			
			iterateCameraCombination();
		}while(cameraCombination.gen_result == GEN_NEXT);

		printf("adding another camera\n");
		//IO::waitForEnter();

		currentNumberOfCams++;
		initCameraCombination();
	}
	IO::waitForEnter();

}

void CF2::run_completeEnumeration()
{
		//outer loop for combination of multiple cameras
	initCameraCombination();
	sC = new CompleteEnumeration(&samplePoints, &sampleRotations, currentNumberOfCams, MAX_ITE, NULL);		
	initParallelOptiRuns();		
	printf("starting optimisation...\n");
	IO::waitForEnter();
	time_t start;			
	time(&start);

	while(sC->iterate(optiSession.pI, optiSession.aI, probResult.maxp, probResult.maxd, probResult.maxw) )
	{
		zeroProb();
		for(int i=0; i< samplePositions.nP; i++)
		{				
			setCurrentTans(i);
			transformVertexBuffer();
			transformBoundingBoxBuffer();
			transformSamplePointBuffer();

			rayTrace();
			//IO::saveDepthBufferToFile(&depthBuffer, "depthBuffer.bin");
			calculateCluster();
			calculateCentroid();
			//printCentroid(&depthBuffer);
			calculateProbOfHumanDetection();
			calculateMaxProb();
			//Progress::printProgress((double)i, (double)samplePositions.nP, start, "raytracing rp ");	
		}
		//IO::saveDepthBufferToFile(&depthBuffer, "depthBuffer.bin");
		//IO::saveDepthBufferToFileSuperSamples(&depthBuffer, "depthBuffer.bin");
		Progress::printProgress((double)optiSession.pI[0], (double)samplePoints.n, start, "raytracing cp ");
		//IO::waitForEnter();

	}
	

	
	//saving results	
	setCurrentTans(0);
	transformSamplePointBuffer();
	IO::saveOptimisationResults(&samplePointsBuffer, &samplePoints, &sampleRotations, sC->prop, sC->dist,sC->weights,  "completeEnumeration.bin");
	sC->writeResultsToFile(cameraCombination.vector, currentNumberOfCams, &samplePointsBuffer);
	delete sC;
	freeParallelOptiRuns();

	printf("finished current camera combination\n");

	//IO::waitForEnter();
			
	iterateCameraCombination();

	IO::waitForEnter();
}

void CF2::checkIntermediateResults()
{


	RAYTRACING_LAUNCH* p_rtl;

	for(int j=0; j<optiSession.n; j++)
	{
		p_rtl = &optiSession.launchs[j];	

		float* p = new float[p_rtl->probResult.n];
		float* d = new float[p_rtl->probResult.n];
		float maxp, mind;

		CudaMem::cudaMemCpyReport(p, p_rtl->probResult.d_p, p_rtl->probResult.n*sizeof(float), cudaMemcpyDeviceToHost);
		CudaMem::cudaMemCpyReport(d, p_rtl->probResult.d_d, p_rtl->probResult.n*sizeof(float), cudaMemcpyDeviceToHost);
		CudaMem::cudaMemCpyReport(&maxp, p_rtl->probResult.d_maxp, sizeof(float), cudaMemcpyDeviceToHost);
		CudaMem::cudaMemCpyReport(&mind, p_rtl->probResult.d_maxd, sizeof(float), cudaMemcpyDeviceToHost);

		float maxp2 = 0.0f, mind2= FLT_MAX;
		for(int i=0;i<p_rtl->probResult.n; i++)
		{
			maxp2 = std::max(maxp2,p[i]);
			//mind2 = std::min(mind2,d[i]);
		}
		float eps=1e-5;
		float diff_p = abs(maxp-maxp2);
		//float diff_d = abs(mind-mind2);
		printf("j=%d: maxp cuda %.10f\tmaxp host %.10f\n", j, maxp, maxp2);
//#ifndef NDEBUG
//		if(diff_p >= eps)
//		{
//			printf("diff p: %.10f\t iteration: %d\n", diff_p, j);
//		}
//#endif
		//assert(diff_p < eps);
		//assert(diff_d < eps);
		delete p;
		delete d;
	}
}

void CF2::initBoundingBoxBuffer()
{
	boundingBoxBuffer.nBB = robot.nBB+environment.nBB+human.nBB;
	humanBB = robot.nBB+environment.nBB;

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

	boundingBoxBufferHuman.nBB = human.nBB;
	boundingBoxBufferHuman.d_BB = boundingBoxBuffer.d_BB+ (boundingBoxBufferRobot.nBB +boundingBoxBufferEnvironment.nBB) * NUMELEM_H;
	boundingBoxBufferHuman.d_D = boundingBoxBuffer.d_D+ (boundingBoxBufferRobot.nBB +boundingBoxBufferEnvironment.nBB) * 3;


	//CudaMem::cudaMemCpyReport(boundingBoxBufferRobot.d_BB, robot.d_bb_H, boundingBoxBufferRobot.nBB*NUMELEM_H*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(boundingBoxBufferRobot.d_D, robot.d_bb_D, boundingBoxBufferRobot.nBB*3*sizeof(float), cudaMemcpyDeviceToDevice);

	//CudaMem::cudaMemCpyReport(boundingBoxBufferEnvironment.d_BB, environment.d_bb_H, boundingBoxBufferEnvironment.nBB*NUMELEM_H*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(boundingBoxBufferEnvironment.d_D, environment.d_bb_D, boundingBoxBufferEnvironment.nBB*3*sizeof(float), cudaMemcpyDeviceToDevice);

		//CudaMem::cudaMemCpyReport(boundingBoxBufferEnvironment.d_BB, environment.d_bb_H, boundingBoxBufferEnvironment.nBB*NUMELEM_H*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(boundingBoxBufferHuman.d_D, human.d_bb_D, boundingBoxBufferHuman.nBB*3*sizeof(float), cudaMemcpyDeviceToDevice);

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
				IO::waitForEnter();
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
				IO::waitForEnter();
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



	//starting the real number of sessions
	optiSession.n = MAX_ITE;
	optiSession.launchs = new struct RAYTRACING_LAUNCH[optiSession.n];

	optiSession.pI = new int[optiSession.n*currentNumberOfCams];
	optiSession.aI = new int[optiSession.n*currentNumberOfCams];
	optiSession.ecs = new EC*[optiSession.n];
	





	//starting to initilize the rest of the model	
	int nOfRays = 0;
	int nOfSSRays = 0;
	int raysPerLaunch = 0;
	int raysSSPerLaunch = 0;
	for(int ite=0; ite<optiSession.n; ite++)
	{		
		optiSession.launchs[ite].cams = new SAMPLE_CAMERA *[currentNumberOfCams];
		optiSession.launchs[ite].depthBuffers = new DEPTH_BUFFER[currentNumberOfCams];
		optiSession.launchs[ite].cudaStream = new cudaStream_t*[currentNumberOfCams];	
		optiSession.launchs[ite].aI = new int*[currentNumberOfCams];
		optiSession.launchs[ite].pI = new int*[currentNumberOfCams];
		optiSession.launchs[ite].n = currentNumberOfCams;
		raysPerLaunch = 0;
		raysSSPerLaunch = 0;

		for(int i=0; i<currentNumberOfCams; i++)
		{			
			optiSession.launchs[ite].cams[i] = &sampleCameraTypes.possibleCameraTypes[cameraCombination.vector[i]];
			nOfRays += sampleCameraTypes.possibleCameraTypes[cameraCombination.vector[i]].nRays;
			nOfSSRays += sampleCameraTypes.possibleCameraTypes[cameraCombination.vector[i]].ssnRays;

			raysPerLaunch += optiSession.launchs[ite].cams[i]->nRays;
			raysSSPerLaunch += optiSession.launchs[ite].cams[i]->ssnRays;
		}

	}


	////init the rest of the launch
	initDepthBuffer(&depthBuffer,optiSession.n, currentNumberOfCams, nOfRays, nOfSSRays);	
	initCentroidBuffer(&centroid, optiSession.n);
	initPropBuffer(&probResult ,sampleFitting.n, optiSession.n);	

	clearPropBuffer(&probResult,sampleFitting.n*optiSession.n);
	createCudaStream(&cudaStream, currentNumberOfCams*optiSession.n);

	//setting all the pointer
	nOfRays = 0;
	nOfSSRays = 0;
	int nofProps = 0;

	for(int ite=0; ite<optiSession.n; ite++)
	{	

		
		//setting pointer for depth
		optiSession.launchs[ite].depthBuffer.devStates = depthBuffer.devStates + nOfRays;

		optiSession.launchs[ite].depthBuffer.d_dx = depthBuffer.d_dx + nOfRays;
		optiSession.launchs[ite].depthBuffer.d_dy = depthBuffer.d_dy + nOfRays;
		optiSession.launchs[ite].depthBuffer.d_dz = depthBuffer.d_dz + nOfRays;

		optiSession.launchs[ite].depthBuffer.dx = depthBuffer.dx + nOfRays;
		optiSession.launchs[ite].depthBuffer.dy = depthBuffer.dy + nOfRays;
		optiSession.launchs[ite].depthBuffer.dz = depthBuffer.dz + nOfRays;

		optiSession.launchs[ite].depthBuffer.hp = depthBuffer.hp + nOfRays;	

		optiSession.launchs[ite].depthBuffer.cis = depthBuffer.cis + nOfRays;		
		optiSession.launchs[ite].depthBuffer.d_cis = depthBuffer.d_cis + nOfRays;


		optiSession.launchs[ite].depthBuffer.d_ss_dx = depthBuffer.d_ss_dx + nOfSSRays;
		optiSession.launchs[ite].depthBuffer.d_ss_dy = depthBuffer.d_ss_dy + nOfSSRays;
		optiSession.launchs[ite].depthBuffer.d_ss_dz = depthBuffer.d_ss_dz + nOfSSRays;

		optiSession.launchs[ite].centroid.d_cx = centroid.d_cx + ite;
		optiSession.launchs[ite].centroid.d_cy = centroid.d_cy + ite;
		optiSession.launchs[ite].centroid.d_cz = centroid.d_cz + ite;

		optiSession.launchs[ite].probResult.p = probResult.p + nofProps;
		optiSession.launchs[ite].probResult.d_p = probResult.d_p + nofProps;

		optiSession.launchs[ite].probResult.d = probResult.d + nofProps;
		optiSession.launchs[ite].probResult.d_d = probResult.d_d + nofProps;

		optiSession.launchs[ite].probResult.w = probResult.w + nofProps;
		optiSession.launchs[ite].probResult.d_w = probResult.d_w + nofProps;

		optiSession.launchs[ite].probResult.d_maxp = probResult.d_maxp+ite;
		optiSession.launchs[ite].probResult.maxp = probResult.maxp+ite;

		optiSession.launchs[ite].probResult.d_maxd = probResult.d_maxd+ite;
		optiSession.launchs[ite].probResult.maxd = probResult.maxd+ite;

		optiSession.launchs[ite].probResult.d_maxw = probResult.d_maxw+ite;
		optiSession.launchs[ite].probResult.maxw = probResult.maxw+ite;

		optiSession.launchs[ite].probResult.n = sampleFitting.n;

		//init random states for rays
		//
		raysPerLaunch = 0;
		raysSSPerLaunch = 0;
		for(int i=0; i<currentNumberOfCams; i++)
		{
			optiSession.launchs[ite].depthBuffers[i].size = optiSession.launchs[ite].cams[i]->nRays;

			optiSession.launchs[ite].depthBuffers[i].devStates = depthBuffer.devStates + nOfRays;
			optiSession.launchs[ite].depthBuffers[i].d_dx = depthBuffer.d_dx + nOfRays;
			optiSession.launchs[ite].depthBuffers[i].d_dy = depthBuffer.d_dy + nOfRays;
			optiSession.launchs[ite].depthBuffers[i].d_dz = depthBuffer.d_dz + nOfRays;

			optiSession.launchs[ite].depthBuffers[i].d_bb_hit = depthBuffer.d_bb_hit+(currentNumberOfCams*ite+i)*boundingBoxBuffer.nBB;
			optiSession.launchs[ite].depthBuffers[i].bb_hit = depthBuffer.bb_hit+(currentNumberOfCams*ite+i)*boundingBoxBuffer.nBB;

			optiSession.launchs[ite].depthBuffers[i].d_ss_dx = depthBuffer.d_ss_dx + nOfSSRays;
			optiSession.launchs[ite].depthBuffers[i].d_ss_dy = depthBuffer.d_ss_dy + nOfSSRays;
			optiSession.launchs[ite].depthBuffers[i].d_ss_dz = depthBuffer.d_ss_dz + nOfSSRays;

			optiSession.launchs[ite].aI[i] = optiSession.aI+ite*currentNumberOfCams+i;
			optiSession.launchs[ite].pI[i] = optiSession.pI+ite*currentNumberOfCams+i;

			optiSession.launchs[ite].cudaStream[i] = cudaStream+ite*currentNumberOfCams+i;
			
			initRadomNumberGenerator(optiSession.launchs[ite].depthBuffers[i].devStates, optiSession.launchs[ite].cams[i]);

			nOfRays += optiSession.launchs[ite].cams[i]->nRays;
			nOfSSRays += optiSession.launchs[ite].cams[i]->ssnRays;

			raysPerLaunch += optiSession.launchs[ite].cams[i]->nRays;
			raysSSPerLaunch += optiSession.launchs[ite].cams[i]->ssnRays;
		}
		optiSession.launchs[ite].depthBuffer.size = raysPerLaunch;
		optiSession.launchs[ite].depthBuffer.sssize = raysSSPerLaunch;


		
	

		optiSession.ecs[ite] = new EC(optiSession.launchs[ite].depthBuffer.dx,
								optiSession.launchs[ite].depthBuffer.dy,
								optiSession.launchs[ite].depthBuffer.dz,
								optiSession.launchs[ite].depthBuffer.cis,
								optiSession.launchs[ite].depthBuffer.hp,
								optiSession.launchs[ite].depthBuffer.size);



		//setting pointer for launch configuration of multiple sensors
		nofProps += sampleFitting.n;
	}

	//init launch configurations
	int nblocks;
	if (optiSession.launchs[0].depthBuffer.size % DIST_MATRIX_BLOCK_X_SIZE == 0)
	{
		nblocks = optiSession.launchs[0].depthBuffer.size/DIST_MATRIX_BLOCK_X_SIZE;
	}else{
		nblocks = (int)(optiSession.launchs[0].depthBuffer.size/DIST_MATRIX_BLOCK_X_SIZE)+1;
	}
	launchConfigDistanceMatrix.nthreads = DIST_MATRIX_BLOCK_X_SIZE;
	//finding ggt until block size is 10 or less
	int di = 200;
	do{
		if(nblocks % di == 0 && nblocks/di <= 10)
		{
			//possible configuration
			launchConfigDistanceMatrix.nblocks = nblocks/di;
			launchConfigDistanceMatrix.nsample = di;
		}
		di--;
	}while(di > 0);
	
	int nsample;
	if (optiSession.launchs[0].depthBuffer.size % MAX_CLUSTER_BUFFER_SIZE == 0)
	{
		nsample = optiSession.launchs[0].depthBuffer.size/MAX_CLUSTER_BUFFER_SIZE;
	}else{
		nsample = (int)(optiSession.launchs[0].depthBuffer.size/MAX_CLUSTER_BUFFER_SIZE)+1;
	}

	launchConfigMaxCluster.nblocks = 1;
	launchConfigMaxCluster.nthreads = MAX_CLUSTER_BUFFER_SIZE;
	launchConfigMaxCluster.nsample = nsample;


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
		delete optiSession.ecs[ite];
	}
	delete optiSession.launchs;
	
	
	

	////init the rest of the launch
	freeDepthBuffer(&depthBuffer);	
	freeCentroidBuffer(&centroid);
	freePropBuffer(&probResult);	
	freeCudaStream(cudaStream, currentNumberOfCams*numberOfSessions);


	delete optiSession.pI;
	delete optiSession.aI;
	delete optiSession.ecs;


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
	CudaMem::cudaMemAllocReport((void**)&vertexBuffer.d_f_bbi, faceSize*sizeof(int));

	//setting up face buffer for complete mesh
	int* fb_x = new int[faceSize];
	int* fb_y = new int[faceSize];
	int* fb_z = new int[faceSize];
	int* fb_bbi = new int[faceSize];
	
	//setting vertexBufferPointer
	vertexBufferRobot.d_vx = vertexBuffer.d_vx;
	vertexBufferRobot.d_vy = vertexBuffer.d_vy;
	vertexBufferRobot.d_vz = vertexBuffer.d_vz;

	vertexBufferRobot.d_fx = vertexBuffer.d_fx;
	vertexBufferRobot.d_fy = vertexBuffer.d_fy;
	vertexBufferRobot.d_fz = vertexBuffer.d_fz;
	vertexBufferRobot.d_f_bbi = vertexBuffer.d_f_bbi;

	vertexBufferRobot.nF = robot.nF;
	vertexBufferRobot.nV = robot.nV;

	vertexBufferHuman.d_vx = vertexBuffer.d_vx+robot.nV;
	vertexBufferHuman.d_vy = vertexBuffer.d_vy+robot.nV;
	vertexBufferHuman.d_vz = vertexBuffer.d_vz+robot.nV;

	vertexBufferHuman.d_fx = vertexBuffer.d_fx+robot.nF;
	vertexBufferHuman.d_fy = vertexBuffer.d_fy+robot.nF;
	vertexBufferHuman.d_fz = vertexBuffer.d_fz+robot.nF;
	vertexBufferHuman.d_f_bbi = vertexBuffer.d_f_bbi+robot.nF;

	vertexBufferHuman.nF = human.nF;
	vertexBufferHuman.nV = human.nV;

	vertexBufferEnvironment.d_vx = vertexBuffer.d_vx+robot.nV+human.nV;
	vertexBufferEnvironment.d_vy = vertexBuffer.d_vy+robot.nV+human.nV;
	vertexBufferEnvironment.d_vz = vertexBuffer.d_vz+robot.nV+human.nV;

	vertexBufferEnvironment.d_fx = vertexBuffer.d_fx+robot.nF+human.nF;
	vertexBufferEnvironment.d_fy = vertexBuffer.d_fy+robot.nF+human.nF;
	vertexBufferEnvironment.d_fz = vertexBuffer.d_fz+robot.nF+human.nF;
	vertexBufferEnvironment.d_f_bbi = vertexBuffer.d_f_bbi+robot.nF+human.nF;

	vertexBufferEnvironment.nF = environment.nF;
	vertexBufferEnvironment.nV = environment.nV;

	memcpy(fb_x, robot.fx, robot.nF*sizeof(int));
	memcpy(fb_y, robot.fy, robot.nF*sizeof(int));
	memcpy(fb_z, robot.fz, robot.nF*sizeof(int));
	memcpy(fb_bbi, robot.f_bbi, robot.nF*sizeof(int));

	for(int i=0; i<human.nF; i++)
	{
		fb_x[i+robot.nF] = human.fx[i]+robot.nV;
		fb_y[i+robot.nF] = human.fy[i]+robot.nV;
		fb_z[i+robot.nF] = human.fz[i]+robot.nV;

		fb_bbi[i+robot.nF] = human.f_bbi[i]+robot.nBB;
	}

	for(int i=0; i<environment.nF; i++)
	{
		fb_x[i+robot.nF+human.nF] = environment.fx[i]+robot.nV+human.nV;
		fb_y[i+robot.nF+human.nF] = environment.fy[i]+robot.nV+human.nV;
		fb_z[i+robot.nF+human.nF] = environment.fz[i]+robot.nV+human.nV;
		fb_bbi[i+robot.nF+human.nF] = environment.f_bbi[i]+robot.nBB+human.nBB;
	}

	CudaMem::cudaMemCpyReport(vertexBuffer.d_fx, fb_x, faceSize*sizeof(int), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(vertexBuffer.d_fy, fb_y, faceSize*sizeof(int), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(vertexBuffer.d_fz, fb_z, faceSize*sizeof(int), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(vertexBuffer.d_f_bbi, fb_bbi, faceSize*sizeof(int), cudaMemcpyHostToDevice);

	delete fb_x;
	delete fb_y;
	delete fb_z;
	delete fb_bbi;
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
		IO::waitForEnter();
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching transformVertexRobot!\n", cudaStatus);
		IO::waitForEnter();
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
		IO::waitForEnter();
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching transformVertexHumanOrEnvironment!\n", cudaStatus);
		IO::waitForEnter();

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
		IO::waitForEnter();
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching transformVertexHumanOrEnvironment!\n", cudaStatus);
		IO::waitForEnter();
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
	cuda_calc2::transformBoundingBoxEnvironment<<<human.nBB,1>>>(
													currentTrans.d_h,													
													human.d_bb_H,
													boundingBoxBufferHuman.d_BB);

					

	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "transformVertexHuman launch failed: %s\n", cudaGetErrorString(cudaStatus));
		IO::waitForEnter();
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching transformVertexHuman!\n", cudaStatus);
		IO::waitForEnter();
	}


	cuda_calc2::transformBoundingBoxRobot<<<robot.nBB,1>>>(
													currentTrans.d_r,													
													robot.d_bbi,
													robot.d_bb_H,
													boundingBoxBufferRobot.d_BB);

					

	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "transformVertexRobot launch failed: %s\n", cudaGetErrorString(cudaStatus));
		IO::waitForEnter();
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching transformVertexRobot!\n", cudaStatus);
		IO::waitForEnter();
	}

	//(float* hi_robot, float* h_bb, float* h_inv)
	cuda_calc2::transformBoundingBoxEnvironment<<<environment.nBB,1>>>(
													currentTrans.d_e,													
													environment.d_bb_H,
													boundingBoxBufferEnvironment.d_BB);

					

	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "transformVertexRobot launch failed: %s\n", cudaGetErrorString(cudaStatus));
		IO::waitForEnter();
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching transformVertexRobot!\n", cudaStatus);
		IO::waitForEnter();
	}




}

void CF2::initDepthBuffer(DEPTH_BUFFER* depthBuffer,int nSessions, int nCams, int size, int ss_size)
{
	depthBuffer->size = size;
	depthBuffer->sssize = ss_size;

	depthBuffer->dx = new float[size];
	depthBuffer->dy = new float[size];
	depthBuffer->dz = new float[size];
	
	depthBuffer->hp = new bool[size];
	depthBuffer->cis = new int[size];
	depthBuffer->bb_hit = new bool[nSessions*nCams*boundingBoxBuffer.nBB];

	

	CudaMem::cudaMemAllocReport((void**)&depthBuffer->d_dx, size*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&depthBuffer->d_dy, size*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&depthBuffer->d_dz, size*sizeof(float));
	
	CudaMem::cudaMemAllocReport((void**)&depthBuffer->d_bb_hit, nSessions*nCams*boundingBoxBuffer.nBB*sizeof(bool));
	//square size of distance matrix
	

	CudaMem::cudaMemAllocReport((void**)&depthBuffer->d_cis, size*sizeof(int));
	

	CudaMem::cudaMemAllocReport((void**)&depthBuffer->d_ss_dx, ss_size*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&depthBuffer->d_ss_dy, ss_size*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&depthBuffer->d_ss_dz, ss_size*sizeof(float));

	CudaMem::cudaMemAllocReport((void**)&depthBuffer->devStates, sizeof(curandState) * size);	

}

void CF2::freeDepthBuffer(DEPTH_BUFFER* depthBuffer)
{
	
	delete depthBuffer->dx;
	delete depthBuffer->dy;
	delete depthBuffer->dz;	
	delete depthBuffer->hp;	
	delete depthBuffer->cis;
	delete depthBuffer->bb_hit;
	
	CudaMem::cudaFreeReport(depthBuffer->d_dx);
	CudaMem::cudaFreeReport(depthBuffer->d_dy);
	CudaMem::cudaFreeReport(depthBuffer->d_dz);		

	CudaMem::cudaFreeReport(depthBuffer->d_cis);

	CudaMem::cudaFreeReport(depthBuffer->d_ss_dx);
	CudaMem::cudaFreeReport(depthBuffer->d_ss_dy);
	CudaMem::cudaFreeReport(depthBuffer->d_ss_dz);
	CudaMem::cudaFreeReport(depthBuffer->d_bb_hit);

	CudaMem::cudaFreeReport(depthBuffer->devStates);

}






void CF2::initRadomNumberGenerator(curandState *devStates, SAMPLE_CAMERA* sampleCamera )
{
	cudaError_t cudaStatus;
	cuda_calc2::setup_kernel<<<sampleCamera->nBlocks,sampleCamera->nThreads>>>(devStates);

	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "raytraceVertices launch failed: %s\n", cudaGetErrorString(cudaStatus));
		IO::waitForEnter();
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching raytraceVertices!\n", cudaStatus);
		IO::waitForEnter();
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

	indexPos = 16;
	indexRot = 196;

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


			indexPos = *(p_rtl->pI[j]);
			indexRot = *(p_rtl->aI[j]);
			
			samplePosOffet = samplePointsBuffer.d_H	+indexPos*NUMELEM_H;
			sampleRotOffet = sampleRotations.d_R	+indexRot*NUMELEM_H;

			p_camera = p_rtl->cams[j];		
			//raytraceBox(float* camPos_H, float* camRot_H, float* camRayX, float* camRayY, float* camRayZ, int nRays,
			//			   float* bb_H, float* bb_D, int nBB, bool* bb_intersect)

			
			
			cuda_calc2::raytraceBox<<<1,MAX_RAYTRACE_BOX, 0, *(p_rtl->cudaStream[j])>>>(
															samplePosOffet,
															sampleRotOffet,
															p_camera->d_x,
															p_camera->d_y,
															p_camera->d_z,
															p_camera->nRays,
															boundingBoxBuffer.d_BB,
															boundingBoxBuffer.d_D,
															boundingBoxBuffer.nBB,
															p_rtl->depthBuffers[j].d_bb_hit
														);


			cudaStatus = cudaGetLastError();
			if (cudaStatus != cudaSuccess) {
				fprintf(stderr, "raytraceBox stream %d camera %d launch failed: %s\n", i,j,cudaGetErrorString(cudaStatus));
				IO::waitForEnter();
			}
		}
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code after launching raytraceBox: %s\n", cudaGetErrorString(cudaStatus));
		fprintf(stderr, "position: %d\t rotation: %d\n", indexPos, indexRot);
		IO::waitForEnter();

	}

	CudaMem::cudaMemCpyReport(depthBuffer.bb_hit, depthBuffer.d_bb_hit, optiSession.n*currentNumberOfCams*boundingBoxBuffer.nBB*sizeof(bool), cudaMemcpyDeviceToHost);
	//printf("human bonding box hit: %s", depthBuffer.bb_hit[humanBB]?"true":"false");

	for(int i=0; i<optiSession.n; i++)
	{
		p_rtl = &optiSession.launchs[i];
		for(int j=0; j<p_rtl->n; j++)
		{
			//setting position and orientation offset
			//indexPos = optiSession.pI[


			//if(!p_rtl->depthBuffers[j].bb_hit[humanBB])
			//	continue;

			//

			indexPos = *(p_rtl->pI[j]);
			indexRot = *(p_rtl->aI[j]);
			//printf("%d\t%d\n", indexPos, indexRot);
			samplePosOffet = samplePointsBuffer.d_H	+indexPos*NUMELEM_H;
			sampleRotOffet = sampleRotations.d_R	+indexRot*NUMELEM_H;

			

			p_camera = p_rtl->cams[j];		
			cuda_calc2::raytraceVertices<<<p_camera->ssnBlocks,p_camera->ssnThreads, 0, *(p_rtl->cudaStream[j])>>>(
															vertexBuffer.d_vx,
															vertexBuffer.d_vy,
															vertexBuffer.d_vz,																										
															vertexBuffer.d_fx,
															vertexBuffer.d_fy,
															vertexBuffer.d_fz,													
															vertexBuffer.nF,
															p_camera->d[0],
															p_camera->d[1],
															samplePosOffet,
															sampleRotOffet,
															p_camera->d_ss_x,
															p_camera->d_ss_y,
															p_camera->d_ss_z,
															p_rtl->depthBuffers[j].d_ss_dx,
															p_rtl->depthBuffers[j].d_ss_dy,
															p_rtl->depthBuffers[j].d_ss_dz,
															vertexBuffer.d_f_bbi,
															p_rtl->depthBuffers[j].d_bb_hit,
															boundingBoxBuffer.nBB,
															humanBB);


			cudaStatus = cudaGetLastError();
			if (cudaStatus != cudaSuccess) {
				fprintf(stderr, "raytraceVertices stream %d camera %d launch failed: %s\n", i,j,cudaGetErrorString(cudaStatus));
				IO::waitForEnter();
			}
		}
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code after launching raytraceVertices: %s\n", cudaGetErrorString(cudaStatus));
		fprintf(stderr, "position: %d\t rotation: %d\n", indexPos, indexRot);
		IO::waitForEnter();

	}


	for(int i=0; i<optiSession.n; i++)
	{
		p_rtl = &optiSession.launchs[i];
		for(int j=0; j<p_rtl->n; j++)
		{


			
	//mergeSuperSampling(float* s_dx, float* s_dy, float* s_dz,
	//									int ss_x, int ss_y,
	//									int nx, int ny,
	//									int minW, 
	//									float* bb_H, float* bb_D, int nBB,
	//									float* camPos_H, float* camRot_H,
	//									float* camRayX, float* camRayY, float* camRayZ,
	//									float* c, curandState_t* devStates,
	//									float* dx, float* dy, float* dz)



			//

			//if(!p_rtl->depthBuffers[j].bb_hit[humanBB])
			//	continue;

			indexPos = *(p_rtl->pI[j]);
			indexRot = *(p_rtl->aI[j]);

			samplePosOffet = samplePointsBuffer.d_H	+indexPos*NUMELEM_H;
			sampleRotOffet = sampleRotations.d_R	+indexRot*NUMELEM_H;

			

			p_camera = p_rtl->cams[j];		
			cuda_calc2::mergeSuperSampling<<<p_camera->nBlocks,p_camera->nThreads, 0, *(p_rtl->cudaStream[j])>>>(
															p_rtl->depthBuffers[j].d_ss_dx,
															p_rtl->depthBuffers[j].d_ss_dy,
															p_rtl->depthBuffers[j].d_ss_dz,	
															p_camera->ss_x,
															p_camera->ss_y,
															p_camera->nx,
															p_camera->ny,
															p_camera->minW,
															p_camera->rmax,
															boundingBoxBuffer.d_BB,
															boundingBoxBuffer.d_D,
															boundingBoxBuffer.nBB,
															samplePosOffet,
															sampleRotOffet,
															p_camera->d_x,
															p_camera->d_y,
															p_camera->d_z,														
															p_camera->d_c,
															p_rtl->depthBuffers[j].devStates,
															p_rtl->depthBuffers[j].d_dx,
															p_rtl->depthBuffers[j].d_dy,
															p_rtl->depthBuffers[j].d_dz,
															humanBB);


			//cuda_calc2::raytraceVertices<<<p_camera->nBlocks,p_camera->nThreads, 0, *(p_rtl->cudaStream[j])>>>(
			//												vertexBuffer.d_vx,
			//												vertexBuffer.d_vy,
			//												vertexBuffer.d_vz,																										
			//												vertexBuffer.d_fx,
			//												vertexBuffer.d_fy,
			//												vertexBuffer.d_fz,													
			//												vertexBuffer.nF,
			//												p_camera->d[0],
			//												p_camera->d[1],
			//												samplePosOffet,
			//												sampleRotOffet,
			//												p_camera->d_x,
			//												p_camera->d_y,
			//												p_camera->d_z,
			//												p_rtl->depthBuffers[j].d_dx,
			//												p_rtl->depthBuffers[j].d_dy,
			//												p_rtl->depthBuffers[j].d_dz);


			cudaStatus = cudaGetLastError();
			if (cudaStatus != cudaSuccess) {
				fprintf(stderr, "raytraceVertices stream %d camera %d launch failed: %s\n", i,j,cudaGetErrorString(cudaStatus));
				IO::waitForEnter();
			}
		}
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code after launching mergeSuperSampling: %s\n", cudaGetErrorString(cudaStatus));
		IO::waitForEnter();

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
		IO::waitForEnter();
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching transformSamplePointBuffer!\n", cudaStatus);
		IO::waitForEnter();

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

void CF2::printCentroid(struct DEPTH_BUFFER* depth)
{
	float* x = new float[depth->size];
	float* y = new float[depth->size];
	float* z = new float[depth->size];

	CudaMem::cudaMemCpyReport(x, depth->d_dx, depth->size*sizeof(float), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(y, depth->d_dy, depth->size*sizeof(float), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(z, depth->d_dz, depth->size*sizeof(float), cudaMemcpyDeviceToHost);

	//check avg value
	float vx = 0.0f;
	float vy = 0.0f;
	float vz = 0.0f;
	int w = 0;
	for(int i=0; i<depth->size; i++)
	{
		if(!IO::is_nan(x[i]))
		{
			vx += x[i];
			vy += y[i];
			vz += z[i];
			w++;
		}
	}
	vx = vx/w;
	vy = vy/w;
	vz = vz/w;
	printf("centroid: [%.6f  %.6f  %.6f]\t%d\n", vx, vy, vz, w);
	delete x;
	delete y;
	delete z;
}

void CF2::calculateCentroid()
{
	cudaError_t cudaStatus;


	RAYTRACING_LAUNCH* p_rtl;
	SAMPLE_CAMERA* p_camera;
	for(int i=0; i<optiSession.n; i++)
	{
		p_rtl = &optiSession.launchs[i];

	
		
		//calcMiddlePoint(float* Dx, float* Dy, float* Dz, int nP, float* a_x, float* a_y, float* a_z)			
		cuda_calc2::calcMiddlePoint<<<1,AVG_BUFFER_SIZE, 0, *(p_rtl->cudaStream[0])>>>(
														p_rtl->depthBuffer.d_dx,													
														p_rtl->depthBuffer.d_dy,
														p_rtl->depthBuffer.d_dz,
														p_rtl->depthBuffer.d_cis,
														p_rtl->depthBuffer.size,
														optiSession.ecs[i]->getMaxIndex(),
														p_rtl->centroid.d_cx,
														p_rtl->centroid.d_cy,
														p_rtl->centroid.d_cz);





		cudaStatus = cudaGetLastError();
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "calcMiddlePoint stream %d launch failed: %s\n", i,cudaGetErrorString(cudaStatus));
			IO::waitForEnter();
		}
		
	}

			
	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching calcMiddlePoint!\n", cudaStatus);
		IO::waitForEnter();
	}
}

void CF2::calculateCluster()
{
	cudaError_t cudaStatus;


	RAYTRACING_LAUNCH* p_rtl;
	//for(int i=0; i<optiSession.n; i++)
	//{
	//	p_rtl = &optiSession.launchs[i];
	//	
	//	dim3 blocks(launchConfigDistanceMatrix.nblocks,launchConfigDistanceMatrix.nblocks,1);
	//	dim3 threads(launchConfigDistanceMatrix.nthreads,launchConfigDistanceMatrix.nthreads,1);
	//	cuda_calc2::calcDistanceMatrix<<<blocks, threads,  0, *(p_rtl->cudaStream[0])>>>(p_rtl->depthBuffer.d_dx, 
	//																					p_rtl->depthBuffer.d_dy,
	//																					p_rtl->depthBuffer.d_dz, 
	//																					launchConfigDistanceMatrix.nsample, 
	//																					p_rtl->depthBuffer.size,
	//																					p_rtl->depthBuffer.d_dm);
	//	cudaStatus = cudaGetLastError();
	//	if (cudaStatus != cudaSuccess) {	
	//		fprintf(stderr, "calcDistanceMatrix stream %d launch failed: %s\n", i,cudaGetErrorString(cudaStatus));
	//		IO::waitForEnter();
	//	}
	//	
	//}
	//		
	//cudaStatus = cudaDeviceSynchronize();
	//if (cudaStatus != cudaSuccess) {
	//	fprintf(stderr, "cudaDeviceSynchronize returned error code %s after launching calcDistanceMatrix!\n", cudaGetErrorString(cudaStatus));
	//	IO::waitForEnter();
	//}


	//copying distance matrix asynchronius
	for(int i=0; i<optiSession.n; i++)
	{
		p_rtl = &optiSession.launchs[i];
		
		//CudaMem::cudaMemCpyAsyncReport(p_rtl->depthBuffer.dm, p_rtl->depthBuffer.d_dm,
		//p_rtl->depthBuffer.size*p_rtl->depthBuffer.size*sizeof(float), cudaMemcpyDeviceToHost, *(p_rtl->cudaStream[0]));

		CudaMem::cudaMemCpyReport(p_rtl->depthBuffer.dx, p_rtl->depthBuffer.d_dx,
		p_rtl->depthBuffer.size*sizeof(float), cudaMemcpyDeviceToHost);
		CudaMem::cudaMemCpyReport(p_rtl->depthBuffer.dy, p_rtl->depthBuffer.d_dy,
		p_rtl->depthBuffer.size*sizeof(float), cudaMemcpyDeviceToHost);
		CudaMem::cudaMemCpyReport(p_rtl->depthBuffer.dz, p_rtl->depthBuffer.d_dz,
		p_rtl->depthBuffer.size*sizeof(float), cudaMemcpyDeviceToHost);
	}
	//cudaStatus = cudaDeviceSynchronize();
	//if (cudaStatus != cudaSuccess) {
	//	fprintf(stderr, "cudaDeviceSynchronize returned error code %s after launching cluster!\n", cudaGetErrorString(cudaStatus));
	//	IO::waitForEnter();
	//}

	#pragma omp parallel for
	for(int i=0; i<optiSession.n; i++)
	{
		p_rtl = &optiSession.launchs[i];
		optiSession.ecs[i]->cluster();
		//printf("parallel sesssion %d\n", i);
		CudaMem::cudaMemCpyReport(p_rtl->depthBuffer.d_cis, p_rtl->depthBuffer.cis, p_rtl->depthBuffer.size*sizeof(int), cudaMemcpyHostToDevice);
	}

	
//	CudaMem::cudaMemsetReport(p_rtl->depthBuffer.d_cis, 0, p_rtl->depthBuffer.size*sizeof(int));
		//cluster(bool* isProcessed, bool* inQ, float* dx, float*dm, int* cis, int nsample, int N)
		//cuda_calc2::cluster<<<launchConfigMaxCluster.nblocks, launchConfigMaxCluster.nthreads,
		//												0, *(p_rtl->cudaStream[0])>>>(p_rtl->depthBuffer.d_hp, 
		//																				p_rtl->depthBuffer.d_iiq,
		//																				p_rtl->depthBuffer.d_dx, 
		//																				p_rtl->depthBuffer.d_dm,
		//																				p_rtl->depthBuffer.d_cis,
		//																				launchConfigMaxCluster.nsample, 
		//																				p_rtl->depthBuffer.size);
		//cudaStatus = cudaGetLastError();
		//if (cudaStatus != cudaSuccess) {	
		//	fprintf(stderr, "cluster stream %d launch failed: %s\n", i,cudaGetErrorString(cudaStatus));
		//	IO::waitForEnter();
		//}
		
	
			



	
			








	//for(int i=0; i<optiSession.n; i++)
	//{
	//	p_rtl = &optiSession.launchs[i];

	//	//init indices 
	//	CudaMem::cudaMemsetReport(p_rtl->depthBuffer.d_ci, -1, p_rtl->depthBuffer.size*sizeof(int));
	//	cuda_calc2::calcCluster<<<launchConfigMaxCluster.nblocks, launchConfigMaxCluster.nthreads, 0, *(p_rtl->cudaStream[0])>>>(p_rtl->depthBuffer.d_ci,
	//																															p_rtl->depthBuffer.d_dm,
	//																															launchConfigMaxCluster.nsample,
	//																															p_rtl->depthBuffer.size,
	//																															p_rtl->depthBuffer.d_maxnc);
	//	cudaStatus = cudaGetLastError();
	//	if (cudaStatus != cudaSuccess) {	
	//		fprintf(stderr, "calcCluster stream %d launch failed: %s\n", i,cudaGetErrorString(cudaStatus));
	//		IO::waitForEnter();
	//	}
	//	
	//}
	//		
	//cudaStatus = cudaDeviceSynchronize();
	//if (cudaStatus != cudaSuccess) {
	//	fprintf(stderr, "cudaDeviceSynchronize returned  %s after launching calcCluster!\n", cudaGetErrorString(cudaStatus));
	//	IO::waitForEnter();
	//}


	//
	//for(int i=0; i<optiSession.n; i++)
	//{
	//	p_rtl = &optiSession.launchs[i];	
	//	cuda_calc2::calcNmembersOfCluster<<<launchConfigMaxCluster.nblocks, launchConfigMaxCluster.nthreads, 0, *(p_rtl->cudaStream[0])>>>(
	//																															p_rtl->depthBuffer.d_ci,
	//																															p_rtl->depthBuffer.size,
	//																															launchConfigMaxCluster.nsample,
	//																															p_rtl->depthBuffer.d_cm,
	//																															p_rtl->depthBuffer.d_maxclusterindice);
	//	cudaStatus = cudaGetLastError();
	//	if (cudaStatus != cudaSuccess) {	
	//		fprintf(stderr, "calcNmembersOfCluster stream %d launch failed: %s\n", i,cudaGetErrorString(cudaStatus));
	//		IO::waitForEnter();
	//	}
	//	
	//}
	//		
	//cudaStatus = cudaDeviceSynchronize();
	//if (cudaStatus != cudaSuccess) {
	//	fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching calcNmembersOfCluster!\n", cudaStatus);
	//	IO::waitForEnter();
	//}

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
														p_rtl->probResult.d_p,
														p_rtl->probResult.d_d,
														p_rtl->probResult.d_w);



		cudaStatus = cudaGetLastError();
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "distanceToEllipseModel stream %d launch failed: %s\n", i,cudaGetErrorString(cudaStatus));
			IO::waitForEnter();
		}
		
	}

	
	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching distanceToEllipseModel!\n", cudaStatus);
		IO::waitForEnter();
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

	probResult->d = new float[n*session];
	probResult->maxd = new float[session];

	CudaMem::cudaMemAllocReport((void**)&probResult->d_d, n*session*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&probResult->d_maxd, session*sizeof(float));

	probResult->w = new int[n*session];
	probResult->maxw = new int[session];

	CudaMem::cudaMemAllocReport((void**)&probResult->d_w, n*session*sizeof(int));
	CudaMem::cudaMemAllocReport((void**)&probResult->d_maxw, session*sizeof(int));
}

void CF2::freePropBuffer(PROB_RESULT* probResult)
{
	
	delete probResult->p;
	delete probResult->maxp;
	CudaMem::cudaFreeReport(probResult->d_p);
	CudaMem::cudaFreeReport(probResult->d_maxp);

	delete probResult->d;
	delete probResult->maxd;
	CudaMem::cudaFreeReport(probResult->d_d);
	CudaMem::cudaFreeReport(probResult->d_maxd);

	delete probResult->w;
	delete probResult->maxw;
	CudaMem::cudaFreeReport(probResult->d_w);
	CudaMem::cudaFreeReport(probResult->d_maxw);
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
			IO::waitForEnter();
		}
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching calcMiddlePoint!\n", cudaStatus);
		IO::waitForEnter();
	}

	for(int i=0; i<optiSession.n; i++)
	{
		p_rtl = &optiSession.launchs[i];
		cuda_calc2::calculateMinDist<<<1,MAX_BUFFER_SIZE,0, *(p_rtl->cudaStream[0])>>>(
														p_rtl->probResult.d_d,
														p_rtl->probResult.n,
														p_rtl->probResult.d_maxd,
														currentTrans.d_pr);
		

					

		cudaStatus = cudaGetLastError();
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "calculateMaxProb stream %d launch failed: %s\n", i,cudaGetErrorString(cudaStatus));
			IO::waitForEnter();
		}
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching calcMiddlePoint!\n", cudaStatus);
		IO::waitForEnter();
	}


	for(int i=0; i<optiSession.n; i++)
	{
		//calculateMinWeight(int* weights, int n, float* dist, float* mindist, int* w)
		p_rtl = &optiSession.launchs[i];
		cuda_calc2::calculateMinWeight<<<1,MAX_BUFFER_SIZE,0, *(p_rtl->cudaStream[0])>>>(
														p_rtl->probResult.d_w,
														p_rtl->probResult.n,
														p_rtl->probResult.d_d,
														p_rtl->probResult.d_maxd,
														p_rtl->probResult.d_maxw);
		

					

		cudaStatus = cudaGetLastError();
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "calculateMaxProb stream %d launch failed: %s\n", i,cudaGetErrorString(cudaStatus));
			IO::waitForEnter();
		}
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching calcMiddlePoint!\n", cudaStatus);
		IO::waitForEnter();
	}



	for(int i=0; i<optiSession.n; i++)
	{
		p_rtl = &optiSession.launchs[i];
		CudaMem::cudaMemCpyReport(p_rtl->probResult.maxp, p_rtl->probResult.d_maxp, sizeof(float), cudaMemcpyDeviceToHost);
		CudaMem::cudaMemCpyReport(p_rtl->probResult.maxd, p_rtl->probResult.d_maxd, sizeof(float), cudaMemcpyDeviceToHost);
		CudaMem::cudaMemCpyReport(p_rtl->probResult.maxw, p_rtl->probResult.d_maxw, sizeof(int), cudaMemcpyDeviceToHost);

	}
	
}
