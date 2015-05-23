#include "CF2.h"

#include "CudaMem.h"


#include <cuda_profiler_api.h>
#include "device_launch_parameters.h"
#include "cuda_runtime.h"
#include "mathcuda.h"
#include "NearestNeighbour.h"
#include "InversionSearch.h"
#include "EC.h"

#include <Windows.h>
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

	resultingSolution.cameraTypes = NULL;
	resultingSolution.angleIndex = NULL;
	resultingSolution.pclIndex = NULL;
	



	
	IO::loadPCL(&robot,"robot_short.bin");
	IO::loadPCL(&human,"human.bin");
	IO::loadPCL(&environment, "environment.bin");

	IO::loadSamplePositions(&samplePositions, "samplePositions.bin");
	IO::loadSamplePCL(&samplePoints, "samplePoints.bin");
	IO::loadSampleRotations(&sampleRotations, "sampleRotations.bin");
	IO::loadSampleCamera(&sampleCameraTypes, "sampleCamera.bin");
	IO::loadValidPos(&sampleValidPositions, "sampleValidPositions.bin");
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
	//cameraCombination.gen_result = gen_comb_norep_lex_init(cameraCombination.vector, sampleCameraTypes.nCameraTypes, currentNumberOfCams);
	cameraCombination.gen_result = gen_comb_rep_lex_init(cameraCombination.vector, sampleCameraTypes.nCameraTypes, currentNumberOfCams);
}

void CF2::overlayData()
{
	for(int i=0; i<probResult.nmax; i++)
	{
		//assert(probResult.maxp_normalized[i] <= 1.0);
	
		optiSession.pI[i*currentNumberOfCams+0] = 109;
		optiSession.aI[i*currentNumberOfCams+0] = 430;
		optiSession.pI[i*currentNumberOfCams+1] = 130;
		optiSession.aI[i*currentNumberOfCams+1] = 315;
	}
}

void CF2::checkFirstTwoCameras()
{

	int* cis1 = new int[optiSession.launchs[0].depthBuffer.size];
	int* cis2 = new int[optiSession.launchs[0].depthBuffer.size];

	CudaMem::cudaMemCpyReport(cis1, optiSession.launchs[0].depthBuffer.d_cis, optiSession.launchs[0].depthBuffer.size*sizeof(int), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(cis2, optiSession.launchs[1].depthBuffer.d_cis, optiSession.launchs[1].depthBuffer.size*sizeof(int), cudaMemcpyDeviceToHost);


	for(int i=0; i<optiSession.launchs[0].depthBuffer.size; i++)
	{
		assert(cis1[i] == cis2[i]);
	}

	delete cis1;
	delete cis2;

	float* depth1 = new float[optiSession.launchs[0].depthBuffer.size];
	float* depth2 = new float[optiSession.launchs[1].depthBuffer.size];

	const float epsf = 1e-5f;
	double const eps = 1e-5;

	CudaMem::cudaMemCpyReport(depth1, optiSession.launchs[0].depthBuffer.d_dx, optiSession.launchs[0].depthBuffer.size*sizeof(float), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(depth2, optiSession.launchs[1].depthBuffer.d_dx, optiSession.launchs[1].depthBuffer.size*sizeof(float), cudaMemcpyDeviceToHost);
	for(int i=0; i<optiSession.launchs[0].depthBuffer.size; i++)
	{
		if(isnan(depth1[i]) || isnan(depth2[i]))
		{
			assert(isnan(depth1[i]) && isnan(depth1[i]));
		}else{
			assert(abs(depth1[i]-depth2[i]) < epsf);
		}		
	}

	CudaMem::cudaMemCpyReport(depth1, optiSession.launchs[0].depthBuffer.d_dy, optiSession.launchs[0].depthBuffer.size*sizeof(float), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(depth2, optiSession.launchs[1].depthBuffer.d_dy, optiSession.launchs[1].depthBuffer.size*sizeof(float), cudaMemcpyDeviceToHost);
	for(int i=0; i<optiSession.launchs[0].depthBuffer.size; i++)
	{
		if(isnan(depth1[i]) || isnan(depth2[i]))
		{
			assert(isnan(depth1[i]) && isnan(depth1[i]));
		}else{
			assert(abs(depth1[i]-depth2[i]) < epsf);
		}		
	}

	CudaMem::cudaMemCpyReport(depth1, optiSession.launchs[0].depthBuffer.d_dz, optiSession.launchs[0].depthBuffer.size*sizeof(float), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(depth2, optiSession.launchs[1].depthBuffer.d_dz, optiSession.launchs[1].depthBuffer.size*sizeof(float), cudaMemcpyDeviceToHost);
	for(int i=0; i<optiSession.launchs[0].depthBuffer.size; i++)
	{
		if(isnan(depth1[i]) || isnan(depth2[i]))
		{
			assert(isnan(depth1[i]) && isnan(depth1[i]));
		}else{
			assert(abs(depth1[i]-depth2[i]) < epsf);
		}		
	}

	delete depth1;
	delete depth2;

	float c1, c2;
	CudaMem::cudaMemCpyReport(&c1, optiSession.launchs[0].centroid.d_cx, sizeof(float), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(&c2, optiSession.launchs[1].centroid.d_cx, sizeof(float), cudaMemcpyDeviceToHost);	
	if(isnan(c1) || isnan(c2))
	{
		assert(isnan(c1) && isnan(c2));
	}else{
		assert(abs(c1-c2) < epsf);
	}	
	CudaMem::cudaMemCpyReport(&c1, optiSession.launchs[0].centroid.d_cy, sizeof(float), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(&c2, optiSession.launchs[1].centroid.d_cy, sizeof(float), cudaMemcpyDeviceToHost);
	if(isnan(c1) || isnan(c2))
	{
		assert(isnan(c1) && isnan(c2));
	}else{
		assert(abs(c1-c2) < epsf);
	}	
	CudaMem::cudaMemCpyReport(&c1, optiSession.launchs[0].centroid.d_cz, sizeof(float), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(&c2, optiSession.launchs[1].centroid.d_cz, sizeof(float), cudaMemcpyDeviceToHost);
	if(isnan(c1) || isnan(c2))
	{
		assert(isnan(c1) && isnan(c2));
	}else{
		assert(abs(c1-c2) < epsf);
	}	




	double *prob1 = new double[probResult.n];
	double *prob2 = new double[probResult.n];

	

	CudaMem::cudaMemCpyReport(prob1, optiSession.launchs[0].probResult.d_p, optiSession.launchs[0].probResult.n*sizeof(double), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(prob2, optiSession.launchs[1].probResult.d_p, optiSession.launchs[1].probResult.n*sizeof(double), cudaMemcpyDeviceToHost);

	for(int i=0; i<probResult.n; i++)
	{
		assert(abs(prob1[i]-prob2[i]) < eps);
	}

		

	delete prob1;
	delete prob2;

}

void CF2::run()
{
	//outer loop for combination of multiple cameras
	initCameraCombination();

	//camera specific allocation
	initParallelOptiRuns();
	IO::waitForEnter();
	//////////finding first the probability density funtion of the angles to reduces the amount of raytracing angles
	sC = new InversionSearch(&samplePoints, &sampleRotations,&sampleValidPositions, 1, MAX_ITE, nn->getNN());

	//((InversionSearch*)sC)->setInversionParamters(&samplePointsBuffer);
	//while(sC->iterate(optiSession.pI, optiSession.aI, probResult.maxp, probResult.maxd, probResult.maxw) )
	//{	
	//	zeroProb();
	//	for(int i=0; i< samplePositions.nP; i++)
	//	{				

	//		setCurrentTans(i);
	//		transformVertexBuffer();
	//		transformBoundingBoxBuffer();
	//		transformSamplePointBuffer();

	//		rayTrace();			
	//		//IO::saveDepthBufferToFile(&depthBuffer, "depthBuffer.bin");
	//		calculateCluster();
	//		//printCentroid(&depthBuffer);

	//		calculateCentroid();
	//		calculateProbOfHumanDetection();
	//		calculateMaxProb();	
	//		//checkIntermediateResults();
	//	

	//	}
	//	printf("angle initializing....\n");
	//}	
	//IO::saveInversionSearch(sC->prop, sC->dist, sC->weights, SEARCH_DOF*sampleRotations.nRotations, "inversionSearch.bin");
	////IO::waitForEnter();

	//int n;
	//IO::loadInversionSearch(sC->prop, sC->dist, sC->weights, &n, "inversionSearch.bin");
	AngleGenerator aG(sC->prop, sampleRotations.nRotations, SEARCH_DOF);
	delete sC;

	freeParallelOptiRuns();
#ifdef RAISING_CAMERAS
	currentNumberOfCams = 1;
#else
	currentNumberOfCams = 4;
#endif
	
	initCameraCombination();
	printf("starting optimisation...\n");
	//IO::waitForEnter();
	
#ifdef RAISING_CAMERAS
	while(currentNumberOfCams < 10)
#else
	while(currentNumberOfCams < 5)
#endif
	{
	
		int ite =0;
		do
		{
			//sC = new CompleteEnumeration(&samplePoints, &sampleRotations, currentNumberOfCams, MAX_ITE, NULL);
			sC = new SimulatedAnnealing(&samplePoints, &sampleRotations, &sampleValidPositions,  currentNumberOfCams, MAX_ITE, nn->getNN(), &aG);
			

			initParallelOptiRuns();			
			time_t start;			
			time(&start);

			//if (currentNumberOfCams == 2)
			//{
				while(sC->iterate(optiSession.pI, optiSession.aI, probResult.maxp_normalized, probResult.maxd, probResult.maxw)/*&&ite<50*/ )
				{
					//overlayData();
					//Sleep(4000);
					//system("cls");	
					ite++;
					zeroProb();
					for(int i=0; i< samplePositions.nP; i++)
					{
						reinitLoop();
						setCurrentTans(i);
						transformVertexBuffer();
						transformBoundingBoxBuffer();
						transformSamplePointBuffer();

						rayTrace();
						//IO::saveDepthBufferToFile(&depthBuffer, "depthBuffer.bin");
						calculateCluster();
						calculateCentroid();
						//checkFirstTwoCameras();
						//printCentroid(&depthBuffer);
						calculateProbOfHumanDetection();
						//checkFirstTwoCameras();
						calculateMaxProb();
						
						
						//Progress::printProgress((double)i, (double)samplePositions.nP, start, "raytracing rp ");	
					}
					
					normalizeMaxProb();
					
					
					//printf("global min CF %.10lf\n", resultingSolution.minCost);
					//IO::saveDepthBufferToFile(&depthBuffer, "depthBuffer.bin");
					//IO::saveDepthBufferToFileSuperSamples(&depthBuffer, "depthBuffer.bin");
					//Progress::printProgress((double)optiSession.pI[0], (double)samplePoints.n, start, "raytracing cp ");
					//IO::waitForEnter();
				
				}
			//}

	
			//saving results
			//saveAllVertices();
			//
			//transformSamplePointBuffer();
			//IO::saveOptimisationResults(&samplePointsBuffer, &samplePoints, &sampleRotations, sC->prop, sC->dist,sC->weights,  "completeEnumeration.bin");
			setCurrentTans(0);
			IO::writeMinCostToFile(cameraCombination.vector, resultingSolution.pclIndex, resultingSolution.angleIndex, currentNumberOfCams);
			sC->writeResultsToFile(cameraCombination.vector, currentNumberOfCams, &samplePointsBuffer);
			delete sC;
			freeParallelOptiRuns();

			printf("finished current camera combination with max prob %.10lf\n", resultingSolution.minCost);

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

void CF2::run_evaluation()
{
	IO::loadResultingSolution(&resultingSolution, "resultingSolution.bin");

	//setting current number of cameras
	currentNumberOfCams = resultingSolution.nC;
	initCameraCombination();
	for(int i=0; i<currentNumberOfCams; i++)
	{
		cameraCombination.vector[i] = (unsigned long long)resultingSolution.cameraTypes[i];
	}
	//having camera types set
	//setting camera configuration
	initParallelOptiRuns();
	memcpy(optiSession.pI, resultingSolution.pclIndex, currentNumberOfCams*sizeof(int));
	memcpy(optiSession.aI, resultingSolution.angleIndex, currentNumberOfCams*sizeof(int));
	memcpy(optiSession.pI+currentNumberOfCams, resultingSolution.pclIndex, currentNumberOfCams*sizeof(int));
	memcpy(optiSession.aI+currentNumberOfCams, resultingSolution.angleIndex, currentNumberOfCams*sizeof(int));

	zeroProb();
	for(int i=0; i< samplePositions.nP; i++)
	{
		reinitLoop();
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
	normalizeMaxProb();
	//checkIntermediateResults();
	 
	printf("resulting prop: %.10lf\n", probResult.maxp_normalized[0]);
	IO::waitForEnter();


}

void CF2::run_completeEnumeration()
{
		//outer loop for combination of multiple cameras
	initCameraCombination();
	sC = new CompleteEnumeration(&samplePoints, &sampleRotations, &sampleValidPositions, currentNumberOfCams, MAX_ITE, NULL);		
	initParallelOptiRuns();		
	printf("starting optimisation...\n");
	IO::waitForEnter();
	time_t start;			
	time(&start);

	while(sC->iterate(optiSession.pI, optiSession.aI, probResult.maxp_normalized, probResult.maxd, probResult.maxw) )
	{
		zeroProb();
		for(int i=0; i< samplePositions.nP; i++)
		{
			reinitLoop();
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
		normalizeMaxProb();
		//IO::saveDepthBufferToFile(&depthBuffer, "depthBuffer.bin");
		//IO::saveDepthBufferToFileSuperSamples(&depthBuffer, "depthBuffer.bin");
		//Progress::printProgress((double)optiSession.pI[0], (double)samplePoints.n, start, "raytracing cp ");
		//IO::waitForEnter();

	}
	

	
	//saving results	
	double percentage = ((double)(100.0*sC->numberOfCalculations))/((double)(samplePoints.n*sampleRotations.nRotations));
	printf("%.10lf\t%d\t%d\t%.2lf\n", sC->maxProb, sC->bestPindex, sC->bestAngleIndex, percentage); 
	IO::waitForEnter();
	//setCurrentTans(0);
	//transformSamplePointBuffer();
	//IO::saveOptimisationResults(&samplePointsBuffer, &samplePoints, &sampleRotations, sC->prop, sC->dist,sC->weights,  "completeEnumeration.bin");
	//sC->writeResultsToFile(cameraCombination.vector, currentNumberOfCams, &samplePointsBuffer);
	delete sC;
	freeParallelOptiRuns();

	printf("finished complete enumeration\n");

	IO::waitForEnter();
}

void CF2::checkIntermediateResults()
{


	RAYTRACING_LAUNCH* p_rtl;
	p_rtl = &optiSession.launchs[0];
	double* p = new double[p_rtl->probResult.n];
	double maxp, maxp2;
	double eps=1e-5;
	for(int j=0; j<optiSession.n; j++)
	{
		p_rtl = &optiSession.launchs[j];	
		maxp2 = 0.0;
		
		CudaMem::cudaMemCpyReport(p, p_rtl->probResult.d_p, p_rtl->probResult.n*sizeof(double), cudaMemcpyDeviceToHost);		
		CudaMem::cudaMemCpyReport(&maxp, p_rtl->probResult.d_maxp, sizeof(double), cudaMemcpyDeviceToHost);
				
		for(int i=0;i<p_rtl->probResult.n; i++)
		{
			maxp2 = std::max(maxp2,p[i]);			
		}		
		assert( abs(maxp-maxp2) < eps && maxp <= 1.0  && maxp2 <= 1.0 );				
	}
	delete p;
}

void CF2::initBoundingBoxBuffer()
{
	
	boundingBoxBuffer.nBB = robot.nBB+environment.nBB+human.nBB ;
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

void CF2::freeBoundingBoxBuffer()
{
	CudaMem::cudaFreeReport(boundingBoxBuffer.d_BB);
	CudaMem::cudaFreeReport(boundingBoxBuffer.d_D);

	boundingBoxBuffer.d_BB = NULL;
	boundingBoxBuffer.d_D = NULL;
}

void CF2::createCudaStream(cudaStream_t** streams, int n)
{
		cudaError_t cudaStatus;

		*streams = new cudaStream_t[n];
		cudaStream_t* pStream = *streams;
		for(int i=0;i<n;i++)
		{
			
			cudaStatus = cudaStreamCreate (&(pStream[i]));
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

	//CudaMem::cudaMemsetReport(probResult.d_p, 0, probResult.n*sizeof(double));
	//CudaMem::cudaMemsetReport(probResult.d_maxp, 0, probResult.nmax*sizeof(double));

	for(int i=0;i<probResult.nmax; i++)
	{
		probResult.maxp_normalized[i] = 0.0;
	}


}

void CF2::reinitLoop()
{
	//reinit all props of the loop for 
	for(int i=0; i<probResult.n; i++)
	{
		probResult.p[i] = 0.0;
		probResult.d[i] = 0.0;
		probResult.w[i] = 0.0;
	}
	CudaMem::cudaMemCpyReport(probResult.d_p, probResult.p, probResult.n*sizeof(double), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(probResult.d_d, probResult.d, probResult.n*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(probResult.d_w, probResult.w, probResult.n*sizeof(float), cudaMemcpyHostToDevice);
	for(int i=0; i<probResult.nmax; i++)
	{
		probResult.maxp[i] = 0.0;
	}
	CudaMem::cudaMemCpyReport(probResult.d_maxp, probResult.maxp, probResult.nmax*sizeof(double), cudaMemcpyHostToDevice);


	//setting bounding box
	for(int i=0; i<optiSession.n*currentNumberOfCams*boundingBoxBuffer.nBB; i++)
	{
		depthBuffer.bb_hit[i] = true;
	}
	CudaMem::cudaMemCpyReport(depthBuffer.d_bb_hit, depthBuffer.bb_hit, optiSession.n*currentNumberOfCams*boundingBoxBuffer.nBB*sizeof(bool), cudaMemcpyHostToDevice);

	//setting up ss
	cudaMemset(depthBuffer.d_ss_dx, 0, depthBuffer.sssize*sizeof(int));
	cudaMemset(depthBuffer.d_ss_dy, 0, depthBuffer.sssize*sizeof(int));
	cudaMemset(depthBuffer.d_ss_dz, 0, depthBuffer.sssize*sizeof(int));

	cudaMemset(depthBuffer.d_dx, 0, depthBuffer.size*sizeof(int));
	cudaMemset(depthBuffer.d_dy, 0, depthBuffer.size*sizeof(int));
	cudaMemset(depthBuffer.d_dz, 0, depthBuffer.size*sizeof(int));
	cudaMemset(depthBuffer.d_cis, -1, depthBuffer.size*sizeof(int));

	memset(depthBuffer.dx, 0, depthBuffer.size*sizeof(int));
	memset(depthBuffer.dy, 0, depthBuffer.size*sizeof(int));
	memset(depthBuffer.dz, 0, depthBuffer.size*sizeof(int));	
	memset(depthBuffer.cis, -1, depthBuffer.size*sizeof(int));

	//setting 
	memset(centroid.cx, 0, optiSession.n*sizeof(float));	
	memset(centroid.cy, 0, optiSession.n*sizeof(float));	
	memset(centroid.cz, 0, optiSession.n*sizeof(float));	

	cudaMemset(centroid.d_cx, 0, optiSession.n*sizeof(float));	
	cudaMemset(centroid.d_cy, 0, optiSession.n*sizeof(float));	
	cudaMemset(centroid.d_cz, 0, optiSession.n*sizeof(float));	


	
}


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
	int cameraVerticesPerLaunch = 0;
	int cameraFacesPerLaunch = 0;
	for(int ite=0; ite<optiSession.n; ite++)
	{		
		optiSession.launchs[ite].cams = new SAMPLE_CAMERA *[currentNumberOfCams];
		optiSession.launchs[ite].depthBuffers = new DEPTH_BUFFER[currentNumberOfCams];
		optiSession.launchs[ite].boundingBoxBuffers = new BB_BUFFER[currentNumberOfCams];
		optiSession.launchs[ite].vertexBuffers = new VERTEX_BUFFER[currentNumberOfCams];
		optiSession.launchs[ite].cudaStream = new cudaStream_t*[currentNumberOfCams];	
		optiSession.launchs[ite].aI = new int*[currentNumberOfCams];
		optiSession.launchs[ite].pI = new int*[currentNumberOfCams];
		optiSession.launchs[ite].n = currentNumberOfCams;

		raysPerLaunch = 0;
		raysSSPerLaunch = 0;
		cameraVerticesPerLaunch = 0;
		cameraFacesPerLaunch = 0;

		for(int i=0; i<currentNumberOfCams; i++)
		{			
			optiSession.launchs[ite].cams[i] = &sampleCameraTypes.possibleCameraTypes[cameraCombination.vector[i]];
			nOfRays += sampleCameraTypes.possibleCameraTypes[cameraCombination.vector[i]].nRays;
			nOfSSRays += sampleCameraTypes.possibleCameraTypes[cameraCombination.vector[i]].ssnRays;

			raysPerLaunch += optiSession.launchs[ite].cams[i]->nRays;
			raysSSPerLaunch += optiSession.launchs[ite].cams[i]->ssnRays;

			cameraVerticesPerLaunch += optiSession.launchs[ite].cams[i]->pcl.nV;
			cameraFacesPerLaunch += optiSession.launchs[ite].cams[i]->pcl.nF;
		}

	}


	////init the rest of the launch
	initDepthBuffer(&depthBuffer,optiSession.n, currentNumberOfCams, nOfRays, nOfSSRays);	
	initCameraVertexBuffer(&vertexBufferCamera, optiSession.n);
	initCentroidBuffer(&centroid, optiSession.n);
	initPropBuffer(&probResult ,sampleFitting.n, optiSession.n);	

	//clearPropBuffer(&probResult);
	createCudaStream(&cudaStream, currentNumberOfCams*optiSession.n);

	//setting all the pointer
	nOfRays = 0;
	nOfSSRays = 0;
	int nofProps = 0;
	
	int nOfVertices = 0;
	int nOfFaces = 0;
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

		optiSession.launchs[ite].vertexBuffer.d_vx = vertexBufferCamera.d_vx + nOfVertices;
		optiSession.launchs[ite].vertexBuffer.d_vy = vertexBufferCamera.d_vy + nOfVertices;
		optiSession.launchs[ite].vertexBuffer.d_vz = vertexBufferCamera.d_vz + nOfVertices;

		optiSession.launchs[ite].vertexBuffer.d_fx = vertexBufferCamera.d_fx;
		optiSession.launchs[ite].vertexBuffer.d_fy = vertexBufferCamera.d_fy;
		optiSession.launchs[ite].vertexBuffer.d_fz = vertexBufferCamera.d_fz;			
		optiSession.launchs[ite].vertexBuffer.d_f_bbi = vertexBufferCamera.d_f_bbi;

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
		optiSession.launchs[ite].probResult.maxp_normalized = probResult.maxp_normalized+ite;

		optiSession.launchs[ite].probResult.d_maxd = probResult.d_maxd+ite;
		optiSession.launchs[ite].probResult.maxd = probResult.maxd+ite;

		optiSession.launchs[ite].probResult.d_maxw = probResult.d_maxw+ite;
		optiSession.launchs[ite].probResult.maxw = probResult.maxw+ite;

		optiSession.launchs[ite].probResult.n = sampleFitting.n;
		optiSession.launchs[ite].cameraFaces = cameraFacesPerLaunch;

		//init random states for rays
		//
		raysPerLaunch = 0;
		raysSSPerLaunch = 0;
		cameraVerticesPerLaunch = 0;
		cameraFacesPerLaunch = 0;
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

			optiSession.launchs[ite].vertexBuffers[i].d_vx = vertexBufferCamera.d_vx + nOfVertices;
			optiSession.launchs[ite].vertexBuffers[i].d_vy = vertexBufferCamera.d_vy + nOfVertices;
			optiSession.launchs[ite].vertexBuffers[i].d_vz = vertexBufferCamera.d_vz + nOfVertices;

			optiSession.launchs[ite].vertexBuffers[i].d_fx = vertexBufferCamera.d_fx + cameraFacesPerLaunch;
			optiSession.launchs[ite].vertexBuffers[i].d_fy = vertexBufferCamera.d_fy + cameraFacesPerLaunch;
			optiSession.launchs[ite].vertexBuffers[i].d_fz = vertexBufferCamera.d_fz + cameraFacesPerLaunch;			
			optiSession.launchs[ite].vertexBuffers[i].d_f_bbi = vertexBufferCamera.d_f_bbi + cameraFacesPerLaunch;

			optiSession.launchs[ite].aI[i] = optiSession.aI+ite*currentNumberOfCams+i;
			optiSession.launchs[ite].pI[i] = optiSession.pI+ite*currentNumberOfCams+i;

			optiSession.launchs[ite].cudaStream[i] = cudaStream+ite*currentNumberOfCams+i;
			
			initRadomNumberGenerator(optiSession.launchs[ite].depthBuffers[i].devStates, optiSession.launchs[ite].cams[i]);

			nOfRays += optiSession.launchs[ite].cams[i]->nRays;
			nOfSSRays += optiSession.launchs[ite].cams[i]->ssnRays;

			raysPerLaunch += optiSession.launchs[ite].cams[i]->nRays;
			raysSSPerLaunch += optiSession.launchs[ite].cams[i]->ssnRays;

			nOfVertices += optiSession.launchs[ite].cams[i]->pcl.nV;
			cameraVerticesPerLaunch += optiSession.launchs[ite].cams[i]->pcl.nV;
			cameraFacesPerLaunch += optiSession.launchs[ite].cams[i]->pcl.nF;
		}
		optiSession.launchs[ite].depthBuffer.size = raysPerLaunch;
		optiSession.launchs[ite].depthBuffer.sssize = raysSSPerLaunch;

		optiSession.launchs[ite].vertexBuffer.nF = cameraFacesPerLaunch;
		optiSession.launchs[ite].vertexBuffer.nV = cameraVerticesPerLaunch;
		
	

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


	
	launchConfigCameras.nblocks = 1;
	launchConfigCameras.nthreads = 512;



	


	cudaMemGetInfo( &avail3, &total3 );
	div = (double)avail3/(double)total3;
	usage =  (1.0-div)*100.0;		
	printf("total memory usage is: %.2f\n",usage);

}

void CF2::initCameraVertexBuffer(VERTEX_BUFFER* vertexBuffer, int sessions)
{
	//calculating camera vertex and face size
	int vertexSize = 0;
	int faceSize = 0;
	for(int i=0; i<currentNumberOfCams; i++)
	{
		vertexSize += sampleCameraTypes.possibleCameraTypes[cameraCombination.vector[i]].pcl.nV;
		faceSize += sampleCameraTypes.possibleCameraTypes[cameraCombination.vector[i]].pcl.nF;
	}

	CudaMem::cudaMemAllocReport((void**)&vertexBuffer->d_vx, sessions*vertexSize*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&vertexBuffer->d_vy, sessions*vertexSize*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&vertexBuffer->d_vz, sessions*vertexSize*sizeof(float));

	//we need just one face buffer, because there are equal for all sessions
	CudaMem::cudaMemAllocReport((void**)&vertexBuffer->d_fx, faceSize*sizeof(int));
	CudaMem::cudaMemAllocReport((void**)&vertexBuffer->d_fy, faceSize*sizeof(int));
	CudaMem::cudaMemAllocReport((void**)&vertexBuffer->d_fz, faceSize*sizeof(int));
	CudaMem::cudaMemAllocReport((void**)&vertexBuffer->d_f_bbi, faceSize*sizeof(int));

	//setting up face buffer for complete mesh
	int* fb_x = new int[faceSize];
	int* fb_y = new int[faceSize];
	int* fb_z = new int[faceSize];
	int* fb_bbi = new int[faceSize];

	//filling face buffer
	int faceOffset = 0;
	int bbOffset = 0;
	PCL* pcl;
	int ite = 0;
	for(int i=0; i<currentNumberOfCams; i++)
	{
		pcl = &sampleCameraTypes.possibleCameraTypes[cameraCombination.vector[i]].pcl;
		for(int j=0; j<pcl->nF; j++)
		{						
			fb_x[ite] = pcl->fx[j]+faceOffset;
			fb_y[ite] = pcl->fy[j]+faceOffset;
			fb_z[ite] = pcl->fz[j]+faceOffset;
			fb_bbi[ite] = pcl->f_bbi[j]+bbOffset;
			ite++;
		}
		faceOffset += pcl->nV;
		bbOffset += pcl->nBB;
	}

	//copying it into each launch buffer
	CudaMem::cudaMemCpyReport(vertexBuffer->d_fx, fb_x, faceSize*sizeof(int), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(vertexBuffer->d_fy, fb_y, faceSize*sizeof(int), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(vertexBuffer->d_fz, fb_z, faceSize*sizeof(int), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(vertexBuffer->d_f_bbi, fb_bbi, faceSize*sizeof(int), cudaMemcpyHostToDevice);


	delete fb_x;
	delete fb_y;
	delete fb_z;
	delete fb_bbi;

}

void CF2::freeCameraVertexBuffer(VERTEX_BUFFER* vertexBuffer)
{
	CudaMem::cudaFreeReport(vertexBuffer->d_vx);
	CudaMem::cudaFreeReport(vertexBuffer->d_vy);
	CudaMem::cudaFreeReport(vertexBuffer->d_vz);

	//we need just one face buffer, because there are equal for all sessions
	CudaMem::cudaFreeReport(vertexBuffer->d_fx);
	CudaMem::cudaFreeReport(vertexBuffer->d_fy);
	CudaMem::cudaFreeReport(vertexBuffer->d_fz);
	CudaMem::cudaFreeReport(vertexBuffer->d_f_bbi);
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
	freeCameraVertexBuffer(&vertexBufferCamera);
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

	//environment
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

void CF2::freeVertexBuffer()
{
	CudaMem::cudaFreeReport(vertexBuffer.d_vx);
	CudaMem::cudaFreeReport(vertexBuffer.d_vy);
	CudaMem::cudaFreeReport(vertexBuffer.d_vz);

	CudaMem::cudaFreeReport(vertexBuffer.d_fx);
	CudaMem::cudaFreeReport(vertexBuffer.d_fy);
	CudaMem::cudaFreeReport(vertexBuffer.d_fz);
	CudaMem::cudaFreeReport(vertexBuffer.d_f_bbi);

	vertexBuffer.d_vx = NULL;
	vertexBuffer.d_vy = NULL;
	vertexBuffer.d_vz = NULL;

	vertexBuffer.d_fx = NULL;
	vertexBuffer.d_fy = NULL;
	vertexBuffer.d_fz = NULL;
	vertexBuffer.d_f_bbi = NULL;
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
	currentTrans.p_pr = samplePositions.pr+i;
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

void CF2::transformCameras()
{


	//transformVertexCamera(float* xi, float* yi, float* zi, float* bb, float* camPos_H, float* camRot_H, float* xo, float* yo, float* zo, float* bb_r)
	for(int i=0; i<currentNumberOfCams; i++)
	{
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

void CF2::rayTraceCameras()
{


	int indexPos = 0;
	int indexRot = 0;


	float* samplePosOffet;
	float* sampleRotOffet;


	cudaError_t cudaStatus;
	//starting all parallel kernels and wait for finishing

	RAYTRACING_LAUNCH* p_rtl;
	SAMPLE_CAMERA* p_camera;

	//transformVertexCamera(float* xi, float* yi, float* zi, int nV, float* camPos_H, float* camRot_H, float* xo, float* yo, float* zo)
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
			cuda_calc2::transformVertexCamera<<<launchConfigCameras.nblocks,launchConfigCameras.nthreads/*, 0, *(p_rtl->cudaStream[j])*/>>>(
															p_camera->pcl.d_x,
															p_camera->pcl.d_y,
															p_camera->pcl.d_z,
															p_camera->pcl.nV,
															samplePosOffet,
															sampleRotOffet,
															p_rtl->vertexBuffers[j].d_vx,
															p_rtl->vertexBuffers[j].d_vy,
															p_rtl->vertexBuffers[j].d_vz
														);


			cudaStatus = cudaGetLastError();
			if (cudaStatus != cudaSuccess) {
				fprintf(stderr, "transformVertexCamera stream %d camera %d launch failed: %s\n", i,j,cudaGetErrorString(cudaStatus));
				IO::waitForEnter();
			}

			cudaStatus = cudaDeviceSynchronize();
			if (cudaStatus != cudaSuccess) {
				fprintf(stderr, "cudaDeviceSynchronize returned error code after launching transformVertexCamera: %s\n", cudaGetErrorString(cudaStatus));
				fprintf(stderr, "position: %d\t rotation: %d\n", indexPos, indexRot);
				IO::waitForEnter();

			}
		}
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code after launching transformVertexCamera: %s\n", cudaGetErrorString(cudaStatus));
		fprintf(stderr, "position: %d\t rotation: %d\n", indexPos, indexRot);
		IO::waitForEnter();

	}

	//raytraceVerticesCamera(				float* xi, float* yi, float* zi,
	//														int* fx, int* fy, int* fz, int nF,														
	//														float* camPos_H, float* camRot_H,
	//														float* camRayX, float* camRayY, float* camRayZ,
	//														float* Dx, float* Dy, float* Dz)


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
			cuda_calc2::raytraceVerticesCamera<<<p_camera->ssnBlocks,p_camera->ssnThreads/*, 0, *(p_rtl->cudaStream[j])*/>>>(
															p_rtl->vertexBuffer.d_vx,
															p_rtl->vertexBuffer.d_vy,
															p_rtl->vertexBuffer.d_vz,																										
															p_rtl->vertexBuffer.d_fx,
															p_rtl->vertexBuffer.d_fy,
															p_rtl->vertexBuffer.d_fz,													
															p_rtl->vertexBuffer.nF,
															samplePosOffet,
															sampleRotOffet,
															p_camera->d_ss_x,
															p_camera->d_ss_y,
															p_camera->d_ss_z,
															p_rtl->depthBuffers[j].d_ss_dx,
															p_rtl->depthBuffers[j].d_ss_dy,
															p_rtl->depthBuffers[j].d_ss_dz);


			cudaStatus = cudaGetLastError();
			if (cudaStatus != cudaSuccess) {
				fprintf(stderr, "raytraceVerticesCamera stream %d camera %d launch failed: %s\n", i,j,cudaGetErrorString(cudaStatus));
				IO::waitForEnter();
			}

			cudaStatus = cudaDeviceSynchronize();
			if (cudaStatus != cudaSuccess) {
				fprintf(stderr, "cudaDeviceSynchronize returned error code after launching raytraceVerticesCamera: %s\n", cudaGetErrorString(cudaStatus));
				fprintf(stderr, "position: %d\t rotation: %d\n", indexPos, indexRot);
				IO::waitForEnter();

			}

		}
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code after launching raytraceVerticesCamera: %s\n", cudaGetErrorString(cudaStatus));
		fprintf(stderr, "position: %d\t rotation: %d\n", indexPos, indexRot);
		IO::waitForEnter();

	}






}

void CF2::rayTrace()
{


	int indexPos;
	int indexRot;



	float* samplePosOffet;
	float* sampleRotOffet;

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

			//printf("%i\t%i\t%i\t%i\n", i, j, indexPos, indexRot); 
			
			samplePosOffet = samplePointsBuffer.d_H	+indexPos*NUMELEM_H;
			sampleRotOffet = sampleRotations.d_R	+indexRot*NUMELEM_H;

			p_camera = p_rtl->cams[j];		
			
			cuda_calc2::raytraceBox<<<1,MAX_RAYTRACE_BOX/*, 0, *(p_rtl->cudaStream[j])*/>>>(
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

			cudaStatus = cudaDeviceSynchronize();
			if (cudaStatus != cudaSuccess) {
				fprintf(stderr, "cudaDeviceSynchronize returned error code after launching raytraceBox: %s\n", cudaGetErrorString(cudaStatus));
				fprintf(stderr, "position: %d\t rotation: %d\n", indexPos, indexRot);
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




	//checking bounding box input
	//bool* p1 = optiSession.launchs[0].depthBuffers[0].bb_hit;
	//bool* p2 = optiSession.launchs[1].depthBuffers[0].bb_hit;
	//CudaMem::cudaMemCpyReport(p1, optiSession.launchs[0].depthBuffers[0].d_bb_hit, boundingBoxBuffer.nBB*sizeof(bool), cudaMemcpyDeviceToHost);
	//CudaMem::cudaMemCpyReport(p2, optiSession.launchs[1].depthBuffers[0].d_bb_hit, boundingBoxBuffer.nBB*sizeof(bool), cudaMemcpyDeviceToHost);


	//for(int i=0; i<optiSession.launchs[0].cams[0]->ssnRays; i++)
	//{
	//	if
	//}


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
			//, 0, *(optiSession.launchs[i].cudaStream[j]
			cuda_calc2::raytraceVertices<<<p_camera->ssnBlocks,p_camera->ssnThreads>>>(
															vertexBuffer.d_vx,
															vertexBuffer.d_vy,
															vertexBuffer.d_vz,																										
															vertexBuffer.d_fx,
															vertexBuffer.d_fy,
															vertexBuffer.d_fz,													
															vertexBuffer.nF,
															optiSession.launchs[i].cams[j]->d[0],
															optiSession.launchs[i].cams[j]->d[1],
															samplePosOffet,
															sampleRotOffet,
															optiSession.launchs[i].cams[j]->d_ss_x,
															optiSession.launchs[i].cams[j]->d_ss_y,
															optiSession.launchs[i].cams[j]->d_ss_z,
															optiSession.launchs[i].depthBuffers[j].d_ss_dx,
															optiSession.launchs[i].depthBuffers[j].d_ss_dy,
															optiSession.launchs[i].depthBuffers[j].d_ss_dz,
															vertexBuffer.d_f_bbi,
															optiSession.launchs[i].depthBuffers[j].d_bb_hit,
															boundingBoxBuffer.nBB,
															humanBB, i);


			cudaStatus = cudaGetLastError();
			if (cudaStatus != cudaSuccess) {
				fprintf(stderr, "raytraceVertices stream %d camera %d launch failed: %s\n", i,j,cudaGetErrorString(cudaStatus));
				IO::waitForEnter();
			}

			//cudaStatus = cudaStreamSynchronize(*(optiSession.launchs[i].cudaStream[j]));
			//if (cudaStatus != cudaSuccess) {
			//	fprintf(stderr, "cudaDeviceSynchronize returned error code after launching raytraceVertices: %s\n", cudaGetErrorString(cudaStatus));
			//	fprintf(stderr, "position: %d\t rotation: %d\n", indexPos, indexRot);
			//	IO::waitForEnter();

			//}
			
			cudaStatus = cudaDeviceSynchronize();
			if (cudaStatus != cudaSuccess) {
				fprintf(stderr, "cudaDeviceSynchronize returned error code after launching raytraceVertices: %s\n", cudaGetErrorString(cudaStatus));
				fprintf(stderr, "position: %d\t rotation: %d\n", indexPos, indexRot);
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


	//checking
	



	rayTraceCameras();

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
			cuda_calc2::mergeSuperSampling<<<p_camera->nBlocks,p_camera->nThreads/*, 0, *(p_rtl->cudaStream[j])*/>>>(
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



			cudaStatus = cudaGetLastError();
			if (cudaStatus != cudaSuccess) {
				fprintf(stderr, "raytraceVertices stream %d camera %d launch failed: %s\n", i,j,cudaGetErrorString(cudaStatus));
				IO::waitForEnter();
			}

			cudaStatus = cudaDeviceSynchronize();
			if (cudaStatus != cudaSuccess) {
				fprintf(stderr, "cudaDeviceSynchronize returned error code after launching mergeSuperSampling: %s\n", cudaGetErrorString(cudaStatus));
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
	for(int j=0;j<optiSession.n; j++)
	{
		int size = optiSession.launchs[j].depthBuffer.size;
		float vx = 0.0f;
		float vy = 0.0f;
		float vz = 0.0f;
		int w = 0;
		for(int i=0; i<size; i++)
		{
			if(!IO::is_nan(x[j*size+i]))
			{
				vx += x[j*size+i];
				vy += y[j*size+i];
				vz += z[j*size+i];
				w++;
			}
		}
		vx = vx/w;
		vy = vy/w;
		vz = vz/w;
		printf("centroid: [%.6f  %.6f  %.6f]\t%d\n", vx, vy, vz, w);
	}
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
		cuda_calc2::calcMiddlePoint<<<1,AVG_BUFFER_SIZE/*, 0, *(p_rtl->cudaStream[0])*/>>>(
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

		cudaStatus = cudaDeviceSynchronize();
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching calcMiddlePoint!\n", cudaStatus);
			IO::waitForEnter();
		}
		
	}

			
	//double c1, c2;
	//CudaMem::cudaMemCpyReport(&c1, optiSession.launchs[0].centroid.d_cx, sizeof(double), cudaMemcpyDeviceToHost);
	//CudaMem::cudaMemCpyReport(&c2, optiSession.launchs[1].centroid.d_cx, sizeof(double), cudaMemcpyDeviceToHost);

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

	//#pragma omp parallel for
	for(int i=0; i<optiSession.n; i++)
	{
		p_rtl = &optiSession.launchs[i];
		optiSession.ecs[i]->cluster();
		//printf("parallel sesssion %d\n", i);
		//CudaMem::cudaMemCpyReport(p_rtl->depthBuffer.d_cis, p_rtl->depthBuffer.cis, p_rtl->depthBuffer.size*sizeof(int), cudaMemcpyHostToDevice);
	}

	for(int i=0; i<optiSession.n; i++)
	{
		p_rtl = &optiSession.launchs[i];
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
		cuda_calc2::distanceToEllipseModel<<<launchConfigModelFitting.nblocks,launchConfigModelFitting.nthreads/*, 0, *(p_rtl->cudaStream[0])*/>>>(
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
		
		cudaStatus = cudaDeviceSynchronize();
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching distanceToEllipseModel!\n", cudaStatus);
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

	probResult->p = new double[n*session];
	probResult->maxp = new double[session];
	probResult->maxp_normalized = new double[session];

#ifndef EVALUATE
	resultSolution.nC = currentNumberOfCams;
	if(resultingSolution.cameraTypes != NULL)
		delete resultingSolution.cameraTypes;
	if(resultingSolution.nC != NULL)
		delete resultingSolution.pclIndex;
	if(resultingSolution.angleIndex != NULL)
		delete resultingSolution.angleIndex;

	resultingSolution.minCost = 0.0;
	resultingSolution.pclIndex = new int[currentNumberOfCams];
	resultingSolution.angleIndex = new int[currentNumberOfCams];
	resultingSolution.cameraTypes = new int[currentNumberOfCams];
#endif


	CudaMem::cudaMemAllocReport((void**)&probResult->d_p, n*session*sizeof(double));
	CudaMem::cudaMemAllocReport((void**)&probResult->d_maxp, session*sizeof(double));

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
	delete probResult->maxp_normalized;

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



void CF2::calculateMaxProb()
{

	//calculateMaxProb(float* prob, int n, float* maxp)

	cudaError_t cudaStatus;	

	RAYTRACING_LAUNCH* p_rtl;
	
	for(int i=0; i<optiSession.n; i++)
	{
		p_rtl = &optiSession.launchs[i];
		cuda_calc2::calculateMaxProb<<<1,MAX_BUFFER_SIZE/*,0, *(p_rtl->cudaStream[0])*/>>>(
														p_rtl->probResult.d_p,
														p_rtl->probResult.n,
														p_rtl->probResult.d_maxp,
														currentTrans.d_pr);
		

					

		cudaStatus = cudaGetLastError();
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "calculateMaxProb stream %d launch failed: %s\n", i,cudaGetErrorString(cudaStatus));
			IO::waitForEnter();
		}

		cudaStatus = cudaDeviceSynchronize();
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching calcMiddlePoint!\n", cudaStatus);
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
		cuda_calc2::calculateMinDist<<<1,MAX_BUFFER_SIZE/*,0, *(p_rtl->cudaStream[0])*/>>>(
														p_rtl->probResult.d_d,
														p_rtl->probResult.n,
														p_rtl->probResult.d_maxd,
														currentTrans.d_pr);
		

					

		cudaStatus = cudaGetLastError();
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "calculateMaxProb stream %d launch failed: %s\n", i,cudaGetErrorString(cudaStatus));
			IO::waitForEnter();
		}

		cudaStatus = cudaDeviceSynchronize();
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching calcMiddlePoint!\n", cudaStatus);
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
		cuda_calc2::calculateMinWeight<<<1,MAX_BUFFER_SIZE/*,0, *(p_rtl->cudaStream[0])*/>>>(
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

		cudaStatus = cudaDeviceSynchronize();
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching calcMiddlePoint!\n", cudaStatus);
			IO::waitForEnter();
		}
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching calcMiddlePoint!\n", cudaStatus);
		IO::waitForEnter();
	}

#ifdef _DEBUG
	checkIntermediateResults();
#endif

	//instead copying the complete memory down to host	
	CudaMem::cudaMemCpyReport(probResult.maxp, probResult.d_maxp, probResult.nmax*		sizeof(double), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(probResult.maxd, probResult.d_maxd, probResult.nmax*		sizeof(float), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(probResult.maxw, probResult.d_maxw, probResult.nmax*		sizeof(int),	cudaMemcpyDeviceToHost);

	//adding the normalized weighted value to the normalized prob
	for(int i=0; i<probResult.nmax; i++)
	{
		
		probResult.maxp_normalized[i] += ((double)(*currentTrans.p_pr))*probResult.maxp[i];
		assert(probResult.maxp[i] <= 1.0 /*&& probResult.maxp_normalized[i] <= 1.0*/);
		
	}

	
}

void CF2::normalizeMaxProb()
{


	for(int i=0; i<probResult.nmax; i++)
	{
		//assert(probResult.maxp_normalized[i] <= 1.0);
		probResult.maxp_normalized[i] /= samplePositions.sumAllPriorities;		
		//for(int cam=0; cam<currentNumberOfCams; cam++)
		//{
		//	printf("%d\t%d\t", optiSession.pI[i*currentNumberOfCams+cam], optiSession.aI[i*currentNumberOfCams+cam]);
		//}
		//printf("%.10lf\n", probResult.maxp_normalized[i]);
		
		if(resultingSolution.minCost < probResult.maxp_normalized[i])
		{
			resultingSolution.minCost = probResult.maxp_normalized[i];
			for(int j=0; j<currentNumberOfCams; j++)
			{

				resultingSolution.cameraTypes[j] = cameraCombination.vector[j];
				resultingSolution.pclIndex[j] = optiSession.pI[i*currentNumberOfCams+j];
				resultingSolution.angleIndex[j] = optiSession.aI[i*currentNumberOfCams+j];
			}
		}
	}




}
