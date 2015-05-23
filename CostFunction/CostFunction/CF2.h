
#ifndef CF2_H_
#define CF2_H_


#include "cuda_runtime.h"
#include "struct_definitions.h"
#include "SimulatedAnnealing.h"
#include "time.h"
#include <iostream>
#include <string>
#include "global.h"
#include "SearchClass.h"
#include "NearestNeighbour.h"


class CF2 {
public:
	CF2();

private:
	PCL						robot;
	PCL						human;
	PCL						environment;
	SAMPLE_POSITIONS		samplePositions;
	SAMPLE_PCL				samplePoints;
	SAMPLE_ROTATIONS		sampleRotations;
	VALID_POS				sampleValidPositions;
	POSSIBLE_CAMERA_TYPES	sampleCameraTypes;

	//vertex buffer
	VERTEX_BUFFER		vertexBuffer;
	VERTEX_BUFFER		vertexBufferRobot;
	VERTEX_BUFFER		vertexBufferHuman;
	VERTEX_BUFFER		vertexBufferEnvironment;
	VERTEX_BUFFER		vertexBufferCamera;
	

	//bounding box buffer
	BB_BUFFER			boundingBoxBuffer;
	BB_BUFFER			boundingBoxBufferRobot;
	BB_BUFFER			boundingBoxBufferEnvironment;
	BB_BUFFER			boundingBoxBufferHuman;
	
	RESULT_SOLUTION		resultSolution;

	//depth buffer
	DEPTH_BUFFER		depthBuffer;

	//sample points buffer
	SAMPLE_POINTS_BUFFER	samplePointsBuffer;

	//centroid
	CENTROID			centroid;

	//sample fitting human cloud into model
	SAMPLE_FITTING		sampleFitting;

	//launch configuration for model fitting
	LAUNCH_CONFIG		launchConfigModelFitting;

	//launch configuration for distance matrix
	LAUNCH_CONFIG		launchConfigDistanceMatrix;

	//launch configuration for max cluster
	LAUNCH_CONFIG		launchConfigMaxCluster;

	//launch config rayTrace Cameras
	LAUNCH_CONFIG		launchConfigCameras;

	//probability calculation
	PROB_RESULT			probResult;

	//currentTransformations
	CURRENT_TRANS		currentTrans;

	RAYTRACING_LAUNCH*	raytracingLaunchs;
	int					nRaytracingLaunch;

	CAM_COMBINATION		cameraCombination;
	int					currentNumberOfCams;

	
	int					humanBB;
	int					currentCameraConfig;

	OPTIMIZATION_SESSION optiSession;

	cudaStream_t*	cudaStream;

	SearchClass*	sC;

	DISTANCE_MATRIX distMatrix;

	NearestNeighbour* nn;

	float* debug_distance_buffer;
	float* debug_propability_buffer;

	RESULT_SOLUTION		resultingSolution;

	void getCurrentPA_Index(int *const pI, int* const aI, int cam);





	//init functions
	void initVertexBuffer();
	void initBoundingBoxBuffer();
	void freeVertexBuffer();
	void freeBoundingBoxBuffer();
	void reinitLoop();
	void overlayData();
	void checkFirstTwoCameras();
	


	void initSamplePointsBuffer();

	void initDepthBuffer(DEPTH_BUFFER* depthBuffer, int nSessions, int nCam, int size, int ss_size);	
	void initCentroidBuffer(CENTROID* centroid, int n);
	void initPropBuffer(PROB_RESULT* probResult, int n, int session);

	void initCameraVertexBuffer(VERTEX_BUFFER* vertexBuffer, int sessions);
	void freeCameraVertexBuffer(VERTEX_BUFFER* vertexBuffer);

	void normalizeMaxProb();



	
	void createCudaStream(cudaStream_t** streams, int n);
	

	void initRadomNumberGenerator(curandState *devStates, SAMPLE_CAMERA* sampleCamera );
	void initRaytracingLaunch();
	void initCameraCombination();
	void iterateCameraCombination();

	void saveAllVertices();
	void printCentroid(struct DEPTH_BUFFER* depth);
	

	

	void transformVertexBuffer();
	void transformBoundingBoxBuffer();
	void transformSamplePointBuffer();
	void transformCameras();
	void setCurrentTans(int i);
	
	void rayTrace();
	void rayTraceCameras();
	void calculateCentroid();
	void calculateCluster();
	void calculateProbOfHumanDetection();
	void calculateMaxProb();

	void initParallelOptiRuns();
	void freeParallelOptiRuns();

	void freeDepthBuffer(DEPTH_BUFFER* depthBuffer);
	void freeCentroidBuffer(CENTROID* centroid);
	void freePropBuffer(PROB_RESULT* probResult);
	void freeCudaStream(cudaStream_t* streams,int n);

	void zeroProb();
	void normalizeProb();

	void checkIntermediateResults();

public:
	void run();
	void run_completeEnumeration();
	void run_evaluation();
	
	
};

#endif