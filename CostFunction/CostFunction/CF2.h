
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
	ROBOT_PCL				robot;
	HUMAN_PCL				human;
	ENVIRONMENT_PCL			environment;
	SAMPLE_POSITIONS		samplePositions;
	SAMPLE_PCL				samplePoints;
	SAMPLE_ROTATIONS		sampleRotations;
	POSSIBLE_CAMERA_TYPES	sampleCameraTypes;

	//vertex buffer
	VERTEX_BUFFER		vertexBuffer;
	VERTEX_BUFFER		vertexBufferRobot;
	VERTEX_BUFFER		vertexBufferHuman;
	VERTEX_BUFFER		vertexBufferEnvironment;

	//bounding box buffer
	BB_BUFFER			boundingBoxBuffer;
	BB_BUFFER			boundingBoxBufferRobot;
	BB_BUFFER			boundingBoxBufferEnvironment;

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

	//probability calculation
	PROB_RESULT			probResult;

	//currentTransformations
	CURRENT_TRANS		currentTrans;

	RAYTRACING_LAUNCH*	raytracingLaunchs;
	int					nRaytracingLaunch;

	CAM_COMBINATION		cameraCombination;
	int					currentNumberOfCams;

	OPTIMIZATION_SESSION optiSession;

	cudaStream_t*	cudaStream;

	SearchClass*	sC;

	DISTANCE_MATRIX distMatrix;

	NearestNeighbour* nn;

	float* debug_distance_buffer;
	float* debug_propability_buffer;




	//init functions
	void initVertexBuffer();
	void initBoundingBoxBuffer();
	void initSamplePointsBuffer();

	void initDepthBuffer(DEPTH_BUFFER* depthBuffer, int size, int ss_size);	
	void initCentroidBuffer(CENTROID* centroid, int n);
	void initPropBuffer(PROB_RESULT* probResult, int n, int session);
	void clearPropBuffer(PROB_RESULT* probResult, int n);
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
	void setCurrentTans(int i);
	
	void rayTrace();
	void calculateCentroid();
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
	
	
};

#endif