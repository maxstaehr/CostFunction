
#ifndef CF2_H_
#define CF2_H_


#include "cuda_runtime.h"
#include "struct_definitions.h"
#include "SimulatedAnnealing.h"
#include "time.h"
#include <iostream>
#include <string>
#include "global.h"


class CF2 {
public:
	CF2();

private:
	ROBOT_PCL			robot;
	HUMAN_PCL			human;
	ENVIRONMENT_PCL		environment;
	SAMPLE_POSITIONS	samplePositions;
	SAMPLE_PCL			samplePoints;
	SAMPLE_ROTATIONS	sampleRotations;
	SAMPLE_CAMERA		sampleCamera;

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





	//init functions
	void initVertexBuffer();
	void initBoundingBoxBuffer();
	void initDepthBuffer(int size);
	void initSamplePointsBuffer();
	void initCentroidBuffer();
	void initPropBuffer(int n);
	void clearPropBuffer();

	void transformVertexBuffer(int i);
	void transformBoundingBoxBuffer(int i);
	void transformSamplePointBuffer(int i);
	
	void rayTrace();
	void calculateCentroid();
	void calculateProbOfHumanDetection();
	void calculateMaxProb();
	
	
};

#endif