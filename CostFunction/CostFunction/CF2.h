
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




	//init functions
	void initVertexBuffer();
	void initBoundingBoxBuffer();
	void initDepthBuffer(int size);
	void initSamplePointsBuffer();

	void transformVertexBuffer(int i);
	void transformBoundingBoxBuffer(int i);
	void transformSamplePointBuffer(int i);
	
	void rayTrace();
	
	
};

#endif