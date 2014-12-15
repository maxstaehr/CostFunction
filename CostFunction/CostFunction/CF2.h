
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
	SAMPLE_PCL			samplePoints;
	SAMPLE_ROTATIONS	sampleRotations;
	SAMPLE_CAMERA		sampleCamera;
	
};

#endif