#include "AngleGenerator.h"
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <assert.h>
#include <math.h>
#include <time.h>
#include "IO.h"

AngleGenerator::AngleGenerator()
{
	nOfLimits = 2;
	limits = new float[nOfLimits];
	limits[0] = 0.5;
	limits[1] = 1.0;
	indices = new int[nOfLimits];
	indices[0] = 198;
	indices[1] = 198;
}

AngleGenerator::AngleGenerator(float* prop, int nAngle, int DOF)
{
	time_t t;
	time(&t);
	srand((unsigned int)t);              /* Zufallsgenerator initialisieren */

	float* w = new float[nAngle];
	memset(w,0, nAngle*sizeof(float));

	nAng = nAngle;
	for(int r=0; r<nAngle; r++)
	{
		for(int d=0; d<DOF; d++)
		{
			w[r] += prop[d*nAngle+r];
		}
	}

	//now checking non zeros elements and over all sum for normalisation
	float sum = 0.0f;
	nOfLimits = 0;
	for(int r=0; r<nAngle; r++)
	{
		sum += w[r];
		if(w[r] > 0.0f)
		{
			nOfLimits++;
		}
	}

	limits = new float[nOfLimits];
	indices = new int[nOfLimits];

	float offset = 0.0f;
	int currentIndice = 0;
	for(int r=0; r<nAngle; r++)
	{
		if(w[r] > 0)
		{
			offset += w[r]/sum;
			indices[currentIndice] = r;
			limits[currentIndice] = offset;
			currentIndice++;
		}

	}
	printf("Generated %d angles\n", nOfLimits);
	if(nOfLimits == 0 || abs(limits[nOfLimits-1] - 1.0f) > 1.e-4)
	{
		printf("no valid rotations in angle generation found!\n");
		IO::waitForEnter();
	}
	delete w;
}

bool AngleGenerator::checkIndex(int i, float value)
{
	if (i>0)
	{
		return  value <= limits[i] &&  value > limits[i-1];
	}else
	{
		return value <= limits[0];
	}
}

int AngleGenerator::generateRandomAngle()
{
	float value = (float)rand()/RAND_MAX;
	int index = -1;
	for(int i=0; i<nOfLimits; i++)
	{
		if(checkIndex(i,value))
		{
			index = i;
			break;
		}
	}
	int angle = indices[index];
	//printf("angle at %d gene: %d\t%d\n",index, angle,nOfLimits ); 
	assert(index > -1);
	assert(index<nOfLimits);
	assert(angle > -1);
	assert(indices[index] < nAng);                       
	return indices[index];
	//return rand()%nAng;   
}


AngleGenerator::~AngleGenerator(void)
{
	delete limits;
	delete indices;
}
