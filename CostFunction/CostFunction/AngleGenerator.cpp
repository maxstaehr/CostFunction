#include "AngleGenerator.h"
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <assert.h>
#include <math.h>
#include <time.h>


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
		if(w[r] > 0)
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
	
	assert(nOfLimits > 0 && abs(limits[nOfLimits-1] - 1.0f) < 1.e-4);
	delete w;
}

bool AngleGenerator::checkIndex(int i, float value)
{
	if (i>0)
	{
		return  value < limits[i] &&  value >= limits[i-1];
	}else
	{
		return value < limits[0];
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
	assert(index>=0 && index<nOfLimits );
	return indices[index];
}


AngleGenerator::~AngleGenerator(void)
{
	delete limits;
	delete indices;
}
