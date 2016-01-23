#include "Cluster.h"

#include <iostream>
#include <assert.h>

double calculateProbabilityOfDetection(float meanDistance);
double calculateWeightOfDetection(int w);

Cluster::Cluster():x(NULL), y(NULL), z(NULL), maxSize(0), prob(0.0), meanDist(0.0f)
{
	
}

Cluster::Cluster(const Cluster& rhs)
{
	maxSize = rhs.maxSize;
	x = new float[maxSize];
	y = new float[maxSize];
	z = new float[maxSize];
	memcpy(x, rhs.x, sizeof(float)*maxSize);
	memcpy(y, rhs.y, sizeof(float)*maxSize);
	memcpy(z, rhs.z, sizeof(float)*maxSize);
	memcpy(centroid, rhs.centroid, sizeof(float)*3);
}


void Cluster::setX(int i, float v)
{
	assert(i >= 0 && i < maxSize);
	x[i] = v;
}
void Cluster::setY(int i, float v)
{
	assert(i >= 0 && i < maxSize);
	y[i] = v;

}
void Cluster::setZ(int i, float v)
{
	assert(i >= 0 && i < maxSize);
	z[i] = v;

}

Cluster::Cluster(int size)
{
	maxSize = size;
	this->x = new float[size];
	this->y = new float[size];
	this->z = new float[size];
}
void Cluster::operator=(const Cluster& rhs )
{
	if(x != NULL)
		delete x;
	if(y != NULL)
		delete y;
	if(z != NULL)
		delete z;
	maxSize = rhs.maxSize;
	x = new float[maxSize];
	y = new float[maxSize];
	z = new float[maxSize];
	memcpy(x, rhs.x, sizeof(float)*maxSize);
	memcpy(y, rhs.y, sizeof(float)*maxSize);
	memcpy(z, rhs.z, sizeof(float)*maxSize);
	memcpy(centroid, rhs.centroid, sizeof(float)*3);
}

Cluster::~Cluster(void)
{
	if(x != NULL)
		delete x;
	if(y != NULL)
		delete y;
	if(z != NULL)
		delete z;
}

void Cluster::transform(Transformation2D t)
{
	float xtemp, ytemp;
	for(int i=0; i<maxSize; i++)
	{
		xtemp = x[i];
		ytemp = y[i];
		x[i] = t.getR()[0]*xtemp + t.getR()[1]*ytemp + t.getv()[0];
		y[i] = t.getR()[2]*xtemp + t.getR()[3]*ytemp + t.getv()[1];			
	}
}

void Cluster::calculateCentroid()
{
	centroid[0] = 0.0f;
	centroid[1] = 0.0f;
	centroid[2] = 0.0f;

	for(int i=0; i< maxSize; i++)
	{
		centroid[0] += x[i];
		centroid[1] += y[i];
		centroid[2] += z[i];
	}

	centroid[0] /= (float)maxSize;
	centroid[1] /= (float)maxSize;
	centroid[2] /= (float)maxSize;
}

void Cluster::calculate()
{

	calculateCentroid();
	meanDist = 0.0f;

	for(int i=0; i< maxSize; i++)
	{
		meanDist += distanceFromModel(x[i], y[i], z[i]);
	}

	meanDist /= (float)maxSize;
	double detectionProb = calculateProbabilityOfDetection(meanDist);
	double detectionWeight = calculateWeightOfDetection(maxSize);
	prob = detectionProb;//*detectionWeight;
	
}

float Cluster::distanceFromModel(float x, float y, float z)
{
	z -= ELLIPSE_PARAM_Z;

	float t1 = powf((x/ELLIPSE_PARAM_X),	2.0f);
	float t2 = powf((y/ELLIPSE_PARAM_Y),	2.0f);
	float t3 = powf((z/ELLIPSE_PARAM_Z),	2.0f);
	float sum = t1 + t2 + t3;
	float sqrtSum = sqrt(sum);
	float tx = x/sqrtSum;
	float ty = y/sqrtSum;
	float tz = z/sqrtSum;
	float dist = sqrt(powf(tx-x,2.0f) + powf(ty-y,2.0f) + powf(tz-z,2.0f));
	return dist;
}
//utility functions
#define	tailor_a (2.055792709094123)
#define	tailor_b (-13.314579265182624)
#define tailor_min (0.054125744849443)

#define tailor_weight_a (0.056234132519035)
#define tailor_weight_b (0.028782313662426)
#define tailor_weight_max 100


double calculateProbabilityOfDetection(float meanDistance)
{	
	if(meanDistance < tailor_min){		
		return 1.0;
	}else{
		return tailor_a*exp(tailor_b*meanDistance);
	}
}

double calculateWeightOfDetection(int w)
{
	if(w > tailor_weight_max){			
		return 1.0;
	}else{
		return tailor_weight_a*exp(tailor_weight_b*w);
	}
}