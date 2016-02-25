#pragma once

#include "global.h"
#include "Transformation2D.h"

class CFOBJECT_EXPORT Cluster
{
public:

	//const value_type& val
	Cluster();
	Cluster(const Cluster& rhs);
	Cluster(int size);
	~Cluster(void);

	void setX(int i, float value);
	void setY(int i, float value);
	void setZ(int i, float value);
	void operator=(const Cluster& rhs );


	int getMaxSize(){return maxSize;}
	const float* getX(){return x;}
	const float* getY(){return y;}
	const float* getZ(){return z;}
	const float* getCentroid(){return centroid;}
	double getHumanProb(){return prob;}
	float getMeanDistance2Model(){return meanDist;}

	void transform(Transformation2D transform);
	
	void calculate();
	void calculateCentroid();

private:
	float distanceFromModel(float x, float y, float z);
	float* x;
	float* y;
	float* z;
	int maxSize;

	double prob;
	float meanDist;

	float *centroid;
	
};

