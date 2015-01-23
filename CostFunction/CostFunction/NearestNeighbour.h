#pragma once

#include "struct_definitions.h"
class NearestNeighbour
{


public:

	enum GRADIENT_DIM
	{
		plus,
		zero,
		minus
	};


	NearestNeighbour(SAMPLE_PCL* sp, SAMPLE_ROTATIONS* sr);
	virtual ~NearestNeighbour(void);

	void calcNearestNeighbourIndices();
	float calcDistance(float x1, float y1, float z1, float x2, float y2, float z2);
	unsigned int calcMin(float x, float y, float z);
	void setNearestNeighbour(const int* const indices, int pointIndex, int angleIndex, int* pclIndices, int* angleIndices);
	void findNextPointIndexAndAngleIndex(const int* const indices, int pointIndex, int angleIndex, enum GRADIENT_DIM *direction,  int* pointIndexReturn, int* angleIndexReturn);
	int findGradieantIndices(int* x, int* y, int* z);
	void findGradienDirection(const double * const costs, const double* const cost_t1, enum GRADIENT_DIM direction[6]);
	void setNextIteration(double cm, double cp, double c_t1, unsigned char dim, const int* const indices, int pointIndex, int angleIndex, int* pointIndexReturn, int* angleIndexReturn, enum GRADIENT_DIM *dir_return);

	static int xm[];
	static int xz[];
	static int xp[];

	static int ym[];
	static int yz[];
	static int yp[];

	static int zm[];
	static int zz[];
	static int zp[];


	int* getNN(){return indices;}

private: 
	SAMPLE_PCL* pcl;
	SAMPLE_ROTATIONS* sr;
	SAMPLE_PCL* spp;
	int* indices;


};

