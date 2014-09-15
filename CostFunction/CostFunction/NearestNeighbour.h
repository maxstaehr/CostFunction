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


	NearestNeighbour(void);
	virtual ~NearestNeighbour(void);

	static void calcNearestNeighbourIndices(int* indices, struct PCL* pcl);
	static float calcDistance(float x1, float y1, float z1, float x2, float y2, float z2);
	static unsigned int calcMin(struct PCL* pcl, float x, float y, float z);
	static void setNearestNeighbour(const int* const indices, int pointIndex, int angleIndex, int* pclIndices, int* angleIndices);
	static void findNextPointIndexAndAngleIndex(const int* const indices, int pointIndex, int angleIndex, enum GRADIENT_DIM *direction,  int* pointIndexReturn, int* angleIndexReturn);
	static int findGradieantIndices(int* x, int* y, int* z);
	static void findGradienDirection(const double * const costs, const double* const cost_t1, enum GRADIENT_DIM direction[6]);
	static void setNextIteration(double cm, double cp, double c_t1, unsigned char dim, const int* const indices, int pointIndex, int angleIndex, int* pointIndexReturn, int* angleIndexReturn, enum GRADIENT_DIM *dir_return);

	static int xm[];
	static int xz[];
	static int xp[];

	static int ym[];
	static int yz[];
	static int yp[];

	static int zm[];
	static int zz[];
	static int zp[];


};

