#pragma once

#include "struct_definitions.h"
class NearestNeighbour
{
public:
	NearestNeighbour(void);
	virtual ~NearestNeighbour(void);

	static void calcNearestNeighbourIndices(unsigned int* indices, struct PCL* pcl);
	static float calcDistance(float x1, float y1, float z1, float x2, float y2, float z2);
	static unsigned int calcMin(struct PCL* pcl, float x, float y, float z);
	static void setNearestNeighbour(const int* const indices, int pointIndex, int angleIndex, int* pclIndices, int* angleIndices);
};

