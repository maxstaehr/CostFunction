#include "NearestNeighbour.h"


NearestNeighbour::NearestNeighbour(void)
{
}


NearestNeighbour::~NearestNeighbour(void)
{
}

float NearestNeighbour::calcDistance(float x1, float y1, float z1, float x2, float y2, float z2)
{
	float t1, t2, t3;
	t1 = pow(x1-x2, 2);
	t2 = pow(y1-y2, 2);
	t3 = pow(z1-z2, 2);
	return sqrtf(t1+t2+t3);

}

unsigned int NearestNeighbour::calcMin(struct PCL* pcl, float x, float y, float z)
{
	float minCost=FLT_MAX, cost;
	unsigned int minIndex= 0;
	for(unsigned int i=0; i< pcl->n; i++)
	{
		cost = calcDistance(pcl->x[i],
							pcl->y[i],
							pcl->z[i],
							x,
							y,
							z);
		if(cost < minCost)
		{
			minCost = cost;
			minIndex = i;
		}
	}
	return minIndex;
}

void NearestNeighbour::calcNearestNeighbourIndices(unsigned int* indices, struct PCL* pcl)
{
	
	float eps = 0.05;
	for(unsigned int r=0; r < pcl->n; r++)
	{
		indices[6*r+0] = calcMin(pcl, pcl->x[r]-eps, pcl->y[r], pcl->z[r]);
		indices[6*r+1] = calcMin(pcl, pcl->x[r]+eps, pcl->y[r], pcl->z[r]);
		indices[6*r+2] = calcMin(pcl, pcl->x[r], pcl->y[r]-eps, pcl->z[r]);
		indices[6*r+3] = calcMin(pcl, pcl->x[r], pcl->y[r]+eps, pcl->z[r]);
		indices[6*r+4] = calcMin(pcl, pcl->x[r], pcl->y[r], pcl->z[r]-eps);
		indices[6*r+5] = calcMin(pcl, pcl->x[r], pcl->y[r], pcl->z[r]+eps);
	}
}

void NearestNeighbour::setNearestNeighbour(const int* const indices, int pointIndex, int angleIndex, int* pclIndices, int* angleIndices)
{
	//12 possible permutation for one single point

	//first 6 permutations are positions with constant angle
	memcpy(pclIndices, indices+6*pointIndex, 6*sizeof(int));
	for(unsigned int i=0; i<6; i++)
		angleIndices[i] = angleIndex;


	//last 6 permuations are angle changes with const positions
	for(unsigned int i=6; i<12; i++)
		pclIndices[i] = pointIndex;

	int ri = angleIndex/100;
	int pi = (angleIndex - ri*100)/10;
	int yi = angleIndex-ri*100-pi*10;

	int ri_m = (ri+10-1)%10;
	int ri_p = (ri+10+1)%10;

	int yi_m = (yi+10-1)%10;
	int yi_p = (yi+10+1)%10;

	int pi_m = (pi+10-1)%10;
	int pi_p = (pi+10+1)%10;

	angleIndices[6] = ri_m * 100 + pi * 10 + yi;
	angleIndices[7] = ri_p * 100 + pi * 10 + yi;
	angleIndices[8] = ri * 100 + pi_m * 10 + yi;
	angleIndices[9] = ri * 100 + pi_p * 10 + yi;
	angleIndices[10] = ri * 100 + pi * 10 + yi_p;
	angleIndices[11] = ri * 100 + pi * 10 + yi_m;
}
