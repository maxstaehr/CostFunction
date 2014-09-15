#include "NearestNeighbour.h"

#include <algorithm>
#include <assert.h>
#include "global.h"

int NearestNeighbour::xm[] = {6, 15, 24, 7, 16, 25, 8, 17, 26};
int NearestNeighbour::xz[] = {5, 14, 23, 0, 9, 18, 1, 10, 19};
int NearestNeighbour::xp[] = {4, 13, 22, 3, 12, 21, 2, 11, 20};

int NearestNeighbour::ym[] = {8, 17, 26, 1, 10, 19, 2, 11, 20};
int NearestNeighbour::yz[] = {7, 16, 25, 0, 9, 18, 3, 12, 21};
int NearestNeighbour::yp[] = {6, 15, 24, 5, 14, 23, 4, 13, 22};

int NearestNeighbour::zm[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
int NearestNeighbour::zz[] = {9, 10, 11, 12, 13, 14, 15, 16, 17};
int NearestNeighbour::zp[] = {18, 19, 20, 21, 22, 23, 24, 25, 26};

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

void NearestNeighbour::calcNearestNeighbourIndices(int* indices, struct PCL* pcl)
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

	int ri = angleIndex/N_OF_A_SQ;
	int pi = (angleIndex - ri*N_OF_A_SQ)/N_OF_A;
	int yi = angleIndex-ri*N_OF_A_SQ-pi*N_OF_A;

	int ri_m = (ri+N_OF_A-1)%N_OF_A;
	int ri_p = (ri+N_OF_A+1)%N_OF_A;

	int yi_m = (yi+N_OF_A-1)%N_OF_A;
	int yi_p = (yi+N_OF_A+1)%N_OF_A;

	int pi_m = (pi+N_OF_A-1)%N_OF_A;
	int pi_p = (pi+N_OF_A+1)%N_OF_A;

	angleIndices[6] = ri_m * N_OF_A_SQ + pi * N_OF_A + yi;
	angleIndices[7] = ri_p * N_OF_A_SQ + pi * N_OF_A + yi;
	angleIndices[8] = ri * N_OF_A_SQ + pi_m * N_OF_A + yi;
	angleIndices[9] = ri * N_OF_A_SQ + pi_p * N_OF_A + yi;
	angleIndices[10] = ri * N_OF_A_SQ + pi * N_OF_A + yi_p;
	angleIndices[11] = ri * N_OF_A_SQ + pi * N_OF_A + yi_m;
}

int NearestNeighbour::findGradieantIndices(int* x, int* y, int* z)
{
	std::sort (x,x+9);   
	std::sort (y,y+9);
	std::sort (z,z+9);
	std::vector<int> v1(18);
	std::vector<int> v2(18);
	std::vector<int> v3(27);
	std::vector<int>::iterator it;

	it=std::set_intersection (x, x+9, y, y+9, v1.begin());
	v1.resize(it-v1.begin());   

	it=std::set_intersection (y, y+9, z, z+9, v2.begin());
	v2.resize(it-v2.begin());

	std::sort (v1.begin(),v1.end());
	std::sort (v2.begin(),v2.end());

	it=std::set_intersection (v1.begin(), v1.end(), v2.begin(), v2.end(), v3.begin());
	v3.resize(it-v3.begin());

	assert(v3.size() == 1);
	return v3[0];

}

void NearestNeighbour::findGradienDirection(const double * const costs, const double* const costs_t1, enum GRADIENT_DIM direction[6])
{
	//for(unsigned int i=0; i < 6; i++)
	//{
	//	if(costs[i] < costs_t1[i])
	//	{
	//		direction[i] GRADIENT_DIM::
	//	}else if(costs[i] > costs_t1[i])
	//	{
	//	}else
	//	{
	//	}
	//}
}

void NearestNeighbour::setNextIteration(double cm, double cp, double c_t1, unsigned char dim, const int* const indices, int pointIndex, int angleIndex, int* pointIndexReturn, int* angleIndexReturn, enum GRADIENT_DIM *dir_return)
{
	GRADIENT_DIM dir;
	if(cm < c_t1)
	{
		//change to minus
		dir = GRADIENT_DIM::minus;
	}else if(cp < c_t1)
	{
		//change to plus
		dir = GRADIENT_DIM::plus;
	}else
	{
		//no change
		dir = GRADIENT_DIM::zero;
	}
	*dir_return = dir;

	int ri = angleIndex/N_OF_A_SQ;
	int pi = (angleIndex - ri*N_OF_A_SQ)/N_OF_A;
	int yi = angleIndex-ri*N_OF_A_SQ-pi*N_OF_A;
	int roll = ri, pitch = pi, yaw = yi;

	switch(dim){
	case 0:		
	case 1:
	case 2:
		//x,y,z
		switch(dir){
		case GRADIENT_DIM::minus:
			*pointIndexReturn = indices[pointIndex*6+dim*2+0];
			break;
		case GRADIENT_DIM::zero:
			*pointIndexReturn = pointIndex;
			break;
		case GRADIENT_DIM::plus:
			*pointIndexReturn = indices[pointIndex*6+dim*2+1];
			break;
		};		
		break;
	case 3:
		//roll
		switch(dir){
		case GRADIENT_DIM::minus:
			roll = (ri+N_OF_A-1)%N_OF_A;
			break;
		case GRADIENT_DIM::zero:
			roll = ri;
			break;
		case GRADIENT_DIM::plus:
			roll = (ri+N_OF_A+1)%N_OF_A;
			break;
		};
		break;
	case 4:
		//pitch
		switch(dir){
		case GRADIENT_DIM::minus:
			pitch = (pi+N_OF_A-1)%N_OF_A;;
			break;
		case GRADIENT_DIM::zero:
			pitch = pi;
			break;
		case GRADIENT_DIM::plus:
			pitch = (pi+N_OF_A+1)%N_OF_A;;
			break;
		};
		break;
	case 5:
		//yaw
		switch(dir){
		case GRADIENT_DIM::minus:
			yaw = (yi+N_OF_A-1)%N_OF_A;
			break;
		case GRADIENT_DIM::zero:
			yaw = yi;
			break;
		case GRADIENT_DIM::plus:
			yaw = (yi+N_OF_A+1)%N_OF_A;
			break;
		};
		break;

	};
	*angleIndexReturn = roll * N_OF_A_SQ + pitch * N_OF_A + yaw;

}



void NearestNeighbour::findNextPointIndexAndAngleIndex(const int* const indices, int pointIndex, int angleIndex, enum GRADIENT_DIM *direction,  int* pointIndexReturn, int* angleIndexReturn)
{
	//12 possible permutation for one single point
	//int i = 0;
	int *x, *y, *z;
	switch(direction[0]){
	case GRADIENT_DIM::minus:
		x = xm;
		break;
	case GRADIENT_DIM::zero:
		x = xz;
		break;
	case GRADIENT_DIM::plus:
		x = xp;
		break;
	};

	switch(direction[1]){
	case GRADIENT_DIM::minus:
		y = ym;
		break;
	case GRADIENT_DIM::zero:
		y = yz;
		break;
	case GRADIENT_DIM::plus:
		y = yp;
		break;
	};

	switch(direction[2]){
	case GRADIENT_DIM::minus:
		z = zm;
		break;
	case GRADIENT_DIM::zero:
		z = zz;
		break;
	case GRADIENT_DIM::plus:
		z = zp;
		break;
	};
	*pointIndexReturn = NearestNeighbour::findGradieantIndices(x,y,z);


	int ri = angleIndex/N_OF_A_SQ;
	int pi = (angleIndex - ri*N_OF_A_SQ)/N_OF_A;
	int yi = angleIndex-ri*N_OF_A_SQ-pi*N_OF_A;

	int roll, pitch, yaw;
	switch(direction[3]){
	case GRADIENT_DIM::minus:
		roll = (ri+N_OF_A-1)%N_OF_A;
		break;
	case GRADIENT_DIM::zero:
		roll = ri;
		break;
	case GRADIENT_DIM::plus:
		roll = (ri+N_OF_A+1)%N_OF_A;
		break;
	};

	switch(direction[4]){
	case GRADIENT_DIM::minus:
		pitch = (pi+N_OF_A-1)%N_OF_A;
		break;
	case GRADIENT_DIM::zero:
		pitch = pi;
		break;
	case GRADIENT_DIM::plus:
		pitch = (pi+N_OF_A+1)%N_OF_A;
		break;
	};

	switch(direction[5]){
	case GRADIENT_DIM::minus:
		yaw = (yi+N_OF_A-1)%N_OF_A;;
		break;
	case GRADIENT_DIM::zero:
		yaw = yi;
		break;
	case GRADIENT_DIM::plus:
		yaw = (yi+N_OF_A+1)%N_OF_A;;
		break;
	};
	*angleIndexReturn = roll * N_OF_A_SQ + pitch * N_OF_A + yaw;
}