#include "PCL.h"
#include <string.h>

PCL::PCL(int nV, int nF,  const float const * x, 
						 const float const * y,
						 const float const * z,
						 const int const * fx, 
						 const int const * fy,
						 const int const * fz)
{
	this->nV = nV;
	this->nF = nF;
	this->x = new float[nV];
	this->y = new float[nV];
	this->z = new float[nV];

	this->fx = new int[nF];
	this->fy = new int[nF];
	this->fz = new int[nF];

	memcpy(this->x, x, sizeof(float)*nV);
	memcpy(this->y, y, sizeof(float)*nV);
	memcpy(this->z, z, sizeof(float)*nV);

	memcpy(this->fx, fx, sizeof(int)*nF);
	memcpy(this->fy, fy, sizeof(int)*nF);
	memcpy(this->fz, fz, sizeof(int)*nF);
}

void PCL::rotate(Rotation rot)
{
	float xb, yb, zb;
	for(int i=0; i<nV; i++)
	{
		xb = x[i];
		yb = y[i];
		zb = z[i];

		x[i] = xb*rot.V()[0] + yb*rot.V()[1] + zb*rot.V()[2];
		y[i] = xb*rot.V()[3] + yb*rot.V()[4] + zb*rot.V()[5];
		z[i] = xb*rot.V()[6] + yb*rot.V()[7] + zb*rot.V()[8];
	}
}
void PCL::transform(HomogeneTransformation trans)
{
	float xb, yb, zb;
	for(int i=0; i<nV; i++)
	{
		xb = x[i];
		yb = y[i];
		zb = z[i];

		x[i] = xb*trans.V()[0] + yb*trans.V()[1] + zb*trans.V()[2] + trans.V()[3];
		y[i] = xb*trans.V()[4] + yb*trans.V()[5] + zb*trans.V()[6] + trans.V()[7];
		z[i] = xb*trans.V()[8] + yb*trans.V()[9] + zb*trans.V()[10] + trans.V()[11];
	}

}


PCL::~PCL(void)
{
	delete x;
	delete y;
	delete z;
	delete fx;
	delete fy;
	delete fz;
}


