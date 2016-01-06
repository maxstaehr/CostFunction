#include "CameraType.h"

#include <string>

CameraType::CameraType(	
			int nx, 
			int ny,
			const float const* x,
			const float const* y,
			const float const* z,
			int ssnx,
			int ssny,
			const float const* ssx,
			const float const* ssy,
			const float const* ssz)
{
	this->nx = nx;
	this->ny = ny;
	this->ssnx = ssnx;
	this->ssny = ssny;

	this->x = new float[nx * ny];
	this->y = new float[nx * ny];
	this->z = new float[nx * ny];
	memcpy(this->x, x, nx*ny*sizeof(float));
	memcpy(this->y, y, nx*ny*sizeof(float));
	memcpy(this->z, z, nx*ny*sizeof(float));
	

	this->ssx = new float[nx * ny];
	this->ssy = new float[nx * ny];
	this->ssz = new float[nx * ny];
	memcpy(this->ssx, ssx, nx*ny*sizeof(float));
	memcpy(this->ssy, ssy, nx*ny*sizeof(float));
	memcpy(this->ssz, ssz, nx*ny*sizeof(float));
}


CameraType::~CameraType(void)
{
	delete x;
	delete y;
	delete z;
	delete ssx;
	delete ssy;
	delete ssz;
}

