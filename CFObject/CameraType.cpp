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
			const float const* ssz,
			Link link)
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

	this->link = link;
}

CameraType::CameraType(CFIO::SAMPLE_CAMERA* pC)
{
		
		PCL p(pC->pcl.nV, 
				pC->pcl.nF,
				pC->pcl.x, 
				pC->pcl.y,
				pC->pcl.z,
				pC->pcl.fx, 
				pC->pcl.fy,
				pC->pcl.fz);
		HomogeneTransformation H(pC->pcl.bb.H);
		BoundingBox bb(H, pC->pcl.bb.d[0], pC->pcl.bb.d[1], pC->pcl.bb.d[2]);
		link.addPCL(p);
		link.addBB(bb);

		this->nx = pC->nx;
		this->ny = pC->ny;
		this->ssnx = pC->ssnx*pC->nx;
		this->ssny = pC->ssny*pC->ny;

		this->x = new float[nx * ny];
		this->y = new float[nx * ny];
		this->z = new float[nx * ny];
		memcpy(this->x, pC->x, nx*ny*sizeof(float));
		memcpy(this->y, pC->y, nx*ny*sizeof(float));
		memcpy(this->z, pC->z, nx*ny*sizeof(float));
	

		this->ssx = new float[ssnx * ssny];
		this->ssy = new float[ssnx * ssny];
		this->ssz = new float[ssnx * ssny];
		memcpy(this->ssx, pC->ssx, ssnx*ssny*sizeof(float));
		memcpy(this->ssy, pC->ssy, ssnx*ssny*sizeof(float));
		memcpy(this->ssz, pC->ssz, ssnx*ssny*sizeof(float));
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

