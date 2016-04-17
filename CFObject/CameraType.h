#pragma once

#include "global.h"
#include "Link.h"
#include "CFIO.h"


class CFOBJECT_EXPORT CameraType
{
public:
	CameraType(	int nx, 
			int ny,
			const float const* x,
			const float const* y,
			const float const* z,
			int ssnx,
			int ssny,
			const float const* ssx,
			const float const* ssy,
			const float const* ssz, 
			Link link);
	CameraType(CFIO::SAMPLE_CAMERA*);

	int getnx(){return nx;}
	int getny(){return ny;}
	const float const* getx(){return x;}
	const float const* gety(){return y;}	
	const float const* getz(){return z;}	

	int getssnx(){return ssnx;}
	int getssny(){return ssny;}
	const float const* getssx(){return ssx;}
	const float const* getssy(){return ssy;}
	const float const* getssz(){return ssz;}

	Link getLink(){return link;};



	

	~CameraType(void);

private:
	int nx;
	int ny;
	float* x;
	float* y;
	float* z;

	int ssnx;
	int ssny;
	float* ssx;
	float* ssy;
	float* ssz;
	Link link;



};

