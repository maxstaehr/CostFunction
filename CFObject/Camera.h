#pragma once

#include "global.h"
#include "CameraType.h"
#include "HomogeneTransformation.h"
#include "Link.h"
#include <iostream>
#include <fstream>

class CFOBJECT_EXPORT Camera
{

	friend class ECTestAdapter;
	friend class ECTest;
public:
	Camera(CameraType& camType);
	void saveCurrentState(std::ofstream* stream);
	~Camera(void);

	const float const* getx(){return x;}
	const float const* gety(){return y;}	
	const float const* getz(){return z;}	



	const float const* getssx(){return ssx;}
	const float const* getssy(){return ssy;}
	const float const* getssz(){return ssz;}
	const float const* getd(){return d;}
	const float const* getssd(){return ssd;}

	const float const* getdx(){return dx;}
	const float const* getdy(){return dy;}
	const float const* getdz(){return dz;}

	float* setdx(){return dx;}
	float* setdy(){return dy;}
	float* setdz(){return dz;}

	const float const* getssdx(){return ssdx;}
	const float const* getssdy(){return ssdy;}
	const float const* getssdz(){return ssdz;}

	Link getLink(){return cameraLink;}
	

	CameraType& getCameraType(){return type;}

	int getSize(){return type.getnx()*type.getny();}
	int getNRays(){return type.getnx()*type.getny();}
	int getNSSRays(){return type.getssnx()*type.getssny();}
	
	void updateCameraPos(HomogeneTransformation h);
	void raytrace(PCL& pcl);
	bool hitBox(BoundingBox& box);
	void raytrace(Link& link);
	void checkinBB(Link& link);

private:
	CameraType& type;
	HomogeneTransformation h;
	Link cameraLink;

	float* x;
	float* y;
	float* z;
	float* ssx;
	float* ssy;
	float* ssz;
	float* d;
	float* ssd;

	float* dx;
	float* dy;
	float* dz;
	bool* hasBoxhit;

	float* ssdx;
	float* ssdy;
	float* ssdz;
};

