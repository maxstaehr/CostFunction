#include "Camera.h"

#include <string>
#include <algorithm>
#include <float.h>
#include <assert.h>
#include <cmath>

//local algorithmic helper functions
int triangle_intersection		( const float*   V1,  // Triangle vertices
								   const float*   V2,
								   const float*   V3,
								   const float*    O,  //Ray origin
								   const float*    D,  //Ray direction
										 float* out );
float DOT(const float *a, const float *b);
void CROSS(float* r, const float *a, const float *b );
void SUB(float* r, const float *a, const float *b );
bool intersectBox(float* origin, float* direction, float* vmin, float* vmax);
#define EPSILON (1e-5)
//

Camera::Camera(CameraType& camType):type(camType)
{


	this->x = new float[type.getnx() * type.getny()];
	this->y = new float[type.getnx() * type.getny()];
	this->z = new float[type.getnx() * type.getny()];
	memcpy(this->x, type.getx(), type.getnx()*type.getny()*sizeof(float));
	memcpy(this->y, type.gety(), type.getnx()*type.getny()*sizeof(float));
	memcpy(this->z, type.getz(), type.getnx()*type.getny()*sizeof(float));
	

	this->ssx = new float[type.getssnx() * type.getssny()];
	this->ssy = new float[type.getssnx() * type.getssny()];
	this->ssz = new float[type.getssnx() * type.getssny()];
	memcpy(this->ssx, type.getssx(), type.getssnx() * type.getssny()*sizeof(float));
	memcpy(this->ssy, type.getssy(), type.getssnx() * type.getssny()*sizeof(float));
	memcpy(this->ssz, type.getssz(), type.getssnx() * type.getssny()*sizeof(float));

	this->d = new float[type.getnx() * type.getny()];
	this->ssd = new float[type.getssnx() * type.getssny()];

	for(int ray=0; ray<type.getnx() * type.getny(); ray++)
	{
		this->d[ray] = FLT_MAX;
	}

	for(int ray=0; ray<type.getssnx() * type.getssny(); ray++)
	{
		this->ssd[ray] = FLT_MAX;
	}
	
	this->dx = new float[type.getnx() * type.getny()];
	this->dy = new float[type.getnx() * type.getny()];
	this->dz = new float[type.getnx() * type.getny()];

	this->ssdx = new float[type.getssnx() * type.getssny()];
	this->ssdy = new float[type.getssnx() * type.getssny()];
	this->ssdz = new float[type.getssnx() * type.getssny()];
}


Camera::~Camera(void)
{
	delete x;
	delete y;
	delete z;
	delete ssx;
	delete ssy;
	delete ssz;

	delete dx;
	delete dy;
	delete dz;
	delete ssdx;
	delete ssdy;
	delete ssdz;
}

void Camera::updateCameraPos(HomogeneTransformation trans)
{
	h = trans;
	float xb, yb, zb;
	int n = type.getnx() * type.getny();
	for(int i=0; i<n; i++)
	{
		xb = type.getx()[i];
		yb = type.gety()[i];
		zb = type.getz()[i];

		x[i] = xb*trans.getH()[0] + yb*trans.getH()[1] + zb*trans.getH()[2];
		y[i] = xb*trans.getH()[4] + yb*trans.getH()[5] + zb*trans.getH()[6];
		z[i] = xb*trans.getH()[8] + yb*trans.getH()[9] + zb*trans.getH()[10];
	}

	n = type.getssnx() * type.getssny();
	for(int i=0; i<n; i++)
	{
		xb = type.getssx()[i];
		yb = type.getssy()[i];
		zb = type.getssz()[i];

		ssx[i] = xb*trans.getH()[0] + yb*trans.getH()[1] + zb*trans.getH()[2];
		ssy[i] = xb*trans.getH()[4] + yb*trans.getH()[5] + zb*trans.getH()[6];
		ssz[i] = xb*trans.getH()[8] + yb*trans.getH()[9] + zb*trans.getH()[10];
	}
}

void Camera::raytrace(Link& link)
{
	assert(link.getnPCL() == link.getnBB());
	BoundingBox box;
	for(int i=0;i<link.getnPCL(); i++)
	{		
		if(hitBox(link.getBB()[i]))
		{
			raytrace(link.getPCL()[i]);
		}
	}
}

bool Camera::hitBox(BoundingBox& box)
{
	HomogeneTransformation boxh = box.getH().inv();
	HomogeneTransformation hr = boxh.mul(h);

	int n = type.getssnx() * type.getssny();
	float origin[3];
	float direction[3];
	float vmin[3];
	float vmax[3];

	origin[0] = hr.getH()[3];
	origin[1] = hr.getH()[7];
	origin[2] = hr.getH()[11];

	vmin[0] = .0f;
	vmin[1] = .0f;
	vmin[2] = .0f;

	vmax[0] = box.getXDim();
	vmax[1] = box.getYDim();
	vmax[2] = box.getZDim();

	float xb, yb, zb;
	
	for(int i=0; i<n; i++)
	{
		xb = type.getssx()[i];
		yb = type.getssy()[i];
		zb = type.getssz()[i];

		direction[0] = xb*hr.getH()[0] + yb*hr.getH()[1] + zb*hr.getH()[2];
		direction[1] = xb*hr.getH()[4] + yb*hr.getH()[5] + zb*hr.getH()[6];
		direction[2] = xb*hr.getH()[8] + yb*hr.getH()[9] + zb*hr.getH()[10];


		if(intersectBox(origin, direction, vmin, vmax))
		{
			return true;
		}
	}
	return false;
}


void Camera::raytrace(PCL& pcl)
{
	int nRays = type.getnx()*type.getny();
	int fix, fiy, fiz, hit;
	float v1[3], v2[3], v3[3], D[3], O[3];
	float dist;
	using namespace std;
	
	O[0] = h.getH()[3];
	O[1] = h.getH()[7];
	O[2] = h.getH()[11];
	for(int ray=0; ray<nRays; ray++)
	{
		D[0] = x[ray];
		D[1] = y[ray];
		D[2] = z[ray];

		dist = FLT_MAX;		
		for(int face=0; face<pcl.getnF();face++)
		{
			fix = pcl.getfx()[face];
			fiy = pcl.getfy()[face];
			fiz = pcl.getfz()[face];

			v1[0] = pcl.getx()[fix];
			v1[1] = pcl.gety()[fix];
			v1[2] = pcl.getz()[fix];
					
			v2[0] = pcl.getx()[fiy];
			v2[1] = pcl.gety()[fiy];
			v2[2] = pcl.getz()[fiy];
					
			v3[0] = pcl.getx()[fiz];
			v3[1] = pcl.gety()[fiz];
			v3[2] = pcl.getz()[fiz];

			hit = triangle_intersection(v1, v2, v3, O, D, &dist);
			if(hit)
			{
				d[ray] = min(d[ray], dist);
			}
		}
		if(d[ray] != FLT_MAX)
		{
			dx[ray] = O[0] + d[ray]*D[0];
			dy[ray] = O[1] + d[ray]*D[1];
			dz[ray] = O[2] + d[ray]*D[2];
		}else
		{
			dx[ray] = FLT_MAX;
			dy[ray] = FLT_MAX;
			dz[ray] = FLT_MAX;
		}
	}

	nRays = type.getssnx()*type.getssny();
	for(int ray=0; ray<nRays; ray++)
	{
		D[0] = ssx[ray];
		D[1] = ssy[ray];
		D[2] = ssz[ray];

		dist = FLT_MAX;		
		for(int face=0; face<pcl.getnF();face++)
		{
			fix = pcl.getfx()[face];
			fiy = pcl.getfy()[face];
			fiz = pcl.getfz()[face];

			v1[0] = pcl.getx()[fix];
			v1[1] = pcl.gety()[fix];
			v1[2] = pcl.getz()[fix];
					
			v2[0] = pcl.getx()[fiy];
			v2[1] = pcl.gety()[fiy];
			v2[2] = pcl.getz()[fiy];
					
			v3[0] = pcl.getx()[fiz];
			v3[1] = pcl.gety()[fiz];
			v3[2] = pcl.getz()[fiz];

			hit = triangle_intersection(v1, v2, v3, O, D, &dist);
			if(hit)
			{
				ssd[ray] = min(ssd[ray], dist);
			}
		}

		if(ssd[ray] != FLT_MAX)
		{
			ssdx[ray] = O[0] + ssd[ray]*D[0];
			ssdy[ray] = O[1] + ssd[ray]*D[1];
			ssdz[ray] = O[2] + ssd[ray]*D[2];
		}else
		{
			ssdx[ray] = FLT_MAX;
			ssdy[ray] = FLT_MAX;
			ssdz[ray] = FLT_MAX;
		}
	}
}


int triangle_intersection(	const float*   V1,  // Triangle vertices
							const float*   V2,
							const float*   V3,
							const float*    O,  //Ray origin
							const float*    D,  //Ray direction
							float* out )
{
	float e1[3], e2[3];  //Edge1, Edge2
	float P[3], Q[3], T[3];
	float det, inv_det, u, v;
	float t;
 
	//Find vectors for two edges sharing V1
	SUB(e1, V2, V1);
	SUB(e2, V3, V1);
	//Begin calculating determinant - also used to calculate u parameter
	CROSS(P, D, e2);
	//if determinant is near zero, ray lies in plane of triangle
	det = DOT(e1, P);
	//NOT CULLING
	if(det > -EPSILON && det < EPSILON) return 0;
	inv_det = 1.0f / det;
 
	//calculate distance from V1 to ray origin
	SUB(T, O, V1);
 
	//Calculate u parameter and test bound
	u = DOT(T, P) * inv_det;
	//The intersection lies outside of the triangle
	if(u < 0.f || u > 1.f) return 0;
 
	//Prepare to test v parameter
	CROSS(Q, T, e1);
 
	//Calculate V parameter and test bound
	v = DOT(D, Q) * inv_det;
	//The intersection lies outside of the triangle
	if(v < 0.0f || u + v  > 1.0f) return 0;
 
	t = DOT(e2, Q) * inv_det;
 
	if(t > EPSILON) { //ray intersection
	*out = t;
	return 1;
	}
 
	// No hit, no win
	return 0;
}

float DOT(const float *a, const float *b)
{
  return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
}

void CROSS(float* r, const float *a, const float *b ) 
{
	  r[0] =   ( (a[1] * b[2]) - (a[2] * b[1]) );
	  r[1] =   ( (a[2] * b[0]) - (a[0] * b[2]) );
	  r[2] =   ( (a[0] * b[1]) - (a[1] * b[0]) );
}

void SUB(float* r, const float *a, const float *b ) 
{
	  r[0] =   a[0] - b[0];
	  r[1] =   a[1] - b[1];
	  r[2] =   a[2] - b[2];
}

bool intersectBox(float* origin, float* direction, float* vmin, float* vmax)
 {
	 float tmin = -FLT_MAX, tmax = FLT_MAX;
	 bool hitPx = true, hitPy = true, hitPz = true;
	using namespace std;

    if (direction[0] != 0.0) {
		float tx1 = (vmin[0] - origin[0])/direction[0];
        float tx2 = (vmax[0] - origin[0])/direction[0];
 
        tmin = max(tmin, min(tx1, tx2));
        tmax = min(tmax, max(tx1, tx2));
    }else
	{
		if (origin[0] >= vmin[0] && origin[0] <= vmin[0])
		{
			hitPx = true;
		}else{
			hitPx = false;
		}
	}

 
    if (direction[1] != 0.0) {
        float ty1 = (vmin[1] - origin[1])/direction[1];
        float ty2 = (vmax[1] - origin[1])/direction[1];
 
        tmin = max(tmin, min(ty1, ty2));
        tmax = min(tmax, max(ty1, ty2));
    }else
	{
		if (origin[1] >= vmin[1] && origin[1] <= vmin[1])
		{
			hitPy = true;
		}else{
			hitPy = false;
		}
	}

	if (direction[2] != 0.0) {
        float tz1 = (vmin[2] - origin[2])/direction[2];
        float tz2 = (vmax[2] - origin[2])/direction[2];
 
        tmin = max(tmin, min(tz1, tz2));
        tmax = min(tmax, max(tz1, tz2));
    }else
	{
		if (origin[2] >= vmin[2] && origin[2] <= vmin[2])
		{
			hitPz = true;
		}else{
			hitPz = false;
		}
	}
 
    return tmax >= tmin && hitPx && hitPy && hitPz;
}
//
//bool intersectBox(float* origin, float* direction, float* vmin, float* vmax)
//{
//
//	float tmin, tmax, tymin, tymax, tzmin, tzmax;
//	bool flag;
//	if (direction[0] >= 0) 
//	{
//		tmin = (vmin[0] - origin[0]) / direction[0];
//		tmax = (vmax[0] - origin[0]) / direction[0];
//	}
//	else
//	{
//		tmin = (vmax[0] - origin[0]) / direction[0];
//		tmax = (vmin[0] - origin[0]) / direction[0];
//	}
//  
//	if (direction[1] >= 0) 
//	{
//		tymin = (vmin[1] - origin[1]) / direction[1];
//		tymax = (vmax[1] - origin[1]) / direction[1];
//	}
//	else
//	{
//		tymin = (vmax[1] - origin[1]) / direction[1];
//		tymax = (vmin[1] - origin[1]) / direction[1];
//	}
//    
//
//	if ( (tmin > tymax) || (tymin > tmax) )
//	{
//		return false;
//	}
//       
//	if (tymin > tmin)
//	{
//		tmin = tymin;
//	}
//    
//    
//	if (tymax < tmax)
//	{
//		tmax = tymax;
//	}
//    
//    
//	if (direction[2] >= 0)
//	{
//		tzmin = (vmin[2] - origin[2]) / direction[2];
//		tzmax = (vmax[2] - origin[2]) / direction[2];
//	}
//	else
//	{
//		tzmin = (vmax[2] - origin[2]) / direction[2];
//		tzmax = (vmin[2] - origin[2]) / direction[2];
//	}
//
//
//	if ((tmin > tzmax) || (tzmin > tmax))
//	{
//		return false;
//	}
//    
//	return true;
//}