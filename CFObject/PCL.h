#pragma once
#include "global.h"
#include "Rotation.h"
#include "HomogeneTransformation.h"
#include <iostream>
#include <fstream>

class CFOBJECT_EXPORT PCL 	
{
public:
	PCL(int nV, int nF,  const float* x, 
						 const float* y,
						 const float* z,
						 const int const * fx, 
						 const int const * fy,
						 const int const * fz);
	PCL();
	PCL(PCL& pcl);
	void operator=(PCL& rhs );
	~PCL(void);

	int getnV(void){return nV;};
	int getnF(void){return nF;};
	const float const * getx(void){return x;}
	const float const * gety(void){return y;}
	const float const * getz(void){return z;}
	const int const * getfx(void){return fx;}
	const int const * getfy(void){return fy;}
	const int const * getfz(void){return fz;}

	void rotate(Rotation rot);
	void transform(HomogeneTransformation trans);
	void savePCL(std::ofstream* outbin);

private:
	int nV;
	int nF;
	

	float*	x;
	float*	y;
	float*	z;
	
	int*	fx;
	int*	fy;
	int*	fz;	

};

