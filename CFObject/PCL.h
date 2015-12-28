#pragma once
#include "global.h"
#include "Rotation.h"
#include "HomogeneTransformation.h"

class CFOBJECT_EXPORT PCL 	
{
public:
	PCL(int nV, int nF,  const float const * x, 
						 const float const * y,
						 const float const * z,
						 const int const * fx, 
						 const int const * fy,
						 const int const * fz);
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
