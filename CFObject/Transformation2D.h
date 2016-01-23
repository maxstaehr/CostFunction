#pragma once

#include "global.h"

class CFOBJECT_EXPORT Transformation2D
{
public:

	Transformation2D();		
	Transformation2D(const Transformation2D& inst);
	Transformation2D(float yaw, float x, float y);
	void operator=(Transformation2D& rhs );

	const float* getR(){return R;}
	const float* getv(){return v;}

	void setR(int i, float value);
	void setv(int i, float value);

	~Transformation2D(void);
private:
	float R[4];
	float v[2];

};

