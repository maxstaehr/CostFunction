#include "Transformation2D.h"

#include <assert.h>
#include <string>

Transformation2D::Transformation2D()
{
	R[0] = 1.0f;
	R[1] = 0.0f;
	R[2] = 0.0f;
	R[3] = 1.0f;
	v[0] = 0;
	v[1] = 0;
}

Transformation2D::Transformation2D(float w, float x, float y)
{
	float ct = cos(w);
	float st = sin(w);
	R[0] = ct;
	R[1] = -st;	
	R[2] = st;
	R[3] = ct;
	v[0] = x;
	v[1] = y;

}


Transformation2D::Transformation2D(const Transformation2D& inst)
{
	memcpy(R, inst.R, sizeof(float)*4);
	memcpy(v, inst.v, sizeof(float)*2);
}
void Transformation2D::operator=(Transformation2D& rhs )
{
	memcpy(R, rhs.R, sizeof(float)*4);
	memcpy(v, rhs.v, sizeof(float)*2);
}

void Transformation2D::setR(int i, float value)
{
	assert(i >= 0 && i < 4);
	R[i] = value;
}

void Transformation2D::setv(int i, float value)
{
	assert(i >= 0 && i < 2);
	v[i] = value;
}


Transformation2D::~Transformation2D(void)
{
}
