#include "Rotation.h"
#include <string>


Rotation::Rotation(void)
{
	float temp[9] = EYE_9;
	memcpy(H, temp, sizeof(float)*9);
}


Rotation::Rotation(Rotation& inst)
{
	memcpy(H, inst.H, sizeof(float)*9);
}

Rotation Rotation::mul(Rotation in)
{	
	Rotation ret;	
	float const * const a = H;
	float const * const b = in.H;
	ret.H[0] = a[0]*b[0] + a[1]*b[3] + a[2]*b[6];
	ret.H[1] = a[0]*b[1] + a[1]*b[4] + a[2]*b[7];
	ret.H[2] = a[0]*b[2] + a[1]*b[5] + a[2]*b[8];

	ret.H[3] = a[3]*b[0] + a[4]*b[3] + a[5]*b[6];
	ret.H[4] = a[3]*b[1] + a[4]*b[4] + a[5]*b[7];
	ret.H[5] = a[3]*b[2] + a[4]*b[5] + a[5]*b[8];

	ret.H[6] = a[6]*b[0] + a[7]*b[3] + a[8]*b[6];
	ret.H[7] = a[6]*b[1] + a[7]*b[4] + a[8]*b[7];
	ret.H[8] = a[6]*b[2] + a[7]*b[5] + a[8]*b[8];
	return ret;	
}

void Rotation::initRoll(float w)
{
	float ct = cos(w);
	float st = sin(w);
	H[0] = 1.0f;
	H[1] = 0.0f;
	H[2] = 0.0f;

	H[3] = 0.0f;
	H[4] = ct;
	H[5] = -st;
	
	H[6] = 0.0f;
	H[7] = st;
	H[8] = ct;

}
void Rotation::initPitch(float w)
{
	float ct = cos(w);
	float st = sin(w);
	H[0] = ct;
	H[1] = 0.0f;
	H[2] = st;	

	H[3] = 0.0f;
	H[4] = 1.0f;
	H[5] = 0.0f;	

	H[6] = -st;
	H[7] = 0.0f;
	H[8] = ct;
}

void Rotation::initYaw(float w)
{
	float ct = cos(w);
	float st = sin(w);
	H[0] = ct;
	H[1] = -st;
	H[2] = 0.0f;
	
	H[3] = st;
	H[4] = ct;
	H[5] = 0.0f;	

	H[6] = 0.0;
	H[7] = 0.0f;
	H[8] = 1.0f;	
}


void Rotation::setH(const float const* res)
{
	memcpy(H, res, sizeof(float)*9);
}



Rotation::~Rotation(void)
{
}



