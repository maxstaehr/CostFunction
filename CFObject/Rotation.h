#pragma once
#include "global.h"

class CFOBJECT_EXPORT Rotation
{
public:
	Rotation(void);
	Rotation(Rotation& r);
	Rotation mul(Rotation r);
	void initRoll(float w);
	void initPitch(float w);
	void initYaw(float w);
	
	void setH(const float* res);
	float const* const getH(void){return H;}

	~Rotation(void);

private: 
	float H[9];
};

