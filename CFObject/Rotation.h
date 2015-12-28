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

	void getH(float * res);
	void setH(const float const* res);
	float const* const V(void);

	~Rotation(void);

private: 
	float H[9];
};

