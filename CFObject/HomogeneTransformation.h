#pragma once
#include "global.h"

class CFOBJECT_EXPORT HomogeneTransformation
{
public:
	HomogeneTransformation(void);
	HomogeneTransformation(HomogeneTransformation& inst);
	~HomogeneTransformation(void);

	void getH(float * res);
	void setH(const float const* res);
	float const* const V(void);



	HomogeneTransformation inv(void);
	HomogeneTransformation mul(HomogeneTransformation in);
	void init(float roll, float pitch, float yaw, float x, float y, float z);

private:
	float H[16];

};

