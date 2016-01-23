#pragma once
#include "global.h"

class CFOBJECT_EXPORT HomogeneTransformation
{
public:
	enum DIM_DIR{	XM = 0, XP = 1,
					YM = 2, YP = 3,
					ZM = 4, ZP = 5,
					ROLLM = 6, ROLLP= 7,
					PITCHM = 8, PITCHP = 9,
					YAWM = 10, YAWP = 11
	};

	HomogeneTransformation();
	HomogeneTransformation(const float * h);
	HomogeneTransformation(const HomogeneTransformation& inst);
	void operator=(HomogeneTransformation& rhs );

	~HomogeneTransformation(void);

	
	void setH(const float* res);
	const float* getH(void){return H;}

	bool isEqual(HomogeneTransformation& rhs);
	float getDist(HomogeneTransformation& rhs, DIM_DIR dim);
	float getDist(HomogeneTransformation& rhs);
	void tr2rpy(float* rpy);



	HomogeneTransformation inv(void);
	HomogeneTransformation mul(HomogeneTransformation in);
	void init(float roll, float pitch, float yaw, float x, float y, float z);

private:
	float H[16];

};

