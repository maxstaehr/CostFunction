#pragma once
#include "global.h"



class CFOBJECT_EXPORT Link
{
public:
	Link(void);
	~Link(void);

	void setH(float H[16]);
	void getH(float* H);

private:
	float H[16];


};

