#pragma once
#include "global.h"

#include "HomogeneTransformation.h"

class CFOBJECT_EXPORT BoundingBox
{
public:
	BoundingBox(HomogeneTransformation ih,float xdim,float ydim,float zdim);
	BoundingBox();
	void operator=(BoundingBox& rhs );

	void setH(HomogeneTransformation h);

	HomogeneTransformation getIH(){return ih;}
	HomogeneTransformation getH(){return h;}
	HomogeneTransformation getInvH(){return invh;}
	
	bool isInBoundingBox(const float const * p);
	float getXDim() {return xdim;}
	float getYDim() {return ydim;}
	float getZDim() {return zdim;}

	void transform(HomogeneTransformation h);
	~BoundingBox(void);

private:
	HomogeneTransformation ih;
	HomogeneTransformation h;
	HomogeneTransformation invh;
	float xdim;
	float ydim;
	float zdim;
};

