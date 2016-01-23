#include "BoundingBox.h"


BoundingBox::BoundingBox(HomogeneTransformation ih,float xdim,float ydim,float zdim)
{
	HomogeneTransformation newH;
	this->ih = ih;
	this->h = newH.mul(this->ih);
	this->invh = this->h.inv();
	this->xdim = xdim;
	this->ydim = ydim;
	this->zdim = zdim;	
}
BoundingBox::BoundingBox()
{
}

void BoundingBox::setH(HomogeneTransformation ih) 
{
	HomogeneTransformation newH;
	this->ih = ih;
	this->h = newH.mul(this->ih);
	this->invh = this->h.inv();
}

void BoundingBox::transform(HomogeneTransformation newH)
{
	this->h = newH.mul(this->ih);
	this->invh = this->h.inv();
}

BoundingBox::~BoundingBox(void)
{
}

void BoundingBox::operator=(BoundingBox& rhs )
{
	ih = rhs.getIH();
	h = rhs.getH();
	invh = rhs.getInvH();
	xdim = rhs.getXDim();
	ydim = rhs.getYDim();
	zdim = rhs.getZDim();
}

bool  BoundingBox::isInBoundingBox(const float* p)
{
	float x, y, z;
	bool isInX, isInY, isInZ;
	x = p[0]*h.getH()[0] + p[1]*h.getH()[1] + p[2]*h.getH()[2] + h.getH()[3];
	y = p[0]*h.getH()[4] + p[1]*h.getH()[5] + p[2]*h.getH()[6] + h.getH()[7];
	z = p[0]*h.getH()[8] + p[1]*h.getH()[9] + p[2]*h.getH()[10] + h.getH()[11];

	isInX = x > 0.0f && x < xdim;
	isInY = y > 0.0f && y < ydim;
	isInZ = z > 0.0f && z < zdim;
	return isInX && isInY && isInZ;
}
