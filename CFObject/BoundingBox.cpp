#include "BoundingBox.h"

#define eps_in_bb (1e-2)

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
void BoundingBox::saveBB(std::ofstream* outbin)
{
	outbin->write((char*)h.getH(),NELEM_H*sizeof(float));
	if (!outbin) std::cerr << "error";

	outbin->write((char*)&xdim,sizeof(float));
	if (!outbin) std::cerr << "error";

	outbin->write((char*)&ydim,sizeof(float));
	if (!outbin) std::cerr << "error";

	outbin->write((char*)&zdim,sizeof(float));
	if (!outbin) std::cerr << "error";
}

BoundingBox::BoundingBox():xdim(1.0), ydim(1.0), zdim(1.0)
{
	HomogeneTransformation newH;
	HomogeneTransformation ih;
	this->ih = ih;
	this->h = newH.mul(this->ih);
	this->invh = this->h.inv();
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
	//x = p[0]*h.getH()[0] + p[1]*h.getH()[1] + p[2]*h.getH()[2] + h.getH()[3];
	//y = p[0]*h.getH()[4] + p[1]*h.getH()[5] + p[2]*h.getH()[6] + h.getH()[7];
	//z = p[0]*h.getH()[8] + p[1]*h.getH()[9] + p[2]*h.getH()[10] + h.getH()[11];

	x = p[0]*invh.getH()[0] + p[1]*invh.getH()[1] + p[2]*invh.getH()[2] + invh.getH()[3];
	y = p[0]*invh.getH()[4] + p[1]*invh.getH()[5] + p[2]*invh.getH()[6] + invh.getH()[7];
	z = p[0]*invh.getH()[8] + p[1]*invh.getH()[9] + p[2]*invh.getH()[10] + invh.getH()[11];

	isInX = x > -eps_in_bb && x < xdim+eps_in_bb;
	isInY = y > -eps_in_bb && y < ydim+eps_in_bb;
	isInZ = z > -eps_in_bb && z < zdim+eps_in_bb;
	return isInX && isInY && isInZ;
}
