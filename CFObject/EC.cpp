#include "EC.h"

#include <iostream>
EC::EC(int maxSize):x(NULL), y(NULL), z(NULL), Q(NULL), Ci(NULL)
{
	this->maxSize = maxSize;
	this->x = new float[maxSize];
	this->y = new float[maxSize];
	this->z = new float[maxSize];
	
}
void EC::reset()
{
}
void EC::addDepthData(Camera& cam)
{

}
void EC::cluster()
{

}


EC::~EC(void)
{
}
