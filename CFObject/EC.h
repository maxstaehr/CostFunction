#pragma once

#include "global.h"
#include "Camera.h"

#include <set>

class CFOBJECT_EXPORT EC
{
public:
	EC(int maxBufferSize);
	~EC(void);

	int getMaxSize() {return maxSize;}
	void reset();
	void addDepthData(Camera& cam);
	void cluster();

	const float * const getX(){return x;}
	const float * const getY(){return y;}
	const float * const getZ(){return z;}

	std::set<int>&				getP(){return P;}
	std::set<std::set<int>*>&	getC(){return C;}
	std::set<int>*				getQ(){return Q;}
	std::set<int>*				getCi(){return Ci;}

private:
	int maxSize;

	float* x;
	float* y;
	float* z;
	bool* hasProcessed;

	std::set<int> P;
	std::set<std::set<int>*> C;
	std::set<int>* Q;	
	std::set<int>* Ci;
};

