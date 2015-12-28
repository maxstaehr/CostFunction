#pragma once
#include "fobject.h"
class PCL :
	public FObject
{
public:
	PCL(void);
	~PCL(void);

	
	virtual void allocCudaMemory(void);
	virtual void freeCudaMemory(void);
};

