#pragma once
class FObject
{
public:
	FObject(void);
	~FObject(void);

	virtual void allocCudaMemory(void) = 0;
	virtual void freeCudaMemory(void) = 0;
};

