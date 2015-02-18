#pragma once

#include <set>

class EC
{
public:
	EC(float* dx, float* dy, float* dz, int* cis, bool* hasProcessed, int size);
	~EC(void);
	void cluster(void);

	int getMaxIndex(void){return this->maxIndex;}
	

private:	
	float* dx;
	float* dy;
	float* dz;
	int* cis;
	int size;
	bool* hasProcessed;

	int maxSize;
	int maxIndex;

	bool hasAllBeenProcessed(std::set<int>* set);
	void process(std::set<int>* Q, std::set<int>* P);
	int findFirstNotProcessedIndice(std::set<int>* set);

	float calcDistance(int i1, int i2);
	

};

