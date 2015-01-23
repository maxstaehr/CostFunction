#include "SearchClass.h"


SearchClass::SearchClass(SAMPLE_PCL* sp, SAMPLE_ROTATIONS* sr, int nC, int nI, int* nn_indices)
{
	this->sp = sp;
	this->sr = sr;
	this->nC = nC;
	this->nI = nI;
	this->nn_indices = nn_indices;
}



SearchClass::~SearchClass(void)
{
}
