#include "SearchClass.h"
#include <assert.h>


SearchClass::SearchClass(SAMPLE_PCL* sp, SAMPLE_ROTATIONS* sr, VALID_POS* vp, int nC, int nI, int* nn_indices, AngleGenerator* ag)
{
	this->sp = sp;
	this->sr = sr;
	this->nC = nC;
	this->nI = nI;
	this->nn_indices = nn_indices;
	this->aG = ag;
	this->vp = vp;
	maxProb = 0;
	numberOfCalculations = 0;


}

bool SearchClass::isPositionValid(int pI, int aI)
{
	assert(pI >= 0 && pI < vp->nPCL && aI >= 0 && aI < vp->nRot);
	int index = this->vp->nRot*pI + aI;
	if (this->vp->validPos[index] > 0)
		return false;
	else
		return true;
}



SearchClass::~SearchClass(void)
{
}
