#include "KinChain.h"

#include <string>
#include <assert.h>

KinChain::KinChain(int nPos)
{
	this->nPos = nPos;
	rP = new float[nPos*NELEM_H*DOF_R];
	eP = new float[nPos*NELEM_H*DOF_E];
	hP = new float[nPos*NELEM_H*DOF_H];

	memset(rP, 0, sizeof(float)*nPos*NELEM_H*DOF_R);
	memset(eP, 0, sizeof(float)*nPos*NELEM_H*DOF_E);
	memset(hP, 0, sizeof(float)*nPos*NELEM_H*DOF_H);

}


KinChain::~KinChain(void)
{
	delete rP;
	delete eP;
	delete hP;
}


void KinChain::setHumanPos(const float const * pos)
{
	memcpy(hP, pos, sizeof(float)*DOF_H*NELEM_H*nPos);

}
void KinChain::setEnvPos(const float const* pos)
{
	memcpy(eP, pos, sizeof(float)*DOF_E*NELEM_H*nPos);
}
void KinChain::setRobotPos(const float const* pos)
{
	memcpy(rP, pos, sizeof(float)*DOF_R*NELEM_H*nPos);
}

void KinChain::setPosIndex(int i)
{
	assert(i >= 0 && i < nPos);
	//copy things into the link
}