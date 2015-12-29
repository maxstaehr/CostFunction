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
	
	float* start_h = hP+i*NELEM_H*DOF_H;
	for(int ih=0; ih<DOF_H; ih++)
	{
		h[ih].getH().setH(start_h+ih*NELEM_H);
	}

	float* start_r = rP+i*NELEM_H*DOF_R; 
	for(int ir=0; ir<DOF_R; ir++)
	{
		r[ir].getH().setH(start_r+ir*NELEM_H);
	}

	float* start_e = eP+i*NELEM_H*DOF_E; 
	for(int ie=0; ie<DOF_E; ie++)
	{
		e[ie].getH().setH(start_e+ie*NELEM_H);
	}	
}