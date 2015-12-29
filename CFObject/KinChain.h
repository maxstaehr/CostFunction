#pragma once

#include "global.h"

#include "Link.h"


class CFOBJECT_EXPORT KinChain
{
public:
	KinChain(int nPos);

	void setHumanPos(const float const * pos);
	void setEnvPos(const float const* pos);
	void setRobotPos(const float const* pos);

	void setPosIndex(int i);

	const float const* getRobotPos(){return rP;}
	const float const* getHumanPos(){return hP;}
	const float const* getEnvPos(){return eP;};
	int getNPos(){return nPos;}

	Link* getHumanLinks(){return h;}
	Link* getRobotLinks(){return r;}
	Link* getEnvLinks(){return e;}


	~KinChain(void);


private:
	float* hP;
	float* eP;
	float* rP;
	int nPos;

	Link h[DOF_H];
	Link e[DOF_E];
	Link r[DOF_R];

};

