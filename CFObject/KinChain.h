#pragma once

#include "global.h"

#include "Link.h"
#include "CFIO.h"



class CFOBJECT_EXPORT KinChain
{
public:
	KinChain(int nPos, int samplesPerPos);

	KinChain();
	KinChain(KinChain& inst);
	void operator=(KinChain& rhs );



	void setHumanPos(const float const * pos);
	
	void addHumanPCL(CFIO::LINK* pcl);
	void addRobotPCL(CFIO::LINK* pcl);
	void addEnvPCL(CFIO::LINK* pcl);
	

	void setEnvPos(const float const* pos);
	void setRobotPos(const float const* pos);
	void saveCurrentSzene(const char* fileName);

	void setPosIndex(int i);

	const float const* getRobotPos(){return rP;}
	const float const* getHumanPos(){return hP;}
	const float const* getEnvPos(){return eP;};
	int getNPos(){return nPos;}

	Link* getHumanLinks(){return h;}
	int getNHumanLinks(){return DOF_H;}
	Link* getRobotLinks(){return r;}
	int getNRobotLinks(){return DOF_R;}
	Link* getEnvLinks(){return e;}
	int getNEnvLinks(){return DOF_E;}


	~KinChain(void);


private:
	float* hP;
	float* eP;
	float* rP;
	int nPos;
	int samplesPerPos;

	Link h[DOF_H];
	Link e[DOF_E];
	Link r[DOF_R];

	void addPCL(Link* links, int DOF, CFIO::LINK* pcl);


};

