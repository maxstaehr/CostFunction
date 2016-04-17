#pragma once
#include "global.h"


#include "HomogeneTransformation.h"
#include "PCL.h"
#include "BoundingBox.h"

class CFOBJECT_EXPORT Link
{
public:
	Link(void);

	
	Link(Link& inst);
	void operator=(Link& rhs );

	~Link(void);



	void setH(HomogeneTransformation H);
	HomogeneTransformation& getH(){ return H;}

	PCL* getPCL() {return pcl;}
	BoundingBox* getBB() {return bbl;}

	void addPCL(PCL pcl);
	void addBB(BoundingBox bb);

	int getnPCL(void) {return nPCL;}
	int getnBB(void) {return nBB;};

private:
	HomogeneTransformation H;

	PCL* pcl;
	int nPCL;

	BoundingBox* bbl;
	int nBB;

};

