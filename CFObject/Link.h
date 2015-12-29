#pragma once
#include "global.h"


#include "HomogeneTransformation.h"

class CFOBJECT_EXPORT Link
{
public:
	Link(void);
	~Link(void);

	void setH(HomogeneTransformation H) {this->H = H;}
	HomogeneTransformation& getH(){ return H;}

private:
	HomogeneTransformation H;


};

