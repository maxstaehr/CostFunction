#pragma once

#include "global.h"


#include "Transformation2D.h"
#include "Cluster.h"

class CFOBJECT_EXPORT PossibleConfigurations
{
public:
	PossibleConfigurations(Transformation2D* hs, int size);

	const Transformation2D* getHS()					{return hs;}
	int getSize()									{return size;}
	double getMaxProb()								{return maxProb;}
	const double* getProb()							{return prob;}
	Transformation2D getBestTransformation()	{return bestTransformation;}
	
	void findBestFit(Cluster cluster);

	~PossibleConfigurations(void);

private:
	Transformation2D*	hs;
	int					size;

	double				maxProb;
	double*				prob;
	Transformation2D	bestTransformation;
	

};

