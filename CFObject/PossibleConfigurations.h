#pragma once

#include "global.h"


#include "Transformation2D.h"
#include "Cluster.h"
#include "CFIO.h"

class CFOBJECT_EXPORT PossibleConfigurations
{
public:
	PossibleConfigurations();
	PossibleConfigurations(Transformation2D* hs, int size);
	PossibleConfigurations(PossibleConfigurations& inst);
	PossibleConfigurations(CFIO::SAMPLE_HUMAN_POSITIONS* pcl);

	void saveState(const char* name);
	void operator=(PossibleConfigurations& rhs );

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

