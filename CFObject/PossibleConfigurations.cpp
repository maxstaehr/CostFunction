#include "PossibleConfigurations.h"
#include <float.h>

PossibleConfigurations::PossibleConfigurations(Transformation2D* hs, int size)
{
	this->size = size;
	this->hs = new Transformation2D[size];
	this->prob = new double[size];
	for(int i=0; i<size; i++)
	{
		this->hs[i] = hs[i];
	}
	maxProb = 0;
}

void PossibleConfigurations::findBestFit(Cluster cluster)
{
	//first convert the Cluster to the center
	cluster.calculateCentroid();
	Transformation2D transformation(0.0f, -cluster.getCentroid()[0], -cluster.getCentroid()[1]);
	cluster.transform(transformation);

	Cluster temp;
	double p;
	for(int i=0; i<size; i++)
	{
		temp = cluster;
		temp.transform(hs[i]);
		temp.calculate();
		p = temp.getHumanProb();
		prob[i] = p;
		if(p > maxProb)
		{
			maxProb = p;
			bestTransformation = hs[i];
		}
	}
}


PossibleConfigurations::~PossibleConfigurations(void)
{
	delete[] this->hs;
	delete this->prob;
}
