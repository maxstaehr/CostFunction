#include "Search.h"
#include <float.h>

Search::Search(SampleCameraConfiguration* config, int n):currentCosts(1.0)
{
	this->sampleConfig = new SampleCameraConfiguration[n];
	this->n = n;
	for(int i=0; i<n; i++)
	{
		this->sampleConfig[i] = config[i];
	}

	currentTrans = new HomogeneTransformation[n];

}


Search::~Search(void)
{
	delete [] sampleConfig;
	delete [] currentTrans;
}
