#include "CE.h"


CE::CE(SampleCameraConfiguration* sampleConfig, int n):Search(sampleConfig, n), currentCompleteIndex(0)
{
	maxIndex = 0;
	for(int i=0; i<n; i++)
	{
		maxIndex += sampleConfig[i].getNRelative();
	}
}


CE::~CE(void)
{
}

bool CE::nextIteration(double cost_m, double cost_p)
{
	if(currentCompleteIndex < maxIndex)
	{
		int sampleconfigurationIndex = 0;
		int indexInConfiguration = 0;
		int indexOfSampleConfiguration = 0;
		for(int i=0; i<n; i++)
		{
			if(sampleconfigurationIndex + sampleConfig[i].getNRelative() > currentCompleteIndex)
			{
				indexInConfiguration = currentCompleteIndex - sampleconfigurationIndex;
				indexOfSampleConfiguration = i;
				break;
			}
			sampleconfigurationIndex += sampleConfig[i].getNRelative();
		}
		nextEvalMinus = sampleConfig[indexOfSampleConfiguration].getInitialH(indexInConfiguration);
		nextEvalPlus = sampleConfig[indexOfSampleConfiguration].getInitialH(indexInConfiguration+1);

		return true;
	}else
	{
		return false;
	}
	
}
void CE::setCurrentTransformation(HomogeneTransformation h, int i)
{

}
