#include "Search.h"
#include <float.h>

Search::Search(SampleCameraConfiguration& config):sampleConfig(config), currentCosts(1.0),currentTrans(HomogeneTransformation()), 
	nextEvalMinus(HomogeneTransformation()), nextEvalPlus(HomogeneTransformation())
{

}


Search::~Search(void)
{
}
