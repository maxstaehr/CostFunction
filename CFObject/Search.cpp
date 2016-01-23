#include "Search.h"


Search::Search(const SampleCameraConfiguration& config):sampleConfig(config), currentCosts(0),currentTrans(HomogeneTransformation()), 
	nextEvalMinus(HomogeneTransformation()), nextEvalPlus(HomogeneTransformation())
{

}


Search::~Search(void)
{
}
