#include "SA.h"


SA::SA(const SampleCameraConfiguration& sampleConfig):Search(sampleConfig)
{

}


SA::~SA(void)
{
}
bool SA::nextIteration(double cost_m, double cost_p)
{
	return true;
}
