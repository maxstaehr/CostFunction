#include "SA.h"
#include <algorithm>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

SA::SA(SampleCameraConfiguration* sampleConfig, int n):Search(sampleConfig, n), state(STATE::HC), T(1.0), alpha(0.9), minThres(0.1),currentSampleIndex(0)
{
	srand (time(NULL));
	currentDim = new EVAL_DIM[n];
	for(int i=0; i<n; i++)
	{
		currentDim[i] = EVAL_DIM::X;
	}
	localMinima = new bool[6*n];
	resetLocalMinima();
}



SA::~SA(void)
{
	delete currentDim;
	delete localMinima;
}

void SA::setCurrentTransformation(HomogeneTransformation h, int i)
{
	currentTrans[i] = h;
	currentDim[i] = EVAL_DIM::X;
	HomogeneTransformation::DIM_DIR minusDir, plusDir;
	minusDir = HomogeneTransformation::DIM_DIR::XM;
	plusDir = HomogeneTransformation::DIM_DIR::XP;
	nextEvalMinus = sampleConfig[i].findNN(currentTrans[i], minusDir);
	nextEvalPlus = sampleConfig[i].findNN(currentTrans[i],  plusDir);
	resetLocalMinima();
}

void SA::nextDim()
{
	switch(currentDim[currentSampleIndex])
	{
	case EVAL_DIM::X:
		currentDim[currentSampleIndex] = EVAL_DIM::Y;
		break;
	case EVAL_DIM::Y:
		currentDim[currentSampleIndex] = EVAL_DIM::Z;
		break;
	case EVAL_DIM::Z:
		currentDim[currentSampleIndex] = EVAL_DIM::ROLL;
		break;
	case EVAL_DIM::ROLL:
		currentDim[currentSampleIndex] = EVAL_DIM::PITCH;
		break;
	case EVAL_DIM::PITCH:
		currentDim[currentSampleIndex] = EVAL_DIM::YAW;
		break;
	case EVAL_DIM::YAW:
		currentDim[currentSampleIndex] = EVAL_DIM::X;
		currentSampleIndex = (currentSampleIndex+1)%n;
		break;
	};
}

void SA::resetLocalMinima()
{
	for(int i=0; i<n*6; i++)
	{
		localMinima[i] = false;
	}
}
bool SA::isLocalMinimaReached()
{
	bool ret = true;
	for(int i=0; i<n*6; i++)
	{
		ret &= localMinima[i];
	}
	return ret;
	return true;
}

void SA::setLocalMinima()
{
	int index = -1;
	switch(currentDim[currentSampleIndex])
	{
	case EVAL_DIM::X:
		index = currentSampleIndex*6+0;
		break;
	case EVAL_DIM::Y:
		index = currentSampleIndex*6+1;
		break;
	case EVAL_DIM::Z:
		index = currentSampleIndex*6+2;
		break;
	case EVAL_DIM::ROLL:
		index = currentSampleIndex*6+3;
		break;
	case EVAL_DIM::PITCH:
		index = currentSampleIndex*6+4;
		break;
	case EVAL_DIM::YAW:
		index = currentSampleIndex*6+5;
		break;
	};
	localMinima[index] = true;
}

void SA::setNextTransformations()
{
		HomogeneTransformation::DIM_DIR minusDir, plusDir;
		switch(currentDim[currentSampleIndex])
		{
		case EVAL_DIM::X:
			minusDir = HomogeneTransformation::DIM_DIR::XM;
			plusDir = HomogeneTransformation::DIM_DIR::XP;
			break;
		case EVAL_DIM::Y:
			minusDir = HomogeneTransformation::DIM_DIR::YM;
			plusDir = HomogeneTransformation::DIM_DIR::YP;
			break;
		case EVAL_DIM::Z:
			minusDir = HomogeneTransformation::DIM_DIR::ZM;
			plusDir = HomogeneTransformation::DIM_DIR::ZP;
			break;
		case EVAL_DIM::ROLL:
			minusDir = HomogeneTransformation::DIM_DIR::ROLLM;
			plusDir = HomogeneTransformation::DIM_DIR::ROLLP;
			break;
		case EVAL_DIM::PITCH:
			minusDir = HomogeneTransformation::DIM_DIR::PITCHM;
			plusDir = HomogeneTransformation::DIM_DIR::PITCHP;
			break;
		case EVAL_DIM::YAW:
			minusDir = HomogeneTransformation::DIM_DIR::YAWM;
			plusDir = HomogeneTransformation::DIM_DIR::YAWP;
			break;
		};

		nextEvalMinus = sampleConfig[currentSampleIndex].findNN(currentTrans[currentSampleIndex], minusDir);
		nextEvalPlus = sampleConfig[currentSampleIndex].findNN(currentTrans[currentSampleIndex],  plusDir);	
}


void SA::performRandomJump()
{
	for(int i=0; i<n; i++)
	{
		currentTrans[i] = sampleConfig[i].getRandom();
	}
}

void SA::resetDim()
{
	for(int i=0; i<n; i++)
	{
		currentDim[i] = SA::EVAL_DIM::X;
	}
}

#define e (2.7182818284590452353602874713526624977572)
bool SA::nextIteration(double cost_m, double cost_p)
{

	double prop, newprop;

	switch(state)
	{
	case STATE::HC:
/****************************************************/
		if (currentCosts <= std::min(cost_m, cost_p))
		{
			//set local minima
			setLocalMinima();
		}
		else
		{
			if(cost_m < cost_p)
			{
				//setting minus to current
				this->currentTrans[currentSampleIndex] = this->nextEvalMinus;
				currentCosts = cost_m;
			}else
			{
				//setting plus to current
				this->currentTrans[currentSampleIndex]  = this->nextEvalPlus;
				currentCosts = cost_p;
			}
		}
		//find new dimension
		nextDim();
		setNextTransformations();

		//check condition for random jump
		if(isLocalMinimaReached())
		{
			performRandomJump();
			resetLocalMinima();
			state = STATE::RJ;
		}
		break;
	case STATE::RJ:
/****************************************************/
		//check condition for acceptance

		prop = (rand() / RAND_MAX);
		newprop = pow(e, (currentCosts - std::min(cost_m, cost_p)) / T);
		
		if(prop <= newprop)		
		{
			
			resetDim();
			setNextTransformations();
			state = STATE::HC;
			T *= alpha;
		}
		break;
	case STATE::FI:		
		break;
	};


	if(T < minThres)
	{
		state = STATE::FI;
		return false;
	}else
	{
		return true;
	}
	
}
