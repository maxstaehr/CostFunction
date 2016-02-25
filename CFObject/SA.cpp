#include "SA.h"
#include <algorithm>

SA::SA(SampleCameraConfiguration& sampleConfig):Search(sampleConfig), currentDim(EVAL_DIM::X), state(STATE::HC), T(1.0), alpha(0.9), minThres(0.1)
{
	resetLocalMinima();
}



SA::~SA(void)
{


}

void SA::setCurrentTransformation(HomogeneTransformation h)
{
	currentTrans = h;
	currentDim = EVAL_DIM::X;
	HomogeneTransformation::DIM_DIR minusDir, plusDir;
	minusDir = HomogeneTransformation::DIM_DIR::XM;
	plusDir = HomogeneTransformation::DIM_DIR::XP;
	nextEvalMinus = sampleConfig.findNN(currentTrans, minusDir);
	nextEvalPlus = sampleConfig.findNN(currentTrans,  plusDir);
	resetLocalMinima();
}

void SA::nextDim()
{
	switch(currentDim)
	{
	case EVAL_DIM::X:
		currentDim = EVAL_DIM::Y;
		break;
	case EVAL_DIM::Y:
		currentDim = EVAL_DIM::Z;
		break;
	case EVAL_DIM::Z:
		currentDim = EVAL_DIM::ROLL;
		break;
	case EVAL_DIM::ROLL:
		currentDim = EVAL_DIM::PITCH;
		break;
	case EVAL_DIM::PITCH:
		currentDim = EVAL_DIM::YAW;
		break;
	case EVAL_DIM::YAW:
		currentDim = EVAL_DIM::X;
		break;
	};
}

void SA::resetLocalMinima()
{
	for(int i=0; i<6; i++)
	{
		localMinima[i] = false;
	}
}
bool SA::isLocalMinimaReached()
{
	bool ret = true;
	for(int i=0; i<6; i++)
	{
		ret &= localMinima[i];
	}
	return ret;
}

void SA::setLocalMinima()
{
	int index = -1;
	switch(currentDim)
	{
	case EVAL_DIM::X:
		index = 0;
		break;
	case EVAL_DIM::Y:
		index = 1;
		break;
	case EVAL_DIM::Z:
		index = 2;
		break;
	case EVAL_DIM::ROLL:
		index = 3;
		break;
	case EVAL_DIM::PITCH:
		index = 4;
		break;
	case EVAL_DIM::YAW:
		index = 5;
		break;
	};
	localMinima[index] = true;
}

void SA::setNextTransformations()
{
		HomogeneTransformation::DIM_DIR minusDir, plusDir;
		switch(currentDim)
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

		nextEvalMinus = sampleConfig.findNN(currentTrans, minusDir);
		nextEvalPlus = sampleConfig.findNN(currentTrans,  plusDir);	
}


void SA::performRandomJump()
{
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
				this->currentTrans = this->nextEvalMinus;
				currentCosts = cost_m;
			}else
			{
				//setting plus to current
				this->currentTrans = this->nextEvalPlus;
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
			currentDim = SA::EVAL_DIM::X;
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
