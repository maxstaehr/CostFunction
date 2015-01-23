#include "SimulatedAnnealing.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <float.h>
#include <iostream>
#include <string>
#include <assert.h>
#include <fstream>
#include <iostream>

#include "global.h"
  



double const SimulatedAnnealing::e( 2.7182818284590452353602874713526624977572 );


SimulatedAnnealing::SimulatedAnnealing(SAMPLE_PCL* sp, SAMPLE_ROTATIONS* sr, int nC, int nI, int* nn_indices):SearchClass(sp, sr, nC, nI, nn_indices),ite(0),neighbourRadius(1.2), neighbourAngle(2.4), firstEvaluation(true), NofIteration(300), maxIteration(nOfCams*NofIteration)
{
	NofE = (int)nI/2;

	Npcl = sp->n;
	Nangle = sr->nRotations;
	nOfCams = nC;

	minEnergy = new double [NofE];
	DOF = 6*nOfCams;
	noChange = new bool[NofE*DOF];
	cDim = new unsigned char[NofE];	

	pclIndex_t1 = new int[nOfCams*MAX_ITE];
	angleIndex_t1 = new int[nOfCams*MAX_ITE];


	this->solu = new struct SOLUTION[NofE];
	this->curStates = new enum STATE[NofE];
	this->globalMin.costs = DBL_MAX;
	this->NN = NN;
	this->robot = robot;


	//checking if we have enough angle to iterate
	assert((sr->angleLimits[1]-sr->angleLimits[0])/sr->nRoll < 2*neighbourAngle);
	assert((sr->angleLimits[3]-sr->angleLimits[2])/sr->nPitch < 2*neighbourAngle);
	assert((sr->angleLimits[5]-sr->angleLimits[4])/sr->nYaw < 2*neighbourAngle);
	
	//we need to check this for the angles
	angleIndexRandomRange_roll = (2*neighbourAngle)/((sr->angleLimits[1]-sr->angleLimits[0])/sr->nRoll);
	angleIndexRandomRange_pitch = (2*neighbourAngle)/((sr->angleLimits[3]-sr->angleLimits[2])/sr->nPitch);
	angleIndexRandomRange_yaw = (2*neighbourAngle)/((sr->angleLimits[5]-sr->angleLimits[4])/sr->nYaw);


	for(unsigned int i=0; i<NofE; i++)
	{
		minEnergy[i] = DBL_MAX;
		cDim[i] = 0;
		for(unsigned int j=0; j<DOF; j++)
		{
			noChange[i*DOF+j] = false;
		}
		curStates[i] = STATE::HC;
		solu[i].currProp = 0.0;
	}



	this->T = nOfCams*T;
	this->alpha = alpha;
	
	time_t t;
	time(&t);
	srand((unsigned int)t);              /* Zufallsgenerator initialisieren */
}

//SimulatedAnnealing::SimulatedAnnealing(int NumerOfParrlelCalculation, double T, double alpha, int Npcl, int Nangle, int nOfCams, int* NN, struct PCL* robot):ite(0),
//	neighbourRadius(1.2), neighbourAngle(2.4), firstEvaluation(true), NofIteration(300), maxIteration(nOfCams*NofIteration)
//{
//
//	NofE = (int)NumerOfParrlelCalculation/2;
//	minEnergy = new double [NofE];
//	DOF = 6*nOfCams;
//	noChange = new bool[NofE*DOF];
//	cDim = new unsigned char[NofE];	
//
//	pclIndex_t1 = new int[nOfCams*MAX_ITE];
//	angleIndex_t1 = new int[nOfCams*MAX_ITE];
//
//	this->Nangle = Nangle;
//	this->Npcl = Npcl;
//	this->nOfCams = nOfCams;
//	this->solu = new struct SOLUTION[NofE];
//	this->curStates = new enum STATE[NofE];
//	this->globalMin.costs = DBL_MAX;
//	this->NN = NN;
//	this->robot = robot;
//
//	assert((2*MATH_PI/N_OF_A) < (2*neighbourAngle));
//	angleIndexRandomRange = (2*neighbourAngle)/(2*MATH_PI/N_OF_A);
//
//
//	for(unsigned int i=0; i<NofE; i++)
//	{
//		minEnergy[i] = DBL_MAX;
//		cDim[i] = 0;
//		for(unsigned int j=0; j<DOF; j++)
//		{
//			noChange[i*DOF+j] = false;
//		}
//		curStates[i] = STATE::HC;
//		solu[i].currProp = 0.0;
//	}
//
//
//
//	this->T = nOfCams*T;
//	this->alpha = alpha;
//	
//    time_t t;
//    time(&t);
//    srand((unsigned int)t);              /* Zufallsgenerator initialisieren */
//}

void SimulatedAnnealing::setCoolingPlan(float* costs)
{
	float maxTemperature(FLT_MIN);
	for(unsigned int i=0; i<2*NofE; i++)
	{
		if(costs[i] > maxTemperature)
		{
			maxTemperature = costs[i];
		}
	}
	T = maxTemperature/2.0;
	float basis = 1.0/((float)T);
	float expo = 1.0/((float)maxIteration);
	alpha = pow(basis, expo);
	time_t end;
	time(&end);
	loopTime = difftime(end, start);
	maxTime = maxIteration*loopTime;
}


SimulatedAnnealing::~SimulatedAnnealing(void)
{
	delete minEnergy;
	delete noChange;
	delete cDim;
	delete solu;
	delete curStates;
	delete pclIndex_t1;
	delete angleIndex_t1;
}

void SimulatedAnnealing::initializeFirstRun(int* pclIndex, int* angleIndex)
{
	for(unsigned int i=0; i<nOfCams*MAX_ITE; i++)
	{
		pclIndex[i] = rand() % Npcl;
		angleIndex[i] = rand() % Nangle;
	}
	time(&start);
}

int SimulatedAnnealing::createPCLIndexInRange(int i1)
{
	int i2;
	float xd, yd, zd;
	do
	{
		i2 = rand()%Npcl;
		xd = robot->x[i1] - robot->x[i2];
		yd = robot->y[i1] - robot->y[i2];
		zd = robot->z[i1] - robot->z[i2];
	}
	while(sqrt(pow(xd,2)+pow(yd,2)+pow(zd,2)) > neighbourRadius);
	return i2;
}
int SimulatedAnnealing::createAngleIndexInRange(int a_i)
{
	
	int ri = a_i/(sr->nPitch*sr->nYaw);
	int pi = (a_i - ri*sr->nPitch*sr->nYaw)/sr->nYaw;
	int yi = a_i-ri*sr->nPitch*sr->nYaw-pi*sr->nYaw;

	


	int di;
	di = (rand()%angleIndexRandomRange_roll) - angleIndexRandomRange_roll/2;
	ri = (ri+di+sr->nRoll)%sr->nRoll;

	di = (rand()%angleIndexRandomRange_pitch) - angleIndexRandomRange_pitch/2;
	pi = (pi+di+sr->nPitch)%sr->nPitch;

	di = (rand()%angleIndexRandomRange_yaw) - angleIndexRandomRange_yaw/2;
	yi = (yi+di+sr->nYaw)%sr->nYaw;


	return ri * sr->nPitch*sr->nYaw + pi * sr->nYaw + yi;
}

void SimulatedAnnealing::chooseRandomConfiguration(int* pclIndex, int* angleIndex, int index)
{

	for(int i=0; i<nOfCams; i++)
	{
		pclIndex[2*index+0+i*MAX_ITE] = createPCLIndexInRange(pclIndex_t1[2*index+0+i*MAX_ITE]);
		angleIndex[2*index+0+i*MAX_ITE] = createAngleIndexInRange(angleIndex_t1[2*index+0+i*MAX_ITE]);
		pclIndex[2*index+1+i*MAX_ITE] = createPCLIndexInRange(pclIndex_t1[2*index+1+i*MAX_ITE]);
		angleIndex[2*index+1+i*MAX_ITE] = createAngleIndexInRange(angleIndex_t1[2*index+1+i*MAX_ITE]);
	}

	
}

void SimulatedAnnealing::findGlobalMinimum(void)
{
	for(unsigned int i=0; i< NofE; i++)
	{
		if(solu[i].costs < globalMin.costs)
		{
			globalMin = solu[i];
		}
	}
}

void SimulatedAnnealing::printCurrentStatus(void)
{
	system("cls");	
	printf("minI\tST\tminE\t\tcosts\t\tprob\t\tpcl\tangle\n");
	for(unsigned int i=0; i< 50; i++)
	{
		
		if(curStates[i] == STATE::HC)
		{
			printf("%i\tHC\t%.5lf\t%.5f\t%.10f\t%i\t%i  %i\n", i, minEnergy[i], solu[i].costs, solu[i].currProp, solu[i].pcl, solu[i].angle, cDim[i]);
		}else if(curStates[i] == STATE::NS)
		{
			printf("%i\tNS\t%.5lf\t%.5f\t%.10f\t%i\t%i  %i\n", i, minEnergy[i], solu[i].costs, solu[i].currProp, solu[i].pcl, solu[i].angle, cDim[i]);
		}else if (curStates[i] == STATE::OR)
		{
			printf("%i\tOR\t%.5lf\t%.5f\t%.10f\t%i\t%i  %i\n", i, minEnergy[i], solu[i].costs, solu[i].currProp, solu[i].pcl, solu[i].angle, cDim[i]);
		}
		
	}
	double currPro = (double)ite/(double)(maxIteration);
	double remainingTime = loopTime * (double) (maxIteration - ite);
	printf("max : %.1fmin\trem : %.2fmin\tpro: %.6f%%\n", maxTime/60.0, remainingTime/60.0, currPro*100.0);
	printf("global min costs at: %.5f at %i:%i at T:%.3f  pos:%i  cams:%i\n\n\n", globalMin.costs, globalMin.pcl, globalMin.angle, T, ite, nOfCams);

	
	
}



void SimulatedAnnealing::resetHasChanged(int i)
{
	for(unsigned int j=0; j<DOF; j++)
	{
		noChange[i*DOF+j] = false;
	}
}

bool SimulatedAnnealing::hasHillClimbingFinished(int i)
{
	bool hasFinished = true;
	for(unsigned int j=0; j<DOF; j++)
	{
		hasFinished &= noChange[i*DOF+j];
	}
	return hasFinished; 
}

double SimulatedAnnealing::iterateSingle(const int* const nn_indices, int* pclIndex, int* angleIndex, float* costs, int i)
{
	int p_i, a_i;
	float localMinE = 0.0;


	if(costs[2*i+0] < costs[2*i+1])
	{	//change to minus
		localMinE = costs[2*i+0];
		p_i = pclIndex[2*i+0];
		a_i = angleIndex[2*i+0];
	}else
	{	//change to plus
		localMinE = costs[2*i+1];
		p_i = pclIndex[2*i+1];
		a_i = angleIndex[2*i+1];				
	}

	//double prop = (rand() / (double)RAND_MAX);
	//double newprop = pow(e, (minEnergy[i]-localMinE) / T);
	//		

	//if(prop <= newprop)
	if(localMinE < minEnergy[i])
	{	//has changed
	//	noChange[6*i+cDim[i]] = false;
		resetHasChanged(i);	
	}else{
		noChange[DOF*i+cDim[i]] = true;
	}


	//found best position setting next position
	solu[i].angle = a_i;
	solu[i].pcl = p_i;
	solu[i].costs = localMinE;
	cDim[i] = (cDim[i]+1)%DOF;
	
	

	int ri = a_i/(sr->nPitch*sr->nYaw);
	int pi = (a_i - ri*sr->nPitch*sr->nYaw)/sr->nYaw;
	int yi = a_i-ri*sr->nPitch*sr->nYaw-pi*sr->nYaw;


	int roll = ri, pitch = pi, yaw = yi;
	int am, ap;

	switch(cDim[i]%DOF){
	case 0:		
	case 1:
	case 2:
		//x,y,z
		pclIndex[2*i+0] = nn_indices[p_i*6+cDim[i]*2+0];
		pclIndex[2*i+1] = nn_indices[p_i*6+cDim[i]*2+1];	
		break;
	case 3:
		//roll
		am = (ri+sr->nRoll-1)%sr->nRoll;
		ap = (ri+sr->nRoll+1)%sr->nRoll;
		angleIndex[2*i+0] = am * sr->nRoll*sr->nPitch + pitch * sr->nYaw + yaw;
		angleIndex[2*i+0] = ap * sr->nRoll*sr->nPitch + pitch * sr->nYaw + yaw;
		break;
	case 4:
		//pitch
		am = (pi+sr->nPitch-1)%sr->nPitch;
		ap = (pi+sr->nPitch+1)%sr->nPitch;
		angleIndex[2*i+0] = roll * sr->nRoll*sr->nPitch + am * sr->nYaw + yaw;
		angleIndex[2*i+0] = roll * sr->nRoll*sr->nPitch + ap * sr->nYaw + yaw;
		break;
	case 5:
		//yaw
		am = (yi+sr->nRoll-1)%sr->nRoll;
		ap = (yi+sr->nRoll+1)%sr->nRoll;
		angleIndex[2*i+0] = roll * sr->nRoll*sr->nPitch + pitch * sr->nYaw + am;
		angleIndex[2*i+0] = roll * sr->nRoll*sr->nPitch + pitch * sr->nYaw + ap;
		break;
	};
	//assert(localMinE > 0.0);
	return localMinE;
}

void SimulatedAnnealing::writeEnergyResultsToFile(std::string pre)
{

	std::string fn = pre+".bin";
	int n = recordSol.size();
	double *buffer = new double[n*7];

	for(unsigned int j=0; j<n; j++)
	{
		//x,y,z,roll,pitch,yaw,costvalue
		buffer[j*7+0] = recordSol[j].ite;
		buffer[j*7+1] = recordSol[j].curT;
		buffer[j*7+2] = recordSol[j].minEnergy;
		buffer[j*7+3] = recordSol[j].state;
		buffer[j*7+4] = recordSol[j].currProp;
		buffer[j*7+5] = recordSol[j].costs;
		buffer[j*7+6] = recordSol[j].globalMin;
		
	}

	std::ofstream outbin( fn.c_str(), std::ios::binary );
	outbin.write((char*)buffer, n*7*sizeof(double));
	outbin.close();
	delete buffer;

	

}

void SimulatedAnnealing::addResult(void)
{
	solu[0].curT = T;
	solu[0].minEnergy = minEnergy[0];
	solu[0].state = curStates[0];
	solu[0].ite = ite;
	solu[0].globalMin = globalMin.costs;
	recordSol.push_back(solu[0]);
	
}


void SimulatedAnnealing::addSingleResult(int i)
{
	recordedSolution.back()[i] =  minEnergy[i];	
}

void SimulatedAnnealing::writeAllResultToFile(std::string pre)
{
	//calculating the memory buffer size
	int n = recordedSolution.size();
	int nDouble = NofE * n+1+1;
	double* buffer = new double[nDouble];

	buffer[0] = n;
	buffer[1] = NofE;
	for(unsigned int i=0; i<n; i++)
	{
		memcpy(buffer+i*NofE+2, recordedSolution[i], NofE*sizeof(double)); 
	}

	std::string fn = pre + ".bin";
	std::ofstream outbin( fn.c_str(), std::ios::binary );
	outbin.write((char*)buffer, (nDouble)*sizeof(double));
	outbin.close();
	delete buffer;
}

void SimulatedAnnealing::writeResultsToFile(std::string pre, struct PCL *pcl, struct POSITIONS *pos)
{
	//for(unsigned int i=0; i<solutions.size(); i++)
	//{
	//	std::string fn = pre+ std::to_string((_ULONGLONG)i)+".bin";
	//	int n = solutions[i].size();
	//	double *buffer = new double[n*8];

	//	for(unsigned int j=0; j<n; j++)
	//	{
	//		//x,y,z,roll,pitch,yaw,costvalue
	//		buffer[j*8+0] = pcl->x[solutions[i][j].pcl];
	//		buffer[j*8+1] = pcl->y[solutions[i][j].pcl];
	//		buffer[j*8+2] = pcl->z[solutions[i][j].pcl];
	//		buffer[j*8+3] = pos->roll[solutions[i][j].angle];
	//		buffer[j*8+4] = pos->pitch[solutions[i][j].angle];
	//		buffer[j*8+5] = pos->yaw[solutions[i][j].angle];
	//		buffer[j*8+6] = solutions[i][j].costs;
	//		buffer[j*8+7] = solutions[i][j].currProp;
	//	}

	//	std::ofstream outbin( fn.c_str(), std::ios::binary );
	//	outbin.write((char*)buffer, n*8*sizeof(double));
	//	outbin.close();
	//	delete buffer;

	//}

}
void SimulatedAnnealing::setNewVector(int i)
{
	if(i!=0)
		return;
	//if(solutions[solutions.size()-1].size() <= 12)
	//{
	//	solutions.pop_back();
	//}

	

}
//bool SimulatedAnnealing::iterate(int* pI, int* aI, float* prob)
//{
//	return true;
//}

bool SimulatedAnnealing::iterate(int* pclIndex, int* angleIndex, float* costs)
{
	
	if(firstEvaluation)
	{
		firstEvaluation = false;
		setCoolingPlan(costs);
	}

	//iterating through all possible configuration
	int rp, ra;
	float localMinE;

	float* p = new float[NofE];
	recordedSolution.push_back(p);


	//saving current s 
	memcpy(pclIndex_t1, pclIndex, nOfCams*MAX_ITE*sizeof(int));
	memcpy(angleIndex_t1, angleIndex, nOfCams*MAX_ITE*sizeof(int));


	for(unsigned int i=0; i<NofE; i++)
	{

		//setting next iteration and find local minimum
		localMinE = iterateSingle(nn_indices, pclIndex, angleIndex, costs, i);

		float prop;
		float newprop;

		switch(curStates[i]){
		case STATE::HC:
			//check if is still in HC
			if(hasHillClimbingFinished(i))
			{   //switch to ns
				curStates[i] = STATE::NS;
				setNewVector(i);
				chooseRandomConfiguration(pclIndex, angleIndex, i);			
			}

			//check if current alternation has brought improvement
			if(localMinE < minEnergy[i])
			{
				minEnergy[i] = localMinE;
			}
			break;
		case STATE::NS:
			//try to accept new bet
			prop = (rand() / (float)RAND_MAX);
			newprop = pow(e, (minEnergy[i]-localMinE) / T);
			solu[i].currProp = newprop;

			if(prop <= newprop)
			{	//switch to HC
				minEnergy[i] = localMinE;
				curStates[i] = STATE::HC;
				resetHasChanged(i);				
			}else
			{
				//generate new random configuration
				setNewVector(i);
				chooseRandomConfiguration(pclIndex, angleIndex, i);	
			}
			break;
		//case STATE::OR:
		//	prop = (rand() / (double)RAND_MAX);
		//	newprop = pow(e, (minEnergy[i]-localMinE) / T);
		//	solu[i].currProp = newprop;
		//	if(prop <= newprop)
		//	{	
		//		minEnergy[i] = localMinE;
		//	}else
		//	{
		//		memcpy(pclIndex+i*2,pclTemp, 2*sizeof(int));
		//		memcpy(angleIndex+i*2,angleTemp, 2*sizeof(int));
		//		iterateSingle(nn_indices, pclIndex, angleIndex, costs, i);
		//	}

		//	break;
		};
		addSingleResult(i);
	}
	//p_c_Solutions->push_back(solu[0]);
	addResult();
	findGlobalMinimum();
	printCurrentStatus();
	ite++;
	//std::cout << "waiting for key..." << std::endl;
	//std::string mystr;
	//getline (std::cin,mystr);
	//if(minCostOverAll == 0.0)
	//{
	//	std::cout << "global minimum found" << std::endl;
	//	std::string mystring;
	//	getline (std::cin, mystring);

	//}

	//if(solutions.size() > 99)
	//{
	//	return false;
	//}

	if(T >1)
	{
		T *= alpha;
		return true;
	}else
	{
		return false;
	}
}