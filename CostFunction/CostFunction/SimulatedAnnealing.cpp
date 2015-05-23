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
#include "mathcuda.h"

#include "CudaMem.h"
#include "IO.h"

	//		int pa_index_m1 = i*2*nC+0*nC+cam1;
	//		int pa_index_p1 = i*2*nC+1*nC+cam1;

/*
 3 cameras, 2 configurationen, 2 energy, 4 costs
 <--------------energy0-------------><--------------energy1------------->
 <-----cost0------><-----cost1------><-----cost2------><-----cost3------>
 [[[c0-][c1-][c2-]][[c0+][c1+][c2+]]][[[c0-][c1-][c2-]][[c0+][c1+][c2+]]]
*/



static double calcDistance(double xd, double yd, double zd)
{
	 return sqrt(pow(xd,2.0)+pow(yd,2.0)+pow(zd,2.0));
}
double const SimulatedAnnealing::e( 2.7182818284590452353602874713526624977572 );


int inline SimulatedAnnealing::getIndex(int enegeryIndex, int sign, int cam)
{
	return enegeryIndex*2*nC+sign*nC+cam;
}

SimulatedAnnealing::SimulatedAnnealing(SAMPLE_PCL* sp, SAMPLE_ROTATIONS* sr, VALID_POS* vp, int nC, int nI, int* nn_indices, AngleGenerator* ag):SearchClass(sp, sr, vp, nC, nI, nn_indices, ag),ite(0),neighbourRadius(1.2), neighbourAngle(2.4), firstEvaluation(true), NofIteration(SIM_ITE*800), maxIteration(nC*NofIteration), firstIteration(true), min_threshold(1.0e-1)
{
	assert(nI>0 && nI%2==0);
	NofE = (int)nI/2;
	Npcl = sp->n;
	Nangle = sr->nRotations;
	nOfCams = nC;
	robot = sp;
	globalMin.costs = DBL_MAX;

	minEnergy = new double [NofE];
	DOF = 6*nOfCams;
	noChange = new bool[NofE*DOF];
	cDim = new unsigned char[NofE];	
	hasImproved = new int[NofE];
	nsRuns = new int[NofE];

	pclIndex_t1 = new int[nOfCams*nI];
	angleIndex_t1 = new int[nOfCams*nI];
	minEnergy_t1 = new double[NofE];
	
	globalMin.minEnergy = 1.0;


	this->solu = new struct SOLUTION[NofE];
	for(int i=0; i<NofE; i++)
	{
		this->solu[i].angle = new int[nC];
		memset(this->solu[i].angle, 0, nC*sizeof(int));
		this->solu[i].pcl = new int[nC];
		memset(this->solu[i].pcl, 0, nC*sizeof(int));
		minEnergy_t1[i] = DBL_MAX;
		hasImproved[i] = DOF;
		nsRuns[i] = 0;
	}

	this->curStates = new enum STATE[NofE];
	this->globalMin.angle = new int[nC];
	this->globalMin.pcl = new int[nC];
	this->globalMin.costs = DBL_MAX;	


	


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
		curStates[i] = STATE::NS;
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

void SimulatedAnnealing::setCoolingPlan(double const * const costs)
{	
	time_t end;
	time(&end);
	loopTime = difftime(end, start);

	float maxTemperature(FLT_MIN);
	for(unsigned int i=0; i<2*NofE; i++)
	{
		if(1.0-costs[i] > maxTemperature)
		{
			maxTemperature = 1.0-costs[i];
		}
	}
	//float maxTemperature = 1.0;
	T = maxTemperature/2.0;	
	min_threshold = T/5.0;

	double basis = min_threshold/T;
	double expo = 1.0/((double)nOfCams*maxIteration);
	alpha = pow(basis, expo);
	assert(alpha < 1.0);

	maxTime = maxIteration*loopTime;
}

void SimulatedAnnealing::writeResultsToFile(unsigned long long* vec, int nOfCams,  struct SAMPLE_POINTS_BUFFER* samplePoints)
{
	std::string suffix = "";
	for(int i=0; i<nOfCams; i++)
	{
		suffix += "_";
		suffix += std::to_string((_Longlong)vec[i]);
		
	}
	std::string fn = "cost_behave" +suffix;
	writeEnergyResultsToFile(fn);
	fn = "cost_energy" + suffix;
	writeAllResultToFile(fn);
	fn = "min_cost" + suffix;
	printMinPositionToFile(fn, samplePoints);

	//writeMinCostToFile(vec);
	
}

SimulatedAnnealing::~SimulatedAnnealing(void)
{
	delete minEnergy;
	delete noChange;
	delete cDim;
	for(int i=0; i<NofE; i++)
	{
		delete solu[i].angle;
		delete solu[i].pcl;
	}
	delete solu;
	delete curStates;
	delete pclIndex_t1;
	delete angleIndex_t1;
	delete globalMin.angle;
	delete globalMin.pcl;

	
}





bool SimulatedAnnealing::isConfigurationValidAssert(int* pI, int i, int sign)
{
	//checking the pcl index by doing a cross product check
	double d,xd, yd, zd;
	int pcl_i1, pcl_i2;
	bool isValid = true;

	for(int cam1=0; cam1<this->nC; cam1++)
	{
		for(int cam2=0; cam2<this->nC; cam2++)
		{
			if(cam1 == cam2)
				continue;

			pcl_i1 = getIndex(i, sign, cam1);
			pcl_i2 = getIndex(i, sign, cam2);

			xd = robot->x[pI[pcl_i1]] - robot->x[pI[pcl_i2]];
			yd = robot->y[pI[pcl_i1]] - robot->y[pI[pcl_i2]];
			zd = robot->z[pI[pcl_i1]] - robot->z[pI[pcl_i2]];
			d = calcDistance(xd, yd, zd);
			if(!(MIN_DIST_BETWEEN_CAMERA < d))
			{
				printf("error\n");
			}
			assert(MIN_DIST_BETWEEN_CAMERA < d);
			isValid &= (MIN_DIST_BETWEEN_CAMERA < d);
		}
	}

	return isValid;

}



bool SimulatedAnnealing::isConfigurationValid(int* pI, int i, int sign)
{
	//checking the pcl index by doing a cross product check
	double d,xd, yd, zd;
	int pcl_i1, pcl_i2;
	bool isValid = true;

	for(int cam1=0; cam1<this->nC; cam1++)
	{
		for(int cam2=0; cam2<this->nC; cam2++)
		{
			if(cam1 == cam2)
				continue;

			pcl_i1 = getIndex(i, sign, cam1);
			pcl_i2 = getIndex(i, sign, cam2);

			xd = robot->x[pI[pcl_i1]] - robot->x[pI[pcl_i2]];
			yd = robot->y[pI[pcl_i1]] - robot->y[pI[pcl_i2]];
			zd = robot->z[pI[pcl_i1]] - robot->z[pI[pcl_i2]];
			d = calcDistance(xd, yd, zd);
			//printf("%.4f\n", d);
			//assert(d < MIN_DIST_BETWEEN_CAMERA);
			isValid &= (MIN_DIST_BETWEEN_CAMERA < d);
		}
	}
	
	return isValid;

}




void SimulatedAnnealing::chooseRandomConfiguration(int* pclIndex, int* angleIndex, int index)
{

	
	
	// generating random configuration until we have found a valid configuration

	bool isValid;	
	int pcl_i1, sign =0;
	int a, p;
	//generating pcl until constraints fit
	do{		
		//generating random configuration for a single camera
		for(int cam = 0; cam<nC; cam++)
		{
			pcl_i1 = getIndex(index, sign, cam);			
			do
			{
				a = aG->generateRandomAngle();
				p = rand()%Npcl;

				angleIndex[pcl_i1] = a;
				pclIndex[pcl_i1] = p;

				angleIndex_t1[pcl_i1] = a;
				pclIndex_t1[pcl_i1] = p;

			}while(!isPositionValid(pclIndex[pcl_i1], angleIndex[pcl_i1]));			
		}			
		
	}while(!isConfigurationValid(pclIndex, index, sign));

	sign = 1;
	//generating pcl until constraints fit
	do{		
		//generating random configuration for a single camera
		for(int cam = 0; cam<nC; cam++)
		{
			pcl_i1 = getIndex(index, sign, cam);			
			do
			{
				a = aG->generateRandomAngle();
				p = rand()%Npcl;

				angleIndex[pcl_i1] = a;
				pclIndex[pcl_i1] = p;

				angleIndex_t1[pcl_i1] = a;
				pclIndex_t1[pcl_i1] = p;
			}while(!isPositionValid(pclIndex[pcl_i1], angleIndex[pcl_i1]));			
		}					
	}while(!isConfigurationValid(pclIndex, index, sign));	
}

void SimulatedAnnealing::findGlobalMinimum(int i)
{
	
	if(solu[i].costs < globalMin.costs)
	{
		// not allowed because copying pointer is not good
		// globalMin = solu[i];


		globalMin.costs = solu[i].costs;
		globalMin.globalMin = solu[i].globalMin;
		globalMin.currProp = solu[i].currProp;
		globalMin.curT = solu[i].curT;
		globalMin.ite = solu[i].ite;
		globalMin.minEnergy = solu[i].minEnergy;
		globalMin.state = solu[i].state;
		memcpy(globalMin.pcl, solu[i].pcl, nC*sizeof(int));
		memcpy(globalMin.angle, solu[i].angle, nC*sizeof(int));

		//setting global minimum solution
		//memcpy(minSol.aI, 
	}
	
}

void SimulatedAnnealing::printCurrentStatus(void)
{
	system("cls");	
	//system("cls");	
	//printf("minI\tST\tminE\t\tcosts\t\tprob\t\tpcl\tangle\n");
	//for(unsigned int i=0; i< 50 && i< NofE; i++)
	//{
	//	
	//	if(curStates[i] == STATE::HC)
	//	{
	//		printf("%i\tHC\t%.5f\t\t%.5f\t\t%.10f\t%i\t%i  %i\n", i, minEnergy[i], solu[i].costs, solu[i].currProp, solu[i].pcl[0], solu[i].angle[0], cDim[i]);
	//	}else if(curStates[i] == STATE::NS)
	//	{
	//		printf("%i\tNS\t%.5f\t\t%.5f\t\t%.10f\t%i\t%i  %i\n", i, minEnergy[i], solu[i].costs, solu[i].currProp, solu[i].pcl[0], solu[i].angle[0], cDim[i]);
	//	}else if (curStates[i] == STATE::OR)
	//	{
	//		printf("%i\tOR\t%.5f\t\t%.5f\t\t%.10f\t%i\t%i  %i\n", i, minEnergy[i], solu[i].costs, solu[i].currProp, solu[i].pcl[0], solu[i].angle[0], cDim[i]);
	//	}
	//	
	//}
	//double currPro = (double)ite/(double)(maxIteration);
	//double remainingTime = loopTime * (double) (maxIteration - ite);
	//printf("max : %.1fmin\trem : %.2fmin\tpro: %.6f%%\n", maxTime/60.0, remainingTime/60.0, currPro*100.0);
	//printf("global min costs at: %.10f at %i:%i at T:%.3f  pos:%i  cams:%i\n\n\n", globalMin.costs, globalMin.pcl[0], globalMin.angle[0], T, ite, nOfCams);

	std::string state;
	switch(curStates[0]){
	case STATE::HC:
		state = "HC";
		break;
	case STATE::NS:
		state = "NS";
		break;
	case STATE::OR:
		state = "OR";
		break;
	};

	
	printf("ST\tminE\tcosts\t\tprob\n");
	for(int i=0; i<NofE; i++)
	{
		printf("%s\t%.5f\t%.5f\t%.10f\t%d\t%d\n",   state.c_str(),minEnergy[i], solu[i].costs, solu[i].currProp, hasImproved[i], nsRuns[i]);
	}

	printf("\n\ncam\tpcl\troll\tpitch\tyaw\tdim\n");
	for(int i=0; i<nC; i++)
	{			
		int roll, pitch, yaw;
		convertAItoRPY(solu[0].angle[i], sr, &roll, &pitch, &yaw);
		int dim = (cDim[0]+DOF-1)%DOF;
		printf("%i\t%i\t%i\t%i\t%i\t%i\n", i, solu[0].pcl[i], roll, pitch, yaw, cDim[0]);
	}
	printf("\n\n\n");

		


	double currPro = (double)ite/(double)(maxIteration);
	double remainingTime = loopTime * (double) (maxIteration - ite);
	printf("max : %.1fmin\trem : %.2fmin\tpro: %.6f%%\n", maxTime/60.0, remainingTime/60.0, currPro*100.0);
	printf("global min costs at: %.10f at T:%.3f  pos:%i  cams:%i\n\n\n", globalMin.costs,T, ite, nOfCams);
	printf("cam\tpcl\tangle\troll\tpitch\tyaw\n");
	for(int i=0; i<nC; i++)
	{			
		int roll, pitch, yaw;
		convertAItoRPY(globalMin.angle[i], sr, &roll, &pitch, &yaw);		
		printf("%i\t%i\t%i\t%i\t%i\n",i, globalMin.pcl[i], globalMin.angle[i], roll, pitch, yaw);
	}
	printf("\n\n\n");

	
	
}


void SimulatedAnnealing::writeMinCostToFile(unsigned long long* vec)
{
	int *buffer = new int[nC];
	for(int i=0; i<nC; i++)
	{
		buffer[i] = (int)vec[i];
	}

	std::string fn = "resultingSolution.bin";
	std::ofstream outbin( fn.c_str(), std::ios::binary );

	printf("optimal solution: \n");
	for(int i=0; i<nC; i++)
	{
		printf("%d\t%d\n", globalMin.pcl[i], globalMin.angle[i]);
	}


	outbin.write((char*)&nC, sizeof(int));
	outbin.write((char*)buffer, nC*sizeof(int));
	outbin.write((char*)globalMin.pcl,	nC*sizeof(int));	
	outbin.write((char*)globalMin.angle, nC*sizeof(int));
	outbin.close();
	delete buffer;

}


void SimulatedAnnealing::resetHasChanged(int i)
{
	//for(unsigned int j=0; j<DOF; j++)
	//{
	//	noChange[i*DOF+j] = false;
	//}
	hasImproved[i] = DOF;
}

bool SimulatedAnnealing::hasHillClimbingFinished(int i)
{
	//bool hasFinished = true;
	//for(unsigned int j=0; j<DOF; j++)
	//{
	//	hasFinished &= noChange[i*DOF+j];
	//}
	//return hasFinished; 
	return hasImproved[i] < 0; 
}

void SimulatedAnnealing::setXYZ(int* const pclIndex, int*const angleIndex, int enegeryIndex, int dim, int cam)
{
	
	int sign = 0;
	int index = getIndex(enegeryIndex, sign, cam);
	int p_i = pclIndex[index];
	int pi_new = nn_indices[p_i*6+dim*2+sign];

	//checking if consistent before
	assert(areAllPositionsValidAssert(pclIndex, angleIndex));
	//setting new version an checking if consistent
	pclIndex[index] = pi_new;
	if(!areAllPositionsValid(pclIndex, angleIndex))
	{
		//setting back to previous and check agains
		pclIndex[index] = p_i;
	}
	assert(areAllPositionsValidAssert(pclIndex, angleIndex));

	//doing the same for the plus sign
	sign = 1;
	index = getIndex(enegeryIndex, sign, cam);
	p_i = pclIndex[index];
	pi_new = nn_indices[p_i*6+dim*2+sign];
	//checking if consistent before
	assert(areAllPositionsValidAssert(pclIndex, angleIndex));
	//setting new version an checking if consistent
	pclIndex[index] = pi_new;
	if(!areAllPositionsValid(pclIndex, angleIndex))
	{
		//setting back to previous and check agains
		pclIndex[index] = p_i;
	}
	assert(areAllPositionsValidAssert(pclIndex, angleIndex));
	
}
void SimulatedAnnealing::setAngle(int* const pclIndex, int*const angleIndex, int enegeryIndex, int dim, int cam)
{
	int ri, pi, yi, t; 
	int ai_new;	
	
	int sign = 0;
	int index = getIndex(enegeryIndex, sign, cam);
	int a_i = angleIndex[index];


		
	convertAItoRPY(a_i, sr, &ri, &pi, &yi);	
	switch(dim){
	case 3:
		t = (ri+sr->nRoll-1)%sr->nRoll;
		convertRPYtoAI(t, pi, yi, sr, &ai_new);
		break;
	case 4:
		t = (pi+sr->nPitch-1)%sr->nPitch;
		convertRPYtoAI(ri, t, yi, sr, &ai_new);
		break;
	case 5:
		t = (yi+sr->nYaw-1)%sr->nYaw;
		convertRPYtoAI(ri, pi, t, sr, &ai_new);
		break;
	default:
		assert(false);
	};

	//check first if is allright
	assert(areAllPositionsValidAssert(pclIndex, angleIndex));
	//setting new version an checking if consistent
	angleIndex[index] = ai_new;
	if(!areAllPositionsValid(pclIndex, angleIndex))
	{
		//setting back to previous and check agains
		angleIndex[index] = a_i;
	}
	assert(areAllPositionsValidAssert(pclIndex, angleIndex));


	sign = 1;
	index = getIndex(enegeryIndex, sign, cam);
	a_i = angleIndex[index];
	convertAItoRPY(a_i, sr, &ri, &pi, &yi);	
	switch(dim){
	case 3:		
		t = (ri+1)%sr->nRoll;
		convertRPYtoAI(t, pi, yi, sr, &ai_new);
		break;
	case 4:
		t = (pi+1)%sr->nPitch;
		convertRPYtoAI(ri, t, yi, sr, &ai_new);
		break;
	case 5:
		t = (yi+1)%sr->nYaw;
		convertRPYtoAI(ri, pi, t, sr, &ai_new);
		break;
	default:
		assert(false);
	};

	//check first if is allright
	assert(areAllPositionsValidAssert(pclIndex, angleIndex));
	//setting new version an checking if consistent
	angleIndex[index] = ai_new;
	if(!areAllPositionsValid(pclIndex, angleIndex))
	{
		//setting back to previous and check agains
		angleIndex[index] = a_i;
	}
	assert(areAllPositionsValidAssert(pclIndex, angleIndex));


}

double SimulatedAnnealing::iterateSingle(const int* const nn_indices, int* const pclIndex, int*const angleIndex, double const*const costs, int i)
{

	assert(areAllPositionsValidAssert(pclIndex, angleIndex));
	int p_i, a_i;
	double localMinE = 0.0;

	double cm = 1.0-costs[2*i+0];
	double cp = 1.0-costs[2*i+1];

	int dim = cDim[i]%6;
	int cam = cDim[i]/6;
	
	int pa_index_m = getIndex(i,0,cam);;
	int pa_index_p = getIndex(i,1,cam);;
	




	if(cm < cp)
	{	//change to minus
		//copying the complete solution to plus
		localMinE = cm;

		p_i = pclIndex[pa_index_m];
		a_i = angleIndex[pa_index_m];

		memcpy(angleIndex+i*2*nC+1*nC, angleIndex+i*2*nC+0*nC, nC*sizeof(int));
		memcpy(pclIndex+i*2*nC+1*nC, pclIndex+i*2*nC+0*nC, nC*sizeof(int));

		memcpy(solu[i].angle, angleIndex+i*2*nC+0*nC, nC*sizeof(int));
		memcpy(solu[i].pcl, pclIndex+i*2*nC+0*nC, nC*sizeof(int));

		solu[i].costs = cm;

		
	}else
	{	//change to plus
		//copying the complete solution to minus
		// cm > cp
		localMinE = cp;

		p_i = pclIndex[pa_index_p];
		a_i = angleIndex[pa_index_p];	

		memcpy(angleIndex+i*2*nC+0*nC, angleIndex+i*2*nC+1*nC, nC*sizeof(int));
		memcpy(pclIndex+i*2*nC+0*nC, pclIndex+i*2*nC+1*nC, nC*sizeof(int));

		memcpy(solu[i].angle, angleIndex+i*2*nC+1*nC, nC*sizeof(int));
		memcpy(solu[i].pcl, pclIndex+i*2*nC+1*nC, nC*sizeof(int));

		solu[i].costs = cp;

		
	}
	findGlobalMinimum(i);





	

	if(localMinE < minEnergy_t1[i])
	{
		//there has been actual improvement
		//solution is stored into history
		memcpy(pclIndex_t1+i*2*nC, pclIndex+i*2*nC, 2*nOfCams*sizeof(int));
		memcpy(angleIndex_t1+i*2*nC, angleIndex+i*2*nC, 2*nOfCams*sizeof(int));
		memcpy(minEnergy_t1+i, minEnergy+i, sizeof(double));
		
	}else
	{
		//there has no actual improvement 
		//the state before mutation is restored
		memcpy(pclIndex+i*2*nC, pclIndex_t1+i*2*nC, 2*nOfCams*sizeof(int));
		memcpy(angleIndex+i*2*nC, angleIndex_t1+i*2*nC, 2*nOfCams*sizeof(int));
		memcpy(minEnergy+i, minEnergy_t1+i, sizeof(double));
		
	}
	


	
	cDim[i] = (cDim[i]+1)%DOF;
	dim = cDim[i]%6;
	cam = cDim[i]/6;
	

	


	
	switch(dim){
	case 0:		
	case 1:
	case 2:
		setXYZ(pclIndex, angleIndex, i, dim, cam);
		break;
	case 3:
	case 4:
	case 5:
		setAngle(pclIndex, angleIndex, i, dim, cam);
		break;
	default:
		assert(false);
	};


	assert(angleIndex[pa_index_p] >= 0 && angleIndex[pa_index_p] <sr->nRotations);
	assert(angleIndex[pa_index_m] >= 0 && angleIndex[pa_index_m] <sr->nRotations);
	



	//printf("iterate single...\n");
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

//(struct SAMPLE_POINTS_BUFFER* samplePoints, struct SAMPLE_PCL* sP,struct SAMPLE_ROTATIONS* sR, float* costs, float* d,int* weights,  const char* name)
//{
//	float* buffer = new float[samplePoints->n*NUMELEM_H];
//	float* res_pos_buffer = new float[samplePoints->n*sR->nRotations*NUMELEM_H];
//
//	//float* x = new float[samplePoints->n];
//	//float* y = new float[samplePoints->n];
//	//float* z = new float[samplePoints->n];
//	/*float* max = new float[samplePoints->n*sample];*/
//
//	CudaMem::cudaMemCpyReport(buffer, samplePoints->d_H, samplePoints->n*NUMELEM_H*sizeof(float), cudaMemcpyDeviceToHost);
//	//calculating the resulting transformations
//	for(int i=0; i<samplePoints->n; i++)
//	{
//		for(int j=0; j<sR->nRotations; j++)
//		{
//			mm16(buffer+i*NUMELEM_H, sR->R+j*NUMELEM_H, res_pos_buffer+(i*sR->nRotations+j)*NUMELEM_H);
//		}
//	}

void SimulatedAnnealing::printMinPositionToFile(std::string pre, struct SAMPLE_POINTS_BUFFER* samplePoints)
{
	float* buf1 = new float[nC*NUMELEM_H];
	float* buf2 = new float[nC*NUMELEM_H];
	int*   buf3 = new int[nC];
	int*   buf4 = new int[nC];

	int index = 0;
	for(int i=0; i<nC; i++)
	{
		index = globalMin.pcl[i];
		buf3[i] = index;
		CudaMem::cudaMemCpyReport(buf1+i*NUMELEM_H, samplePoints->d_H+index*NUMELEM_H, NUMELEM_H*sizeof(float), cudaMemcpyDeviceToHost);
		index = globalMin.angle[i];
		buf4[i] = index;
		mm16(buf1+i*NUMELEM_H, this->sr->R+index*NUMELEM_H, buf2+i*NUMELEM_H);
	}


	std::string fn = "minCameraPos" + pre+ ".bin";
	std::ofstream outbin(fn, std::ofstream::binary );

	float globalMinFloat = (float)globalMin.costs;
	outbin.write((char*)&nC, sizeof(int));
	outbin.write((char*)buf2, nC*NUMELEM_H*sizeof(float));
	outbin.write((char*)&globalMinFloat, sizeof(float));
	outbin.write((char*)buf3, nC*sizeof(int));
	outbin.write((char*)buf4, nC*sizeof(int));
	outbin.close();

	delete buf1;
	delete buf2;
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
	float* buffer = new float[nDouble];

	buffer[0] = n;
	buffer[1] = NofE;
	for(unsigned int i=0; i<n; i++)
	{
		memcpy(buffer+i*NofE+2, recordedSolution[i], NofE*sizeof(float)); 
	}

	std::string fn = pre + ".bin";
	std::ofstream outbin( fn.c_str(), std::ios::binary );
	outbin.write((char*)buffer, (nDouble)*sizeof(float));
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



bool SimulatedAnnealing::iterate(int* const pclIndex, int *const angleIndex, double const *const costs, float const *const d, int const *const weights)
{
	//for(int i=0; i<2*NofE; i++)
	//{
	//	printf("energy: %d\t%.10lf\n", i, costs[i]);
	//}
	//IO::waitForEnter();
	if(firstIteration)
	{
		for(int energy=0;energy<NofE; energy++)
		{
			chooseRandomConfiguration(pclIndex, angleIndex, energy);
		}
		time(&start);
		firstIteration = false;
		assert(areAllPositionsValid(pclIndex, angleIndex) == true);
		return true;
	}else
	{
		if(firstEvaluation)
		{
			firstEvaluation = false;
			setCoolingPlan(costs);
		}
	}



	


	//iterating through all possible configuration
	int rp, ra;
	float localMinE;

	float* p = new float[NofE];
	recordedSolution.push_back(p);


	////saving current s 
	//memcpy(pclIndex_t1, pclIndex, nOfCams*MAX_ITE*sizeof(int));
	//memcpy(angleIndex_t1, angleIndex, nOfCams*MAX_ITE*sizeof(int));
	//memcpy(minEnergy_t1, minEnergy, NofE*sizeof(float));


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
				nsRuns[i]++;
				setNewVector(i);
				//printf("random jump...\n");
				chooseRandomConfiguration(pclIndex, angleIndex, i);			
			}

			//check if current alternation has brought improvement
			if(localMinE < minEnergy[i])
			{
				minEnergy[i] = localMinE;
				resetHasChanged(i);	
			}else
			{
				hasImproved[i]--;
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
				//printf("random config...\n");
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
	//if(!isCompleteConfigurationValid(pclIndex))
	//{
	//	printf("error\n");
	//	IO::waitForEnter();
	//}

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

	//if(T >min_threshold)
	addResult();	
	printCurrentStatus();
	assert(areAllPositionsValid(pclIndex, angleIndex) == true);
	if(ite<maxIteration && abs(globalMin.costs) > 1e-8f)
	{
		T *= alpha;
		return true;
	}else
	{
		printf("finishing sa\n");
		return false;
	}
}



bool SimulatedAnnealing::areAllPositionsValidAssert(int*const pI, int*const aI)
{
	bool valid = true;
	int index = 0;
	for(int energy=0; energy<NofE; energy++)
	{
		for(int sign=0; sign<2; sign++)
		{
			for(int cam=0;cam<nC; cam++)
			{
				index = getIndex(energy, sign, cam);	
				valid &= isPositionValid(pI[index], aI[index]);	
			}
			valid &= isConfigurationValidAssert(pI, energy, sign);
		}
				
	}
	return valid;
}

bool SimulatedAnnealing::areAllPositionsValid(int* pI, int* aI)
{
	bool valid = true;
	int index = 0;
	for(int energy=0; energy<NofE; energy++)
	{
		for(int sign=0; sign<2; sign++)
		{
			for(int cam=0;cam<nC; cam++)
			{
				index = getIndex(energy, sign, cam);	
				valid &= isPositionValid(pI[index], aI[index]);	
			}
			valid &= isConfigurationValid(pI, energy, sign);
		}
				
	}
	return valid;
}