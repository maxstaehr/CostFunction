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



double const SimulatedAnnealing::e( 2.7182818284590452353602874713526624977572 );


SimulatedAnnealing::SimulatedAnnealing(SAMPLE_PCL* sp, SAMPLE_ROTATIONS* sr, int nC, int nI, int* nn_indices, AngleGenerator* ag):SearchClass(sp, sr, nC, nI, nn_indices, ag),ite(0),neighbourRadius(1.2), neighbourAngle(2.4), firstEvaluation(true), NofIteration(800), maxIteration(nC*NofIteration), firstIteration(true), min_threshold(1.0e-1)
{
	assert(nI>0 && nI%2==0);
	NofE = (int)nI/2;
	Npcl = sp->n;
	Nangle = sr->nRotations;
	nOfCams = nC;
	robot = sp;

	minEnergy = new float [NofE];
	DOF = 6*nOfCams;
	noChange = new bool[NofE*DOF];
	cDim = new unsigned char[NofE];	

	pclIndex_t1 = new int[nOfCams*nI];
	angleIndex_t1 = new int[nOfCams*nI];
	minEnergy_t1 = new float[NofE];
	


	this->solu = new struct SOLUTION[NofE];
	for(int i=0; i<NofE; i++)
	{
		this->solu[i].angle = new int[nC];
		memset(this->solu[i].angle, 0, nC*sizeof(int));
		this->solu[i].pcl = new int[nC];
		memset(this->solu[i].pcl, 0, nC*sizeof(int));
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
	time_t end;
	time(&end);
	loopTime = difftime(end, start);

	float maxTemperature(FLT_MIN);
	for(unsigned int i=0; i<2*NofE; i++)
	{
		if(1.0f-costs[i] > maxTemperature)
		{
			maxTemperature = 1.0f-costs[i];
		}
	}
	//float maxTemperature = 1.0;
	T = maxTemperature/2.0;	
	min_threshold = T/5.0;

	float basis = min_threshold/T;
	float expo = 1.0f/((float)nOfCams*maxIteration);
	alpha = powf(basis, expo);
	assert(alpha < 1.0f);

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

bool SimulatedAnnealing::isCompleteConfigurationValid(int* pclIndex)
{
	bool isValid = true;
	for(unsigned int i=0; i<NofE; i++)
	{
		
		for(unsigned int cam=0; cam<nC; cam++)
		{
			int pa_index_m = i*2*nC+0*nC+cam;
			int pa_index_p = i*2*nC+1*nC+cam;

			isValid &= isPCLIndexValid(pclIndex, pa_index_m, pa_index_p,i, pclIndex[pa_index_m]);
			isValid &= isPCLIndexValid(pclIndex, pa_index_m, pa_index_p,i, pclIndex[pa_index_p]);
		}
	}
	return isValid;
}

void SimulatedAnnealing::initializeFirstRun(int* pclIndex, int* angleIndex)
{	
	
	
	for(unsigned int i=0; i<NofE; i++)
	{
		minEnergy_t1[i] = 1.0f;
		bool isValidConfiguration = false;
		do{
			for(unsigned int cam=0; cam<nC; cam++)
			{
				int pa_index_m = i*2*nC+0*nC+cam;
				int pa_index_p = i*2*nC+1*nC+cam;



				int p_index =  rand() % Npcl;
				int a_index =	aG->generateRandomAngle();
				pclIndex[pa_index_m] = p_index;
				angleIndex[pa_index_m] = a_index; 
				pclIndex[pa_index_p] = p_index;
				angleIndex[pa_index_p] = a_index;
				pclIndex_t1[pa_index_m] = p_index;
				angleIndex_t1[pa_index_m] = a_index; 
				pclIndex_t1[pa_index_p] = p_index;
				angleIndex_t1[pa_index_p] = a_index;

			}


			bool isValid = true;
			for(unsigned int cam=0; cam<nC; cam++)
			{
				int pa_index_m = i*2*nC+0*nC+cam;
				int pa_index_p = i*2*nC+1*nC+cam;

				isValid &= isPCLIndexValid(pclIndex, pa_index_m, pa_index_p,i, pclIndex[pa_index_m]);
				isValid &= isPCLIndexValid(pclIndex, pa_index_m, pa_index_p,i, pclIndex[pa_index_p]);
			}
			isValidConfiguration = isValid;
			//printf("\n\n\n\n\n");
		}while(!isValidConfiguration);

	}
	time(&start);
	//printDistanceMatrix();

}

void SimulatedAnnealing::printDistanceMatrix()
{
	//float d, xd, yd, zd;
	//for(unsigned int i=0; i<NofE; i++)
	//{

	//	for(unsigned int cam1=0; cam1<nC; cam1++)
	//	{
	//		int pa_index_m1 = i*2*nC+0*nC+cam1;
	//		int pa_index_p1 = i*2*nC+1*nC+cam1;

	//		for(unsigned int i=0; i<NofE; i++)
	//		{

	//			for(unsigned int cam2=0; cam2<nC; cam2++)
	//			{
	//				int pa_index_m2 = i*2*nC+0*nC+cam2;
	//				int pa_index_p2 = i*2*nC+1*nC+cam2;

	//				xd = robot->x[pa_index_m1] - robot->x[pa_index_m2];
	//				yd = robot->y[pa_index_m1] - robot->y[pa_index_m2];
	//				zd = robot->z[pa_index_m1] - robot->z[pa_index_m2];
	//				d = sqrt(powf(xd,2.0f)+powf(yd,2.0f)+powf(zd,2.0f));
	//				printf("%.2f\t", d);

	//				xd = robot->x[pa_index_m1] - robot->x[pa_index_p2];
	//				yd = robot->y[pa_index_m1] - robot->y[pa_index_p2];
	//				zd = robot->z[pa_index_m1] - robot->z[pa_index_p2];
	//				d = sqrt(powf(xd,2.0f)+powf(yd,2.0f)+powf(zd,2.0f));
	//				printf("%.2f\t", d);

	//				xd = robot->x[pa_index_p1] - robot->x[pa_index_m2];
	//				yd = robot->y[pa_index_p1] - robot->y[pa_index_m2];
	//				zd = robot->z[pa_index_p1] - robot->z[pa_index_m2];
	//				d = sqrt(powf(xd,2.0f)+powf(yd,2.0f)+powf(zd,2.0f));
	//				printf("%.2f\t", d);

	//				xd = robot->x[pa_index_p1] - robot->x[pa_index_p2];
	//				yd = robot->y[pa_index_p1] - robot->y[pa_index_p2];
	//				zd = robot->z[pa_index_p1] - robot->z[pa_index_p2];
	//				d = sqrt(powf(xd,2.0f)+powf(yd,2.0f)+powf(zd,2.0f));
	//				printf("%.2f\t", d);
	//			}
	//		}
	//		printf("\n", d);
	//	}		
	//}
}

int SimulatedAnnealing::createPCLIndexInRange(int* pcl, int offset_m_in, int offset_p_in, int index)
{
	int i2;
	float xd, yd, zd, d;
	int offset_m,offset_p;
	bool isValid = true;
	
	do
	{
	
		i2 = rand()%Npcl;
		isValid = true;
		for(int cam=0; cam<nOfCams; cam++)
		{
			offset_m = index*2*nC+0*nC+cam;
			offset_p = index*2*nC+1*nC+cam;

			if(offset_m != offset_m_in)
			{
			
				//check if conditions are fullfilled
				xd = robot->x[pcl[offset_m]] - robot->x[i2];
				yd = robot->y[pcl[offset_m]] - robot->y[i2];
				zd = robot->z[pcl[offset_m]] - robot->z[i2];
				d = sqrt(powf(xd,2.0f)+powf(yd,2.0f)+powf(zd,2.0f));
				//printf("%.5f\n", d);
				isValid &= (MIN_DIST_BETWEEN_CAMERA < d);
			}
			

			if(offset_p == offset_p_in)
			{
				//check if conditions are fullfilled
				xd = robot->x[pcl[offset_p]] - robot->x[i2];
				yd = robot->y[pcl[offset_p]] - robot->y[i2];
				zd = robot->z[pcl[offset_p]] - robot->z[i2];
				d = sqrt(powf(xd,2.0f)+powf(yd,2.0f)+powf(zd,2.0f));
				//printf("%.5f\n", d);
				isValid &= (MIN_DIST_BETWEEN_CAMERA < d);
			}
		}
		
	}while(!isValid);
	return i2;
}
bool SimulatedAnnealing::isPCLIndexValid(int* pcl, int offset_m_in, int offset_p_in, int index, int i2)
{
	
	float xd, yd, zd, d;
	int offset_m,offset_p;
	bool isValid = true;
	


	for(int cam=0; cam<nOfCams; cam++)
	{
		offset_m = index*2*nC+0*nC+cam;
		offset_p = index*2*nC+1*nC+cam;

		if(offset_m != offset_m_in)
		{
			//check if conditions are fullfilled
			xd = robot->x[pcl[offset_m]] - robot->x[i2];
			yd = robot->y[pcl[offset_m]] - robot->y[i2];
			zd = robot->z[pcl[offset_m]] - robot->z[i2];
			d = sqrt(powf(xd,2.0f)+powf(yd,2.0f)+powf(zd,2.0f));
			//printf("%.5f\n", d);
			isValid &= (MIN_DIST_BETWEEN_CAMERA < d);
		}
			
		if(offset_p != offset_p_in)
		{
			//check if conditions are fullfilled
			xd = robot->x[pcl[offset_p]] - robot->x[i2];
			yd = robot->y[pcl[offset_p]] - robot->y[i2];
			zd = robot->z[pcl[offset_p]] - robot->z[i2];
			d = sqrt(powf(xd,2.0f)+powf(yd,2.0f)+powf(zd,2.0f));
			//printf("%.5f\n", d);
			isValid &= (MIN_DIST_BETWEEN_CAMERA < d);
		}

		//if(!isValid)
		//{
		//	printf("error distance\n");
		//	IO::waitForEnter();
		//}
	}
	return isValid;	

}


int SimulatedAnnealing::createAngleIndexInRange(int a_i)
{
	//int ri, pi, yi;
	//convertAItoRPY(a_i, sr, &ri, &pi, &yi);


	//int di;
	//di = (rand()%angleIndexRandomRange_roll) - angleIndexRandomRange_roll/2;
	//ri = (ri+di+sr->nRoll)%sr->nRoll;

	//di = (rand()%angleIndexRandomRange_pitch) - angleIndexRandomRange_pitch/2;
	//pi = (pi+di+sr->nPitch)%sr->nPitch;

	//di = (rand()%angleIndexRandomRange_yaw) - angleIndexRandomRange_yaw/2;
	//yi = (yi+di+sr->nYaw)%sr->nYaw;


	int ret;
	//convertRPYtoAI(ri, pi, yi, sr, &ret);
	ret = aG->generateRandomAngle();
	return ret;
}

void SimulatedAnnealing::chooseRandomConfiguration(int* pclIndex, int* angleIndex, int index)
{

	int offset_m,offset_p;
	for(int cam=0; cam<nOfCams; cam++)
	{
		offset_m = index*2*nC+0*nC+cam;
		offset_p = index*2*nC+1*nC+cam;

		int p_index = createPCLIndexInRange( pclIndex, offset_m,offset_p, index);
		int a_index = createAngleIndexInRange(angleIndex[offset_m]);

		pclIndex[offset_m] = p_index;
		angleIndex[offset_m] = a_index; 

		pclIndex[offset_p] = p_index;
		angleIndex[offset_p] = a_index;

	}


	
}

void SimulatedAnnealing::findGlobalMinimum(void)
{
	for(unsigned int i=0; i< NofE; i++)
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
}

void SimulatedAnnealing::printCurrentStatus(void)
{
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

	system("cls");	
	printf("ST\tminE\tcosts\t\tprob\n");
	printf("%s\t%.5f\t%.5f\t%.10f\n\n\n",   state.c_str(),minEnergy[0], solu[0].costs, solu[0].currProp);

	printf("cam\tpcl\troll\tpitch\tyaw\tdim\n");
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
	printf("global min costs at: %.10f at %i:%i at T:%.3f  pos:%i  cams:%i\n\n\n", globalMin.costs, globalMin.pcl[0], globalMin.angle[0], T, ite, nOfCams);

	
	
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

	float cm = 1.0-costs[2*i+0];
	float cp = 1.0-costs[2*i+1];

	int dim = cDim[i]%6;
	int cam = cDim[i]/6;
	int pa_index_m = i*2*nC+0*nC+cam;
	int pa_index_p = i*2*nC+1*nC+cam;
	int p_o_m;




	if(cm < cp)
	{	//change to minus
		//copying the complete solution to minus
		localMinE = cm;

		p_i = pclIndex[pa_index_m];
		a_i = angleIndex[pa_index_m];

		memcpy(angleIndex+i*2*nC+1*nC, angleIndex+i*2*nC+0*nC, nC*sizeof(int));
		memcpy(pclIndex+i*2*nC+1*nC, pclIndex+i*2*nC+0*nC, nC*sizeof(int));
		p_o_m = 0;
	}else
	{	//change to plus
		//copying the complete solution to plus
		localMinE = cp;

		p_i = pclIndex[pa_index_p];
		a_i = angleIndex[pa_index_p];	

		memcpy(angleIndex+i*2*nC+0*nC, angleIndex+i*2*nC+1*nC, nC*sizeof(int));
		memcpy(pclIndex+i*2*nC+0*nC, pclIndex+i*2*nC+1*nC, nC*sizeof(int));

		p_o_m = 1;
	}




	int offset = i*2*nC+p_o_m*nC;
	memcpy(solu[i].angle, angleIndex+offset, nC*sizeof(int));
	memcpy(solu[i].pcl, pclIndex+offset, nC*sizeof(int));
	solu[i].costs = localMinE;

	if(localMinE < minEnergy_t1[i])
	{
		//there has been actual improvement
		//solution is stored into history
		memcpy(pclIndex_t1+i*2*nC, pclIndex, nOfCams*sizeof(int));
		memcpy(angleIndex_t1+i*2*nC, angleIndex, nOfCams*sizeof(int));
		memcpy(minEnergy_t1+i, minEnergy+i, sizeof(float));
	}else
	{
		//there has no actual improvement 
		//the state before mutation is restored
		memcpy(pclIndex+i*2*nC, pclIndex_t1, nOfCams*sizeof(int));
		memcpy(angleIndex+i*2*nC, angleIndex_t1, nOfCams*sizeof(int));
		memcpy(minEnergy+i, minEnergy_t1+i, sizeof(float));
	}

	//if(costs[2*i+0] < costs[2*i+1])
	//{	//change to minus
	//	localMinE = costs[2*i+0];
	//	p_i = pclIndex[2*i+0];
	//	a_i = angleIndex[2*i+0];
	//}else
	//{	//change to plus
	//	localMinE = costs[2*i+1];
	//	p_i = pclIndex[2*i+1];
	//	a_i = angleIndex[2*i+1];				
	//}

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

	
	cDim[i] = (cDim[i]+1)%DOF;
	dim = cDim[i]%6;
	cam = cDim[i]/6;
	pa_index_m = i*2*nC+0*nC+cam;
	pa_index_p = i*2*nC+1*nC+cam;
	
	
	//int ri = a_i/(sr->nPitch*sr->nYaw);
	//int pi = (a_i - ri*sr->nPitch*sr->nYaw)/sr->nYaw;
	//int yi = a_i-ri*sr->nPitch*sr->nYaw-pi*sr->nYaw;
	int ri, pi, yi;
	convertAItoRPY(a_i, sr, &ri, &pi, &yi);

	int roll = ri, pitch = pi, yaw = yi;
	int am, ap;


	int index_m =0, index_p=0;
	switch(dim){
	case 0:		
	case 1:
	case 2:
		//x,y,z
		index_m = nn_indices[p_i*6+dim*2+0];
		index_p = nn_indices[p_i*6+dim*2+1];

		
		if(isPCLIndexValid(pclIndex, pa_index_m, pa_index_p,i, index_m))
			pclIndex[pa_index_m] = index_m;

		if(isPCLIndexValid(pclIndex, pa_index_m, pa_index_p,i, index_p))
			pclIndex[pa_index_p] = index_p;

/*		pclIndex[pa_index_m] = nn_indices[p_i*6+dim*2+0];
		pclIndex[pa_index_p] = nn_indices[p_i*6+dim*2+1];*/	
		break;
	case 3:
		//roll
		am = (ri+sr->nRoll-1)%sr->nRoll;
		ap = (ri+sr->nRoll+1)%sr->nRoll;
		convertRPYtoAI(am, yi, yi, sr, &(angleIndex[pa_index_m]));
		convertRPYtoAI(ap, yi, yi, sr, &(angleIndex[pa_index_p]));
		break;
	case 4:
		//pitch
		am = (pi+sr->nPitch-1)%sr->nPitch;
		ap = (pi+sr->nPitch+1)%sr->nPitch;
		convertRPYtoAI(ri, am, yi, sr, &(angleIndex[pa_index_m]));
		convertRPYtoAI(ri, ap, yi, sr, &(angleIndex[pa_index_p]));
		break;
	case 5:
		//yaw
		am = (yi+sr->nYaw-1)%sr->nYaw;
		ap = (yi+sr->nYaw+1)%sr->nYaw;
		convertRPYtoAI(ri, pi, am, sr, &(angleIndex[pa_index_m]));
		convertRPYtoAI(ri, pi, ap, sr, &(angleIndex[pa_index_p]));
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

	outbin.write((char*)&nC, sizeof(int));
	outbin.write((char*)buf2, nC*NUMELEM_H*sizeof(float));
	outbin.write((char*)&globalMin.costs, sizeof(float));
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

bool SimulatedAnnealing::iterate(int* pclIndex, int* angleIndex, float* costs, float* d, int* weights)
{
	if(firstIteration)
	{
		initializeFirstRun(pclIndex, angleIndex);
		firstIteration = false;
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
				setNewVector(i);
				//printf("random jump...\n");
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

	//if(T >min_threshold)
	if(ite<maxIteration)
	{
		T *= alpha;
		return true;
	}else
	{
		return false;
	}
}