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
  


SimulatedAnnealing::SimulatedAnnealing(int NumerOfParrlelCalculation, double T, double alpha, int Npcl, int Nangle):ite(0)
{

	NofE = (int)NumerOfParrlelCalculation/2;
	minEnergy = new double [NofE];
	noChange = new bool[NofE*6];
	cDim = new unsigned char[NofE];	

	this->Nangle = Nangle;
	this->Npcl = Npcl;
	this->solu = new struct SOLUTION[NofE];
	this->curStates = new enum STATE[NofE];
	this->globalMin.costs = DBL_MAX;




	

	for(unsigned int i=0; i<NofE; i++)
	{
		minEnergy[i] = DBL_MAX;
		cDim[i] = 0;
		for(unsigned int j=0; j<6; j++)
		{
			noChange[i*6+j] = false;
		}
		curStates[i] = STATE::HC;
		solu[i].currProp = 0.0;
	}

	this->T = T;
	this->alpha = alpha;
	
    time_t t;
    time(&t);
    srand((unsigned int)t);              /* Zufallsgenerator initialisieren */
}


SimulatedAnnealing::~SimulatedAnnealing(void)
{
	delete minEnergy;
}

void SimulatedAnnealing::initializeFirstRun(int* pclIndex, int* angleIndex)
{
	for(unsigned int i=0; i<NofE*2; i++)
	{
		pclIndex[i] = rand() % Npcl;
		angleIndex[i] = rand() % Nangle;
	}
}

void SimulatedAnnealing::chooseRandomConfiguration(int* pclIndex, int* angleIndex)
{
	for(unsigned int i=0; i<2; i++)
	{
		pclIndex[i] = rand() % Npcl;
		angleIndex[i] = rand() % Nangle;
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
	for(unsigned int i=0; i< 100; i++)
	{
		
		if(curStates[i] == STATE::HC)
		{
			printf("%i\tHC\t%.5lf\t%.5f\t%.10f\t%i\t%i\n", i, minEnergy[i], solu[i].costs, solu[i].currProp, solu[i].pcl, solu[i].angle);
		}else if(curStates[i] == STATE::NS)
		{
			printf("%i\tNS\t%.5lf\t%.5f\t%.10f\t%i\t%i\n", i, minEnergy[i], solu[i].costs, solu[i].currProp, solu[i].pcl, solu[i].angle);
		}
		
	}
	printf("\n\nglobal min costs at: %.5f at %i:%i at T:%.3f  pos:%i l:\n\n\n", globalMin.costs, globalMin.pcl, globalMin.angle, T, ite);

	
	
}



void SimulatedAnnealing::resetHasChanged(int i)
{
	for(unsigned int j=0; j<6; j++)
	{
		noChange[i*6+j] = false;
	}
}

bool SimulatedAnnealing::hasHillClimbingFinished(int i)
{
	bool hasFinished = true;
	for(unsigned int j=0; j<6; j++)
	{
		hasFinished &= noChange[i*6+j];
	}
	return hasFinished; 
}

double SimulatedAnnealing::iterateSingle(const int* const nn_indices, int* pclIndex, int* angleIndex, double* costs, int i)
{
	int p_i, a_i;
	double localMinE = 0.0;


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


	if(localMinE < minEnergy[i])
	{	//has changed
	//	noChange[6*i+cDim[i]] = false;
		resetHasChanged(i);	
	}else{
		noChange[6*i+cDim[i]] = true;
	}


	//found best position setting next position
	solu[i].angle = a_i;
	solu[i].pcl = p_i;
	solu[i].costs = localMinE;
	cDim[i] = (cDim[i]+1)%6;
	
	

	int ri = a_i/N_OF_A_SQ;
	int pi = (a_i - ri*N_OF_A_SQ)/N_OF_A;
	int yi = a_i-ri*N_OF_A_SQ-pi*N_OF_A;
	int roll = ri, pitch = pi, yaw = yi;
	int am, ap;

	switch(cDim[i]){
	case 0:		
	case 1:
	case 2:
		//x,y,z
		pclIndex[2*i+0] = nn_indices[p_i*6+cDim[i]*2+0];
		pclIndex[2*i+1] = nn_indices[p_i*6+cDim[i]*2+1];	
		break;
	case 3:
		//roll
		am = (ri+N_OF_A-1)%N_OF_A;
		ap = (ri+N_OF_A+1)%N_OF_A;
		angleIndex[2*i+0] = am * N_OF_A_SQ + pitch * N_OF_A + yaw;
		angleIndex[2*i+0] = ap * N_OF_A_SQ + pitch * N_OF_A + yaw;
		break;
	case 4:
		//pitch
		am = (pi+N_OF_A-1)%N_OF_A;
		ap = (pi+N_OF_A+1)%N_OF_A;
		angleIndex[2*i+0] = roll * N_OF_A_SQ + am * N_OF_A + yaw;
		angleIndex[2*i+0] = roll * N_OF_A_SQ + ap * N_OF_A + yaw;
		break;
	case 5:
		//yaw
		am = (yi+N_OF_A-1)%N_OF_A;
		ap = (yi+N_OF_A+1)%N_OF_A;
		angleIndex[2*i+0] = roll * N_OF_A_SQ + pitch * N_OF_A + am;
		angleIndex[2*i+0] = roll * N_OF_A_SQ + pitch * N_OF_A + ap;
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

bool SimulatedAnnealing::iterate(const int* const nn_indices, int* pclIndex, int* angleIndex, double* costs)
{
	const double e = 2.718281828;
	//iterating through all possible configuration
	int rp, ra;
	double localMinE;

	double* p = new double[NofE];
	recordedSolution.push_back(p);

	for(unsigned int i=0; i<NofE; i++)
	{
		//setting next iteration and find local minimum
		localMinE = iterateSingle(nn_indices, pclIndex, angleIndex, costs, i);

		switch(curStates[i]){
		case STATE::HC:
			//check if is still in HC
			if(hasHillClimbingFinished(i))
			{   //switch to ns
				curStates[i] = STATE::NS;
				setNewVector(i);
				chooseRandomConfiguration(pclIndex+i*2, angleIndex+i*2);			
			}

			//check if current alternation has brought improvement
			if(localMinE < minEnergy[i])
			{
				minEnergy[i] = localMinE;
			}
			break;
		case STATE::NS:
			//try to accept new bet
			double prop = (rand() / (double)RAND_MAX);
			double newprop = pow(e, (minEnergy[i]-localMinE) / T);
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
				chooseRandomConfiguration(pclIndex+i*2, angleIndex+i*2);	
			}
			break;
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