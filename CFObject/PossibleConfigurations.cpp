#include "PossibleConfigurations.h"
#include <float.h>
#include <stdio.h>
#include <fstream>
#include <iostream>

PossibleConfigurations::PossibleConfigurations():size(0), hs(NULL), prob(NULL), maxProb(0)
{
}

void PossibleConfigurations::saveState(const char* fN)
{
	float* RBuffer = new float[size*4];
	float* xBuffer = new float[size];
	float* yBuffer = new float[size];

	for(int i=0; i<size; i++)
	{
		for(int j=0; j<4; j++)
		{
			RBuffer[4*i+j] = hs[i].getR()[j];
		}
		xBuffer[i] = hs[i].getv()[0];
		yBuffer[i] = hs[i].getv()[1];
	}

	std::ofstream outbin(fN, std::ofstream::binary );
	if (!outbin) std::cerr << "error";

	outbin.write((char*)&size,sizeof(int));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)RBuffer,size*4*sizeof(float));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)xBuffer,size*sizeof(float));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)yBuffer,size*sizeof(float));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)prob,size*sizeof(double));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)&maxProb,sizeof(double));
	if (!outbin) std::cerr << "error";

	delete RBuffer;
	delete xBuffer;
	delete yBuffer;

	outbin.close();
}

PossibleConfigurations::PossibleConfigurations(CFIO::SAMPLE_HUMAN_POSITIONS* pcl)
{
	size = pcl->n;
	hs = new Transformation2D[size];
	prob = new double[size];
	for(int i=0; i<size; i++)
	{
		hs[i].setR(0, pcl->R[i*9+0]);
		hs[i].setR(1, pcl->R[i*9+1]);
		hs[i].setR(2, pcl->R[i*9+3]);
		hs[i].setR(3, pcl->R[i*9+4]);
		hs[i].setv(0, pcl->x[i]);
		hs[i].setv(1, pcl->y[i]);
		prob[i] = 0;
	}
	maxProb = 0;	
}
	
PossibleConfigurations::PossibleConfigurations(PossibleConfigurations& inst)
{
	this->size = inst.size;
	this->hs = new Transformation2D[size];
	this->prob = new double[size];
	for(int i=0; i<size; i++)
	{
		this->hs[i] = inst.hs[i];
	}
	maxProb = 0;
}
void PossibleConfigurations::operator=(PossibleConfigurations& inst )
{
	if(hs != NULL)
		delete hs;
	if(prob != NULL)
		delete prob;

	this->size = inst.size;
	this->hs = new Transformation2D[size];
	this->prob = new double[size];
	for(int i=0; i<size; i++)
	{
		this->hs[i] = inst.hs[i];
	}
	maxProb = 0;

}

PossibleConfigurations::PossibleConfigurations(Transformation2D* hs, int size)
{
	this->size = size;
	this->hs = new Transformation2D[size];
	this->prob = new double[size];
	for(int i=0; i<size; i++)
	{
		this->hs[i] = hs[i];
	}
	maxProb = 0;
}

void PossibleConfigurations::findBestFit(Cluster cluster)
{
	//first convert the Cluster to the center
	maxProb = 0;
	for(int i=0; i<size; i++)
	{
		prob[i] = 0;
	}

	if(cluster.getMaxSize() >0)
	{
		cluster.calculateCentroid();
		Transformation2D transformation(0.0f, -cluster.getCentroid()[0], -cluster.getCentroid()[1]);
		cluster.transform(transformation);

		Cluster temp;
		double p;
		for(int i=0; i<size; i++)
		{
			temp = cluster;
			temp.transform(hs[i]);
			temp.calculate();
			p = temp.getHumanProb();
			prob[i] = p;
			if(p > maxProb)
			{
				maxProb = p;
				bestTransformation = hs[i];
			}
		}
	}
}


PossibleConfigurations::~PossibleConfigurations(void)
{
	delete[] this->hs;
	delete this->prob;
}
