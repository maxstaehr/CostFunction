#include "Search.h"
#include <float.h>
#include <iostream>
#include <fstream>

Search::Search(SampleCameraConfiguration* config, int n, bool lo):currentCosts(1.0),state(STATE::HC)
{
	this->sampleConfig = new SampleCameraConfiguration[n];
	this->n = n;
	this->log = lo;
	currentDim = new EVAL_DIM[n];
	for(int i=0; i<n; i++)
	{
		this->sampleConfig[i] = config[i];
		currentDim[i] = EVAL_DIM::X;
	}

		


	currentTrans = new HomogeneTransformation[n];
	nextEvalMinus = new HomogeneTransformation[n];
	nextEvalPlus = new HomogeneTransformation[n];

}

void Search::logging(double m, double p)
{
	struct LOG_DATA log;
	log.current = new HomogeneTransformation[n];
	log.plus = new HomogeneTransformation[n];
	log.minus = new HomogeneTransformation[n];
	log.dim = new int[n];
	
	log.state = 0;
	log.costm = m;
	log.costp = p;
	log.cost = currentCosts;

	for(int i=0; i<n; i++)
	{
		log.current[i] = currentTrans[i];
		log.minus[i] = nextEvalMinus[i];
		log.plus[i] = nextEvalPlus[i];
		log.dim[i] = currentDim[i];
	}
	dataLog.push_back(log);	
}

void Search::saveLog(const char* fileName)
{
	std::ofstream outbin(fileName, std::ofstream::binary );
	if (!outbin) std::cerr << "error";

	int value = dataLog.size();
	outbin.write((char*)&value,sizeof(int));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)&n,sizeof(int));
	if (!outbin) std::cerr << "error";

	struct LOG_DATA log;
	for(int j=0; j<value; j++)
	{
		log = dataLog[j];
		
		outbin.write((char*)&log.state,sizeof(int));
		if (!outbin) std::cerr << "error";

		outbin.write((char*)&log.costm,sizeof(double));
		if (!outbin) std::cerr << "error";

		outbin.write((char*)&log.costp,sizeof(double));
		if (!outbin) std::cerr << "error";

		outbin.write((char*)&log.cost,sizeof(double));
		if (!outbin) std::cerr << "error";

		for(int i=0; i<n; i++)
		{
			for(int k=0; k<NELEM_H; k++)
			{
				outbin.write((char*)log.current[k].getH(),sizeof(float));
				if (!outbin) std::cerr << "error";
			}
		}

		for(int i=0; i<n; i++)
		{
			for(int k=0; k<NELEM_H; k++)
			{
				outbin.write((char*)log.minus[k].getH(),sizeof(float));
				if (!outbin) std::cerr << "error";
			}
		}

		for(int i=0; i<n; i++)
		{
			for(int k=0; k<NELEM_H; k++)
			{
				outbin.write((char*)log.plus[k].getH(),sizeof(float));
				if (!outbin) std::cerr << "error";
			}
		}

		for(int i=0; i<n; i++)
		{
			outbin.write((char*)log.dim[i],sizeof(int));
			if (!outbin) std::cerr << "error";

		}
	}	
	outbin.close();	
}


Search::~Search(void)
{
	delete [] sampleConfig;
	delete [] currentTrans;
	delete [] nextEvalMinus;
	delete [] nextEvalPlus;
	delete currentDim;
}
