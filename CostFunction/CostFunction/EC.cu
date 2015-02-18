#include "EC.h"
#include <set>
#include <float.h>
#include <cstdio>
#include <string.h>
#include <math.h>
#include <time.h>
#include <fstream>
#include <iostream> 
#include <assert.h>
#include <sstream> 
#include <string> 
#include "global.h"


EC::EC(float* dx, float* dy, float* dz, int* cis, bool* hasProcessed, int size)
{
	this->dx = dx;
	this->dy = dy;
	this->dz = dz;
	this->cis = cis;	
	this->size = size;	
	this->hasProcessed = hasProcessed;
}


bool EC::hasAllBeenProcessed(std::set<int>* set)
{
	std::set<int>::iterator it;
	bool ret = true;
	for (it=set->begin(); it!=set->end(); ++it)
		ret &= hasProcessed[*it];
	return ret;
}

float EC::calcDistance(int i1, int i2)
{
	return powf(dx[i1]-dx[i2], 2.0f)+powf(dy[i1]-dy[i2], 2.0f)+powf(dz[i1]-dz[i2], 2.0f);
}

void EC::process(std::set<int>* Q, std::set<int>* P)
{
	std::set<int>::iterator it_q, it_p;	
	int index;
	for(it_q=Q->begin(); it_q!=Q->end(); ++it_q)
	{
		if(hasProcessed[*it_q] || isnan(this->dx[*it_q]))
			continue;

		for(it_p=P->begin(); it_p!=P->end(); ++it_p)
		{
			//version 1
			//index = (*it_q)*this->size+(*it_p);
			//if(!isnan(this->dm[index]) && this->dm[index] < EUCLIDEAN_THRESHOLD && !hasProcessed[*it_p])
			//{
			//	Q->insert(*it_p);
			//}

			if(!isnan(this->dx[*it_p]) && !hasProcessed[*it_p] && calcDistance(*it_q, *it_p) < EUCLIDEAN_THRESHOLD )
			{
				Q->insert(*it_p);
			}
		}
		hasProcessed[*it_q] = true;
		
	}
	
}

int EC::findFirstNotProcessedIndice(std::set<int>* set)
{
	int ret = -1;
	std::set<int>::iterator it;
	for (it=set->begin(); it!=set->end(); ++it)
	{
		if(!hasProcessed[*it])
		{
			ret = *it;
			break;
		}
	}
	return ret;
		
}

void EC::cluster(void)
{
	std::set<int> P;
	std::set<int> *Q = new std::set<int>;
	std::set<std::set<int>*> C;
	std::set<int>* Ci = new std::set<int>();
	for(int i=0; i<this->size; i++)
	{
		if(!isnan(this->dx[i]))
		{
			P.insert(i);
		}
		hasProcessed[i] = false;
	}

	//actual clustering
	while(!hasAllBeenProcessed(&P))
	{
		//inserting new element
		Q->insert(findFirstNotProcessedIndice(&P));
		while(!hasAllBeenProcessed(Q))		
			process(Q,&P);

		//adding Q to to C
		C.insert(Q);
		Q = new std::set<int>();

	}


	//printing results
	//std::set<std::set<int>*>::iterator it;
	//int i =0;
	//for(it=C.begin(); it!=C.end(); ++it, i++)
	//{
	//	//printf("cluster: %d\t size: %d\n", i, (*it)->size());
	//	delete *it;
	//}

	memset(this->cis, -1, this->size*sizeof(int));


	//finding maximum cluster indice
	std::set<std::set<int>*>::iterator it;
	std::set<int>::iterator it_p;
	int i =0;
	this->maxIndex = 0;
	this->maxSize = 0;
	for(it=C.begin(),i =0; it!=C.end(); ++it, i++)
	{
		if((*it)->size() > maxSize)
		{
			this->maxSize = (*it)->size();
			this->maxIndex = i;

		}
		for(it_p=(*it)->begin(); it_p!=(*it)->end(); ++it_p)
		{
			this->cis[*it_p] = i;
		}		
		//printf("cluster: %d\t size: %d\n", i, (*it)->size());
		delete *it;
				
	}


}

EC::~EC(void)
{
	
}
