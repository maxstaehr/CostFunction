#include "EC.h"

#include <iostream>
#include <assert.h>
#include <cmath>
#include <limits>

#define EUCLIDEAN_THRESHOLD (1.0f)
EC::EC(int maxSize):x(NULL), y(NULL), z(NULL), hasProcessed(NULL)
{
	assert(maxSize > 0);
	this->maxSize = maxSize;
	this->x = new float[maxSize];
	this->y = new float[maxSize];
	this->z = new float[maxSize];
	this->hasProcessed = new bool[maxSize];
	reset();
	
}
EC::~EC(void)
{
	delete x;
	delete y;
	delete z;
	delete hasProcessed;
	P.clear();
	if(C.size() > 0)
	{
		std::set<std::set<int>*>::iterator it;
		for(it=C.begin(); it!=C.end(); ++it)
		{
			delete *it;
		}
			
	}
	C.clear();
	clu.clear();

}



void EC::reset()
{
	for(int i=0; i< maxSize; i++)
	{		
		x[i] = std::numeric_limits<float>::quiet_NaN();
		y[i] = std::numeric_limits<float>::quiet_NaN();
		z[i] = std::numeric_limits<float>::quiet_NaN();
		hasProcessed[i] = false;
	}
	P.clear();
	if(C.size() > 0)
	{
		std::set<std::set<int>*>::iterator it;
		for(it=C.begin(); it!=C.end(); ++it)
		{
			delete *it;
		}
			
	}
	C.clear();
	clu.clear();
}
Cluster EC::getLargestCluster()
{
	int maxSize = 0;
	int index = 0;
	for(int i=0; i<clu.size(); i++)
	{
		if(clu[i].getMaxSize()>maxSize)
		{
			index = i;
		}
	}
	return clu[index];
}

void EC::addDepthData(Camera& cam)
{
	int j=P.size();
	for(int i=0; i<cam.getSize(); i++)
	{
		
		if(cam.getdx()[i] < FLT_MAX && cam.getdy()[i]<FLT_MAX && cam.getdz()[i]<FLT_MAX)
		{
			x[j] = cam.getdx()[i];
			y[j] = cam.getdy()[i];
			z[j] = cam.getdz()[i];
			P.insert(j);
			j++;
		}
		hasProcessed[i] = false;
	}

}
float EC::calcDistance(int i1, int i2)
{
	return powf(x[i1]-x[i2], 2.0f)+powf(y[i1]-y[i2], 2.0f)+powf(z[i1]-z[i2], 2.0f);
}

void EC::process(std::set<int>* Q, std::set<int>* P)
{
	std::set<int>::iterator it_q, it_p;	
	int index;
	for(it_q=Q->begin(); it_q!=Q->end(); ++it_q)
	{
		if(hasProcessed[*it_q])
			continue;

		for(it_p=P->begin(); it_p!=P->end(); ++it_p)
		{

			if(!hasProcessed[*it_p] && calcDistance(*it_q, *it_p) < EUCLIDEAN_THRESHOLD )
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
bool EC::hasAllBeenProcessed(std::set<int>* set)
{
	std::set<int>::iterator it;
	bool ret = true;
	for (it=set->begin(); it!=set->end(); ++it)
		ret &= hasProcessed[*it];
	return ret;
}


void EC::cluster()
{
	std::set<int>* Q;	
	while(!hasAllBeenProcessed(&P))
	{
		//inserting new element
		Q = new std::set<int>();
		Q->insert(findFirstNotProcessedIndice(&P));
		while(!hasAllBeenProcessed(Q))		
			process(Q,&P);

		//adding Q to to C
		C.insert(Q);
		

	}


	//printing results
	//std::set<std::set<int>*>::iterator it;
	//int i =0;
	//for(it=C.begin(); it!=C.end(); ++it, i++)
	//{
	//	//printf("cluster: %d\t size: %d\n", i, (*it)->size());
	//	delete *it;
	//}




	//finding maximum cluster indice
	std::set<std::set<int>*>::iterator it;
	std::set<int>::iterator it_p;
	int i =0;
	int maxIndex = 0;
	int maxSize = 0;
	for(it=C.begin(),i =0; it!=C.end(); ++it, i++)
	{
		if((*it)->size() > maxSize)
		{
			maxSize = (*it)->size();
			maxIndex = i;

		}

		Cluster c((*it)->size());
		int index = 0;
		for(it_p=(*it)->begin(), index =0; it_p!=(*it)->end(); ++it_p, index++)
		{
			c.setX(index, x[*it_p]);
			c.setY(index, y[*it_p]);
			c.setZ(index, z[*it_p]);						
		}
		clu.push_back(c);
		

/*		for(it_p=(*it)->begin(); it_p!=(*it)->end(); ++it_p)
		{
			this->cis[*it_p] = i;
		}*/		
		//printf("cluster: %d\t size: %d\n", i, (*it)->size());
		//delete *it;
				
	}


}


