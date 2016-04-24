#include "Link.h"

#include <iostream>
Link::Link(void):nPCL(0), pcl(NULL), nBB(0), bbl(NULL)
{
}

Link::Link(Link& inst)
{
	H = inst.H;
	nPCL = inst.nPCL;
	nBB = inst.nBB;


	pcl = new PCL[nPCL];
	for(int i=0; i<nPCL; i++)
	{
		pcl[i] = inst.pcl[i];
	}
	bbl = new BoundingBox[nBB];
	for(int i=0; i<nBB; i++)
	{
		bbl[i] = inst.bbl[i];
	}
}
void Link::operator=(Link& inst )
{
	H = inst.H;
	nPCL = inst.nPCL;
	nBB = inst.nBB;

	if(pcl != NULL)
	{
		delete [] pcl;
		delete [] bbl;
	}
	pcl = new PCL[nPCL];
	for(int i=0; i<nPCL; i++)
	{
		pcl[i] = inst.pcl[i];
	}
	bbl = new BoundingBox[nBB];
	for(int i=0; i<nBB; i++)
	{
		bbl[i] = inst.bbl[i];
	}
}


Link::~Link(void)
{
	if(pcl != NULL)	
		delete[] this->pcl;
	if(bbl != NULL)
		delete[] this->bbl;	
	
}

void Link::setH(HomogeneTransformation H) 
{
	this->H = H;
	for(int i=0; i<nPCL; i++)
	{
		pcl[i].transform(H);
	}
	
	for(int i=0; i<nBB; i++)
	{
		bbl[i].transform(H);
	}
}

void Link::addBB(BoundingBox inst)
{
	nBB++;
	if(this->bbl == NULL)
	{		
		this->bbl = new BoundingBox[1];
		this->bbl[0] = inst;		

	}else
	{

		BoundingBox* tmp = new BoundingBox[nBB];
		for(int i=0; i<nBB-1; i++)
		{
			tmp[i] = this->bbl[i];			
		}

		if (this->bbl != NULL)
			delete[] (this->bbl);

		tmp[nBB-1] = inst;
		this->bbl = tmp;
	}
}

void Link::addPCL(PCL inst)
{
	nPCL++;
	if(this->pcl == NULL)
	{		
		this->pcl = new PCL[1];
		this->pcl[0] = inst;		

	}else
	{

		PCL* tmp = new PCL[nPCL];
		for(int i=0; i<nPCL-1; i++)
		{
			tmp[i] = this->pcl[i];			
		}

		if (this->pcl != NULL)
			delete[] (this->pcl);

		tmp[nPCL-1] = inst;
		this->pcl = tmp;
	}
}


