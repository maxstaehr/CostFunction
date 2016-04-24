#include "KinChain.h"

#include <string>
#include <iostream>
#include <fstream>
#include <assert.h>
#include <vector>
KinChain::KinChain(int nPos, int sPP)
{
	this->nPos = nPos;
	this->samplesPerPos = sPP;
	rP = new float[nPos*sPP*NELEM_H*DOF_R];
	eP = new float[nPos*sPP*NELEM_H*DOF_E];
	hP = new float[nPos*sPP*NELEM_H*DOF_H];

	memset(rP, 0, sizeof(float)*nPos*sPP*NELEM_H*DOF_R);
	memset(eP, 0, sizeof(float)*nPos*sPP*NELEM_H*DOF_E);
	memset(hP, 0, sizeof(float)*nPos*sPP*NELEM_H*DOF_H);

}
void KinChain::addHumanPCL(CFIO::LINK* pcl)
{
	addPCL(h, DOF_H,pcl);
}

void KinChain::addRobotPCL(CFIO::LINK* pcl)
{
	addPCL(r, DOF_R,pcl);
}
void KinChain::addEnvPCL(CFIO::LINK* pcl)
{
	addPCL(e, DOF_E,pcl);
}





void KinChain::addPCL(Link* linkArray, int DOF, CFIO::LINK* link)
{

	struct CFIO::PCL* pPCL;
	for(int i=0; i<link->n; i++)
	{
		pPCL = &(link->pcls[i]);
		assert(pPCL->li >= 0 && pPCL->li < DOF);

		linkArray[pPCL->li].addPCL(PCL(pPCL->nV, pPCL->nF,
			pPCL->x, pPCL->y, pPCL->z, 
			pPCL->fx, pPCL->fy, pPCL->fz));		

		linkArray[pPCL->li].addBB(BoundingBox(pPCL->bb.H, 
			pPCL->bb.d[0], pPCL->bb.d[1], pPCL->bb.d[2]));

	}
}

KinChain::KinChain():rP(NULL), eP(NULL), hP(NULL), nPos(0)
{

}
KinChain::KinChain(KinChain& inst)
{
	this->nPos = inst.nPos;
	this->samplesPerPos = inst.samplesPerPos;
	rP = new float[nPos*samplesPerPos*NELEM_H*DOF_R];
	eP = new float[nPos*samplesPerPos*NELEM_H*DOF_E];
	hP = new float[nPos*samplesPerPos*NELEM_H*DOF_H];

	memcpy(hP, inst.hP, sizeof(float)*DOF_H*NELEM_H*samplesPerPos*nPos);
	memcpy(eP, inst.eP, sizeof(float)*DOF_E*NELEM_H*samplesPerPos*nPos);
	memcpy(rP, inst.rP, sizeof(float)*DOF_R*NELEM_H*samplesPerPos*nPos);

	for(int i=0; i< DOF_H; i++)
	{
		h[i] = inst.h[i];
	}

	for(int i=0; i< DOF_E; i++)
	{
		e[i] = inst.e[i];
	}

	for(int i=0; i< DOF_R; i++)
	{
		r[i] = inst.r[i];
	}

}
void KinChain::operator=(KinChain& inst )
{
	if(rP != NULL)
	{
		delete rP;
		delete eP;
		delete hP;
	}
	this->nPos = inst.nPos;
	this->samplesPerPos = inst.samplesPerPos;
	rP = new float[nPos*samplesPerPos*NELEM_H*DOF_R];
	eP = new float[nPos*samplesPerPos*NELEM_H*DOF_E];
	hP = new float[nPos*samplesPerPos*NELEM_H*DOF_H];

	memcpy(hP, inst.hP, sizeof(float)*DOF_H*NELEM_H*samplesPerPos*nPos);
	memcpy(eP, inst.eP, sizeof(float)*DOF_E*NELEM_H*samplesPerPos*nPos);
	memcpy(rP, inst.rP, sizeof(float)*DOF_R*NELEM_H*samplesPerPos*nPos);

	for(int i=0; i< DOF_H; i++)
	{
		h[i] = inst.h[i];
	}

	for(int i=0; i< DOF_E; i++)
	{
		e[i] = inst.e[i];
	}

	for(int i=0; i< DOF_R; i++)
	{
		r[i] = inst.r[i];
	}

}


KinChain::~KinChain(void)
{
	delete rP;
	delete eP;
	delete hP;
}

void KinChain::saveCurrentSzene(const char* fileName)
{
	int maxNF = 0;
	int maxNV = 0;
	int maxNBB = 0;
	for(int n=0; n<DOF_H; n++)
	{
		if (h[n].getnPCL() > 0)
		{
			for(int nPCL=0; nPCL<h[n].getnPCL(); nPCL++)
			{
				maxNF += h[n].getPCL()[nPCL].getnF();
				maxNV += h[n].getPCL()[nPCL].getnV();
			}
		}
		maxNBB += h[n].getnBB();
	}

	for(int n=0; n<DOF_R; n++)
	{
		if (r[n].getnPCL() > 0)
		{
			for(int nPCL=0; nPCL<r[n].getnPCL(); nPCL++)
			{
				maxNF += r[n].getPCL()[nPCL].getnF();
				maxNV += r[n].getPCL()[nPCL].getnV();
			}
		}
		maxNBB += r[n].getnBB();
	}

	for(int n=0; n<DOF_E; n++)
	{
		if (e[n].getnPCL() > 0)
		{
			for(int nPCL=0; nPCL<e[n].getnPCL(); nPCL++)
			{
				maxNF += e[n].getPCL()[nPCL].getnF();
				maxNV += e[n].getPCL()[nPCL].getnV();
			}
		}
		maxNBB += e[n].getnBB();
	}

	int* xFBuffer = new int[maxNF];
	int* yFBuffer = new int[maxNF];
	int* zFBuffer = new int[maxNF];
	float* xVBuffer = new float[maxNV];
	float* yVBuffer = new float[maxNV];
	float* zVBuffer = new float[maxNV];

	float* bbBuffer = new float[maxNBB*NELEM_H];
	float* dimBuffer = new float[maxNBB*3];

	int foffset =0;
	int voffset =0;
	int bboffset = 0;
	
	for(int n=0; n<DOF_H; n++)
	{
		if (h[n].getnPCL() > 0)
		{
			for(int nPCL=0; nPCL<h[n].getnPCL(); nPCL++)
			{
				for(int f = 0; f< h[n].getPCL()[nPCL].getnF(); f++)
				{
					xFBuffer[foffset+f] =  h[n].getPCL()[nPCL].getfx()[f]+voffset;
					yFBuffer[foffset+f] =  h[n].getPCL()[nPCL].getfy()[f]+voffset;
					zFBuffer[foffset+f] =  h[n].getPCL()[nPCL].getfz()[f]+voffset;
				}

				for(int v = 0; v< h[n].getPCL()[nPCL].getnV(); v++)
				{
					xVBuffer[voffset+v] =  h[n].getPCL()[nPCL].getx()[v];
					yVBuffer[voffset+v] =  h[n].getPCL()[nPCL].gety()[v];
					zVBuffer[voffset+v] =  h[n].getPCL()[nPCL].getz()[v];
				}
				foffset += h[n].getPCL()[nPCL].getnF();
				voffset += h[n].getPCL()[nPCL].getnV();
			}
		}

		for(int nBB=0; nBB<h[n].getnBB(); nBB++)
		{
			memcpy(bbBuffer+NELEM_H*bboffset, h[n].getBB()[nBB].getH().getH(), NELEM_H*sizeof(float));
			float tmp[3];
			tmp[0] = h[n].getBB()[nBB].getXDim();
			tmp[1] = h[n].getBB()[nBB].getYDim();
			tmp[2] = h[n].getBB()[nBB].getZDim();
			memcpy(dimBuffer+3*bboffset, tmp, 3*sizeof(float));
			bboffset++;

		}
	}

	for(int n=0; n<DOF_R; n++)
	{
		if (r[n].getnPCL() > 0)
		{
			for(int nPCL=0; nPCL<r[n].getnPCL(); nPCL++)
			{
				for(int f = 0; f< r[n].getPCL()[nPCL].getnF(); f++)
				{
					xFBuffer[foffset+f] =  r[n].getPCL()[nPCL].getfx()[f]+voffset;
					yFBuffer[foffset+f] =  r[n].getPCL()[nPCL].getfy()[f]+voffset;
					zFBuffer[foffset+f] =  r[n].getPCL()[nPCL].getfz()[f]+voffset;
				}

				for(int v = 0; v< r[n].getPCL()[nPCL].getnV(); v++)
				{
					xVBuffer[voffset+v] =  r[n].getPCL()[nPCL].getx()[v];
					yVBuffer[voffset+v] =  r[n].getPCL()[nPCL].gety()[v];
					zVBuffer[voffset+v] =  r[n].getPCL()[nPCL].getz()[v];
				}
				foffset += r[n].getPCL()[nPCL].getnF();
				voffset += r[n].getPCL()[nPCL].getnV();
			}
		}

		for(int nBB=0; nBB<r[n].getnBB(); nBB++)
		{
			memcpy(bbBuffer+NELEM_H*bboffset, r[n].getBB()[nBB].getH().getH(), NELEM_H*sizeof(float));
			float tmp[3];
			tmp[0] = r[n].getBB()[nBB].getXDim();
			tmp[1] = r[n].getBB()[nBB].getYDim();
			tmp[2] = r[n].getBB()[nBB].getZDim();
			memcpy(dimBuffer+3*bboffset, tmp, 3*sizeof(float));
			bboffset++;
		}
	}

	for(int n=0; n<DOF_E; n++)
	{
		if (e[n].getnPCL() > 0)
		{
			for(int nPCL=0; nPCL<e[n].getnPCL(); nPCL++)
			{
				for(int f = 0; f< e[n].getPCL()[nPCL].getnF(); f++)
				{
					xFBuffer[foffset+f] =  e[n].getPCL()[nPCL].getfx()[f]+voffset;
					yFBuffer[foffset+f] =  e[n].getPCL()[nPCL].getfy()[f]+voffset;
					zFBuffer[foffset+f] =  e[n].getPCL()[nPCL].getfz()[f]+voffset;
				}

				for(int v = 0; v< e[n].getPCL()[nPCL].getnV(); v++)
				{
					xVBuffer[voffset+v] =  e[n].getPCL()[nPCL].getx()[v];
					yVBuffer[voffset+v] =  e[n].getPCL()[nPCL].gety()[v];
					zVBuffer[voffset+v] =  e[n].getPCL()[nPCL].getz()[v];
				}
				foffset += e[n].getPCL()[nPCL].getnF();
				voffset += e[n].getPCL()[nPCL].getnV();
			}
		}

		for(int nBB=0; nBB<e[n].getnBB(); nBB++)
		{
			memcpy(bbBuffer+NELEM_H*bboffset, e[n].getBB()[nBB].getH().getH(), NELEM_H*sizeof(float));
			float tmp[3];
			tmp[0] = e[n].getBB()[nBB].getXDim();
			tmp[1] = e[n].getBB()[nBB].getYDim();
			tmp[2] = e[n].getBB()[nBB].getZDim();
			memcpy(dimBuffer+3*bboffset, tmp, 3*sizeof(float));
			bboffset++;
		}
	}

	std::ofstream outbin(fileName, std::ofstream::binary );
	if (!outbin) std::cerr << "error";

	outbin.write((char*)&maxNV,sizeof(int));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)&maxNF,sizeof(int));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)&maxNBB,sizeof(int));
	if (!outbin) std::cerr << "error";
		
	outbin.write((char*)xVBuffer,maxNV*sizeof(float));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)yVBuffer,maxNV*sizeof(float));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)zVBuffer,maxNV*sizeof(float));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)xFBuffer,maxNF*sizeof(int));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)yFBuffer,maxNF*sizeof(int));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)zFBuffer,maxNF*sizeof(int));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)bbBuffer,maxNBB*NELEM_H*sizeof(float));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)dimBuffer,maxNBB*3*sizeof(float));
	if (!outbin) std::cerr << "error";

	outbin.close();

	delete xFBuffer;
	delete yFBuffer;
	delete zFBuffer;
	delete xVBuffer;
	delete yVBuffer;
	delete zVBuffer;

	delete bbBuffer;
	delete dimBuffer;
}

void KinChain::setHumanPos(const float const * pos)
{
	memcpy(hP, pos, sizeof(float)*DOF_H*NELEM_H*samplesPerPos*nPos);

}
void KinChain::setEnvPos(const float const* pos)
{
	memcpy(eP, pos, sizeof(float)*DOF_E*NELEM_H*samplesPerPos*nPos);
}
void KinChain::setRobotPos(const float const* pos)
{
	memcpy(rP, pos, sizeof(float)*DOF_R*NELEM_H*samplesPerPos*nPos);
}

void KinChain::setPosIndex(int i)
{
	assert(i >= 0 && i < nPos);
	
	float* start_h = hP+i*samplesPerPos*NELEM_H*DOF_H;
	for(int ih=0; ih<DOF_H; ih++)
	{
		h[ih].setH(start_h+ih*NELEM_H);
	}

	float* start_r = rP+i*samplesPerPos*NELEM_H*DOF_R; 
	for(int ir=0; ir<DOF_R; ir++)
	{
		r[ir].setH(start_r+ir*NELEM_H);
	}

	float* start_e = eP+i*samplesPerPos*NELEM_H*DOF_E; 
	for(int ie=0; ie<DOF_E; ie++)
	{
		e[ie].setH(start_e+ie*NELEM_H);
	}	
}