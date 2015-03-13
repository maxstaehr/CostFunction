/*
 * IO.cpp
 *
 *  Created on: 13.06.2014
 *      Author: tsdf
 */

#include "IO.h"
#include "CudaMem.h"
#include <fstream>
#include <iostream>
#include <string>
#include "struct_definitions.h"
#include "cuda.h"
#include "global.h"
#include "mathcuda.h"

#include <cstdlib>
#include <cmath>
//#include <stdlib.h>


using namespace std;
IO::IO() {
	// TODO Auto-generated constructor stub

}

IO::~IO() {
	// TODO Auto-generated destructor stub
}


void IO::loadSamplePCL(struct SAMPLE_PCL* pcl, const char* name)
{		
	ifstream inbin;
	inbin.open(name, ifstream::binary);
	if(!inbin.is_open()) std::cerr << "error";

	inbin.read((char*)&pcl->n, sizeof(int));
	if (!inbin) std::cerr << "error";


	//allocating memory
	pcl->h = new float[pcl->n*NUMELEM_H];
	pcl->x = new float[pcl->n];
	pcl->y = new float[pcl->n];
	pcl->z = new float[pcl->n];
	pcl->i = new int[pcl->n];

	CudaMem::cudaMemAllocReport((void**)&pcl->d_h, pcl->n*NUMELEM_H*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&pcl->d_i, pcl->n*sizeof(int));

	inbin.read((char*)pcl->h, pcl->n*NUMELEM_H*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)pcl->i, pcl->n*sizeof(int));
	if (!inbin) std::cerr << "error";

	char c;
	inbin.get(c);

	if(!inbin.eof()) std::cerr << "error";
	inbin.close();

	//setting x,y and z to the appropriate 
	for(int i=0; i<pcl->n; i++)
	{
		pcl->x[i] = (pcl->h+i*NUMELEM_H)[3];
		pcl->y[i] = (pcl->h+i*NUMELEM_H)[7];
		pcl->z[i] = (pcl->h+i*NUMELEM_H)[11];
	}

	CudaMem::cudaMemCpyReport(pcl->d_h, pcl->h, pcl->n*NUMELEM_H*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pcl->d_i, pcl->i, pcl->n*sizeof(int), cudaMemcpyHostToDevice);

	//pcl->n = 1;


}
void IO::saveInversionSearch(float* p, float* d, int* w, int n, const char* name)
{
		

	ofstream outbin(name, ofstream::binary );
	if (!outbin) std::cerr << "error";

	outbin.write((char*)&n, sizeof(int));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)p, n*sizeof(float));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)d, n*sizeof(float));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)w, n*sizeof(int));
	if (!outbin) std::cerr << "error";

	outbin.close();
}

void IO::loadInversionSearch(float* p, float* d, int* w, int* n, const char* name)
{
	ifstream inbin;
	inbin.open(name, ifstream::binary);
	if(!inbin.is_open()) std::cerr << "error";

	inbin.read((char*)n, sizeof(int));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)p, (*n)*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)d, (*n)*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)w, (*n)*sizeof(int));
	if (!inbin) std::cerr << "error";
		
	char c;
	inbin.get(c);

	if(!inbin.eof()) std::cerr << "error";
	inbin.close();
	
}

void IO::loadSampleRotations(struct SAMPLE_ROTATIONS* rot, const char* name)
{
	rot->angleLimits = new float[6];

	ifstream inbin;
	inbin.open(name, ifstream::binary);
	if(!inbin.is_open()) std::cerr << "error";

	inbin.read((char*)&rot->nRoll, sizeof(int));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)&rot->nPitch, sizeof(int));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)&rot->nYaw, sizeof(int));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)rot->angleLimits, 6*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)&rot->nRotations, sizeof(int));
	if (!inbin) std::cerr << "error";
	

	rot->R = new float[rot->nRotations*NUMELEM_H];

	inbin.read((char*)rot->R, rot->nRotations*NUMELEM_H*sizeof(float));
	if (!inbin) std::cerr << "error";

	char c;
	inbin.get(c);

	if(!inbin.eof()) std::cerr << "error";
	inbin.close();

	CudaMem::cudaMemAllocReport((void**)&rot->d_R, rot->nRotations*NUMELEM_H*sizeof(float));	
	CudaMem::cudaMemCpyReport(rot->d_R, rot->R, rot->nRotations*NUMELEM_H*sizeof(float), cudaMemcpyHostToDevice);


	
}


void IO::loadPCL(struct PCL* pcl, const char* name)
{
	ifstream inbin;
	inbin.open(name, ifstream::binary);
	if(!inbin.is_open()) std::cerr << "error";
	
	inbin.read((char*)&pcl->nV, sizeof(int));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)&pcl->nF, sizeof(int));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)&pcl->nBB, sizeof(int));
	if (!inbin) std::cerr << "error";


	pcl->x = new float[pcl->nV];
	pcl->y = new float[pcl->nV];
	pcl->z = new float[pcl->nV];
	pcl->vi = new int[pcl->nV];

	pcl->fx = new int[pcl->nF];
	pcl->fy = new int[pcl->nF];
	pcl->fz = new int[pcl->nF];
	pcl->f_bbi = new int[pcl->nF];

	pcl->bb_H = new float[pcl->nBB*16];
	pcl->bb_D = new float[pcl->nBB*3];
	pcl->bbi = new int[pcl->nBB];

	CudaMem::cudaMemAllocReport((void**)&pcl->d_x, pcl->nV*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&pcl->d_y, pcl->nV*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&pcl->d_z, pcl->nV*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&pcl->d_vi, pcl->nV*sizeof(int));

	CudaMem::cudaMemAllocReport((void**)&pcl->d_fx, pcl->nF*sizeof(int));
	CudaMem::cudaMemAllocReport((void**)&pcl->d_fy, pcl->nF*sizeof(int));
	CudaMem::cudaMemAllocReport((void**)&pcl->d_fz, pcl->nF*sizeof(int));
	CudaMem::cudaMemAllocReport((void**)&pcl->d_f_bbi, pcl->nF*sizeof(int));

	CudaMem::cudaMemAllocReport((void**)&pcl->d_bb_H, pcl->nBB*16*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&pcl->d_bb_D, pcl->nBB*3*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&pcl->d_bbi, pcl->nBB*sizeof(int));


	inbin.read((char*)pcl->x, pcl->nV*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)pcl->y, pcl->nV*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)pcl->z, pcl->nV*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)pcl->vi, pcl->nV*sizeof(int));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)pcl->fx, pcl->nF*sizeof(int));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)pcl->fy, pcl->nF*sizeof(int));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)pcl->fz, pcl->nF*sizeof(int));
	if (!inbin) std::cerr << "error";

	//here the bullshit comes
	//reading normals and middlepoints, we don't need
	inbin.ignore(pcl->nF*6*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)pcl->f_bbi, pcl->nF*sizeof(int));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)pcl->bb_H, pcl->nBB*16*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)pcl->bb_D, pcl->nBB*3*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)pcl->bbi, pcl->nBB*sizeof(int));
	if (!inbin) std::cerr << "error";

	char c;
	inbin.get(c);

	if(!inbin.eof()) std::cerr << "error";
	inbin.close();

	CudaMem::cudaMemCpyReport(pcl->d_x, pcl->x, pcl->nV * sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pcl->d_y, pcl->y, pcl->nV * sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pcl->d_z, pcl->z, pcl->nV * sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pcl->d_vi, pcl->vi, pcl->nV * sizeof(int), cudaMemcpyHostToDevice);

	CudaMem::cudaMemCpyReport(pcl->d_fx, pcl->fx, pcl->nF * sizeof(int), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pcl->d_fy, pcl->fy, pcl->nF * sizeof(int), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pcl->d_fz, pcl->fz, pcl->nF * sizeof(int), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pcl->d_f_bbi, pcl->f_bbi, pcl->nF * sizeof(int), cudaMemcpyHostToDevice);

	CudaMem::cudaMemCpyReport(pcl->d_bb_H, pcl->bb_H, pcl->nBB*16*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pcl->d_bb_D, pcl->bb_D, pcl->nBB*3*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pcl->d_bbi, pcl->bbi, pcl->nBB*sizeof(int), cudaMemcpyHostToDevice);

}

void IO::loadPCL(struct PCL* pcl, std::ifstream* inbin)
{

	if(!inbin->is_open()) std::cerr << "error";
	
	inbin->read((char*)&pcl->nV, sizeof(int));
	if (!inbin) std::cerr << "error";

	inbin->read((char*)&pcl->nF, sizeof(int));
	if (!inbin) std::cerr << "error";

	inbin->read((char*)&pcl->nBB, sizeof(int));
	if (!inbin) std::cerr << "error";


	pcl->x = new float[pcl->nV];
	pcl->y = new float[pcl->nV];
	pcl->z = new float[pcl->nV];
	pcl->vi = new int[pcl->nV];

	pcl->fx = new int[pcl->nF];
	pcl->fy = new int[pcl->nF];
	pcl->fz = new int[pcl->nF];
	pcl->f_bbi = new int[pcl->nF];

	pcl->bb_H = new float[pcl->nBB*16];
	pcl->bb_D = new float[pcl->nBB*3];
	pcl->bbi = new int[pcl->nBB];

	CudaMem::cudaMemAllocReport((void**)&pcl->d_x, pcl->nV*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&pcl->d_y, pcl->nV*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&pcl->d_z, pcl->nV*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&pcl->d_vi, pcl->nV*sizeof(int));

	CudaMem::cudaMemAllocReport((void**)&pcl->d_fx, pcl->nF*sizeof(int));
	CudaMem::cudaMemAllocReport((void**)&pcl->d_fy, pcl->nF*sizeof(int));
	CudaMem::cudaMemAllocReport((void**)&pcl->d_fz, pcl->nF*sizeof(int));
	CudaMem::cudaMemAllocReport((void**)&pcl->d_f_bbi, pcl->nF*sizeof(int));

	CudaMem::cudaMemAllocReport((void**)&pcl->d_bb_H, pcl->nBB*16*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&pcl->d_bb_D, pcl->nBB*3*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&pcl->d_bbi, pcl->nBB*sizeof(int));


	inbin->read((char*)pcl->x, pcl->nV*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin->read((char*)pcl->y, pcl->nV*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin->read((char*)pcl->z, pcl->nV*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin->read((char*)pcl->vi, pcl->nV*sizeof(int));
	if (!inbin) std::cerr << "error";

	inbin->read((char*)pcl->fx, pcl->nF*sizeof(int));
	if (!inbin) std::cerr << "error";

	inbin->read((char*)pcl->fy, pcl->nF*sizeof(int));
	if (!inbin) std::cerr << "error";

	inbin->read((char*)pcl->fz, pcl->nF*sizeof(int));
	if (!inbin) std::cerr << "error";

	//here the bullshit comes
	//reading normals and middlepoints, we don't need
	inbin->ignore(pcl->nF*6*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin->read((char*)pcl->f_bbi, pcl->nF*sizeof(int));
	if (!inbin) std::cerr << "error";

	inbin->read((char*)pcl->bb_H, pcl->nBB*16*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin->read((char*)pcl->bb_D, pcl->nBB*3*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin->read((char*)pcl->bbi, pcl->nBB*sizeof(int));
	if (!inbin) std::cerr << "error";
	

	CudaMem::cudaMemCpyReport(pcl->d_x, pcl->x, pcl->nV * sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pcl->d_y, pcl->y, pcl->nV * sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pcl->d_z, pcl->z, pcl->nV * sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pcl->d_vi, pcl->vi, pcl->nV * sizeof(int), cudaMemcpyHostToDevice);

	CudaMem::cudaMemCpyReport(pcl->d_fx, pcl->fx, pcl->nF * sizeof(int), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pcl->d_fy, pcl->fy, pcl->nF * sizeof(int), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pcl->d_fz, pcl->fz, pcl->nF * sizeof(int), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pcl->d_f_bbi, pcl->f_bbi, pcl->nF * sizeof(int), cudaMemcpyHostToDevice);

	CudaMem::cudaMemCpyReport(pcl->d_bb_H, pcl->bb_H, pcl->nBB*16*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pcl->d_bb_D, pcl->bb_D, pcl->nBB*3*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pcl->d_bbi, pcl->bbi, pcl->nBB*sizeof(int), cudaMemcpyHostToDevice);
}

void IO::loadSamplePositions(struct SAMPLE_POSITIONS* pos, const char* name)
{
	ifstream inbin;
	inbin.open(name, ifstream::binary);
	if(!inbin.is_open()) std::cerr << "error";

	inbin.read((char*)&pos->nS, sizeof(int));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)&pos->nP, sizeof(int));
	if (!inbin) std::cerr << "error";
	 

	pos->qr = new float[pos->nP*pos->nS*N_ELEMENT_T *NUMELEM_H];
	pos->qh = new float[pos->nP*N_ELEMENT_HU*NUMELEM_H];
	pos->qe = new float[pos->nP*pos->nS*N_ELEMENT_EV*NUMELEM_H];
	pos->pr = new float[pos->nP];

	CudaMem::cudaMemAllocReport((void**)&pos->d_qr, pos->nP*pos->nS*N_ELEMENT_T *NUMELEM_H*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&pos->d_qh, pos->nP*N_ELEMENT_HU*NUMELEM_H*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&pos->d_qe, pos->nP*pos->nS*N_ELEMENT_EV*NUMELEM_H*sizeof(float));	
	CudaMem::cudaMemAllocReport((void**)&pos->d_pr, pos->nP*sizeof(float));

	inbin.read((char*)pos->qr,pos->nP*pos->nS*N_ELEMENT_T *NUMELEM_H*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)pos->qh, pos->nP*N_ELEMENT_HU*NUMELEM_H*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)pos->qe,pos->nP*pos->nS*N_ELEMENT_EV*NUMELEM_H*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)pos->pr,pos->nP*sizeof(float));
	if (!inbin) std::cerr << "error";

	char c;
	inbin.get(c);

	if(!inbin.eof()) std::cerr << "error";

	inbin.close();

	//pos->nP = 2;
	//normalizing priorities to one
	float sum = 0.0;
	for(int i=0; i<pos->nP; i++)
	{
		sum += pos->pr[i];
	}
	printf("sum of all priorities %.5f\n",sum);

	for(int i=0; i<pos->nP; i++)
	{
		pos->pr[i] /= sum;
		//pos->pr[i] = 1.0f/pos->nP;
	}

	sum = 0.0;
	for(int i=0; i<pos->nP; i++)
	{
		sum += pos->pr[i];
	}
	printf("normalized sum of all priorities %.5f\n",sum);



	CudaMem::cudaMemCpyReport(pos->d_qr, pos->qr, pos->nP*pos->nS*N_ELEMENT_T *NUMELEM_H*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pos->d_qh, pos->qh, pos->nP*N_ELEMENT_HU*NUMELEM_H*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pos->d_qe, pos->qe, pos->nP*pos->nS*N_ELEMENT_EV*NUMELEM_H*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pos->d_pr, pos->pr, pos->nP*sizeof(float), cudaMemcpyHostToDevice);

	for(int i=0; i<pos->nP;i++)
	{
		printf("%d\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.8f\n", i,	pos->qr[i*N_ELEMENT_T*NUMELEM_H+3], pos->qr[i*N_ELEMENT_T*NUMELEM_H+7],
																pos->qe[i*N_ELEMENT_HU*NUMELEM_H+3], pos->qe[i*N_ELEMENT_HU*NUMELEM_H+7],
																pos->qh[i*N_ELEMENT_EV*NUMELEM_H+3], pos->qh[i*N_ELEMENT_EV*NUMELEM_H+7],
																 pos->pr[i]);
	}

}


void IO::loadSampleCamera(struct POSSIBLE_CAMERA_TYPES* cams, const char* name)
{
	ifstream inbin;
	inbin.open(name, ifstream::binary);
	if(!inbin.is_open()) std::cerr << "error";

	inbin.read((char*)&cams->nCameraTypes, sizeof(int));
	if (!inbin) std::cerr << "error";

	//allocating memor for the camera types
	cams->possibleCameraTypes = new struct SAMPLE_CAMERA[cams->nCameraTypes];

	struct SAMPLE_CAMERA* pCamera;
	for(int i=0; i<cams->nCameraTypes; i++)
	{	
		pCamera = &cams->possibleCameraTypes[i];

		inbin.read((char*)&pCamera->nBlocks, sizeof(int));
		if (!inbin) std::cerr << "error";

		inbin.read((char*)&pCamera->nThreads, sizeof(int));
		if (!inbin) std::cerr << "error";
	
		inbin.read((char*)&pCamera->nRays, sizeof(int));
		if (!inbin) std::cerr << "error";

		inbin.read((char*)&pCamera->ssnBlocks, sizeof(int));
		if (!inbin) std::cerr << "error";

		inbin.read((char*)&pCamera->ssnThreads, sizeof(int));
		if (!inbin) std::cerr << "error";

		inbin.read((char*)&pCamera->ssnRays, sizeof(int));
		if (!inbin) std::cerr << "error";

		inbin.read((char*)&pCamera->nx, sizeof(int));
		if (!inbin) std::cerr << "error";

		inbin.read((char*)&pCamera->ny, sizeof(int));
		if (!inbin) std::cerr << "error";

		inbin.read((char*)&pCamera->ss_x, sizeof(int));
		if (!inbin) std::cerr << "error";

		inbin.read((char*)&pCamera->ss_y, sizeof(int));
		if (!inbin) std::cerr << "error";

		inbin.read((char*)&pCamera->minW, sizeof(int));
		if (!inbin) std::cerr << "error";


		pCamera->x = new float[pCamera->nRays];
		pCamera->y = new float[pCamera->nRays];
		pCamera->z = new float[pCamera->nRays];

		pCamera->ssx = new float[pCamera->ssnRays];
		pCamera->ssy = new float[pCamera->ssnRays];
		pCamera->ssz = new float[pCamera->ssnRays];

		pCamera->c = new float[N_COEFFS];
		pCamera->d = new float[2];

	
		CudaMem::cudaMemAllocReport((void**)&pCamera->d_x, pCamera->nRays*sizeof(float));
		CudaMem::cudaMemAllocReport((void**)&pCamera->d_y, pCamera->nRays*sizeof(float));
		CudaMem::cudaMemAllocReport((void**)&pCamera->d_z, pCamera->nRays*sizeof(float));	

		CudaMem::cudaMemAllocReport((void**)&pCamera->d_ss_x, pCamera->ssnRays*sizeof(float));
		CudaMem::cudaMemAllocReport((void**)&pCamera->d_ss_y, pCamera->ssnRays*sizeof(float));
		CudaMem::cudaMemAllocReport((void**)&pCamera->d_ss_z, pCamera->ssnRays*sizeof(float));	


		CudaMem::cudaMemAllocReport((void**)&pCamera->d_c, N_COEFFS*sizeof(float));	
		CudaMem::cudaMemAllocReport((void**)&pCamera->d_d, 2*sizeof(float));


		inbin.read((char*)pCamera->x,pCamera->nRays*sizeof(float));
		if (!inbin) std::cerr << "error";

		inbin.read((char*)pCamera->y, pCamera->nRays*sizeof(float));
		if (!inbin) std::cerr << "error";

		inbin.read((char*)pCamera->z, pCamera->nRays*sizeof(float));
		if (!inbin) std::cerr << "error";


		inbin.read((char*)pCamera->ssx,pCamera->ssnRays*sizeof(float));
		if (!inbin) std::cerr << "error";

		inbin.read((char*)pCamera->ssy, pCamera->ssnRays*sizeof(float));
		if (!inbin) std::cerr << "error";

		inbin.read((char*)pCamera->ssz, pCamera->ssnRays*sizeof(float));
		if (!inbin) std::cerr << "error";


		inbin.read((char*)pCamera->c, N_COEFFS*sizeof(float));
		if (!inbin) std::cerr << "error";

		inbin.read((char*)pCamera->d, 2*sizeof(float));
		if (!inbin) std::cerr << "error";

		CudaMem::cudaMemCpyReport(pCamera->d_x, pCamera->x, pCamera->nRays*sizeof(float), cudaMemcpyHostToDevice);
		CudaMem::cudaMemCpyReport(pCamera->d_y, pCamera->y, pCamera->nRays*sizeof(float), cudaMemcpyHostToDevice);
		CudaMem::cudaMemCpyReport(pCamera->d_z, pCamera->z, pCamera->nRays*sizeof(float), cudaMemcpyHostToDevice);

		CudaMem::cudaMemCpyReport(pCamera->d_ss_x, pCamera->ssx, pCamera->ssnRays*sizeof(float), cudaMemcpyHostToDevice);
		CudaMem::cudaMemCpyReport(pCamera->d_ss_y, pCamera->ssy, pCamera->ssnRays*sizeof(float), cudaMemcpyHostToDevice);
		CudaMem::cudaMemCpyReport(pCamera->d_ss_z, pCamera->ssz, pCamera->ssnRays*sizeof(float), cudaMemcpyHostToDevice);

		CudaMem::cudaMemCpyReport(pCamera->d_c, pCamera->c, N_COEFFS*sizeof(float), cudaMemcpyHostToDevice);
		CudaMem::cudaMemCpyReport(pCamera->d_d, pCamera->d, 2*sizeof(float), cudaMemcpyHostToDevice);

		int rx = pCamera->nx/2;
		int ry = pCamera->ny/2;
		pCamera->rmax = rx*rx+ry*ry;

		
		IO::loadPCL(&(pCamera->pcl), &inbin);

	}
	char c;
	inbin.get(c);

	if(!inbin.eof()) std::cerr << "error";
	inbin.close();


}

void IO::loadResultingSolution(struct RESULT_SOLUTION* solu, const char* name)
{
	ifstream inbin;
	inbin.open(name, ifstream::binary);
	if(!inbin.is_open()) std::cerr << "error";
	
	inbin.read((char*)&solu->nC, sizeof(int));
	if (!inbin) std::cerr << "error";

	solu->cameraTypes = new int[solu->nC];
	solu->pclIndex = new int[solu->nC];
	solu->angleIndex = new int[solu->nC];

	inbin.read((char*)solu->cameraTypes, solu->nC*sizeof(int));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)solu->pclIndex, solu->nC*sizeof(int));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)solu->pclIndex, solu->nC*sizeof(int));
	if (!inbin) std::cerr << "error";

	char c;
	inbin.get(c);

	if(!inbin.eof()) std::cerr << "error";
	inbin.close();


}
void IO::saveBoundingBoxBuffer(struct BB_BUFFER* bbBuffer, const char* name)
{

	float* bb_buffer = new float[bbBuffer->nBB*NUMELEM_H];
	float* dim_buffer = new float[bbBuffer->nBB*3];
	CudaMem::cudaMemCpyReport(bb_buffer, bbBuffer->d_BB, bbBuffer->nBB*NUMELEM_H*sizeof(float), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(dim_buffer, bbBuffer->d_D, bbBuffer->nBB*3*sizeof(float), cudaMemcpyDeviceToHost);

	ofstream outbin(name, ofstream::binary );
	if (!outbin) std::cerr << "error";

	outbin.write((char*)& bbBuffer->nBB, sizeof(int));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)bb_buffer, bbBuffer->nBB*NUMELEM_H*sizeof(float));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)dim_buffer, bbBuffer->nBB*3*sizeof(float));
	if (!outbin) std::cerr << "error";

	outbin.close();
}

void IO::saveDepthBufferToFileSuperSamples(struct DEPTH_BUFFER* depth, const char* name)
{
	float* x = new float[depth->sssize];
	float* y = new float[depth->sssize];
	float* z = new float[depth->sssize];

	CudaMem::cudaMemCpyReport(x, depth->d_ss_dx, depth->sssize*sizeof(float), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(y, depth->d_ss_dy, depth->sssize*sizeof(float), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(z, depth->d_ss_dz, depth->sssize*sizeof(float), cudaMemcpyDeviceToHost);

	ofstream outbin(name, ofstream::binary );
	if (!outbin) std::cerr << "error";

	outbin.write((char*)&depth->size, sizeof(int));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)x, depth->size*sizeof(float));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)y, depth->size*sizeof(float));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)z, depth->size*sizeof(float));
	if (!outbin) std::cerr << "error";

	outbin.close();

	delete x;
	delete y;
	delete z;
}

void IO::saveDepthBufferToFile(struct DEPTH_BUFFER* depth, const char* name)
{
	float* x = new float[depth->size];
	float* y = new float[depth->size];
	float* z = new float[depth->size];

	CudaMem::cudaMemCpyReport(x, depth->d_dx, depth->size*sizeof(float), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(y, depth->d_dy, depth->size*sizeof(float), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(z, depth->d_dz, depth->size*sizeof(float), cudaMemcpyDeviceToHost);

	ofstream outbin(name, ofstream::binary );
	if (!outbin) std::cerr << "error";

	outbin.write((char*)&depth->size, sizeof(int));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)x, depth->size*sizeof(float));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)y, depth->size*sizeof(float));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)z, depth->size*sizeof(float));
	if (!outbin) std::cerr << "error";

	outbin.close();

	//check avg value
	float vx = 0.0f;
	float vy = 0.0f;
	float vz = 0.0f;
	int w = 0;
	for(int i=0; i<depth->size; i++)
	{
		if(!IO::is_nan(x[i]))
		{
			vx += x[i];
			vy += y[i];
			vz += z[i];
			w++;
		}
	}
	vx = vx/w;
	vy = vy/w;
	vz = vz/w;
	printf("centroid: [%.6f  %.6f  %.6f]\t%d\n", vx, vy, vz, w);
	delete x;
	delete y;
	delete z;
}

void IO::saveVerticeBufferToFile(struct VERTEX_BUFFER* buffer, const char* name)
{
	float* buf = new float[3*buffer->nV];
	int*   bufFace = new int[3*buffer->nF];

	CudaMem::cudaMemCpyReport(buf+0*buffer->nV, buffer->d_vx, buffer->nV*sizeof(float), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(buf+1*buffer->nV, buffer->d_vy, buffer->nV*sizeof(float), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(buf+2*buffer->nV, buffer->d_vz, buffer->nV*sizeof(float), cudaMemcpyDeviceToHost);

	CudaMem::cudaMemCpyReport(bufFace+0*buffer->nF, buffer->d_fx, buffer->nF*sizeof(int), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(bufFace+1*buffer->nF, buffer->d_fy, buffer->nF*sizeof(int), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(bufFace+2*buffer->nF, buffer->d_fz, buffer->nF*sizeof(int), cudaMemcpyDeviceToHost);

	ofstream outbin(name, ofstream::binary );
	if (!outbin) std::cerr << "error";

	outbin.write((char*)&buffer->nV,sizeof(int));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)&buffer->nF,sizeof(int));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)buf,3*buffer->nV*sizeof(float));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)bufFace,3*buffer->nF*sizeof(int));
	if (!outbin) std::cerr << "error";

	outbin.close();

	delete buf;
	delete bufFace;
}

void IO::saveBoundingBoxBufferToFile(struct BB_BUFFER* buffer, const char* name)
{
	float* buf = new float[NUMELEM_H*buffer->nBB];
	CudaMem::cudaMemCpyReport(buf, buffer->d_BB, buffer->nBB*NUMELEM_H*sizeof(float), cudaMemcpyDeviceToHost);

	ofstream outbin(name, ofstream::binary );
	if (!outbin) std::cerr << "error";

	outbin.write((char*)&buffer->nBB,sizeof(int));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)buf, buffer->nBB*NUMELEM_H*sizeof(float));
	if (!outbin) std::cerr << "error";

	outbin.close();

	delete buf;
}

void IO::printCentroid(struct CENTROID* centroid)
{
	float x, y, z;
	CudaMem::cudaMemCpyReport(&x,centroid->d_cx, sizeof(float), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(&y,centroid->d_cy, sizeof(float), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(&z,centroid->d_cz, sizeof(float), cudaMemcpyDeviceToHost);
	printf("centroid: [%.6f  %.6f  %.6f]\n", x, y, z);

}

void IO::loadSampleFitting(struct SAMPLE_FITTING* sampleFitting,struct LAUNCH_CONFIG* config, const char* name)
{
	ifstream inbin;
	inbin.open(name, ifstream::binary);
	if(!inbin.is_open()) std::cerr << "error";
	
	inbin.read((char*)&sampleFitting->n, sizeof(int));
	if (!inbin) std::cerr << "error";
	 
	sampleFitting->R = new float[sampleFitting->n*9];
	sampleFitting->Fx = new float[sampleFitting->n];
	sampleFitting->Fy = new float[sampleFitting->n];
	
	CudaMem::cudaMemAllocReport((void**)&sampleFitting->d_R, sampleFitting->n*9*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&sampleFitting->d_Fx, sampleFitting->n*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&sampleFitting->d_Fy, sampleFitting->n*sizeof(float));	

	inbin.read((char*)sampleFitting->R, sampleFitting->n*9*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)sampleFitting->Fx, sampleFitting->n*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)sampleFitting->Fy, sampleFitting->n*sizeof(float));
	if (!inbin) std::cerr << "error";

	char c;
	inbin.get(c);

	if(!inbin.eof()) std::cerr << "error";
	inbin.close();

	CudaMem::cudaMemCpyReport(sampleFitting->d_R, sampleFitting->R, sampleFitting->n*9*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(sampleFitting->d_Fx,sampleFitting->Fx, sampleFitting->n*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(sampleFitting->d_Fy,sampleFitting->Fy, sampleFitting->n*sizeof(float), cudaMemcpyHostToDevice);

	if (sampleFitting->n%THREADS_MODEL_FITTING != 0) std::cerr << "error";

	config->nblocks = sampleFitting->n/THREADS_MODEL_FITTING;
	config->nthreads = THREADS_MODEL_FITTING;

}

void IO::saveProbResult2File(struct PROB_RESULT* probResult, const char* name)
{
	float* p = new float[probResult->n];
	float max;
	CudaMem::cudaMemCpyReport(p, probResult->d_p, probResult->n*sizeof(float), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(&max, probResult->d_maxp, sizeof(float), cudaMemcpyDeviceToHost);

	float m = 0.0f;
	for(int i=0;i<probResult->n; i++)
	{
		m = std::max(m,p[i]);
	}
	printf("max_p: %.5f\t%.5f\n", max, m);
	ofstream outbin(name, ofstream::binary );
	if (!outbin) std::cerr << "error";

	outbin.write((char*)&probResult->n,sizeof(int));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)p, probResult->n*sizeof(float));
	if (!outbin) std::cerr << "error";

	outbin.close();
	delete p;

}

void IO::plotIntermediateResults(struct PROB_RESULT* probResult, struct CENTROID* centroid)
{

	float x, y, z, max;
	CudaMem::cudaMemCpyReport(&x,centroid->d_cx, sizeof(float), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(&y,centroid->d_cy, sizeof(float), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(&z,centroid->d_cz, sizeof(float), cudaMemcpyDeviceToHost);		
	CudaMem::cudaMemCpyReport(&max, probResult->d_maxp, sizeof(float), cudaMemcpyDeviceToHost);
	printf("centroid: [%.6f  %.6f  %.6f]\t%.6f\n", x, y, z, max);

}

void IO::loadDistanceMatrix(struct DISTANCE_MATRIX* dM, const char* name)
{
	//xmin xmax ymin ymax zmin zmax nx ny nz
	dM->ws_l_params = new float[9];
	dM->ws_n_params = new int[3];
	CudaMem::cudaMemAllocReport((void**)&dM->d_ws_l_params, 9*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&dM->d_ws_n_params, 3*sizeof(int));

	ifstream inbin;
	inbin.open(name, ifstream::binary);
	if(!inbin.is_open()) std::cerr << "error";
	
	inbin.read((char*)dM->ws_l_params, 6*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)dM->ws_n_params, 3*sizeof(int));
	if (!inbin) std::cerr << "error";

	dM->ws_l_params[6] = (dM->ws_l_params[1]-dM->ws_l_params[0])/dM->ws_n_params[0];
	dM->ws_l_params[7] = (dM->ws_l_params[3]-dM->ws_l_params[2])/dM->ws_n_params[1];
	dM->ws_l_params[8] = (dM->ws_l_params[5]-dM->ws_l_params[4])/dM->ws_n_params[2];

	dM->n = dM->ws_n_params[0]*dM->ws_n_params[1]*dM->ws_n_params[2];

	dM->d = new float[dM->n];
	CudaMem::cudaMemAllocReport((void**)&dM->d_d,dM->n*sizeof(float));
	
	inbin.read((char*)dM->d, dM->n*sizeof(float));
	if (!inbin) std::cerr << "error";

	char c;
	inbin.get(c);

	if(!inbin.eof()) std::cerr << "error";
	inbin.close();

	CudaMem::cudaMemCpyReport(dM->d_ws_l_params, dM->ws_l_params,	9*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(dM->d_ws_n_params, dM->ws_n_params,	3*sizeof(int), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(dM->d_d, dM->d,						dM->n*sizeof(float), cudaMemcpyHostToDevice);




}

void IO::saveOptimisationResults(struct SAMPLE_POINTS_BUFFER* samplePoints, struct SAMPLE_PCL* sP,struct SAMPLE_ROTATIONS* sR, float* costs, float* d,int* weights,  const char* name)
{
	float* buffer = new float[samplePoints->n*NUMELEM_H];
	float* res_pos_buffer = new float[samplePoints->n*sR->nRotations*NUMELEM_H];

	//float* x = new float[samplePoints->n];
	//float* y = new float[samplePoints->n];
	//float* z = new float[samplePoints->n];
	/*float* max = new float[samplePoints->n*sample];*/

	CudaMem::cudaMemCpyReport(buffer, samplePoints->d_H, samplePoints->n*NUMELEM_H*sizeof(float), cudaMemcpyDeviceToHost);
	//calculating the resulting transformations
	for(int i=0; i<samplePoints->n; i++)
	{
		for(int j=0; j<sR->nRotations; j++)
		{
			mm16(buffer+i*NUMELEM_H, sR->R+j*NUMELEM_H, res_pos_buffer+(i*sR->nRotations+j)*NUMELEM_H);
		}
	}

	//for(int i=0; i<samplePoints->n; i++)
	//{
	//	x[i] = buffer[i*NUMELEM_H+3];
	//	y[i] = buffer[i*NUMELEM_H+7];
	//	z[i] = buffer[i*NUMELEM_H+11];

	//	//float m = 0.0f;
	//	//for(int j=0; j<sR->nRotations; j++)
	//	//{
	//	//	m = std::max(m, costs[i*sR->nRotations+j]);
	//	//}
	//	//max[i] = m;
	//}

	ofstream outbin(name, ofstream::binary );
	if (!outbin) std::cerr << "error";

	outbin.write((char*)&samplePoints->n,sizeof(int));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)&sR->nRotations,sizeof(int));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)res_pos_buffer,samplePoints->n*sR->nRotations*NUMELEM_H*sizeof(float));
	if (!outbin) std::cerr << "error";

	//outbin.write((char*)x, samplePoints->n*sizeof(float));
	//if (!outbin) std::cerr << "error";

	//outbin.write((char*)y, samplePoints->n*sizeof(float));
	//if (!outbin) std::cerr << "error";

	//outbin.write((char*)z, samplePoints->n*sizeof(float));
	//if (!outbin) std::cerr << "error";

	outbin.write((char*)costs, samplePoints->n*sR->nRotations*sizeof(float));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)d, samplePoints->n*sR->nRotations*sizeof(float));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)weights, samplePoints->n*sR->nRotations*sizeof(int));
	if (!outbin) std::cerr << "error";

	outbin.close();


	delete buffer;
	//delete x;
	//delete y;
	//delete z;
	//delete max;
}

void IO::waitForEnter() 
{ 
        std::cout << "Bitte druecken Sie die Eingabetaste um fortzufahren..." << std::endl; 
        std::cin.clear(); 
        std::cin.ignore(std::cin.rdbuf()->in_avail()); 
        std::cin.get(); 
}  

