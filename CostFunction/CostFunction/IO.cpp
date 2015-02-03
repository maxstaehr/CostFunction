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

void IO::savePCL(struct PCL* pcl, const char* name)
{
	ofstream outbin(name, ofstream::binary );
	outbin.write((char*)&pcl->n, sizeof(unsigned int));
	outbin.write((char*)pcl->x, pcl->n*sizeof(float));
	outbin.write((char*)pcl->y, pcl->n*sizeof(float));
	outbin.write((char*)pcl->z, pcl->n*sizeof(float));
	outbin.write((char*)pcl->i, pcl->n*sizeof(int));
	outbin.close();
}
//void IO::printMinPositionToFile(std::vector<struct COST_POINT*>* p, struct POSITIONS* pos, struct PCL* robotPCL_qa0, int nofCams)
//{
	//int size = p->size()*(nofCams*6+1)+2;
	//double* buffer = new double[size];
	//int pitch = nofCams*6+1;
	//int offset = 2;
	//buffer[0] = p->size();
	//buffer[1] = nofCams;

	//
	//for(unsigned int i=0; i<p->size(); i++)
	//{		
	//	for(unsigned int j=0; j<nofCams; j++)
	//	{
	//		buffer[i*pitch+j*6+0+offset] = robotPCL_qa0->x[(*p)[i][j].pcl];
	//		buffer[i*pitch+j*6+1+offset] = robotPCL_qa0->y[(*p)[i][j].pcl];
	//		buffer[i*pitch+j*6+2+offset] = robotPCL_qa0->z[(*p)[i][j].pcl];
	//		buffer[i*pitch+j*6+3+offset] = pos->roll[(*p)[i][j].angle];
	//		buffer[i*pitch+j*6+4+offset] = pos->pitch[(*p)[i][j].angle];
	//		buffer[i*pitch+j*6+5+offset] = pos->yaw[(*p)[i][j].angle];
	//	}
	//	buffer[i*pitch+nofCams*6+offset] = (*p)[i][0].c;
	//}

	//std::string fn = "minCameraPos" + to_string((_Longlong)nofCams) + ".bin";
	//ofstream outbin(fn, ofstream::binary );
	//outbin.write((char*)buffer, size*sizeof(double));
	//outbin.close();
	//delete buffer;
//}

void IO::loadPCL(struct PCL* pcl, const char* name)
{
	ifstream inbin(name, ofstream::binary );
	inbin.read((char*)&pcl->n, sizeof(unsigned int));
	CudaMem::allocPCLN(pcl);

	inbin.read((char*)pcl->x, pcl->n*sizeof(float));
	inbin.read((char*)pcl->y, pcl->n*sizeof(float));
	inbin.read((char*)pcl->z, pcl->n*sizeof(float));
	inbin.read((char*)pcl->i, pcl->n*sizeof(int));
	inbin.close();
	CudaMem::copyPCLHostToDevice(pcl);
}

//robotPos.ksdf = new float*[robot_positions->n];
void IO::writeKSDF(struct ROBOT_POSITION* robot_positions, unsigned int N)
{
	ofstream outbin( "ksdf.bin", ios::binary );
	for(unsigned int loop = 0; loop<robot_positions->n; loop++)
	{
			outbin.write((char*)robot_positions->ksdf[loop],N*sizeof(float));

	}
	outbin.close();
}

void IO::loadKSDF(struct ROBOT_POSITION* robot_positions, unsigned int N)
{
	ifstream inbin( "ksdf.bin", ios::binary );
	if(!inbin.good())
	{
		std::cerr << "could not find robotvelocity.dat" << std::endl;
		return;
	}
	for(unsigned int loop = 0; loop<robot_positions->n; loop++)
	{
		inbin.read((char*)robot_positions->ksdf[loop],N*sizeof(float));

	}
	inbin.close();
}

void IO::loadRobotPos(struct ROBOT_POSITION* pos, unsigned int N)
{
	std::ifstream file ( "robotposition.dat" );
	if(!file.good())
	{
		std::cerr << "could not find robotposition.dat" << std::endl;
		return;
	}
	std::ifstream fileVel ( "robotvelocity.dat" );
	if(!file.good())
	{
		std::cerr << "could not find robotvelocity.dat" << std::endl;
		return;
	}

	std::string value;
	float floatValue;
	pos->n = 0;
	while ( getline (file,value) )
    {
		pos->n++;
    }
	file.close();
	file.open("robotposition.dat");

	std::cout << "start reading : " << pos->n << " positions" << std::endl;

	pos->positions = new float[pos->n*9];
	pos->velocities = new float[pos->n*9];
	pos->ksdf = new float*[pos->n];
	for(unsigned int i=0;i<pos->n; i++)
		pos->ksdf[i] = new float[N];

	for(int r=0;r<pos->n; r++)
	{
		getline ( file, value, ',' );
		floatValue = (float)atof(value.c_str());
		pos->positions[r*9+0] = floatValue;

		getline ( file, value, ',' );
		floatValue = (float)atof(value.c_str());
		pos->positions[r*9+1] = floatValue;

		getline ( file, value, ',' );
		floatValue = (float)atof(value.c_str());
		pos->positions[r*9+2] = floatValue;

		getline ( file, value, ',' );
		floatValue = (float)atof(value.c_str());
		pos->positions[r*9+3] = floatValue;

		getline ( file, value, ',' );
		floatValue = (float)atof(value.c_str());
		pos->positions[r*9+4] = floatValue;

		getline ( file, value, ',' );
		floatValue = (float)atof(value.c_str());
		pos->positions[r*9+5] = floatValue;

		getline ( file, value, ',' );
		floatValue = (float)atof(value.c_str());
		pos->positions[r*9+6] = floatValue;

		getline ( file, value, ',' );
		floatValue = (float)atof(value.c_str());
		pos->positions[r*9+7] = floatValue;

		getline ( file, value);
		floatValue = (float)atof(value.c_str());
		pos->positions[r*9+8] = floatValue;


		//reading veclocities

		getline ( fileVel, value, ',' );
		floatValue = (float)atof(value.c_str());
		pos->velocities[r*9+0] = floatValue;

		floatValue = 0.0f;
		pos->velocities[r*9+1] = floatValue;

		getline ( fileVel, value, ',' );
		floatValue = (float)atof(value.c_str());
		pos->velocities[r*9+2] = floatValue;

		getline ( fileVel, value, ',' );
		floatValue = (float)atof(value.c_str());
		pos->velocities[r*9+3] = floatValue;

		getline ( fileVel, value, ',' );
		floatValue = (float)atof(value.c_str());
		pos->velocities[r*9+4] = floatValue;

		getline ( fileVel, value, ',' );
		floatValue = (float)atof(value.c_str());
		pos->velocities[r*9+5] = floatValue;

		getline ( fileVel, value, ',' );
		floatValue = (float)atof(value.c_str());
		pos->velocities[r*9+6] = floatValue;

		getline ( fileVel, value, ',' );
		floatValue = (float)atof(value.c_str());
		pos->velocities[r*9+7] = floatValue;

		getline ( fileVel, value);
		floatValue = (float)atof(value.c_str());
		pos->velocities[r*9+8] = floatValue;
	}

}

void IO::loadHumanPos(struct HUMAN_POSITION* pos)
{
	std::ifstream file ( "humanposition.dat" );

	if(!file.is_open())
	{
		std::cerr << "could not find humanposition.dat" << std::endl;
	}
	std::string value;
	float floatValue;

	pos->n = 0;
	while ( getline (file,value) )
	{
		pos->n++;
	}
	file.close();
	file.open("humanposition.dat");

	std::cout << "start reading : " << pos->n << " positions" << std::endl;

	pos->positions = new float[pos->n*3];
	for(int r=0;r<pos->n; r++)
	{
		getline ( file, value, ',' );
		floatValue = (float)atof(value.c_str());
		pos->positions[r*3+0] = floatValue;

		getline ( file, value, ',' );
		floatValue = (float)atof(value.c_str());
		pos->positions[r*3+1] = floatValue;

		getline ( file, value);
		floatValue = (float)atof(value.c_str());
		pos->positions[r*3+2] = floatValue;
	}

}

void IO::loadFileOntoCuda(std::string fn, void* p_memcuda, unsigned int size)
{
	//load robot occupancy in to cuda
	char* buffer = new char[size];
	cudaError		cudaStatus;
	ifstream inbin;


	inbin.open( fn.c_str(), ifstream::binary);
	if(!inbin.is_open())
	{
		std::cerr << "error";
	}
	inbin.read(buffer, size);
	if (!inbin)
	{
	      std::cerr << "error";
	}
	inbin.close();
	cudaStatus = cudaMemcpy(p_memcuda, buffer,  size, cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "error loading onto cuda", cudaStatus);
	}
	delete [] buffer;
}

void IO::compareFloatValuesCuda(float* p_d1,  unsigned int n, int offset,  const char* task, const char* fn)
{
	float* b1 = new float[n];
	float* b2 = new float[n];


	ifstream inbin;
	inbin.open( fn, ifstream::binary);
	if(!inbin.is_open())
	{
		std::cerr << "error";
	}

	inbin.seekg(offset*sizeof(float));
	inbin.read((char*)b2, n*sizeof(float));
	if (!inbin)
	{
	      std::cerr << "error";
	}
	inbin.close();

	CudaMem::cudaMemCpyReport(b1, p_d1, n*sizeof(float), cudaMemcpyDeviceToHost);


	for(unsigned int i=0; i<n; i++)
	{
		if(abs(b1[i]-b2[i])>1e-4)
		{
			std::cout <<b1[i] << "\t" << b2[i] << std::endl;
			printf("error in %s at position %i\n", task, i);
		}
	}
	delete b1;
	delete b2;
}

void IO::compareIntValuesCuda(unsigned int* p_d1, unsigned int n, int offset,  const char* task, const char* fn)
{
	unsigned int* b1 = new unsigned int[n];
	unsigned int* b2 = new unsigned int[n];


	ifstream inbin;
	inbin.open( fn, ifstream::binary);
	if(!inbin.is_open())
	{
		std::cerr << "error";
	}

	inbin.seekg(offset*sizeof(unsigned int));
	inbin.read((char*)b2, n*sizeof(unsigned int));
	if (!inbin)
	{
	      std::cerr << "error";
	}
	inbin.close();

	CudaMem::cudaMemCpyReport(b1, p_d1, n*sizeof(unsigned int), cudaMemcpyDeviceToHost);


	for(unsigned int i=0; i<n; i++)
	{
		if(b1[i] != b2[i])
		{
			printf("error in %s at position %i\n", task, i);
		}
	}
	delete b1;
	delete b2;
}


void IO::loadFileOntoHost(std::string fn, void* p_memhost, unsigned int size)
{
	//load robot occupancy in to host
	ifstream inbin;
	inbin.open( fn.c_str(), ifstream::binary);
	if(!inbin.good())
	{
		std::cerr << "error";
	}
	inbin.read((char*)p_memhost, size);
	if (!inbin)
	{
	      std::cerr << "error";
	}
	inbin.close();
}

void IO::printMinCostSingleCameraToFile(double* h_costs, struct POSITIONS* pos, struct PCL* robotPCL_qa0)
{

	double *buffer = new double[robotPCL_qa0->n*pos->nOfAngles*4];
	unsigned int cnt = 0;
	for(unsigned int pi=0; pi<robotPCL_qa0->n; pi++)
	{

		for(unsigned int anglei = 0; anglei<pos->nOfAngles; anglei++)
		{


			buffer[(pi*pos->nOfAngles + anglei)*4+0] =(double) robotPCL_qa0->x[pi];
			buffer[(pi*pos->nOfAngles + anglei)*4+1] =(double) robotPCL_qa0->y[pi];
			buffer[(pi*pos->nOfAngles + anglei)*4+2] =(double) robotPCL_qa0->z[pi];
			buffer[(pi*pos->nOfAngles + anglei)*4+3] =(double) h_costs[pi*pos->nOfAngles + anglei];


		}

	}



	std::string filename = "costs.bin";
	ofstream outbin( filename.c_str(), ios::binary );
	outbin.write((char*)buffer, robotPCL_qa0->n*pos->nOfAngles*4*sizeof(double));
	outbin.close();
	delete buffer;
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

	//pcl->n = 100;


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

	rot->nRotations = 200;
	
}


void IO::loadRobotPCL(struct ROBOT_PCL* pcl, const char* name)
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
	pcl->fi = new int[pcl->nF];

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
	CudaMem::cudaMemAllocReport((void**)&pcl->d_fi, pcl->nF*sizeof(int));

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

	inbin.read((char*)pcl->fi, pcl->nF*sizeof(int));
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
	CudaMem::cudaMemCpyReport(pcl->d_fi, pcl->fi, pcl->nF * sizeof(int), cudaMemcpyHostToDevice);

	CudaMem::cudaMemCpyReport(pcl->d_bb_H, pcl->bb_H, pcl->nBB*16*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pcl->d_bb_D, pcl->bb_D, pcl->nBB*3*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pcl->d_bbi, pcl->bbi, pcl->nBB*sizeof(int), cudaMemcpyHostToDevice);

}
void IO::loadEnvironmentPCL(struct ENVIRONMENT_PCL* pcl, const char* name)
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
	

	pcl->fx = new int[pcl->nF];
	pcl->fy = new int[pcl->nF];
	pcl->fz = new int[pcl->nF];
	

	pcl->bb_H = new float[pcl->nBB*16];
	pcl->bb_D = new float[pcl->nBB*3];
	

	CudaMem::cudaMemAllocReport((void**)&pcl->d_x, pcl->nV*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&pcl->d_y, pcl->nV*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&pcl->d_z, pcl->nV*sizeof(float));
	

	CudaMem::cudaMemAllocReport((void**)&pcl->d_fx, pcl->nF*sizeof(int));
	CudaMem::cudaMemAllocReport((void**)&pcl->d_fy, pcl->nF*sizeof(int));
	CudaMem::cudaMemAllocReport((void**)&pcl->d_fz, pcl->nF*sizeof(int));
	

	CudaMem::cudaMemAllocReport((void**)&pcl->d_bb_H, pcl->nBB*16*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&pcl->d_bb_D, pcl->nBB*3*sizeof(float));
	


	inbin.read((char*)pcl->x, pcl->nV*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)pcl->y, pcl->nV*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)pcl->z, pcl->nV*sizeof(float));
	if (!inbin) std::cerr << "error";


	inbin.read((char*)pcl->fx, pcl->nF*sizeof(int));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)pcl->fy, pcl->nF*sizeof(int));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)pcl->fz, pcl->nF*sizeof(int));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)pcl->bb_H, pcl->nBB*16*sizeof(int));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)pcl->bb_D, pcl->nBB*3*sizeof(int));
	if (!inbin) std::cerr << "error";

	char c;
	inbin.get(c);

	if(!inbin.eof()) std::cerr << "error";
	inbin.close();

	CudaMem::cudaMemCpyReport(pcl->d_x, pcl->x, pcl->nV * sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pcl->d_y, pcl->y, pcl->nV * sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pcl->d_z, pcl->z, pcl->nV * sizeof(float), cudaMemcpyHostToDevice);	

	CudaMem::cudaMemCpyReport(pcl->d_fx, pcl->fx, pcl->nF * sizeof(int), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pcl->d_fy, pcl->fy, pcl->nF * sizeof(int), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pcl->d_fz, pcl->fz, pcl->nF * sizeof(int), cudaMemcpyHostToDevice);	

	CudaMem::cudaMemCpyReport(pcl->d_bb_H, pcl->bb_H, pcl->nBB*16*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pcl->d_bb_D, pcl->bb_D, pcl->nBB*3*sizeof(float), cudaMemcpyHostToDevice);	
}

void IO::loadHumanPCL(struct HUMAN_PCL* pcl, const char* name)
{
	ifstream inbin;
	inbin.open(name, ifstream::binary);
	if(!inbin.is_open()) std::cerr << "error";
	
	inbin.read((char*)&pcl->nV, sizeof(int));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)&pcl->nF, sizeof(int));
	if (!inbin) std::cerr << "error";

	pcl->x = new float[pcl->nV];
	pcl->y = new float[pcl->nV];
	pcl->z = new float[pcl->nV];
	

	pcl->fx = new int[pcl->nF];
	pcl->fy = new int[pcl->nF];
	pcl->fz = new int[pcl->nF];
	

	CudaMem::cudaMemAllocReport((void**)&pcl->d_x, pcl->nV*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&pcl->d_y, pcl->nV*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&pcl->d_z, pcl->nV*sizeof(float));
	

	CudaMem::cudaMemAllocReport((void**)&pcl->d_fx, pcl->nF*sizeof(int));
	CudaMem::cudaMemAllocReport((void**)&pcl->d_fy, pcl->nF*sizeof(int));
	CudaMem::cudaMemAllocReport((void**)&pcl->d_fz, pcl->nF*sizeof(int));
	

	inbin.read((char*)pcl->x, pcl->nV*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)pcl->y, pcl->nV*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)pcl->z, pcl->nV*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)pcl->fx, pcl->nF*sizeof(int));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)pcl->fy, pcl->nF*sizeof(int));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)pcl->fz, pcl->nF*sizeof(int));
	if (!inbin) std::cerr << "error";


	char c;
	inbin.get(c);

	if(!inbin.eof()) std::cerr << "error";
	inbin.close();

	CudaMem::cudaMemCpyReport(pcl->d_x, pcl->x, pcl->nV * sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pcl->d_y, pcl->y, pcl->nV * sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pcl->d_z, pcl->z, pcl->nV * sizeof(float), cudaMemcpyHostToDevice);

	CudaMem::cudaMemCpyReport(pcl->d_fx, pcl->fx, pcl->nF * sizeof(int), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pcl->d_fy, pcl->fy, pcl->nF * sizeof(int), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pcl->d_fz, pcl->fz, pcl->nF * sizeof(int), cudaMemcpyHostToDevice);

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

	pos->nP = 2;
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

	}
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

