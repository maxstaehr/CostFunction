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
void IO::printMinPositionToFile(std::vector<struct COST_POINT*>* p, struct POSITIONS* pos, struct PCL* robotPCL_qa0, int nofCams)
{
	int size = p->size()*(nofCams*6+1)+2;
	double* buffer = new double[size];
	int pitch = nofCams*6+1;
	int offset = 2;
	buffer[0] = p->size();
	buffer[1] = nofCams;

	
	for(unsigned int i=0; i<p->size(); i++)
	{		
		for(unsigned int j=0; j<nofCams; j++)
		{
			buffer[i*pitch+j*6+0+offset] = robotPCL_qa0->x[(*p)[i][j].pcl];
			buffer[i*pitch+j*6+1+offset] = robotPCL_qa0->y[(*p)[i][j].pcl];
			buffer[i*pitch+j*6+2+offset] = robotPCL_qa0->z[(*p)[i][j].pcl];
			buffer[i*pitch+j*6+3+offset] = pos->roll[(*p)[i][j].angle];
			buffer[i*pitch+j*6+4+offset] = pos->pitch[(*p)[i][j].angle];
			buffer[i*pitch+j*6+5+offset] = pos->yaw[(*p)[i][j].angle];
		}
		buffer[i*pitch+nofCams*6+offset] = (*p)[i][0].c;
	}

	std::string fn = "minCameraPos" + to_string((_Longlong)nofCams) + ".bin";
	ofstream outbin(fn, ofstream::binary );
	outbin.write((char*)buffer, size*sizeof(double));
	outbin.close();
	delete buffer;
}

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

	CudaMem::cudaMemCpyReport(pcl->d_h, pcl->h, pcl->n*NUMELEM_H*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pcl->d_i, pcl->i, pcl->n*sizeof(int), cudaMemcpyHostToDevice);



}
void IO::loadSampleRotations(struct SAMPLE_ROTATIONS* rot, const char* name)
{
	ifstream inbin;
	inbin.open(name, ifstream::binary);
	if(!inbin.is_open()) std::cerr << "error";

	inbin.read((char*)&rot->nRoll, sizeof(int));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)&rot->nPitch, sizeof(int));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)&rot->nYaw, sizeof(int));
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

	inbin.read((char*)pcl->bb_H, pcl->nBB*16*sizeof(int));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)pcl->bb_D, pcl->nBB*3*sizeof(int));
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

	
	inbin.read((char*)&pos->nP, sizeof(int));
	if (!inbin) std::cerr << "error";
	 
	pos->qr = new float[pos->nP*N_ELEMENT_T *NUMELEM_H];
	pos->qh = new float[pos->nP*N_ELEMENT_HU*NUMELEM_H];
	pos->qe = new float[pos->nP*N_ELEMENT_EV*NUMELEM_H];
	pos->pr = new float[pos->nP];

	CudaMem::cudaMemAllocReport((void**)&pos->d_qr, pos->nP*N_ELEMENT_T *NUMELEM_H*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&pos->d_qh, pos->nP*N_ELEMENT_HU*NUMELEM_H*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&pos->d_qe, pos->nP*N_ELEMENT_EV*NUMELEM_H*sizeof(float));	

	inbin.read((char*)pos->qr,pos->nP*N_ELEMENT_T *NUMELEM_H*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)pos->qh, pos->nP*N_ELEMENT_HU*NUMELEM_H*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)pos->qe,pos->nP*N_ELEMENT_EV*NUMELEM_H*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)pos->pr,pos->nP*sizeof(float));
	if (!inbin) std::cerr << "error";

	char c;
	inbin.get(c);

	if(!inbin.eof()) std::cerr << "error";

	inbin.close();

	CudaMem::cudaMemCpyReport(pos->d_qr, pos->qr, pos->nP*N_ELEMENT_T *NUMELEM_H*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pos->d_qh, pos->qh, pos->nP*N_ELEMENT_HU*NUMELEM_H*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(pos->d_qe, pos->qe, pos->nP*N_ELEMENT_EV*NUMELEM_H*sizeof(float), cudaMemcpyHostToDevice);
	

}

void IO::loadSampleCamera(struct SAMPLE_CAMERA* cam, const char* name)
{
	ifstream inbin;
	inbin.open(name, ifstream::binary);
	if(!inbin.is_open()) std::cerr << "error";
	
	inbin.read((char*)&cam->nRays, sizeof(int));
	if (!inbin) std::cerr << "error";
	 
	cam->x = new float[cam->nRays];
	cam->y = new float[cam->nRays];
	cam->z = new float[cam->nRays];
	
	CudaMem::cudaMemAllocReport((void**)&cam->d_x, cam->nRays*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&cam->d_y, cam->nRays*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&cam->d_z, cam->nRays*sizeof(float));	

	inbin.read((char*)cam->x,cam->nRays*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)cam->y, cam->nRays*sizeof(float));
	if (!inbin) std::cerr << "error";

	inbin.read((char*)cam->z, cam->nRays*sizeof(float));
	if (!inbin) std::cerr << "error";

	char c;
	inbin.get(c);

	if(!inbin.eof()) std::cerr << "error";
	inbin.close();

	CudaMem::cudaMemCpyReport(cam->d_x, cam->x, cam->nRays*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(cam->d_y, cam->y, cam->nRays*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(cam->d_z, cam->z, cam->nRays*sizeof(float), cudaMemcpyHostToDevice);
}

void IO::saveDepthBufferToFile(struct DEPTH_BUFFER* depth, const char* name)
{
	CudaMem::cudaMemCpyReport(depth->dx, depth->d_dx, depth->size*sizeof(float), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(depth->dy, depth->d_dy, depth->size*sizeof(float), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(depth->dz, depth->d_dz, depth->size*sizeof(float), cudaMemcpyDeviceToHost);

	ofstream outbin(name, ofstream::binary );
	if (!outbin) std::cerr << "error";

	outbin.write((char*)depth->dx, depth->size*sizeof(float));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)depth->dy, depth->size*sizeof(float));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)depth->dz, depth->size*sizeof(float));
	if (!outbin) std::cerr << "error";

	outbin.close();

	//check avg value
	float vx = 0.0f;
	float vy = 0.0f;
	float vz = 0.0f;
	int w = 0;
	for(int i=0; i<depth->size; i++)
	{
		if(depth->dx[i] < INF_DIST)
		{
			vx += depth->dx[i];
			vy += depth->dy[i];
			vz += depth->dz[i];
			w++;
		}
	}
	vx = vx/w;
	vy = vy/w;
	vz = vz/w;
	printf("centroid: [%.6f  %.6f  %.6f]\n", vx, vy, vz);
}

void IO::saveVerticeBufferToFile(struct VERTEX_BUFFER* buffer, const char* name)
{
	float* buf = new float[3*buffer->nV];
	CudaMem::cudaMemCpyReport(buf+0*buffer->nV, buffer->d_vx, buffer->nV*sizeof(float), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(buf+1*buffer->nV, buffer->d_vy, buffer->nV*sizeof(float), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(buf+2*buffer->nV, buffer->d_vz, buffer->nV*sizeof(float), cudaMemcpyDeviceToHost);

	ofstream outbin(name, ofstream::binary );
	if (!outbin) std::cerr << "error";

	outbin.write((char*)&buffer->nV,sizeof(int));
	if (!outbin) std::cerr << "error";

	outbin.write((char*)buf,3*buffer->nV*sizeof(float));
	if (!outbin) std::cerr << "error";

	outbin.close();

	delete buf;
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
	CudaMem::cudaMemCpyReport(centroid->cx,centroid->d_cx, sizeof(float), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(centroid->cy,centroid->d_cy, sizeof(float), cudaMemcpyDeviceToHost);
	CudaMem::cudaMemCpyReport(centroid->cz,centroid->d_cz, sizeof(float), cudaMemcpyDeviceToHost);
	printf("centroid: [%.6f  %.6f  %.6f]\n", centroid->cx[0], centroid->cy[0], centroid->cz[0]);

}