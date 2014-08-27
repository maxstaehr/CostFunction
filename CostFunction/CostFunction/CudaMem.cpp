/*
 * CudaMem.cpp
 *
 *  Created on: 18.06.2014
 *      Author: tsdf
 */

#include "CudaMem.h"
#include "device_launch_parameters.h"
#include <string.h>
#include <cstdio>
#include <cstdlib>


#define EYE {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f}

CudaMem::CudaMem() {
	// TODO Auto-generated constructor stub

}

CudaMem::~CudaMem() {
	// TODO Auto-generated destructor stub
}



void CudaMem::cpyPCL(struct PCL* dst, const struct PCL* src)
{
	memcpy(dst->x, src->x, src->n*sizeof(float));
	memcpy(dst->y, src->y, src->n*sizeof(float));
	memcpy(dst->z, src->z, src->n*sizeof(float));
	memcpy(dst->i, src->i, src->n*sizeof(int));

}

void CudaMem::allocPCLN(struct PCL* pcl)
{
	pcl->x = new float[pcl->n];
	pcl->y = new float[pcl->n];
	pcl->z = new float[pcl->n];
	pcl->v_x = new float[pcl->n];
	pcl->v_y = new float[pcl->n];
	pcl->v_z = new float[pcl->n];
	pcl->h = new float[16];
	pcl->i = new int[pcl->n];



	memset(pcl->x, 0, pcl->n*sizeof(float));
	memset(pcl->y, 0, pcl->n*sizeof(float));
	memset(pcl->z, 0, pcl->n*sizeof(float));

	memset(pcl->v_x, 0, pcl->n*sizeof(float));
	memset(pcl->v_y, 0, pcl->n*sizeof(float));
	memset(pcl->v_z, 0, pcl->n*sizeof(float));

	memset(pcl->i, 0, pcl->n*sizeof(int));
	float eye[] = EYE;
	memcpy(pcl->h, eye, 16*sizeof(float));


	cudaMemAllocReport((void**)&pcl->d_x, pcl->n * sizeof(float));
	cudaMemAllocReport((void**)&pcl->d_y, pcl->n * sizeof(float));
	cudaMemAllocReport((void**)&pcl->d_z, pcl->n * sizeof(float));

	cudaMemAllocReport((void**)&pcl->d_v_x, pcl->n * sizeof(float));
	cudaMemAllocReport((void**)&pcl->d_v_y, pcl->n * sizeof(float));
	cudaMemAllocReport((void**)&pcl->d_v_z, pcl->n * sizeof(float));

	cudaMemAllocReport((void**)&pcl->d_h, 16 * sizeof(float));
	cudaMemAllocReport((void**)&pcl->d_i, pcl->n * sizeof(int));
	cudaMemAllocReport((void**)&pcl->d_index, pcl->n * sizeof(int));

}


void CudaMem::deletePCL(struct PCL* pcl)
{
	delete [] pcl->x;
	delete [] pcl->y;
	delete [] pcl->z;
	delete [] pcl->v_x;
	delete [] pcl->v_y;
	delete [] pcl->v_z;
	delete [] pcl->h;
	delete [] pcl->i;



	cudaFree(pcl->d_x);
	cudaFree(pcl->d_y);
	cudaFree(pcl->d_z);

	cudaFree(pcl->d_v_x);
	cudaFree(pcl->d_v_y);
	cudaFree(pcl->d_v_z);

	cudaFree(pcl->d_h);
	cudaFree(pcl->d_i);
	cudaFree(pcl->d_index);

}

void CudaMem::cudaMemAllocReport(void ** 	devPtr,size_t 	size)
{
	cudaError_t cudaStatus;
	cudaStatus = cudaMalloc(devPtr, size);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "error cudaMemAlloc");

	}
}


void CudaMem::allocRobotPositions(struct ROBOT_POSITION* robotPoses)
{
	cudaMemAllocReport((void**)&robotPoses->d_positions, robotPoses->n*sizeof(float)*9);
	cudaMemAllocReport((void**)&robotPoses->d_velocity, robotPoses->n*sizeof(float)*9);
}

void CudaMem::deleteRobotPositions(struct ROBOT_POSITION* robotPoses)
{
	cudaFree(robotPoses->d_positions);
	cudaFree(robotPoses->d_velocity);
}

void CudaMem::allocHumanPositions(struct HUMAN_POSITION* humanPoses)
{
	cudaMemAllocReport((void**)&humanPoses->d_positions, humanPoses->n*sizeof(float)*3);
}

void CudaMem::deleteHumanPositions(struct HUMAN_POSITION* humanPoses)
{
	cudaFree(humanPoses->d_positions);
}

void CudaMem::copyPCLHostToDevice(struct PCL* pcl)
{
	cudaError_t cudaStatus;
	cudaStatus = cudaMemcpy(pcl->d_x,pcl->x, pcl->n*sizeof(float), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(pcl->d_y,pcl->y, pcl->n*sizeof(float), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(pcl->d_z,pcl->z, pcl->n*sizeof(float), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(pcl->d_v_x,pcl->v_x, pcl->n*sizeof(float), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(pcl->d_v_y,pcl->v_y, pcl->n*sizeof(float), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(pcl->d_v_z,pcl->v_z, pcl->n*sizeof(float), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(pcl->d_h,pcl->h, 16*sizeof(float), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(pcl->d_i,pcl->i, pcl->n*sizeof(int), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}

}


void CudaMem::copyPCLDeviceToDevice(const struct PCL* const in, const struct  PCL* out)
{
	cudaError_t cudaStatus;
	cudaStatus = cudaMemcpy(out->d_x,in->d_x, in->n*sizeof(float), cudaMemcpyDeviceToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(out->d_y,in->d_y, in->n*sizeof(float), cudaMemcpyDeviceToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(out->d_z,in->d_z, in->n*sizeof(float), cudaMemcpyDeviceToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(out->d_v_x,in->d_v_x, in->n*sizeof(float), cudaMemcpyDeviceToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(out->d_v_y,in->d_v_y, in->n*sizeof(float), cudaMemcpyDeviceToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(out->d_v_z,in->d_v_z, in->n*sizeof(float), cudaMemcpyDeviceToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(out->d_h,in->d_h, 16*sizeof(float), cudaMemcpyDeviceToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(out->d_i,in->d_i, in->n*sizeof(int), cudaMemcpyDeviceToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}

}

void CudaMem::copyPCLDeviceToHost(struct PCL* pcl)
{
	cudaError_t cudaStatus;
	cudaStatus = cudaMemcpy(pcl->x,pcl->d_x, pcl->n*sizeof(float), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(pcl->y,pcl->d_y, pcl->n*sizeof(float), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(pcl->z,pcl->d_z, pcl->n*sizeof(float), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(pcl->v_x,pcl->d_v_x, pcl->n*sizeof(float), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(pcl->v_y,pcl->d_v_y, pcl->n*sizeof(float), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(pcl->v_z,pcl->d_v_z, pcl->n*sizeof(float), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(pcl->h,pcl->d_h, 16*sizeof(float), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(pcl->i,pcl->d_i, pcl->n*sizeof(int), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}

}

void CudaMem::allocGrid(struct OCCUPANCY_GRIDS* grid, float xmin, float xmax, float ymin, float ymax, float zmin, float zmax,
	unsigned int nx, unsigned int ny, unsigned int nz)
{
	//alloc host memory
	grid->N = nx * ny * nz;
	grid->E = new unsigned char[grid->N];
	grid->R = new unsigned char[grid->N];
	grid->H = new unsigned char[grid->N];
	grid->HS = new  unsigned char[grid->N];
	grid->HV = new unsigned char[grid->N];
	grid->FA = new unsigned char[grid->N];

	grid->x = new float[grid->N];
	grid->y = new float[grid->N];
	grid->z = new float[grid->N];
	grid->ksdf = new float[grid->N];
	grid->accum_hs = new float[1];

	unsigned int id = 0;
	float dx = (xmax-xmin)/(nx-1);
	float dy = (ymax-ymin)/(ny-1);
	float dz = (zmax-zmin)/(nz-1);
	float x = 0.0;
	float y = 0.0;
	float z = 0.0;
	grid->minDist = (float)1.7f* abs((xmax - xmin)/nx);
	for(unsigned int zi=0; zi<nz; zi++){
		for(unsigned int xi=0; xi<nx; xi++){
			for(unsigned int yi=0; yi<ny; yi++){
				id = zi*nx*ny + xi*ny + yi;
				x = dx * xi + xmin;
				y = dy * yi + ymin;
				z = dz * zi + zmin;
				grid->x[id] = x;
				grid->y[id] = y;
				grid->z[id] = z;
				grid->R[id] = 0;
				grid->E[id] = 0;
				grid->H[id] = 0;
				grid->HS[id] = 0;
			}
		}
	}
	grid->xmin = xmin;
	grid->xmax = xmax;
	grid->ymin = ymin;
	grid->ymax = ymax;
	grid->zmin = zmin;
	grid->zmax = zmax;
	grid->xstep = dx;
	grid->ystep = dy;
	grid->zstep = dz;
	grid->nx = nx;
	grid->ny = ny;
	grid->nz = nz;

	cudaMalloc((void**)&grid->d_x, grid->N * sizeof(float));
	cudaMalloc((void**)&grid->d_y, grid->N * sizeof(float));
	cudaMalloc((void**)&grid->d_z, grid->N * sizeof(float));
	cudaMalloc((void**)&grid->d_ksdf, grid->N * sizeof(float));
	cudaMalloc((void**)&grid->d_buffer1, grid->N * sizeof(float));
	cudaMalloc((void**)&grid->d_buffer2, grid->N * sizeof(float));

	cudaMalloc((void**)&grid->d_E, grid->N * sizeof(unsigned char));
	cudaMalloc((void**)&grid->d_R, grid->N * sizeof(unsigned char));
	cudaMalloc((void**)&grid->d_H, grid->N * sizeof(unsigned char));
	cudaMalloc((void**)&grid->d_HS, grid->N * sizeof(unsigned char));
	cudaMalloc((void**)&grid->d_HV, grid->N * sizeof(unsigned char));
	cudaMalloc((void**)&grid->d_FA, grid->N * sizeof(unsigned char));
	cudaMalloc((void**)&grid->d_accum_hs, sizeof(float));

	cudaMemcpy(grid->d_x, grid->x, grid->N * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(grid->d_y, grid->y, grid->N * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(grid->d_z, grid->z, grid->N * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(grid->d_ksdf, grid->ksdf, grid->N * sizeof(float), cudaMemcpyHostToDevice);

	cudaMemcpy(grid->d_E, grid->E, grid->N * sizeof(unsigned char), cudaMemcpyHostToDevice);
	cudaMemcpy(grid->d_R, grid->R, grid->N * sizeof(unsigned char), cudaMemcpyHostToDevice);
	cudaMemcpy(grid->d_H, grid->H, grid->N * sizeof(unsigned char), cudaMemcpyHostToDevice);
	cudaMemcpy(grid->d_HS, grid->HS, grid->N * sizeof(unsigned char), cudaMemcpyHostToDevice);


}


void CudaMem::deleteGrid(struct OCCUPANCY_GRIDS* grid)
{

	delete [] grid->E;
	delete [] grid->R;
	delete [] grid->H;
	delete [] grid->HS;
	delete [] grid->HV;
	delete [] grid->FA;

	delete [] grid->x;
	delete [] grid->y;
	delete [] grid->z;
	delete [] grid->ksdf;
	delete [] grid->accum_hs;



	cudaFree(grid->d_x);
	cudaFree(grid->d_y);
	cudaFree(grid->d_z);
	cudaFree(grid->d_ksdf);
	cudaFree(grid->d_buffer1);
	cudaFree(grid->d_buffer2);

	cudaFree(grid->d_E);
	cudaFree(grid->d_R);
	cudaFree(grid->d_H);
	cudaFree(grid->d_HS);
	cudaFree(grid->d_HV);
	cudaFree(grid->d_FA);
	cudaFree(grid->d_accum_hs);


}

void CudaMem::copyGridHostToDevice(struct OCCUPANCY_GRIDS* grid)
{
	cudaError_t cudaStatus;
	cudaStatus = cudaMemcpy(grid->d_x, grid->x, grid->N * sizeof(float), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(grid->d_y, grid->y, grid->N * sizeof(float), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(grid->d_z, grid->z, grid->N * sizeof(float), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(grid->d_ksdf, grid->ksdf, grid->N * sizeof(float), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}

	cudaStatus = cudaMemcpy(grid->d_E, grid->E, grid->N * sizeof(unsigned char), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(grid->d_R, grid->R, grid->N * sizeof(unsigned char), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(grid->d_H, grid->H, grid->N * sizeof(unsigned char), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(grid->d_HS, grid->HS, grid->N * sizeof(unsigned char), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(grid->d_HV, grid->HV, grid->N * sizeof(unsigned char), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}

	cudaStatus = cudaMemcpy(grid->d_FA, grid->FA, grid->N * sizeof(unsigned char), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}


}

void CudaMem::copyGridDeviceToHost(struct OCCUPANCY_GRIDS* grid)
{

	cudaError_t cudaStatus;

	cudaStatus = cudaMemcpy(grid->x, grid->d_x, grid->N * sizeof(float), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(grid->y, grid->d_y, grid->N * sizeof(float), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(grid->z, grid->d_z, grid->N * sizeof(float), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(grid->ksdf, grid->d_ksdf, grid->N * sizeof(float), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}

	cudaStatus = cudaMemcpy(grid->E, grid->d_E, grid->N * sizeof(unsigned char), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(grid->R, grid->d_R, grid->N * sizeof(unsigned char), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(grid->H, grid->d_H, grid->N * sizeof(unsigned char), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(grid->HS, grid->d_HS, grid->N * sizeof(unsigned char), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(grid->HV, grid->d_HV, grid->N * sizeof(unsigned char), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
	cudaStatus = cudaMemcpy(grid->FA, grid->d_FA, grid->N * sizeof(unsigned char), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}

}

void CudaMem::copyDHParameterToDevice(struct DH_parameter* dh)
{

	cudaError_t cudaStatus;
	cudaStatus = cudaMemcpy(dh->d_T, dh->T, NUMELEM_H * (N_ELEMENT_T) * sizeof(float), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}

	cudaStatus = cudaMemcpy(dh->d_v_x, dh->v_x, NUMELEM_H * (N_ELEMENT_T) * sizeof(float), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}

	cudaStatus = cudaMemcpy(dh->d_v_y, dh->v_y, NUMELEM_H * (N_ELEMENT_T) * sizeof(float), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}

	cudaStatus = cudaMemcpy(dh->d_v_z, dh->v_z, NUMELEM_H * (N_ELEMENT_T) * sizeof(float), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}

	cudaStatus = cudaMemcpy(dh->d_w_x, dh->w_x, NUMELEM_H * (N_ELEMENT_T) * sizeof(float), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}

	cudaStatus = cudaMemcpy(dh->d_w_y, dh->w_y, NUMELEM_H * (N_ELEMENT_T) * sizeof(float), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}

	cudaStatus = cudaMemcpy(dh->d_w_z, dh->w_z, NUMELEM_H * (N_ELEMENT_T) * sizeof(float), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudamemcpy error");

	}
}





void CudaMem::cudaMemCpyReport(void* dst, const void * 	src, size_t 	count, enum cudaMemcpyKind 	kind)
{
	cudaError_t cudaStatus;
	cudaStatus = cudaMemcpy(dst, src, count, kind);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr,cudaGetErrorString(cudaStatus));
	}

}

void CudaMem::cudaMemCpyToSymbolReport(void* dst, const void * 	src, size_t 	count, enum cudaMemcpyKind 	kind)
{
	cudaError_t cudaStatus;
	cudaStatus = cudaMemcpyToSymbol(dst, src, count, kind);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr,"error cudaMemCopy");
	}

}

void CudaMem::cudaMemsetReport(void *devPtr, int value, size_t count)
{
	cudaError_t cudaStatus;
	cudaStatus = cudaMemset(devPtr, value, count);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "memset launch failed: %s\n", cudaGetErrorString(cudaStatus));
		return;
	}

}

void CudaMem::cudaMemCpyAsyncReport(void* dst, const void * 	src, size_t 	count, enum cudaMemcpyKind 	kind)
{
	cudaError_t cudaStatus;
	cudaStatus = cudaMemcpy(dst, src, count, kind);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr,"error cudaMemCopy");
	}

}

void CudaMem::cudaFreeReport(void* dst)
{
	cudaError_t cudaStatus;
	cudaStatus = cudaFree(dst);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr,"error cudaFree");
	}
}

void CudaMem::cudaMemsetAsyncReport(void *devPtr, int value, size_t count)
{
	cudaError_t cudaStatus;
	cudaStatus = cudaMemset(devPtr, value, count);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "memset launch failed: %s\n", cudaGetErrorString(cudaStatus));
		return;
	}

}


void CudaMem::allocDHTransformations(struct DH_transformations* s, int n)
{
	cudaMemAllocReport((void**)&s->d_T, n * NUMELEM_H * (N_ELEMENT_T) * sizeof(float));
	cudaMemAllocReport((void**)&s->d_v_x,n *(N_ELEMENT_T) * sizeof(float));
	cudaMemAllocReport((void**)&s->d_v_y,n *(N_ELEMENT_T) * sizeof(float));
	cudaMemAllocReport((void**)&s->d_v_z,n *(N_ELEMENT_T) * sizeof(float));
	cudaMemAllocReport((void**)&s->d_w_x,n *(N_ELEMENT_T) * sizeof(float));
	cudaMemAllocReport((void**)&s->d_w_y,n *(N_ELEMENT_T) * sizeof(float));
	cudaMemAllocReport((void**)&s->d_w_z,n *(N_ELEMENT_T) * sizeof(float));
}
void CudaMem::deleteDHTransformations(struct DH_transformations* s)
{
	cudaFreeReport(s->d_T);
	cudaFreeReport(s->d_v_x);
	cudaFreeReport(s->d_v_y);
	cudaFreeReport(s->d_v_z);
	cudaFreeReport(s->d_w_x);
	cudaFreeReport(s->d_w_y);
	cudaFreeReport(s->d_w_z);

}

void CudaMem::allocHTransformations(struct H_transformations* s, int n)
{
	cudaMemAllocReport((void**)&s->d_h, n * NUMELEM_H  * sizeof(float));
}

