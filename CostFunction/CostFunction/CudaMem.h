/*
 * CudaMem.h
 *
 *  Created on: 18.06.2014
 *      Author: tsdf
 */

#ifndef CUDAMEM_H_
#define CUDAMEM_H_

#include "struct_definitions.h"
#include "CostFunctionClass.h"
#define N_ELEMENT_T 10
#define N_ELEMENT_HU 1
#define N_ELEMENT_EV 1


class CudaMem {
public:
	CudaMem();
	virtual ~CudaMem();


	static void cudaMemAllocReport(void ** 	devPtr,size_t 	size);
	static void cudaMemsetReport(void *devPtr, int value, size_t count);
	static void cudaMemCpyAsyncReport(void* dst, const void * 	src, size_t 	count, enum cudaMemcpyKind 	kind, cudaStream_t stream);
	static void cudaMemsetAsyncReport(void *devPtr, int value, size_t count);
	static void cudaMemCpyReport(void* dst, const void * 	src, size_t 	count, enum cudaMemcpyKind 	kind);
	static void cudaMemCpyToSymbolReport(void* dst, const void * 	src, size_t 	count, enum cudaMemcpyKind 	kind);
	static void cudaFreeReport(void* dst);

	static void allocPCLN(struct PCL* pcl);
	static void deletePCL(struct PCL* pcl);

	static void allocDHTransformations(struct DH_transformations* s, int n);
	static void deleteDHTransformations(struct DH_transformations* s);

	static void allocHTransformations(struct H_transformations* s, int n);

	static void allocRobotPositions(struct ROBOT_POSITION* robotPoses);
	static void deleteRobotPositions(struct ROBOT_POSITION* robotPoses);

	static void allocHumanPositions(struct HUMAN_POSITION* humanPositions);
	static void deleteHumanPositions(struct HUMAN_POSITION* humanPositions);

	static void copyPCLDeviceToDevice(const struct PCL* const in, const struct  PCL* out);
	static void copyDHParameterToDevice(struct DH_parameter* dh);

	static void cpyPCL(struct PCL* dst, const struct PCL* src);
	static void copyPCLDeviceToHost(struct PCL* pcl);
	static void copyPCLHostToDevice(struct PCL* pcl);
	static void allocGrid(struct OCCUPANCY_GRIDS* grid,
			float xmin, float xmax, float ymin, float ymax, float zmin, float zmax,
			unsigned int nx, unsigned int ny, unsigned int nz);
	static void deleteGrid(struct OCCUPANCY_GRIDS* grid);
	static void copyGridDeviceToHost(struct OCCUPANCY_GRIDS* grid);
	static void copyGridHostToDevice(struct OCCUPANCY_GRIDS* grid);


};

#endif /* CUDAMEM_H_ */
