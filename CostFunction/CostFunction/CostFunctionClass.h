/*
 * CostFunctionClass.h
 *
 *  Created on: 13.06.2014
 *      Author: tsdf
 */





#ifndef COSTFUNCTIONCLASS_H_
#define COSTFUNCTIONCLASS_H_


#include "cuda_runtime.h"
#include "struct_definitions.h"
#include "SimulatedAnnealing.h"
#include "time.h"
#include <iostream>
#include <string>
#include "global.h"


class CostFunctionClass {
public:
	CostFunctionClass();
	virtual ~CostFunctionClass();

	struct OPTIMISTATION_LAUNCH{
		unsigned int*	d_robotoccupancy;
		unsigned int*	d_humanoccupancy;

		unsigned int*	d_fa;
		unsigned int*	d_hs;

		float*			d_c;
		float*			d_mi;
		float*			d_cp;

		float*			d_ksdf;
		double*			d_costs;
		float*			d_fa_costs;
		float*			d_hs_costs;
		float*			d_hdm;
		float*			d_edm;

		unsigned int* d_fa_final;
		unsigned int* d_hs_final;
		cudaStream_t 	cudaStreams;
	};


	struct OPTIMISTATION_DATA{
		struct OPTIMISTATION_LAUNCH allData;
		struct OPTIMISTATION_LAUNCH launchConfigurations[PAR_KERNEL_LAUNCHS];

		struct DH_transformations s_mmkr16;
		struct DH_transformations s_mmkr16_q0;
		struct H_transformations s_human;


		unsigned int*	d_robotoccupancy_all;
		unsigned int*	d_humanoccupancy_all;
		


		double*			h_costs;
		double*			h_costs_buffer;

#ifdef DEBUG_RECORD
		double*			h_costs_result;
#endif
		float* 			d_h_camera;

		int* 			h_pcl_index;
		int* 			d_pcl_index;
		int* 			h_angle_index;
		int* 			d_angle_index;


		double maxIteration;
		int 			currentNofValidIterations;
		int*			nearesNeighbourIndex;

		unsigned int*			d_hasCollision;

		float* 			d_ws_x;
		float* 			d_ws_y;
		float* 			d_ws_z;
		
		











	};


private:
	cudaError_t cudaStatus;
	int cudaDevice;
	unsigned int nx;
	unsigned int ny;
	unsigned int nz;
	unsigned int N;
	unsigned int N_char;
	unsigned int N_int;
	float xmin;
	float xmax;
	float ymin;
	float ymax;
	float zmin;
	float zmax;
	float* d_eye;
	SimulatedAnnealing* sA;
	bool *isValidPosition;
	int				nOfCams;
	unsigned long long* comp_vec;
	int gen_result;
	int numberOfIteration;


	std::vector<struct COST_POINT* > currentMultiCameraCosts;


	double globalMin;



	struct DH_parameter  s_mmkr16;
	struct DH_parameter  s_mmkr16_q0;
	struct PCL robotpcl_0;
	struct PCL robotpcl;
	struct PCL robotPCL_qa0;
	struct PCL humanpcl_0;
	struct PCL humanpcl;
	struct ROBOT_POSITION robotPos;
	struct HUMAN_POSITION humanPos;
	struct POSITIONS pos;
	struct OPTIMISTATION_DATA opt_data;
	struct CAM cam;


protected:
	void init_mmkr16(void);
	void init_mmkr16_q0(void);
	static void initCam(struct CAM* cam);
	void initCameraPositions(struct POSITIONS* pos);
	void adjustRobotTransformation(float* qvin);
	void transformHumanIntoRobotSystem(float* qr, float* qh, float* robotToHuman);
	void initWSCoordinates(void);

	void calculateVelocity(float* qdin);
	void adjustrobotpclandvelocity(struct PCL* dst, float vxp, float vyp);
	void assignRobotPclIntoWS_shared(struct PCL* pcl, unsigned int* const R);
	void assignHumanPclIntoWS_shared(struct PCL* pcl, unsigned int* const H, float* h);
	void printProgress(double ci, double ei, time_t start, const char* task);
	void getCameraPosition(int id, float roll, float pitch, float yaw, struct PCL* pcl, float* h);
	static void updateCamera(struct CAM* cam, float *h);
	void calculateKSDF(struct PCL* dst, struct OCCUPANCY_GRIDS* grid);
	void adjustrobotpcl(struct PCL* dst);
	void checkMemoryUsage(void);
	void setRobotOccupancyGrid(int posIndex, bool online);
	void setHumanOccupancyGrid(int humanPosIndex, int robotPosIndex, bool online);
	bool generatePCLandAngleIndex(void);
	void adjustCameraParameters(int index);
	void initDHTransformations(void);
	void initHTransformations(void);
	void adjustrobotpclandvelocity_memory(struct PCL* dst,int i,  float vxp, float vyp);
	void assignHumanPclIntoWS_memory(struct PCL* pcl, unsigned int* const H, int i);
	void calculateKSDF_memory(void);
	void calcNextCandidates(void);
	void initCostArray(void);
	void assignHumanAndRobotOccupancyGrids(void);
	
	bool doesHumanCollideWithRobot(void);
	double maxNumberOfIteration(unsigned int long long n, unsigned int long long k);
	void initHSandFAFinal(void);
	void calculateCosts(void);
	void checkMinimumCosts(void);
	void clearMinCosts(void);


	static double stirling_approximation(double n);






public:
	void initAllKSDF(bool writeToFile);
	void init_costfunction(bool savetofile);
	void allocOptimisationMemory(void);
	void optimize_single(int c);
	void optimize_all_memory(void);

	static void testCudaFunctions();
	static void testCudaInverse();
	static void testUpdateCameraParameters();
	void testSelectNN(void);
	void assignNewCamera(void);



};

#endif /* COSTFUNCTIONCLASS_H_ */
