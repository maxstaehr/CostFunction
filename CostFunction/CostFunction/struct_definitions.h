
#ifndef STRUCT_DEFINITIONS_H
#define STRUCT_DEFINITIONS_H

#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <curand_kernel.h>
#include <vector>
#include <set>

#define ROBOT_PCL_NUMBER_CUDA 4368
#define NUMELEM_H 16



enum STATE{
	HC,
	NS,
	OR
};

struct SOLUTION{
	int* pcl;
	int* angle;
	float costs;
	float currProp;
	float curT;
	int ite;
	float minEnergy;
	enum STATE state;	
	float globalMin;
};








struct PCL
{
	float* x;
	float* y;
	float* z;
	float* v_x;
	float* v_y;
	float* v_z;
	float* h;
	int *i;
	int *index;

	float* d_x;
	float* d_y;
	float* d_z;
	float* d_v_x;
	float* d_v_y;
	float* d_v_z;
	float* d_h;
	int *d_i;
	int *d_index;

	std::set<int> indices;
	unsigned int n;
};

struct DEPTH_BUFFER{
	int size;
	int sssize;

	float* dx;
	float* dy;
	float* dz;
	float* d_dx;
	float* d_dy;
	float* d_dz;

	float* d_ss_dx;
	float* d_ss_dy;
	float* d_ss_dz;

	curandState *devStates;

};

struct SAMPLE_FITTING{
	int n;

	float* R;
	float* Fx;
	float* Fy;

	float* d_R;
	float* d_Fx;
	float* d_Fy;
};

struct LAUNCH_CONFIG{
	int nblocks;
	int nthreads;
};

struct PROB_RESULT{
	int n;
	int nmax;

	float* p;
	float* maxp;

	float* d_p;
	float* d_maxp;

	float* d;
	float* maxd;

	float* d_d;
	float* d_maxd;

	int* w;
	int* maxw;

	int* d_w;
	int* d_maxw;
	
};



struct ROBOT_PCL
{
	int nV;
	int nF;
	int nBB;

	float*	x;
	float*	y;
	float*	z;
	int*	vi;

	int*	fx;
	int*	fy;
	int*	fz;
	int*	fi;

	float*	bb_H;
	float*	bb_D;
	int*	bbi;

	float*	d_x;
	float*	d_y;
	float*	d_z;
	int*	d_vi;	

	int*	d_fx;
	int*	d_fy;
	int*	d_fz;
	int*	d_fi;

	float*	d_bb_H;
	float*	d_bb_D;
	int*	d_bbi;
};

struct ENVIRONMENT_PCL
{
	int nV;
	int nF;
	int nBB;

	float*	x;
	float*	y;
	float*	z;
	int*	fx;
	int*	fy;
	int*	fz;
	float*	bb_H;
	float*	bb_D;
	

	float*	d_x;
	float*	d_y;
	float*	d_z;
	int*	d_fx;
	int*	d_fy;
	int*	d_fz;
	float*	d_bb_H;
	float*	d_bb_D;
	
};

struct HUMAN_PCL
{
	int nV;
	int nF;
	
	float*	x;
	float*	y;
	float*	z;
	

	int*	fx;
	int*	fy;
	int*	fz;
	


	float*	d_x;
	float*	d_y;
	float*	d_z;
	

	int*	d_fx;
	int*	d_fy;
	int*	d_fz;
	

};

struct SAMPLE_PCL
{
	int n;

	float* x;
	float* y;
	float* z;
	
	float* h;
	int *i;

	float* d_h;
	int* d_i;
};

struct SAMPLE_ROTATIONS
{
	int nRoll;
	int nPitch;
	int nYaw;
	int nRotations;

	float* R;
	float* d_R;

	float* angleLimits;
};

struct SAMPLE_POSITIONS
{
	int nP;
	int nS;

	float* qr;
	float* qh;
	float* qe;
	float* pr;

	float* d_qr;
	float* d_qh;
	float* d_qe;
	float* d_pr;
};

struct SAMPLE_CAMERA
{
	int nBlocks;
	int nThreads;
	int nRays;

	int ssnBlocks;
	int ssnThreads;
	int ssnRays;


	int nx;
	int ny;
	int ss_x;
	int ss_y;
	int minW;


	float* x;
	float* y;
	float* z;

	float* ssx;
	float* ssy;
	float* ssz;

	float* c;
	float* d;
	

	float* d_x;
	float* d_y;
	float* d_z;

	float* d_ss_x;
	float* d_ss_y;
	float* d_ss_z;


	float* d_c;
	float* d_d;

	int* d_angleIndex;
	int* d_positionIndex;
};

struct CAM_COMBINATION{
	unsigned long long n;
	unsigned long long k;
	unsigned long long* vector;
	int gen_result;
};

struct POSSIBLE_CAMERA_TYPES
{
	int nCameraTypes;
	struct SAMPLE_CAMERA* possibleCameraTypes;
};
struct CENTROID{

	float* cx;
	float* cy;
	float* cz;

	float* d_cx;
	float* d_cy;
	float* d_cz;

};
struct RAYTRACING_LAUNCH{
	int						n;	
	SAMPLE_CAMERA**	cams;
	cudaStream_t**	cudaStream;
	
	
	DEPTH_BUFFER*	depthBuffers;

	//for all camera within one launch
	DEPTH_BUFFER	depthBuffer;
	CENTROID		centroid;
	PROB_RESULT		probResult;

	int**	aI;
	int**	pI;
};

struct OPTIMIZATION_SESSION
{
	int n;
	struct RAYTRACING_LAUNCH* launchs;

	int*	aI;
	int*	pI;
		
};

struct DISTANCE_MATRIX{
	int n;
	float*		ws_l_params;
	int*		ws_n_params;
	float*		d;

	float*		d_ws_l_params;
	int*		d_ws_n_params;
	float*		d_d;

};




struct CURRENT_TRANS{
	float* d_r;
	float* d_h;
	float* d_e;
	float* d_pr;
};


struct CAM
{
	float f;
	float x;
	float y;
	float z;
	int nu;
	int nv;

	float rho[2];
	float pp[2];
	float C0[12];
	float C[12];
	float Mi[9];

	float* EDM;
	float* HDM;


	float* d_C;
	float* d_C0;
	float* d_Mi;
	float* d_EDM;
	float* d_HDM;


};

struct HUMAN_INPUT
{
	unsigned int npos;
	float* q;
	PCL pcl;	
};

struct ROBOT_INPUT
{
	float* q;
	float* qd;
	unsigned int npos;
	PCL pcl;
	unsigned int *linkIndex;
	unsigned int njoints;
	float* initialLink;
	float* currentLinkTrans;
};

struct ENVIRONMENT_INPUT{
	PCL pcl;
};

struct OCCUPANCY_GRIDS{
	unsigned char* R;
	unsigned char* H;
	unsigned char* E;
	unsigned char* HS;
	unsigned char* HV;
	unsigned char* FA;
	float* x;
	float* y;
	float* z;
	float* ksdf;
	float* accum_hs;

	unsigned char* d_R;
	unsigned char* d_H;
	unsigned char* d_E;
	unsigned char* d_HS;
	unsigned char* d_HV;
	unsigned char* d_FA;
	float* d_x;
	float* d_y;
	float* d_z;
	float* d_ksdf;
	float* d_buffer1;
	float* d_buffer2;
	float* d_accum_hs;

	int N;
	float xmin;
	float xmax;
	float ymin;
	float ymax;
	float zmin;
	float zmax;
	float xstep;
	float ystep;
	float zstep;
	float minDist;
	int nx;
	int ny;
	int nz;

};

struct DH_parameter{
	float* theta;
	float* d;
	float* a;
	float* alpha;
	float* sigma;
	float* offset;
	float* base;
	float* T;
	float* v_x;
	float* v_y;
	float* v_z;
	float* w_x;
	float* w_y;
	float* w_z;

	float* d_T;
	float* d_v_x;
	float* d_v_y;
	float* d_v_z;
	float* d_w_x;
	float* d_w_y;
	float* d_w_z;
};

struct DH_transformations{
	float* d_T;
	float* d_v_x;
	float* d_v_y;
	float* d_v_z;
	float* d_w_x;
	float* d_w_y;
	float* d_w_z;
};

struct H_transformations{
	float* d_h;
};


 struct POSITIONS{
	 unsigned int nOfAngles;
	 float* roll;
	 float* pitch;
	 float* yaw;

	 //float* d_yaw;
	 //float* d_roll;
	 //float* d_pitch;

 };


  struct COST_RESULTS{
	 unsigned int nOfCosts;
	 int * posID;
	 float* x;
	 float* y;
	 float* z;
	 float* roll;
	 float* pitch;
	 float* yaw;
	 double* costs;

 };

  struct ROBOT_POSITION
  {
	 unsigned int  n;
	 float* positions;
	 float* velocities;
	 float** ksdf;

	 float* d_positions;
	 float* d_velocity;
 };

  struct HUMAN_POSITION
  {
	 unsigned int  n;
	 float* positions;

	 float* d_positions;

 };

  struct SINGLE_ROBOT_HUMAN_CAMERA_COST
  {
	  unsigned int robotIndex;
	  unsigned int humanIndex;
	  bool isValid;
	  bool humanIsInWorkspace;
	  double maximumCosts;
	  std::vector<int> indices;
	  std::vector<int> indices_fa;
  };

  struct SINGLE_CAMERA_POS_COSTS
  {
	  unsigned int nOfAllHumanRobotCosts;
	  struct SINGLE_ROBOT_HUMAN_CAMERA_COST* allHumanRobotCosts;
	  unsigned int PCLindex;
	  unsigned int AngleIndex;
  };

  struct MINIMUM_COSTS
  {
	  int nOfCameras;
	  int* robotPCLIndices;
	  int* angleIndices;
	  double minimumCosts;
  };




  struct VERTEX_BUFFER{
	  int nF;
	  int nV;

	  float* d_vx;
	  float* d_vy;
	  float* d_vz;

	  int* d_fx;
	  int* d_fy;
	  int* d_fz;

  };

  struct BB_BUFFER{
	  int nBB;
	  float* d_BB;
	  float* d_D;
  };

  struct SAMPLE_POINTS_BUFFER{
	  int n;
	  float* d_H;
  };




#endif
