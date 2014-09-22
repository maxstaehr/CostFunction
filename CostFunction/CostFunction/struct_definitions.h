
#ifndef STRUCT_DEFINITIONS_H
#define STRUCT_DEFINITIONS_H

#include "cuda_runtime.h"
#include "device_launch_parameters.h"
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
	int pcl;
	int angle;
	double costs;
	double currProp;
	double curT;
	int ite;
	double minEnergy;
	enum STATE state;	
	double globalMin;
};

struct COST_POINT{
	int pcl;
	int angle;
	double c;
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


 




#endif
