#pragma once
#include "global.h"

#include <iostream>




class CFOBJECT_EXPORT CFIO {
public:



struct BB
{
	
	float H[16];
	float d[3];
};


struct PCL
{
	int nV;
	int nF;	
	int li;

	float*	x;
	float*	y;
	float*	z;	

	int*	fx;
	int*	fy;
	int*	fz;		

	struct CFIO::BB bb;		
};

struct LINK
{
	int n;
	struct PCL* pcls;
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
	int ssnx;
	int ssny;
	int minW;
	int rmax;

	float* x;
	float* y;
	float* z;

	float* ssx;
	float* ssy;
	float* ssz;

	float* c;
	float* d;
	
	CFIO::PCL pcl;
};

struct POSSIBLE_CAMERA_TYPES
{
	int nCameraTypes;
	struct CFIO::SAMPLE_CAMERA* possibleCameraTypes;
};

struct SAMPLE_POSITIONS
{
	int nP;
	int nS;

	float* qr;
	float* qh;
	float* qe;
	float* pr;


	double sumAllPriorities;
};

struct SAMPLE_CAMERA_POSITIONS
{
	int n;
	float* h;
	int* i;
};

struct SAMPLE_HUMAN_POSITIONS
{
	int n;
	float* R;
	float* x;
	float* y;
};






	CFIO();
	virtual ~CFIO();

	static void CFIO::loadCameraPos(struct SAMPLE_CAMERA_POSITIONS* pcl, const char* name);
	static void clearCameraPos(struct SAMPLE_CAMERA_POSITIONS* pcl);
	//new IO load Functions
	//static void loadSamplePCL(struct SAMPLE_PCL* pcl, const char* name);
	//static void loadSampleRotations(struct SAMPLE_ROTATIONS* rot, const char* name);
	static void loadPCL(struct CFIO::PCL* pcl, const char* name);
	static void clearPCL(struct CFIO::LINK* pcl);
	static void loadPCL(struct CFIO::PCL* pcl, std::ifstream* inbin);
	static void loadLink(struct CFIO::LINK* link, const char* name);
	static void loadBB(struct CFIO::BB* bb, std::ifstream* inbin);
	static void loadSamplePositions(struct CFIO::SAMPLE_POSITIONS* pso, const char* name);
	static void loadSampleCamera(struct CFIO::POSSIBLE_CAMERA_TYPES* cams, const char* name);
	static void loadSampleHumanPositions(struct CFIO::SAMPLE_HUMAN_POSITIONS* shp, const char* name);
	//static void saveDepthBufferToFile(struct DEPTH_BUFFER* depth, const char* name);
	//static void saveDepthBufferToFileSuperSamples(struct DEPTH_BUFFER* depth, const char* name);
	//static void saveVerticeBufferToFile(struct VERTEX_BUFFER* buffer, const char* name);
	//static void saveBoundingBoxBufferToFile(struct BB_BUFFER* buffer, const char* name);
	//static void printCentroid(struct CENTROID* centroid);
	//static void loadSampleFitting(struct SAMPLE_FITTING* sampleFitting, struct LAUNCH_CONFIG* config, const char* name);
	//static void saveProbResult2File(struct PROB_RESULT* probResult,  const char* name);
	//static void saveBoundingBoxBuffer(struct BB_BUFFER* bbBuffer, const char* name);
	//static bool is_nan(float x) { return x != x; }
	//static void saveOptimisationResults(struct SAMPLE_POINTS_BUFFER* samplePoints, struct SAMPLE_PCL* sP,struct SAMPLE_ROTATIONS* sR, float* costs,float* dist, int* weights,  const char* name);
	//static void loadDistanceMatrix(struct DISTANCE_MATRIX* dM, const char* name);
	//static void waitForEnter();
	//static void plotIntermediateResults(struct PROB_RESULT* probResult ,struct CENTROID* centroid);
	//static void saveInversionSearch(float* p, float* d, int* w, int n, const char* name);
	//static void loadInversionSearch(float* p, float* d, int* w, int* n, const char* name);
	//static void loadResultingSolution(struct RESULT_SOLUTION* solu, const char* name);
	//static void loadValidPos(struct VALID_POS* pos, const char* name);
	//static void writeMinCostToFile(unsigned long long* vec, int* pcl, int* angle, int nC);

	



};


