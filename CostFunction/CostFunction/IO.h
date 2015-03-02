/*
 * IO.h
 *
 *  Created on: 13.06.2014
 *      Author: tsdf
 */

#ifndef IO_H_
#define IO_H_

#include <iostream>
#include <vector>


class IO {
public:
	IO();
	virtual ~IO();


	//new IO load Functions
	static void loadSamplePCL(struct SAMPLE_PCL* pcl, const char* name);
	static void loadSampleRotations(struct SAMPLE_ROTATIONS* rot, const char* name);
	static void loadPCL(struct PCL* pcl, const char* name);
	static void loadSamplePositions(struct SAMPLE_POSITIONS* pso, const char* name);
	static void loadSampleCamera(struct POSSIBLE_CAMERA_TYPES* cams, const char* name);
	static void saveDepthBufferToFile(struct DEPTH_BUFFER* depth, const char* name);
	static void saveDepthBufferToFileSuperSamples(struct DEPTH_BUFFER* depth, const char* name);
	static void saveVerticeBufferToFile(struct VERTEX_BUFFER* buffer, const char* name);
	static void saveBoundingBoxBufferToFile(struct BB_BUFFER* buffer, const char* name);
	static void printCentroid(struct CENTROID* centroid);
	static void loadSampleFitting(struct SAMPLE_FITTING* sampleFitting, struct LAUNCH_CONFIG* config, const char* name);
	static void saveProbResult2File(struct PROB_RESULT* probResult,  const char* name);
	static void saveBoundingBoxBuffer(struct BB_BUFFER* bbBuffer, const char* name);
	static bool is_nan(float x) { return x != x; }
	static void saveOptimisationResults(struct SAMPLE_POINTS_BUFFER* samplePoints, struct SAMPLE_PCL* sP,struct SAMPLE_ROTATIONS* sR, float* costs,float* dist, int* weights,  const char* name);
	static void loadDistanceMatrix(struct DISTANCE_MATRIX* dM, const char* name);
	static void waitForEnter();
	static void plotIntermediateResults(struct PROB_RESULT* probResult ,struct CENTROID* centroid);
	static void saveInversionSearch(float* p, float* d, int* w, int n, const char* name);
	static void loadInversionSearch(float* p, float* d, int* w, int* n, const char* name);
	



};

#endif /* IO_H_ */
