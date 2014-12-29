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

	static void savePCL(struct PCL* pcl, const char* name);
	static void loadPCL(struct PCL* pcl, const char* name);
	static void loadRobotPos(struct ROBOT_POSITION* pos, unsigned int N);
	static void loadHumanPos(struct HUMAN_POSITION* pos);
	static void writeKSDF(struct ROBOT_POSITION* robot_positions, unsigned int N);
	static void loadKSDF(struct ROBOT_POSITION* robot_positions, unsigned int N);
	static void loadFileOntoCuda(std::string fn, void* p_memcuda, unsigned int size);
	static void loadFileOntoHost(std::string fn, void* p_memhost, unsigned int size);
	static void compareFloatValuesCuda(float* p_d1,  unsigned int n, int offset,  const char* task, const char* fn);
	static void compareIntValuesCuda(unsigned int* p_d1, unsigned int n, int offset,  const char* task, const char* fn);
	static void printMinCostSingleCameraToFile(double* h_costs, struct POSITIONS* pos, struct PCL* robotPCL_qa0);
	static void printMinPositionToFile(std::vector<struct COST_POINT*>* p, struct POSITIONS* pos, struct PCL* robotPCL_qa0, int nofCams);

	//new IO load Functions
	static void loadSamplePCL(struct SAMPLE_PCL* pcl, const char* name);
	static void loadSampleRotations(struct SAMPLE_ROTATIONS* rot, const char* name);
	static void loadRobotPCL(struct ROBOT_PCL* pcl, const char* name);
	static void loadEnvironmentPCL(struct ENVIRONMENT_PCL* pcl, const char* name);
	static void loadHumanPCL(struct HUMAN_PCL* pcl, const char* name);
	static void loadSamplePositions(struct SAMPLE_POSITIONS* pso, const char* name);
	static void loadSampleCamera(struct SAMPLE_CAMERA* cam, const char* name);
	static void saveDepthBufferToFile(struct DEPTH_BUFFER* depth, const char* name);
	static void saveVerticeBufferToFile(struct VERTEX_BUFFER* buffer, const char* name);
	static void saveBoundingBoxBufferToFile(struct BB_BUFFER* buffer, const char* name);
	static void printCentroid(struct CENTROID* centroid);

};

#endif /* IO_H_ */
