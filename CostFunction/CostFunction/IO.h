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
};

#endif /* IO_H_ */
