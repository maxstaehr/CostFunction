#pragma once

#include "global.h"
#include "Camera.h"
#include "Cluster.h"

#include <set>
#include <vector>


class CFOBJECT_EXPORT EC
{

	
public:
	EC(int maxBufferSize);
	~EC(void);

	int getMaxSize() {return maxSize;}
	void reset();
	void addDepthData(Camera& cam);
	void cluster();

	const float * const getX(){return x;}
	const float * const getY(){return y;}
	const float * const getZ(){return z;}



	const bool* const getHasProcessed() {return hasProcessed;}
	

	//std::set<int>&				getP(){return P;}
	//std::set<std::set<int>*>&	getC(){return C;}
	Cluster&					getCluster(int i){return clu[i];}
	int							getNumOfClusters(){return clu.size();}


	float						calcDistance(int i1, int i2);
	void						process(std::set<int>* Q, std::set<int>* P);
	int							findFirstNotProcessedIndice(std::set<int>* set);
	bool						hasAllBeenProcessed(std::set<int>* set);

private:
	int maxSize;

	float* x;
	float* y;
	float* z;
	bool* hasProcessed;

	std::set<int> P;
	std::set<std::set<int>*> C;
	


	std::vector<Cluster> clu;
};

