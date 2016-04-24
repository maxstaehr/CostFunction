#include "CE.h"
using namespace std;

void xp(const vector<vector<int>*>& vecs, vector<vector<int>*> *result) {
  vector<vector<int>*>* rslts;
  for (int ii = 0; ii < vecs.size(); ++ii) {
    const vector<int>& vec = *vecs[ii];
    if (ii == 0) {
      // vecs=[[1,2],...] ==> rslts=[[1],[2]]
      rslts = new vector<vector<int>*>;
      for (int jj = 0; jj < vec.size(); ++jj) {
        vector<int>* v = new vector<int>;
        v->push_back(vec[jj]);
        rslts->push_back(v);
      }
    } else {
      // vecs=[[1,2],[3,4],...] ==> rslts=[[1,3],[1,4],[2,3],[2,4]]
      vector<vector<int>*>* tmp = new vector<vector<int>*>;
      for (int jj = 0; jj < vec.size(); ++jj) {  // vec[jj]=3 (first iter jj=0)
        for (vector<vector<int>*>::const_iterator it = rslts->begin();
             it != rslts->end(); ++it) {
          vector<int>* v = new vector<int>(**it);       // v=[1]
          v->push_back(vec[jj]);                        // v=[1,3]
          tmp->push_back(v);                            // tmp=[[1,3]]
        }
      }
      for (int kk = 0; kk < rslts->size(); ++kk) {
        delete (*rslts)[kk];
      }
      delete rslts;
      rslts = tmp;
    }
  }
  result->insert(result->end(), rslts->begin(), rslts->end());
  delete rslts;
}



CE::CE(SampleCameraConfiguration* sampleConfig, int n, bool log):Search(sampleConfig, n, log), currentCompleteIndex(0)
{
	vector<vector<int>*> vecs;
	for(int i=0; i<n; i++)
	{
		vector<int>* p = new vector<int>();
		for(int j=0; j<sampleConfig[i].getNRelative(); j++)
		{
			p->push_back(j);
		}
		vecs.push_back(p);
	}	
	xp(vecs, &indices);
}

int CE::getIndices(int m, int n)
{
	vector<int>*p;
	p = indices[m];
	return (*p)[n];
}





CE::~CE(void)
{
}

bool CE::nextIteration(double cost_m, double cost_p)
{
	if(currentCompleteIndex < getM())
	{
		for(int i=0; i<n; i++)
		{
			nextEvalMinus[i]	= sampleConfig[i].getInitialH(getIndices(currentCompleteIndex, i));
			nextEvalPlus[i]		= sampleConfig[i].getInitialH(getIndices(currentCompleteIndex+1, i));
		}

		if(log)
		{
			logging(cost_m, cost_p);
		}
		currentCompleteIndex += 2;
		return true;
	}else
	{
		return false;
	}
	
}
void CE::setCurrentTransformation(HomogeneTransformation h, int i)
{

}
