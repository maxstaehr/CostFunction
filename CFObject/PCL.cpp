#include "PCL.h"
#include <string.h>
#include <iostream>

PCL::PCL(int nV, int nF,  const float const * x, 
						 const float const * y,
						 const float const * z,
						 const int const * fx, 
						 const int const * fy,
						 const int const * fz)
{


	this->nV = nV;
	this->nF = nF;

	this->x = new float[nV];
	this->y = new float[nV];
	this->z = new float[nV];

	this->ix = new float[nV];
	this->iy = new float[nV];
	this->iz = new float[nV];

	this->fx = new int[nF];
	this->fy = new int[nF];
	this->fz = new int[nF];

	memcpy(this->x, x, sizeof(float)*nV);
	memcpy(this->y, y, sizeof(float)*nV);
	memcpy(this->z, z, sizeof(float)*nV);

	memcpy(this->ix, x, sizeof(float)*nV);
	memcpy(this->iy, y, sizeof(float)*nV);
	memcpy(this->iz, z, sizeof(float)*nV);

	memcpy(this->fx, fx, sizeof(int)*nF);
	memcpy(this->fy, fy, sizeof(int)*nF);
	memcpy(this->fz, fz, sizeof(int)*nF);
}

void PCL::savePCL(std::ofstream* outbin)
{
	
	outbin->write((char*)&nV,sizeof(int));
	if (!outbin) std::cerr << "error";

	outbin->write((char*)&nF,sizeof(int));
	if (!outbin) std::cerr << "error";

	outbin->write((char*)x,sizeof(float)*nV);
	if (!outbin) std::cerr << "error";

	outbin->write((char*)y,sizeof(float)*nV);
	if (!outbin) std::cerr << "error";

	outbin->write((char*)z,sizeof(float)*nV);
	if (!outbin) std::cerr << "error";

	outbin->write((char*)fx,sizeof(float)*nF);
	if (!outbin) std::cerr << "error";

	outbin->write((char*)fy,sizeof(float)*nF);
	if (!outbin) std::cerr << "error";

	outbin->write((char*)fz,sizeof(float)*nF);
	if (!outbin) std::cerr << "error";

}
PCL::PCL():nV(0), nF(0), x(NULL), y(NULL), z(NULL), ix(NULL), iy(NULL), iz(NULL),fx(NULL), fy(NULL), fz(NULL)
{

}
void PCL::operator=(PCL& inst )
{
	
	if(x!=NULL)
		delete[] x;
	if(y!=NULL)
		delete[] y;
	if(z!=NULL)
		delete[] z;
	if(ix!=NULL)
		delete[] ix;
	if(iy!=NULL)
		delete[] iy;
	if(iz!=NULL)
		delete[] iz;

	if(fx!=NULL)
		delete[] fx;
	if(fy!=NULL)
		delete[] fy;
	if(fz!=NULL)
		delete[] fz;

	this->nF = inst.nF;
	this->nV = inst.nV;

	this->x = new float[nV];
	this->y = new float[nV];
	this->z = new float[nV];

	this->ix = new float[nV];
	this->iy = new float[nV];
	this->iz = new float[nV];

	this->fx = new int[nF];
	this->fy = new int[nF];
	this->fz = new int[nF];

	memcpy(this->x, inst.getx(), sizeof(float)*nV);
	memcpy(this->y, inst.gety(), sizeof(float)*nV);
	memcpy(this->z, inst.getz(), sizeof(float)*nV);

	memcpy(this->ix, inst.getix(), sizeof(float)*nV);
	memcpy(this->iy, inst.getiy(), sizeof(float)*nV);
	memcpy(this->iz, inst.getiz(), sizeof(float)*nV);

	memcpy(this->fx, inst.getfx(), sizeof(int)*nF);
	memcpy(this->fy, inst.getfy(), sizeof(int)*nF);
	memcpy(this->fz, inst.getfz(), sizeof(int)*nF);
}

PCL::PCL(PCL& inst)
{


	this->nF = inst.nF;
	this->nV = inst.nV;

	this->x = new float[nV];
	this->y = new float[nV];
	this->z = new float[nV];

	this->ix = new float[nV];
	this->iy = new float[nV];
	this->iz = new float[nV];

	this->fx = new int[nF];
	this->fy = new int[nF];
	this->fz = new int[nF];

	memcpy(this->x, inst.getx(), sizeof(float)*nV);
	memcpy(this->y, inst.gety(), sizeof(float)*nV);
	memcpy(this->z, inst.getz(), sizeof(float)*nV);

	memcpy(this->ix, inst.getix(), sizeof(float)*nV);
	memcpy(this->iy, inst.getiy(), sizeof(float)*nV);
	memcpy(this->iz, inst.getiz(), sizeof(float)*nV);

	memcpy(this->fx, inst.getfx(), sizeof(int)*nF);
	memcpy(this->fy, inst.getfy(), sizeof(int)*nF);
	memcpy(this->fz, inst.getfz(), sizeof(int)*nF);

}

void PCL::rotate(Rotation rot)
{
	float xb, yb, zb;
	for(int i=0; i<nV; i++)
	{
		xb = ix[i];
		yb = iy[i];
		zb = iz[i];

		x[i] = xb*rot.getH()[0] + yb*rot.getH()[1] + zb*rot.getH()[2];
		y[i] = xb*rot.getH()[3] + yb*rot.getH()[4] + zb*rot.getH()[5];
		z[i] = xb*rot.getH()[6] + yb*rot.getH()[7] + zb*rot.getH()[8];
	}
}
void PCL::transform(HomogeneTransformation trans)
{
	float xb, yb, zb;
	for(int i=0; i<nV; i++)
	{
		xb = ix[i];
		yb = iy[i];
		zb = iz[i];

		x[i] = xb*trans.getH()[0] + yb*trans.getH()[1] + zb*trans.getH()[2] + trans.getH()[3];
		y[i] = xb*trans.getH()[4] + yb*trans.getH()[5] + zb*trans.getH()[6] + trans.getH()[7];
		z[i] = xb*trans.getH()[8] + yb*trans.getH()[9] + zb*trans.getH()[10] + trans.getH()[11];
	}

}


PCL::~PCL(void)
{
	if(x!=NULL)
		delete[] x;
	if(y!=NULL)
		delete[] y;
	if(z!=NULL)
		delete[] z;

	if(x!=NULL)
		delete[] ix;
	if(y!=NULL)
		delete[] iy;
	if(z!=NULL)
		delete[] iz;
	if(fx!=NULL)
		delete[] fx;
	if(fy!=NULL)
		delete[] fy;
	if(fz!=NULL)
		delete[] fz;
}


