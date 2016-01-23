#include "ECTestAdapter.h"

#include "EC.h"
#include <iostream>
ECTestAdapter::ECTestAdapter(void)
{
}


ECTestAdapter::~ECTestAdapter(void)
{
}

bool ECTestAdapter::test1()
{
	EC ec(1);
	bool ret = true;

	ret &= ec.getMaxSize() == 1;
	//ret &= ec.getP().size() == 0;
	//ret &= ec.getC().size() == 0;

	ret &= ec.getX() != NULL;
	ret &= ec.getY() != NULL;
	ret &= ec.getZ() != NULL;

	ret &= ec.getHasProcessed() != NULL;
	return ret;
}

bool ECTestAdapter::test2()
{

	float x[] = {0, 0, FLT_MAX};
	float y[] = {0, 0, FLT_MAX};
	float z[] = {0, 0, FLT_MAX};
	int nx = 3;
	int ny = 1;
	CameraType type(nx, ny, x, y, z, nx, ny, x, y, z);
	Camera cam(type);
	for(int i=0; i<nx*ny; i++)
	{
		cam.dx[i] = x[i];
		cam.dy[i] = y[i];
		cam.dz[i] = z[i];
	}


	EC ec(3);	
	ec.addDepthData(cam);
	bool ret = true;
	ret &= ec.getMaxSize() == 3;
	//ret &= ec.getP().size() == 2;
	//ret &= ec.getC().size() == 0;
	ret &= ec.getX() != NULL;
	ret &= ec.getY() != NULL;
	ret &= ec.getZ() != NULL;

	ret &= ec.getHasProcessed() != NULL;
	//std::set<int>::iterator it_p = ec.getP().begin();
	//for(int i=0; i<2; i++, ++it_p)
	//{
	//	//ret &= *it_p == i;
	//	ret &= ec.getX()[i] == x[i];
	//	ret &= ec.getY()[i] == y[i];
	//	ret &= ec.getZ()[i] == z[i];
	//	ret &= ec.getHasProcessed()[i] == false;
	//}


	return ret;
}


bool ECTestAdapter::test3()
{

	float x[] = {0, 0, FLT_MAX};
	float y[] = {0, 0, FLT_MAX};
	float z[] = {0, 0, FLT_MAX};
	int nx = 3;
	int ny = 1;
	CameraType type(nx, ny, x, y, z, nx, ny, x, y, z);
	Camera cam(type);
	for(int i=0; i<nx*ny; i++)
	{
		cam.dx[i] = x[i];
		cam.dy[i] = y[i];
		cam.dz[i] = z[i];
	}


	EC ec(6);	
	ec.addDepthData(cam);
	ec.addDepthData(cam);
	bool ret = true;
	ret &= ec.getMaxSize() == 6;
	//ret &= ec.getP().size() == 4;
	//ret &= ec.getC().size() == 0;
	ret &= ec.getX() != NULL;
	ret &= ec.getY() != NULL;
	ret &= ec.getZ() != NULL;

	ret &= ec.getHasProcessed() != NULL;
	//std::set<int>::iterator it_p = ec.getP().begin();
	//for(int i=0; i<4; i++, ++it_p)
	//{
	//	ret &= *it_p == i;
	//	ret &= ec.getHasProcessed()[i] == false;
	//}

	ret &= ec.getX()[0] == x[0];
	ret &= ec.getY()[0] == y[0];
	ret &= ec.getZ()[0] == z[0];

	ret &= ec.getX()[1] == x[1];
	ret &= ec.getY()[1] == y[1];
	ret &= ec.getZ()[1] == z[1];

	ret &= ec.getX()[2] == x[0];
	ret &= ec.getY()[2] == y[0];
	ret &= ec.getZ()[2] == z[0];

	ret &= ec.getX()[3] == x[1];
	ret &= ec.getY()[3] == y[1];
	ret &= ec.getZ()[3] == z[1];


	return ret;
}


bool ECTestAdapter::test4()
{

	float x[] = {0, 0, FLT_MAX};
	float y[] = {0, 0, FLT_MAX};
	float z[] = {0, 0, FLT_MAX};
	int nx = 3;
	int ny = 1;
	CameraType type(nx, ny, x, y, z, nx, ny, x, y, z);
	Camera cam(type);
	for(int i=0; i<nx*ny; i++)
	{
		cam.dx[i] = x[i];
		cam.dy[i] = y[i];
		cam.dz[i] = z[i];
	}


	EC ec(6);	
	ec.addDepthData(cam);
	ec.addDepthData(cam);
	bool ret = true;
	ret &= ec.getMaxSize() == 6;
	//ret &= ec.getP().size() == 4;
	//ret &= ec.getC().size() == 0;
	ret &= ec.getX() != NULL;
	ret &= ec.getY() != NULL;
	ret &= ec.getZ() != NULL;

	ret &= ec.getHasProcessed() != NULL;
	/*std::set<int>::iterator it_p = ec.getP().begin();
	for(int i=0; i<4; i++, ++it_p)
	{
		ret &= *it_p == i;
		ret &= ec.getHasProcessed()[i] == false;
	}

	ret &= ec.getX()[0] == x[0];
	ret &= ec.getY()[0] == y[0];
	ret &= ec.getZ()[0] == z[0];

	ret &= ec.getX()[1] == x[1];
	ret &= ec.getY()[1] == y[1];
	ret &= ec.getZ()[1] == z[1];

	ret &= ec.getX()[2] == x[0];
	ret &= ec.getY()[2] == y[0];
	ret &= ec.getZ()[2] == z[0];

	ret &= ec.getX()[3] == x[1];
	ret &= ec.getY()[3] == y[1];
	ret &= ec.getZ()[3] == z[1];
*/

	return ret;
}


bool ECTestAdapter::test5()
{

	float x[] = {0, 0, FLT_MAX};
	float y[] = {0, 0, FLT_MAX};
	float z[] = {0, 0, FLT_MAX};
	int nx = 3;
	int ny = 1;
	CameraType type(nx, ny, x, y, z, nx, ny, x, y, z);
	Camera cam(type);
	for(int i=0; i<nx*ny; i++)
	{
		cam.dx[i] = x[i];
		cam.dy[i] = y[i];
		cam.dz[i] = z[i];
	}


	EC ec(6);	
	ec.addDepthData(cam);
	ec.addDepthData(cam);
	bool ret = true;
	ret &= ec.getMaxSize() == 6;
	//ret &= ec.getP().size() == 4;
	//ret &= ec.getC().size() == 0;
	ret &= ec.getX() != NULL;
	ret &= ec.getY() != NULL;
	ret &= ec.getZ() != NULL;

	ret &= ec.getHasProcessed() != NULL;
	//std::set<int>::iterator it_p = ec.getP().begin();
	//for(int i=0; i<4; i++, ++it_p)
	//{
	//	ret &= *it_p == i;
	//	ret &= ec.getHasProcessed()[i] == false;
	//}

	ret &= ec.getX()[0] == x[0];
	ret &= ec.getY()[0] == y[0];
	ret &= ec.getZ()[0] == z[0];

	ret &= ec.getX()[1] == x[1];
	ret &= ec.getY()[1] == y[1];
	ret &= ec.getZ()[1] == z[1];

	ret &= ec.getX()[2] == x[0];
	ret &= ec.getY()[2] == y[0];
	ret &= ec.getZ()[2] == z[0];

	ret &= ec.getX()[3] == x[1];
	ret &= ec.getY()[3] == y[1];
	ret &= ec.getZ()[3] == z[1];

	ec.cluster();
	std::set<std::set<int>*>::iterator it;
	//ret &= ec.getC().size() == 1;

	//it=ec.getC().begin();
	//it_p=(*it)->begin();

	//ret &= *it_p == 0;
	//ret &= ec.getX()[*it_p] == x[0];
	//ret &= ec.getY()[*it_p] == y[0];
	//ret &= ec.getZ()[*it_p] == z[0];
	//ret &= ec.getHasProcessed()[*it_p];

	//++it_p;
	//ret &= *it_p == 1;
	//ret &= ec.getX()[*it_p] == x[1];
	//ret &= ec.getY()[*it_p] == y[1];
	//ret &= ec.getZ()[*it_p] == z[1];
	//ret &= ec.getHasProcessed()[*it_p];

	//++it_p;
	//ret &= *it_p == 2;
	//ret &= ec.getX()[*it_p] == x[0];
	//ret &= ec.getY()[*it_p] == y[0];
	//ret &= ec.getZ()[*it_p] == z[0];
	//ret &= ec.getHasProcessed()[*it_p];

	//++it_p;
	//ret &= *it_p == 3;
	//ret &= ec.getX()[*it_p] == x[1];
	//ret &= ec.getY()[*it_p] == y[1];
	//ret &= ec.getZ()[*it_p] == z[1];
	//ret &= ec.getHasProcessed()[*it_p];

	return ret;
}


bool ECTestAdapter::test6()
{

	float x[] = {0, 0, FLT_MAX};
	float y[] = {0, 0, FLT_MAX};
	float z[] = {0, 0, FLT_MAX};
	int nx = 3;
	int ny = 1;
	CameraType type(nx, ny, x, y, z, nx, ny, x, y, z);
	Camera cam(type);
	for(int i=0; i<nx*ny; i++)
	{
		cam.dx[i] = x[i];
		cam.dy[i] = y[i];
		cam.dz[i] = z[i];
	}


	EC ec(6);	
	ec.addDepthData(cam);
	ec.addDepthData(cam);
	bool ret = true;
	ret &= ec.getMaxSize() == 6;
	//ret &= ec.getP().size() == 4;
	//ret &= ec.getC().size() == 0;
	ret &= ec.getX() != NULL;
	ret &= ec.getY() != NULL;
	ret &= ec.getZ() != NULL;

	ret &= ec.getHasProcessed() != NULL;
	//std::set<int>::iterator it_p = ec.getP().begin();
	//for(int i=0; i<4; i++, ++it_p)
	//{
	//	ret &= *it_p == i;
	//	ret &= ec.getHasProcessed()[i] == false;
	//}

	ret &= ec.getX()[0] == x[0];
	ret &= ec.getY()[0] == y[0];
	ret &= ec.getZ()[0] == z[0];

	ret &= ec.getX()[1] == x[1];
	ret &= ec.getY()[1] == y[1];
	ret &= ec.getZ()[1] == z[1];

	ret &= ec.getX()[2] == x[0];
	ret &= ec.getY()[2] == y[0];
	ret &= ec.getZ()[2] == z[0];

	ret &= ec.getX()[3] == x[1];
	ret &= ec.getY()[3] == y[1];
	ret &= ec.getZ()[3] == z[1];

	ec.cluster();
	std::set<std::set<int>*>::iterator it;
	//ret &= ec.getC().size() == 1;

	//it=ec.getC().begin();
	//it_p=(*it)->begin();
/*
	ret &= *it_p == 0;
	ret &= ec.getX()[*it_p] == x[0];
	ret &= ec.getY()[*it_p] == y[0];
	ret &= ec.getZ()[*it_p] == z[0];
	ret &= ec.getHasProcessed()[*it_p];

	++it_p;
	ret &= *it_p == 1;
	ret &= ec.getX()[*it_p] == x[1];
	ret &= ec.getY()[*it_p] == y[1];
	ret &= ec.getZ()[*it_p] == z[1];
	ret &= ec.getHasProcessed()[*it_p];

	++it_p;
	ret &= *it_p == 2;
	ret &= ec.getX()[*it_p] == x[0];
	ret &= ec.getY()[*it_p] == y[0];
	ret &= ec.getZ()[*it_p] == z[0];
	ret &= ec.getHasProcessed()[*it_p];

	++it_p;
	ret &= *it_p == 3;
	ret &= ec.getX()[*it_p] == x[1];
	ret &= ec.getY()[*it_p] == y[1];
	ret &= ec.getZ()[*it_p] == z[1];
	ret &= ec.getHasProcessed()[*it_p];
*/
	return ret;
}

