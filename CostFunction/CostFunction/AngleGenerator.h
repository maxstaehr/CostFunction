#pragma once
class AngleGenerator
{
public:
	AngleGenerator();
	AngleGenerator(float* costs, int nAngle, int DOF);
	~AngleGenerator(void);

	int generateRandomAngle();


private:
	bool checkIndex(int i, float value);

	float *limits;
	int* indices;
	int nOfLimits;
	int nAng;
	
};

