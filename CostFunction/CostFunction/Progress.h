#pragma once
#include <time.h>
class Progress
{
public:
	Progress(void);
	~Progress(void);

	static void printProgress(double ci, double ei, time_t start, const char* task);
};

