#include "Progress.h"
#include <stdio.h>


Progress::Progress(void)
{
}

void Progress::printProgress(double ci, double ei, time_t start, const char* task)
{
	double loopTime, timePerLoop, remainingTime, currPro,maxTime;
	time_t end;
	time(&end);
	loopTime = difftime(end, start);
	timePerLoop =  (loopTime /(double)ci);
	remainingTime = timePerLoop * (double) (ei - ci);
	maxTime = timePerLoop * ei;
	currPro = (double)ci/ (double)(ei);
	printf("%s max : %.1fmin\trem : %.2fmin\tpro: %.6f%%\n",task, maxTime/60.0, remainingTime/60.0, currPro*100.0);

}


Progress::~Progress(void)
{
}
