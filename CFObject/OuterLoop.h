#pragma once

#include "global.h"
#include <string>

class  CFOBJECT_EXPORT OuterLoop
{
public:
	OuterLoop(std::string root, bool debug, int maxNumberCamera);
	~OuterLoop(void);
	void run();

private:
	std::string root;
	bool debug;
	int currentNumberCamera;
	int maxNumberCamera;
};

