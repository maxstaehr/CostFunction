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
	EC ec(0);
	bool ret = true;
	ret &= ec.getQ() != NULL;
	ret &= ec.getCi() != NULL;
	return ret;
}
