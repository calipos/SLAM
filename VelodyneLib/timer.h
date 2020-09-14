#ifndef _TIMER_H_
#define _TIMER_H_

#include<time.h>
static double getCurrentTime()
{
	return	(double)clock();// / CLOCKS_PER_SEC;
}
static float toSec(const double&time)
{
	return time / CLOCKS_PER_SEC;
}
#endif // !_TIMER_H_

