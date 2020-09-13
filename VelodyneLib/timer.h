#ifndef _TIMER_H_
#define _TIMER_H_

#include<time.h>
double getCurrentTime()
{
	return	(double)clock() / CLOCKS_PER_SEC;
}
#endif // !_TIMER_H_

