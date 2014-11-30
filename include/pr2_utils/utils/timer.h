#ifndef TIMER_H_DEF
#define TIMER_H_DEF

#include <time.h>

// from /usr/include/linux/time.h
#ifndef CLOCK_MONOTONIC_RAW
#define CLOCK_MONOTONIC_RAW 4

#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif

#endif

namespace pr2_utils {

typedef struct Timer{
	struct timespec tic;
	struct timespec toc;
} Timer;

/* read current time */
inline void Timer_tic(Timer* t) {
#ifdef __MACH__
	clock_get_time(CLOCK_MONOTONIC_RAW, &t->tic);
#else
	clock_gettime(CLOCK_MONOTONIC_RAW, &t->tic);
#endif
}

/* return time passed since last call to tic on this timer */
inline double Timer_toc(Timer* t)
{
	struct timespec temp;
#ifdef __MACH__
	clock_get_time(CLOCK_MONOTONIC_RAW, &t->toc);
#else
	clock_gettime(CLOCK_MONOTONIC_RAW, &t->toc);
#endif

	if ((t->toc.tv_nsec - t->tic.tv_nsec)<0) {
		temp.tv_sec = t->toc.tv_sec - t->tic.tv_sec-1;
		temp.tv_nsec = 1000000000+(t->toc.tv_nsec - t->tic.tv_nsec);
	} else {
		temp.tv_sec = t->toc.tv_sec - t->tic.tv_sec;
		temp.tv_nsec = t->toc.tv_nsec - t->tic.tv_nsec;
	}

	return (double)temp.tv_sec + (double)temp.tv_nsec / 1000000000;
}

}
#endif // TIMER_H_DEF
