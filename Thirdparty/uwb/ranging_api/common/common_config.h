#pragma once

#include <stdint.h>

#define IPC_NAME "/telemetry"
#define SEM_NAME "telemetry_sem"

#define UNAVAILABLE_NODE (UINT16_MAX - 1)

// Timing definitions
//
// differences between two timespec in seconds
#define timespec_to_sec(t1, t2) ((double)(t1.tv_nsec - t2.tv_nsec) * 1e-9 + (double)(t1.tv_sec - t2.tv_sec))
// differences between two timespec in milliseconds
#define timespec_to_msec(t1, t2) ((double)(t1.tv_nsec - t2.tv_nsec) * 1e-6 + (double)(t1.tv_sec - t2.tv_sec) * 1e3)

// add milliseconds to a timespec for timeout purposes
#define timespec_add_msec(ts, msec) uint64_t adj_nsec = 1000000ULL * msec;\
ts.tv_nsec += adj_nsec;\
if (adj_nsec >= 1000000000) {\
	uint64_t adj_sec = adj_nsec / 1000000000;\
	ts.tv_nsec -= adj_sec * 1000000000;\
	ts.tv_sec += adj_sec;\
}\
if (ts.tv_nsec >= 1000000000){\
	ts.tv_nsec -= 1000000000;\
	ts.tv_sec++;\
}

#define msleep(msec) usleep((msec) * 1000)

// utilities
#define UNSET_BIT(mask, index) mask &= ~(1 << (index))
#define SET_BIT(mask, index) mask |= 1 << (index)
#define GET_BIT(mask, index) mask & (1 << index)

// structures
typedef struct coordinates {
	double c_lat;
	double c_lon;
} coordinates_t;

