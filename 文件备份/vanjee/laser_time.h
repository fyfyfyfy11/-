#ifndef LASER_TIME_H
#define LASER_TIME_H
#include <sys/time.h>
static inline long laser_timeval_diff(struct timeval* start, struct timeval* end) {
    return (end->tv_sec - start->tv_sec) * 1000 + (end->tv_usec - start->tv_usec) / 1000;
}
#endif