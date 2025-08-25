#ifndef RTTHREAD_PORT_H
#define RTTHREAD_PORT_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/time.h>

#define rt_malloc      malloc
#define rt_free        free
#define rt_memset      memset
#define rt_memcpy      memcpy
#define rt_strcpy      strcpy
#define rt_thread_mdelay(ms) usleep((ms)*1000)
#define RT_ASSERT(x)   assert(x)
#define LOG_E(fmt, ...) printf("[ERROR] " fmt "\n", ##__VA_ARGS__)
#define LOG_I(fmt, ...) printf("[INFO] " fmt "\n", ##__VA_ARGS__)
#define LOG_D(fmt, ...) printf("[DEBUG] " fmt "\n", ##__VA_ARGS__)

typedef int rt_base_t;
static inline rt_base_t rt_hw_interrupt_disable(void) { return 0; }
static inline void rt_hw_interrupt_enable(rt_base_t x) { (void)x; }

#endif