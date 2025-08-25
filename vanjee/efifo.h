#ifndef EFIFO_H
#define EFIFO_H
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t *buffer;
    int size;
    int head;
    int tail;
    int used;
} efifo_t;

efifo_t* efifo_creat(const char* name, int size, bool overwrite);
void efifo_clean(efifo_t* fifo);
int efifo_write(efifo_t* fifo, uint8_t* data, int len);
int efifo_get_used(efifo_t* fifo);
int efifo_pick(efifo_t* fifo, uint8_t* out, int len);
int efifo_cut(efifo_t* fifo, int len);
void efifo_destroy(efifo_t* fifo);

#endif