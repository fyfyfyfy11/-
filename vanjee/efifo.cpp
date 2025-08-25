#include "efifo.h"
#include <stdlib.h>
#include <string.h>

efifo_t* efifo_creat(const char* name, int size, bool overwrite) {
    efifo_t* fifo = (efifo_t*)malloc(sizeof(efifo_t));
    fifo->buffer = (uint8_t*)malloc(size);
    fifo->size = size;
    fifo->head = 0;
    fifo->tail = 0;
    fifo->used = 0;
    (void)name; (void)overwrite;
    return fifo;
}

void efifo_clean(efifo_t* fifo) {
    fifo->head = fifo->tail = fifo->used = 0;
}

int efifo_write(efifo_t* fifo, uint8_t* data, int len) {
    int written = 0;
    for (int i = 0; i < len; ++i) {
        if (fifo->used >= fifo->size) break;
        fifo->buffer[fifo->tail] = data[i];
        fifo->tail = (fifo->tail + 1) % fifo->size;
        fifo->used++;
        written++;
    }
    return written;
}

int efifo_get_used(efifo_t* fifo) {
    return fifo->used;
}

int efifo_pick(efifo_t* fifo, uint8_t* out, int len) {
    if (len > fifo->used) return 0;
    int idx = fifo->head;
    for (int i = 0; i < len; ++i) {
        out[i] = fifo->buffer[idx];
        idx = (idx + 1) % fifo->size;
    }
    return len;
}

int efifo_cut(efifo_t* fifo, int len) {
    if (len > fifo->used) len = fifo->used;
    fifo->head = (fifo->head + len) % fifo->size;
    fifo->used -= len;
    return len;
}

void efifo_destroy(efifo_t* fifo) {
    if (fifo) {
        free(fifo->buffer);
        free(fifo);
    }
}