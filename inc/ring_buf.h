#ifndef RING_BUF_H_
#define RING_BUF_H_

#include <stdint.h>
#include <stddef.h>
#include <stdatomic.h>
#define BUFFER_SIZE 128
typedef struct {
    uint8_t buf[BUFFER_SIZE];
    _Atomic volatile uint8_t read;
    _Atomic volatile uint8_t write;
    _Atomic volatile uint8_t count;
} rx_buf;

void init_buf(rx_buf* buffer);
void enqueue(rx_buf* buffer, const uint8_t byte);
void read_from_buf(rx_buf* buffer, uint8_t* payload, uint8_t len);

#endif // RING_BUF_H_