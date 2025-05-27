#pragma once
#include <stdint.h>
#include <stdint.h>

static inline uint8_t bit_get(uint8_t * byte, uint8_t bit) {
    return ((*byte)>>bit)&1;
}

static inline void bit_set(uint8_t * byte, uint8_t bit)
{
    *byte |= (1<<bit);
}

static inline void bit_clr(uint8_t * byte, uint8_t bit)
{
    *byte &= ~(1<<bit);
}

static inline void bit_set_value(uint8_t * byte, uint8_t bit, uint8_t v)
{
    if (v) bit_set(byte, bit);
    else bit_clr(byte, bit);
}

static inline void memory_io(void * memory, uint16_t offset, uint8_t * data, uint8_t is_write)
{
    uint8_t * bytes = (uint8_t*)memory;
    if (is_write) {
        bytes[offset] = *data;
    }else {
        *data = bytes[offset];
    }
}



