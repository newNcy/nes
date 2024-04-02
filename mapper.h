#pragma once
#include <stdint.h>
void mapper_0_cpu_io(void * device, uint16_t address, uint8_t * data, uint8_t is_write);
void mapper_0_ppu_io(void * device, uint16_t address, uint8_t * data, uint8_t is_write);
