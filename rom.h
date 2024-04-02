#pragma once
#include <stdint.h>

typedef struct {
    uint8_t prg_units;
    uint8_t chr_units;
    uint8_t flags_6;
    uint8_t flags_7;
    uint8_t flags_8;
    uint8_t flags_9;
    uint8_t flags_10;
    uint8_t padding[5];
}rom_header_t;

typedef struct {
    uint8_t prg_units;
    uint8_t chr_units;
    uint8_t mirroring;
    uint8_t has_trainer;
    uint8_t has_pram;
    uint8_t mapper;
    uint8_t * trainer;
    uint8_t * prg_rom;
    uint8_t * chr_rom;
}rom_t;


rom_t * rom_load(char * path);
void rom_destroy(rom_t * rom);
