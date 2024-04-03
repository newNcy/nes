#include "mapper.h"
#include "nes.h"
#include "rom.h"
void mapper_0_cpu_io(void * device, uint16_t address, uint8_t * data, uint8_t is_write)
{
    rom_t * rom = (rom_t*)device;
    address -= 0x8000 - 0x4020;
    memory_io(rom->prg_rom, address & 0x3fff, data, is_write);
}

void mapper_0_ppu_io(void * device, uint16_t address, uint8_t * data, uint8_t is_write)
{
    rom_t * rom = (rom_t*)device;
    memory_io(rom->chr_rom, address, data, is_write);
}


