#include "nes.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

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
    rom_header_t header;
    uint8_t * trainer;
    uint8_t * prg_rom;
    uint8_t * chr_rom;
}rom_t;


typedef struct bus_device_t{
    uint16_t address_start;
    uint16_t address_end;
    uint8_t * memory;
    struct bus_device_t * next;
}bus_device_t;

typedef struct {
    bus_device_t * devices;
}bus_t;

enum {
    C, Z, I, D, B, V = 6, N
};

typedef struct {
    uint16_t PC;
    uint8_t SP;
    uint8_t A;
    uint8_t X;
    uint8_t Y;
    uint8_t P;
    bus_t bus;
}cpu_t;



typedef struct {
    cpu_t cpu;
}nes_t;

int take_bit(uint8_t byte, uint8_t bit) {
    return (byte>>bit)&1;
}

int set_bit(uint8_t byte, uint8_t bit, uint8_t v)
{
    uint8_t m = 1<<bit;
    if (v) {
        return byte |  m;
    } 
    return byte & ~m;
}

bus_device_t * make_bus_device(uint16_t start, uint16_t end) 
{
    bus_device_t * device = (bus_device_t*)malloc(sizeof (bus_device_t));
    device->address_start = start;
    device->address_end = end;
    device->memory = malloc(end-start);
    device->next = NULL;
    return device;
}

void bus_mount(bus_t * bus, bus_device_t * new_device)
{
    if (!bus->devices) {
        bus->devices = new_device;
        return;
    }

    bus_device_t * device = bus->devices;
    while (device->next) {
        device = device->next;
    }
    device->next = new_device;
}

void bus_operate(bus_t * bus, uint8_t is_write, uint16_t address, uint8_t * data) {
    bus_device_t * device = bus->devices;
    while (device) {
        if (device->address_start <= address && address <= device->address_end) {
            uint16_t offset = address - device->address_start;
            if (is_write) {
                device->memory[offset] = *data;
            } else {
                *data = device->memory[offset];
            }
            break;
        }

        device = device->next;
    }
}

uint8_t bus_read(bus_t * bus, uint16_t address) 
{
    uint8_t ret = 0;
    bus_operate(bus, 0, address, &ret);
    return ret;
}

uint8_t bus_read2(bus_t * bus, uint16_t address) 
{
    return bus_read(bus, address+1)<<8 | bus_read(bus, address);
}

void bus_write(bus_t * bus, uint16_t address, uint8_t data) 
{
    bus_operate(bus, 1, address, &data);
}




int load_rom(char * path, rom_t * rom) {
    FILE * fp = fopen(path, "rb");
    if (!fp) {
        printf("file not found\n");
        return 0;
    }
    char magic[4] = {0};
    int read_count = fread(magic, 1, 4, fp);
    if (read_count != 4) {
        printf("not enough bytes %d\n", read_count);
        return 0;
    }
    if (memcmp(magic, "NES\x1a", 4)) {
        printf("not a nes rom image %s\n", magic);
        return 0;
    }

    rom_header_t header;
    if (fread(&header, 1, sizeof(header), fp) != sizeof(header)) {
        printf("cant read header %d\n", sizeof(header));
        return 0;
    }

    printf("prg rom size:%d\n", header.prg_units);
    printf("chr rom size:%d\n", header.chr_units);
    printf("mirroring:%d\n", header.flags_6 & 0x1);
    printf("contains battery-backed prg ram:%d\n", take_bit(header.flags_6, 2));
    printf("has trainer at $7000-$71ff:%d\n", take_bit(header.flags_6,3));
    printf("ignore mirroring control:%d\n", take_bit(header.flags_6,4));
    printf("mapper number:%d\n", header.flags_6 >> 4);

    printf("vs unisystem:%d\n", take_bit(header.flags_7, 0));
    printf("play choice - 10:%d\n", take_bit(header.flags_7, 1));
    printf("nes version:%d\n", (take_bit(header.flags_7, 4)<<1) | take_bit(header.flags_7, 3));
    printf("mapper number:%d\n", header.flags_7 >> 4);

    printf("prg ram size:%d\n", header.flags_8 );
    printf("tv system:%s\n", header.flags_9 & 1? "pal":"ntsc");
    printf("tv system:%s\n", header.flags_10 & 3 == 0? "pal":"ntsc");
    fclose(fp);
}


void cpu_bootup(cpu_t * cpu) 
{
    cpu->P = 0x34;
    cpu->A = 0;
    cpu->X = 0;
    cpu->Y = 0;
    cpu->SP = 0xFD;

    for (int i =0 ;i < 0xf; ++ i) {
        bus_write(&cpu->bus, 0x4000 + i, 0);
    }
    bus_write(&cpu->bus, 0x4015, 0);
    bus_write(&cpu->bus, 0x4017, 0);
}

void bootup(nes_t * nes)
{
    cpu_bootup(&nes->cpu);
}

int main(int argc, char * argv[]) 
{
    /*
    if (argc < 2) {
        printf("usage: %s rom\n", argv[0]);
        return -1;
    }
    */
    const char * rom_path = "rom/cf.nes";
    rom_t rom;
    if (!load_rom(rom_path, &rom)) {
        printf("load rom failed\n");
    }

    nes_t nes;
    bootup(&nes);
    return 0;
}
