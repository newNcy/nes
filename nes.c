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


typedef void (*device_io_t)(void * device, uint16_t address, uint8_t * data, uint8_t is_write);

typedef struct bus_device_t {
    uint16_t start;
    uint16_t end;
    void * device;
    device_io_t device_io;
    struct bus_device_t * next;
}bus_device_t;

typedef struct {
    bus_device_t * devices;
}bus_t;

typedef enum {
    C, Z, I, D, B, V = 6, N
}cpu_flag_t;

typedef enum {
    NMI, RESET, IRQ, BRK
}cpu_int_t;

typedef struct {
    uint16_t PC;
    uint8_t SP;
    uint8_t A;
    uint8_t X;
    uint8_t Y;
    uint8_t P;
    bus_t bus;
    uint8_t * ram;
}cpu_t;



typedef struct {
    cpu_t cpu;
}nes_t;

int bit_get(uint8_t byte, uint8_t bit) {
    return (byte>>bit)&1;
}

void bit_set(uint8_t * byte, uint8_t bit)
{
    *byte |= (1<<bit);
}

void bit_clr(uint8_t * byte, uint8_t bit)
{

    *byte &= ~(1<<bit);
}

void memory_io(void * memory, uint16_t offset, uint8_t * data, uint8_t is_write)
{
    uint8_t * bytes = (uint8_t*)memory;
    if (is_write) {
        bytes[offset] = *data;
    }else {
        *data = bytes[offset];
    }
}

void bus_mount(bus_t * bus, void * device, uint16_t start, uint16_t end, device_io_t device_io)
{
    bus_device_t * mount = (bus_device_t*)malloc(sizeof(bus_device_t));
    memset(mount, 0, sizeof(bus_device_t));
    mount->start = start;
    mount->end = end;
    mount->device = device;
    mount->device_io = device_io;

    if (!bus->devices) {
        bus->devices = mount;
        return;
    }

    bus_device_t * cur = bus->devices;
    while (cur->next) {
        cur = cur->next;
    }
    cur->next = mount;
}

void bus_io(bus_t * bus, uint16_t address, uint8_t * data, uint8_t is_write) {
    bus_device_t * mount = bus->devices;
    while (mount) {
        if (mount->start <= address && address <= mount->end && mount->device && mount->device_io) {
            uint16_t offset = address - mount->start;
            mount->device_io(mount->device, offset, data, is_write);
            break;
        }

        mount = mount->next;
    }
}

uint8_t bus_read(bus_t * bus, uint16_t address) 
{
    uint8_t ret = 0;
    bus_io(bus, address, &ret, 0);
    return ret;
}

uint8_t bus_read2(bus_t * bus, uint16_t address) 
{
    return bus_read(bus, address+1)<<8 | bus_read(bus, address);
}

void bus_write(bus_t * bus, uint16_t address, uint8_t data) 
{
    bus_io(bus, address, &data, 1);
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
    printf("contains battery-backed prg ram:%d\n", bit_get(header.flags_6, 2));
    printf("has trainer at $7000-$71ff:%d\n", bit_get(header.flags_6,3));
    printf("ignore mirroring control:%d\n", bit_get(header.flags_6,4));
    printf("mapper number:%d\n", header.flags_6 >> 4);

    printf("vs unisystem:%d\n", bit_get(header.flags_7, 0));
    printf("play choice - 10:%d\n", bit_get(header.flags_7, 1));
    printf("nes version:%d\n", (bit_get(header.flags_7, 4)<<1) | bit_get(header.flags_7, 3));
    printf("mapper number:%d\n", header.flags_7 >> 4);

    printf("prg ram size:%d\n", header.flags_8 );
    printf("tv system:%s\n", header.flags_9 & 1? "pal":"ntsc");
    printf("tv system:%s\n", header.flags_10 & 3 == 0? "pal":"ntsc");
    rom->header = header;
    rom->prg_rom = (uint8_t*)malloc(header.prg_units * 16 * 1024);
    rom->chr_rom = (uint8_t*)malloc(header.chr_units * 8 * 1024);
    fclose(fp);
    return 1;
}


void cpu_power_up(cpu_t * cpu) 
{
    memset(cpu, 0, sizeof(cpu_t));
    cpu->ram = (uint8_t*)malloc(0x800);

    bus_mount(&cpu->bus, cpu->ram, 0x0000, 0x07ff, memory_io);
    bus_mount(&cpu->bus, cpu->ram, 0x0800, 0x0fff, memory_io);
    bus_mount(&cpu->bus, cpu->ram, 0x1000, 0x17ff, memory_io);
    bus_mount(&cpu->bus, cpu->ram, 0x1800, 0x1fff, memory_io);

    cpu->P = 0x34;
    cpu->A = 0;
    cpu->X = 0;
    cpu->Y = 0;
    cpu->SP = 0xFD;

    for (int i =0x4000 ;i <= 0x4013; ++ i) {
        bus_write(&cpu->bus, i, 0);
    }
}

void cpu_reset(cpu_t * cpu) 
{
    cpu->SP -= 3;
    bit_set(&cpu->P, I);
}

void cpu_interupt(cpu_t * cpu, cpu_int_t i)
{
}

void nes_power_up(nes_t * nes)
{
    cpu_power_up(&nes->cpu);
}

void nes_set_rom(nes_t * nes, rom_t * rom)
{
    bus_mount(&nes->cpu.bus, rom->prg_rom, 0x8000, 0xffff, memory_io);
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
    nes_power_up(&nes);
    nes_set_rom(&nes,&rom);
    return 0;
}
