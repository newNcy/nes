
#include "rom.h"
#include "nes.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

rom_t * rom_load(char * path)
{
    FILE * fp = fopen(path, "rb");
    if (!fp) {
        printf("file not found\n");
        return 0;
    }
    char magic[4] = {0};
    int read_count = fread(magic, 1, 4, fp);
    if (read_count != 4) {
        printf("not enough bytes %d\n", read_count);
        fclose(fp);
        return 0;
    }
    if (memcmp(magic, "NES\x1a", 4)) {
        printf("not a nes rom image %s\n", magic);
        fclose(fp);
        return 0;
    }

    rom_header_t header;
    if (fread(&header, 1, sizeof(header), fp) != sizeof(header)) {
        printf("cant read header %d\n", sizeof(header));
        fclose(fp);
        return 0;
    }

    rom_t * rom = malloc(sizeof(rom_t));
    printf("prg rom size:%d\n", header.prg_units);
    printf("chr rom size:%d\n", header.chr_units);
    printf("mirroring:%d\n", header.flags_6 & 0x1);
    printf("contains battery-backed prg ram:%d\n", bit_get(&header.flags_6, 2));
    printf("has trainer at $7000-$71ff:%d\n", bit_get(&header.flags_6,3));
    printf("ignore mirroring control:%d\n", bit_get(&header.flags_6,4));
    printf("mapper number:%d\n", header.flags_6 >> 4);

    printf("vs unisystem:%d\n", bit_get(&header.flags_7, 0));
    printf("play choice - 10:%d\n", bit_get(&header.flags_7, 1));
    printf("nes version:%d\n", (bit_get(&header.flags_7, 4)<<1) | bit_get(&header.flags_7, 3));
    printf("mapper number:%d\n", header.flags_7 >> 4);

    printf("prg ram size:%d\n", header.flags_8 );
    printf("tv system:%s\n", header.flags_9 & 1? "pal":"ntsc");
    printf("tv system:%s\n", (header.flags_10 & 3) == 0? "pal":"ntsc");

    rom->mirroring = bit_get(&header.flags_6, 0);
    rom->has_pram = bit_get(&header.flags_6, 1);
    rom->has_trainer = bit_get(&header.flags_6, 2);

    rom->mapper = (header.flags_6 >> 4) & 0xf;
    uint16_t prg_size = header.prg_units * 16 * 1024;
    uint16_t chr_size = header.chr_units * 8 * 1024;
    rom->prg_rom = (uint8_t*)malloc(prg_size);
    rom->chr_rom = (uint8_t*)malloc(chr_size);
    fread(rom->prg_rom, prg_size, 1, fp);
    fread(rom->chr_rom, chr_size, 1, fp);

    /*
    for (uint16_t b = 0; b < prg_size; b += 0x10) {
        printf("%04X ", b+0x8000);
        for (uint16_t p = 0; p < 0x10; p ++) {
            uint8_t byte = rom->prg_rom[b+p];
            printf("%02X ", byte);
        }
        printf("\n");
    }
    */
    fclose(fp);
    return rom;
}


void rom_destroy(rom_t * rom)
{
    free(rom->prg_rom);
    free(rom->chr_rom);
    free(rom);
}

