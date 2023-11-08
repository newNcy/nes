#include "nes.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


#include <SDL3/SDL.h>

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
    NMI, RESET, IRQ
}cpu_int_t;

/* https://www.nesdev.org/wiki/CPU_unofficial_opcodes */
typedef enum{
    NOP,BRK,PHP,BPL,CLC,ORA,STP,ASL,SLO,ANC,
    JSR,BIT,PLP,BMI,SEC,AND,ROL,RLA,
    RTI,PHA,JMP,BVC,CLI,EOR,LSR,SRE,ALR,
    RTS,PLA,BVS,SEI,ADC,ROR,ARR,RRA,
    STY,DEY,BCC,TYA,SHY,STA,STX,TXA,TXS,SHX,SAX,XAA,AHX,TAS,
    LDY,TAY,BCS,CLV,LDA,LDX,LAX,LAS,TAX,TSX,
    CPY,INY,BNE,CLD,CMP,DEC,DEX,DCP,AXS,
    CPX,INX,BEQ,SED,SBC,INC,ISC,
}inst_type_t;

typedef enum {
    IMP,IMM,ZP,ZPX,ZPY,IZX,IZY,ABS,ABX,ABY,IND,REL
}addressing_mode_t;

typedef struct {
    inst_type_t t;
    addressing_mode_t am;
    uint8_t cycles;
}inst_t;

/* chatgpt */
static char * inst_tag[] = {
    "nop","brk","php","bpl","clc","ora","stp","asl","slo","anc",
    "jsr","bit","plp","bmi","sec","and","rol","rla",
    "rti","pha","jmp","bvc","cli","eor","lsr","sre","alr",
    "rts","pla","bvs","sei","adc","ror","arr","rra",
    "sty","dey","bcc","tya","shy","sta","stx","txa","txs","shx","sax","xaa","ahx","tas",
    "ldy","tay","bcs","clv","lda","ldx","lax","las","tax","tsx",
    "cpy","iny","bne","cld","cmp","dec","dex","dcp","axs",
    "cpx","inx","beq","sed","sbc","inc","isc"
};

/* http://www.oxyron.de/html/opcodes02.html */ 
static inst_t inst_map[] = {
    {BRK, IMP, 7}, 
    {ORA, IZX, 6}, 
    {STP, IMP, 0},
    {SLO, IZX, 8},

    {NOP, ZP, 3},
    {ORA, ZP, 3},
    {ASL, ZP, 5},
    {SLO, ZP, 5},

    {PHP, IMP, 3},
    {ORA, IMM, 2},
    {ASL, IMM, 2},
    {ANC, IMM, 2},

    {NOP, ABS, 4},
    {ORA, ABS, 4},
    {ASL, ABS, 6},
    {SLO, ABS, 6},


    {BPL, REL, 2},
    {ORA, IZY, 5},
    {STP, IMP, 0},
    {SLO, IZY, 8},

    {NOP, ZPX, 4},
    {ORA, ZPX, 4},
    {ASL, ZPX, 6},
    {SLO, ZPX, 6},

    {CLC, IMP, 2},
    {ORA, ABY, 4},
    {NOP, IMP, 2},
    {SLO, ABY, 7},

    {NOP, ABX, 4},
    {ORA, ABX, 4},
    {ASL, ABX, 7},
    {SLO, ABX, 7},


    {JSR, ABS, 6},
    {AND, IZX, 6},
    {STP, IMP, 0},
    {RLA, IZX, 8},

    {BIT, ZP, 3},
    {AND, ZP, 3},
    {ROL, ZP, 5},
    {RLA, ZP, 5},

    {PLP, IMP, 4},
    {AND, IMM, 2},
    {ROL, IMP, 2},
    {ANC, IMM, 2},

    {BIT, ABS, 4},
    {AND, ABS, 4},
    {ROL, ABS, 6},
    {RLA, ABS, 6},


    {BMI, REL, 2},
    {AND, IZY, 5},
    {STP, IMP, 0},
    {RLA, IZY, 8},

    {NOP, ZPX, 4},
    {AND, ZPX, 4},
    {ROL, ZPX, 6},
    {RLA, ZPX, 6},

    {SEC, IMP, 2},
    {AND, ABY, 4},
    {NOP, IMP, 2},
    {RLA, ABY, 7},

    {NOP, ABX, 4},
    {AND, ABX, 4},
    {ROL, ABX, 7},
    {RLA, ABX, 7},


    {RTI, IMP, 6},
    {EOR, IZX, 6},
    {STP, IMP, 0},
    {SRE, IZX, 8},

    {NOP, ZP, 3},
    {EOR, ZP, 3},
    {LSR, ZP, 5},
    {SRE, ZP, 5},

    {PHA, IMP, 3},
    {EOR, IMM, 2},
    {LSR, IMP, 2},
    {ALR, IMM, 2},

    {JMP, ABS, 3},
    {EOR, ABS, 4},
    {LSR, ABS, 6},
    {SRE, ABS, 6},


    {BVC, REL, 2},
    {EOR, IZY, 5},
    {STP, IMP, 0},
    {SRE, IZY, 8},

    {NOP, ZPX, 4},
    {EOR, ZPX, 4},
    {LSR, ZPX, 6},
    {SRE, ZPX, 6},

    {CLI, IMP, 2},
    {EOR, ABY, 4},
    {NOP, IMP, 2},
    {SRE, ABY, 7},

    {NOP, ABX, 4},
    {EOR, ABX, 4},
    {LSR, ABX, 7},
    {SRE, ABX, 7},


    //6x
    {RTS, IMP, 6},
    {ADC, IZX, 6},
    {STP, IMP, 0},
    {RRA, IZX, 8},

    {NOP, ZP, 3},
    {ADC, ZP, 3},
    {ROR, ZP, 5},
    {RRA, ZP, 5},

    {PLA, IMP, 4},
    {ADC, IMM, 2},
    {ROR, IMP, 2},
    {ARR, IMM, 2},

    {JMP, IND, 5},
    {ADC, ABS, 4},
    {ROR, ABS, 6},
    {RRA, ABS, 6},

    //7x
    {BVS, REL, 2},
    {ADC, IZY, 5},
    {STP, IMP, 0},
    {RRA, IZY, 8},

    {NOP, ZPX, 4},
    {ADC, ZPX, 4},
    {ROR, ZPX, 6},
    {RRA, ZPX, 6},

    {SEI, IMP, 2},
    {ADC, ABY, 4},
    {NOP, IMP, 2},
    {RRA, ABY, 7},

    {NOP, ABX, 4},
    {ADC, ABX, 4},
    {ROR, ABX, 7},
    {RRA, ABX, 7},

    //8x
    {NOP, IMM, 2},
    {STA, IZX, 6},
    {NOP, IMM, 2},
    {SAX, IZX, 6},

    {STY, ZP, 3},
    {STA, ZP, 3},
    {STX, ZP, 3},
    {SAX, ZP, 3},

    {DEY, IMP, 2},
    {NOP, IMM, 2},
    {TXA, IMP, 2},
    {XAA, IMM, 2},

    {STY, ABS, 4},
    {STA, ABS, 4},
    {STX, ABS, 4},
    {SAX, ABS, 4},

    //9x
    {BCC, REL, 2},
    {STA, IZY, 6},
    {STP, IMP, 0},
    {AHX, IZY, 6},

    {STY, ZPX, 4},
    {STA, ZPX, 4},
    {STX, ZPY, 4},
    {SAX, ZPY, 4},

    {TYA, IMP, 2},
    {STA, ABY, 5},
    {TXS, IMP, 2},
    {TAS, ABY, 5},

    {SHY, ABX, 5},
    {STA, ABX, 5},
    {SHX, ABY, 5},
    {AHX, ABY, 5},

    {LDY, IMM, 2},
    {LDA, IZX, 6},
    {LDX, IMM, 2},
    {LAX, IZX, 6},

    {LDY, ZP, 3},
    {LDA, ZP, 3},
    {LDX, ZP, 3},
    {LAX, ZP, 3},

    {TAY, IMP, 2},
    {LDA, IMM, 2},
    {TAX, IMP, 2},
    {LAX, IMM, 2},

    {LDY, ABS, 4},
    {LDA, ABS, 4},
    {LDX, ABS, 4},
    {LAX, ABS, 4},

    {BCS, REL, 2},
    {LDA, IZY, 5},
    {STP, IMP, 0},
    {LAX, IZY, 5},

    {LDY, ZPX, 4},
    {LDA, ZPX, 4},
    {LDX, ZPY, 4},
    {LAX, ZPY, 4},

    {CLV, IMP, 2},
    {LDA, ABY, 4},
    {TSX, IMP, 2},
    {LAS, ABY, 4},

    {LDY, ABX, 4},
    {LDA, ABX, 4},
    {LDX, ABY, 4},
    {LAX, ABY, 4},

    {CPY, IMM, 2},
    {CMP, IZX, 6},
    {NOP, IMM, 2},
    {DCP, IZX, 8},

    {CPY, ZP, 3},
    {CMP, ZP, 3},
    {DEC, ZP, 5},
    {DCP, ZP, 5},

    {INY, IMP, 2},
    {CMP, IMM, 2},
    {DEX, IMP, 2},
    {AXS, IMM, 2},

    {CPY, ABS, 4},
    {CMP, ABS, 4},
    {DEC, ABS, 6},
    {DCP, ABS, 6},

    {BNE, REL, 2},
    {CMP, IZY, 5},
    {STP, IMP, 0},
    {DCP, IZY, 8},

    {NOP, ZPX, 4},
    {CMP, ZPX, 4},
    {DEC, ZPX, 6},
    {DCP, ZPX, 6},

    {CLD, IMP, 2},
    {CMP, ABY, 4},
    {NOP, IMP, 2},
    {DCP, ABY, 7},

    {NOP, ABX, 4},
    {CMP, ABX, 4},
    {DEC, ABX, 7},
    {DCP, ABX, 7},

    {CPX, IMM, 2},
    {SBC, IZX, 6},
    {NOP, IMM, 2},
    {ISC, IZX, 8},

    {CPX, ZP, 3},
    {SBC, ZP, 3},
    {INC, ZP, 5},
    {ISC, ZP, 5},

    {INX, IMP, 2},
    {SBC, IMM, 2},
    {NOP, IMP, 2},
    {SBC, IMM, 2},

    {CPX, ABS, 4},
    {SBC, ABS, 4},
    {INC, ABS, 6},
    {ISC, ABS, 6},

    {BEQ, REL, 2},
    {SBC, IZY, 5},
    {STP, IMP, 0},
    {ISC, IZY, 8},

    {NOP, ZPX, 4},
    {SBC, ZPX, 4},
    {INC, ZPX, 6},
    {ISC, ZPX, 6},

    {SED, IMP, 2},
    {SBC, ABY, 4},
    {NOP, IMP, 2},
    {ISC, ABY, 7},

    {NOP, ABX, 4},
    {SBC, ABX, 4},
    {INC, ABX, 7},
    {ISC, ABX, 7}
};

typedef struct {
    uint8_t a;
    uint8_t x;
    uint8_t y;
    uint16_t pc;
    uint8_t s;
    uint8_t p;
    uint8_t cycles;
    bus_t * bus;
    uint8_t * ram;
    uint8_t inst;
    uint16_t abs;
    int8_t rel;
    uint8_t operand;
}cpu_t;

typedef struct {
    uint8_t ppu_ctrl;
    uint8_t ppu_mask;
    uint8_t ppu_status;
    uint8_t oam_addr;
    uint8_t ppu_scroll;
    uint8_t ppu_addr;
    uint8_t ppu_data;
    uint8_t oamdma;
}ppu_t;

typedef struct {
    cpu_t * cpu;
    ppu_t * ppu;
}nes_t;

int bit_get(uint8_t * byte, uint8_t bit) {
    return ((*byte)>>bit)&1;
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

bus_t * bus_create()
{
    bus_t * bus = (bus_t*)malloc(sizeof(bus_t));
    memset(bus, 0, sizeof(bus_t));
    return bus;
}

void bus_destroy(bus_t * bus)
{
    bus_device_t * mount = bus->devices;
    while (mount) {
        bus_device_t * d = mount;
        mount = mount->next;
        free(d);
    }
    free(bus);
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

uint16_t bus_read2(bus_t * bus, uint16_t address) 
{
    return (uint16_t)(bus_read(bus, address+1)<<8) | bus_read(bus, address);
}

void bus_write(bus_t * bus, uint16_t address, uint8_t data) 
{
    bus_io(bus, address, &data, 1);
}

void bus_write2(bus_t * bus, uint16_t address, uint16_t data) 
{
    uint8_t hi = data>>8;
    uint8_t lo = data & 0xff;
    bus_io(bus, address, &lo, 1);
    bus_io(bus, address + 1, &hi, 1);
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
    printf("tv system:%s\n", header.flags_10 & 3 == 0? "pal":"ntsc");
    rom->header = header;
    uint16_t prg_size = header.prg_units * 16 * 1024;
    uint16_t chr_size = header.chr_units * 8 * 1024;
    rom->prg_rom = (uint8_t*)malloc(prg_size);
    rom->chr_rom = (uint8_t*)malloc(chr_size);
    fread(rom->prg_rom, prg_size, 1, fp);
    fread(rom->chr_rom, chr_size, 1, fp);
    fclose(fp);
    return 1;
}

cpu_t * cpu_create()
{
    cpu_t * cpu = (cpu_t*)malloc(sizeof(cpu_t));
    cpu->ram = (uint8_t*)malloc(0x800);
    cpu->bus = bus_create();
    return cpu;
}

void cpu_destroy(cpu_t * cpu)
{
    bus_destroy(cpu->bus);
    free(cpu->ram);
    free(cpu);
}

void cpu_stack_push(cpu_t * cpu, uint8_t byte)
{
    bus_write(cpu->bus, cpu->s, byte);
    cpu->s --;
}
void cpu_stack_push2(cpu_t * cpu, uint16_t byte)
{
    bus_write2(cpu->bus, cpu->s, byte);
    cpu->s -= 2;
}

uint8_t cpu_stack_pop(cpu_t * cpu)
{
    return bus_read(cpu->bus, cpu->s ++);
}

uint16_t cpu_stack_pop2(cpu_t * cpu)
{
    uint16_t d = bus_read2(cpu->bus, cpu->s ++);
    cpu->s += 2;
    return d;
}
/* https://www.nesdev.org/wiki/CPU_power_up_state */
void cpu_power_up(cpu_t * cpu) 
{

    bus_mount(cpu->bus, cpu->ram, 0x0000, 0x07ff, memory_io);
    bus_mount(cpu->bus, cpu->ram, 0x0800, 0x0fff, memory_io);
    bus_mount(cpu->bus, cpu->ram, 0x1000, 0x17ff, memory_io);
    bus_mount(cpu->bus, cpu->ram, 0x1800, 0x1fff, memory_io);

    cpu->p = 0x34;
    cpu->a = 0;
    cpu->x = 0;
    cpu->y = 0;
    cpu->s = 0xFD;

    bus_write(cpu->bus, 0x4017, 0);
    bus_write(cpu->bus, 0x4015, 0);

    for (int i =0x4000 ;i <= 0x4013; ++ i) {
        bus_write(cpu->bus, i, 0);
    }
}

/* https://www.nesdev.org/NESDoc.pdf#2.4 */
void cpu_interupt(cpu_t * cpu, cpu_int_t i)
{
    if (i == IRQ && bit_get(&cpu->p,I)) {
        printf("ignore irq\n");
        return;
    }

    bit_set(&cpu->p, I);
    uint16_t vector = 0xfffa + i * 2;
    cpu_stack_push2(cpu, cpu->pc);
    cpu_stack_push(cpu, cpu->p);
    cpu->pc = bus_read2(cpu->bus, vector);
    cpu->cycles += 7;
}

uint8_t cpu_fetch(cpu_t * cpu)
{
    return bus_read(cpu->bus, cpu->pc++);
}

uint16_t cpu_fetch2(cpu_t * cpu)
{
    cpu->pc += 2;
    return bus_read2(cpu->bus, cpu->pc-2);
}

uint8_t cpu_fetch_inst(cpu_t * cpu)
{
    printf("$%04x ", cpu->pc);
    cpu->inst = cpu_fetch(cpu);

    inst_t * inst = inst_map + cpu->inst;
    cpu->cycles += inst->cycles;
    printf("%02x %s ", cpu->inst, inst_tag[inst->t], inst->am, inst->cycles);
    if (inst->am == IMP) {
    }else if (inst->am == IMM) {
        cpu->operand = cpu_fetch(cpu);
        printf("#$%02x", cpu->operand);
    }else if (inst->am == ZP) {
        uint8_t zp = cpu_fetch(cpu);
        cpu->abs = zp;
        cpu->operand = bus_read(cpu->bus, zp);
        printf("$%02x", zp);
    }else if (inst->am == ZPX) {
        uint8_t zp = cpu_fetch(cpu);
        cpu->abs = zp + cpu->x;
        cpu->operand = bus_read(cpu->bus, zp+cpu->x);
        printf("$%02x,X[%x]", zp, cpu->x);
    }else if (inst->am == ZPY) {
        uint8_t zp = cpu_fetch(cpu);
        cpu->abs = zp + cpu->y;
        cpu->operand = bus_read(cpu->bus, zp+cpu->y);
        printf("$%02x,Y[%x]", zp, cpu->y);
    }else if (inst->am == IZX) {
        uint8_t z = cpu_fetch(cpu);
        uint8_t ptr = bus_read(cpu->bus, z + cpu->x); 
        cpu->operand = bus_read(cpu->bus, ptr);
        printf("($%02x,X[%x])", z, cpu->x);
    }else if (inst->am == IZY) {
        uint8_t z = cpu_fetch(cpu);
        uint8_t ptr = bus_read(cpu->bus, z + cpu->y); 
        cpu->operand = bus_read(cpu->bus, ptr);
        printf("($%02x,Y[%x])", z, cpu->y);
    }else if (inst->am == ABS) {
        uint16_t ptr = cpu_fetch2(cpu);
        cpu->abs = ptr;
        cpu->operand = bus_read(cpu->bus, ptr);
        printf("$%04x", ptr);
    }else if (inst->am == ABX) {
        uint16_t ptr = cpu_fetch2(cpu);
        cpu->operand = bus_read(cpu->bus, ptr+cpu->x);
        printf("$%04x,X[%x]", ptr, cpu->x);
    }else if (inst->am == ABY) {
        uint16_t ptr = cpu_fetch2(cpu);
        cpu->operand = bus_read(cpu->bus, ptr+cpu->y);
        printf("$%04x,Y[%x]", ptr, cpu->y);
    }else if (inst->am == IND) {
        uint16_t ptr = cpu_fetch2(cpu);
        uint16_t real_ptr = bus_read2(cpu->bus, ptr);
        cpu->operand = bus_read(cpu->bus, real_ptr);
        printf("($%04x)", ptr);
    }else if (inst->am == REL) {
        int8_t offset = cpu_fetch(cpu);
        cpu->rel = offset;
        cpu->operand = bus_read(cpu->bus, cpu->pc - offset);
        printf("pc-%x", offset & 0xff);
    }

    return 0;
}

uint8_t cpu_exec(cpu_t * cpu)
{
    printf("\n");
    if (cpu->inst > sizeof(inst_map)/sizeof(inst_t)) {
        return 0;
    }

    inst_t * inst = inst_map + cpu->inst;
    switch(inst->t) {
        default:
            printf("un supported instruction %x\n", cpu->inst);
            break;
        case JSR:
            cpu_stack_push2(cpu, cpu->pc);
            cpu->pc = cpu->abs;
            break;
        case INC:
            bus_write(cpu->bus, cpu->abs, cpu->operand+1);
            break;
        case DEC:
            bus_write(cpu->bus, cpu->abs, cpu->operand-1);
            break;
        case LDA:
            cpu->a = bus_read(cpu->bus, cpu->abs);
            break;
        case BPL:
            cpu->pc = cpu->pc + cpu->rel * (bit_get(&cpu->p, N) == 0);
            break;
        case BRK:
            bit_clr(&cpu->p, I);
            bit_set(&cpu->p, B);
            cpu_interupt(cpu, IRQ);
            break;
        case RTI:
            cpu->p = cpu_stack_pop(cpu);
            cpu->pc = cpu_stack_pop2(cpu);
            break;
        case RTS:
            cpu->pc = cpu_stack_pop2(cpu);
            break;
        case STA:
            bus_write(cpu->bus, cpu->abs, cpu->a);
            break;
    }
    return 0;
}

void cpu_clock(cpu_t * cpu)
{
    if (cpu->cycles == 0) {

        printf("pc:$%04x a:%x x:%x y:%x N:%d V:%d B:%d D:%d I:%d Z:%d C:%d ", cpu->pc, cpu->a, cpu->x, cpu->y, 
            bit_get(&cpu->p, N),
            bit_get(&cpu->p, V),
            bit_get(&cpu->p, B),
            bit_get(&cpu->p, D),
            bit_get(&cpu->p, I),
            bit_get(&cpu->p, Z),
            bit_get(&cpu->p, C)
            );
        cpu->cycles += cpu_fetch_inst(cpu);
        cpu->cycles += cpu_exec(cpu);
    }
    cpu->cycles --;
}

ppu_t * ppu_create()
{
    ppu_t * ppu = (ppu_t*)malloc(sizeof(ppu_t));
    memset(ppu, 0, sizeof(ppu_t));

    return ppu;
}

nes_t * nes_create()
{
    nes_t * nes = (nes_t*)malloc(sizeof(nes_t));
    memset(nes, 0, sizeof(nes_t));
    nes->cpu = cpu_create();
    nes->ppu = ppu_create();
    return nes;
}

void nes_power_up(nes_t * nes)
{
    cpu_power_up(nes->cpu);
}

void nes_set_rom(nes_t * nes, rom_t * rom)
{
    if (rom->header.prg_units == 1) {
        bus_mount(nes->cpu->bus, rom->prg_rom, 0x8000, 0xbfff, memory_io);
        bus_mount(nes->cpu->bus, rom->prg_rom, 0xc000, 0xffff, memory_io);
    }else {
        bus_mount(nes->cpu->bus, rom->prg_rom, 0x8000, 0xffff, memory_io);
    }

    cpu_interupt(nes->cpu, RESET);
}

void nes_clock(nes_t * nes)
{
    cpu_clock(nes->cpu);
}

int main(int argc, char * argv[]) 
{
    /*
       if (argc < 2) {
       printf("usage: %s rom\n", argv[0]);
       return -1;
       }
       */
    char * rom_path = "rom/aa.nes";
    rom_t rom;
    if (!load_rom(rom_path, &rom)) {
        printf("load rom failed\n");
    }

    nes_t * nes = nes_create();
    nes_power_up(nes);
    nes_set_rom(nes,&rom);
    for (int i = 0 ; i < 1000; ++ i) {
        nes_clock(nes);
    }



	SDL_Init(SDL_INIT_EVERYTHING);
    SDL_Window * window = SDL_CreateWindow("nes", 300, 300, 0);
	SDL_Quit();
    return 0;
}
