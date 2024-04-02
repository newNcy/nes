#include "nes.h"
#include "rom.h"
#include <complex.h>
#include <stdatomic.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <wctype.h>


#include <SDL.h>
#include <SDL_hints.h>

#include "rom.h"
#include "mapper.h"
#ifdef I
#undef I
#endif

typedef void (*device_io_t)(void * device, uint16_t address, uint8_t * data, uint8_t is_write);

typedef struct {
    void * data;
    device_io_t  io;
}device_t;


typedef struct bus_device_t {
    uint16_t start;
    uint16_t end;
    device_t * device;
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
    uint16_t cycles;
    bus_t * bus;
    uint8_t * ram;
    uint8_t inst;
    uint16_t abs;
    int8_t rel;
    uint8_t operand;
}cpu_t;


typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
}color_t;

static color_t system_palette[] = {
    {0x75, 0x75, 0x75},
    {0x27, 0x1B, 0x8F},
    {0x00, 0x00, 0xAB},
    {0x47, 0x00, 0x9F},
    {0x8F, 0x00, 0x77},
    {0xAB, 0x00, 0x13},
    {0xA7, 0x00, 0x00},
    {0x7F, 0x0B, 0x00},
    {0x43, 0x2F, 0x00},
    {0x00, 0x47, 0x00},
    {0x00, 0x51, 0x00},
    {0x00, 0x3F, 0x17},
    {0x1B, 0x3F, 0x5F},
    {0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00},
    {0xBC, 0xBC, 0xBC},
    {0x00, 0x73, 0xEF},
    {0x23, 0x3B, 0xEF},
    {0x83, 0x00, 0xF3},
    {0xBF, 0x00, 0xBF},
    {0xE7, 0x00, 0x5B},
    {0xDB, 0x2B, 0x00},
    {0xCB, 0x4F, 0x0F},
    {0x8B, 0x73, 0x00},
    {0x00, 0x97, 0x00},
    {0x00, 0xAB, 0x00},
    {0x00, 0x93, 0x3B},
    {0x00, 0x83, 0x8B},
    {0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00},
    {0xFF, 0xFF, 0xFF},
    {0x3F, 0xBF, 0xFF},
    {0x5F, 0x97, 0xFF},
    {0xA7, 0x8B, 0xFD},
    {0xF7, 0x7B, 0xFF},
    {0xFF, 0x77, 0xB7},
    {0xFF, 0x77, 0x63},
    {0xFF, 0x9B, 0x3B},
    {0xF3, 0xBF, 0x3F},
    {0x83, 0xD3, 0x13},
    {0x4F, 0xDF, 0x4B},
    {0x58, 0xF8, 0x98},
    {0x00, 0xEB, 0xDB},
    {0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00},
    {0xFF, 0xFF, 0xFF},
    {0xAB, 0xE7, 0xFF},
    {0xC7, 0xD7, 0xFF},
    {0xD7, 0xCB, 0xFF},
    {0xFF, 0xC7, 0xFF},
    {0xFF, 0xC7, 0xDB},
    {0xFF, 0xBF, 0xB3},
    {0xFF, 0xDB, 0xAB},
    {0xFF, 0xE7, 0xA3},
    {0xE3, 0xFF, 0xA3},
    {0xAB, 0xF3, 0xBF},
    {0xB3, 0xFF, 0xCF},
    {0x9F, 0xFF, 0xF3},
    {0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00}
};

typedef enum {
    SPRITE_OVERFLOW = 5, SPRITE_0_HINT, V_BLANK,
}ppu_status_bit_t;

typedef enum {
    ADDR_INC,
}ppu_control_bit_t;

typedef struct 
{
    uint8_t y;
    uint8_t id;
    uint8_t attribute;
    uint8_t x;
}object_attribute_t;

typedef struct {
    uint8_t ppu_ctrl;
    uint8_t ppu_mask;
    uint8_t ppu_status;
    uint8_t oam_addr;
    uint8_t ppu_scroll;
    uint8_t ppu_addr;
    uint8_t ppu_data;
    uint8_t oam_dma;
    bus_t * bus;
    device_t * name_tables;
    device_t * paletters;
    device_t * registers;
    device_t * oma_registers;
    object_attribute_t oam[64];
    uint16_t cycles;
    int16_t scanline;
    uint8_t address_latch;
    uint16_t ppu_data_address;
    uint8_t ppu_data_buffer;
}ppu_t;


typedef struct {
    device_io_t cpu_io;
    device_io_t ppu_io;
}mapper_t;

typedef struct {
    cpu_t * cpu;
    ppu_t * ppu;
    device_t * cpu_mapper;
    device_t * ppu_mapper;
}nes_t;

uint8_t calc_color(color_t c)
{
    return 16 + (36 * (c.r / 51)) + (6 * (c.g / 51)) + (c.b / 51);    
}

void show_block(uint8_t code)
{
    printf("\e[48;5;%dm \e[0m", code);
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

void bus_mount(bus_t * bus, uint16_t start, uint16_t end, device_t * device)
{
    bus_device_t * mount = (bus_device_t*)malloc(sizeof(bus_device_t));
    memset(mount, 0, sizeof(bus_device_t));
    mount->start = start;
    mount->end = end;
    mount->device = device;

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
        if (mount->start <= address && address <= mount->end && mount->device) {
            uint16_t offset = address - mount->start;
            mount->device->io(mount->device->data, offset, data, is_write);
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


device_t * memory_device_create(int size)
{
    device_t * device = (device_t *) malloc(sizeof(device_t));
    device->data = malloc(size);
    device->io = memory_io;
    return device;
}

void memory_device_destroy(device_t * device)
{
    free(device->data);
    free(device);
}

device_t * device_create(void * data, device_io_t io)
{
    device_t * device = (device_t *) malloc(sizeof(device_t));
    device->data = data;
    device->io = io;
    return device;
}

void device_destroy(device_t * device)
{
    free(device);
}


cpu_t * cpu_create()
{
    cpu_t * cpu = (cpu_t*)malloc(sizeof(cpu_t));
    cpu->ram = (uint8_t*)malloc(0x800);
    cpu->ram = memory_device_create(0x800);
    cpu->bus = bus_create();

    bus_mount(cpu->bus, 0x0000, 0x07ff, cpu->ram);
    bus_mount(cpu->bus, 0x0800, 0x0fff, cpu->ram);
    bus_mount(cpu->bus, 0x1000, 0x17ff, cpu->ram);
    bus_mount(cpu->bus, 0x1800, 0x1fff, cpu->ram);

    return cpu;
}

void cpu_destroy(cpu_t * cpu)
{
    bus_destroy(cpu->bus);
    memory_device_destroy(cpu->ram);
    free(cpu);
}

void cpu_stack_push(cpu_t * cpu, uint8_t byte)
{
    cpu->s --;
    bus_write(cpu->bus, cpu->s, byte);
}
void cpu_stack_push2(cpu_t * cpu, uint16_t byte)
{
    cpu->s -= 2;
    bus_write2(cpu->bus, cpu->s, byte);
}

uint8_t cpu_stack_pop(cpu_t * cpu)
{
    return bus_read(cpu->bus, cpu->s ++);
}

uint16_t cpu_stack_pop2(cpu_t * cpu)
{
    uint16_t d = bus_read2(cpu->bus, cpu->s);
    cpu->s += 2;
    return d;
}
/* https://www.nesdev.org/wiki/CPU_power_up_state */
void cpu_power_up(cpu_t * cpu) 
{
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
    printf("[%02x %s] ", cpu->inst, inst_tag[inst->t], inst->am, inst->cycles);
    if (inst->am == IMP) {
    }else if (inst->am == IMM) {
        cpu->operand = cpu_fetch(cpu);
        printf("#$%02x", cpu->operand);
    }else if (inst->am == ZP) {
        uint8_t zp = cpu_fetch(cpu);
        cpu->abs = zp;
        cpu->operand = bus_read(cpu->bus, zp);
        printf("$%02x %x", zp, cpu->operand);
    }else if (inst->am == ZPX) {
        uint8_t zp = cpu_fetch(cpu);
        cpu->abs = zp + cpu->x;
        cpu->operand = bus_read(cpu->bus, zp+cpu->x);
        printf("$%02x,X[%x] %x", zp, cpu->x, cpu->operand);
    }else if (inst->am == ZPY) {
        uint8_t zp = cpu_fetch(cpu);
        cpu->abs = zp + cpu->y;
        cpu->operand = bus_read(cpu->bus, zp+cpu->y);
        printf("$%02x,Y[%x]", zp, cpu->y);
    }else if (inst->am == IZX) {
        uint8_t z = cpu_fetch(cpu);
        uint8_t ptr = bus_read(cpu->bus, z + cpu->x); 
        cpu->abs = ptr;
        cpu->operand = bus_read(cpu->bus, ptr);
        printf("($%02x,X[%x])", z, cpu->x);
    }else if (inst->am == IZY) {
        uint8_t z = cpu_fetch(cpu);
        uint8_t ptr = bus_read(cpu->bus, z + cpu->y); 
        cpu->abs = ptr;
        cpu->operand = bus_read(cpu->bus, ptr);
        printf("($%02x,Y[%x])", z, cpu->y);
    }else if (inst->am == ABS) {
        uint16_t ptr = cpu_fetch2(cpu);
        cpu->abs = ptr;
        cpu->operand = bus_read(cpu->bus, ptr);
        printf("$%04x", ptr);
    }else if (inst->am == ABX) {
        uint16_t ptr = cpu_fetch2(cpu);
        cpu->abs = ptr + cpu->x;
        cpu->operand = bus_read(cpu->bus, ptr+cpu->x);
        printf("$%04x,X[%x]", ptr, cpu->x);
    }else if (inst->am == ABY) {
        uint16_t ptr = cpu_fetch2(cpu);
        cpu->abs = ptr + cpu->y;
        cpu->operand = bus_read(cpu->bus, ptr+cpu->y);
        printf("$%04x,Y[%x]", ptr, cpu->y);
    }else if (inst->am == IND) {
        uint16_t ptr = cpu_fetch2(cpu);
        uint16_t real_ptr = bus_read2(cpu->bus, ptr);
        cpu->abs = real_ptr;
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

uint8_t cpu_exec_single(cpu_t * cpu)
{
    inst_t * inst = inst_map + cpu->inst;
    switch(inst->t) {
        default:
            printf(" unsupported");
            break;
        case BMI:
            cpu->pc += cpu->rel * bit_get(&cpu->p, N);
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
            bit_setv(&cpu->p, N, cpu->a & 0x80);
            bit_setv(&cpu->p, Z, !cpu->a);
            break;
        case BPL:
            cpu->pc = cpu->pc + cpu->rel * (bit_get(&cpu->p, N) == 0);
            break;
        case BRK:
            bit_clr(&cpu->p, I);
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
        case SEI:
            bit_set(&cpu->p, I);
            break;
        case LDX:
            cpu->x = cpu->operand;
            bit_setv(&cpu->p, Z, !cpu->x);
            bit_setv(&cpu->p, N, 0x8 & cpu->x);
            break;
        case TXS:
            cpu->s = cpu->x;
            break;
        case TXA:
            cpu->a = cpu->x;
            break;
        case CMP:
            bit_setv(&cpu->p, C, cpu->a >= cpu->operand);
            bit_setv(&cpu->p, Z, cpu->a == cpu->operand);
            bit_setv(&cpu->p, N, (cpu->a - cpu->operand) & 0x80);
            break;
        case BNE:
            cpu->pc = cpu->pc + cpu->rel * (bit_get(&cpu->p, Z) == 0);
            break;
        case STX:
            bus_write(cpu->bus, cpu->abs, cpu->x);
            break;
        case LDY:
            cpu->y = cpu->operand;
            bit_setv(&cpu->p, Z, !cpu->y);
            bit_setv(&cpu->p, N, 0x80 & cpu->y);
            break;
        case DEY:
            cpu->y --;
            bit_setv(&cpu->p, Z, !cpu->y);
            bit_setv(&cpu->p, N, 0x80 & cpu->y);
            break;
    }
    printf("\n");
    return 0;
}

void cpu_clock(cpu_t * cpu)
{
    if (cpu->cycles == 0) {

        printf("[%02x:%02x:%02x] [%d:%d:%d:%d:%d:%d:%d] ", cpu->a, cpu->x, cpu->y, 
            bit_get(&cpu->p, N),
            bit_get(&cpu->p, V),
            bit_get(&cpu->p, B),
            bit_get(&cpu->p, D),
            bit_get(&cpu->p, I),
            bit_get(&cpu->p, Z),
            bit_get(&cpu->p, C)
            );
        cpu->cycles += cpu_fetch_inst(cpu);
        cpu->cycles += cpu_exec_single(cpu);
    }
    cpu->cycles --;
}



void ppu_register_io(void * device, uint16_t address, uint8_t * byte, uint8_t is_write) 
{
    ppu_t * ppu = (ppu_t*)device;
    switch(address) {
        case 0:
            break;
        case 2:
            if (!is_write) {
                bit_set(&ppu->ppu_status, V_BLANK);
                *byte = (ppu->ppu_status & 0xe0) | (ppu->ppu_data_buffer & 0x1f);
                bit_clr(&ppu->ppu_status, V_BLANK);
            }
            break;
        case 3:
            if (is_write) {
            }
            break;
        case 4:
            if (!is_write) {
                *byte = ((uint8_t*)(ppu->oam))[ppu->oam_addr];
            }
            break;
        case 6:
            if (is_write) {
                if (ppu->address_latch == 0) {
                    ppu->ppu_data_address = (ppu->ppu_data_address & 0x00ff) |8 <<*byte;
                    ppu->address_latch = 1;
                }else {
                    ppu->ppu_data_address = (ppu->ppu_data_address & 0xff00) | *byte;
                    ppu->address_latch = 0;
                }
            }
            break;
        case 7:
            if (is_write) {
                bus_write(ppu->bus, ppu->ppu_data_address, *byte);
                ppu->ppu_data_address ++;
            }else {
                *byte = ppu->ppu_data_buffer;
                ppu->ppu_data_buffer = bus_read(ppu->bus, ppu->ppu_data_address);
                ppu->ppu_data_address ++ ;
            }
            break;
    }
}

void ppu_oam_register_io(void * device, uint16_t address, uint8_t * byte, uint8_t is_write) 
{
    ppu_t * ppu = (ppu_t*)device;
}


ppu_t * ppu_create()
{
    ppu_t * ppu = (ppu_t*)malloc(sizeof(ppu_t));
    memset(ppu, 0, sizeof(ppu_t));
    ppu->bus = bus_create();

    ppu->paletters = memory_device_create(0x3f20 - 0x3f00);
    bus_mount(ppu->bus, 0x3f00, 0x3f1f, ppu->paletters);
    bus_mount(ppu->bus, 0x3f20, 0x3fff, ppu->paletters);

    ppu->registers = device_create(ppu, ppu_register_io);
    ppu->oma_registers = device_create(ppu, ppu_oam_register_io);
    return ppu;
}

void * ppu_destroy(ppu_t * ppu) 
{
    bus_destroy(ppu->bus);
    memory_device_destroy(ppu->paletters);
    device_destroy(ppu->registers);
    device_destroy(ppu->oma_registers);
    free(ppu);
}


void ppu_power_up(ppu_t * ppu)
{
    bit_set(&ppu->ppu_status, SPRITE_OVERFLOW);
    bit_clr(&ppu->ppu_status, SPRITE_0_HINT);
    bit_set(&ppu->ppu_status, V_BLANK);
}

void ppu_show_pattern_table(ppu_t * ppu, uint8_t id)
{
    SDL_Window * window = SDL_CreateWindow("pattern", 256, 256, 0);

    SDL_Renderer * renderer = SDL_CreateRenderer(window, NULL, 0);
    SDL_Texture * renderBuffer = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_XRGB8888, SDL_TEXTUREACCESS_STREAMING, 128, 128);
    SDL_SetTextureScaleMode(renderBuffer, SDL_SCALEMODE_NEAREST);

    
    uint32_t color[] = {0xffffffff, 0xaaaaaaaa, 0x88888888, 0x0};
    uint32_t pixels [128*128] = {0};
    uint8_t pattern[128*128] = {0};

    for (uint16_t ty = 0 ; ty < 16; ++ ty) {
        for (uint16_t tx = 0; tx < 16; ++ tx) {
            uint16_t tile_start = id * 0x1000 + ty * 16 * 16 + tx * 16;

            for (uint16_t row = 0; row < 8; row ++)  {
                uint8_t byte = bus_read(ppu->bus, tile_start + row);
                uint8_t byte2 = bus_read(ppu->bus, tile_start + row + 8);
                for (uint16_t col = 0; col < 8; col ++)  {
                    uint8_t out = bit_get(&byte, 7-col) + bit_get(&byte2, 7-col);
                    assert(out < 4);
                    pixels[(ty * 8+row)*128 + (tx*8 + col)] = color[out];
                    pattern[(ty * 8+row)*128 + tx*8 + col] = out;                 
                }
            }
        }
    }

    for (int i = 0; i < 128; ++ i) {
        for (int j = 0; j < 128; ++ j) {
            printf("%d", pattern[i*128+j]);
        }
        printf("\n");
    }
    SDL_UpdateTexture(renderBuffer, NULL, pixels, 4*128);
    SDL_RenderTexture(renderer, renderBuffer, NULL, NULL);
    SDL_RenderPresent(renderer);
    SDL_ShowWindow(window);
}

void ppu_show_nametable()
{
}

/*
 * https://www.nesdev.org/wiki/PPU_rendering#Frame_timing_diagram
 */
void ppu_clock(ppu_t * ppu)
{
    if (ppu->scanline == -1 && ppu->cycles == 1) {
        bit_clr(&ppu->ppu_status, V_BLANK);
    }
    if (ppu->scanline == 241 && ppu->cycles == 1) {
        bit_set(&ppu->ppu_status, V_BLANK);
    }
    if (ppu->scanline >=0 
        && ppu->scanline <= 239 
        && ppu->cycles > 0 
        && ppu->cycles < 341) {

    }
}



nes_t * nes_create()
{
    nes_t * nes = (nes_t*)malloc(sizeof(nes_t));
    memset(nes, 0, sizeof(nes_t));
    cpu_t * cpu = cpu_create();
    ppu_t * ppu = ppu_create();

    for (uint16_t i = 0x2000; i <= 0x3450; i += 8) {
        bus_mount(cpu->bus, i, i+7, ppu->registers);
    }
    bus_mount(cpu->bus, 0x4014, 0x4014, ppu->oma_registers);

    nes->cpu = cpu;
    nes->ppu = ppu;
    return nes;
}

void nes_destroy(nes_t * nes)
{
    cpu_destroy(nes->cpu);
    ppu_destroy(nes->ppu);

    free(nes);
}

void nes_power_up(nes_t * nes)
{
    cpu_power_up(nes->cpu);
    ppu_power_up(nes->ppu);
}

void nes_set_rom(nes_t * nes, rom_t * rom)
{
    if (!nes->cpu_mapper) {
        nes->cpu_mapper = device_create(NULL, NULL);
    }
    if (!nes->ppu_mapper) {
        nes->ppu_mapper = device_create(NULL, NULL);
    }
    mapper_t mappers[] = {
        {mapper_0_cpu_io, mapper_0_ppu_io},
    };

    mapper_t * mapper = mappers + rom->mapper;

    nes->cpu_mapper = device_create(rom, mapper->cpu_io);
    nes->ppu_mapper = device_create(rom, mapper->ppu_io);

    bus_mount(nes->cpu->bus, 0x4020, 0xffff, nes->cpu_mapper);
    bus_mount(nes->ppu->bus, 0x0000, 0x1fff, nes->ppu_mapper);
    ppu_show_pattern_table(nes->ppu, 0);
    printf("%\n");
    ppu_show_pattern_table(nes->ppu, 1);
    cpu_interupt(nes->cpu, RESET);
}

void nes_clock(nes_t * nes)
{
    cpu_clock(nes->cpu);
    ppu_clock(nes->ppu);
    ppu_clock(nes->ppu);
    ppu_clock(nes->ppu);
}


void nes_set_output(nes_t * nes, device_t * output_device)
{

}

typedef struct {
    SDL_Window * window;
    SDL_Renderer * renderer;
    SDL_Texture * texture;
}sdl_ouput_t;

sdl_ouput_t * sdl_ouput_create()
{
    sdl_ouput_t * sdl_ouput = (sdl_ouput_t*)malloc(sizeof(sdl_ouput_t));
    sdl_ouput->window = SDL_CreateWindow("nes", 480, 512, 0);
    sdl_ouput->renderer = SDL_CreateRenderer(sdl_ouput->window, NULL, 0);
    sdl_ouput->texture = SDL_CreateTexture(sdl_ouput->renderer, SDL_PIXELFORMAT_XRGB8888, SDL_TEXTUREACCESS_STREAMING, 240, 256);
    SDL_SetTextureScaleMode(sdl_ouput->texture, SDL_SCALEMODE_NEAREST);
    SDL_ShowWindow(sdl_ouput->window);
    return sdl_ouput;
}

void sdl_ouput_destroy(sdl_ouput_t * sdl_ouput)
{
    SDL_DestroyWindow(sdl_ouput->window);
    SDL_DestroyRenderer(sdl_ouput->renderer);
    SDL_DestroyTexture(sdl_ouput->texture);

    free(sdl_ouput);
}

/*
 * https://austinmorlan.com/posts/nes_rendering_overview/
 */

int main(int argc, char * argv[]) 
{
	SDL_Init(SDL_INIT_VIDEO);

    char * rom_path = "../rom/aa.nes";
    rom_t * rom = rom_load(rom_path);
    if (!rom) {
        printf("load rom failed\n");
    }

    nes_t * nes = nes_create();
    sdl_ouput_t * sdl_ouput = sdl_ouput_create();
    device_t * output = device_create(sdl_ouput, NULL);
    nes_set_output(nes, output);
    nes_power_up(nes);
    nes_set_rom(nes, rom);


    SDL_Event event;
    int app_quit = 0;
    while (!app_quit) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_QUIT)
                app_quit = 1;
            break;
        }
        nes_clock(nes);
    }
    rom_destroy(rom);
    sdl_ouput_destroy(output);
    nes_destroy(nes);

    SDL_Quit();
    return 0;
}
