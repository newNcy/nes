#include "nes.h"
#include <stdint.h> 
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <assert.h>


#include <SDL3/SDL.h>
#include <SDL3_ttf/SDL_ttf.h>

#include "rom.h"
#include "log.h"
#include "mapper.h"
#include <time.h>

#ifdef I
#undef I
#endif

uint64_t time_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts); // or CLOCK_REALTIME
    return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

typedef void (*device_io_t)(void * device, uint16_t address, uint8_t * data, uint8_t is_write);

typedef struct {
    void * data;
    device_io_t  io;
}device_t;


typedef void (*set_pixel_t)(void * data, uint16_t x, uint16_t y, uint32_t pixel);

typedef struct 
{
    void * data;
    set_pixel_t set_pixel;
}output_t;


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

typedef struct cpu_debug_stack_frame_t 
{
    struct cpu_debug_stack_frame_t * last;
    uint16_t address;
    uint16_t return_address;
}cpu_debug_stack_frame_t;


cpu_debug_stack_frame_t * cpu_debug_stack_frame_create(cpu_debug_stack_frame_t * last, uint16_t address, uint16_t return_address)
{
    cpu_debug_stack_frame_t * frame = (cpu_debug_stack_frame_t*)malloc(sizeof(cpu_debug_stack_frame_t));
    frame->last = last;
    frame->address = address;
    frame->return_address = return_address;
    return frame;
}

void cpu_debug_stack_frame_destroy(cpu_debug_stack_frame_t * frame)
{
    free(frame);
}

void cpu_debug_stack_frame_unwind(cpu_debug_stack_frame_t * frame, char * buff)
{
    if (!frame) {
        return;
    }
    cpu_debug_stack_frame_unwind(frame->last, buff);
    if (frame->last) {
        char tmp[50] = {0};
        sprintf(tmp, "...%04X>>", frame->return_address);
        strcat(buff,tmp);
    }
    char tmp[50] = {0};
    sprintf(tmp, "%04X", frame->address);

    strcat(buff, tmp);
}

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

static char * address_mode_name[] = {
    "IMP", "IMM", "ZP", "ZP+X", "ZP+Y", "IZX", "IZY", "ABS", "ABX", "ABY", "IND", "REL"
};

/* http://www.oxyron.de/html/opcodes02.html */ 
static inst_t inst_map[] = {
    {BRK, IMP, 0x07},      // 0x00
    {ORA, IZX, 0x06},      // 0x01
    {STP, IMP, 0x00},      // 0x02
    {SLO, IZX, 0x08},      // 0x03

    {NOP, ZP, 0x03},       // 0x04
    {ORA, ZP, 0x03},       // 0x05
    {ASL, ZP, 0x05},       // 0x06
    {SLO, ZP, 0x05},       // 0x07

    {PHP, IMP, 0x03},      // 0x08
    {ORA, IMM, 0x02},      // 0x09
    {ASL, IMP, 0x02},      // 0x0A
    {ANC, IMM, 0x02},      // 0x0B

    {NOP, ABS, 0x04},      // 0x0C
    {ORA, ABS, 0x04},      // 0x0D
    {ASL, ABS, 0x06},      // 0x0E
    {SLO, ABS, 0x06},      // 0x0F

    {BPL, REL, 0x02},      // 0x10
    {ORA, IZY, 0x05},      // 0x11
    {STP, IMP, 0x00},      // 0x12
    {SLO, IZY, 0x08},      // 0x13

    {NOP, ZPX, 0x04},      // 0x14
    {ORA, ZPX, 0x04},      // 0x15
    {ASL, ZPX, 0x06},      // 0x16
    {SLO, ZPX, 0x06},      // 0x17

    {CLC, IMP, 0x02},      // 0x18
    {ORA, ABY, 0x04},      // 0x19
    {NOP, IMP, 0x02},      // 0x1A
    {SLO, ABY, 0x07},      // 0x1B

    {NOP, ABX, 0x04},      // 0x1C
    {ORA, ABX, 0x04},      // 0x1D
    {ASL, ABX, 0x07},      // 0x1E
    {SLO, ABX, 0x07},      // 0x1F

    {JSR, ABS, 0x06},      // 0x20
    {AND, IZX, 0x06},      // 0x21
    {STP, IMP, 0x00},      // 0x22
    {RLA, IZX, 0x08},      // 0x23

    {BIT, ZP, 0x03},       // 0x24
    {AND, ZP, 0x03},       // 0x25
    {ROL, ZP, 0x05},       // 0x26
    {RLA, ZP, 0x05},       // 0x27

    {PLP, IMP, 0x04},      // 0x28
    {AND, IMM, 0x02},      // 0x29
    {ROL, IMP, 0x02},      // 0x2A
    {ANC, IMM, 0x02},      // 0x2B

    {BIT, ABS, 0x04},      // 0x2C
    {AND, ABS, 0x04},      // 0x2D
    {ROL, ABS, 0x06},      // 0x2E
    {RLA, ABS, 0x06},      // 0x2F

    {BMI, REL, 0x02},      // 0x30
    {AND, IZY, 0x05},      // 0x31
    {STP, IMP, 0x00},      // 0x32
    {RLA, IZY, 0x08},      // 0x33

    {NOP, ZPX, 0x04},      // 0x34
    {AND, ZPX, 0x04},      // 0x35
    {ROL, ZPX, 0x06},      // 0x36
    {RLA, ZPX, 0x06},      // 0x37

    {SEC, IMP, 0x02},      // 0x38
    {AND, ABY, 0x04},      // 0x39
    {NOP, IMP, 0x02},      // 0x3A
    {RLA, ABY, 0x07},      // 0x3B

    {NOP, ABX, 0x04},      // 0x3C
    {AND, ABX, 0x04},      // 0x3D
    {ASL, ABX, 0x07},      // 0x3E
    {SLO, ABX, 0x07},      // 0x3F

    {RTI, IMP, 0x06},      // 0x40
    {EOR, IZX, 0x06},      // 0x41
    {STP, IMP, 0x00},      // 0x42
    {SRE, IZX, 0x08},      // 0x43

    {NOP, ZP, 0x03},       // 0x44
    {EOR, ZP, 0x03},       // 0x45
    {LSR, ZP, 0x05},       // 0x46
    {SRE, ZP, 0x05},       // 0x47

    {PHA, IMP, 0x03},      // 0x48
    {EOR, IMM, 0x02},      // 0x49
    {LSR, IMP, 0x02},      // 0x4A
    {ALR, IMM, 0x02},      // 0x4B

    {JMP, ABS, 0x03},      // 0x4C
    {EOR, ABS, 0x04},      // 0x4D
    {LSR, ABS, 0x06},      // 0x4E
    {SRE, ABS, 0x06},      // 0x4F

    {BVC, REL, 0x02},      // 0x50
    {EOR, IZY, 0x05},      // 0x51
    {STP, IMP, 0x00},      // 0x52
    {SRE, IZY, 0x08},      // 0x53

    {NOP, ZPX, 0x04},      // 0x54
    {EOR, ZPX, 0x04},      // 0x55
    {LSR, ZPX, 0x06},      // 0x56
    {SRE, ZPX, 0x06},      // 0x57

    {CLI, IMP, 0x02},      // 0x58
    {EOR, ABY, 0x04},      // 0x59
    {NOP, IMP, 0x02},      // 0x5A
    {SRE, ABY, 0x07},      // 0x5B

    {NOP, ABX, 0x04},      // 0x5C
    {EOR, ABX, 0x04},      // 0x5D
    {LSR, ABX, 0x07},      // 0x5E
    {SRE, ABX, 0x07},      // 0x5F

    {RTS, IMP, 0x06},      // 0x60
    {ADC, IZX, 0x06},      // 0x61
    {STP, IMP, 0x00},      // 0x62
    {RRA, IZX, 0x08},      // 0x63

    {NOP, ZP, 0x03},       // 0x64
    {ADC, ZP, 0x03},       // 0x65
    {ROR, ZP, 0x05},       // 0x66
    {RRA, ZP, 0x05},       // 0x67

    {PLA, IMP, 0x04},      // 0x68
    {ADC, IMM, 0x02},      // 0x69
    {ROR, IMP, 0x02},      // 0x6A
    {ARR, IMM, 0x02},      // 0x6B

    {JMP, IND, 0x05},      // 0x6C
    {ADC, ABS, 0x04},      // 0x6D
    {ROR, ABS, 0x06},      // 0x6E
    {RRA, ABS, 0x06},      // 0x6F

    {BVS, REL, 0x02},      // 0x70
    {ADC, IZY, 0x05},      // 0x71
    {STP, IMP, 0x00},      // 0x72
    {RRA, IZY, 0x08},      // 0x73

    {NOP, ZPX, 0x04},      // 0x74
    {ADC, ZPX, 0x04},      // 0x75
    {ROR, ZPX, 0x06},      // 0x76
    {RRA, ZPX, 0x06},      // 0x77

    {SEI, IMP, 0x02},      // 0x78
    {ADC, ABY, 0x04},      // 0x79
    {NOP, IMP, 0x02},      // 0x7A
    {RRA, ABY, 0x07},      // 0x7B

    {NOP, ABX, 0x04},      // 0x7C
    {ADC, ABX, 0x04},      // 0x7D
    {ROR, ABX, 0x07},      // 0x7E
    {RRA, ABX, 0x07},      // 0x7F

    {NOP, IMM, 0x02},      // 0x80
    {STA, IZX, 0x06},      // 0x81
    {NOP, IMM, 0x02},      // 0x82
    {SAX, IZX, 0x06},      // 0x83

    {STY, ZP, 0x03},       // 0x84
    {STA, ZP, 0x03},       // 0x85
    {STX, ZP, 0x03},       // 0x86
    {SAX, ZP, 0x03},       // 0x87

    {DEY, IMP, 0x02},      // 0x88
    {NOP, IMM, 0x02},      // 0x89
    {TXA, IMP, 0x02},      // 0x8A
    {XAA, IMM, 0x02},      // 0x8B

    {STY, ABS, 0x04},      // 0x8C
    {STA, ABS, 0x04},      // 0x8D
    {STX, ABS, 0x04},      // 0x8E
    {SAX, ABS, 0x04},      // 0x8F

    {BCC, REL, 0x02},      // 0x90
    {STA, IZY, 0x06},      // 0x91
    {STP, IMP, 0x00},      // 0x92
    {AHX, IZY, 0x06},      // 0x93

    {STY, ZPX, 0x04},      // 0x94
    {STA, ZPX, 0x04},      // 0x95
    {STX, ZPY, 0x04},      // 0x96
    {SAX, ZPY, 0x04},      // 0x97

    {TYA, IMP, 0x02},      // 0x98
    {STA, ABY, 0x05},      // 0x99
    {TXS, IMP, 0x02},      // 0x9A
    {TAS, ABY, 0x05},      // 0x9B

    {SHY, ABX, 0x05},      // 0x9C
    {STA, ABX, 0x05},      // 0x9D
    {SHX, ABY, 0x05},      // 0x9E
    {AHX, ABY, 0x05},      // 0x9F

    {LDY, IMM, 0x02},      // 0xA0
    {LDA, IZX, 0x06},      // 0xA1
    {LDX, IMM, 0x02},      // 0xA2
    {LAX, IZX, 0x06},      // 0xA3

    {LDY, ZP, 0x03},       // 0xA4
    {LDA, ZP, 0x03},       // 0xA5
    {LDX, ZP, 0x03},       // 0xA6
    {LAX, ZP, 0x03},       // 0xA7

    {TAY, IMP, 0x02},      // 0xA8
    {LDA, IMM, 0x02},      // 0xA9
    {TAX, IMP, 0x02},      // 0xAA
    {LAX, IMM, 0x02},      // 0xAB

    {LDY, ABS, 0x04},      // 0xAC
    {LDA, ABS, 0x04},      // 0xAD
    {LDX, ABS, 0x04},      // 0xAE
    {LAX, ABS, 0x04},      // 0xAF

    {BCS, REL, 0x02},      // 0xB0
    {LDA, IZY, 0x05},      // 0xB1
    {STP, IMP, 0x00},      // 0xB2
    {LAX, IZY, 0x05},      // 0xB3

    {LDY, ZPX, 0x04},      // 0xB4
    {LDA, ZPX, 0x04},      // 0xB5
    {LDX, ZPY, 0x04},      // 0xB6
    {LAX, ZPY, 0x04},      // 0xB7

    {CLV, IMP, 0x02},      // 0xB8
    {LDA, ABY, 0x04},      // 0xB9
    {TSX, IMP, 0x02},      // 0xBA
    {LAS, ABY, 0x04},      // 0xBB

    {LDY, ABX, 0x04},      // 0xBC
    {LDA, ABX, 0x04},      // 0xBD
    {LDX, ABY, 0x04},      // 0xBE
    {LAX, ABY, 0x04},      // 0xBF

    {CPY, IMM, 0x02},      // 0xC0
    {CMP, IZX, 0x06},      // 0xC1
    {NOP, IMM, 0x02},      // 0xC2
    {DCP, IZX, 0x08},      // 0xC3

    {CPY, ZP, 0x03},       // 0xC4
    {CMP, ZP, 0x03},       // 0xC5
    {DEC, ZP, 0x05},       // 0xC6
    {DCP, ZP, 0x05},       // 0xC7

    {INY, IMP, 0x02},      // 0xC8
    {CMP, IMM, 0x02},      // 0xC9
    {DEX, IMP, 0x02},      // 0xCA
    {AXS, IMM, 0x02},      // 0xCB

    {CPY, ABS, 0x04},      // 0xCC
    {CMP, ABS, 0x04},      // 0xCD
    {DEC, ABS, 0x06},      // 0xCE
    {DCP, ABS, 0x06},      // 0xCF

    {BNE, REL, 0x02},      // 0xD0
    {CMP, IZY, 0x05},      // 0xD1
    {STP, IMP, 0x00},      // 0xD2
    {DCP, IZY, 0x08},      // 0xD3

    {NOP, ZPX, 0x04},      // 0xD4
    {CMP, ZPX, 0x04},      // 0xD5
    {DEC, ZPX, 0x06},      // 0xD6
    {DCP, ZPX, 0x06},      // 0xD7

    {CLD, IMP, 0x02},      // 0xD8
    {CMP, ABY, 0x04},      // 0xD9
    {NOP, IMP, 0x02},      // 0xDA
    {DCP, ABY, 0x07},      // 0xDB

    {NOP, ABX, 0x04},      // 0xDC
    {CMP, ABX, 0x04},      // 0xDD
    {DEC, ABX, 0x07},      // 0xDE
    {DCP, ABX, 0x07},      // 0xDF

    {CPX, IMM, 0x02},      // 0xE0
    {SBC, IZX, 0x06},      // 0xE1
    {NOP, IMM, 0x02},      // 0xE2
    {ISC, IZX, 0x08},      // 0xE3

    {CPX, ZP, 0x03},       // 0xE4
    {SBC, ZP, 0x03},       // 0xE5
    {INC, ZP, 0x05},       // 0xE6
    {ISC, ZP, 0x05},       // 0xE7

    {INX, IMP, 0x02},      // 0xE8
    {SBC, IMM, 0x02},      // 0xE9
    {NOP, IMP, 0x02},      // 0xEA
    {SBC, IMM, 0x02},      // 0xEB

    {CPX, ABS, 0x04},      // 0xEC
    {SBC, ABS, 0x04},      // 0xED
    {INC, ABS, 0x06},      // 0xEE
    {ISC, ABS, 0x06},      // 0xEF

    {BEQ, REL, 0x02},      // 0xF0
    {SBC, IZY, 0x05},      // 0xF1
    {STP, IMP, 0x00},      // 0xF2
    {ISC, IZY, 0x08},      // 0xF3

    {NOP, ZPX, 0x04},      // 0xF4
    {SBC, ZPX, 0x04},      // 0xF5
    {INC, ZPX, 0x06},      // 0xF6
    {ISC, ZPX, 0x06},      // 0xF7

    {SED, IMP, 0x02},      // 0xF8
    {SBC, ABY, 0x04},      // 0xF9
    {NOP, IMP, 0x02},      // 0xFA
    {ISC, ABY, 0x07},      // 0xFB

    {NOP, ABX, 0x04},      // 0xFC
    {SBC, ABX, 0x04},      // 0xFD
    {INC, ABX, 0x07},      // 0xFE
    {ISC, ABX, 0x07}       // 0xFF
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
    device_t * ram;
    inst_t * inst;
    uint16_t inst_addr;
    uint16_t base;
    uint16_t abs;
    int8_t rel;
    uint32_t debug_ins_count;
    cpu_debug_stack_frame_t * debug_call_stack;
    int8_t stoped;
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


typedef struct 
{
    uint8_t y;
    uint8_t id;
    uint8_t attribute;
    uint8_t x;
}object_attribute_t;

typedef struct {
    uint8_t vram_addr_inc;
    uint16_t base_nametable_addrress;
    uint16_t sprite_pattern_table_address;
    uint16_t background_pattern_table_address;
    uint8_t sprite_size;
    uint8_t v_blank_nmi;
    uint16_t v, t;
    uint8_t x, w; // v, t, x, w for scroll and ppu vram accessing
    uint8_t reg_mask;
    uint8_t reg_status;
    uint8_t reg_oam_addr;
    uint8_t reg_oam_data;
    uint8_t reg_scroll;
    uint16_t reg_ppu_addr;
    uint8_t reg_ppu_data;
    uint8_t reg_oam_dma;
    uint8_t nmi;

    bus_t * bus;
    device_t * name_tables;
    device_t * paletters;
    device_t * registers;
    object_attribute_t oam[64];
    uint16_t cycles;
    int16_t scanline;

    uint8_t nt; 
    uint8_t at; 
    /* 为什么分2次，因为一行的8个像素是由两个字节的第i位合起来表示的 */
    uint8_t pattern_0;
    uint8_t pattern_1;
    output_t * output;
}ppu_t;


typedef struct {
    device_io_t cpu_io;
    device_io_t ppu_io;
}mapper_t;

typedef struct {
    cpu_t * cpu;
    ppu_t * ppu;
    device_t * oma_registers; // 虽然是ppu上的，不过往这里写入时是从cpu传到ppu，要放这里方便同时访问
    device_t * cpu_mapper;
    device_t * ppu_mapper;
}nes_t;

uint8_t calc_color(color_t c)
{
    return 16 + (36 * (c.r / 51)) + (6 * (c.g / 51)) + (c.b / 51);    
}

void show_block(uint8_t code)
{
    nes_log("\e[48;5;%dm \e[0m", code);
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
    int found = 0;
    while (mount) {
        if (mount->start <= address && address <= mount->end && mount->device) {
            uint16_t offset = address - mount->start;
            mount->device->io(mount->device->data, offset, data, is_write);
            found = 1;
            break;
        }

        mount = mount->next;
    }

    if (!found) {
        printf("bus io to address %04X failed\n", address);
    }
}

static inline uint8_t bus_read(bus_t * bus, uint16_t address) 
{
    uint8_t ret = 0;
    bus_io(bus, address, &ret, 0);
    return ret;
}

static inline uint16_t bus_read2(bus_t * bus, uint16_t address) 
{
    return ( ((uint16_t)bus_read(bus, address+1))<<8) | bus_read(bus, address);
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
    memset(device->data, 0, size);
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

output_t * output_create(void * data, set_pixel_t set_pixel)
{
    output_t * output = (output_t*)malloc(sizeof(output_t));
    output->data = data;
    output->set_pixel = set_pixel;
    return output;
}

void output_destroy(output_t * output)
{
    free(output);
}


cpu_t * cpu_create()
{
    cpu_t * cpu = (cpu_t*)malloc(sizeof(cpu_t));
    memset(cpu, 0, sizeof(cpu_t));
    cpu->ram = memory_device_create(0x800);
    cpu->bus = bus_create();
    cpu->debug_call_stack = NULL;

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
    bus_write(cpu->bus, 0x100 + cpu->s, byte);
    cpu->s --;
}

void cpu_stack_push2(cpu_t * cpu, uint16_t byte)
{
    cpu_stack_push(cpu, byte >> 8);
    cpu_stack_push(cpu, byte & 0xff);
}

uint8_t cpu_stack_pop(cpu_t * cpu)
{
    cpu->s ++;
    uint8_t byte = bus_read(cpu->bus, 0x100 + cpu->s);
    return byte;
}

uint16_t cpu_stack_pop2(cpu_t * cpu)
{
    uint8_t lo = cpu_stack_pop(cpu);
    uint16_t data = cpu_stack_pop(cpu);
    data <<= 8;
    data |= lo;
    return data;
}
/* https://www.nesdev.org/wiki/CPU_power_up_state */
void cpu_power_up(cpu_t * cpu) 
{
    bit_clr(&cpu->p, C);
    bit_clr(&cpu->p, Z);
    bit_clr(&cpu->p, D);
    bit_clr(&cpu->p, V);
    bit_clr(&cpu->p, N);
    bit_set(&cpu->p, I);
    cpu->pc = 0xfffc;
    cpu->a = 0;
    cpu->x = 0;
    cpu->y = 0;
    cpu->s = 0xFF;
    cpu->debug_ins_count = 0;
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
        return;
    }
    printf("cpu_interupt %d\n", i);

    bit_set(&cpu->p, I);
    uint16_t vector = 0xfffa + i * 2;
    cpu_stack_push2(cpu, cpu->pc);
    cpu_stack_push(cpu, cpu->p);
    cpu->pc = bus_read2(cpu->bus, vector);
    cpu->cycles += 7;
}

static inline uint8_t cpu_fetch(cpu_t * cpu)
{
    return bus_read(cpu->bus, cpu->pc++);
}

static inline uint16_t cpu_fetch2(cpu_t * cpu)
{
    cpu->pc += 2;
    return bus_read2(cpu->bus, cpu->pc-2);
}

static inline uint8_t cpu_fetch_data(cpu_t * cpu)
{
    return bus_read(cpu->bus, cpu->abs);
}

static inline void cpu_write_data(cpu_t * cpu, uint8_t data)
{
    bus_write(cpu->bus, cpu->abs, data);
}

/*
 * https://www.nesdev.org/wiki/CPU_addressing_modes
 */
uint8_t cpu_fetch_inst(cpu_t * cpu)
{
    cpu->inst_addr = cpu->pc;

    uint8_t inst_code = cpu_fetch(cpu);
    inst_t * inst = cpu->inst = &inst_map[inst_code];
    cpu->cycles += inst->cycles;
    if (inst->am == IMM) {
        cpu->abs = cpu->pc;
        cpu->pc ++;
    }else if (inst->am == ZP) {
        uint8_t zp = cpu_fetch(cpu);
        cpu->abs = zp;
    }else if (inst->am == ZPX) {
        cpu->base = cpu_fetch(cpu);
        cpu->abs = cpu->base + cpu->x;
    }else if (inst->am == ZPY) {
        cpu->base = cpu_fetch(cpu);
        cpu->abs = cpu->base + cpu->y;
    }else if (inst->am == IZX) {
        cpu->base = cpu_fetch(cpu);
        cpu->abs = bus_read2(cpu->bus, cpu->base + cpu->x); 
    }else if (inst->am == IZY) {
        cpu->base = cpu_fetch(cpu);
        uint16_t ptr = bus_read2(cpu->bus, cpu->base) + cpu->y; 
        cpu->abs = ptr;
    }else if (inst->am == ABS) {
        uint16_t ptr = cpu_fetch2(cpu);
        cpu->abs = ptr;
    }else if (inst->am == ABX) {
        cpu->base = cpu_fetch2(cpu);
        cpu->abs = cpu->base + cpu->x;
    }else if (inst->am == ABY) {
        cpu->base = cpu_fetch2(cpu);
        cpu->abs = cpu->base + cpu->y;
    }else if (inst->am == IND) {
        cpu->base = cpu_fetch2(cpu);
        uint16_t real_ptr = bus_read2(cpu->bus, cpu->base);
        cpu->abs = real_ptr;
    }else if (inst->am == REL) {
        cpu->rel = cpu_fetch(cpu);
        cpu->abs = cpu->pc + cpu->rel;
    }
    return 0;
}

static inline void cpu_status_set_zn(cpu_t * cpu, uint8_t res)
{
    bit_set_value(&cpu->p, Z, res == 0);
    bit_set_value(&cpu->p, N, res & 0x80);
}

/* 
 * 设置上下溢出标志 
 * */
static inline void cpu_status_set_v(cpu_t * cpu, uint8_t operand1, uint8_t operand2, uint8_t res)
{
    bit_set_value(&cpu->p, V, (~((operand1&0x80) ^ (operand2&0x80))) & ((res & 0x80) ^ (operand1 & 0x80)));
}

static inline void cpu_status_set_c(cpu_t * cpu, uint16_t res)
{
    bit_set_value(&cpu->p, C, res > 0xff);
}

void cpu_debug_push_frame(cpu_t * cpu)
{
    cpu_debug_stack_frame_t * frame = cpu_debug_stack_frame_create(cpu->debug_call_stack, cpu->abs, cpu->pc);
    cpu->debug_call_stack = frame;
}

void cpu_debug_pop_frame(cpu_t * cpu)
{
    if (cpu->debug_call_stack) {
        cpu_debug_stack_frame_t * cur = cpu->debug_call_stack;
        cpu->debug_call_stack = cur->last;
        cpu_debug_stack_frame_destroy(cur);
    }
}

int cpu_branch(cpu_t * cpu, uint8_t value) {
    if (value) {
        cpu->pc = cpu->abs;
        return 1;
    }
    return 0;
}

void cpu_compare(cpu_t * cpu, uint8_t v) 
{
    uint16_t res = v - cpu_fetch_data(cpu) + bit_get(&cpu->p, C);
    cpu_status_set_zn(cpu, res);
    cpu_status_set_c(cpu, res);
}

int cpu_opps(cpu_t * cpu)
{
    addressing_mode_t am = cpu->inst->am;
    if ((am == ABX || am == ABY || am == IZY) && (0xff00 & cpu->base) != (0xff00&cpu->abs)) {
        return 1;
    }
    return 0;
}

int cpu_op_ror(cpu_t * cpu)
{
    uint8_t operand = cpu_fetch_data(cpu);
    bit_set_value(&cpu->p, C, operand & 0x1);
    operand >>= 1;
    operand |= bit_get(&cpu->p, C) << 7;
    cpu_write_data(cpu, operand);
    return 0;
}

int cpu_op_adc(cpu_t * cpu)
{
    uint8_t operand = cpu_fetch_data(cpu);
    uint16_t res = cpu->a + operand + bit_get(&cpu->p, C);
    cpu_status_set_v(cpu, cpu->a, operand, res);
    cpu_status_set_c(cpu, res);
    cpu_status_set_zn(cpu, res);
    cpu->a = res;
    return cpu_opps(cpu);
}

void format_inst(cpu_t * cpu, char * buff) 
{
    inst_t * inst = cpu->inst;
    sprintf(buff, "%04X (%02X) %s %s ", cpu->inst_addr, inst - inst_map, inst_tag[inst->t], address_mode_name[inst->am]);
    char temp[20] = {0};
    if (inst->am == IMM) {
        uint8_t data = bus_read(cpu->bus, cpu->abs);
        sprintf(temp, "#0x%02X(%d)", data, data);
    }else if (inst->am == ZP || inst->am == ABS) {
        sprintf(temp, "0x%04X", cpu->abs);
    }else if (inst->am == ZPX) {
        sprintf(temp, "0x%04X(0x%02X+%02x)", cpu->abs, cpu->base, cpu->x);
    }else if (inst->am == ZPY) {
        sprintf(temp, "0x%04X(0x%02X+%02x)", cpu->abs, cpu->base, cpu->y);
    }else if (inst->am == IZX) {
        sprintf(temp, "0x%04X([0x%02X+%02x])", cpu->abs, cpu->base, cpu->x);
    }else if (inst->am == IZY) {
        sprintf(temp, "0x%04X([0x%02X]+%02x)", cpu->abs, cpu->base, cpu->y);
    }else if (inst->am == REL) {
        sprintf(temp, "%04X (%04X+(%d))", cpu->abs, cpu->pc , cpu->rel);
    }else if (inst->am == ABX) {
        sprintf(temp, "0x%04X(0x%04X + %02X)", cpu->abs, cpu->base, cpu->x );
    } else if (inst->am == ABY) {
        sprintf(temp, "0x%04X(0x%04X + %02X)", cpu->abs, cpu->base, cpu->y );
    }else if (inst->am == IND) {
        sprintf(temp, "0x%04X([0x%04X])", cpu->abs, cpu->base);
    }
    strcat(buff, temp);
}



/*
 * https://www.oxyron.de/html/opcodes02.html
 */
uint8_t cpu_exec_single(cpu_t * cpu)
{
    inst_t * inst = cpu->inst;
    int8_t operand = 0;
    uint16_t res = 0;
    uint8_t add_cycle = 0;

    char buff[100] = {0};
    format_inst(cpu, buff);
    printf("%s\n", buff);
    switch(inst->t) {
        //Logical and arithmetic commands
        case ORA:
            cpu->a = cpu->a | cpu_fetch_data(cpu);
            cpu_status_set_zn(cpu, cpu->a);
            add_cycle = cpu_opps(cpu);
            break;
        case AND:
            cpu->a &= cpu_fetch_data(cpu);
            cpu_status_set_zn(cpu, cpu->a);
            add_cycle = cpu_opps(cpu);
            break;
        case EOR:
            cpu->a = cpu->a ^ cpu_fetch_data(cpu);
            cpu_status_set_zn(cpu, cpu->a);
            add_cycle = cpu_opps(cpu);
            break;
        case ADC:
            add_cycle = cpu_op_adc(cpu);
            break;
        case SBC:
            operand = cpu_fetch_data(cpu);
            res = cpu->a - operand - (1 - bit_get(&cpu->p, C));
            cpu_status_set_v(cpu, operand, cpu->a, res);
            cpu_status_set_c(cpu, res);
            cpu_status_set_zn(cpu, res);
            cpu->a = res;
            break;
        case CMP:
            cpu_compare(cpu, cpu->a);
            break;
        case CPX:
            cpu_compare(cpu, cpu->x);
            break;
        case CPY:
            cpu_compare(cpu, cpu->y);
            break;
        case DEC:
            operand = cpu_fetch_data(cpu) - 1;
            cpu_status_set_zn(cpu, operand);
            cpu_write_data(cpu, operand);
            break;
        case DEX:
            cpu->x --;
            cpu_status_set_zn(cpu, cpu->x);
            break;
        case DEY:
            cpu->y --;
            cpu_status_set_zn(cpu, cpu->y);
            break;
        case INC:
            operand = cpu_fetch_data(cpu) + 1;
            cpu_status_set_zn(cpu, operand);
            cpu_write_data(cpu, operand);
            break;
        case INX:
            cpu->x ++;
            cpu_status_set_zn(cpu, cpu->x);
            break;
        case INY:
            cpu->y ++;
            cpu_status_set_zn(cpu, cpu->y);
            break;
        case ASL:
            operand = cpu_fetch_data(cpu);
            bit_set_value(&cpu->p, C, operand & 0x80);
            operand <<= 1;
            cpu_status_set_zn(cpu, operand);
            cpu_write_data(cpu, operand);
            break;
        case ROL:
            operand = cpu_fetch_data(cpu);
            bit_set_value(&cpu->p, C, operand& 0x80);
            operand <<= 1 ;
            operand |= bit_get(&cpu->p, C);
            cpu_write_data(cpu, operand);
            break;
        case LSR:
            operand = cpu_fetch_data(cpu);
            bit_set_value(&cpu->p, C, operand & 0x1);
            operand >>= 1;
            cpu_status_set_zn(cpu, operand);
            cpu_write_data(cpu, operand);
            break;
        case ROR:
            add_cycle = cpu_op_ror(cpu);
            break;
        //Move commands
        case LDA:
            cpu->a = cpu_fetch_data(cpu);
            cpu_status_set_zn(cpu, cpu->a);
            break;
        case STA:
            cpu_write_data(cpu, cpu->a);
            break;
        case LDX:
            cpu->x = cpu_fetch_data(cpu);
            cpu_status_set_zn(cpu, cpu->x);
            break;
        case STX:
            cpu_write_data(cpu, cpu->x);
            break;
        case LDY:
            cpu->y = cpu_fetch_data(cpu);
            cpu_status_set_zn(cpu, cpu->y);
            break;
        case STY:
            cpu_write_data(cpu, cpu->x);
            break;
        case TAX:
            cpu->x = cpu->a;
            cpu_status_set_zn(cpu, cpu->x);
            break;
        case TXA:
            cpu->a = cpu->x;
            cpu_status_set_zn(cpu, cpu->a);
            break;
        case TAY:
            cpu->y = cpu->a;
            cpu_status_set_zn(cpu, cpu->y);
            break;
        case TYA:
            cpu->a = cpu->y;
            cpu_status_set_zn(cpu, cpu->a);
            break;
        case TSX:
            cpu->x = cpu->s;
            cpu_status_set_zn(cpu, cpu->x);
            break;
        case TXS:
            cpu->s = cpu->x;
            break;
        case PLA:
            cpu->a = cpu_stack_pop(cpu);
            cpu_status_set_zn(cpu, cpu->a);
            break;
        case PHA:
            cpu_stack_push(cpu, cpu->a);
            break;
        case PLP:
            operand = bit_get(&cpu->p, B);
            cpu->p = cpu_stack_pop(cpu);
            cpu->p |= operand;
            break;
        case PHP:
            cpu_stack_push(cpu, cpu->p);
            break;
        //ump/Flag commands
        case BPL:
            add_cycle = cpu_branch(cpu, bit_get(&cpu->p, N) == 0);
            break;
        case BMI:
            add_cycle = cpu_branch(cpu, bit_get(&cpu->p, N) == 1);
            break;
        case BVC:
            add_cycle = cpu_branch(cpu, bit_get(&cpu->p, V) == 0);
            break;
        case BVS:
            add_cycle = cpu_branch(cpu, bit_get(&cpu->p, V) == 1);
            break;
        case BCC:
            add_cycle = cpu_branch(cpu, bit_get(&cpu->p, C) == 0);
            break;
        case BCS:
            add_cycle = cpu_branch(cpu, bit_get(&cpu->p, C) == 1);
            break;
        case BNE:
            add_cycle = cpu_branch(cpu, bit_get(&cpu->p, Z) == 0);
            break;
        case BEQ:
            add_cycle = cpu_branch(cpu, bit_get(&cpu->p, Z) == 1);
            break;
        case BRK:
            bit_set(&cpu->p, B);
            cpu_interupt(cpu, IRQ);
            break;
        case RTI:
            cpu->p = cpu_stack_pop(cpu);
            cpu->pc = cpu_stack_pop2(cpu);
            break;
        case JSR:
            cpu_debug_push_frame(cpu);
            cpu_stack_push2(cpu, cpu->pc);
            cpu->pc = cpu->abs;
            break;
        case RTS:
            cpu_debug_pop_frame(cpu);
            cpu->pc = cpu_stack_pop2(cpu);
            break;
        case JMP:
            cpu->pc = cpu->abs;
            break;
        case BIT:
            operand = cpu_fetch_data(cpu);
            bit_set_value(&cpu->p, N, bit_get((uint8_t*)&operand, 7));
            bit_set_value(&cpu->p, V, bit_get((uint8_t*)&operand, 6));
            bit_set_value(&cpu->p, Z, (cpu->a & operand) == 0);
            break;
        case CLC:
            bit_clr(&cpu->p, C);
            break;
        case SEC:
            bit_set(&cpu->p, C);
            break;
        case CLD:
            bit_clr(&cpu->p, D);
            break;
        case SED:
            bit_set(&cpu->p, D);
            break;
        case CLI:
            bit_clr(&cpu->p, I);
            break;
        case SEI:
            bit_set(&cpu->p, I);
            break;
        case CLV:
            bit_clr(&cpu->p, I);
            break;
        case NOP: 
            add_cycle = cpu_opps(cpu);
            break; 
        //Illegal opcodes
        case SLO: //asl + or
            operand = cpu_fetch_data(cpu);
            bit_set_value(&cpu->p, C, operand & 0x80); 
            operand = operand<<1;
            cpu_write_data(cpu, operand);

            cpu->a |= operand;
            cpu_status_set_zn(cpu, cpu->a);
            break;
        case RLA:
            operand = cpu_fetch_data(cpu);
            bit_set_value(&cpu->p, C, operand & 0x80); 
            operand = (operand<<1) | bit_get(&cpu->p, C);
            cpu_write_data(cpu, operand);
            cpu->a &= operand;
            cpu_status_set_zn(cpu, cpu->a);
            break;
        case SRE:
            operand = cpu_fetch_data(cpu);
            bit_set_value(&cpu->p, C, operand & 0x1);
            operand <<= 1;
            cpu->a ^= operand;
            cpu_status_set_zn(cpu, cpu->a);
            break;
        case RRA:
            add_cycle = cpu_op_ror(cpu) + cpu_op_adc(cpu);
            break;
        case SAX:
            cpu_write_data(cpu, cpu->a & cpu->x);
            break;
        case LAX:
            cpu->a = cpu->x = cpu_fetch_data(cpu);
            cpu_status_set_zn(cpu, cpu->a);
            break;
        case DCP:
            operand = cpu_fetch_data(cpu) - 1;
            cpu_write_data(cpu, operand);
            bit_set_value(&cpu->p, C, cpu->a >= operand) ;
            cpu_status_set_zn(cpu, cpu->a - operand);
            break;
        case STP:
            cpu->stoped = 1;
            break;
        default:
            printf("unsupported %s\n", inst_tag[inst->t]);
            break;
    }

    cpu->cycles += add_cycle;
    cpu->inst = NULL;
    return 0;
}


void cpu_debug_dump_stack(cpu_t * cpu, char * buff)
{
    nes_log(" ");
    for (uint16_t addr = 0xfe; addr > cpu->s; -- addr) {
        uint8_t byte = bus_read(cpu->bus, addr + 0x100);
        nes_log("%02x ", byte);
    }
}

void cpu_debug_show_detail(cpu_t * cpu)
{
    nes_log("%d [A=%02x,X=%02x,Y=%02x,S=%02x] [%c%c%c%c%c%c%c] ", 
            cpu->debug_ins_count,
            cpu->a, cpu->x, cpu->y, cpu->s,
            bit_get(&cpu->p, N) ? 'N':'-',
            bit_get(&cpu->p, V) ? 'V':'-',
            bit_get(&cpu->p, B) ? 'B':'-',
            bit_get(&cpu->p, D) ? 'D':'-',
            bit_get(&cpu->p, I) ? 'I':'-',
            bit_get(&cpu->p, Z) ? 'Z':'-',
            bit_get(&cpu->p, C) ? 'C':'-'
          );
    inst_t * inst = cpu->inst;
    nes_log("$%04x %02x[%s %s %02x(%d)]", cpu->inst_addr, cpu->inst, inst_tag[inst->t], address_mode_name[inst->am], inst->am == IMP ? 0 : cpu->abs, inst->am == REL ? cpu->rel: 0);
    //cpu_debug_stack_frame_unwind(cpu->debug_call_stack);
    //cpu_debug_dump_stack(cpu);
    nes_log("\n");
}

void cpu_clock(cpu_t * cpu)
{
    if (cpu->stoped) {
        return;
    }
    if (cpu->cycles == 0) {
        if (!cpu->inst) {
            cpu_fetch_inst(cpu);
        }else {
            cpu_exec_single(cpu);
        }
    }else {
        cpu->cycles --;
    }
}

void cpu_dump_prg(cpu_t * cpu, uint16_t from, uint8_t count)
{
    uint16_t pc = cpu->pc;
    while(count --) {
        cpu_fetch_inst(cpu);
    }
    cpu->pc = pc;
}


static inline void ppu_vt_set_nametable_idx(uint16_t * reg, uint8_t idx)
{
    *reg |= (idx&0x3) << 15;
}

static inline void ppu_vt_set_coarse_x(uint16_t * reg, uint8_t x)
{
    *reg |= x&0x1f;
}
static inline void ppu_vt_set_coarse_y(uint16_t * reg, uint8_t y)
{
    *reg |= (y&0x1f) << 5;
}
static inline void ppu_vt_set_fine_y(uint16_t * reg, uint8_t y)
{
    *reg |= (y & 0x7) << 12;
}

static inline uint8_t ppu_vt_get_nametable_idx(uint16_t * reg)
{
    return (*reg>>15) & 0x3;
}
static inline uint8_t ppu_vt_get_coarse_x(uint16_t * reg)
{
    return *reg & 0x1f;
}
static inline uint8_t ppu_vt_get_coarse_y(uint16_t * reg)
{
    return (*reg>>5) & 0x1f;
}

static inline uint8_t ppu_vt_get_fine_y(uint16_t * reg)
{
    return (*reg >> 12) & 0x7;
}

/* 
 * https://www.nesdev.org/wiki/PPU_registers 
 * https://www.nesdev.org/wiki/PPU_scrolling
 */

void ppu_register_io(void * device, uint16_t address, uint8_t * byte, uint8_t is_write) 
{
    ppu_t * ppu = (ppu_t*)device;
    if (is_write) {
        switch(address) {
            case 0: {
                        ppu->base_nametable_addrress = 0x2000 + 0x400 * (*byte & 0x3);
                        ppu->vram_addr_inc = 1 + 31*bit_get(byte, 2);
                        ppu->sprite_pattern_table_address = 0x1000 * bit_get(byte, 3);
                        ppu->background_pattern_table_address = 0x10000 * bit_get(byte, 4);
                        ppu->sprite_size = 8 + 8*bit_get(byte, 5);
                        ppu->v_blank_nmi = bit_get(byte, 7);
                        if (ppu->v_blank_nmi) {
                            ppu->nmi = 1;
                        }
                    }
                    break;
            case 1: ppu->reg_mask = *byte; break;
            case 3: ppu->reg_oam_addr = *byte; break;
            case 4: memory_io( ppu->oam, ppu->reg_oam_addr, byte, is_write); ppu->reg_oam_addr ++; break;
            case 5: 
                    if (!ppu->w) {
                        ppu->x = *byte & 0x7;
                        ppu_vt_set_coarse_x(&ppu->t, ppu->reg_scroll>>3);
                    }else {
                        ppu_vt_set_fine_y(&ppu->t, ppu->reg_scroll & 0x7);
                        ppu_vt_set_coarse_y(&ppu->t, ppu->reg_scroll >> 3);
                    }
                    ppu->w = !ppu->w;
                    break;
            case 6: 
                    if (!ppu->w) {
                        ppu->reg_ppu_addr  = *byte << 8;
                    }else {
                        ppu->reg_ppu_addr  = ppu->reg_ppu_addr | *byte;
                    }
                    ppu->w = !ppu->w;
                    break;
            case 7:
                    bus_io(ppu->bus, ppu->reg_ppu_addr, byte, is_write);
                    ppu->reg_ppu_addr += ppu->vram_addr_inc;
                    break;
        }
    }else {
        switch(address) {
            case 2:
                *byte = ppu->reg_status;
                ppu->w = 0;
                break;
            case 4:
                memory_io(ppu->oam, address, byte, is_write);
                break;
            case 7:
                bus_io(ppu->bus, ppu->reg_ppu_addr, byte, is_write);
                ppu->reg_ppu_addr += ppu->vram_addr_inc;
                break;
        }
    }
}

void nes_oam_register_io(void * device, uint16_t address, uint8_t * byte, uint8_t is_write) 
{
    nes_t * nes = (nes_t*)device;
    uint16_t cpu_base = (uint16_t)*byte << 8;
    for (uint16_t offset = 0x00; offset <= 0xff; ++ offset) {
        uint8_t data = bus_read(nes->cpu->bus, cpu_base + offset);
        memory_io(nes->ppu->oam, offset, &data, 1);
    }
}


ppu_t * ppu_create()
{
    ppu_t * ppu = (ppu_t*)malloc(sizeof(ppu_t));
    memset(ppu, 0, sizeof(ppu_t));
    ppu->bus = bus_create();

    ppu->paletters = memory_device_create(0x3f20 - 0x3f00);
    bus_mount(ppu->bus, 0x3f00, 0x3f1f, ppu->paletters);
    bus_mount(ppu->bus, 0x3f20, 0x3fff, ppu->paletters);

    ppu->name_tables = memory_device_create(0x2800 - 0x2000);
    bus_mount(ppu->bus, 0x2000, 0x2800-1, ppu->name_tables);

    ppu->registers = device_create(ppu, ppu_register_io);
    return ppu;
}

void ppu_destroy(ppu_t * ppu) 
{
    bus_destroy(ppu->bus);
    memory_device_destroy(ppu->paletters);
    memory_device_destroy(ppu->name_tables);
    device_destroy(ppu->registers);
    free(ppu);
}


void ppu_power_up(ppu_t * ppu)
{
    ppu->reg_status = 0;
    bit_set(&ppu->reg_status, V_BLANK);
    ppu->reg_mask = 0;

    ppu->scanline = 261;
    ppu->cycles = 0;
}

SDL_Texture * create_pattern_table_texture(ppu_t * ppu, SDL_Renderer * renderer, int id)
{
    SDL_Texture * renderBuffer = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_XRGB8888, SDL_TEXTUREACCESS_STREAMING, 128, 128);
    SDL_SetTextureScaleMode(renderBuffer, SDL_SCALEMODE_NEAREST);

    uint32_t color[] = {0xffff0000, 0xff00ff00, 0xff0000ff, 0x0};
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

    SDL_UpdateTexture(renderBuffer, NULL, pixels, 4*128);
    return renderBuffer;
}

void ppu_show_nametable(ppu_t * ppu)
{
    uint16_t nametable_address = 0x2000;
    for (int i = 0; i < 30; ++ i) {
        for (int j = 0; j < 32; ++ j) {
            uint8_t tile_idx = bus_read(ppu->bus, nametable_address + i*30 + j);
            nes_log("%02x ", tile_idx);
        }
        nes_log("\n");
    }
}


void ppu_debug_show_detail(ppu_t * ppu)
{
    nes_log("%d%d%d%d %d%d%d%d\n", 
            bit_get(&ppu->reg_status, 7),
            bit_get(&ppu->reg_status, 6),
            bit_get(&ppu->reg_status, 5),
            bit_get(&ppu->reg_status, 4),
            bit_get(&ppu->reg_status, 3),
            bit_get(&ppu->reg_status, 2),
            bit_get(&ppu->reg_status, 1),
            bit_get(&ppu->reg_status, 0)
            );
            if (bit_get(&ppu->reg_status, 5)) {
                getchar();
            }
}

void ppu_produce_bg_pixel(ppu_t * ppu) 
{
}

/*
 * https://www.nesdev.org/wiki/PPU_rendering#Frame_timing_diagram
 * 一帧有261行，一行341个tick
 * 作为模拟器，可以任意一个时间做完大部分事，剩下的tick做必须在那个期间才做的
 * -1/261 是预扫描行
 *  0-239 可见行
 *  240 后扫描行
 *  241-261 垂直同步(扫描枪头从底部返回顶部）
 */
void ppu_clock(ppu_t * ppu)
{
    uint16_t v = ppu->v;
    /* pre-scanline */
    if (ppu->scanline == 261 && ppu->cycles == 0) {
        bit_clr(&ppu->reg_status, V_BLANK);
        printf("clear v blank\n");
        /* 257 复制水平位置相关的bits */
        if (ppu->cycles >= 280 && ppu->cycles <= 304) {
            v |= ppu->t & 0x7be0;
        }
    }
    

    if (ppu->cycles >= 1 && ppu->cycles <= 256 ) {
        /* 4次访存 每次2 周期 */
        switch((ppu->cycles)% 8) {
            /* 
             * v = NN YYYYY XXXXX
             * NN 最低位是第 10 位，1<<10 = 2^10 = 1024 正好是一个nametable大小
             * YYYYY最低位第 5，1<<5  = 2 ^ 5 = 32 正好一行
             * 所以直接用低12位索引就能得到tile
             * https://www.nesdev.org/wiki/PPU_scrolling#Wrapping_around
             * Tile and attribute fetching
             */
            case 2: ppu->nt = bus_read(ppu->bus, 0x2000 | (v & 0x0fff)); break;
            case 4: ppu->at = bus_read(ppu->bus, 0x2300 | (v & 0x0C00) | ((v >> 4) & 0x38) | ((v >> 2) & 0x07)); break;
            case 6: ppu->pattern_0 = bus_read(ppu->bus, ppu->background_pattern_table_address  + ppu->nt * 0x10); break;
            case 8: ppu->pattern_1 = bus_read(ppu->bus, ppu->background_pattern_table_address  + ppu->nt* 0x10 + 8); break;
        }
        /* visible scanline */
        if (ppu->scanline >= 0 && ppu->scanline <= 239 && ((ppu->cycles)%8 == 0)) {
            if ((v & 0x001F) == 31){ // if coarse X == 31
                v &= ~0x001F;          // coarse X = 0
                v ^= 0x0400;           // switch horizontal nametable
            }
            else {
                v += 1;                // increment coarse X
            }
        }
    }

    /* increase Y and wrapping around*/
    else if (ppu->cycles == 256) {
        if ((v & 0x7000) != 0x7000)        // if fine Y < 7
            v += 0x1000;                     // increment fine Y
        else {
            v &= ~0x7000;                     // fine Y = 0
            int y = (v & 0x03E0) >> 5;        // let y = coarse Y
            if (y == 29){
                y = 0 ;                         // coarse Y = 0
                v ^= 0x0800 ;                   // switch vertical nametable
            } else if (y == 31) {
                y = 0;                         // coarse Y = 0, nametable not switched
            }
            else 
                y += 1 ;                        // increment coarse Y
            v = (v & ~0x03E0) | (y << 5);     
        }
    }else if (ppu->cycles == 257) {
        v |= ppu->t & 0x041f;
    }
 

    ppu->v = v;

    /* vertical blank*/
    if (ppu->scanline == 241 && ppu->cycles == 1) {
        bit_set(&ppu->reg_status, V_BLANK);
        printf("set v blank\n");
        if (ppu->v_blank_nmi) {
            ppu->nmi = 1;
        }
    }

    /* 261 * 340 cycles wrap */
    ppu->cycles  = (ppu->cycles + 1)%341;
    if (ppu->cycles == 0) {
        ppu->scanline = (ppu->scanline + 1)%262; 
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

    nes->oma_registers = device_create(nes, nes_oam_register_io);
    bus_mount(cpu->bus, 0x4014, 0x4014, nes->oma_registers);

    nes->cpu = cpu;
    nes->ppu = ppu;
    return nes;
}

void nes_destroy(nes_t * nes)
{
    if (nes->cpu_mapper) {
        device_destroy(nes->cpu_mapper);
    }
    if (nes->ppu_mapper) {
        device_destroy(nes->ppu_mapper);
    }
    device_destroy(nes->oma_registers);
    cpu_destroy(nes->cpu);
    ppu_destroy(nes->ppu);

    free(nes);
}

void nes_power_up(nes_t * nes)
{
    cpu_power_up(nes->cpu);
    ppu_power_up(nes->ppu);
}


void nes_clock(nes_t * nes)
{
    cpu_clock(nes->cpu);
    for (int i = 0; i < 3; ++ i) {
        ppu_clock(nes->ppu);
        if (nes->ppu->nmi) {
            cpu_interupt(nes->cpu, NMI);
            nes->ppu->nmi = 0;
        }
    }
}

void nes_until_inst(nes_t * nes) 
{
    while (!nes->cpu->inst) {
        nes_clock(nes);
    }
}

void nes_until_exec(nes_t * nes) 
{
    int clock = 0;
    char buff[100] = {0};
    while (nes->cpu->inst) {
        nes_clock(nes);
        clock ++;
    }
}

void nes_step(nes_t * nes)
{
    assert(nes->cpu->inst);
    nes_until_exec(nes);
    nes_until_inst(nes);
}

void nes_line(nes_t * nes) {
    int line = nes->ppu->scanline;
    while (line == nes->ppu->scanline) {
        nes_clock(nes);
    }
}
void nes_frame(nes_t * nes) {
    int line = nes->ppu->scanline;
    do {
        nes_clock(nes);
    }while(nes->ppu->scanline == line);
    do {
        nes_clock(nes);
    }while(nes->ppu->scanline != line);
}

void nes_reset(nes_t * nes) 
{
    nes->cpu->stoped = 0;
    cpu_interupt(nes->cpu, RESET);
    nes_until_inst(nes);
}

void nes_set_rom(nes_t * nes, rom_t * rom)
{
    if (nes->cpu_mapper) {
        device_destroy(nes->cpu_mapper);
    }
    if (nes->ppu_mapper) {
        device_destroy(nes->ppu_mapper);
    }
    mapper_t mappers[] = {
        {mapper_0_cpu_io, mapper_0_ppu_io},
    };

    mapper_t * mapper = mappers + rom->mapper;

    nes->cpu_mapper = device_create(rom, mapper->cpu_io);
    nes->ppu_mapper = device_create(rom, mapper->ppu_io);

    bus_mount(nes->cpu->bus, 0x4020, 0xffff, nes->cpu_mapper);
    bus_mount(nes->ppu->bus, 0x0000, 0x1fff, nes->ppu_mapper);

    nes_reset(nes);
}


void nes_set_output(nes_t * nes, output_t * output_device)
{
    nes->ppu->output = output_device;
}

typedef struct {
    uint32_t * pixels;
    uint16_t weight;
    uint16_t height;
}sdl_output_t;

sdl_output_t * sdl_output_create(uint16_t weight, uint16_t height)
{
    sdl_output_t * sdl_output = (sdl_output_t*)malloc(sizeof(sdl_output_t));
    return sdl_output;
}

void sdl_output_destroy(sdl_output_t * sdl_output)
{
    free(sdl_output->pixels);

    free(sdl_output);
}

void sdl_output_set_pixel(sdl_output_t * sdl_output, uint16_t x, uint16_t y, uint32_t pixel)
{
    sdl_output->pixels[y * sdl_output->weight + x] = pixel | 0xff000000;
    //SDL_UpdateTexture(sdl_output->texture, NULL, sdl_output->pixels, sizeof(pixel) * sdl_output->weight);
    //SDL_RenderTexture(sdl_output->renderer, sdl_output->texture, NULL, NULL);
}

/*
 * https://austinmorlan.com/posts/nes_rendering_overview/
 */
void show_text(SDL_Renderer * renderer, TTF_Font * font, int x, int y, SDL_Color color, char * text) 
{
    SDL_Surface * surf = TTF_RenderText_Blended(font, text, 0, color);
    SDL_Texture * texture = SDL_CreateTextureFromSurface(renderer, surf);
    SDL_SetTextureScaleMode(texture, SDL_SCALEMODE_NEAREST);

    SDL_FRect dst = {x, y, texture->w, texture->h};
    SDL_RenderTexture(renderer, texture, NULL, &dst);
    SDL_DestroySurface(surf);
    SDL_DestroyTexture(texture);
    //SDL_RenderPresent(renderer);
}

typedef struct 
{
    nes_t * nes_ref;
    SDL_Texture * pattern_0;
    SDL_Texture * pattern_1;
    TTF_Font * font_ref;
    SDL_Color white, gray;
    int p0_x, p0_y;
    int p1_x, p1_y;
    int zp_x, zp_y;
    int p_x, p_y;
    int s_x, s_y;
    int pp_x, pp_y;
    int mem_page;
}debug_info_t;

debug_info_t * create_debug_info(nes_t * nes, SDL_Renderer * renderer)
{
    debug_info_t * info = (debug_info_t*)malloc(sizeof(debug_info_t));
    memset(info, 0, sizeof(debug_info_t));
    info->pattern_0 = create_pattern_table_texture(nes->ppu, renderer, 0);
    info->pattern_1 = create_pattern_table_texture(nes->ppu, renderer, 1);
    info->nes_ref = nes;
    return info;
}

void destroy_debug_info(debug_info_t * info) 
{
    SDL_DestroyTexture(info->pattern_0);
    SDL_DestroyTexture(info->pattern_1);
    free(info);
}

void debug_show_ram(SDL_Renderer * renderer, TTF_Font * font, cpu_t * cpu, debug_info_t * info) {
    uint16_t start = info->mem_page * 0x100;
    uint16_t end = (info->mem_page +1) * 0x100;
    for (uint16_t b = start; b < end; b += 0x10 ) {
        char line[1024] = {0};
        sprintf(line, "%04X", b);
        for (u_int16_t i = 0x00 ; i<= 0xf ; i ++) {
            uint8_t byte = bus_read(cpu->bus, b + i);
            char temp2[4] = {0};
            sprintf(temp2, " %02X", byte);
            strcat(line, temp2);
        }
        SDL_Color white = {0xff, 0xff, 0xff};
        show_text(renderer, font, info->zp_x, info->zp_y + (b-start) / 0x10 * 15, white,  line);
    }
}

void debug_show_stack(SDL_Renderer * renderer, TTF_Font * font, cpu_t * cpu, int x, int y) 
{
    for (uint16_t s = 0xff ; s > cpu->s; -- s) {
        char line[1024] = {0};
        uint8_t byte = bus_read(cpu->bus, 0x0100 | s);
        sprintf(line, "%04X %02X", 0x100|s, byte);
        SDL_Color white = {0xff, 0xff, 0xff};
        show_text(renderer, font, x, y + (0xff-s) * 15, white,  line);
    }
}



void debug_show_cpu(SDL_Renderer * renderer, debug_info_t * info) 
{
    TTF_Font * font = info->font_ref;
    cpu_t * cpu = info->nes_ref->cpu;
    debug_show_ram(renderer, font, cpu, info);
    debug_show_stack(renderer, font, cpu, 512 + 256 + 320, 0);
    int x = info->p_x;
    int y = info->p_y;
    char buff [256] = {0};
    sprintf(buff, "A=%02X X=%02X Y=%02X  S=%02X pc=%04X", cpu->a, cpu->x, cpu->y, cpu->s, cpu->pc);
    SDL_Color white = {0xff, 0xff, 0xff};
    SDL_Color gray = {0x60, 0x60, 0x60};
    int space = 15;
    show_text(renderer, font, x, y, white, buff);
    show_text(renderer, font, x, y + space, bit_get(&cpu->p, N)? white : gray, "N");
    show_text(renderer, font, x + 18, y + space, bit_get(&cpu->p, V)? white : gray, "V");
    show_text(renderer, font, x + 36, y + space, bit_get(&cpu->p, B)? white : gray, "B");
    show_text(renderer, font, x + 54, y + space, bit_get(&cpu->p, D)? white : gray, "D");
    show_text(renderer, font, x + 72, y + space, bit_get(&cpu->p, I)? white : gray, "I");
    show_text(renderer, font, x + 90, y + space, bit_get(&cpu->p, Z)? white : gray, "Z");
    show_text(renderer, font, x + 108, y + space, bit_get(&cpu->p, C)? white : gray, "C");

    memset(buff, 0, 100);
    inst_t * inst = cpu->inst;
    
    format_inst(cpu, buff);
    show_text(renderer, font, x, y + 2*space, white, buff);

    for (uint16_t i = 1; i < 5; ++ i) {
        uint16_t addr = cpu->inst_addr + i;
        char buff[16] = {0};
        uint8_t byte = bus_read(cpu->bus, addr);
        sprintf(buff, "%04X %02X", addr, byte);
        show_text(renderer, font, x, y + space*(2+i), white, buff);
    }
    memset(buff, 0, 100);
    cpu_debug_stack_frame_unwind(cpu->debug_call_stack, buff);
    if (strlen(buff)) {
        show_text(renderer, font, x, y + space*7, white, buff);
    }
}



void debug_show_ppu(SDL_Renderer * renderer, debug_info_t * info) 
{

    TTF_Font * font = info->font_ref;
    ppu_t * ppu = info->nes_ref->ppu;
    SDL_Color white = {0xff, 0xff, 0xff};

    {
        char buff [100] = {0};
        sprintf(buff, "scanline=%03d:%03d", ppu->scanline, ppu->cycles);
        show_text(renderer, info->font_ref, info->pp_x, info->pp_y, white, buff);
    }
    {
        char buff [100] = {0};
        sprintf(buff, "V=%d VNMI=%d", bit_get(&ppu->reg_status, V_BLANK), ppu->v_blank_nmi);
        show_text(renderer, info->font_ref, info->pp_x, info->pp_y + 15, white, buff);
    }
    {
        char buff [100] = {0};
        sprintf(buff, "reg_oam_addr=%02X", ppu->reg_oam_addr);
        show_text(renderer, info->font_ref, info->pp_x, info->pp_y + 2*15, white, buff);
    }
    {
        char buff [100] = {0};
        sprintf(buff, "ppu_addr=%04X", ppu->reg_ppu_addr);
        show_text(renderer, info->font_ref, info->pp_x, info->pp_y + 3*15, white, buff);
    }
    {
        char buff [100] = {0};
        sprintf(buff, "background_pattern_address=%04X", ppu->background_pattern_table_address);
        show_text(renderer, info->font_ref, info->pp_x, info->pp_y + 4*15, white, buff);
    }
}

void debug_view(debug_info_t * info, SDL_Renderer * renderer) 
{
    SDL_FRect p0 = {info->p0_x, info->p0_y, 128, 128};
    SDL_FRect p1 = {info->p1_x, info->p1_y, 128, 128};
    SDL_RenderTexture(renderer, info->pattern_0, NULL, &p0);
    SDL_RenderTexture(renderer, info->pattern_1, NULL, &p1);
    debug_show_cpu(renderer, info);
    debug_show_ppu(renderer, info);
}


int main(int argc, char * argv[]) 
{
    SDL_Init(SDL_INIT_VIDEO);
    int ret = TTF_Init();
    printf("ret %d\n", ret);

    //run_nes("../rom/aa.nes")

    int pattern = 128;
    int zp = 310;
    int stack = 80;
    int w = 512 + pattern* 2 + zp + stack, h = 480;
    SDL_Window * window = SDL_CreateWindow("nes", w, h, 0);
    SDL_Renderer * renderer = SDL_CreateRenderer(window, NULL); 
    TTF_Font *font = TTF_OpenFont("../fonts/Roboto-Bold.ttf", 12); 
    SDL_ShowWindow(window);


    char * rom_path = "../rom/sm.nes";
    rom_t * rom = rom_load(rom_path);

    nes_t * nes = nes_create();
    nes_set_logfile("nes.log");
    sdl_output_t * sdl_output = sdl_output_create(256, 240);
    output_t * output = output_create(sdl_output, (set_pixel_t)sdl_output_set_pixel);
    nes_set_output(nes, output);
    nes_power_up(nes);
    nes_set_rom(nes, rom);

    debug_info_t * debug_info  = create_debug_info(nes, renderer);
    debug_info->font_ref = font;
    debug_info->white = (SDL_Color){0xff, 0xff, 0xff};
    debug_info->gray = (SDL_Color){0x60, 0x60, 0x60};
    debug_info->p0_x = 512;
    debug_info->p1_x = 512 + 128;
    debug_info->zp_x = 512 + 256 + 10;
    debug_info->p_x = 512;
    debug_info->p_y = 128+128;
    debug_info->pp_x = 512;
    debug_info->pp_y = 128;


    SDL_Event event;
    int app_quit = 0;

    int step = 1;
    while (!app_quit) {
        uint64_t s = time_ns();
        SDL_SetRenderDrawColor(renderer, 0x0, 0, 0x0, 255);
        
        SDL_RenderClear(renderer);

        cpu_t * cpu = nes->cpu;
        if (!step) {
            nes_frame(nes);
        } 
        
        nes_until_inst(nes);
        debug_view(debug_info, renderer);
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_QUIT)
                app_quit = 1;
            else if (event.type == SDL_EVENT_KEY_DOWN) {
                int k = event.key.key;
                if (event.key.key == SDLK_S) {
                    step = !step;
                }else if (event.key.key == SDLK_N && step) {
                    nes_step(nes);
                }else if (event.key.key == SDLK_R) {
                    nes_reset(nes);
                }else if (k == SDLK_M) {
                    debug_info->mem_page  = (debug_info->mem_page + 1) % 8;
                }else if (k == SDLK_L) {
                    nes_line(nes);
                }else if (k == SDLK_F) {
                    nes_frame(nes);
                }
            }
            break;
        }
        {
            uint64_t e = time_ns();
            SDL_Color white = {0xff, 0xff, 0xff};
            char buff [50] = {0};
            sprintf(buff, "%lld", ((uint64_t)(e-s))/1000000);

            //show_text(renderer, font, 0, 0, white, buff);
        }
        SDL_RenderPresent(renderer);
        SDL_Delay(10);
    }


    destroy_debug_info(debug_info);
    rom_destroy(rom);
    sdl_output_destroy(sdl_output);
    device_destroy(output);
    nes_destroy(nes);

    SDL_DestroyWindow(window);
    TTF_CloseFont(font);
    TTF_Quit();
    SDL_Quit();
    return 0;
}
