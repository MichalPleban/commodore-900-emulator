/* emu.h — Commodore C900 classic (instruction-level) emulator.
 *
 * Zilog Z8001 CPU + Z8010 MMU, boot ROM, DRAM, WD2010-style hard disk (flat
 * file, DMA memcpy), and a Z8030 SCC serial console redirected to stdin/stdout.
 * No video, no GUI, no FDC.
 */
#ifndef EMU_H
#define EMU_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/* ─────────────────────────── FCW flag bits ─────────────────────────── */
#define F_C   0x0080u  /* Carry */
#define F_Z   0x0040u  /* Zero */
#define F_S   0x0020u  /* Sign */
#define F_PV  0x0010u  /* Parity(byte)/Overflow(word,long) */
#define F_DA  0x0008u  /* Decimal Adjust */
#define F_H   0x0004u  /* Half carry */
#define F_MASK (F_C|F_Z|F_S|F_PV|F_DA|F_H)

#define FCW_SEG  0x8000u /* Segmented mode (Z8001) */
#define FCW_SN   0x4000u /* System/Normal (1=system) */
#define FCW_EPA  0x2000u /* Extended processor */
#define FCW_VIE  0x1000u /* Vectored interrupt enable */
#define FCW_NVIE 0x0800u /* Non-vectored interrupt enable */

/* ─────────────────────────── ALU ops ─────────────────────────── */
typedef enum {
    OpAdd, OpAdc, OpSub, OpSbc, OpAnd, OpOr, OpXor, OpCom, OpNeg,
    OpTest, OpCp, OpInc, OpDec, OpSla, OpSra, OpSll, OpSrl, OpRl, OpRr,
    OpRlc, OpRrc, OpDab, OpExts, OpLd, OpBit, OpSet, OpRes, OpTset
} AluOp;

/* widths (byte=1, word=2, long=4) */
typedef enum { WByte = 1, WWord = 2, WLong = 4 } Width;

typedef struct { uint64_t value; uint16_t flags; uint16_t mask; } AluResult;
AluResult alu_exec(AluOp op, uint64_t a, uint64_t b, bool carry_in, Width w);

/* ─────────────────────────── decode ───────────────────────────
 * The decoder is data-driven: every opcode resolves to a CtrlWord (below) whose
 * fields tell the execute engine which handler class to run and where its
 * operands come from. The enums here name the choices those fields can take.
 * This keeps execute() a compact switch on InstrClass rather than one branch
 * per opcode. */

/* InstrClass — which execute handler runs the instruction. Each value maps to
 * a case in cpu.c's execute dispatch (arithmetic/logic = ClsUnified, control
 * transfers = ClsBranch, the four block families, MMU/CPU-control ops, etc.). */
typedef enum {
    ClsUnified, ClsFcwOp, ClsExts, ClsStack, ClsBranch, ClsTCC, ClsDAB,
    ClsExchange, ClsBCDRotate, ClsBitDynamic, ClsLEA, ClsIO, ClsLDCTL,
    ClsShift, ClsBlockTransfer, ClsBlockCompare, ClsBlockString, ClsBlockIO,
    ClsLDM, ClsMultiProc, ClsMul, ClsDiv, ClsLDPS, ClsHalt, ClsSysCall,
    ClsIRET, ClsTranslate
} InstrClass;

typedef enum {
    BranchJP_DA, BranchJP_X, BranchJP_IR, BranchJR, BranchCALL_DA,
    BranchCALL_X, BranchCALL_IR, BranchCALR, BranchCALR12, BranchRET, BranchDJNZ
} BranchMode;

/* FcwOpKind — how the flag-control instructions (SETFLG/RESFLG/COMFLG,
 * LDCTLB FCW-low) manipulate the FCW low byte. */
typedef enum { FcwSet, FcwClear, FcwToggle, FcwLoadLo, FcwStoreLo } FcwOpKind;

/* StackMode — the PUSH/POP variants: word vs long (L), implicit vs
 * source/destination register, and immediate push. */
typedef enum {
    StackPush, StackPop, StackPushL, StackPopL, StackPushSrc, StackPopDst,
    StackPushLSrc, StackPopLDst, StackPushImm
} StackMode;

/* SrcSel — where the second operand comes from: a register, an immediate, an
 * indirect-register (IR) / direct-address (DA) / indexed (X) / based / based-
 * indexed memory operand, a PC-relative target, or a synthesized constant
 * (0/1/2, the nib3 count field, etc.). Drives operand fetch in execute(). */
typedef enum {
    SrcNone, SrcReg, SrcImm, SrcIR, SrcDA, SrcX, SrcOne, SrcTwo, SrcNib3Cnt,
    SrcZero, SrcImm2, SrcPCRel, SrcBased, SrcBX, SrcNib3Bit, SrcNib3CntPlus1,
    SrcOpLoByte
} SrcSel;

/* DstSel — where the result is written: a register, memory (IR/DA/X/based/
 * based-indexed), flags only (compares), or nowhere. */
typedef enum {
    DstReg, DstNone, DstFlagsOnly, DstIR, DstDA, DstX, DstPCRel, DstBased, DstBX
} DstSel;

/* RegSel — which opcode nibble names a register operand: nib3=op&0xF,
 * nib2=(op>>4)&0xF, nib1=(op>>8)&0xF. */
typedef enum { RegNib3, RegNib2, RegNib1 } RegSel;

/* MPOpKind — multi-micro-processor control ops (MBIT/MRES/MSET/MREQ). */
typedef enum { MPBit, MPRes, MPSet, MPReq } MPOpKind;

/* CtrlWord — the fully decoded description of one instruction. The decoder
 * produces it; execute() reads it. Every field is a "knob" the generic execute
 * engine consults instead of special-casing the opcode, so adding/adjusting an
 * instruction is a table change rather than new control flow. */
typedef struct {
    const char *name;    /* mnemonic, for tracing/disassembly */
    InstrClass  exec;    /* which execute handler class runs this op */
    AluOp       alu_op;  /* ALU operation (for ClsUnified/ClsShift/etc.) */
    Width       width;   /* operand width; 0 means "unset" → treat as WWord */
    int         words;   /* instruction length in 16-bit words */
    bool        priv;    /* privileged: traps if executed in Normal mode */
    SrcSel      src2;    /* source of the second operand */
    DstSel      dst;     /* destination of the result */
    SrcSel      count;   /* shift/rotate count or block length source */
    RegSel      src1_nib;/* opcode nibble naming source-1 register */
    RegSel      src2_nib;/* opcode nibble naming source-2 register */
    RegSel      dst_nib; /* opcode nibble naming destination register */
    FcwOpKind   fcw_op;  /* flag-control variant (ClsFcwOp) */
    uint16_t    fcw_mask;/* which FCW bits the flag op touches */
    StackMode   stk_mode;/* PUSH/POP variant (ClsStack) */
    BranchMode  br_mode; /* branch/call/return addressing (ClsBranch) */
    bool        io_write;/* I/O op direction: true = OUT, false = IN */
    bool        io_spec; /* special I/O space (MMU command ports) */
    bool        block_decr;/* block op auto-decrements (…D variants) vs increments */
    bool        block_rept;/* repeating block op (…R variants) */
    MPOpKind    mp_op;   /* multi-processor control variant */
    bool        valid;   /* false → illegal/unimplemented encoding */
} CtrlWord;

/* Decode: fill *out for the instruction whose words are words[0..].
 * Returns number of 16-bit words the instruction occupies. */
void decode_init(void);
int  decode_length(uint16_t op0, bool segmented);
const CtrlWord *decode_lookup(uint16_t op0, const uint16_t *words, int navail,
                              bool segmented);

/* ─────────────────────────── CPU ─────────────────────────── */
typedef struct Machine Machine;

typedef struct {
    uint16_t R[16];
    uint32_t pc;       /* 23-bit segmented internal form: seg<<16 | off */
    uint16_t fcw;
    uint32_t psap;     /* seg<<16 | off */
    uint16_t refresh;
    uint16_t nspseg, nspoff;

    bool     halted;
    uint8_t  irq_req;      /* pending trap/int bits */
    bool     nvi_line, vi_line, nmi_line, segt_line;
    bool     nmi_prev;
    /* external vector supplied on VI INTACK (device-driven) */
    uint16_t vi_vector;

    uint16_t cur_op;       /* opcode[0] of executing instr (for trap tag) */

    /* per-instruction decode context */
    uint16_t opcode[6];
    int      oplen;
    bool     seg_w1, seg_w2;
    uint32_t instr_start;  /* PC at start of current instruction */
    bool     repeat_pending; /* block op should re-execute (PC rewound) */
    bool     fault;        /* a segment-trap fault occurred this instruction */

    Machine *m;
    uint64_t insns;
    uint8_t  serviced;     /* irq bit serviced on the last step (0 if none) */
} CPU;

/* decode.c helpers */
bool decode_seg_addr_w1(uint16_t op, bool segmented);
bool decode_seg_addr_w2(uint16_t op, bool segmented);

/* irq bits */
#define IRQ_RESET   0x01
#define IRQ_SYSCALL 0x02
#define IRQ_VI      0x04
#define IRQ_NVI     0x08
#define IRQ_SEGTRAP 0x10
#define IRQ_NMI     0x20
#define IRQ_TRAP    0x40
#define IRQ_EPU     0x80

void cpu_reset(CPU *c);
void cpu_step(CPU *c);   /* execute exactly one instruction (or take a pending trap) */

/* ─────────────────────────── MMU (Z8010) ─────────────────────────── */
typedef struct { uint16_t base; uint8_t limit; uint8_t attr; } MmuDesc;

/* attribute bits */
#define A_REF  0x80
#define A_CHG  0x40
#define A_DIRW 0x20
#define A_DMAI 0x10
#define A_EXC  0x08
#define A_CPU  0x04
#define A_SYS  0x02
#define A_RD   0x01

typedef struct {
    MmuDesc  desc[64];
    uint8_t  mode;     /* MSEN 0x80, TRNS 0x40, URS 0x20, MST 0x10, NMS 0x08, ID */
    uint8_t  sar;      /* 6-bit */
    uint8_t  dsc;      /* 2-bit */
    uint8_t  viol_type;
    uint8_t  viol_sn;
    uint16_t viol_off;
    uint8_t  bcsr;
    uint8_t  isn;
    uint8_t  ioff;
    bool     viol_captured;
    /* trap output for current translate */
    bool     segt;     /* segment trap requested (level) */
    bool     warn_pending; /* one-shot: a write warning (PWW/SWW) fired this
                            * access; the CPU consumes it after the instruction
                            * completes and takes a segment trap WITHOUT
                            * rewinding PC (the write went through) */

    /* fast-path cache: for descriptors with full limit (0xFF) and no
     * protection bits, translation is just (base<<8)+off (+ REF/CHG). */
    uint32_t fast_base[64];
    bool     fast_ok[64];
} MMU;

#define MODE_MSEN 0x80
#define MODE_TRNS 0x40
#define MODE_URS  0x20
#define MODE_MST  0x10
#define MODE_NMS  0x08

void mmu_reset(MMU *u);
/* command access via special I/O; cmd = high byte of port. */
uint16_t mmu_cmd_read(MMU *u, uint8_t cmd);
void     mmu_cmd_write(MMU *u, uint8_t cmd, uint8_t data);
/* Translate a logical (seg,off) to 24-bit physical; sets *fault on hard viol. */
uint32_t mmu_translate(MMU *u, uint8_t sn, uint16_t off,
                       bool is_write, bool is_normal, bool is_fetch, bool *fault);

/* ─────────────────────────── Machine / bus ─────────────────────────── */

#define ROM_SIZE   0x8000u        /* 32 KiB */
#define PHYS_SIZE  0x1000000u     /* 24-bit physical space */

/* One Z8030 SCC channel. Channel B is the C900 console; Channel A is the
 * second RS-232 port. Bytes are moved through the host terminal (see main.c). */
typedef struct {
    uint8_t  wr[16];       /* write-register file (WR0..WR15) */
    uint8_t  ptr;          /* Z8030 register pointer: if non-zero, overrides the
                            * address-decoded register for the next access, then
                            * auto-resets (set by a WR0 write; see scc_read/write) */
    bool     rx_avail;     /* a received byte is waiting in rx_data */
    uint8_t  rx_data;      /* last byte delivered to this channel's receiver */
} SCCChan;

/* Hard disk: flat file */
typedef struct {
    FILE    *fp;
    uint32_t sectors;    /* total 512-byte sectors */
    uint32_t cyls, heads, spt;
    bool     present;
} Disk;

struct Machine {
    CPU  cpu;
    MMU  mmu;
    uint8_t rom[ROM_SIZE];
    uint8_t *ram;        /* full 24-bit space for simplicity (16 MiB) */

    SCCChan scc_b;       /* console channel (AD5=0) */
    SCCChan scc_a;       /* second channel */

    Disk disk;

    /* PDMAC DMA address latch (23-bit) for HDC */
    uint32_t dma_addr;

    /* system-control latches 0x0200-0x02FF */
    uint16_t latch[0x80];

    /* CIO functional state (reg file per chip; reg = (port>>1)&0x3F) */
    uint8_t  cio1[64];
    uint8_t  cio2[64];
    bool     cio1_reset, cio2_reset;
    /* CIO#1 Port A keyboard input */
    uint8_t  pa_data;
    bool     pa_irf;

    /* interrupt aggregation */
    bool     disk_vi;      /* PDMAC disk-completion VI (vector 0x80), one-shot */
    bool     last_vi_disk; /* the VI currently asserted came from the disk */

    /* CIO #1 Counter/Timer 3 — the 100 Hz Coherent system tick */
    bool     ct3_running;
    uint32_t ct3_accum;    /* instruction accumulator toward terminal count */

    /* HDC command block base (physical); relocatable via WDCCBA */
    uint32_t hdc_cmdblk;

    /* CIO timer emulation for Coherent system tick */
    uint64_t tick_counter;

    bool     trace;
    bool     stop;       /* request emulator halt */
    uint64_t insn_limit; /* 0 = unlimited */

    /* scripted serial input (for non-interactive testing) */
    uint8_t  inq[8192];
    int      inq_len, inq_pos;
    uint64_t last_tx_insn;  /* insn count of the last console output byte */
    bool     shell_up;      /* '#' prompt seen after boot → safe to feed input */
    uint32_t prompt_seq;    /* count of '#' prompts printed since boot */
    uint32_t inq_wait_seq;  /* scripted input waits for this prompt_seq before its next byte */
};

Machine *machine_new(void);
int  machine_load_rom(Machine *m, const char *dir);
int  machine_attach_disk(Machine *m, const char *path);
void machine_run(Machine *m);

/* physical memory access (post-translation) */
uint8_t  phys_read8(Machine *m, uint32_t addr);
void     phys_write8(Machine *m, uint32_t addr, uint8_t v);
uint16_t phys_read16(Machine *m, uint32_t addr);
void     phys_write16(Machine *m, uint32_t addr, uint16_t v);

/* I/O access (normal + special). Returns data (word). */
uint16_t io_read(Machine *m, uint16_t port, bool is_byte, bool special);
void     io_write(Machine *m, uint16_t port, uint16_t data, bool is_byte, bool special);

/* console glue */
void console_init(void);
void console_shutdown(void);
int  console_poll_char(void);   /* returns byte or -1 */
void console_put_char(int ch);

#endif /* EMU_H */
