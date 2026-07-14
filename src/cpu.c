/* cpu.c — Z8001 instruction-level interpreter.
 *
 * No cycle/pin/bus-phase machinery: each cpu_step() fetches and executes
 * exactly one instruction (or takes one
 * pending trap/interrupt), advancing PC. Block/repeat instructions execute one
 * element per step and rewind PC while repeating, so interrupts remain
 * serviceable between iterations (matching the interruptible hardware). */
#include "emu.h"
#include <string.h>
#include <stdlib.h>

/* SEG: true when FCW.SEG is set — the Z8001 is running in segmented mode
 * (23-bit segmented addresses). SYSMODE: true in system (privileged) mode. */
#define SEG(c)   (((c)->fcw & FCW_SEG) != 0)
#define SYSMODE(c) (((c)->fcw & FCW_SN) != 0)

/* ── register file (big-endian words) ──
 * 16 general 16-bit regs R0..R15. Byte regs RH0..RH7 map to r<8 (high byte of
 * R0..R7) and RL0..RL7 to r 8..15 (low byte of R0..R7). Word/long/quad regs
 * alias overlapping ranges; long uses an even-aligned pair, quad a mod-4 group. */
static uint8_t  rb(CPU *c, uint8_t r){ return r<8 ? (uint8_t)(c->R[r]>>8) : (uint8_t)c->R[r-8]; }     /* read byte reg */
static void     wb(CPU *c, uint8_t r, uint8_t v){ if(r<8) c->R[r]=(uint16_t)(v<<8)|(c->R[r]&0x00FF); else c->R[r-8]=(c->R[r-8]&0xFF00)|v; } /* write byte reg */
static uint16_t rw(CPU *c, uint8_t r){ return c->R[r&0x0F]; }                                          /* read word reg */
static void     ww(CPU *c, uint8_t r, uint16_t v){ c->R[r&0x0F]=v; }                                   /* write word reg */
static uint32_t rl(CPU *c, uint8_t r){ r&=0x0E; return ((uint32_t)c->R[r]<<16)|c->R[r+1]; }            /* read long (pair, hi=even reg) */
static void     wl(CPU *c, uint8_t r, uint32_t v){ r&=0x0E; c->R[r]=(uint16_t)(v>>16); c->R[r+1]=(uint16_t)v; } /* write long pair */
static uint64_t rq(CPU *c, uint8_t r){ r&=0x0C; return ((uint64_t)c->R[r]<<48)|((uint64_t)c->R[r+1]<<32)|((uint64_t)c->R[r+2]<<16)|c->R[r+3]; } /* read quad (4 regs) */
static void     wq(CPU *c, uint8_t r, uint64_t v){ r&=0x0C; c->R[r]=(uint16_t)(v>>48); c->R[r+1]=(uint16_t)(v>>32); c->R[r+2]=(uint16_t)(v>>16); c->R[r+3]=(uint16_t)v; } /* write quad */

/* ── address helpers ──
 * Two 32-bit representations coexist:
 *   internal form  = seg(7 bits)<<16 | offset(16)  — how pc/psap/EAs are stored.
 *   register/"seg" form = bit31 flag | seg(7)<<24 | offset(16) — how a segmented
 *   address looks packed in a register pair (high word: bit15 tag + seg in 14:8). */
static uint32_t seg_addr(uint32_t v){ return ((v&0x7F000000u)>>8)|(v&0xFFFF); }         /* reg/seg form → internal form */
static uint32_t make_seg(uint32_t a){ return ((a&0x007F0000u)<<8)|0x80000000u|(a&0xFFFF); } /* internal form → reg/seg form (sets bit31 tag) */
static uint32_t addr_add(uint32_t a, uint32_t n){ return (a&0xFFFF0000u)|((a+n)&0xFFFF); } /* add within segment: only 16-bit offset wraps */
static uint32_t addr_sub(uint32_t a, uint32_t n){ return (a&0xFFFF0000u)|((a-n)&0xFFFF); } /* subtract within segment (offset wraps, seg fixed) */

/* Read an effective address held in register r. Segmented: r&0x0E is a pair
 * holding a packed seg address; non-segmented: a plain 16-bit offset. */
static uint32_t addr_from_reg(CPU *c, uint8_t r){
    if (SEG(c)) return seg_addr(rl(c, r&0x0E));
    return rw(c, r);
}
/* Store effective address a back into register r (inverse of addr_from_reg):
 * segmented writes seg into the even reg (bits 14:8) and offset into the odd. */
static void addr_to_reg(CPU *c, uint8_t r, uint32_t a){
    if (SEG(c)) { uint8_t e=r&0x0E; ww(c,e,(uint16_t)((a>>8)&0x7F00)); ww(c,e+1,(uint16_t)a); }
    else ww(c, r, (uint16_t)a);
}
/* Add signed delta to a pointer register's OFFSET only (used by block/string
 * ops). MAME semantics: in segmented mode only the odd (offset) reg changes;
 * the segment part is never carried into. */
static void add_to_addr_reg(CPU *c, uint8_t r, int16_t d){
    if (SEG(c)) { uint8_t o=r|1; ww(c,o,(uint16_t)((int16_t)rw(c,o)+d)); }
    else ww(c, r, (uint16_t)((int16_t)rw(c,r)+d));
}
/* PC-relative target: add/subtract a byte displacement to the current PC,
 * wrapping within the segment. */
static uint32_t rel_pc(CPU *c, int32_t disp){
    return disp>=0 ? addr_add(c->pc,(uint32_t)disp) : addr_sub(c->pc,(uint32_t)(-disp));
}

/* Opcode nibble extractors. For opcode word op:
 *   nib1 = bits 11:8 (often dst reg / cc), nib2 = bits 7:4 (reg/index),
 *   nib3 = bits 3:0 (reg / cc / sub-op). Bits 15:12 = nib0 (major opcode). */
static uint8_t nib1(uint16_t op){ return (op>>8)&0x0F; }
static uint8_t nib2(uint16_t op){ return (op>>4)&0x0F; }
static uint8_t nib3(uint16_t op){ return op&0x0F; }

/* getSegAddrFromOpcode: decode a Direct-Address operand starting at opcode[idx].
 * Non-segmented: a single 16-bit offset word. Segmented long form (word bit15
 * set): seg in bits 14:8, followed by a full 16-bit offset word at idx+1.
 * Segmented short form (bit15 clear): seg in 14:8, 8-bit offset in bits 7:0. */
static uint32_t seg_addr_op(CPU *c, int idx){
    if (SEG(c)) {
        uint16_t w1=c->opcode[idx];
        uint32_t seg=(uint32_t)(w1&0x7F00)<<8;
        if (w1&0x8000) return seg | c->opcode[idx+1];  /* long: separate offset word */
        return seg | (w1&0xFF);                         /* short: 8-bit inline offset */
    }
    return c->opcode[idx];
}

/* ── MMU-translated physical memory access ──
 * Every logical access is split into (segment, offset), run through the Z8010
 * MMU, and dispatched to physical memory. A hard MMU violation sets c->fault,
 * which cpu_step turns into a segment trap. is_normal = !SYSMODE (user vs sys
 * access rights); the is_fetch flag distinguishes instruction fetch from data. */

/* Debug: CSIM_TRACE_PC="SS:OFFS" (hex) logs every data access made by the
 * instruction at that (normal-mode) PC: logical addr, physical addr, value. */
static uint32_t tpc_addr; static int tpc_on = -1;
static bool tpc_match(CPU *c){
    if (tpc_on < 0) {
        const char *s = getenv("CSIM_TRACE_PC");
        unsigned sg, of;
        tpc_on = (s && sscanf(s, "%x:%x", &sg, &of) == 2) ? 1 : 0;
        if (tpc_on) tpc_addr = (sg << 16) | of;
    }
    return tpc_on == 1 && c->instr_start == tpc_addr && !SYSMODE(c);
}

/* Instruction fetch (is_fetch=true so the MMU can apply exec permissions). */
static uint16_t ifetch16(CPU *c, uint32_t a){
    uint8_t sn=(a>>16)&0x7F; uint16_t off=a&0xFFFF; bool f=false;
    uint32_t p = mmu_translate(&c->m->mmu, sn, off, false, !SYSMODE(c), true, &f);
    if (f) c->fault = true;
    return phys_read16(c->m, p);
}
/* Data word read. */
static uint16_t mem_r16(CPU *c, uint32_t a){
    uint8_t sn=(a>>16)&0x7F; uint16_t off=a&0xFFFF; bool f=false;
    uint32_t p = mmu_translate(&c->m->mmu, sn, off, false, !SYSMODE(c), false, &f);
    if (f) c->fault = true;
    if (tpc_match(c))
        fprintf(stderr, "[tpc] r16 log=%02X:%04X phys=%06X -> %04X fault=%d insns=%llu\n",
                sn, off, p, phys_read16(c->m,p), f, (unsigned long long)c->insns);
    return phys_read16(c->m, p);
}
/* Data word write (on fault, skip the store — the trap will re-run the instr). */
static void mem_w16(CPU *c, uint32_t a, uint16_t v){
    uint8_t sn=(a>>16)&0x7F; uint16_t off=a&0xFFFF; bool f=false;
    uint32_t p = mmu_translate(&c->m->mmu, sn, off, true, !SYSMODE(c), false, &f);
    if (f) { c->fault = true; return; }
    phys_write16(c->m, p, v);
}
/* Data byte read. */
static uint8_t mem_r8(CPU *c, uint32_t a){
    uint8_t sn=(a>>16)&0x7F; uint16_t off=a&0xFFFF; bool f=false;
    uint32_t p = mmu_translate(&c->m->mmu, sn, off, false, !SYSMODE(c), false, &f);
    if (f) c->fault = true;
    return phys_read8(c->m, p);
}
/* Data byte write. */
static void mem_w8(CPU *c, uint32_t a, uint8_t v){
    uint8_t sn=(a>>16)&0x7F; uint16_t off=a&0xFFFF; bool f=false;
    uint32_t p = mmu_translate(&c->m->mmu, sn, off, true, !SYSMODE(c), false, &f);
    if (f) { c->fault = true; return; }
    phys_write8(c->m, p, v);
}

/* ── flags ──
 * apply_flags merges only the flag bits the ALU op declares "affected"
 * (r.mask) into FCW, preserving all others — this honours the per-instruction
 * "unaffected" columns of the Z8000 manual (e.g. AND leaves C/D/H alone). */
static void apply_flags(CPU *c, AluResult r){
    c->fcw = (c->fcw & ~r.mask) | (r.flags & r.mask);
}
/* Evaluate a 4-bit condition code against the current C/Z/S/PV flags.
 * Codes match the Z8000 encoding; note LT/LE/GE/GT use the S≠V / S=V
 * signed-overflow relation, while ULE/ULT/UGT/UGE are the unsigned (carry)
 * variants. Comment names: 1 LT, 2 LE, 3 ULE, 4 OV, 5 MI, 6 EQ, 7 ULT/C,
 * 8 T(rue), 9 GE, A GT, B UGT, C NOV, D PL, E NE, F UGE/NC. */
static bool cond_true(CPU *c, uint8_t cc){
    bool C=c->fcw&F_C, Z=c->fcw&F_Z, S=c->fcw&F_S, V=c->fcw&F_PV;
    switch (cc & 0x0F) {
        case 0x0: return false;         /* F (never) */
        case 0x1: return S!=V;          /* LT */
        case 0x2: return (S!=V)||Z;     /* LE */
        case 0x3: return C||Z;          /* ULE */
        case 0x4: return V;             /* OV / PE */
        case 0x5: return S;             /* MI */
        case 0x6: return Z;             /* EQ / Z */
        case 0x7: return C;             /* ULT / C */
        case 0x8: return true;          /* T (always) */
        case 0x9: return S==V;          /* GE */
        case 0xA: return (S==V)&&!Z;    /* GT */
        case 0xB: return !C&&!Z;        /* UGT */
        case 0xC: return !V;            /* NOV / PO */
        case 0xD: return !S;            /* PL */
        case 0xE: return !Z;            /* NE / NZ */
        case 0xF: return !C;            /* UGE / NC */
    }
    return false;
}

/* changeFCW: install a new FCW, handling the stack-pointer bank swap on any
 * system↔normal or SEG transition (MAME CHANGE_FCW). The Z8001 keeps separate
 * system and normal stack pointers; only one bank lives in R14/R15 at a time,
 * the other is stashed in c->nspseg/nspoff. When the mode changes we exchange
 * the live SP with the saved one so each mode sees its own stack. R15 (offset)
 * swaps on any S/N change; R14 (segment) swaps on entering/leaving system mode
 * or when the SEG bit toggles. */
static void change_fcw(CPU *c, uint16_t nf){
    uint16_t of = c->fcw;
    if ((nf ^ of) & FCW_SN) {
        uint16_t t = rw(c,15); ww(c,15,c->nspoff); c->nspoff=t;
    }
    if ((nf & FCW_SN) && (!(of&FCW_SN) || ((nf^of)&FCW_SEG))) {
        uint16_t t=rw(c,14); ww(c,14,c->nspseg); c->nspseg=t;
    } else if (!(nf&FCW_SN) && (of&FCW_SN) && (of&FCW_SEG)) {
        uint16_t t=rw(c,14); ww(c,14,c->nspseg); c->nspseg=t;
    }
    c->fcw = nf;
}

/* Which register is the stack pointer: R14/R15 pair in segmented mode, R15 in
 * non-segmented mode. (change_fcw keeps the correct S/N bank loaded there.) */
static uint8_t sp_reg(CPU *c){ return SEG(c) ? 14 : 15; }

/* Predecrement push of one word: SP -= 2 then store. Segmented decrements the
 * offset half of the SP pair only. */
static void push_word(CPU *c, uint16_t v){
    uint8_t sp=sp_reg(c);
    if (SEG(c)) { uint32_t a=addr_sub(addr_from_reg(c,sp),2); addr_to_reg(c,sp,a); mem_w16(c,a,v); }
    else { uint16_t a=rw(c,sp)-2; ww(c,sp,a); mem_w16(c,a,v); }
}
/* Postincrement pop of one word: read at SP then SP += 2. */
static uint16_t pop_word(CPU *c){
    uint8_t sp=sp_reg(c);
    if (SEG(c)) { uint32_t a=addr_from_reg(c,sp); uint16_t v=mem_r16(c,a); addr_to_reg(c,sp,addr_add(a,2)); return v; }
    else { uint16_t a=rw(c,sp); uint16_t v=mem_r16(c,a); ww(c,sp,a+2); return v; }
}

/* Privileged-instruction guard: privileged ops are only legal in system mode.
 * In normal mode, request a privileged-instruction trap (IRQ_TRAP, taken at the
 * next instruction boundary) and return false so the caller aborts execution. */
static bool check_priv(CPU *c){
    if (!SYSMODE(c)) { c->irq_req |= IRQ_TRAP; return false; }
    return true;
}

/* ── operand read helpers for ClsUnified ──
 * ClsUnified is fully data-driven: the CtrlWord's Src2/Dst/Count/*_nib fields
 * act as datapath mux-selects. These helpers resolve those selects into actual
 * register numbers, immediate values, and effective addresses. */

/* Map a RegSel to the register number encoded in the chosen nibble of opcode[0]. */
static uint8_t unib(CPU *c, RegSel s){
    switch (s) { case RegNib2: return nib2(c->opcode[0]); case RegNib1: return nib1(c->opcode[0]); default: return nib3(c->opcode[0]); }
}
/* Read a register operand at the width of the instruction (byte/word/long). */
static uint64_t ureg_read(CPU *c, RegSel s, Width w){
    uint8_t r=unib(c,s);
    switch (w){ case WByte: return rb(c,r); case WLong: return rl(c,r&0x0E); default: return rw(c,r); }
}
/* Write a register operand at instruction width. */
static void ureg_write(CPU *c, RegSel s, uint64_t v, Width w){
    uint8_t r=unib(c,s);
    switch (w){ case WByte: wb(c,r,(uint8_t)v); break; case WLong: wl(c,r&0x0E,(uint32_t)v); break; default: ww(c,r,(uint16_t)v); }
}
/* base ± signed displacement, wrapping within segment. */
static uint32_t displaced(CPU *c, uint32_t base, int16_t disp){
    return disp>=0 ? addr_add(base,(uint32_t)disp) : addr_sub(base,(uint32_t)(-(int32_t)disp));
}
/* Fetch the second source VALUE (register/immediate/constant) per Src2 select.
 * Immediates come from the extra instruction words (byte imm = high byte of
 * opcode[1]; long imm = opcode[1]<<16|opcode[2]). The constant/nibble forms
 * (SrcOne/Two/Zero/Nib3*) serve INC/DEC counts, shifts, and bit numbers. */
static uint64_t uread_src2(CPU *c, const CtrlWord *w){
    switch (w->src2){
        case SrcReg: return ureg_read(c, w->src2_nib, w->width);
        case SrcImm:
            if (w->width==WByte) return c->opcode[1]>>8;
            if (w->width==WLong) return ((uint64_t)c->opcode[1]<<16)|c->opcode[2];
            return c->opcode[1];
        case SrcOne: return 1;                                      /* rotate/shift by 1 */
        case SrcTwo: return 2;                                      /* rotate by 2 */
        case SrcNib3Cnt: { uint64_t n=nib3(c->opcode[0]); return n?n:16; } /* INC/DEC count, 0→16 */
        case SrcZero: return 0;                                     /* CLR: write zero */
        case SrcImm2: {
            /* Immediate in the LAST word of a CP/LD-with-immediate. Its index
             * shifts to 3 when a segmented long-form address consumed an extra
             * word (opcode[1] bit15 = long DA). */
            int idx=2;
            if (c->seg_w1 && SEG(c) && (c->opcode[1]&0x8000)) idx=3;
            return (w->width==WByte) ? (c->opcode[idx]>>8) : c->opcode[idx];
        }
        case SrcNib3Bit: return nib3(c->opcode[0]);                 /* bit number 0-15 (no 0→16) */
        case SrcNib3CntPlus1: return (uint64_t)nib3(c->opcode[0])+1;/* register INC/DEC count 1-16 */
        case SrcOpLoByte: return (uint8_t)c->opcode[0];             /* LDB Rb,#imm8 inline byte */
        default: return 0;
    }
}
/* Resolve the source EFFECTIVE ADDRESS for a memory-source operand.
 *   IR    = @Rm (register indirect); DA = direct address in the operand words;
 *   X     = DA + index register; PCRel = PC + disp; Based = @Rm + disp;
 *   BX    = @Rm + index register (index reg # in opcode[1] bits 11:8). */
static uint32_t usrc_addr(CPU *c, const CtrlWord *w){
    switch (w->src2){
        case SrcIR: return addr_from_reg(c, unib(c,w->src2_nib));
        case SrcDA: return seg_addr_op(c,1);
        case SrcX: return addr_add(seg_addr_op(c,1), rw(c, unib(c,w->src2_nib)));
        case SrcPCRel: return displaced(c, c->pc, (int16_t)c->opcode[1]);
        case SrcBased: return displaced(c, addr_from_reg(c,unib(c,w->src2_nib)), (int16_t)c->opcode[1]);
        case SrcBX: { uint32_t b=addr_from_reg(c,unib(c,w->src2_nib)); uint8_t idx=(c->opcode[1]>>8)&0x0F; return addr_add(b, rw(c,idx)); }
        default: return 0;
    }
}
/* Resolve the destination EFFECTIVE ADDRESS for a memory-write operand
 * (same addressing modes as usrc_addr but driven by the Dst field). */
static uint32_t udst_addr(CPU *c, const CtrlWord *w){
    switch (w->dst){
        case DstIR: return addr_from_reg(c, unib(c,w->dst_nib));
        case DstDA: return seg_addr_op(c,1);
        case DstX: return addr_add(seg_addr_op(c,1), rw(c, unib(c,w->dst_nib)));
        case DstPCRel: return displaced(c, c->pc, (int16_t)c->opcode[1]);
        case DstBased: return displaced(c, addr_from_reg(c,unib(c,w->dst_nib)), (int16_t)c->opcode[1]);
        case DstBX: { uint32_t b=addr_from_reg(c,unib(c,w->dst_nib)); uint8_t idx=(c->opcode[1]>>8)&0x0F; return addr_add(b, rw(c,idx)); }
        default: return 0;
    }
}
/* True if the source/destination select denotes a memory access. */
static bool src_is_mem(SrcSel s){ return s==SrcIR||s==SrcDA||s==SrcX||s==SrcPCRel||s==SrcBased||s==SrcBX; }
static bool dst_is_mem(DstSel d){ return d==DstIR||d==DstDA||d==DstX||d==DstPCRel||d==DstBased||d==DstBX; }

/* read a memory operand of given width at addr (long = two words, hi first) */
static uint64_t mem_read_w(CPU *c, uint32_t a, Width w){
    if (w==WByte) return mem_r8(c,a);
    if (w==WLong) { uint32_t hi=mem_r16(c,a); uint32_t lo=mem_r16(c,addr_add(a,2)); return ((uint64_t)hi<<16)|lo; }
    return mem_r16(c,a);
}
static void mem_write_w(CPU *c, uint32_t a, uint64_t v, Width w){
    if (w==WByte) { mem_w8(c,a,(uint8_t)v); return; }
    if (w==WLong) { mem_w16(c,a,(uint16_t)(v>>16)); mem_w16(c,addr_add(a,2),(uint16_t)v); return; }
    mem_w16(c,a,(uint16_t)v);
}

/* ── ClsUnified ──
 * The general ALU/data-movement engine. Four shapes, distinguished by whether
 * source and/or destination are memory:
 *   1. mem src + mem dst (same EA)  → read-modify-write @addr: value = addr op count
 *   2. mem src + DstFlagsOnly       → unary/compare on memory (TEST/CP @addr): flags only
 *   3. reg/imm src + mem dst        → pure store to memory
 *   4. reg/imm src + reg/none dst   → compute a=src1 reg, b=src2; result to reg (DstReg)
 * For RMW/flags-only the memory value is the ALU's first operand and the second
 * operand comes from the Count select (SrcOne for shifts, SrcNib3Cnt for INC/DEC). */
static void exec_unified(CPU *c, const CtrlWord *w){
    uint64_t b;
    if (src_is_mem(w->src2)) {
        uint32_t sa = usrc_addr(c, w);
        b = mem_read_w(c, sa, w->width);
        if (dst_is_mem(w->dst)) {
            /* case 1 — RMW: memory value is operand a, count is operand b */
            CtrlWord cc = *w; cc.src2 = w->count;   /* reuse uread_src2 to fetch the count */
            uint64_t cnt = uread_src2(c, &cc);
            AluResult r = alu_exec(w->alu_op, b, cnt, c->fcw&F_C, w->width);
            apply_flags(c, r);
            mem_write_w(c, sa, r.value, w->width);   /* write back to same EA */
            return;
        } else if (w->dst == DstFlagsOnly) {
            /* case 2 — unary/compare on memory: flags only, no write-back */
            CtrlWord cc = *w; cc.src2 = w->count;
            uint64_t cnt = uread_src2(c, &cc);
            AluResult r = alu_exec(w->alu_op, b, cnt, c->fcw&F_C, w->width);
            apply_flags(c, r);
            return;
        }
        /* mem src, reg/none dst → b holds the loaded value; fall through to compute */
    } else if (dst_is_mem(w->dst)) {
        /* case 3 — pure store: source value → destination memory */
        b = uread_src2(c, w);
        mem_write_w(c, udst_addr(c, w), b, w->width);
        return;
    } else {
        b = uread_src2(c, w);
    }
    /* case 4 — compute: a op b, where a is the src1 register (default: no
     * DstReg write for CP/TEST which use DstNone). */
    uint64_t a = ureg_read(c, w->src1_nib, w->width);
    AluResult r = alu_exec(w->alu_op, a, b, c->fcw&F_C, w->width);
    apply_flags(c, r);
    if (w->dst == DstReg) ureg_write(c, w->dst_nib, r.value, w->width);
}

/* ── ClsBranch ── JP/JR/CALL/CALR/RET/DJNZ.
 * On entry c->pc already points PAST the instruction (set by cpu_step), so it
 * is the fall-through / return address. Taken branches overwrite c->pc.
 * CALL/CALR push the return address first: in segmented mode the return PC is
 * two words (offset then segment high word from make_seg), matching the Z8001
 * "push return PC, then load destination" order. Displacements are in words, so
 * byte displacements are ×2. */
static void exec_branch(CPU *c, const CtrlWord *w){
    uint16_t op=c->opcode[0];
    switch (w->br_mode){
        case BranchJP_DA: if (cond_true(c,nib3(op))) c->pc=seg_addr_op(c,1); break;                       /* JP cc, addr */
        case BranchJP_X: if (cond_true(c,nib3(op))) c->pc=addr_add(seg_addr_op(c,1), rw(c,nib2(op))); break; /* JP cc, addr(Rx) */
        case BranchJP_IR: if (cond_true(c,nib3(op))) c->pc=addr_from_reg(c,nib2(op)); break;              /* JP cc, @Rs */
        case BranchJR: { uint8_t cc=nib1(op); if (cond_true(c,cc)) c->pc=rel_pc(c,(int32_t)(int8_t)(op&0xFF)*2); break; } /* JR cc, disp8 (signed, ×2) */
        case BranchCALL_DA: { uint32_t t=seg_addr_op(c,1); push_word(c,(uint16_t)c->pc); if (SEG(c)) push_word(c,(uint16_t)(make_seg(c->pc)>>16)); c->pc=t; break; } /* CALL addr */
        case BranchCALL_X: { uint32_t t=addr_add(seg_addr_op(c,1),rw(c,nib2(op))); push_word(c,(uint16_t)c->pc); if (SEG(c)) push_word(c,(uint16_t)(make_seg(c->pc)>>16)); c->pc=t; break; } /* CALL addr(Rx) */
        case BranchCALL_IR: { uint32_t t=addr_from_reg(c,nib2(op)); push_word(c,(uint16_t)c->pc); if (SEG(c)) push_word(c,(uint16_t)(make_seg(c->pc)>>16)); c->pc=t; break; } /* CALL @Rs */
        case BranchCALR: { int8_t d=(int8_t)(op&0xFF); uint32_t t=rel_pc(c,(int32_t)d*2); push_word(c,(uint16_t)c->pc); if (SEG(c)) push_word(c,(uint16_t)(make_seg(c->pc)>>16)); c->pc=t; break; } /* CALR disp8 */
        case BranchCALR12: { int32_t raw=op&0x0FFF; if (raw&0x0800) raw|=~0x0FFF; uint32_t t=rel_pc(c,-raw*2); push_word(c,(uint16_t)c->pc); if (SEG(c)) push_word(c,(uint16_t)(make_seg(c->pc)>>16)); c->pc=t; break; } /* CALR disp12 (sign-extended, branches backward: −raw×2) */
        case BranchDJNZ: {
            /* DJNZ/DBJNZ: decrement reg (word if bit7 set, else byte), and if
             * still non-zero branch BACKWARD by the 7-bit unsigned disp (×2). */
            uint8_t reg=nib1(op); uint32_t disp=op&0x7F; bool word=op&0x0080; bool nz;
            if (word){ uint16_t v=rw(c,reg)-1; ww(c,reg,v); nz=v!=0; }
            else { uint8_t v=rb(c,reg)-1; wb(c,reg,v); nz=v!=0; }
            if (nz) c->pc=addr_sub(c->pc, disp<<1);
            break;
        }
        case BranchRET: {
            /* RET cc: if condition holds, pop the return PC. Segmented pops two
             * words (segment high word first, then offset) and rebuilds the EA. */
            uint8_t cc=nib3(op);
            if (!cond_true(c,cc)) break;
            if (SEG(c)) { uint16_t hi=pop_word(c); uint16_t lo=pop_word(c); c->pc=seg_addr(((uint32_t)hi<<16)|lo); }
            else c->pc=pop_word(c);
            break;
        }
    }
}

/* ── ClsStack (register + indirect + DA/X + imm) ──
 * PUSH/POP/PUSHL/POPL where ANY register may be the stack pointer (nib2 of
 * opcode[0]), not just R15 — these are the general "stack" instructions, not
 * the implicit call/interrupt stack. nib3 selects the data register. PUSH
 * predecrements the chosen SP; POP postincrements it. The *L forms move a
 * long (2 words / 4 bytes). The *Src/*Dst/*Imm forms move between the stack
 * and a memory operand or immediate rather than a register. In segmented mode
 * only the SP pair's offset half is adjusted. */
static void exec_stack(CPU *c, const CtrlWord *w){
    uint16_t op=c->opcode[0]; uint8_t sp=nib2(op), data=nib3(op);
    switch (w->stk_mode){
        case StackPush: {  /* PUSH @Rsp, Rdata: SP-=2, [SP]=Rdata */
            uint32_t a; if (SEG(c)){a=addr_sub(addr_from_reg(c,sp),2);addr_to_reg(c,sp,a);} else {a=addr_from_reg(c,sp)-2;ww(c,sp,(uint16_t)a);}
            mem_w16(c,a,rw(c,data)); break; }
        case StackPop: {  /* POP Rdata, @Rsp: Rdata=[SP], SP+=2 */
            uint32_t a=addr_from_reg(c,sp);
            if (SEG(c)) addr_to_reg(c,sp,addr_add(a,2)); else ww(c,sp,(uint16_t)a+2);
            ww(c,data,mem_r16(c,a)); break; }
        case StackPushL: {  /* PUSHL @Rsp, RRdata: SP-=4, store long (hi word, lo word) */
            uint8_t src=data&0x0E; uint32_t a;
            if (SEG(c)){a=addr_sub(addr_from_reg(c,sp),4);addr_to_reg(c,sp,a);} else {a=addr_from_reg(c,sp)-4;ww(c,sp,(uint16_t)a);}
            mem_w16(c,a,rw(c,src)); mem_w16(c,addr_add(a,2),rw(c,src+1)); break; }
        case StackPopL: {  /* POPL RRdata, @Rsp: load long, SP+=4 */
            uint32_t a=addr_from_reg(c,sp);
            if (SEG(c)) addr_to_reg(c,sp,addr_add(a,4)); else ww(c,sp,(uint16_t)a+4);
            uint16_t hi=mem_r16(c,a); uint16_t lo=mem_r16(c,addr_add(a,2));
            uint8_t dst=nib3(op)&0x0E; ww(c,dst,hi); ww(c,dst+1,lo); break; }
        case StackPushSrc: {  /* PUSH @Rsp, src-mem: read memory operand, then push it */
            uint16_t v=mem_r16(c, usrc_addr(c,w)); uint32_t a;
            if (SEG(c)){a=addr_sub(addr_from_reg(c,sp),2);addr_to_reg(c,sp,a);} else {a=addr_from_reg(c,sp)-2;ww(c,sp,(uint16_t)a);}
            mem_w16(c,a,v); break; }
        case StackPopDst: {  /* POP dst-mem, @Rsp: pop word, store to memory operand */
            uint32_t a=addr_from_reg(c,sp);
            if (SEG(c)) addr_to_reg(c,sp,addr_add(a,2)); else ww(c,sp,(uint16_t)a+2);
            uint16_t v=mem_r16(c,a); mem_w16(c, udst_addr(c,w), v); break; }
        case StackPushLSrc: {  /* PUSHL @Rsp, src-mem: read long from memory operand, push it */
            uint32_t base = (w->src2==SrcIR)? addr_from_reg(c,nib3(op)&0x0E) : usrc_addr(c,w);
            uint16_t hi=mem_r16(c,base); uint16_t lo=mem_r16(c,addr_add(base,2)); uint32_t a;
            if (SEG(c)){a=addr_sub(addr_from_reg(c,sp),4);addr_to_reg(c,sp,a);} else {a=addr_from_reg(c,sp)-4;ww(c,sp,(uint16_t)a);}
            mem_w16(c,a,hi); mem_w16(c,addr_add(a,2),lo); break; }
        case StackPopLDst: {  /* POPL dst-mem, @Rsp: pop long, store to memory operand */
            uint32_t a=addr_from_reg(c,sp);
            if (SEG(c)) addr_to_reg(c,sp,addr_add(a,4)); else ww(c,sp,(uint16_t)a+4);
            uint16_t hi=mem_r16(c,a); uint16_t lo=mem_r16(c,addr_add(a,2));
            uint32_t base = (w->dst==DstIR)? addr_from_reg(c,nib3(op)&0x0E) : udst_addr(c,w);
            mem_w16(c,base,hi); mem_w16(c,addr_add(base,2),lo); break; }
        case StackPushImm: {  /* PUSH @Rsp, #imm: SP-=2, [SP]=opcode[1] */
            uint32_t a; if (SEG(c)){a=addr_sub(addr_from_reg(c,sp),2);addr_to_reg(c,sp,a);} else {a=addr_from_reg(c,sp)-2;ww(c,sp,(uint16_t)a);}
            mem_w16(c,a,c->opcode[1]); break; }
    }
}

/* ── mul/div (direct computation — no hardware state machine here) ──
 * MULT (word): 16×16→32, multiplicand in the LOW word of the dst pair, product
 *   written to the whole pair. MULTL (long): 32×32→64 into a quad register.
 * Signed arithmetic; flags: Z if product 0, S if product negative, C set when
 * the product overflowed the single-operand width (i.e. the high half is a
 * non-trivial sign extension). PV cleared. */
static void exec_mul(CPU *c, const CtrlWord *w){
    uint8_t dst=nib3(c->opcode[0]);
    if (w->width==WLong) dst&=0x0E;
    uint64_t mcand, mplier;
    /* multiplicand = low half of the destination register group */
    if (w->width==WWord) mcand = rw(c,dst|1); else mcand = rl(c,(dst|2)&0x0E);
    /* fetch multiplier from register / immediate / memory per Src2 select */
    switch (w->src2){
        case SrcReg: mplier = (w->width==WWord)? rw(c,nib2(c->opcode[0])) : rl(c,nib2(c->opcode[0])&0x0E); break;
        case SrcImm: mplier = (w->width==WWord)? c->opcode[1] : (((uint32_t)c->opcode[1]<<16)|c->opcode[2]); break;
        default: { uint32_t a=usrc_addr(c,w); mplier = mem_read_w(c,a,w->width); }
    }
    uint16_t flags=0; const uint16_t mask=F_C|F_Z|F_S|F_PV;
    if (w->width==WWord) {
        int32_t prod=(int32_t)((int16_t)mcand)*(int32_t)((int16_t)mplier);
        wl(c,dst&0x0E,(uint32_t)prod);
        if (prod==0) flags|=F_Z;
        if ((uint32_t)prod&0x80000000u) flags|=F_S;
        if (prod<-0x8000 || prod>0x7FFF) flags|=F_C;
    } else {
        int64_t prod=(int64_t)((int32_t)mcand)*(int64_t)((int32_t)mplier);
        wq(c,dst&0x0C,(uint64_t)prod);
        if (prod==0) flags|=F_Z;
        if ((uint64_t)prod&0x8000000000000000ull) flags|=F_S;
        if (prod<-0x80000000ll || prod>0x7FFFFFFFll) flags|=F_C;
    }
    c->fcw = (c->fcw&~mask)|(flags&mask);
}
/* DIV (word): 32-bit dividend (dst pair) ÷ 16-bit divisor → 16-bit quotient in
 * the odd reg, remainder in the even reg. DIVL (long): 64÷32 → 32q/32r in a
 * quad. Signed. Special cases follow the Z8000 manual:
 *   divisor==0            → V and Z set (division by zero), no register change.
 *   quotient just out of range → set C+V (+S/Z as computed), truncated result
 *                            still stored (manual "case 2/overflow").
 *   quotient wildly out of range → V only, no register change ("case 3"). */
static void exec_div(CPU *c, const CtrlWord *w){
    uint8_t dst=nib3(c->opcode[0]);
    if (w->width==WLong) dst&=0x0C;
    const uint16_t mask=F_C|F_Z|F_S|F_PV; uint16_t flags=0;
    if (w->width==WWord) {
        int32_t dividend=(int32_t)rl(c,dst&0x0E);
        int32_t divisor;
        switch (w->src2){
            case SrcReg: divisor=(int16_t)rw(c,nib2(c->opcode[0])); break;
            case SrcImm: divisor=(int16_t)c->opcode[1]; break;
            default: { uint32_t a=usrc_addr(c,w); divisor=(int16_t)(uint16_t)mem_read_w(c,a,WWord); }
        }
        if (divisor==0) { flags=F_Z|F_PV; }
        else {
            int32_t q=dividend/divisor, r=dividend%divisor;
            if (q<-0x8000 || q>0x7FFF) {
                if (q < -0x10000 || q > 0xFFFF) flags=F_PV;         /* case 3 */
                else { ww(c,(dst&0x0E)+1,(uint16_t)q); ww(c,dst&0x0E,(uint16_t)r); flags=F_C|F_PV; if(q<0)flags|=F_S; if((uint16_t)q==0)flags|=F_Z; }
            } else {
                ww(c,(dst&0x0E)+1,(uint16_t)q); ww(c,dst&0x0E,(uint16_t)r);
                if (q==0) flags|=F_Z; if (q<0) flags|=F_S;
            }
        }
    } else {
        int64_t dividend=(int64_t)rq(c,dst&0x0C);
        int64_t divisor;
        switch (w->src2){
            case SrcReg: divisor=(int32_t)rl(c,nib2(c->opcode[0])&0x0E); break;
            case SrcImm: divisor=(int32_t)(((uint32_t)c->opcode[1]<<16)|c->opcode[2]); break;
            default: { uint32_t a=usrc_addr(c,w); divisor=(int32_t)(uint32_t)mem_read_w(c,a,WLong); }
        }
        if (divisor==0) { flags=F_Z|F_PV; }
        else {
            int64_t q=dividend/divisor, r=dividend%divisor;
            if (q<-0x80000000ll || q>0x7FFFFFFFll) {
                if (q<-0x100000000ll || q>0xFFFFFFFFll) flags=F_PV;
                else { wl(c,(dst&0x0C)+2,(uint32_t)q); wl(c,dst&0x0C,(uint32_t)r); flags=F_C|F_PV; if(q<0)flags|=F_S; if((uint32_t)q==0)flags|=F_Z; }
            } else {
                wl(c,(dst&0x0C)+2,(uint32_t)q); wl(c,dst&0x0C,(uint32_t)r);
                if (q==0) flags|=F_Z; if (q<0) flags|=F_S;
            }
        }
    }
    c->fcw=(c->fcw&~mask)|(flags&mask);
}

/* ── shift ── SLA/SRA/SLL/SRL and dynamic (SDA/SDL) forms.
 * dst = nib2(opcode[0]) (long forces even alignment). The shift count comes
 * from opcode[1]: static (SrcImm) uses the whole signed word; dynamic (SrcReg)
 * reads it from the register named in opcode[1] bits 11:8. A NEGATIVE count
 * reverses direction (arithmetic SLA→SRA, logical SLL→SRL) using |count|.
 * Flags come from the ALU (no carry-in). */
static void exec_shift(CPU *c, const CtrlWord *w){
    uint8_t dst=nib2(c->opcode[0]);
    if (w->width==WLong) dst&=0x0E;
    int16_t count = (w->count==SrcReg) ? (int16_t)rw(c,(c->opcode[1]>>8)&0x0F) : (int16_t)c->opcode[1];
    AluOp op=w->alu_op; int n=count;
    if (count<0) { op = (w->alu_op==OpSla)?OpSra:OpSrl; n=-count; }  /* negative count → shift the other way */
    uint64_t val; switch (w->width){ case WByte: val=rb(c,dst); break; case WLong: val=rl(c,dst); break; default: val=rw(c,dst);}
    AluResult r=alu_exec(op, val, (uint64_t)n, false, w->width);
    switch (w->width){ case WByte: wb(c,dst,(uint8_t)r.value); break; case WLong: wl(c,dst,(uint32_t)r.value); break; default: ww(c,dst,(uint16_t)r.value);}
    apply_flags(c, r);
}

/* ── LDM ── Load/store Multiple: transfer a run of n consecutive registers
 * to/from a contiguous memory block. Base EA is @Rs (SrcIR) or a DA/X operand.
 * The "extra word" (opcode[1] for IR, opcode[2] for DA/X) encodes first reg in
 * bits 11:8 and n-1 in bits 3:0. io_write=false loads (mem→regs); true stores
 * (regs→mem). Consecutive words are placed at base, base+2, … */
static void exec_ldm(CPU *c, const CtrlWord *w){
    uint16_t op=c->opcode[0]; uint32_t base; int ew;
    if (w->src2==SrcIR){ base=addr_from_reg(c,nib2(op)); ew=1; }
    else { ew=1; base=seg_addr_op(c,2); uint8_t idx=nib2(op); if(idx) base=addr_add(base, rw(c,idx)); }
    uint16_t ewd=c->opcode[ew];
    uint8_t first=(ewd>>8)&0x0F; int n=(ewd&0x0F)+1;
    for (int i=0;i<n;i++){
        uint32_t a=addr_add(base,(uint32_t)i<<1);
        if (!w->io_write) ww(c,first+i,mem_r16(c,a));   /* load reg from memory */
        else mem_w16(c,a,rw(c,first+i));                /* store reg to memory */
    }
}

/* ── IO ── IN/OUT/INB/OUTB and special (S*) forms. Privileged.
 * Port address: SrcIR = @Rreg (nib2, data reg = nib3); SrcDA = opcode[1]
 * + optional index (nib2), data reg = nib3; default = opcode[1], data reg =
 * nib2. io_spec selects the "special I/O" address space (MMU CS). For byte
 * ops the data lane depends on spec (see io_read/io_write). */
static void exec_io(CPU *c, const CtrlWord *w){
    if (!check_priv(c)) return;
    uint16_t op=c->opcode[0]; uint32_t port; uint8_t dr;
    switch (w->src2){
        case SrcIR: port=rw(c,nib2(op)); dr=nib3(op); break;
        case SrcDA: port=c->opcode[1]; { uint8_t idx=nib2(op); if(idx) port+=rw(c,idx);} dr=nib3(op); break;
        default: port=c->opcode[1]; dr=nib2(op); break;
    }
    bool spec=w->io_spec;
    if (w->io_write){
        if (w->width==WByte){ uint16_t d = spec? rb(c,dr) : (uint16_t)rb(c,dr)<<8; io_write(c->m,(uint16_t)port,d,true,spec); }
        else io_write(c->m,(uint16_t)port,rw(c,dr),false,spec);
    } else {
        uint16_t v=io_read(c->m,(uint16_t)port,w->width==WByte,spec);
        if (w->width==WByte){ if(spec) wb(c,dr,(uint8_t)v); else wb(c,dr,(uint8_t)(v>>8)); }
        else ww(c,dr,v);
    }
}

/* ── LDCTL ── move between a general register and a CPU control register.
 * Privileged. Direction is bit 3 of the opcode (0 = ctrl→Rd read, 1 = Rs→ctrl
 * write); bits 2:0 select the control register: 2=FCW, 3=REFRESH, 4=PSAP seg,
 * 5=PSAP off, 6=NSP seg, 7=NSP off. Writing FCW goes through change_fcw so the
 * SP bank swap happens; PSAP seg keeps only the 7-bit segment (bits 14:8). */
static void exec_ldctl(CPU *c){
    if (!check_priv(c)) return;
    uint16_t op=c->opcode[0]; uint8_t ra=nib2(op); uint8_t cr=op&0x07;
    if (!(op&0x08)) {  /* read: Rd ← control register */
        uint16_t val=0;
        switch (cr){ case 2:val=c->fcw;break; case 3:val=c->refresh;break; case 4:val=(uint16_t)(c->psap>>8);break; case 5:val=(uint16_t)c->psap;break; case 6:val=c->nspseg;break; case 7:val=c->nspoff;break; }
        ww(c,ra,val);
    } else {           /* write: control register ← Rs */
        uint16_t val=rw(c,ra);
        switch (cr){
            case 2: change_fcw(c,val); break;                                     /* FCW (may swap SP bank) */
            case 3: c->refresh=val; break;
            case 4: c->psap=(c->psap&0x0000FFFF)|((uint32_t)(val&0x7F00)<<8); break; /* PSAP segment (internal form) */
            case 5: c->psap=(c->psap&0xFFFF0000)|val; break;                       /* PSAP offset */
            case 6: c->nspseg=val; break;                                          /* saved normal-SP segment */
            case 7: c->nspoff=val; break;                                          /* saved normal-SP offset */
        }
    }
}

/* ── LDPS ── Load Program Status: atomically load a new FCW and PC from a
 * 4-word (segmented) or 2-word (non-seg) block in memory. Privileged. Used for
 * context switching. Segmented layout at base: [reserved][FCW][PCseg][PCoff];
 * PCseg word carries the 7-bit segment in bits 14:8 (same packing as reset). */
static void exec_ldps(CPU *c, const CtrlWord *w){
    if (!check_priv(c)) return;
    uint32_t base;
    if (w->src2==SrcIR) base=addr_from_reg(c,nib2(c->opcode[0]));
    else { base=seg_addr_op(c,1); uint8_t idx=nib2(c->opcode[0]); if(idx) base=addr_add(base,rw(c,idx)); }
    if (SEG(c)) {
        uint16_t nf=mem_r16(c,addr_add(base,2));
        uint32_t pcseg=(uint32_t)(mem_r16(c,addr_add(base,4))&0x7F00)<<8;
        uint16_t pcoff=mem_r16(c,addr_add(base,6));
        c->pc=pcseg|pcoff; change_fcw(c,nf);
    } else {
        uint16_t nf=mem_r16(c,base); uint16_t pc=mem_r16(c,addr_add(base,2));
        c->pc=pc; change_fcw(c,nf);
    }
}

/* ── IRET ── Interrupt return: undo what the trap/interrupt entry pushed, in
 * reverse order. The entry sequence (handle_interrupts) pushed PC, then FCW,
 * then tag; the CPU pops tag (discarded), FCW, then PC. Segmented pops the PC
 * as two words (segment high word first). change_fcw is applied last so the
 * restored FCW's mode drives the SP bank swap back to the interrupted context.
 * Privileged. */
static void exec_iret(CPU *c){
    if (!check_priv(c)) return;
    pop_word(c);                 /* tag (discard) */
    uint16_t nf=pop_word(c);     /* FCW */
    if (SEG(c)) { uint16_t hi=pop_word(c); uint16_t lo=pop_word(c); c->pc=seg_addr(((uint32_t)hi<<16)|lo); }
    else c->pc=pop_word(c);
    change_fcw(c,nf);
}

/* ── exchange ── EX/EXB: swap two word/byte registers, or swap a register with
 * a memory operand (IR/DA/X). nib2=a, nib3=b. No flags. */
static void exec_exchange(CPU *c, const CtrlWord *w){
    uint16_t op=c->opcode[0]; uint8_t a=nib2(op), b=nib3(op);
    if (w->src2==SrcIR||w->src2==SrcDA||w->src2==SrcX){
        uint32_t ma;
        if (w->src2==SrcIR) ma=addr_from_reg(c,a);
        else if (w->src2==SrcDA) ma=seg_addr_op(c,1);
        else ma=addr_add(seg_addr_op(c,1),rw(c,a));
        if (w->width==WByte){ uint8_t mv=mem_r8(c,ma); uint8_t rv=rb(c,b); wb(c,b,mv); mem_w8(c,ma,rv); }
        else { uint16_t mv=mem_r16(c,ma); uint16_t rv=rw(c,b); ww(c,b,mv); mem_w16(c,ma,rv); }
    } else {
        if (w->width==WByte){ uint8_t va=rb(c,a),vb=rb(c,b); wb(c,a,vb); wb(c,b,va); }
        else { uint16_t va=rw(c,a),vb=rw(c,b); ww(c,a,vb); ww(c,b,va); }
    }
}

/* translate: TR* / TRT* family (TRIB/TRDB/TRIRB/… — one element per call).
 * Byte translate through a table: fetch a byte at @dstReg, use it as an index
 * into the table based at @srcReg, read the translated byte. Registers come
 * from opcode[1] (count=11:8, dst=7:4) and opcode[0] nib2 (src/table base).
 *   io_write=true  (TR*):  write the translated byte back to @dstReg; single
 *                          forms set Z from the translated byte.
 *   io_write=false (TRT*): translated byte goes to RH1 (reg 1); Z set from it.
 * Both advance dstReg by ±1, decrement count. Repeat forms set repeat_pending
 * so cpu_step re-runs the instruction: TR* repeats while count≠0; TRT* also
 * requires translated==0 (stop when a non-zero table entry is found). */
static void exec_translate(CPU *c, const CtrlWord *w){
    uint8_t dstReg=(c->opcode[1]>>4)&0x0F, cntReg=(c->opcode[1]>>8)&0x0F, srcReg=nib2(c->opcode[0]);
    int16_t delta = w->block_decr ? -1 : 1;
    uint8_t idx=mem_r8(c, addr_from_reg(c,dstReg));                       /* byte to translate */
    uint8_t translated=mem_r8(c, addr_add(addr_from_reg(c,srcReg), idx)); /* table[idx] */
    if (w->io_write){
        mem_w8(c, addr_from_reg(c,dstReg), translated);
        add_to_addr_reg(c,dstReg,delta);
        uint16_t cnt=rw(c,cntReg)-1; ww(c,cntReg,cnt);
        if (!w->block_rept){ if(translated==0) c->fcw|=F_Z; else c->fcw&=~F_Z; }
        if (w->block_rept && cnt!=0) c->repeat_pending=true;
    } else {
        wb(c,1,translated); /* RH1 */
        add_to_addr_reg(c,dstReg,delta);
        uint16_t cnt=rw(c,cntReg)-1; ww(c,cntReg,cnt);
        if (translated==0) c->fcw|=F_Z; else c->fcw&=~F_Z;
        if (w->block_rept && cnt!=0 && translated==0) c->repeat_pending=true;
    }
}

/* ── block transfer/compare/string — one element per call ──
 * All three copy/scan a run of elements, advancing pointer(s) by ±width and
 * decrementing the count register each iteration. For repeat (*R) forms,
 * repeat_pending is set when the loop should continue, and cpu_step's fast
 * path re-runs execute() in place (interruptible between iterations). Register
 * fields: opcode[0] nib2 = source pointer; opcode[1] 11:8 = count, 7:4 = dst
 * pointer / compare reg, 3:0 = condition code (compare/string). */

/* LDI/LDD/LDIR/LDDR: memory→memory copy. V (PV) set when count hits 0. */
static void exec_block_transfer(CPU *c, const CtrlWord *w){
    uint8_t srcReg=nib2(c->opcode[0]), dstReg=(c->opcode[1]>>4)&0x0F, cntReg=(c->opcode[1]>>8)&0x0F;
    int16_t delta = w->block_decr ? -(int16_t)w->width : (int16_t)w->width;
    if (w->width==WByte){ uint8_t v=mem_r8(c,addr_from_reg(c,srcReg)); mem_w8(c,addr_from_reg(c,dstReg),v); }
    else { uint16_t v=mem_r16(c,addr_from_reg(c,srcReg)); mem_w16(c,addr_from_reg(c,dstReg),v); }
    add_to_addr_reg(c,srcReg,delta); add_to_addr_reg(c,dstReg,delta);
    uint16_t cnt=rw(c,cntReg)-1; ww(c,cntReg,cnt);
    if (cnt==0) c->fcw|=F_PV; else c->fcw&=~F_PV;
    if (w->block_rept && cnt!=0) c->repeat_pending=true;
}
/* CPI/CPD/CPIR/CPDR: compare a register (cmpReg) against @srcReg using cc.
 * The raw CP flags are applied so cond_true(cc) can be evaluated; final flags
 * then set Z=match (cc satisfied) and PV=(count reached 0). Repeat stops on a
 * match or when count hits 0. */
static void exec_block_compare(CPU *c, const CtrlWord *w){
    uint8_t srcReg=nib2(c->opcode[0]), cmpReg=(c->opcode[1]>>4)&0x0F, cntReg=(c->opcode[1]>>8)&0x0F;
    uint8_t cc=c->opcode[1]&0x0F;
    int16_t delta = w->block_decr ? -(int16_t)w->width : (int16_t)w->width;
    AluResult r;
    if (w->width==WByte) r=alu_exec(OpCp, rb(c,cmpReg), mem_r8(c,addr_from_reg(c,srcReg)), false, WByte);
    else r=alu_exec(OpCp, rw(c,cmpReg), mem_r16(c,addr_from_reg(c,srcReg)), false, WWord);
    apply_flags(c,r);                    /* apply raw CP flags so cond_true(cc) is meaningful */
    bool match=cond_true(c,cc);
    add_to_addr_reg(c,srcReg,delta);
    uint16_t cnt=rw(c,cntReg)-1; ww(c,cntReg,cnt);
    uint16_t flags=r.flags & ~(F_Z|F_PV);
    if (match) flags|=F_Z; if (cnt==0) flags|=F_PV;   /* Z=match, PV=count exhausted */
    c->fcw=(c->fcw & ~(F_Z|F_PV)) | (flags & (F_Z|F_PV));
    if (w->block_rept && cnt!=0 && !match) c->repeat_pending=true;
}
/* CPSI/CPSD/CPSIR/CPSDR: like block-compare but compares two MEMORY strings
 * (@srcReg vs @dstReg) rather than a register vs memory; advances both. */
static void exec_block_string(CPU *c, const CtrlWord *w){
    uint8_t srcReg=nib2(c->opcode[0]), dstReg=(c->opcode[1]>>4)&0x0F, cntReg=(c->opcode[1]>>8)&0x0F;
    uint8_t cc=c->opcode[1]&0x0F;
    int16_t delta = w->block_decr ? -(int16_t)w->width : (int16_t)w->width;
    AluResult r;
    if (w->width==WByte) r=alu_exec(OpCp, mem_r8(c,addr_from_reg(c,srcReg)), mem_r8(c,addr_from_reg(c,dstReg)), false, WByte);
    else r=alu_exec(OpCp, mem_r16(c,addr_from_reg(c,srcReg)), mem_r16(c,addr_from_reg(c,dstReg)), false, WWord);
    apply_flags(c,r);
    bool match=cond_true(c,cc);
    add_to_addr_reg(c,srcReg,delta); add_to_addr_reg(c,dstReg,delta);
    uint16_t cnt=rw(c,cntReg)-1; ww(c,cntReg,cnt);
    uint16_t flags=r.flags & ~(F_Z|F_PV);
    if (match) flags|=F_Z; if (cnt==0) flags|=F_PV;
    c->fcw=(c->fcw & ~(F_Z|F_PV)) | (flags & (F_Z|F_PV));
    if (w->block_rept && cnt!=0 && !match) c->repeat_pending=true;
}

/* ── block I/O — one element per call ── INI/IND/OUTI/OUTD and variants.
 * Privileged. Transfers between an I/O port and a memory block, advancing the
 * memory pointer by ±width and decrementing count. PV set when count hits 0. */
static void exec_block_io(CPU *c, const CtrlWord *w){
    if (!check_priv(c)) return;
    uint16_t ew=c->opcode[1];
    uint8_t cntReg=(ew>>8)&0x0F;
    /* Register roles are swapped between input and output:
     *   input  (IN*):  port = nib2(op),      memory = (ew>>4)&0xF
     *   output (OUT*): memory = nib2(op),    port   = (ew>>4)&0xF   */
    uint8_t addrReg, portReg;
    if (w->io_write) { addrReg = nib2(c->opcode[0]); portReg = (ew>>4)&0x0F; }
    else             { portReg = nib2(c->opcode[0]); addrReg = (ew>>4)&0x0F; }
    int16_t delta = w->block_decr ? -(int16_t)w->width : (int16_t)w->width;
    uint16_t port=rw(c,portReg); uint32_t ma=addr_from_reg(c,addrReg);
    if (!w->io_write){
        uint16_t v=io_read(c->m,port,w->width==WByte,w->io_spec);
        if (w->width==WByte) mem_w8(c,ma,w->io_spec?(uint8_t)v:(uint8_t)(v>>8)); else mem_w16(c,ma,v);
    } else {
        if (w->width==WByte){ uint8_t b=mem_r8(c,ma); io_write(c->m,port,w->io_spec?b:(uint16_t)b<<8,true,w->io_spec); }
        else io_write(c->m,port,mem_r16(c,ma),false,w->io_spec);
    }
    add_to_addr_reg(c,addrReg,delta);
    uint16_t cnt=rw(c,cntReg)-1; ww(c,cntReg,cnt);
    if (cnt==0) c->fcw|=F_PV; else c->fcw&=~F_PV;
    if (w->block_rept && cnt!=0) c->repeat_pending=true;
}

/* ── dispatch a decoded instruction ──
 * Route the CtrlWord to its execute-class handler. The class is the "next-state
 * selector" the decode ROM emits; most classes have a dedicated exec_* above,
 * the simpler ones are handled inline here. */
static void execute(CPU *c, const CtrlWord *w){
    switch (w->exec){
        case ClsUnified: exec_unified(c,w); break;
        case ClsFcwOp: {
            /* FCW bit ops: DI/EI (interrupt-enable bits) / SETFLG/RESFLG/COMFLG
             * (condition-flag bits) / LDCTLB FLAGS. Touching any bit outside the
             * low condition-flag byte (mask & ~0x00FC, e.g. the VIE/NVIE enables)
             * is privileged. */
            bool isPriv=(w->fcw_op==FcwSet||w->fcw_op==FcwClear) && (w->fcw_mask & ~0x00FCu);
            if (isPriv && !check_priv(c)) break;
            switch (w->fcw_op){
                case FcwSet: c->fcw|=w->fcw_mask; break;      /* EI / SETFLG */
                case FcwClear: c->fcw&=~w->fcw_mask; break;   /* DI / RESFLG */
                case FcwToggle: c->fcw^=w->fcw_mask; break;   /* COMFLG */
                case FcwLoadLo: { uint8_t v=rb(c,nib2(c->opcode[0])); c->fcw=(c->fcw&~0x00FCu)|(v&0x00FC); break; } /* LDCTLB FLAGS,Rb: copy bits 7:2 */
                case FcwStoreLo: wb(c,nib2(c->opcode[0]),(uint8_t)c->fcw&0xFC); break;                            /* LDCTLB Rb,FLAGS */
            }
            break; }
        case ClsExts: {
            /* EXTSB/EXTS/EXTSL: sign-extend the low half of a reg group into the
             * whole group (byte→word, word→long pair, long→quad). No flags. */
            uint8_t reg=nib2(c->opcode[0]);
            if (w->width==WByte){ int8_t b=(int8_t)(uint8_t)rw(c,reg); ww(c,reg,(uint16_t)(int16_t)b); }
            else if (w->width==WWord){ uint8_t base=reg&0x0E; int16_t v=(int16_t)rw(c,base+1); wl(c,base,(uint32_t)(int32_t)v); }
            else { uint8_t base=reg&0x0C; int32_t lo=(int32_t)rl(c,base+2); wq(c,base,(uint64_t)(int64_t)lo); }
            break; }
        case ClsStack: exec_stack(c,w); break;
        case ClsBranch: exec_branch(c,w); break;
        case ClsTCC: {
            /* TCC/TCCB: if condition cc holds, set bit 0 of the dst register
             * (used to materialise a condition into a boolean). */
            uint8_t dst=nib2(c->opcode[0]), cc=nib3(c->opcode[0]);
            if (cond_true(c,cc)){ if(w->width==WByte) wb(c,dst,rb(c,dst)|1); else ww(c,dst,rw(c,dst)|1); }
            break; }
        case ClsDAB: {
            /* DAB: decimal-adjust a byte register after BCD add/sub, using H and
             * C as ALU inputs. */
            uint8_t dst=nib2(c->opcode[0]);
            AluResult r=alu_exec(OpDab, rb(c,dst), (c->fcw&F_H)?1:0, c->fcw&F_C, WByte);
            wb(c,dst,(uint8_t)r.value); apply_flags(c,r); break; }
        case ClsExchange: exec_exchange(c,w); break;
        case ClsLEA: {
            /* LDA/LDAR: compute an effective address and store it in the dst
             * register (nib3) — no memory access. */
            uint8_t dst=nib3(c->opcode[0]); uint32_t addr=0;
            switch (w->src2){
                case SrcPCRel: addr=displaced(c,c->pc,(int16_t)c->opcode[1]); break;
                case SrcDA: addr=seg_addr_op(c,1); break;
                case SrcX: addr=addr_add(seg_addr_op(c,1),rw(c,unib(c,w->src2_nib))); break;
                case SrcBased: addr=displaced(c,addr_from_reg(c,unib(c,w->src2_nib)),(int16_t)c->opcode[1]); break;
                case SrcBX: { uint32_t b=addr_from_reg(c,unib(c,w->src2_nib)); uint8_t idx=(c->opcode[1]>>8)&0x0F; addr=addr_add(b,rw(c,idx)); break; }
                default: break;
            }
            addr_to_reg(c,dst,addr); break; }
        case ClsBCDRotate: {
            /* RLDB/RRDB: rotate a BCD digit (nibble) between two byte registers.
             * RL (OpRl): dst low nibble ← src high nibble, etc.; RR the reverse.
             * Z set from the resulting dst byte. */
            uint16_t op=c->opcode[0]; uint8_t src=nib2(op),dst=nib3(op);
            uint8_t sv=rb(c,src),dv=rb(c,dst),oldlo=dv&0x0F,ns,nd;
            if (w->alu_op==OpRl){ nd=(dv&0xF0)|(sv>>4); ns=(sv<<4)|oldlo; }
            else { nd=(dv&0xF0)|(sv&0x0F); ns=(sv>>4)|(oldlo<<4); }
            wb(c,src,ns); wb(c,dst,nd);
            if (nd==0) c->fcw|=F_Z; else c->fcw&=~F_Z; break; }
        case ClsBitDynamic: {
            /* BIT/SET/RES with a register-supplied bit number: rs (opcode[0]
             * nib3) holds the bit index (low 3 bits for byte, low 4 for word),
             * rd (opcode[1] 11:8) is the target. BIT tests (sets Z from the
             * complemented bit); SET/RES modify rd in place. */
            uint8_t rs=nib3(c->opcode[0]), rd=(c->opcode[1]>>8)&0x0F;
            if (w->width==WByte){ uint8_t bit=(uint8_t)rw(c,rs)&0x07;
                switch(w->alu_op){ case OpBit: if((rb(c,rd)&(1<<bit))==0)c->fcw|=F_Z; else c->fcw&=~F_Z; break;
                    case OpSet: wb(c,rd,rb(c,rd)|(1<<bit)); break; case OpRes: wb(c,rd,rb(c,rd)&~(1<<bit)); break; default:break; } }
            else { uint8_t bit=(uint8_t)rw(c,rs)&0x0F;
                switch(w->alu_op){ case OpBit: if((rw(c,rd)&(1<<bit))==0)c->fcw|=F_Z; else c->fcw&=~F_Z; break;
                    case OpSet: ww(c,rd,rw(c,rd)|(1<<bit)); break; case OpRes: ww(c,rd,rw(c,rd)&~(1<<bit)); break; default:break; } }
            break; }
        case ClsIO: exec_io(c,w); break;
        case ClsLDCTL: exec_ldctl(c); break;
        case ClsShift: exec_shift(c,w); break;
        case ClsBlockTransfer: exec_block_transfer(c,w); break;
        case ClsBlockCompare: exec_block_compare(c,w); break;
        case ClsBlockString: exec_block_string(c,w); break;
        case ClsBlockIO: exec_block_io(c,w); break;
        case ClsLDM: exec_ldm(c,w); break;
        case ClsMultiProc: {
            /* MBIT/MREQ multi-micro arbitration (privileged). On the C900 the
             * MI (multi-micro input) pin is tied to ground / there is no bus
             * arbitration peer, so these reduce to fixed outcomes: MBIT always
             * sets S, and MREQ takes the "request granted" path (S and Z set).
             * The real mi_n pin is here effectively low. */
            if (!check_priv(c)) break;
            switch (w->mp_op){
                case MPBit: if(!c->m->cpu.nmi_line){} if(true){ /* MI tied GND → S=1 */ c->fcw|=F_S; } break;
                case MPReq: { c->fcw&=~F_Z; c->fcw|=F_S; c->fcw|=F_Z; break; } /* MI low path: granted */
                default: break;
            }
            break; }
        case ClsMul: exec_mul(c,w); break;
        case ClsDiv: exec_div(c,w); break;
        case ClsLDPS: exec_ldps(c,w); break;
        case ClsHalt: if (check_priv(c)) c->halted=true; break;          /* HALT: privileged; wait for interrupt */
        case ClsSysCall: c->irq_req|=IRQ_SYSCALL; break;                 /* SC: request a system-call trap */
        case ClsIRET: exec_iret(c); break;
        case ClsTranslate: exec_translate(c,w); break;
    }
}

/* ── PSA / vectors ──
 * The Program Status Area (PSA) is a table in memory pointed to by PSAP holding
 * the new-{FCW,PC} for each trap/interrupt. On the segmented Z8001 each fixed
 * entry is a longword (2 reserved bytes, FCW, PC-seg, PC-off) — hence the ×2
 * scaling of word offsets below. */
static uint32_t psa_addr(CPU *c){ return c->psap & 0x007FFFFF; }
static uint32_t vector_addr(CPU *c, uint32_t off){ return addr_add(psa_addr(c), 2*off); } /* segmented: mult 2 */

/* PSA vector offsets (in words, pre-×2). V_BASE begins the per-device VI table. */
#define V_EPU 0x0004u      /* extended-processor / illegal-instruction */
#define V_TRAP 0x0008u     /* privileged-instruction trap */
#define V_SYSCALL 0x000Cu  /* SC system call */
#define V_SEGTRAP 0x0010u  /* MMU segment trap */
#define V_NMI 0x0014u      /* non-maskable interrupt */
#define V_NVI 0x0018u      /* non-vectored interrupt */
#define V_VI 0x001Cu       /* vectored interrupt (fixed FCW slot) */
#define V_BASE 0x001Eu     /* base of per-vector VI PC table (indexed by device byte) */

/* Take a pending trap/interrupt at an instruction boundary. Returns true if one
 * was taken (the caller then returns without executing an instruction).
 * Collapses the multi-Tick bus sequence into one atomic step. Priority
 * (highest first):
 *   EPU > TRAP > SYSCALL > SEGTRAP > NMI > NVI > VI.
 * Internal traps (EPU/TRAP/SYSCALL) carry the offending opcode as their stack
 * "tag"; external interrupts take a device vector (0 for NMI/NVI). */
static bool handle_interrupts(CPU *c){
    if (c->irq_req==0 && !c->nvi_line && !c->vi_line && !c->nmi_line && !c->segt_line) {
        /* nothing */
    }
    /* Sample the level-sensitive request lines into irq_req. NMI and segment
     * trap are always latched; NVI/VI are gated by their FCW enable bits. */
    if (c->nmi_line) c->irq_req |= IRQ_NMI;
    if (c->segt_line) c->irq_req |= IRQ_SEGTRAP;
    if (c->nvi_line && (c->fcw & FCW_NVIE)) c->irq_req |= IRQ_NVI;
    if (c->vi_line && (c->fcw & FCW_VIE)) c->irq_req |= IRQ_VI;

    if (c->irq_req==0) return false;

    /* Select the highest-priority pending source. intack_kind marks the
     * external kinds (1=SEGT,2=NMI,3=NVI,4=VI); internal traps set internal. */
    uint32_t vecoff; uint8_t bit; bool internal=false; uint8_t intack_kind=0;
    if (c->irq_req & IRQ_EPU) { vecoff=V_EPU; bit=IRQ_EPU; internal=true; }
    else if (c->irq_req & IRQ_TRAP) { vecoff=V_TRAP; bit=IRQ_TRAP; internal=true; }
    else if (c->irq_req & IRQ_SYSCALL) { vecoff=V_SYSCALL; bit=IRQ_SYSCALL; internal=true; }
    else if (c->irq_req & IRQ_SEGTRAP) { vecoff=V_SEGTRAP; bit=IRQ_SEGTRAP; intack_kind=1; }
    else if (c->irq_req & IRQ_NMI) { vecoff=V_NMI; bit=IRQ_NMI; intack_kind=2; }
    else if ((c->irq_req & IRQ_NVI) && (c->fcw&FCW_NVIE)) { vecoff=V_NVI; bit=IRQ_NVI; intack_kind=3; }
    else if ((c->irq_req & IRQ_VI) && (c->fcw&FCW_VIE)) { vecoff=V_VI; bit=IRQ_VI; intack_kind=4; }
    else return false;

    uint16_t oldfcw=c->fcw;                  /* FCW to push (the interrupted context) */
    uint32_t vecaddr=vector_addr(c, vecoff);
    uint16_t tag = internal ? c->cur_op : 0; /* internal: offending opcode; external: 0 */
    uint32_t pervec=0;

    /* External VI: the device supplies a vector byte during INTACK. A non-zero
     * vector selects a per-device PC entry in the VI table (V_BASE + vector),
     * while the FCW still comes from the fixed VI slot at vecaddr. */
    if (intack_kind==4) {
        tag = c->vi_vector;
        if (tag != 0) {
            uint32_t offbytes = V_BASE + (tag & 0xFF);
            offbytes <<= 1; /* segmented: byte offset */
            pervec = addr_add(psa_addr(c), offbytes);
        }
    }

    /* Enter system+segmented mode BEFORE pushing so the pushes land on the
     * system stack (change_fcw swaps in the system SP bank). */
    uint16_t nm = c->fcw | FCW_SN | FCW_SEG;
    change_fcw(c, nm);
    c->irq_req &= ~bit;

    /* Push the interrupted state: segmented PC (4 bytes: high/segment word then
     * low/offset word), then old FCW, then the tag word. IRET pops these back. */
    {
        uint8_t sp=sp_reg(c);
        /* PC: high word then low word */
        uint32_t a=addr_sub(addr_from_reg(c,sp),4); addr_to_reg(c,sp,a);
        uint32_t segpc=make_seg(c->pc);
        mem_w16(c,a,(uint16_t)(segpc>>16));
        mem_w16(c,addr_add(a,2),(uint16_t)segpc);
    }
    push_word(c, oldfcw);
    push_word(c, tag);

    /* Fetch the handler's new FCW and PC from the PSA. Segmented fixed entry:
     * FCW at vecaddr+2, PC-seg at +4, PC-off at +6. For a VI with a device
     * vector, the PC (seg,off) instead comes from the 4-byte per-vector slot. */
    uint16_t nf = mem_r16(c, addr_add(vecaddr, 2)); /* seg: FCW at +2 */
    uint32_t pcseg, pcoff;
    if (pervec != 0) {
        pcseg = mem_r16(c, pervec);
        pcoff = mem_r16(c, addr_add(pervec,2));
    } else {
        pcseg = mem_r16(c, addr_add(vecaddr,4));
        pcoff = mem_r16(c, addr_add(vecaddr,6));
    }
    /* pcseg word holds the 7-bit segment in bits 14:8 (same packing as reset). */
    c->pc = (uint32_t)((pcseg & 0x7F00) << 8) | (pcoff & 0xFFFF);
    change_fcw(c, nf);          /* install handler FCW (applied after PSA reads) */
    c->halted = false;          /* an interrupt wakes a halted CPU */
    c->serviced = bit;          /* report which source was serviced this step */
    return true;
}

/* Power-on / hardware reset. Clears the register file and CPU state, then loads
 * the initial FCW and PC from the reset vector at the bottom of physical memory
 * (the ROM sits there): FCW from physical word 0x0002, PC segment from 0x0004
 * (the 7-bit segment lives in bits 14:8, so shift <<8 into internal form), and
 * PC offset from 0x0006. (Word 0x0000 is reserved.) These reads are physical
 * (untranslated) because the MMU is not yet meaningful at reset. */
void cpu_reset(CPU *c){
    Machine *m=c->m;
    memset(c->R,0,sizeof c->R);
    c->fcw=0; c->pc=0; c->psap=0; c->refresh=0; c->nspseg=0; c->nspoff=0;
    c->halted=false; c->irq_req=0;
    c->nvi_line=c->vi_line=c->nmi_line=c->segt_line=false; c->nmi_prev=false;
    c->m=m;
    /* reset reads FCW@0x0002, PCseg@0x0004, PCoff@0x0006 (ProgSpace = ROM) */
    c->fcw = phys_read16(m, 0x0002);
    uint16_t pcseg = phys_read16(m, 0x0004);
    uint16_t pcoff = phys_read16(m, 0x0006);
    c->pc = (uint32_t)((pcseg & 0x7F00) << 8) | pcoff;
}

/* Execute exactly one instruction, or take one pending trap/interrupt.
 * Pipeline: (1) service interrupts at the boundary; (2) if halted, idle;
 * (3) fetch word 0, determine instruction length, fetch the rest; (4) decode;
 * (5) advance PC past the instruction; (6) execute; (7) handle faults and
 * block-repeat continuation. */
/* ── debug: rolling pre-execute trace ring (CSIM_TRACE_RING=1) ──
 * Records (PC, opcode words, FCW, all registers) BEFORE each instruction
 * executes; dumped on an MMU fault so the provenance of a bad pointer can be
 * traced backwards. Zero overhead unless the env var is set. */
typedef struct { uint32_t pc; uint16_t op[3]; uint16_t fcw; uint16_t R[16]; } TraceEnt;
#define TRING_N 8192
static TraceEnt tring[TRING_N];
static unsigned tring_pos, tring_fill;
static int tring_on = -1;   /* -1 = unqueried, 0 = off, 1 = on */

static void tring_dump(FILE *f, unsigned max_normal){
    /* print the most recent `max_normal` NORMAL-mode entries, oldest first */
    unsigned n = tring_fill, start = (tring_pos + TRING_N - n) % TRING_N;
    unsigned cnt = 0;
    /* first pass: count normal-mode entries so we can skip the excess */
    for (unsigned i=0;i<n;i++) if (!(tring[(start+i)%TRING_N].fcw & FCW_SN)) cnt++;
    unsigned skip = cnt > max_normal ? cnt - max_normal : 0;
    for (unsigned i=0;i<n;i++){
        TraceEnt *e = &tring[(start+i)%TRING_N];
        if (e->fcw & FCW_SN) continue;
        if (skip){ skip--; continue; }
        fprintf(f, "[tr] %02X:%04X %04X %04X %04X |",
                (e->pc>>16)&0x7F, e->pc&0xFFFF, e->op[0], e->op[1], e->op[2]);
        for (int r=0;r<16;r++) fprintf(f," %04X", e->R[r]);
        fprintf(f,"\n");
    }
}

void cpu_step(CPU *c){
    c->serviced = 0;
    /* service pending traps/interrupts at instruction boundary */
    if (handle_interrupts(c)) return;
    if (c->halted) return;

    c->fault=false; c->repeat_pending=false;
    c->instr_start = c->pc;              /* remember start so faults/repeats can rewind */
    bool segmented = SEG(c);

    /* fetch word 0 and derive its length and segmented-address shape */
    uint16_t op0 = ifetch16(c, c->pc);
    c->opcode[0]=op0;
    int need = decode_length(op0, segmented);              /* max words this opcode may occupy */
    c->seg_w1 = decode_seg_addr_w1(op0, segmented);        /* does word 1 hold a segmented DA? */
    c->seg_w2 = decode_seg_addr_w2(op0, segmented);        /* does word 2? */

    /* fetch remaining words (peek forward from PC; PC is committed below) */
    uint32_t p = c->pc;
    for (int i=1;i<need && i<6;i++){ p=addr_add(p,2); c->opcode[i]=ifetch16(c,p); }
    /* Segmented short-form reduction: a segmented DA operand is 2 words in long
     * form (bit15 set) but only 1 in short form (bit15 clear, 8-bit offset).
     * When short, the instruction is one word shorter than the worst case. */
    if (need>=3 && c->seg_w1 && !(c->opcode[1]&0x8000)) need--;
    if (need>=4 && c->seg_w2 && !(c->opcode[2]&0x8000)) need--;
    c->oplen=need;

    const CtrlWord *w = decode_lookup(op0, &c->opcode[1], need-1<0?0:need, segmented);
    if (!w) {
        /* Unrecognised encoding: advance past it and raise the EPU/illegal
         * trap (taken at the next boundary), tagging it with the opcode. */
        c->cur_op=op0;
        c->pc = addr_add(c->pc, (uint32_t)need*2);
        c->irq_req |= IRQ_EPU;
        return;
    }
    c->cur_op=op0;

    if (tring_on < 0) tring_on = getenv("CSIM_TRACE_RING") ? 1 : 0;
    if (tring_on) {
        TraceEnt *e = &tring[tring_pos];
        e->pc = c->instr_start; e->fcw = c->fcw;
        e->op[0]=c->opcode[0]; e->op[1]=c->opcode[1]; e->op[2]=c->opcode[2];
        memcpy(e->R, c->R, sizeof e->R);
        tring_pos = (tring_pos + 1) % TRING_N;
        if (tring_fill < TRING_N) tring_fill++;
    }

    /* advance PC past the instruction before executing (branches overwrite it) */
    uint32_t pc_next = addr_add(c->instr_start, (uint32_t)need*2);
    c->pc = pc_next;

    execute(c, w);
    c->insns++;
    /* Debug: CSIM_BREAK_INSNS=N — dump registers + trace ring when the
     * instruction counter hits N (runs are deterministic, so an event found
     * by a watchpoint can be revisited with full history). */
    {
        static long long brk = -2;
        if (brk == -2) { const char *s = getenv("CSIM_BREAK_INSNS"); brk = s ? atoll(s) : -1; }
        if (brk >= 0 && c->insns == (uint64_t)brk) {
            fprintf(stderr, "[brk] insns=%llu pc=%02X:%04X op=%04X %04X %04X\n     R:",
                    (unsigned long long)c->insns, (c->instr_start>>16)&0x7F,
                    c->instr_start&0xFFFF, c->opcode[0], c->opcode[1], c->opcode[2]);
            for (int i=0;i<16;i++) fprintf(stderr," %04X", c->R[i]);
            fprintf(stderr,"\n");
            if (tring_on == 1) tring_dump(stderr, 3000);
        }
    }
    /* An MMU write WARNING (PWW — stack-guard page hit) completed its write;
     * take the segment trap at the next boundary WITHOUT rewinding PC, so the
     * handler (Coherent's stack-grow path) returns to the next instruction. */
    if (c->m->mmu.warn_pending) { c->m->mmu.warn_pending=false; c->irq_req |= IRQ_SEGTRAP; }
    /* A hard MMU fault (red zone) aborts the instruction: the store was
     * suppressed and the Z8010 pushes the address of the FOLLOWING
     * instruction.  Coherent's stack-grow path (trap.c stviol) depends on
     * this: it pattern-matches the prologue store by looking AT the pushed
     * PC for the `ld r13,r15` that follows it, then rewinds the saved PC
     * itself by the store length and re-executes it on the grown stack.
     * (Rewinding to instr_start here made stviol's getuwi(pc) read the
     * store opcode instead -> pattern miss -> spurious SIGSEGV for any
     * frame big enough to jump the yellow guard page, e.g. scat's 1120
     * bytes; the shipped /bin/scat died identically.)  Use pc_next, not
     * c->pc: a faulting CALL/branch already overwrote c->pc with the
     * target. */
    if (c->fault) {
        if (getenv("CSIM_MMU_DEBUG")) {
            fprintf(stderr, "[cpu] fault @%02X:%04X op=%04X %04X %04X (%s) insns=%llu\n     R:",
                    (c->instr_start>>16)&0x7F, c->instr_start&0xFFFF,
                    c->opcode[0], c->opcode[1], c->opcode[2], w->name,
                    (unsigned long long)c->insns);
            for (int i=0;i<16;i++) fprintf(stderr," %04X", c->R[i]);
            fprintf(stderr,"\n     desc:");
            for (int s=0;s<8;s++){
                MmuDesc *d=&c->m->mmu.desc[s];
                fprintf(stderr," %d:%04X/%02X/%02X", s, d->base, d->limit, d->attr);
            }
            fprintf(stderr,"\n");
            if (tring_on == 1) tring_dump(stderr, 2000);
        }
        c->irq_req |= IRQ_SEGTRAP; c->pc = pc_next; return;
    }

    /* Block-repeat fast path (LDIR/CPIR/…)
     * processes one element per execute. Rather than rewind PC and re-fetch/
     * re-decode each element (very slow for the megabyte POST memory test),
     * loop the already-decoded instruction in place. The op is interruptible,
     * so break out if an interrupt becomes pending+enabled (leaving PC rewound
     * to re-enter on the next step) — this preserves interrupt latency. */
    while (c->repeat_pending) {
        c->repeat_pending = false;
        /* would a pending interrupt/trap preempt the next iteration? */
        if (c->nmi_line || c->segt_line ||
            (c->vi_line  && (c->fcw & FCW_VIE)) ||
            (c->nvi_line && (c->fcw & FCW_NVIE)) || c->irq_req) {
            c->pc = c->instr_start;
            return;
        }
        execute(c, w);
        c->insns++;
        /* Write warning inside a block op: raise the trap; the loop-head check
         * above will rewind PC so the (interruptible) block op re-enters after
         * the handler and continues from its register state. */
        if (c->m->mmu.warn_pending) { c->m->mmu.warn_pending=false; c->irq_req |= IRQ_SEGTRAP; }
        if (c->fault) { c->irq_req |= IRQ_SEGTRAP; c->pc = c->instr_start; return; }
    }
}
