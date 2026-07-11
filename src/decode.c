/* decode.c — Z8000 decode ROM.
 *
 * Produces a CtrlWord (the hardware-style control vector consumed by the
 * execute engine) from the opcode word, and where needed the following
 * word(s) / segmentation mode.
 *
 * Structure. The Z8000's first opcode byte (op>>8) selects the instruction
 * family, so decoding is a 256-entry table indexed by that byte (`rom[]`).
 * Each RomEntry is one of two kinds:
 *   - `ctrl != NULL`: the whole 0xNN block is a single instruction; the
 *     CtrlWord is returned directly.
 *   - `sub  != NULL`: the encoding is ambiguous within the block and a
 *     sub-decoder refines it using the low byte (nib2 = bits[7:4],
 *     nib3 = bits[3:0]) and/or the second opcode word.
 * decode_init() builds the table once and pre-allocates every CtrlWord in a
 * static pool. decode_lookup() then does an O(1) table read + optional
 * sub-decode. Instruction length is precomputed into g_len_tab for O(1)
 * per-fetch lookup. Mirrors buildDecodeROM() exactly. */
#include "emu.h"
#include <string.h>

/* Field extractors for the two low nibbles of the opcode word. Many Z8000
 * encodings put the second register / a mode selector in nib2 and a sub-op
 * or register in nib3. */
#define NIB2(op) (((op) >> 4) & 0x0F)
#define NIB3(op) ((op) & 0x0F)

/* A sub-decoder: given the opcode word plus up to `navail` following words,
 * return the refined CtrlWord (or NULL if the extra words are not yet
 * available / the encoding is illegal). */
typedef CtrlWord *(*SubFn)(uint16_t op, const uint16_t *words, int navail);

/* One decode-table slot: exactly one of ctrl/sub is set (see file header). */
typedef struct { CtrlWord *ctrl; SubFn sub; } RomEntry;
static RomEntry rom[256];

/* precomputed instruction length: [segmented][op0] → words. O(1) per fetch. */
static uint8_t g_len_tab[2][65536];
static int decode_length_compute(uint16_t op, bool segmented);

/* Pool of statically-allocated CtrlWords. We build them at init. Every
 * CtrlWord the decoder ever returns lives here; decode_lookup hands out
 * const pointers into this pool (immutable, never freed). */
#define POOL_MAX 2048
static CtrlWord pool[POOL_MAX];
static int pool_n;

/* Allocate and zero one CtrlWord from the pool, pre-set to the common
 * defaults: valid, word width, single-word length. */
static CtrlWord *cw(void) {
    CtrlWord *c = &pool[pool_n++];
    memset(c, 0, sizeof *c);
    c->valid = true;
    c->width = WWord;
    c->words = 1;
    return c;
}

/* ── sub-decoder helpers ── */
/* Most ambiguity in the Z8000 map is a simple two-way fork keyed by whether a
 * nibble is zero. In many encodings nib2==0 selects a Direct-Address / PC-
 * relative / immediate form and nib2!=0 selects an indexed / base+reg form
 * (nib2 then being the index register); nib3==0 vs !=0 plays the same role for
 * the PUSH/POP addressed forms. Rather than one function per opcode we key a
 * pair of {zero, nonzero} CtrlWords by high byte and share a generic fork. */

typedef struct { CtrlWord *zero, *nonzero; } Fork;
static Fork fork2[256];   /* nib2==0 fork */
static Fork fork3[256];   /* nib3==0 fork (PUSH/POP DA/X) */

/* Generic nib2==0 fork: zero → DA/PCrel/imm form, nonzero → X/based form. */
static CtrlWord *sub_nib2fork(uint16_t op, const uint16_t *w, int n) {
    (void)w; (void)n;
    Fork *f = &fork2[op >> 8];
    return (NIB2(op) == 0) ? f->zero : f->nonzero;
}
/* Generic nib3==0 fork (PUSH/POP with DA vs X destination/source). */
static CtrlWord *sub_nib3fork(uint16_t op, const uint16_t *w, int n) {
    (void)w; (void)n;
    Fork *f = &fork3[op >> 8];
    return (NIB3(op) == 0) ? f->zero : f->nonzero;
}

/* ── generic ALU imm/IR fork for 0x00-0x0B ── */
/* nib2==0 → immediate operand (extra word follows); nib2!=0 → @Rs register
 * indirect (nib2 = the pointer register). */
static CtrlWord *imm_ir[256][2]; /* [hi][0]=imm(nib2==0), [1]=IR */
static CtrlWord *sub_immir(uint16_t op, const uint16_t *w, int n) {
    (void)w; (void)n;
    return imm_ir[op >> 8][NIB2(op) == 0 ? 0 : 1];
}
/* DA/X fork for 0x40-0x4B: nib2==0 → Direct Address, else indexed by nib2. */
static CtrlWord *da_x[256][2];
static CtrlWord *sub_dax(uint16_t op, const uint16_t *w, int n) {
    (void)w; (void)n;
    return da_x[op >> 8][NIB2(op) == 0 ? 0 : 1];
}

/* ── tables for the multi-way sub-decoders ──
 * Blocks whose low byte selects among many instructions use dedicated lookup
 * tables (indexed by nib3, and sometimes a second-word bit or nib2). Filled
 * in decode_init(); consulted by the matching sub_XX functions below. */
static CtrlWord *tbl3A[16][2], *tbl3B[16][2];
static CtrlWord *tblB2[16], *tblB3[16];
static CtrlWord *tblBA[16][2], *tblBB[16][2];
static CtrlWord *tbl4C[2][16], *tbl4D[2][16];
static CtrlWord *tbl0C[16], *tbl0D[16];
static CtrlWord *tbl8C[16], *tbl8D[16][16];
static CtrlWord *tblB8[8];
static CtrlWord *tblDIEI[8];

/* nib3 values in the 0xB2/0xB3 blocks that are 2-word shifts (need word 1
 * available before they can be decoded); the rest are 1-word rotates. */
static int oddB2[16] = {[0x01]=1,[0x03]=1,[0x09]=1,[0x0B]=1};
static int oddB3[16] = {[0x01]=1,[0x03]=1,[0x05]=1,[0x07]=1,[0x09]=1,[0x0B]=1,[0x0D]=1,[0x0F]=1};

/* 0x3A block-I/O byte forms. nib3 (op&0x0F) picks the group; the second
 * word's low nibble ==8 selects the decrement/alternate sibling ("sw"). */
static CtrlWord *sub_3A(uint16_t op, const uint16_t *w, int n) {
    if (n < 1) return NULL;                       /* need word 1 to resolve sw */
    int sw = (w[0] & 0x0F) == 8 ? 1 : 0;
    return tbl3A[op & 0x0F][sw];
}
/* 0x3B block-I/O word forms — word-width sibling of sub_3A. */
static CtrlWord *sub_3B(uint16_t op, const uint16_t *w, int n) {
    if (n < 1) return NULL;
    int sw = (w[0] & 0x0F) == 8 ? 1 : 0;
    return tbl3B[op & 0x0F][sw];
}
/* 0xB2 byte rotate/shift: nib3 selects the op; odd nib3 (2-word shift) needs
 * word 1 present before it can be returned. */
static CtrlWord *sub_B2(uint16_t op, const uint16_t *w, int n) {
    int nib3 = op & 0x0F;
    if (oddB2[nib3] && n < 1) return NULL;
    return tblB2[nib3];
}
/* 0xB3 word/long rotate/shift — sibling of sub_B2. */
static CtrlWord *sub_B3(uint16_t op, const uint16_t *w, int n) {
    int nib3 = op & 0x0F;
    if (oddB3[nib3] && n < 1) return NULL;
    return tblB3[nib3];
}
/* 0xBA byte block compare/transfer/string: nib3 picks the family; word 1
 * bit3 selects the increment vs decrement / repeat sibling. */
static CtrlWord *sub_BA(uint16_t op, const uint16_t *w, int n) {
    if (n < 1) return NULL;
    int b3 = (w[0] & 0x08) ? 1 : 0;
    return tblBA[op & 0x0F][b3];
}
/* 0xBB word block compare/transfer/string — sibling of sub_BA. */
static CtrlWord *sub_BB(uint16_t op, const uint16_t *w, int n) {
    if (n < 1) return NULL;
    int b3 = (w[0] & 0x08) ? 1 : 0;
    return tblBB[op & 0x0F][b3];
}
/* 0x4C byte unary-with-address: nib2!=0 → indexed (X), nib2==0 → DA; nib3
 * selects COM/CP/NEG/TEST/LD/TSET/CLR. */
static CtrlWord *sub_4C(uint16_t op, const uint16_t *w, int n) {
    (void)w;(void)n; return tbl4C[NIB2(op)?1:0][op & 0x0F];
}
/* 0x4D word unary-with-address — sibling of sub_4C. */
static CtrlWord *sub_4D(uint16_t op, const uint16_t *w, int n) {
    (void)w;(void)n; return tbl4D[NIB2(op)?1:0][op & 0x0F];
}
/* 0x0C/0x0D byte/word unary via @Rs; nib3 selects the operation. */
static CtrlWord *sub_0C(uint16_t op, const uint16_t *w, int n){(void)w;(void)n;return tbl0C[op&0x0F];}
static CtrlWord *sub_0D(uint16_t op, const uint16_t *w, int n){(void)w;(void)n;return tbl0D[op&0x0F];}
/* 0x8C byte register unary/LDCTLB; nib3 selects the operation. */
static CtrlWord *sub_8C(uint16_t op, const uint16_t *w, int n){(void)w;(void)n;return tbl8C[op&0x0F];}
/* 0x8D word register unary + flag ops (SETFLG/RESFLG/COMFLG): nib3 selects the
 * op and nib2 supplies the flag mask (used by the flag-op entries). */
static CtrlWord *sub_8D(uint16_t op, const uint16_t *w, int n){(void)w;(void)n;return tbl8D[NIB2(op)][op&0x0F];}

/* 0x9C: only nib3==8 is defined — TESTL RRd (long test). Everything else in
 * the block is illegal (NULL). */
static CtrlWord *sub_9C(uint16_t op, const uint16_t *w, int n) {
    (void)w;(void)n;
    if ((op & 0x0F) == 0x08) {
        static CtrlWord *t; if (!t) { t = cw(); t->name="TESTL"; t->exec=ClsUnified; t->alu_op=OpTest; t->width=WLong; t->src1_nib=RegNib2; t->src2=SrcNone; t->dst=DstNone; }
        return t;
    }
    return NULL;
}
/* 0xB1: sign-extend register forms. nib3 picks the width — 0x00 EXTSB
 * (byte→word), 0x07 EXTSL (long→quad), 0x0A EXTS (word→long); others illegal. */
static CtrlWord *sub_B1(uint16_t op, const uint16_t *w, int n) {
    (void)w;(void)n;
    static CtrlWord *b,*l,*ww; if(!b){
        b=cw();b->name="EXTSB";b->exec=ClsExts;b->width=WByte;
        l=cw();l->name="EXTSL";l->exec=ClsExts;l->width=WLong;
        ww=cw();ww->name="EXTS";ww->exec=ClsExts;ww->width=WWord;
    }
    switch(op&0x0F){case 0x00:return b;case 0x07:return l;case 0x0A:return ww;}
    return NULL;
}
/* 0x1C: LDM @Rs (load/store multiple) and TESTL @Rs. nib3 selects: 0x01 =
 * LDM load (needs the count word), 0x09 = LDM store, 0x08 = TESTL. */
static CtrlWord *sub_1C(uint16_t op, const uint16_t *w, int n) {
    static CtrlWord *ld,*st,*tl; if(!ld){
        ld=cw();ld->name="LDM";ld->exec=ClsLDM;ld->src2=SrcIR;ld->words=2;
        st=cw();st->name="LDM";st->exec=ClsLDM;st->src2=SrcIR;st->io_write=true;st->words=2;
        tl=cw();tl->name="TESTL";tl->exec=ClsUnified;tl->alu_op=OpTest;tl->width=WLong;tl->src2=SrcIR;tl->src2_nib=RegNib2;tl->dst=DstFlagsOnly;
    }
    switch(op&0x0F){
        case 0x01: if(n>=1) return ld; break;
        case 0x09: if(n>=1) return st; break;
        case 0x08: return tl;
    }
    return NULL;
}
/* 0x5C: addressed LDM (load/store multiple, DA or indexed) and TESTL with an
 * address operand. nib3: 0x08 = TESTL, 0x01 = LDM load, 0x09 = LDM store;
 * nib2==0 → Direct Address, nib2!=0 → indexed by nib2. LDM forms need the
 * count word (n>=2). */
static CtrlWord *sub_5C(uint16_t op, const uint16_t *w, int n) {
    static CtrlWord *tlDA,*tlX,*ldDA,*ldX,*stDA,*stX; if(!tlDA){
        tlDA=cw();tlDA->name="TESTL";tlDA->exec=ClsUnified;tlDA->alu_op=OpTest;tlDA->width=WLong;tlDA->src2=SrcDA;tlDA->dst=DstFlagsOnly;tlDA->words=2;
        tlX=cw();tlX->name="TESTL";tlX->exec=ClsUnified;tlX->alu_op=OpTest;tlX->width=WLong;tlX->src2=SrcX;tlX->src2_nib=RegNib2;tlX->dst=DstFlagsOnly;tlX->words=2;
        ldDA=cw();ldDA->name="LDM";ldDA->exec=ClsLDM;ldDA->src2=SrcDA;ldDA->words=3;
        ldX=cw();ldX->name="LDM";ldX->exec=ClsLDM;ldX->src2=SrcX;ldX->src2_nib=RegNib2;ldX->words=3;
        stDA=cw();stDA->name="LDM";stDA->exec=ClsLDM;stDA->src2=SrcDA;stDA->io_write=true;stDA->words=3;
        stX=cw();stX->name="LDM";stX->exec=ClsLDM;stX->src2=SrcX;stX->src2_nib=RegNib2;stX->io_write=true;stX->words=3;
    }
    switch(op&0x0F){
        case 0x08: if(n>=1) return NIB2(op)==0?tlDA:tlX; break;
        case 0x01: if(n>=2) return NIB2(op)==0?ldDA:ldX; break;
        case 0x09: if(n>=2) return NIB2(op)==0?stDA:stX; break;
    }
    return NULL;
}
/* 0x7B: privileged / multi-processor group. nib3==0x0D with nib2!=0 → MREQ
 * (multi-micro request); otherwise the whole low byte selects IRET (0x00),
 * MSET (0x08), MRES (0x09), MBIT (0x0A). */
static CtrlWord *sub_7B(uint16_t op, const uint16_t *w, int n) {
    (void)w;(void)n;
    static CtrlWord *mreq,*iret,*mset,*mres,*mbit; if(!mreq){
        mreq=cw();mreq->name="MREQ";mreq->exec=ClsMultiProc;mreq->mp_op=MPReq;
        iret=cw();iret->name="IRET";iret->exec=ClsIRET;
        mset=cw();mset->name="MSET";mset->exec=ClsMultiProc;mset->mp_op=MPSet;
        mres=cw();mres->name="MRES";mres->exec=ClsMultiProc;mres->mp_op=MPRes;
        mbit=cw();mbit->name="MBIT";mbit->exec=ClsMultiProc;mbit->mp_op=MPBit;
    }
    if ((op & 0x0F) == 0x0D && NIB2(op) != 0) return mreq;
    switch (op & 0xFF) {
        case 0x00: return iret;
        case 0x08: return mset;
        case 0x09: return mres;
        case 0x0A: return mbit;
    }
    return NULL;
}
/* 0x7C: DI/EI (disable/enable interrupts). The low 3 bits select which
 * interrupt-enable bits (VIE/NVIE) are affected and set vs clear. */
static CtrlWord *sub_7C(uint16_t op, const uint16_t *w, int n){(void)w;(void)n;return tblDIEI[op&0x07];}
/* 0x7D: LDCTL — load/store control register. Direction (load vs store) and
 * the control-register index are decoded at execute time from the opcode
 * bits, so a single CtrlWord serves the whole block. */
static CtrlWord *sub_7D(uint16_t op, const uint16_t *w, int n){
    (void)op;(void)w;(void)n;
    static CtrlWord *t; if(!t){t=cw();t->name="LDCTL";t->exec=ClsLDCTL;}
    return t;
}
/* 0xB8: table-lookup / translate block ops (TRIB..TRTDRB). Only even low
 * bytes are valid; bits[3:1] select one of 8 variants (dir/repeat/translate). */
static CtrlWord *sub_B8(uint16_t op, const uint16_t *w, int n){
    (void)w;(void)n;
    if (op & 0x01) return NULL;
    return tblB8[(op>>1)&0x07];
}

/* ── helpers to define entries ── */
/* set_ctrl: whole 0xNN block is one instruction. set_sub: block needs a
 * sub-decoder. These are mutually exclusive per slot (one field NULLs the
 * other). */
static void set_ctrl(int hi, CtrlWord *c) { rom[hi].ctrl = c; rom[hi].sub = NULL; }
static void set_sub(int hi, SubFn f) { rom[hi].ctrl = NULL; rom[hi].sub = f; }

/* Build a standard register-register ALU CtrlWord: src1 = Rd (nib3),
 * src2 = Rs (nib2), dst = Rd — except compare (OpCp) writes flags only. */
static CtrlWord *mk_reg(const char *name, AluOp op, Width w) {
    CtrlWord *c = cw();
    c->name = name; c->exec = ClsUnified; c->alu_op = op; c->width = w; c->words = 1;
    c->src1_nib = RegNib3; c->src2 = SrcReg; c->src2_nib = RegNib2;
    c->dst = (op == OpCp) ? DstNone : DstReg; c->dst_nib = RegNib3;
    return c;
}

/* Build the 256-entry decode table and all sub-decoder lookup tables, then
 * precompute the instruction-length table. Idempotent (guarded by `done`) and
 * called once before any decode_lookup. Everything below populates rom[] via
 * set_ctrl/set_sub and fills the fork/table arrays the sub_XX functions read. */
void decode_init(void) {
    static bool done = false;
    if (done) return;
    done = true;
    pool_n = 0;
    memset(rom, 0, sizeof rom);

    /* 0x80-0x8B register-register ALU */
    set_ctrl(0x80, mk_reg("ADDB", OpAdd, WByte));
    set_ctrl(0x81, mk_reg("ADD",  OpAdd, WWord));
    set_ctrl(0x82, mk_reg("SUBB", OpSub, WByte));
    set_ctrl(0x83, mk_reg("SUB",  OpSub, WWord));
    set_ctrl(0x84, mk_reg("ORB",  OpOr,  WByte));
    set_ctrl(0x85, mk_reg("OR",   OpOr,  WWord));
    set_ctrl(0x86, mk_reg("ANDB", OpAnd, WByte));
    set_ctrl(0x87, mk_reg("AND",  OpAnd, WWord));
    set_ctrl(0x88, mk_reg("XORB", OpXor, WByte));
    set_ctrl(0x89, mk_reg("XOR",  OpXor, WWord));
    set_ctrl(0x8A, mk_reg("CPB",  OpCp,  WByte));
    set_ctrl(0x8B, mk_reg("CP",   OpCp,  WWord));

    /* 0x00-0x0B ALU imm/@Rs */
    struct { int b, wd; const char *nm; AluOp op; } aluir[] = {
        {0x00,0x01,"ADD",OpAdd},{0x02,0x03,"SUB",OpSub},{0x04,0x05,"OR",OpOr},
        {0x06,0x07,"AND",OpAnd},{0x08,0x09,"XOR",OpXor},{0x0A,0x0B,"CP",OpCp},
    };
    for (unsigned i = 0; i < sizeof(aluir)/sizeof(aluir[0]); i++) {
        DstSel d = (aluir[i].op == OpCp) ? DstNone : DstReg;
        /* byte */
        CtrlWord *bi = cw(); bi->name=aluir[i].nm; bi->exec=ClsUnified; bi->alu_op=aluir[i].op; bi->width=WByte; bi->words=2; bi->src1_nib=RegNib3; bi->src2=SrcImm; bi->dst=d; bi->dst_nib=RegNib3;
        CtrlWord *br = cw(); br->name=aluir[i].nm; br->exec=ClsUnified; br->alu_op=aluir[i].op; br->width=WByte; br->words=1; br->src1_nib=RegNib3; br->src2=SrcIR; br->src2_nib=RegNib2; br->dst=d; br->dst_nib=RegNib3;
        imm_ir[aluir[i].b][0]=bi; imm_ir[aluir[i].b][1]=br; set_sub(aluir[i].b, sub_immir);
        CtrlWord *wi = cw(); wi->name=aluir[i].nm; wi->exec=ClsUnified; wi->alu_op=aluir[i].op; wi->width=WWord; wi->words=2; wi->src1_nib=RegNib3; wi->src2=SrcImm; wi->dst=d; wi->dst_nib=RegNib3;
        CtrlWord *wr = cw(); wr->name=aluir[i].nm; wr->exec=ClsUnified; wr->alu_op=aluir[i].op; wr->width=WWord; wr->words=1; wr->src1_nib=RegNib3; wr->src2=SrcIR; wr->src2_nib=RegNib2; wr->dst=d; wr->dst_nib=RegNib3;
        imm_ir[aluir[i].wd][0]=wi; imm_ir[aluir[i].wd][1]=wr; set_sub(aluir[i].wd, sub_immir);
    }

    /* 0xA0/0xA1 LD reg,reg */
    { CtrlWord *c=cw(); c->name="LDB";c->exec=ClsUnified;c->alu_op=OpLd;c->width=WByte;c->src2=SrcReg;c->src2_nib=RegNib2;c->dst=DstReg;c->dst_nib=RegNib3; set_ctrl(0xA0,c); }
    { CtrlWord *c=cw(); c->name="LD"; c->exec=ClsUnified;c->alu_op=OpLd;c->width=WWord;c->src2=SrcReg;c->src2_nib=RegNib2;c->dst=DstReg;c->dst_nib=RegNib3; set_ctrl(0xA1,c); }

    set_sub(0x8C, sub_8C);
    set_sub(0x8D, sub_8D);

    /* 0x40-0x4B ALU DA/X */
    struct { int b, wd; const char *nm; AluOp op; } alud[] = {
        {0x40,0x41,"ADD",OpAdd},{0x42,0x43,"SUB",OpSub},{0x44,0x45,"OR",OpOr},
        {0x46,0x47,"AND",OpAnd},{0x48,0x49,"XOR",OpXor},{0x4A,0x4B,"CP",OpCp},
    };
    for (unsigned i = 0; i < sizeof(alud)/sizeof(alud[0]); i++) {
        DstSel d = (alud[i].op == OpCp) ? DstNone : DstReg;
        CtrlWord *bd=cw(); bd->name=alud[i].nm;bd->exec=ClsUnified;bd->alu_op=alud[i].op;bd->width=WByte;bd->words=2;bd->src1_nib=RegNib3;bd->src2=SrcDA;bd->dst=d;bd->dst_nib=RegNib3;
        CtrlWord *bx=cw(); bx->name=alud[i].nm;bx->exec=ClsUnified;bx->alu_op=alud[i].op;bx->width=WByte;bx->words=2;bx->src1_nib=RegNib3;bx->src2=SrcX;bx->src2_nib=RegNib2;bx->dst=d;bx->dst_nib=RegNib3;
        da_x[alud[i].b][0]=bd; da_x[alud[i].b][1]=bx; set_sub(alud[i].b, sub_dax);
        CtrlWord *wd=cw(); wd->name=alud[i].nm;wd->exec=ClsUnified;wd->alu_op=alud[i].op;wd->width=WWord;wd->words=2;wd->src1_nib=RegNib3;wd->src2=SrcDA;wd->dst=d;wd->dst_nib=RegNib3;
        CtrlWord *wx=cw(); wx->name=alud[i].nm;wx->exec=ClsUnified;wx->alu_op=alud[i].op;wx->width=WWord;wx->words=2;wx->src1_nib=RegNib3;wx->src2=SrcX;wx->src2_nib=RegNib2;wx->dst=d;wx->dst_nib=RegNib3;
        da_x[alud[i].wd][0]=wd; da_x[alud[i].wd][1]=wx; set_sub(alud[i].wd, sub_dax);
    }

    /* 0x60/0x61 LD DA/X */
    { CtrlWord *z=cw(),*nz=cw();
      z->name="LDB";z->exec=ClsUnified;z->alu_op=OpLd;z->width=WByte;z->words=2;z->src2=SrcDA;z->dst=DstReg;z->dst_nib=RegNib3;
      nz->name="LDB";nz->exec=ClsUnified;nz->alu_op=OpLd;nz->width=WByte;nz->words=2;nz->src2=SrcX;nz->src2_nib=RegNib2;nz->dst=DstReg;nz->dst_nib=RegNib3;
      fork2[0x60].zero=z;fork2[0x60].nonzero=nz;set_sub(0x60,sub_nib2fork); }
    { CtrlWord *z=cw(),*nz=cw();
      z->name="LD";z->exec=ClsUnified;z->alu_op=OpLd;z->width=WWord;z->words=2;z->src2=SrcDA;z->dst=DstReg;z->dst_nib=RegNib3;
      nz->name="LD";nz->exec=ClsUnified;nz->alu_op=OpLd;nz->width=WWord;nz->words=2;nz->src2=SrcX;nz->src2_nib=RegNib2;nz->dst=DstReg;nz->dst_nib=RegNib3;
      fork2[0x61].zero=z;fork2[0x61].nonzero=nz;set_sub(0x61,sub_nib2fork); }

    /* 0x2E/0x2F store @Rd,Rs */
    { CtrlWord *c=cw();c->name="LDB";c->exec=ClsUnified;c->alu_op=OpLd;c->width=WByte;c->src2=SrcReg;c->src2_nib=RegNib3;c->dst=DstIR;c->dst_nib=RegNib2;set_ctrl(0x2E,c);}
    { CtrlWord *c=cw();c->name="LD"; c->exec=ClsUnified;c->alu_op=OpLd;c->width=WWord;c->src2=SrcReg;c->src2_nib=RegNib3;c->dst=DstIR;c->dst_nib=RegNib2;set_ctrl(0x2F,c);}

    /* 0x6E/0x6F store DA/X,Rd */
    { CtrlWord *z=cw(),*nz=cw();
      z->name="LDB";z->exec=ClsUnified;z->alu_op=OpLd;z->width=WByte;z->words=2;z->src2=SrcReg;z->src2_nib=RegNib3;z->dst=DstDA;
      nz->name="LDB";nz->exec=ClsUnified;nz->alu_op=OpLd;nz->width=WByte;nz->words=2;nz->src2=SrcReg;nz->src2_nib=RegNib3;nz->dst=DstX;nz->dst_nib=RegNib2;
      fork2[0x6E].zero=z;fork2[0x6E].nonzero=nz;set_sub(0x6E,sub_nib2fork); }
    { CtrlWord *z=cw(),*nz=cw();
      z->name="LD";z->exec=ClsUnified;z->alu_op=OpLd;z->width=WWord;z->words=2;z->src2=SrcReg;z->src2_nib=RegNib3;z->dst=DstDA;
      nz->name="LD";nz->exec=ClsUnified;nz->alu_op=OpLd;nz->width=WWord;nz->words=2;nz->src2=SrcReg;nz->src2_nib=RegNib3;nz->dst=DstX;nz->dst_nib=RegNib2;
      fork2[0x6F].zero=z;fork2[0x6F].nonzero=nz;set_sub(0x6F,sub_nib2fork); }

    /* PUSH/POP register forms */
    { CtrlWord *c=cw();c->name="PUSH";c->exec=ClsStack;c->stk_mode=StackPush;set_ctrl(0x93,c);}
    { CtrlWord *c=cw();c->name="POP";c->exec=ClsStack;c->stk_mode=StackPop;set_ctrl(0x97,c);}
    { CtrlWord *c=cw();c->name="PUSHL";c->exec=ClsStack;c->stk_mode=StackPushL;set_ctrl(0x91,c);}
    { CtrlWord *c=cw();c->name="POPL";c->exec=ClsStack;c->stk_mode=StackPopL;set_ctrl(0x95,c);}
    { CtrlWord *c=cw();c->name="PUSH";c->exec=ClsStack;c->stk_mode=StackPushSrc;c->src2=SrcIR;set_ctrl(0x13,c);}
    { CtrlWord *c=cw();c->name="POP";c->exec=ClsStack;c->stk_mode=StackPopDst;c->dst=DstIR;set_ctrl(0x17,c);}
    { CtrlWord *c=cw();c->name="PUSHL";c->exec=ClsStack;c->stk_mode=StackPushLSrc;c->src2=SrcIR;set_ctrl(0x11,c);}
    { CtrlWord *c=cw();c->name="POPL";c->exec=ClsStack;c->stk_mode=StackPopLDst;c->dst=DstIR;set_ctrl(0x15,c);}
    /* DA/X 2-word forms sub-decoded by nib3 */
    { CtrlWord *z=cw(),*nz=cw();
      z->name="PUSH";z->exec=ClsStack;z->stk_mode=StackPushSrc;z->src2=SrcDA;z->words=2;
      nz->name="PUSH";nz->exec=ClsStack;nz->stk_mode=StackPushSrc;nz->src2=SrcX;nz->words=2;
      fork3[0x53].zero=z;fork3[0x53].nonzero=nz;set_sub(0x53,sub_nib3fork);}
    { CtrlWord *z=cw(),*nz=cw();
      z->name="POP";z->exec=ClsStack;z->stk_mode=StackPopDst;z->dst=DstDA;z->words=2;
      nz->name="POP";nz->exec=ClsStack;nz->stk_mode=StackPopDst;nz->dst=DstX;nz->words=2;
      fork3[0x57].zero=z;fork3[0x57].nonzero=nz;set_sub(0x57,sub_nib3fork);}
    { CtrlWord *z=cw(),*nz=cw();
      z->name="PUSHL";z->exec=ClsStack;z->stk_mode=StackPushLSrc;z->src2=SrcDA;z->words=2;
      nz->name="PUSHL";nz->exec=ClsStack;nz->stk_mode=StackPushLSrc;nz->src2=SrcX;nz->words=2;
      fork3[0x51].zero=z;fork3[0x51].nonzero=nz;set_sub(0x51,sub_nib3fork);}
    { CtrlWord *z=cw(),*nz=cw();
      z->name="POPL";z->exec=ClsStack;z->stk_mode=StackPopLDst;z->dst=DstDA;z->words=2;
      nz->name="POPL";nz->exec=ClsStack;nz->stk_mode=StackPopLDst;nz->dst=DstX;nz->words=2;
      fork3[0x55].zero=z;fork3[0x55].nonzero=nz;set_sub(0x55,sub_nib3fork);}

    /* branches */
    { CtrlWord *c=cw();c->name="JP";c->exec=ClsBranch;c->br_mode=BranchJP_IR;set_ctrl(0x1E,c);}
    { CtrlWord *z=cw(),*nz=cw();
      z->name="JP";z->exec=ClsBranch;z->br_mode=BranchJP_DA;z->words=2;
      nz->name="JP";nz->exec=ClsBranch;nz->br_mode=BranchJP_X;nz->words=2;
      fork2[0x5E].zero=z;fork2[0x5E].nonzero=nz;set_sub(0x5E,sub_nib2fork);}
    { CtrlWord *c=cw();c->name="CALL";c->exec=ClsBranch;c->br_mode=BranchCALL_IR;set_ctrl(0x1F,c);}
    { CtrlWord *z=cw(),*nz=cw();
      z->name="CALL";z->exec=ClsBranch;z->br_mode=BranchCALL_DA;z->words=2;
      nz->name="CALL";nz->exec=ClsBranch;nz->br_mode=BranchCALL_X;nz->words=2;
      fork2[0x5F].zero=z;fork2[0x5F].nonzero=nz;set_sub(0x5F,sub_nib2fork);}
    { CtrlWord *c=cw();c->name="RET";c->exec=ClsBranch;c->br_mode=BranchRET;set_ctrl(0x9E,c);}

    /* long register-register ALU */
    { CtrlWord *c=cw();c->name="CPL";c->exec=ClsUnified;c->alu_op=OpCp;c->width=WLong;c->src1_nib=RegNib3;c->src2=SrcReg;c->src2_nib=RegNib2;c->dst=DstNone;c->dst_nib=RegNib3;set_ctrl(0x90,c);}
    { CtrlWord *c=cw();c->name="SUBL";c->exec=ClsUnified;c->alu_op=OpSub;c->width=WLong;c->src1_nib=RegNib3;c->src2=SrcReg;c->src2_nib=RegNib2;c->dst=DstReg;c->dst_nib=RegNib3;set_ctrl(0x92,c);}
    { CtrlWord *c=cw();c->name="ADDL";c->exec=ClsUnified;c->alu_op=OpAdd;c->width=WLong;c->src1_nib=RegNib3;c->src2=SrcReg;c->src2_nib=RegNib2;c->dst=DstReg;c->dst_nib=RegNib3;set_ctrl(0x96,c);}

    /* mul/div reg-reg */
    { CtrlWord *c=cw();c->name="MULTL";c->exec=ClsMul;c->width=WLong;c->src2=SrcReg;set_ctrl(0x98,c);}
    { CtrlWord *c=cw();c->name="MULT";c->exec=ClsMul;c->width=WWord;c->src2=SrcReg;set_ctrl(0x99,c);}
    { CtrlWord *c=cw();c->name="DIVL";c->exec=ClsDiv;c->width=WLong;c->src2=SrcReg;set_ctrl(0x9A,c);}
    { CtrlWord *c=cw();c->name="DIV";c->exec=ClsDiv;c->width=WWord;c->src2=SrcReg;set_ctrl(0x9B,c);}
    set_sub(0x9C, sub_9C);

    set_sub(0xBA, sub_BA);
    set_sub(0xBB, sub_BB);
    set_sub(0x3A, sub_3A);
    set_sub(0x3B, sub_3B);

    /* LDL reg-reg / store */
    { CtrlWord *c=cw();c->name="LDL";c->exec=ClsUnified;c->alu_op=OpLd;c->width=WLong;c->src2=SrcReg;c->src2_nib=RegNib2;c->dst=DstReg;c->dst_nib=RegNib3;set_ctrl(0x94,c);}
    { CtrlWord *c=cw();c->name="LDL";c->exec=ClsUnified;c->alu_op=OpLd;c->width=WLong;c->src2=SrcReg;c->src2_nib=RegNib3;c->dst=DstIR;c->dst_nib=RegNib2;set_ctrl(0x1D,c);}

    /* long ALU imm/@Rs 0x10/0x12/0x14/0x16 */
    struct { int hi; const char *nm; AluOp op; } long10[] = {
        {0x10,"CPL",OpCp},{0x12,"SUBL",OpSub},{0x14,"LDL",OpLd},{0x16,"ADDL",OpAdd},
    };
    for (unsigned i=0;i<4;i++){
        DstSel d=(long10[i].op==OpCp)?DstNone:DstReg;
        CtrlWord *z=cw(),*nz=cw();
        z->name=long10[i].nm;z->exec=ClsUnified;z->alu_op=long10[i].op;z->width=WLong;z->words=3;z->src1_nib=RegNib3;z->src2=SrcImm;z->dst=d;z->dst_nib=RegNib3;
        nz->name=long10[i].nm;nz->exec=ClsUnified;nz->alu_op=long10[i].op;nz->width=WLong;nz->words=1;nz->src1_nib=RegNib3;nz->src2=SrcIR;nz->src2_nib=RegNib2;nz->dst=d;nz->dst_nib=RegNib3;
        fork2[long10[i].hi].zero=z;fork2[long10[i].hi].nonzero=nz;set_sub(long10[i].hi,sub_nib2fork);
    }
    /* mul/div imm/@Rs 0x18-0x1B.
     * NB: the @Rs forms MUST name the source register nibble (nib2). Leaving
     * src2_nib at its zero default (RegNib3 = the DST nibble) made e.g.
     * "MULTL RQ0,@RR6" fetch its multiplier through RR0 — the destination —
     * which sent Coherent's `as` chasing a garbage pointer into a segment
     * trap. The 0x58-0x5B indexed forms below always had this right. */
    { CtrlWord *z=cw(),*nz=cw();z->name="MULTL";z->exec=ClsMul;z->width=WLong;z->src2=SrcImm;z->words=3;nz->name="MULTL";nz->exec=ClsMul;nz->width=WLong;nz->src2=SrcIR;nz->src2_nib=RegNib2;nz->words=1;fork2[0x18].zero=z;fork2[0x18].nonzero=nz;set_sub(0x18,sub_nib2fork);}
    { CtrlWord *z=cw(),*nz=cw();z->name="MULT";z->exec=ClsMul;z->width=WWord;z->src2=SrcImm;z->words=2;nz->name="MULT";nz->exec=ClsMul;nz->width=WWord;nz->src2=SrcIR;nz->src2_nib=RegNib2;nz->words=1;fork2[0x19].zero=z;fork2[0x19].nonzero=nz;set_sub(0x19,sub_nib2fork);}
    { CtrlWord *z=cw(),*nz=cw();z->name="DIVL";z->exec=ClsDiv;z->width=WLong;z->src2=SrcImm;z->words=3;nz->name="DIVL";nz->exec=ClsDiv;nz->width=WLong;nz->src2=SrcIR;nz->src2_nib=RegNib2;nz->words=1;fork2[0x1A].zero=z;fork2[0x1A].nonzero=nz;set_sub(0x1A,sub_nib2fork);}
    { CtrlWord *z=cw(),*nz=cw();z->name="DIV";z->exec=ClsDiv;z->width=WWord;z->src2=SrcImm;z->words=2;nz->name="DIV";nz->exec=ClsDiv;nz->width=WWord;nz->src2=SrcIR;nz->src2_nib=RegNib2;nz->words=1;fork2[0x1B].zero=z;fork2[0x1B].nonzero=nz;set_sub(0x1B,sub_nib2fork);}

    set_sub(0x1C, sub_1C);

    /* LDB/LD imm/@Rs 0x20/0x21 */
    { CtrlWord *z=cw(),*nz=cw();
      z->name="LDB";z->exec=ClsUnified;z->alu_op=OpLd;z->width=WByte;z->words=2;z->src2=SrcImm;z->dst=DstReg;z->dst_nib=RegNib3;
      nz->name="LDB";nz->exec=ClsUnified;nz->alu_op=OpLd;nz->width=WByte;nz->words=1;nz->src2=SrcIR;nz->src2_nib=RegNib2;nz->dst=DstReg;nz->dst_nib=RegNib3;
      fork2[0x20].zero=z;fork2[0x20].nonzero=nz;set_sub(0x20,sub_nib2fork);}
    { CtrlWord *z=cw(),*nz=cw();
      z->name="LD";z->exec=ClsUnified;z->alu_op=OpLd;z->width=WWord;z->words=2;z->src2=SrcImm;z->dst=DstReg;z->dst_nib=RegNib3;
      nz->name="LD";nz->exec=ClsUnified;nz->alu_op=OpLd;nz->width=WWord;nz->words=1;nz->src2=SrcIR;nz->src2_nib=RegNib2;nz->dst=DstReg;nz->dst_nib=RegNib3;
      fork2[0x21].zero=z;fork2[0x21].nonzero=nz;set_sub(0x21,sub_nib2fork);}

    /* RES/SET/BIT IR / dynamic 0x22-0x27 — handled by dedicated subs */
    /* We build per-hi static/dynamic pair via fork on nib2, plus n<1 guard for dynamic. */
    {
        struct { int hi; const char *nm; AluOp op; Width w; bool ro; } bt[] = {
            {0x22,"RESB",OpRes,WByte,false},{0x23,"RES",OpRes,WWord,false},
            {0x24,"SETB",OpSet,WByte,false},{0x25,"SET",OpSet,WWord,false},
            {0x26,"BITB",OpBit,WByte,true},{0x27,"BIT",OpBit,WWord,true},
        };
        for (unsigned i=0;i<6;i++){
            DstSel dst = bt[i].ro ? DstFlagsOnly : DstIR;
            CtrlWord *st=cw();
            st->name=bt[i].nm;st->exec=ClsUnified;st->alu_op=bt[i].op;st->width=bt[i].w;st->words=1;
            st->src2=SrcIR;st->src2_nib=RegNib2;st->count=SrcNib3Bit;st->dst=dst;st->dst_nib=RegNib2;
            CtrlWord *dy=cw();
            dy->name=bt[i].nm;dy->exec=ClsBitDynamic;dy->alu_op=bt[i].op;dy->width=bt[i].w;dy->words=2;
            /* store in fork2 with special handling: dynamic needs n<1 guard.
               We reuse a per-hi custom sub via fork2 + a flag: implement inline below */
            fork2[bt[i].hi].zero=dy; fork2[bt[i].hi].nonzero=st; set_sub(bt[i].hi,sub_nib2fork);
        }
    }

    /* INC/DEC via @Rs 0x28-0x2B */
    { struct{int hi;const char*nm;AluOp op;Width w;} id[]={
        {0x28,"INCB",OpInc,WByte},{0x29,"INC",OpInc,WWord},{0x2A,"DECB",OpDec,WByte},{0x2B,"DEC",OpDec,WWord}};
      for(unsigned i=0;i<4;i++){CtrlWord *c=cw();c->name=id[i].nm;c->exec=ClsUnified;c->alu_op=id[i].op;c->width=id[i].w;c->words=1;c->src2=SrcIR;c->src2_nib=RegNib2;c->count=SrcNib3CntPlus1;c->dst=DstIR;c->dst_nib=RegNib2;set_ctrl(id[i].hi,c);} }

    { CtrlWord *c=cw();c->name="EXB";c->exec=ClsExchange;c->width=WByte;c->src2=SrcIR;set_ctrl(0x2C,c);}
    { CtrlWord *c=cw();c->name="EX";c->exec=ClsExchange;c->width=WWord;c->src2=SrcIR;set_ctrl(0x2D,c);}

    { CtrlWord *c=cw();c->name="LDPS";c->exec=ClsLDPS;c->src2=SrcIR;set_ctrl(0x39,c);}

    { CtrlWord *c=cw();c->name="INB";c->exec=ClsIO;c->width=WByte;c->src2=SrcIR;set_ctrl(0x3C,c);}
    { CtrlWord *c=cw();c->name="IN";c->exec=ClsIO;c->width=WWord;c->src2=SrcIR;set_ctrl(0x3D,c);}
    { CtrlWord *c=cw();c->name="OUTB";c->exec=ClsIO;c->width=WByte;c->src2=SrcIR;c->io_write=true;set_ctrl(0x3E,c);}
    { CtrlWord *c=cw();c->name="OUT";c->exec=ClsIO;c->width=WWord;c->src2=SrcIR;c->io_write=true;set_ctrl(0x3F,c);}

    set_sub(0x0C, sub_0C);
    set_sub(0x0D, sub_0D);

    /* PC-relative / displacement 0x30-0x37 */
    { CtrlWord *z=cw(),*nz=cw();
      z->name="LDB";z->exec=ClsUnified;z->alu_op=OpLd;z->width=WByte;z->words=2;z->src2=SrcPCRel;z->dst=DstReg;z->dst_nib=RegNib3;
      nz->name="LDB";nz->exec=ClsUnified;nz->alu_op=OpLd;nz->width=WByte;nz->words=2;nz->src2=SrcBased;nz->src2_nib=RegNib2;nz->dst=DstReg;nz->dst_nib=RegNib3;
      fork2[0x30].zero=z;fork2[0x30].nonzero=nz;set_sub(0x30,sub_nib2fork);}
    { CtrlWord *z=cw(),*nz=cw();
      z->name="LD";z->exec=ClsUnified;z->alu_op=OpLd;z->width=WWord;z->words=2;z->src2=SrcPCRel;z->dst=DstReg;z->dst_nib=RegNib3;
      nz->name="LD";nz->exec=ClsUnified;nz->alu_op=OpLd;nz->width=WWord;nz->words=2;nz->src2=SrcBased;nz->src2_nib=RegNib2;nz->dst=DstReg;nz->dst_nib=RegNib3;
      fork2[0x31].zero=z;fork2[0x31].nonzero=nz;set_sub(0x31,sub_nib2fork);}
    { CtrlWord *z=cw(),*nz=cw();
      z->name="LDB";z->exec=ClsUnified;z->alu_op=OpLd;z->width=WByte;z->words=2;z->src2=SrcReg;z->src2_nib=RegNib3;z->dst=DstPCRel;
      nz->name="LDB";nz->exec=ClsUnified;nz->alu_op=OpLd;nz->width=WByte;nz->words=2;nz->src2=SrcReg;nz->src2_nib=RegNib3;nz->dst=DstBased;nz->dst_nib=RegNib2;
      fork2[0x32].zero=z;fork2[0x32].nonzero=nz;set_sub(0x32,sub_nib2fork);}
    { CtrlWord *z=cw(),*nz=cw();
      z->name="LD";z->exec=ClsUnified;z->alu_op=OpLd;z->width=WWord;z->words=2;z->src2=SrcReg;z->src2_nib=RegNib3;z->dst=DstPCRel;
      nz->name="LD";nz->exec=ClsUnified;nz->alu_op=OpLd;nz->width=WWord;nz->words=2;nz->src2=SrcReg;nz->src2_nib=RegNib3;nz->dst=DstBased;nz->dst_nib=RegNib2;
      fork2[0x33].zero=z;fork2[0x33].nonzero=nz;set_sub(0x33,sub_nib2fork);}
    { CtrlWord *z=cw(),*nz=cw();
      z->name="LDAR";z->exec=ClsLEA;z->src2=SrcPCRel;z->words=2;
      nz->name="LDA";nz->exec=ClsLEA;nz->src2=SrcBased;nz->src2_nib=RegNib2;nz->words=2;
      fork2[0x34].zero=z;fork2[0x34].nonzero=nz;set_sub(0x34,sub_nib2fork);}
    { CtrlWord *z=cw(),*nz=cw();
      z->name="LDL";z->exec=ClsUnified;z->alu_op=OpLd;z->width=WLong;z->words=2;z->src2=SrcPCRel;z->dst=DstReg;z->dst_nib=RegNib3;
      nz->name="LDL";nz->exec=ClsUnified;nz->alu_op=OpLd;nz->width=WLong;nz->words=2;nz->src2=SrcBased;nz->src2_nib=RegNib2;nz->dst=DstReg;nz->dst_nib=RegNib3;
      fork2[0x35].zero=z;fork2[0x35].nonzero=nz;set_sub(0x35,sub_nib2fork);}
    { CtrlWord *z=cw(),*nz=cw();
      z->name="LDL";z->exec=ClsUnified;z->alu_op=OpLd;z->width=WLong;z->words=2;z->src2=SrcReg;z->src2_nib=RegNib3;z->dst=DstPCRel;
      nz->name="LDL";nz->exec=ClsUnified;nz->alu_op=OpLd;nz->width=WLong;nz->words=2;nz->src2=SrcReg;nz->src2_nib=RegNib3;nz->dst=DstBased;nz->dst_nib=RegNib2;
      fork2[0x37].zero=z;fork2[0x37].nonzero=nz;set_sub(0x37,sub_nib2fork);}

    set_sub(0x4C, sub_4C);
    set_sub(0x4D, sub_4D);

    /* long ALU with address 0x50/0x52/0x54/0x56 */
    { struct{int hi;const char*nm;AluOp op;DstSel d;} lg[]={
        {0x50,"CPL",OpCp,DstNone},{0x52,"SUBL",OpSub,DstReg},{0x54,"LDL",OpLd,DstReg},{0x56,"ADDL",OpAdd,DstReg}};
      for(unsigned i=0;i<4;i++){CtrlWord *z=cw(),*nz=cw();
        z->name=lg[i].nm;z->exec=ClsUnified;z->alu_op=lg[i].op;z->width=WLong;z->words=2;z->src1_nib=RegNib3;z->src2=SrcDA;z->dst=lg[i].d;z->dst_nib=RegNib3;
        nz->name=lg[i].nm;nz->exec=ClsUnified;nz->alu_op=lg[i].op;nz->width=WLong;nz->words=2;nz->src1_nib=RegNib3;nz->src2=SrcX;nz->src2_nib=RegNib2;nz->dst=lg[i].d;nz->dst_nib=RegNib3;
        fork2[lg[i].hi].zero=z;fork2[lg[i].hi].nonzero=nz;set_sub(lg[i].hi,sub_nib2fork);} }

    /* mul/div with address 0x58-0x5B */
    { CtrlWord *z=cw(),*nz=cw();z->name="MULTL";z->exec=ClsMul;z->width=WLong;z->src2=SrcDA;z->words=2;nz->name="MULTL";nz->exec=ClsMul;nz->width=WLong;nz->src2=SrcX;nz->src2_nib=RegNib2;nz->words=2;fork2[0x58].zero=z;fork2[0x58].nonzero=nz;set_sub(0x58,sub_nib2fork);}
    { CtrlWord *z=cw(),*nz=cw();z->name="MULT";z->exec=ClsMul;z->width=WWord;z->src2=SrcDA;z->words=2;nz->name="MULT";nz->exec=ClsMul;nz->width=WWord;nz->src2=SrcX;nz->src2_nib=RegNib2;nz->words=2;fork2[0x59].zero=z;fork2[0x59].nonzero=nz;set_sub(0x59,sub_nib2fork);}
    { CtrlWord *z=cw(),*nz=cw();z->name="DIVL";z->exec=ClsDiv;z->width=WLong;z->src2=SrcDA;z->words=2;nz->name="DIVL";nz->exec=ClsDiv;nz->width=WLong;nz->src2=SrcX;nz->src2_nib=RegNib2;nz->words=2;fork2[0x5A].zero=z;fork2[0x5A].nonzero=nz;set_sub(0x5A,sub_nib2fork);}
    { CtrlWord *z=cw(),*nz=cw();z->name="DIV";z->exec=ClsDiv;z->width=WWord;z->src2=SrcDA;z->words=2;nz->name="DIV";nz->exec=ClsDiv;nz->width=WWord;nz->src2=SrcX;nz->src2_nib=RegNib2;nz->words=2;fork2[0x5B].zero=z;fork2[0x5B].nonzero=nz;set_sub(0x5B,sub_nib2fork);}

    set_sub(0x5C, sub_5C);

    /* LDL store to addr 0x5D */
    { CtrlWord *z=cw(),*nz=cw();
      z->name="LDL";z->exec=ClsUnified;z->alu_op=OpLd;z->width=WLong;z->words=2;z->src2=SrcReg;z->src2_nib=RegNib3;z->dst=DstDA;
      nz->name="LDL";nz->exec=ClsUnified;nz->alu_op=OpLd;nz->width=WLong;nz->words=2;nz->src2=SrcReg;nz->src2_nib=RegNib3;nz->dst=DstX;nz->dst_nib=RegNib2;
      fork2[0x5D].zero=z;fork2[0x5D].nonzero=nz;set_sub(0x5D,sub_nib2fork);}

    /* BIT/SET/RES/INC/DEC with addr 0x62-0x6B */
    { struct{int hi;const char*nm;AluOp op;Width w;bool ro;} ba[]={
        {0x62,"RESB",OpRes,WByte,false},{0x63,"RES",OpRes,WWord,false},
        {0x64,"SETB",OpSet,WByte,false},{0x65,"SET",OpSet,WWord,false},
        {0x66,"BITB",OpBit,WByte,true},{0x67,"BIT",OpBit,WWord,true},
        {0x68,"INCB",OpInc,WByte,false},{0x69,"INC",OpInc,WWord,false},
        {0x6A,"DECB",OpDec,WByte,false},{0x6B,"DEC",OpDec,WWord,false}};
      for(unsigned i=0;i<10;i++){
        SrcSel count=(ba[i].op==OpInc||ba[i].op==OpDec)?SrcNib3CntPlus1:SrcNib3Bit;
        DstSel dd=ba[i].ro?DstFlagsOnly:DstDA, dx=ba[i].ro?DstFlagsOnly:DstX;
        CtrlWord *z=cw(),*nz=cw();
        z->name=ba[i].nm;z->exec=ClsUnified;z->alu_op=ba[i].op;z->width=ba[i].w;z->words=2;z->src2=SrcDA;z->src2_nib=RegNib2;z->count=count;z->dst=dd;z->dst_nib=RegNib2;
        nz->name=ba[i].nm;nz->exec=ClsUnified;nz->alu_op=ba[i].op;nz->width=ba[i].w;nz->words=2;nz->src2=SrcX;nz->src2_nib=RegNib2;nz->count=count;nz->dst=dx;nz->dst_nib=RegNib2;
        fork2[ba[i].hi].zero=z;fork2[ba[i].hi].nonzero=nz;set_sub(ba[i].hi,sub_nib2fork);} }

    /* EX with DA/X 0x6C/0x6D */
    { CtrlWord *z=cw(),*nz=cw();z->name="EXB";z->exec=ClsExchange;z->width=WByte;z->src2=SrcDA;z->words=2;nz->name="EXB";nz->exec=ClsExchange;nz->width=WByte;nz->src2=SrcX;nz->src2_nib=RegNib2;nz->words=2;fork2[0x6C].zero=z;fork2[0x6C].nonzero=nz;set_sub(0x6C,sub_nib2fork);}
    { CtrlWord *z=cw(),*nz=cw();z->name="EX";z->exec=ClsExchange;z->width=WWord;z->src2=SrcDA;z->words=2;nz->name="EX";nz->exec=ClsExchange;nz->width=WWord;nz->src2=SrcX;nz->src2_nib=RegNib2;nz->words=2;fork2[0x6D].zero=z;fork2[0x6D].nonzero=nz;set_sub(0x6D,sub_nib2fork);}

    /* LD/LDA/LDL BX 0x70-0x77 */
    { CtrlWord *c=cw();c->name="LDB";c->exec=ClsUnified;c->alu_op=OpLd;c->width=WByte;c->src2=SrcBX;c->src2_nib=RegNib2;c->dst=DstReg;c->dst_nib=RegNib3;c->words=2;set_ctrl(0x70,c);}
    { CtrlWord *c=cw();c->name="LD";c->exec=ClsUnified;c->alu_op=OpLd;c->width=WWord;c->src2=SrcBX;c->src2_nib=RegNib2;c->dst=DstReg;c->dst_nib=RegNib3;c->words=2;set_ctrl(0x71,c);}
    { CtrlWord *c=cw();c->name="LDB";c->exec=ClsUnified;c->alu_op=OpLd;c->width=WByte;c->src2=SrcReg;c->src2_nib=RegNib3;c->dst=DstBX;c->dst_nib=RegNib2;c->words=2;set_ctrl(0x72,c);}
    { CtrlWord *c=cw();c->name="LD";c->exec=ClsUnified;c->alu_op=OpLd;c->width=WWord;c->src2=SrcReg;c->src2_nib=RegNib3;c->dst=DstBX;c->dst_nib=RegNib2;c->words=2;set_ctrl(0x73,c);}
    { CtrlWord *c=cw();c->name="LDA";c->exec=ClsLEA;c->src2=SrcBX;c->src2_nib=RegNib2;c->words=2;set_ctrl(0x74,c);}
    { CtrlWord *c=cw();c->name="LDL";c->exec=ClsUnified;c->alu_op=OpLd;c->width=WLong;c->src2=SrcBX;c->src2_nib=RegNib2;c->dst=DstReg;c->dst_nib=RegNib3;c->words=2;set_ctrl(0x75,c);}
    { CtrlWord *z=cw(),*nz=cw();
      z->name="LDA";z->exec=ClsLEA;z->src2=SrcDA;z->words=2;
      nz->name="LDA";nz->exec=ClsLEA;nz->src2=SrcX;nz->src2_nib=RegNib2;nz->words=2;
      fork2[0x76].zero=z;fork2[0x76].nonzero=nz;set_sub(0x76,sub_nib2fork);}
    { CtrlWord *c=cw();c->name="LDL";c->exec=ClsUnified;c->alu_op=OpLd;c->width=WLong;c->src2=SrcReg;c->src2_nib=RegNib3;c->dst=DstBX;c->dst_nib=RegNib2;c->words=2;set_ctrl(0x77,c);}

    { CtrlWord *c=cw();c->name="LDPS";c->exec=ClsLDPS;c->src2=SrcDA;c->words=2;set_ctrl(0x79,c);}
    { CtrlWord *c=cw();c->name="HALT";c->exec=ClsHalt;set_ctrl(0x7A,c);}
    set_sub(0x7B, sub_7B);

    /* DI/EI 0x7C */
    for (int i=0;i<8;i++){
        uint16_t mask=0;
        if ((i&0x02)==0) mask|=FCW_VIE;
        if ((i&0x01)==0) mask|=FCW_NVIE;
        CtrlWord *c=cw();
        if (i&0x04){c->name="EI";c->exec=ClsFcwOp;c->fcw_op=FcwSet;c->fcw_mask=mask;}
        else{c->name="DI";c->exec=ClsFcwOp;c->fcw_op=FcwClear;c->fcw_mask=mask;}
        tblDIEI[i]=c;
    }
    set_sub(0x7C, sub_7C);
    set_sub(0x7D, sub_7D);
    { CtrlWord *c=cw();c->name="SC";c->exec=ClsSysCall;set_ctrl(0x7F,c);}

    /* bit/set/res register 0xA2-0xA7 */
    { struct{int hi;const char*nm;AluOp op;Width w;} br[]={
        {0xA2,"RESB",OpRes,WByte},{0xA3,"RES",OpRes,WWord},{0xA4,"SETB",OpSet,WByte},
        {0xA5,"SET",OpSet,WWord},{0xA6,"BITB",OpBit,WByte},{0xA7,"BIT",OpBit,WWord}};
      for(unsigned i=0;i<6;i++){CtrlWord *c=cw();c->name=br[i].nm;c->exec=ClsUnified;c->alu_op=br[i].op;c->width=br[i].w;c->src1_nib=RegNib2;c->src2=SrcNib3Bit;c->dst=(br[i].op==OpBit)?DstNone:DstReg;c->dst_nib=RegNib2;set_ctrl(br[i].hi,c);} }

    /* inc/dec register 0xA8-0xAB */
    { struct{int hi;const char*nm;AluOp op;Width w;} id[]={
        {0xA8,"INCB",OpInc,WByte},{0xA9,"INC",OpInc,WWord},{0xAA,"DECB",OpDec,WByte},{0xAB,"DEC",OpDec,WWord}};
      for(unsigned i=0;i<4;i++){CtrlWord *c=cw();c->name=id[i].nm;c->exec=ClsUnified;c->alu_op=id[i].op;c->width=id[i].w;c->src1_nib=RegNib2;c->src2=SrcNib3CntPlus1;c->dst=DstReg;c->dst_nib=RegNib2;set_ctrl(id[i].hi,c);} }
    { CtrlWord *c=cw();c->name="EXB";c->exec=ClsExchange;c->width=WByte;set_ctrl(0xAC,c);}
    { CtrlWord *c=cw();c->name="EX";c->exec=ClsExchange;c->width=WWord;set_ctrl(0xAD,c);}
    { CtrlWord *c=cw();c->name="TCCB";c->exec=ClsTCC;c->width=WByte;set_ctrl(0xAE,c);}
    { CtrlWord *c=cw();c->name="TCC";c->exec=ClsTCC;c->width=WWord;set_ctrl(0xAF,c);}

    { CtrlWord *c=cw();c->name="DAB";c->exec=ClsDAB;set_ctrl(0xB0,c);}
    set_sub(0xB1, sub_B1);
    set_sub(0xB2, sub_B2);
    set_sub(0xB3, sub_B3);
    set_ctrl(0xB4, mk_reg("ADCB", OpAdc, WByte));
    set_ctrl(0xB5, mk_reg("ADC",  OpAdc, WWord));
    set_ctrl(0xB6, mk_reg("SBCB", OpSbc, WByte));
    set_ctrl(0xB7, mk_reg("SBC",  OpSbc, WWord));
    set_sub(0xB8, sub_B8);

    { CtrlWord *c=cw();c->name="RRDB";c->exec=ClsBCDRotate;c->alu_op=OpRr;set_ctrl(0xBC,c);}
    { CtrlWord *c=cw();c->name="LDK";c->exec=ClsUnified;c->alu_op=OpLd;c->width=WWord;c->src2=SrcNib3Bit;c->dst=DstReg;c->dst_nib=RegNib2;set_ctrl(0xBD,c);}
    { CtrlWord *c=cw();c->name="RLDB";c->exec=ClsBCDRotate;c->alu_op=OpRl;set_ctrl(0xBE,c);}

    /* LDB Rd,#imm8 0xC0-0xCF */
    { CtrlWord *c=cw();c->name="LDB";c->exec=ClsUnified;c->alu_op=OpLd;c->width=WByte;c->src2=SrcOpLoByte;c->dst=DstReg;c->dst_nib=RegNib1;
      for(int i=0xC0;i<=0xCF;i++) set_ctrl(i,c); }
    /* CALR dsp12 0xD0-0xDF */
    { CtrlWord *c=cw();c->name="CALR";c->exec=ClsBranch;c->br_mode=BranchCALR12;
      for(int i=0xD0;i<=0xDF;i++) set_ctrl(i,c); }
    /* JR 0xE0-0xEF */
    { CtrlWord *c=cw();c->name="JR";c->exec=ClsBranch;c->br_mode=BranchJR;
      for(int i=0xE0;i<=0xEF;i++) set_ctrl(i,c); }
    /* DJNZ 0xF0-0xFF */
    { CtrlWord *c=cw();c->name="DJNZ";c->exec=ClsBranch;c->br_mode=BranchDJNZ;
      for(int i=0xF0;i<=0xFF;i++) set_ctrl(i,c); }

    /* ── 0x8C table ── */
    { CtrlWord *c;
      c=cw();c->name="COMB";c->exec=ClsUnified;c->alu_op=OpCom;c->width=WByte;c->src1_nib=RegNib2;c->dst=DstReg;c->dst_nib=RegNib2;tbl8C[0x00]=c;
      c=cw();c->name="NEGB";c->exec=ClsUnified;c->alu_op=OpNeg;c->width=WByte;c->src1_nib=RegNib2;c->dst=DstReg;c->dst_nib=RegNib2;tbl8C[0x02]=c;
      c=cw();c->name="TESTB";c->exec=ClsUnified;c->alu_op=OpTest;c->width=WByte;c->src1_nib=RegNib2;c->dst=DstNone;c->dst_nib=RegNib2;tbl8C[0x04]=c;
      c=cw();c->name="TSETB";c->exec=ClsUnified;c->alu_op=OpTset;c->width=WByte;c->src1_nib=RegNib2;c->src2=SrcNone;c->dst=DstReg;c->dst_nib=RegNib2;tbl8C[0x06]=c;
      c=cw();c->name="CLRB";c->exec=ClsUnified;c->alu_op=OpLd;c->width=WByte;c->src2=SrcZero;c->dst=DstReg;c->dst_nib=RegNib2;tbl8C[0x08]=c;
      c=cw();c->name="LDCTLB";c->exec=ClsFcwOp;c->fcw_op=FcwStoreLo;tbl8C[0x01]=c;
      c=cw();c->name="LDCTLB";c->exec=ClsFcwOp;c->fcw_op=FcwLoadLo;tbl8C[0x09]=c;
    }
    /* ── 0x8D table ── */
    { CtrlWord *com,*neg,*test,*tset,*nop,*clr;
      com=cw();com->name="COM";com->exec=ClsUnified;com->alu_op=OpCom;com->width=WWord;com->src1_nib=RegNib2;com->dst=DstReg;com->dst_nib=RegNib2;
      neg=cw();neg->name="NEG";neg->exec=ClsUnified;neg->alu_op=OpNeg;neg->width=WWord;neg->src1_nib=RegNib2;neg->dst=DstReg;neg->dst_nib=RegNib2;
      test=cw();test->name="TEST";test->exec=ClsUnified;test->alu_op=OpTest;test->width=WWord;test->src1_nib=RegNib2;test->dst=DstNone;test->dst_nib=RegNib2;
      tset=cw();tset->name="TSET";tset->exec=ClsUnified;tset->alu_op=OpTset;tset->width=WWord;tset->src1_nib=RegNib2;tset->src2=SrcNone;tset->dst=DstReg;tset->dst_nib=RegNib2;
      nop=cw();nop->name="NOP";nop->exec=ClsUnified;nop->alu_op=OpLd;nop->width=WWord;nop->src2=SrcZero;nop->dst=DstNone;
      clr=cw();clr->name="CLR";clr->exec=ClsUnified;clr->alu_op=OpLd;clr->width=WWord;clr->src2=SrcZero;clr->dst=DstReg;clr->dst_nib=RegNib2;
      for(int n2=0;n2<16;n2++){
        tbl8D[n2][0x00]=com;tbl8D[n2][0x02]=neg;tbl8D[n2][0x04]=test;tbl8D[n2][0x06]=tset;tbl8D[n2][0x07]=nop;tbl8D[n2][0x08]=clr;
        uint16_t mask=(uint16_t)n2<<4;
        CtrlWord *s=cw();s->name="SETFLG";s->exec=ClsFcwOp;s->fcw_op=FcwSet;s->fcw_mask=mask;tbl8D[n2][0x01]=s;
        CtrlWord *r=cw();r->name="RESFLG";r->exec=ClsFcwOp;r->fcw_op=FcwClear;r->fcw_mask=mask;tbl8D[n2][0x03]=r;
        CtrlWord *t=cw();t->name="COMFLG";t->exec=ClsFcwOp;t->fcw_op=FcwToggle;t->fcw_mask=mask;tbl8D[n2][0x05]=t;
      }
    }
    /* ── 0x0C table ── */
    { CtrlWord *c;
      c=cw();c->name="COMB";c->exec=ClsUnified;c->alu_op=OpCom;c->width=WByte;c->src2=SrcIR;c->src2_nib=RegNib2;c->dst=DstIR;c->dst_nib=RegNib2;tbl0C[0x00]=c;
      c=cw();c->name="CPB";c->exec=ClsUnified;c->alu_op=OpCp;c->width=WByte;c->src2=SrcIR;c->src2_nib=RegNib2;c->count=SrcImm;c->dst=DstFlagsOnly;c->words=2;tbl0C[0x01]=c;
      c=cw();c->name="NEGB";c->exec=ClsUnified;c->alu_op=OpNeg;c->width=WByte;c->src2=SrcIR;c->src2_nib=RegNib2;c->dst=DstIR;c->dst_nib=RegNib2;tbl0C[0x02]=c;
      c=cw();c->name="TESTB";c->exec=ClsUnified;c->alu_op=OpTest;c->width=WByte;c->src2=SrcIR;c->src2_nib=RegNib2;c->dst=DstFlagsOnly;tbl0C[0x04]=c;
      c=cw();c->name="LDB";c->exec=ClsUnified;c->alu_op=OpLd;c->width=WByte;c->src2=SrcImm;c->dst=DstIR;c->dst_nib=RegNib2;c->words=2;tbl0C[0x05]=c;
      c=cw();c->name="TSETB";c->exec=ClsUnified;c->alu_op=OpTset;c->width=WByte;c->src2=SrcIR;c->src2_nib=RegNib2;c->dst=DstIR;c->dst_nib=RegNib2;tbl0C[0x06]=c;
      c=cw();c->name="CLRB";c->exec=ClsUnified;c->alu_op=OpLd;c->width=WByte;c->src2=SrcZero;c->dst=DstIR;c->dst_nib=RegNib2;tbl0C[0x08]=c;
    }
    /* ── 0x0D table ── */
    { CtrlWord *c;
      c=cw();c->name="COM";c->exec=ClsUnified;c->alu_op=OpCom;c->width=WWord;c->src2=SrcIR;c->src2_nib=RegNib2;c->dst=DstIR;c->dst_nib=RegNib2;tbl0D[0x00]=c;
      c=cw();c->name="CP";c->exec=ClsUnified;c->alu_op=OpCp;c->width=WWord;c->src2=SrcIR;c->src2_nib=RegNib2;c->count=SrcImm;c->dst=DstFlagsOnly;c->words=2;tbl0D[0x01]=c;
      c=cw();c->name="NEG";c->exec=ClsUnified;c->alu_op=OpNeg;c->width=WWord;c->src2=SrcIR;c->src2_nib=RegNib2;c->dst=DstIR;c->dst_nib=RegNib2;tbl0D[0x02]=c;
      c=cw();c->name="TEST";c->exec=ClsUnified;c->alu_op=OpTest;c->width=WWord;c->src2=SrcIR;c->src2_nib=RegNib2;c->dst=DstFlagsOnly;tbl0D[0x04]=c;
      c=cw();c->name="LD";c->exec=ClsUnified;c->alu_op=OpLd;c->width=WWord;c->src2=SrcImm;c->dst=DstIR;c->dst_nib=RegNib2;c->words=2;tbl0D[0x05]=c;
      c=cw();c->name="TSET";c->exec=ClsUnified;c->alu_op=OpTset;c->width=WWord;c->src2=SrcIR;c->src2_nib=RegNib2;c->dst=DstIR;c->dst_nib=RegNib2;tbl0D[0x06]=c;
      c=cw();c->name="CLR";c->exec=ClsUnified;c->alu_op=OpLd;c->width=WWord;c->src2=SrcZero;c->dst=DstIR;c->dst_nib=RegNib2;tbl0D[0x08]=c;
      c=cw();c->name="PUSH";c->exec=ClsStack;c->stk_mode=StackPushImm;c->words=2;tbl0D[0x09]=c;
    }
    /* ── 0x4C/0x4D tables ── */
    for (int i=0;i<2;i++){
        SrcSel s2 = i?SrcX:SrcDA; DstSel dm = i?DstX:DstDA;
        CtrlWord *c;
        c=cw();c->name="COMB";c->exec=ClsUnified;c->alu_op=OpCom;c->width=WByte;c->src2=s2;c->src2_nib=RegNib2;c->dst=dm;c->dst_nib=RegNib2;c->words=2;tbl4C[i][0x00]=c;
        c=cw();c->name="CPB";c->exec=ClsUnified;c->alu_op=OpCp;c->width=WByte;c->src2=s2;c->src2_nib=RegNib2;c->count=SrcImm2;c->dst=DstFlagsOnly;c->words=2;tbl4C[i][0x01]=c;
        c=cw();c->name="NEGB";c->exec=ClsUnified;c->alu_op=OpNeg;c->width=WByte;c->src2=s2;c->src2_nib=RegNib2;c->dst=dm;c->dst_nib=RegNib2;c->words=2;tbl4C[i][0x02]=c;
        c=cw();c->name="TESTB";c->exec=ClsUnified;c->alu_op=OpTest;c->width=WByte;c->src2=s2;c->src2_nib=RegNib2;c->dst=DstFlagsOnly;c->words=2;tbl4C[i][0x04]=c;
        c=cw();c->name="LDB";c->exec=ClsUnified;c->alu_op=OpLd;c->width=WByte;c->src2=SrcImm2;c->dst=dm;c->dst_nib=RegNib2;c->words=2;tbl4C[i][0x05]=c;
        c=cw();c->name="TSETB";c->exec=ClsUnified;c->alu_op=OpTset;c->width=WByte;c->src2=s2;c->src2_nib=RegNib2;c->dst=dm;c->dst_nib=RegNib2;c->words=2;tbl4C[i][0x06]=c;
        c=cw();c->name="CLRB";c->exec=ClsUnified;c->alu_op=OpLd;c->width=WByte;c->src2=SrcZero;c->dst=dm;c->dst_nib=RegNib2;c->words=2;tbl4C[i][0x08]=c;

        c=cw();c->name="COM";c->exec=ClsUnified;c->alu_op=OpCom;c->width=WWord;c->src2=s2;c->src2_nib=RegNib2;c->dst=dm;c->dst_nib=RegNib2;c->words=2;tbl4D[i][0x00]=c;
        c=cw();c->name="CP";c->exec=ClsUnified;c->alu_op=OpCp;c->width=WWord;c->src2=s2;c->src2_nib=RegNib2;c->count=SrcImm2;c->dst=DstFlagsOnly;c->words=2;tbl4D[i][0x01]=c;
        c=cw();c->name="NEG";c->exec=ClsUnified;c->alu_op=OpNeg;c->width=WWord;c->src2=s2;c->src2_nib=RegNib2;c->dst=dm;c->dst_nib=RegNib2;c->words=2;tbl4D[i][0x02]=c;
        c=cw();c->name="TEST";c->exec=ClsUnified;c->alu_op=OpTest;c->width=WWord;c->src2=s2;c->src2_nib=RegNib2;c->dst=DstFlagsOnly;c->words=2;tbl4D[i][0x04]=c;
        c=cw();c->name="LD";c->exec=ClsUnified;c->alu_op=OpLd;c->width=WWord;c->src2=SrcImm2;c->dst=dm;c->dst_nib=RegNib2;c->words=2;tbl4D[i][0x05]=c;
        c=cw();c->name="TSET";c->exec=ClsUnified;c->alu_op=OpTset;c->width=WWord;c->src2=s2;c->src2_nib=RegNib2;c->dst=dm;c->dst_nib=RegNib2;c->words=2;tbl4D[i][0x06]=c;
        c=cw();c->name="CLR";c->exec=ClsUnified;c->alu_op=OpLd;c->width=WWord;c->src2=SrcZero;c->dst=dm;c->dst_nib=RegNib2;c->words=2;tbl4D[i][0x08]=c;
    }

    /* ── 0x3A/0x3B block/single I/O ── */
    { CtrlWord *c;
      #define B(TBL,IDX,NAME,W,IOW,DEC,REP,SPEC) do{ c=cw();c->name=NAME;c->exec=ClsBlockIO;c->width=W;c->io_write=IOW;c->block_decr=DEC;c->block_rept=REP;c->io_spec=SPEC;c->words=2; }while(0)
      /* single I/O forms */
      c=cw();c->name="INB";c->exec=ClsIO;c->width=WByte;c->words=2;tbl3A[0x04][0]=tbl3A[0x04][1]=c;
      c=cw();c->name="SINB";c->exec=ClsIO;c->width=WByte;c->io_spec=true;c->words=2;tbl3A[0x05][0]=tbl3A[0x05][1]=c;
      c=cw();c->name="OUTB";c->exec=ClsIO;c->width=WByte;c->io_write=true;c->words=2;tbl3A[0x06][0]=tbl3A[0x06][1]=c;
      c=cw();c->name="SOUTB";c->exec=ClsIO;c->width=WByte;c->io_write=true;c->io_spec=true;c->words=2;tbl3A[0x07][0]=tbl3A[0x07][1]=c;
      /* block byte forms: [nonspecial][sw] */
      B(tbl3A,0x00,"INIRB",WByte,false,false,true,false);tbl3A[0x00][0]=c;B(tbl3A,0x00,"INDB",WByte,false,true,false,false);tbl3A[0x00][1]=c;
      B(tbl3A,0x01,"SINIRB",WByte,false,false,true,true);tbl3A[0x01][0]=c;B(tbl3A,0x01,"SINDB",WByte,false,true,false,true);tbl3A[0x01][1]=c;
      B(tbl3A,0x02,"OTIRB",WByte,true,false,true,false);tbl3A[0x02][0]=c;B(tbl3A,0x02,"OUTIB",WByte,true,false,false,false);tbl3A[0x02][1]=c;
      B(tbl3A,0x03,"SOTIRB",WByte,true,false,true,true);tbl3A[0x03][0]=c;B(tbl3A,0x03,"SOUTIB",WByte,true,false,false,true);tbl3A[0x03][1]=c;
      B(tbl3A,0x08,"INDRB",WByte,false,true,true,false);tbl3A[0x08][0]=c;B(tbl3A,0x08,"INIB",WByte,false,false,false,false);tbl3A[0x08][1]=c;
      B(tbl3A,0x09,"SINDRB",WByte,false,true,true,true);tbl3A[0x09][0]=c;B(tbl3A,0x09,"SINIB",WByte,false,false,false,true);tbl3A[0x09][1]=c;
      B(tbl3A,0x0A,"OTDRB",WByte,true,true,true,false);tbl3A[0x0A][0]=c;B(tbl3A,0x0A,"OUTDB",WByte,true,true,false,false);tbl3A[0x0A][1]=c;
      B(tbl3A,0x0B,"SOTDRB",WByte,true,true,true,true);tbl3A[0x0B][0]=c;B(tbl3A,0x0B,"SOUTDB",WByte,true,true,false,true);tbl3A[0x0B][1]=c;

      c=cw();c->name="IN";c->exec=ClsIO;c->width=WWord;c->words=2;tbl3B[0x04][0]=tbl3B[0x04][1]=c;
      c=cw();c->name="SIN";c->exec=ClsIO;c->width=WWord;c->io_spec=true;c->words=2;tbl3B[0x05][0]=tbl3B[0x05][1]=c;
      c=cw();c->name="OUT";c->exec=ClsIO;c->width=WWord;c->io_write=true;c->words=2;tbl3B[0x06][0]=tbl3B[0x06][1]=c;
      c=cw();c->name="SOUT";c->exec=ClsIO;c->width=WWord;c->io_write=true;c->io_spec=true;c->words=2;tbl3B[0x07][0]=tbl3B[0x07][1]=c;
      B(tbl3B,0x00,"INIR",WWord,false,false,true,false);tbl3B[0x00][0]=c;B(tbl3B,0x00,"IND",WWord,false,true,false,false);tbl3B[0x00][1]=c;
      B(tbl3B,0x01,"SINIR",WWord,false,false,true,true);tbl3B[0x01][0]=c;B(tbl3B,0x01,"SIND",WWord,false,true,false,true);tbl3B[0x01][1]=c;
      B(tbl3B,0x02,"OTIR",WWord,true,false,true,false);tbl3B[0x02][0]=c;B(tbl3B,0x02,"OUTI",WWord,true,false,false,false);tbl3B[0x02][1]=c;
      B(tbl3B,0x03,"SOTIR",WWord,true,false,true,true);tbl3B[0x03][0]=c;B(tbl3B,0x03,"SOUTI",WWord,true,false,false,true);tbl3B[0x03][1]=c;
      B(tbl3B,0x08,"INDR",WWord,false,true,true,false);tbl3B[0x08][0]=c;B(tbl3B,0x08,"INI",WWord,false,false,false,false);tbl3B[0x08][1]=c;
      B(tbl3B,0x09,"SINDR",WWord,false,true,true,true);tbl3B[0x09][0]=c;B(tbl3B,0x09,"SINI",WWord,false,false,false,true);tbl3B[0x09][1]=c;
      B(tbl3B,0x0A,"OTDR",WWord,true,true,true,false);tbl3B[0x0A][0]=c;B(tbl3B,0x0A,"OUTD",WWord,true,true,false,false);tbl3B[0x0A][1]=c;
      B(tbl3B,0x0B,"SOTDR",WWord,true,true,true,true);tbl3B[0x0B][0]=c;B(tbl3B,0x0B,"SOUTD",WWord,true,true,false,true);tbl3B[0x0B][1]=c;
      #undef B
    }

    /* ── 0xB2/0xB3 rotate/shift ── */
    { CtrlWord *c;
      c=cw();c->name="RLB";c->exec=ClsUnified;c->alu_op=OpRl;c->width=WByte;c->src1_nib=RegNib2;c->src2=SrcOne;c->dst=DstReg;c->dst_nib=RegNib2;tblB2[0x00]=c;
      c=cw();c->name="RLB";c->exec=ClsUnified;c->alu_op=OpRl;c->width=WByte;c->src1_nib=RegNib2;c->src2=SrcTwo;c->dst=DstReg;c->dst_nib=RegNib2;tblB2[0x02]=c;
      c=cw();c->name="RRB";c->exec=ClsUnified;c->alu_op=OpRr;c->width=WByte;c->src1_nib=RegNib2;c->src2=SrcOne;c->dst=DstReg;c->dst_nib=RegNib2;tblB2[0x04]=c;
      c=cw();c->name="RRB";c->exec=ClsUnified;c->alu_op=OpRr;c->width=WByte;c->src1_nib=RegNib2;c->src2=SrcTwo;c->dst=DstReg;c->dst_nib=RegNib2;tblB2[0x06]=c;
      c=cw();c->name="RLCB";c->exec=ClsUnified;c->alu_op=OpRlc;c->width=WByte;c->src1_nib=RegNib2;c->src2=SrcOne;c->dst=DstReg;c->dst_nib=RegNib2;tblB2[0x08]=c;
      c=cw();c->name="RLCB";c->exec=ClsUnified;c->alu_op=OpRlc;c->width=WByte;c->src1_nib=RegNib2;c->src2=SrcTwo;c->dst=DstReg;c->dst_nib=RegNib2;tblB2[0x0A]=c;
      c=cw();c->name="RRCB";c->exec=ClsUnified;c->alu_op=OpRrc;c->width=WByte;c->src1_nib=RegNib2;c->src2=SrcOne;c->dst=DstReg;c->dst_nib=RegNib2;tblB2[0x0C]=c;
      c=cw();c->name="RRCB";c->exec=ClsUnified;c->alu_op=OpRrc;c->width=WByte;c->src1_nib=RegNib2;c->src2=SrcTwo;c->dst=DstReg;c->dst_nib=RegNib2;tblB2[0x0E]=c;
      c=cw();c->name="SLLB";c->exec=ClsShift;c->alu_op=OpSll;c->width=WByte;c->count=SrcImm;c->words=2;tblB2[0x01]=c;
      c=cw();c->name="SDLB";c->exec=ClsShift;c->alu_op=OpSll;c->width=WByte;c->count=SrcReg;c->words=2;tblB2[0x03]=c;
      c=cw();c->name="SLAB";c->exec=ClsShift;c->alu_op=OpSla;c->width=WByte;c->count=SrcImm;c->words=2;tblB2[0x09]=c;
      c=cw();c->name="SDAB";c->exec=ClsShift;c->alu_op=OpSla;c->width=WByte;c->count=SrcReg;c->words=2;tblB2[0x0B]=c;

      c=cw();c->name="RL";c->exec=ClsUnified;c->alu_op=OpRl;c->width=WWord;c->src1_nib=RegNib2;c->src2=SrcOne;c->dst=DstReg;c->dst_nib=RegNib2;tblB3[0x00]=c;
      c=cw();c->name="RL";c->exec=ClsUnified;c->alu_op=OpRl;c->width=WWord;c->src1_nib=RegNib2;c->src2=SrcTwo;c->dst=DstReg;c->dst_nib=RegNib2;tblB3[0x02]=c;
      c=cw();c->name="RR";c->exec=ClsUnified;c->alu_op=OpRr;c->width=WWord;c->src1_nib=RegNib2;c->src2=SrcOne;c->dst=DstReg;c->dst_nib=RegNib2;tblB3[0x04]=c;
      c=cw();c->name="RR";c->exec=ClsUnified;c->alu_op=OpRr;c->width=WWord;c->src1_nib=RegNib2;c->src2=SrcTwo;c->dst=DstReg;c->dst_nib=RegNib2;tblB3[0x06]=c;
      c=cw();c->name="RLC";c->exec=ClsUnified;c->alu_op=OpRlc;c->width=WWord;c->src1_nib=RegNib2;c->src2=SrcOne;c->dst=DstReg;c->dst_nib=RegNib2;tblB3[0x08]=c;
      c=cw();c->name="RLC";c->exec=ClsUnified;c->alu_op=OpRlc;c->width=WWord;c->src1_nib=RegNib2;c->src2=SrcTwo;c->dst=DstReg;c->dst_nib=RegNib2;tblB3[0x0A]=c;
      c=cw();c->name="RRC";c->exec=ClsUnified;c->alu_op=OpRrc;c->width=WWord;c->src1_nib=RegNib2;c->src2=SrcOne;c->dst=DstReg;c->dst_nib=RegNib2;tblB3[0x0C]=c;
      c=cw();c->name="RRC";c->exec=ClsUnified;c->alu_op=OpRrc;c->width=WWord;c->src1_nib=RegNib2;c->src2=SrcTwo;c->dst=DstReg;c->dst_nib=RegNib2;tblB3[0x0E]=c;
      c=cw();c->name="SLL";c->exec=ClsShift;c->alu_op=OpSll;c->width=WWord;c->count=SrcImm;c->words=2;tblB3[0x01]=c;
      c=cw();c->name="SDL";c->exec=ClsShift;c->alu_op=OpSll;c->width=WWord;c->count=SrcReg;c->words=2;tblB3[0x03]=c;
      c=cw();c->name="SLLL";c->exec=ClsShift;c->alu_op=OpSll;c->width=WLong;c->count=SrcImm;c->words=2;tblB3[0x05]=c;
      c=cw();c->name="SDLL";c->exec=ClsShift;c->alu_op=OpSll;c->width=WLong;c->count=SrcReg;c->words=2;tblB3[0x07]=c;
      c=cw();c->name="SLA";c->exec=ClsShift;c->alu_op=OpSla;c->width=WWord;c->count=SrcImm;c->words=2;tblB3[0x09]=c;
      c=cw();c->name="SDA";c->exec=ClsShift;c->alu_op=OpSla;c->width=WWord;c->count=SrcReg;c->words=2;tblB3[0x0B]=c;
      c=cw();c->name="SLAL";c->exec=ClsShift;c->alu_op=OpSla;c->width=WLong;c->count=SrcImm;c->words=2;tblB3[0x0D]=c;
      c=cw();c->name="SDAL";c->exec=ClsShift;c->alu_op=OpSla;c->width=WLong;c->count=SrcReg;c->words=2;tblB3[0x0F]=c;
    }

    /* ── 0xBA/0xBB block compare/transfer/string ── */
    { CtrlWord *c;
      c=cw();c->name="CPI";c->exec=ClsBlockCompare;c->width=WWord;c->words=2;tblBB[0x0][0]=tblBB[0x0][1]=c;
      c=cw();c->name="LDIR";c->exec=ClsBlockTransfer;c->width=WWord;c->block_rept=true;c->words=2;tblBB[0x1][0]=c;
      c=cw();c->name="LDI";c->exec=ClsBlockTransfer;c->width=WWord;c->words=2;tblBB[0x1][1]=c;
      c=cw();c->name="CPSI";c->exec=ClsBlockString;c->width=WWord;c->words=2;tblBB[0x2][0]=tblBB[0x2][1]=c;
      c=cw();c->name="CPIR";c->exec=ClsBlockCompare;c->width=WWord;c->block_rept=true;c->words=2;tblBB[0x4][0]=tblBB[0x4][1]=c;
      c=cw();c->name="CPSIR";c->exec=ClsBlockString;c->width=WWord;c->block_rept=true;c->words=2;tblBB[0x6][0]=tblBB[0x6][1]=c;
      c=cw();c->name="CPD";c->exec=ClsBlockCompare;c->width=WWord;c->block_decr=true;c->words=2;tblBB[0x8][0]=tblBB[0x8][1]=c;
      c=cw();c->name="LDDR";c->exec=ClsBlockTransfer;c->width=WWord;c->block_decr=true;c->block_rept=true;c->words=2;tblBB[0x9][0]=c;
      c=cw();c->name="LDD";c->exec=ClsBlockTransfer;c->width=WWord;c->block_decr=true;c->words=2;tblBB[0x9][1]=c;
      c=cw();c->name="CPSD";c->exec=ClsBlockString;c->width=WWord;c->block_decr=true;c->words=2;tblBB[0xA][0]=tblBB[0xA][1]=c;
      c=cw();c->name="CPDR";c->exec=ClsBlockCompare;c->width=WWord;c->block_decr=true;c->block_rept=true;c->words=2;tblBB[0xC][0]=tblBB[0xC][1]=c;
      c=cw();c->name="CPSDR";c->exec=ClsBlockString;c->width=WWord;c->block_decr=true;c->block_rept=true;c->words=2;tblBB[0xE][0]=tblBB[0xE][1]=c;

      c=cw();c->name="CPIB";c->exec=ClsBlockCompare;c->width=WByte;c->words=2;tblBA[0x0][0]=tblBA[0x0][1]=c;
      c=cw();c->name="LDIRB";c->exec=ClsBlockTransfer;c->width=WByte;c->block_rept=true;c->words=2;tblBA[0x1][0]=c;
      c=cw();c->name="LDIB";c->exec=ClsBlockTransfer;c->width=WByte;c->words=2;tblBA[0x1][1]=c;
      c=cw();c->name="CPSIB";c->exec=ClsBlockString;c->width=WByte;c->words=2;tblBA[0x2][0]=tblBA[0x2][1]=c;
      c=cw();c->name="CPIRB";c->exec=ClsBlockCompare;c->width=WByte;c->block_rept=true;c->words=2;tblBA[0x4][0]=tblBA[0x4][1]=c;
      c=cw();c->name="CPSIRB";c->exec=ClsBlockString;c->width=WByte;c->block_rept=true;c->words=2;tblBA[0x6][0]=tblBA[0x6][1]=c;
      c=cw();c->name="CPDB";c->exec=ClsBlockCompare;c->width=WByte;c->block_decr=true;c->words=2;tblBA[0x8][0]=tblBA[0x8][1]=c;
      c=cw();c->name="LDDRB";c->exec=ClsBlockTransfer;c->width=WByte;c->block_decr=true;c->block_rept=true;c->words=2;tblBA[0x9][0]=c;
      c=cw();c->name="LDDB";c->exec=ClsBlockTransfer;c->width=WByte;c->block_decr=true;c->words=2;tblBA[0x9][1]=c;
      c=cw();c->name="CPSDB";c->exec=ClsBlockString;c->width=WByte;c->block_decr=true;c->words=2;tblBA[0xA][0]=tblBA[0xA][1]=c;
      c=cw();c->name="CPDRB";c->exec=ClsBlockCompare;c->width=WByte;c->block_decr=true;c->block_rept=true;c->words=2;tblBA[0xC][0]=tblBA[0xC][1]=c;
      c=cw();c->name="CPSDRB";c->exec=ClsBlockString;c->width=WByte;c->block_decr=true;c->block_rept=true;c->words=2;tblBA[0xE][0]=tblBA[0xE][1]=c;
    }

    /* ── 0xB8 translate ── */
    { const char *nm[8]={"TRIB","TRTIB","TRIRB","TRTIRB","TRDB","TRTDB","TRDRB","TRTDRB"};
      bool iow[8]={true,false,true,false,true,false,true,false};
      bool dec[8]={false,false,false,false,true,true,true,true};
      bool rep[8]={false,false,true,true,false,false,true,true};
      for(int i=0;i<8;i++){CtrlWord *c=cw();c->name=nm[i];c->exec=ClsTranslate;c->io_write=iow[i];c->block_decr=dec[i];c->block_rept=rep[i];c->words=2;tblB8[i]=c;} }

    /* Precompute the instruction-length table so the per-fetch length is an
     * O(1) array read instead of the linear if-else chain below. */
    for (int op = 0; op < 65536; op++) {
        g_len_tab[0][op] = (uint8_t)decode_length_compute((uint16_t)op, false);
        g_len_tab[1][op] = (uint8_t)decode_length_compute((uint16_t)op, true);
    }
}

/* Instruction length in 16-bit words for the given first word, in the given
 * segmentation mode. O(1): just indexes the table filled at init. Used by the
 * fetch stage to know how many words to prefetch. */
int decode_length(uint16_t op, bool segmented) { return g_len_tab[segmented?1:0][op]; }

/* Compute (once, at init) the instruction length from the first opcode word.
 * Dispatch on the high byte (family), then on nib2 / low-nibble where an
 * immediate or address operand
 * adds words. `aw` = words an address operand occupies: 1 non-segmented, 2 in
 * segmented mode (segment word + offset). Ported from MAME's word counts. */
static int decode_length_compute(uint16_t op, bool segmented) {
    uint16_t hi = op >> 8;
    uint16_t lo = op & 0xFF;
    uint16_t n2 = (lo >> 4) & 0x0F;
    int aw = segmented ? 2 : 1;

    if (hi <= 0x0B) return (n2 == 0) ? 2 : 1;
    if (hi == 0x0C || hi == 0x0D) {
        uint16_t l = lo & 0x0F;
        if (l==0x01||l==0x05||l==0x09) return 2;
        return 1;
    }
    if (hi == 0x0E || hi == 0x0F) return 1;
    if (hi >= 0x10 && hi <= 0x17) {
        if (n2==0 && (hi==0x10||hi==0x12||hi==0x14||hi==0x16)) return 3;
        return 1;
    }
    if (hi == 0x18) return (n2==0)?3:1;
    if (hi == 0x19) return (n2==0)?2:1;
    if (hi == 0x1A) return (n2==0)?3:1;
    if (hi == 0x1B) return (n2==0)?2:1;
    if (hi == 0x1C) { uint16_t l=lo&0x0F; return (l==0x01||l==0x09)?2:1; }
    if (hi == 0x1D || hi == 0x1E || hi == 0x1F) return 1;
    if (hi == 0x20 || hi == 0x21) return (n2==0)?2:1;
    if (hi >= 0x22 && hi <= 0x27) return (n2==0)?2:1;
    if (hi >= 0x28 && hi <= 0x2B) return 1;
    if (hi >= 0x2C && hi <= 0x2F) return 1;
    if (hi >= 0x30 && hi <= 0x37) return 2;
    if (hi == 0x38) return 1;
    if (hi == 0x39) return 1;
    if (hi == 0x3A || hi == 0x3B) return 2;
    if (hi >= 0x3C && hi <= 0x3F) return 1;
    if (hi >= 0x40 && hi <= 0x6F) {
        if ((hi==0x4C||hi==0x4D) && (((lo&0x0F)==0x01)||((lo&0x0F)==0x05)||((lo&0x0F)==0x09)))
            return 1 + aw + 1;
        if (hi==0x5C && (((lo&0x0F)==0x01)||((lo&0x0F)==0x09)))
            return 1 + aw + 1;
        return 1 + aw;
    }
    if (hi >= 0x70 && hi <= 0x77) return 1 + aw;
    if (hi == 0x78) return 1;
    if (hi == 0x79) return 1 + aw;
    if (hi == 0x7A) return 1;
    if (hi == 0x7B) return 1;
    if (hi == 0x7C) return 1;
    if (hi == 0x7D) return 1;
    if (hi == 0x7E) return 1;
    if (hi == 0x7F) return 1;
    if (hi >= 0x80 && hi <= 0x8F) return 1;
    if (hi >= 0x90 && hi <= 0x9F) return 1;
    if (hi >= 0xA0 && hi <= 0xAF) return 1;
    if (hi >= 0xB0 && hi <= 0xBF) {
        if (hi==0xB2||hi==0xB3) {
            uint16_t l=lo&0x0F;
            if (l==0x01||l==0x03||l==0x05||l==0x07||l==0x09||l==0x0B||l==0x0D||l==0x0F) return 2;
        }
        if (hi==0xB8||hi==0xBA||hi==0xBB) return 2;
        return 1;
    }
    return 1;
}

/* In segmented mode a Direct-Address operand is a two-word long address whose
 * segment word carries bit 15. These helpers tell the fetch/execute stages
 * which instruction word holds that segmented address so the extra offset
 * word is accounted for. seg_addr_w1: address is in word 1 — the DA/X and
 * base-relative families (0x40-0x6F, 0x70-0x77, 0x79), except the 0x5C LDM
 * forms (nib3 1/9) whose address is in word 2. */
static bool seg_addr_w1(uint16_t op, bool segmented) {
    if (!segmented) return false;
    uint16_t hi = op >> 8;
    if (hi == 0x5C) { uint16_t l = op & 0x0F; if (l==0x01||l==0x09) return false; }
    return (hi >= 0x40 && hi <= 0x6F) || (hi >= 0x70 && hi <= 0x77) || hi == 0x79;
}
/* seg_addr_w2: address is in word 2 — only the 0x5C LDM load/store forms,
 * where word 1 is the register/count word and word 2 begins the address. */
static bool seg_addr_w2(uint16_t op, bool segmented) {
    if (!segmented) return false;
    if ((op >> 8) != 0x5C) return false;
    uint16_t l = op & 0x0F;
    return l==0x01 || l==0x09;
}

/* Decode the instruction whose first word is op0 (with up to navail following
 * words available for sub-decoders). Returns a pointer into the static decode
 * pool (no copy), or NULL for an illegal/undefined encoding OR when a sub-
 * decoder still needs more words. The returned CtrlWord is immutable.
 * The high byte selects the rom[] slot: a direct ctrl is returned as-is,
 * otherwise the slot's sub-decoder refines the encoding. */
const CtrlWord *decode_lookup(uint16_t op0, const uint16_t *words, int navail,
                              bool segmented) {
    (void)segmented;
    RomEntry *e = &rom[op0 >> 8];
    CtrlWord *c = e->ctrl ? e->ctrl : (e->sub ? e->sub(op0, words, navail) : NULL);
    if (!c || !c->valid) return NULL;
    return c;
}

/* exposed helpers for cpu.c: report whether the segmented address operand
 * lives in instruction word 1 or word 2 (see seg_addr_w1/w2 above). */
bool decode_seg_addr_w1(uint16_t op, bool segmented) { return seg_addr_w1(op, segmented); }
bool decode_seg_addr_w2(uint16_t op, bool segmented) { return seg_addr_w2(op, segmented); }
