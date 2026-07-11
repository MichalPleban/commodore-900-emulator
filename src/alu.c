/* alu.c — Z8000 ALU.
 * Combinatorial: computes result + affected-flag mask in one call. */
#include "emu.h"

/*
 * Flag bits live in the low byte of the FCW (Flags and Control Word):
 *   F_C  = 0x80  Carry
 *   F_Z  = 0x40  Zero
 *   F_S  = 0x20  Sign
 *   F_PV = 0x10  Parity (byte logical) / Overflow (arithmetic, word/long)
 *   F_DA = 0x08  Decimal-Adjust (records add-vs-subtract for DAB)
 *   F_H  = 0x04  Half-carry (carry/borrow out of bit 3; used by DAB)
 * (Defined in emu.h.)
 *
 * Every helper returns an AluResult { value, flags, mask }. Only the bits set
 * in `mask` are meaningful in `flags`; the CPU merges just those into the FCW,
 * leaving the rest untouched. A zero mask means "no flags affected".
 */

/* popcount8: count set bits in a byte; used only for the P (parity) flag. */
static int popcount8(uint8_t v) { int n = 0; while (v) { n += v & 1; v >>= 1; } return n; }

/* lflags_byte: Z/S/P flags for a byte logical result.
 *   Z if result==0, S from bit 7, P set when the number of set bits is EVEN
 *   (Z8000 parity is "even parity"). */
static uint16_t lflags_byte(uint8_t v) {
    uint16_t f = 0;
    if (v == 0) f |= F_Z;
    if (v & 0x80) f |= F_S;
    if ((popcount8(v) & 1) == 0) f |= F_PV; /* parity even */
    return f;
}
/* lflags_word: Z/S/P for a word result. Z from full 16 bits, S from bit 15,
 * and P is the parity of the LOW BYTE only (Z8000 defines word parity that
 * way). Callers that don't want P mask it out. */
static uint16_t lflags_word(uint16_t v) {
    uint16_t f = 0;
    if (v == 0) f |= F_Z;
    if (v & 0x8000) f |= F_S;
    if ((popcount8((uint8_t)v) & 1) == 0) f |= F_PV; /* parity of low byte */
    return f;
}
/* lflags_long: Z/S only for a 32-bit result. Long logical ops never affect P. */
static uint16_t lflags_long(uint32_t v) {
    uint16_t f = 0;
    if (v == 0) f |= F_Z;
    if (v & 0x80000000u) f |= F_S;
    return f;
}

/* nbits_of: operand width in bits (Byte=8, Word=16, Long=32). */
static int nbits_of(Width w) { return w == WByte ? 8 : (w == WWord ? 16 : 32); }

/* carry_chain: computes a + b + cin as an explicit bit-serial ripple-carry
 * adder (structurally mirroring the hardware), returning the sum. It also
 * exports three carries the flag logic needs:
 *   *cout   = carry out of the MSB   -> Carry flag for ADD (inverted = borrow)
 *   *c4     = carry out of bit 3      -> Half-carry (H) for byte ops / DAB
 *   *cmsbin = carry INTO the MSB      -> signed overflow = (cout != cmsbin)
 * Per bit: sum = ai^bi^c; carry-out c = majority(ai,bi,c). */
static uint64_t carry_chain(uint64_t a, uint64_t b, bool cin, int nbits,
                            bool *cout, bool *c4, bool *cmsbin) {
    uint64_t c = cin ? 1 : 0, result = 0;
    *c4 = false; *cmsbin = false;
    for (int i = 0; i < nbits; i++) {
        if (i == nbits - 1) *cmsbin = c != 0;        /* carry entering the sign bit */
        uint64_t ai = (a >> i) & 1, bi = (b >> i) & 1;
        result |= (ai ^ bi ^ c) << i;                /* sum bit */
        c = (ai & bi) | (bi & c) | (ai & c);         /* carry out = majority */
        if (i == 3) *c4 = c != 0;                    /* half-carry (bit 3 -> 4) */
    }
    *cout = c != 0;
    return result;
}

/* do_add: ADD/ADC (carry=carry-in for ADC, false for ADD). Affects C,Z,S,PV
 * for all widths; byte form additionally touches DA and H (DA cleared, H from
 * the bit-3 carry). Overflow (PV) = carry-out differs from carry-into-MSB. */
static AluResult do_add(uint64_t a, uint64_t b, bool carry, Width w) {
    int nb = nbits_of(w);
    bool cout, c4, cmsb;
    uint64_t r = carry_chain(a, b, carry, nb, &cout, &c4, &cmsb);
    uint64_t mask = (nb == 64) ? ~0ull : ((1ull << nb) - 1); /* width mask */
    r &= mask;
    uint16_t f = 0;
    if (cout) f |= F_C;                          /* Carry = carry-out of MSB */
    if (r == 0) f |= F_Z;
    if ((r >> (nb - 1)) & 1) f |= F_S;           /* Sign = MSB of result */
    if (cout != cmsb) f |= F_PV;                 /* signed Overflow */
    if (c4) f |= F_H;                            /* Half-carry from bit 3 */
    uint16_t fm = F_C | F_Z | F_S | F_PV;
    /* Byte ADD also merges DA and H: F_DA is left clear in `f`, so masking it
     * in clears DA in the FCW (ADDB clears the decimal-adjust flag). */
    if (w == WByte) fm |= F_DA | F_H;
    AluResult res = { r, f, fm };
    return res;
}

/* do_sub: SUB/SBC/CP (borrow=carry-in for SBC, false otherwise). Computed as
 * a + ~b + !borrow (two's-complement subtraction), so carry_chain does the
 * work. Because b is complemented, carry-out is INVERTED sense: no carry-out
 * means a borrow. Affects C,Z,S,PV; byte form also sets DA=1 (marks the op as
 * a subtract for DAB) and H from the bit-3 borrow. */
static AluResult do_sub(uint64_t a, uint64_t b, bool borrow, Width w) {
    int nb = nbits_of(w);
    uint64_t mask = (nb == 64) ? ~0ull : ((1ull << nb) - 1);
    uint64_t notb = (~b) & mask;                 /* ~b within the operand width */
    bool cout, c4, cmsb;
    uint64_t r = carry_chain(a, notb, !borrow, nb, &cout, &c4, &cmsb);
    r &= mask;
    uint16_t f = 0;
    if (!cout) f |= F_C;                         /* borrow = NOT carry-out */
    if (r == 0) f |= F_Z;
    if ((r >> (nb - 1)) & 1) f |= F_S;
    if (cout != cmsb) f |= F_PV;                 /* signed Overflow */
    if (!c4) f |= F_H;                           /* half-borrow = NOT bit-3 carry */
    uint16_t fm = F_C | F_Z | F_S | F_PV;
    if (w == WByte) { fm |= F_DA | F_H; f |= F_DA; } /* byte SUB sets DA=1 */
    AluResult res = { r, f, fm };
    return res;
}

/* logic: package a logical-op result. Computes Z/S/P from the width-specific
 * helper; `mask` selects which of those the caller actually wants merged
 * (word/long typically drop P). C/D/H are never touched by logical ops. */
static AluResult logic(uint64_t v, Width w, uint16_t mask) {
    AluResult r;
    r.value = v; r.mask = mask;
    if (w == WByte) r.flags = lflags_byte((uint8_t)v);
    else if (w == WWord) r.flags = lflags_word((uint16_t)v);
    else r.flags = lflags_long((uint32_t)v);
    return r;
}

/* do_neg: NEG — two's-complement negation of a. Affects C,Z,S,PV (D,H
 * unaffected). C is set when a != 0 (a borrow from the implicit 0 occurred);
 * PV (overflow) is set only for the most-negative input (0x80 / 0x8000 /
 * 0x80000000), which cannot be negated within the width. */
static AluResult do_neg(uint64_t a, Width w) {
    uint16_t fm = F_C | F_Z | F_S | F_PV;
    AluResult r; r.mask = fm; r.flags = 0;
    if (w == WByte) {
        uint8_t v = (uint8_t)(-(int8_t)(uint8_t)a);
        if ((uint8_t)a != 0) r.flags |= F_C;         /* borrow unless a==0 */
        if (v == 0) r.flags |= F_Z;
        if (v & 0x80) r.flags |= F_S;
        if ((uint8_t)a == 0x80) r.flags |= F_PV;     /* only -128 overflows */
        r.value = v;
    } else if (w == WWord) {
        uint16_t v = (uint16_t)(-(int)(uint16_t)a);
        if ((uint16_t)a != 0) r.flags |= F_C;
        if (v == 0) r.flags |= F_Z;
        if (v & 0x8000) r.flags |= F_S;
        if ((uint16_t)a == 0x8000) r.flags |= F_PV;  /* only -32768 overflows */
        r.value = v;
    } else {
        uint32_t v = (uint32_t)(-(int64_t)(uint32_t)a);
        if ((uint32_t)a != 0) r.flags |= F_C;
        if (v == 0) r.flags |= F_Z;
        if (v & 0x80000000u) r.flags |= F_S;
        if ((uint32_t)a == 0x80000000u) r.flags |= F_PV;
        r.value = v;
    }
    return r;
}

/* do_inc: INC by n (n = 1..16). Affects Z,S,PV only — NOT carry (this is what
 * distinguishes INC from ADD). Overflow detection: operands share a sign
 * ((av^nb) sign bit == 0, both non-negative here) yet the result's sign flips
 * ((av^v) sign bit set) => signed overflow. Byte and word only (no long INC). */
static AluResult do_inc(uint64_t a, uint64_t n, Width w) {
    AluResult r; r.mask = F_Z | F_S | F_PV; r.flags = 0;
    if (w == WByte) {
        uint8_t av = (uint8_t)a, nb = (uint8_t)n, v = av + nb;
        if (v == 0) r.flags |= F_Z;
        if (v & 0x80) r.flags |= F_S;
        if (((av ^ nb) & 0x80) == 0 && ((av ^ v) & 0x80)) r.flags |= F_PV; /* same-sign in, sign flipped out */
        r.value = v;
    } else {
        uint16_t av = (uint16_t)a, nb = (uint16_t)n, v = av + nb;
        if (v == 0) r.flags |= F_Z;
        if (v & 0x8000) r.flags |= F_S;
        if (((av ^ nb) & 0x8000) == 0 && ((av ^ v) & 0x8000)) r.flags |= F_PV;
        r.value = v;
    }
    return r;
}
/* do_dec: DEC by n. Affects Z,S,PV only (like INC, no carry). For subtraction
 * the overflow test is the opposite polarity: operands of DIFFERING sign
 * ((av^nb) sign set) and result sign differing from av ((av^v) sign set) =>
 * overflow. */
static AluResult do_dec(uint64_t a, uint64_t n, Width w) {
    AluResult r; r.mask = F_Z | F_S | F_PV; r.flags = 0;
    if (w == WByte) {
        uint8_t av = (uint8_t)a, nb = (uint8_t)n, v = av - nb;
        if (v == 0) r.flags |= F_Z;
        if (v & 0x80) r.flags |= F_S;
        if (((av ^ nb) & 0x80) && ((av ^ v) & 0x80)) r.flags |= F_PV; /* differing-sign in, sign flipped out */
        r.value = v;
    } else {
        uint16_t av = (uint16_t)a, nb = (uint16_t)n, v = av - nb;
        if (v == 0) r.flags |= F_Z;
        if (v & 0x8000) r.flags |= F_S;
        if (((av ^ nb) & 0x8000) && ((av ^ v) & 0x8000)) r.flags |= F_PV;
        r.value = v;
    }
    return r;
}

/* do_shift_left: SLA (arith=true) / SLL (arith=false), shifting `cnt` places.
 * The value is shifted one bit at a time so the last bit out becomes C and,
 * for SLA, we can detect if the sign bit ever changed (arithmetic overflow).
 * Flags: C/Z/S always; PV = overflow for SLA only (SLL leaves PV unaffected).
 * A zero count leaves C unaffected. */
static AluResult do_shift_left(uint64_t a, uint64_t cnt, bool arith, Width w) {
    unsigned n = (unsigned)(cnt & 0x3F);         /* clamp; covers long shift-by-32 */
    uint16_t mask = F_C | F_Z | F_S;
    if (arith) mask |= F_PV;                     /* only SLA writes PV (overflow) */
    if (n == 0) mask &= ~F_C;                    /* zero count: C preserved */
    int nb = nbits_of(w);
    uint64_t sign = 1ull << (nb - 1);
    uint64_t wmask = (nb == 64) ? ~0ull : ((1ull << nb) - 1);
    uint64_t orig = a & wmask, val = orig;
    bool cout = false, ovf = false;
    for (unsigned i = 0; i < n; i++) {
        cout = (val & sign) != 0;                /* bit shifted out of the top */
        val = (val << 1) & wmask;
        if ((val & sign) != (orig & sign)) ovf = true; /* SLA: sign changed => overflow */
    }
    AluResult r = logic(val, w, mask);
    r.flags &= ~F_PV;                            /* drop parity; PV means overflow here */
    if (cout) r.flags |= F_C;
    if (arith && ovf) r.flags |= F_PV;
    r.value = val;
    return r;
}
/* do_shift_right: SRA (arith=true, sign-replicating) / SRL (arith=false,
 * zero-fill), shifting `cnt` places. C = last bit shifted out. For SRA the
 * sign bit is preserved so arithmetic overflow is impossible: PV is in the
 * mask but always ends up 0. SRL leaves PV unaffected. Zero count leaves C
 * unaffected. */
static AluResult do_shift_right(uint64_t a, uint64_t cnt, bool arith, Width w) {
    unsigned n = (unsigned)(cnt & 0x3F);
    uint16_t mask = arith ? (F_C | F_Z | F_S | F_PV) : (F_C | F_Z | F_S);
    if (n == 0) mask &= ~F_C;
    int nb = nbits_of(w);
    uint64_t sign = 1ull << (nb - 1);
    uint64_t wmask = (nb == 64) ? ~0ull : ((1ull << nb) - 1);
    uint64_t val = a & wmask;
    bool cout = false;
    /* First loop performs the logical (zero-fill) shift; for the arithmetic
     * case the sign-extension is not handled here, so it is redone below. */
    for (unsigned i = 0; i < n; i++) {
        cout = (val & 1) != 0;
        val >>= 1;
        if (arith && (val & (sign >> 1))) { /* replicate handled below */ }
        if (arith) { if (a & sign) {} }
    }
    /* recompute arithmetic shift correctly using signed logic */
    /* SRA redo: shift one bit at a time re-injecting the original sign bit
     * (s = val & sign) into the top so the sign is replicated each step. */
    if (arith) {
        val = a & wmask;
        cout = false;
        for (unsigned i = 0; i < n; i++) {
            cout = (val & 1) != 0;
            uint64_t s = val & sign;             /* current sign bit */
            val = (val >> 1) | s;                /* shift in the sign (arithmetic) */
        }
    }
    AluResult r = logic(val & wmask, w, mask);
    if (arith) r.flags &= ~F_PV;                 /* SRA: overflow impossible -> PV=0 */
    if (cout) r.flags |= F_C;
    r.value = val & wmask;
    return r;
}

/* do_rotate: RL/RR (thru=false) and RLC/RRC (thru=true, rotate through the
 * carry bit), `right` selects direction. Rotates `cnt` places (a count of 0
 * is treated as 1, matching the Z8000 convention). C = the bit rotated out on
 * the final step; the carry-in seeds the rotate for the RLC/RRC forms.
 * PV = arithmetic overflow, i.e. the sign bit differs from the original after
 * rotating (NOT parity). C/Z/S/PV all affected. */
static AluResult do_rotate(uint64_t a, uint64_t cnt, bool right, bool thru, bool cin, Width w) {
    unsigned n = (unsigned)(cnt & 0x3F);
    if (n == 0) n = 1;                           /* count 0 acts as 1 */
    int nb = nbits_of(w);
    uint64_t sign = 1ull << (nb - 1);
    uint64_t wmask = (nb == 64) ? ~0ull : ((1ull << nb) - 1);
    uint64_t orig = a & wmask, val = orig;
    bool c = cin;
    for (unsigned i = 0; i < n; i++) {
        if (!right) {
            bool msb = (val & sign) != 0;        /* bit leaving the top */
            val = (val << 1) & wmask;
            if (thru) { if (c) val |= 1; c = msb; }   /* RLC: old carry enters LSB */
            else { if (msb) val |= 1; c = msb; }      /* RL:  old MSB wraps to LSB */
        } else {
            bool lsb = (val & 1) != 0;           /* bit leaving the bottom */
            val >>= 1;
            if (thru) { if (c) val |= sign; c = lsb; }/* RRC: old carry enters MSB */
            else { if (lsb) val |= sign; c = lsb; }   /* RR:  old LSB wraps to MSB */
        }
    }
    AluResult r = logic(val, w, F_C | F_Z | F_S | F_PV);
    r.flags &= ~F_PV;                            /* PV is overflow here, not parity */
    if (c) r.flags |= F_C;                       /* Carry = last bit rotated out */
    if ((val ^ orig) & sign) r.flags |= F_PV;    /* sign changed => overflow */
    r.value = val;
    return r;
}

/* do_dab: DAB — decimal-adjust the byte in `a` after a BCD add/subtract.
 * `hflag` is the Half-carry and `cin` the Carry from the preceding operation.
 * (The add-vs-subtract sense is carried by the DA flag upstream; this
 * implements the addition-adjust path.) Low nibble > 9 or
 * H set adds 0x06; high nibble > 9 or a resulting carry adds 0x60 and sets C.
 * Affects C/Z/S. */
static AluResult do_dab(uint64_t a, bool hflag, bool cin) {
    uint8_t val = (uint8_t)a;
    bool c = cin;
    if ((val & 0x0F) > 9 || hflag) {             /* fix low BCD digit */
        val += 0x06;
        if ((val & 0xF0) == 0) c = true;         /* wrapped past 0xF0 -> carry up */
    }
    if ((val >> 4) > 9 || c) { val += 0x60; c = true; } /* fix high BCD digit */
    AluResult r = logic(val, WByte, F_C | F_Z | F_S);
    if (c) r.flags |= F_C;
    r.value = val;
    return r;
}

/* alu_exec: top-level dispatch. `a` is destination/first operand, `b` the
 * source/second operand, `carry_in` the incoming Carry (ADC/SBC/RLC/RRC), `w`
 * the width. Routes each opcode to the helper above (or handles the trivial
 * bit/load ops inline). The returned AluResult
 * has mask==0 for operations that touch no flags (LD/SET/RES). */
AluResult alu_exec(AluOp op, uint64_t a, uint64_t b, bool carry_in, Width w) {
    AluResult r = { 0, 0, 0 };
    switch (op) {
        case OpAdd: return do_add(a, b, false, w);
        case OpAdc: return do_add(a, b, carry_in, w);      /* add with carry-in */
        case OpSub: case OpCp: return do_sub(a, b, false, w); /* CP = SUB, result discarded by caller */
        case OpSbc: return do_sub(a, b, carry_in, w);      /* subtract with borrow */
        case OpAnd: {
            uint64_t v = (w==WByte)?(uint8_t)(a&b):(w==WWord)?(uint16_t)(a&b):(uint32_t)(a&b);
            return logic(v, w, (w==WByte)?(F_Z|F_S|F_PV):(F_Z|F_S)); /* P for byte only */
        }
        case OpTest: {   /* TEST = AND of a with itself; sets flags, discards result */
            uint64_t v = (w==WByte)?(uint8_t)a:(w==WWord)?(uint16_t)a:(uint32_t)a;
            return logic(v, w, (w==WByte)?(F_Z|F_S|F_PV):(F_Z|F_S));
        }
        case OpOr: {
            uint64_t v = (w==WByte)?(uint8_t)(a|b):(w==WWord)?(uint16_t)(a|b):(uint32_t)(a|b);
            return logic(v, w, (w==WByte)?(F_Z|F_S|F_PV):(F_Z|F_S));
        }
        case OpXor: {
            uint64_t v = (w==WByte)?(uint8_t)(a^b):(w==WWord)?(uint16_t)(a^b):(uint32_t)(a^b);
            return logic(v, w, (w==WByte)?(F_Z|F_S|F_PV):(F_Z|F_S));
        }
        case OpCom: {    /* one's complement (~a); Z/S always, P for byte only */
            uint64_t v = (w==WByte)?(uint8_t)~a:(w==WWord)?(uint16_t)~a:(uint32_t)~a;
            return logic(v, w, (w==WByte)?(F_Z|F_S|F_PV):(F_Z|F_S));
        }
        case OpNeg: return do_neg(a, w);
        case OpInc: return do_inc(a, b, w);                /* b carries the count n */
        case OpDec: return do_dec(a, b, w);
        case OpSla: return do_shift_left(a, b, true, w);   /* arithmetic */
        case OpSll: return do_shift_left(a, b, false, w);  /* logical */
        case OpSra: return do_shift_right(a, b, true, w);
        case OpSrl: return do_shift_right(a, b, false, w);
        case OpRl:  return do_rotate(a, b, false, false, carry_in, w); /* left, not thru carry */
        case OpRr:  return do_rotate(a, b, true, false, carry_in, w);  /* right */
        case OpRlc: return do_rotate(a, b, false, true, carry_in, w);  /* left thru carry */
        case OpRrc: return do_rotate(a, b, true, true, carry_in, w);   /* right thru carry */
        case OpDab: return do_dab(a, b & 1, carry_in);     /* b bit0 = incoming H flag */
        case OpExts:
            /* EXTS — sign-extend: byte->word or word->long. The cast chain
             * goes through the signed narrow type first so the sign bit is
             * replicated into the wider result. Affects Z/S but mask stays 0
             * here (the caller applies its own flag policy for EXTS). */
            if (w == WByte) {
                uint16_t v = (uint16_t)(int16_t)(int8_t)(uint8_t)a; /* byte -> signed word */
                r.value = v; r.flags = 0; r.mask = 0;
                if (v == 0) r.flags |= F_Z; if (v & 0x8000) r.flags |= F_S;
            } else {
                uint32_t v = (uint32_t)(int32_t)(int16_t)(uint16_t)a; /* word -> signed long */
                r.value = v; r.flags = 0; r.mask = 0;
                if (v == 0) r.flags |= F_Z; if (v & 0x80000000u) r.flags |= F_S;
            }
            return r;
        case OpLd: r.value = b; return r;                  /* pass-through; no flags (mask 0) */
        case OpBit:
            /* BIT: test bit #b of a. Z=1 when that bit is 0; no write-back.
             * Only Z is affected. */
            r.value = a; r.mask = F_Z; r.flags = 0;
            if ((a & (1ull << b)) == 0) r.flags |= F_Z;
            return r;
        case OpSet: r.value = a | (1ull << b); return r;   /* set bit #b; no flags */
        case OpRes: r.value = a & ~(1ull << b); return r;  /* clear bit #b; no flags */
        case OpTset:
            /* TSET: S = old MSB of the operand; result is all-ones (write-back
             * as the "set" side of test-and-set). Only S is meaningful; mask
             * stays 0 because the caller special-cases merging just S. */
            r.flags = 0; r.mask = 0;
            if (w == WByte) { if ((uint8_t)a & 0x80) r.flags |= F_S; r.value = 0xFF; }
            else { if ((uint16_t)a & 0x8000) r.flags |= F_S; r.value = 0xFFFF; }
            return r;
        default: return r;
    }
}
