/* mmu.c — Zilog Z8010 MMU (functional subset).
 * Command access via special I/O (cmd = high byte of port); segmented address
 * translation with attribute + limit checks. */
#include "emu.h"
#include <stdlib.h>

/* Violation Type Register (VTR) bit layout, per z8010_mmu.pdf Fig.8 p.564.
 * Latched into u->viol_type when a segment trap fires; the OS reads it via
 * command 0x02 to classify the fault.
 *   FATL  — Fatal: a further violation arrived before VTR was reset.
 *   SWW   — Secondary Write Warning (system-mode stack push during trap).
 *   PWW   — Primary Write Warning (write to boundary page of a stack seg).
 *   EXCV  — Execute-Only Violation (data access to execute-only segment).
 *   CPUIV — CPU-Inhibit Violation (CPU access to a DMA-only segment).
 *   SLV   — Segment Length Violation (offset outside the descriptor limit).
 *   SYSV  — System Violation (normal-mode access to a system-only segment).
 *   RDV   — Read-Only Violation (write to a read-only segment). */
#define VT_FATL 0x80
#define VT_SWW  0x40
#define VT_PWW  0x20
#define VT_EXCV 0x10
#define VT_CPUIV 0x08
#define VT_SLV  0x04
#define VT_SYSV 0x02
#define VT_RDV  0x01

/* Restrictive attribute bits that force the slow translation path. Any of
 * these means an access could trap or needs a limit check, so the descriptor
 * cannot use the check-free fast path (see mmu_refresh_fast). */
#define A_RESTRICT (A_CPU|A_RD|A_SYS|A_EXC|A_DIRW)

/* Recompute the fast-path cache for one descriptor. A descriptor is "simple"
 * (translatable without any protection/limit checks) when it has the full
 * 0xFF page limit and none of the restrictive attribute bits set. In that
 * case every 16-bit offset is in range (limit==0xFF admits pages 0..255) and
 * no attribute can trap, so translation reduces to (base<<8)+offset. We
 * precompute fast_base=(base<<8) here and set fast_ok so mmu_translate can
 * skip the whole protection ladder. Called after any write that could change
 * a descriptor's base/limit/attr (and at reset). Side effect: writes
 * u->fast_ok[seg] and u->fast_base[seg]. */
static void mmu_refresh_fast(MMU *u, uint8_t seg){
    seg &= 0x3F;
    MmuDesc *d = &u->desc[seg];
    u->fast_ok[seg]   = (d->limit == 0xFF) && ((d->attr & A_RESTRICT) == 0);
    u->fast_base[seg] = (uint32_t)d->base << 8;
}

/* Clear all MMU state to the power-on condition. Zeroes the 64-entry
 * descriptor table, all control/violation registers, and rebuilds the
 * fast-path cache for every (now-empty) descriptor. */
void mmu_reset(MMU *u){
    for (int i=0;i<64;i++){ u->desc[i].base=0; u->desc[i].limit=0; u->desc[i].attr=0; }
    /* CS asserted at reset on the C900 → pass-through enabled (MSEN, TRNS=0).
     * Datasheet p.570: CS low at reset sets the Master Enable flag; the MMU
     * therefore drives addresses (MSEN=1) but does not translate (TRNS=0)
     * until firmware programs the descriptors and sets TRNS. */
    u->mode = MODE_MSEN;
    u->sar=0; u->dsc=0;
    u->viol_type=0; u->viol_sn=0; u->viol_off=0; u->bcsr=0; u->isn=0; u->ioff=0;
    u->viol_captured=false; u->segt=false; u->warn_pending=false;
    for (int i=0;i<64;i++) mmu_refresh_fast(u,(uint8_t)i);
}

/* Record a segment-trap violation or write warning and assert SEGT.
 *
 * On the FIRST violation (viol_captured false) the full context is latched:
 * the violation type, the offending segment number and offset, and BCSR =
 * status code (low 4 bits) | write-flag (0x10) | normal-mode-flag (0x20).
 * These registers stay frozen so the OS trap handler can read the exact
 * faulting access. A SUBSEQUENT violation before the OS resets VTR only ORs
 * its type bit into viol_type (context stays pinned to the first). See
 * z8010_mmu.pdf §10.
 *
 * suppress=true for hard violations (access inhibited, instruction re-run),
 * false for write warnings (PWW: the write PROCEEDS; SEGT only informs the
 * OS, which Coherent uses to grow the user stack). FATL/SWW per datasheet
 * p.564 R-MMU-124..128:
 *   FATL — a further violation (or a Normal-mode warning) arrives while VTR
 *          is still latched;
 *   SWW  — a System-mode write warning while PWW is already set (stack push
 *          during trap processing).
 * Either flag inhibits new trap requests until the OS clears it.
 *
 * Inputs: viol=VT_* bit, sn/off=faulting logical address, is_write/is_normal
 * describe the cycle, st=bus status code. Side effects: writes viol_type,
 * viol_sn, viol_off, bcsr, viol_captured, segt, and warn_pending. */
static void latch_viol(MMU *u, uint8_t viol, uint8_t sn, uint16_t off,
                       bool is_write, bool is_normal, uint8_t st, bool suppress){
    bool is_warn = !suppress;
    if (getenv("CSIM_MMU_DEBUG")) {
        MmuDesc *dd = &u->desc[sn & 0x3F];
        fprintf(stderr, "[mmu] viol=%02X sn=%02X off=%04X w=%d n=%d st=%X sup=%d "
                        "desc(base=%04X lim=%02X attr=%02X) vtr=%02X cap=%d\n",
                viol, sn, off, is_write, is_normal, st, suppress,
                dd->base, dd->limit, dd->attr, u->viol_type, u->viol_captured);
    }
    bool fatal = u->viol_captured && (suppress || (is_warn && is_normal));
    bool sww   = is_warn && !is_normal && (u->viol_type & VT_PWW) && !fatal;
    if (!u->viol_captured) {
        u->viol_type=viol; u->viol_sn=sn; u->viol_off=off;
        u->bcsr=(st&0x0F)|(is_write?0x10:0)|(is_normal?0x20:0);
        u->viol_captured=true;
    } else {
        u->viol_type |= viol;
    }
    if (fatal) u->viol_type |= VT_FATL;
    if (sww)   u->viol_type |= VT_SWW;
    /* Once FATL or SWW is set, no further trap requests are generated
     * (R-MMU-128/148); suppression of hard violations continues regardless. */
    if ((u->viol_type & (VT_FATL|VT_SWW)) == 0) {
        u->segt = true;
        if (is_warn) u->warn_pending = true;
    }
}

/* Translate a logical address (segment number + 16-bit offset) to a 24-bit
 * physical address, applying the descriptor's protection/limit checks.
 *
 * Inputs: sn = 7-bit segment number (bit 6 selects the URS half), off = 16-bit
 * logical offset, is_write/is_normal/is_fetch describe the bus cycle.
 * Output: the 24-bit physical address (masked to 0x00FFFFFF). On a hard
 * violation *fault is set true and the untranslated identity address is
 * returned; on success *fault stays false. Side effect: sets A_REF (and A_CHG
 * on writes) in the descriptor, and on a violation latches trap state via
 * latch_viol().
 *
 * Physical = (Base<<8) + Offset: Base is the 256-byte-aligned page frame, the
 * full 16-bit offset is added, giving a 24-bit (16 MB) space. */
uint32_t mmu_translate(MMU *u, uint8_t sn, uint16_t off,
                       bool is_write, bool is_normal, bool is_fetch, bool *fault){
    *fault=false;
    /* Untranslated "identity" address, returned when the MMU does not act on
     * this cycle (disabled / pass-through / other half) or on a fault. */
    uint32_t identity = ((uint32_t)(sn & 0x7F) << 16) | off;
    if (!(u->mode & MODE_MSEN)) return identity;          /* MMU invisible */
    if (!(u->mode & MODE_TRNS)) return identity;          /* pass-through */
    /* URS half select: only translate segments whose SN bit 6 matches the
     * mode-register URS flag; the other half is handled by the paired MMU. */
    uint8_t sn6 = (sn >> 6) & 1;
    uint8_t urs = (u->mode >> 5) & 1;
    if (sn6 != urs) return identity;                       /* not our half */

    uint8_t seg = sn & 0x3F;                                /* descriptor index 0..63 */
    /* Fast path: simple descriptor → no protection/limit checks needed.
     * fast_ok is only set when limit==0xFF (every page in range) and no
     * restrictive attribute bits are present, so the checks below would all
     * pass anyway. We still set the REF/CHG accounting bits, then return the
     * cached (base<<8)+offset. */
    if (u->fast_ok[seg]) {
        MmuDesc *d = &u->desc[seg];
        d->attr |= A_REF;
        if (is_write) d->attr |= A_CHG;
        return (u->fast_base[seg] + off) & 0x00FFFFFF;
    }

    /* Slow path: descriptor has a real limit and/or protection bits. */
    MmuDesc *d = &u->desc[seg];
    uint8_t page = (uint8_t)(off >> 8);                     /* offset page = offset[15:8] (256B granularity) */
    /* Bus status code recorded in BCSR: 0x0D fetch, 0x0A data write, 0x09 data read. */
    uint8_t st = is_fetch ? 0x0D : (is_write ? 0x0A : 0x09);

    /* Protection ladder — first failing test wins and returns the fault.
     * Order matches the datasheet's precedence. */
    /* CPU-inhibit (A_CPU): segment is DMA-only; any CPU access traps. */
    if (d->attr & A_CPU) { latch_viol(u,VT_CPUIV,sn,off,is_write,is_normal,st,true); *fault=true; return identity; }
    /* Segment-length check. A_DIRW marks a descending (stack) segment whose
     * valid pages run limit..255, so page<limit is out of range; a normal
     * ascending segment has valid pages 0..limit, so page>limit is illegal. */
    if (d->attr & A_DIRW) { if (page < d->limit) { latch_viol(u,VT_SLV,sn,off,is_write,is_normal,st,true); *fault=true; return identity; } }
    else                  { if (page > d->limit) { latch_viol(u,VT_SLV,sn,off,is_write,is_normal,st,true); *fault=true; return identity; } }
    /* Read-only (A_RD): writes trap. */
    if ((d->attr & A_RD) && is_write) { latch_viol(u,VT_RDV,sn,off,is_write,is_normal,st,true); *fault=true; return identity; }
    /* System-only (A_SYS): normal-mode accesses trap. */
    if ((d->attr & A_SYS) && is_normal) { latch_viol(u,VT_SYSV,sn,off,is_write,is_normal,st,true); *fault=true; return identity; }
    /* Execute-only (A_EXC): any non-fetch (data) access traps. */
    if ((d->attr & A_EXC) && !is_fetch) { latch_viol(u,VT_EXCV,sn,off,is_write,is_normal,st,true); *fault=true; return identity; }

    /* Access permitted: update REF/CHG accounting and translate. */
    d->attr |= A_REF;
    if (is_write) d->attr |= A_CHG;
    /* Primary Write Warning: a write to the boundary (lowest valid) page of a
     * descending stack segment succeeds but raises SEGT so the OS can extend
     * the segment before it actually overflows ("each write to the last 256
     * bytes of a segment is checked", z8010_mmu.pdf p.562). Coherent grows the
     * user stack from this trap — without it a
     * deep-stacked process runs into the hard SLV above and is killed. */
    if ((d->attr & A_DIRW) && is_write && page == d->limit)
        latch_viol(u,VT_PWW,sn,off,is_write,is_normal,st,false);
    return (((uint32_t)d->base << 8) + off) & 0x00FFFFFF;
}

/* ── command interface ── */
/* The command port is reached through special-I/O cycles with the command
 * code in the high byte of the port address. The descriptor currently
 * addressed by SAR is accessed one byte at a time; `which` selects the byte:
 *   0 = base high, 1 = base low, 2 = limit, 3 = attribute.
 * These two helpers read/write a single descriptor byte for the SAR-selected
 * descriptor; the DSC counter (managed by the callers) supplies `which`. */
static uint16_t read_desc_byte(MMU *u, int which){ /* which: 0 baseH,1 baseL,2 limit,3 attr */
    MmuDesc *d=&u->desc[u->sar&0x3F];
    switch (which){ case 0: return (d->base>>8)&0xFF; case 1: return d->base&0xFF; case 2: return d->limit; default: return d->attr; }
}
static void write_desc_byte(MMU *u, int which, uint8_t v){
    MmuDesc *d=&u->desc[u->sar&0x3F];
    switch (which){ case 0: d->base=(d->base&0x00FF)|((uint16_t)v<<8); break;
                    case 1: d->base=(d->base&0xFF00)|v; break;
                    case 2: d->limit=v; break; default: d->attr=v; }
}

/* Execute a command-port READ for command code `cmd`, returning the data the
 * MMU drives back. Command map (z8010_mmu.pdf p.566):
 *   0x00 Mode register, 0x01 SAR, 0x20 DSC.
 *   0x08/0x0C Base field (DSC bit0 picks high/low byte); 0C also inc SAR.
 *   0x09/0x0D Limit field;                              0D also inc SAR.
 *   0x0A/0x0E Attribute field;                          0E also inc SAR.
 *   0x0B/0x0F full descriptor via DSC (baseH→baseL→limit→attr); 0F inc SAR.
 *   0x02..0x07 violation status: VTR / VSN / VOff-high / BCSR / ISN / IOff.
 * Descriptor reads auto-advance DSC/SAR identically to the writes (see the
 * inline note) so a read-back self-test walks the fields in step. */
uint16_t mmu_cmd_read(MMU *u, uint8_t cmd){
    switch (cmd){
        case 0x00: return u->mode;    /* Mode register */
        case 0x01: return u->sar;     /* Segment Address Register */
        case 0x20: return u->dsc;     /* Descriptor Selection Counter */
        /* Descriptor-field/full reads auto-advance DSC/SAR exactly like the
         * corresponding writes — the Z8010's descriptor pointer sequences on
         * every access, read or write (Z8010 datasheet). Without this a
         * "read-back descriptor" self-test sees the first byte repeated. */
        case 0x08: case 0x0C: {       /* Base field: DSC bit0 = high(0)/low(1) byte */
            uint16_t v = read_desc_byte(u, u->dsc & 1);
            /* After the low byte, DSC wraps to 0 and (0C only) SAR steps to the
             * next descriptor; after the high byte, DSC advances to the low byte. */
            if ((u->dsc & 1) == 1) { u->dsc = 0; if (cmd==0x0C) u->sar=(u->sar+1)&0x3F; }
            else u->dsc = 1;
            return v; }
        case 0x09: case 0x0D: {       /* Limit field (single byte); 0D steps SAR */
            uint16_t v = read_desc_byte(u, 2);
            if (cmd==0x0D) u->sar=(u->sar+1)&0x3F;
            return v; }
        case 0x0A: case 0x0E: {       /* Attribute field (single byte); 0E steps SAR */
            uint16_t v = read_desc_byte(u, 3);
            if (cmd==0x0E) u->sar=(u->sar+1)&0x3F;
            return v; }
        case 0x0B: case 0x0F: {       /* Full descriptor: DSC walks baseH→baseL→limit→attr */
            uint16_t v = read_desc_byte(u, u->dsc & 3);
            uint8_t nd = (u->dsc + 1) & 3;
            /* SAR steps (0F only) when DSC wraps 3→0, i.e. after the attr byte. */
            if (nd==0 && cmd==0x0F) u->sar=(u->sar+1)&0x3F;
            u->dsc = nd;
            return v; }
        /* Violation status registers (read-only). */
        case 0x02:
            if (getenv("CSIM_MMU_DEBUG"))
                fprintf(stderr, "[mmu] read VTR=%02X vsn=%02X voff=%04X bcsr=%02X\n",
                        u->viol_type, u->viol_sn, u->viol_off, u->bcsr);
            return u->viol_type;        /* Violation Type Register */
        case 0x03: return u->viol_sn;          /* Violation Segment Number */
        case 0x04: return (u->viol_off>>8)&0xFF;/* Violation Offset, high byte */
        case 0x05: return u->bcsr;             /* Bus Cycle Status Register */
        case 0x06: return u->isn;              /* Instruction Segment Number */
        case 0x07: return u->ioff;             /* Instruction Offset, high byte */
    }
    return 0;
}

/* Execute a command-port WRITE for command code `cmd` with byte `data`.
 * The descriptor-field commands mirror the read side's DSC/SAR auto-advance,
 * and every descriptor write refreshes that descriptor's fast-path cache
 * (via `tgt`, captured BEFORE any SAR increment so the cache is rebuilt for
 * the descriptor that was actually written, not its successor). */
void mmu_cmd_write(MMU *u, uint8_t cmd, uint8_t data){
    switch (cmd){
        case 0x00: u->mode=data; break;              /* set Mode register */
        /* Loading SAR re-anchors the field pointer: DSC is reset to 0 so a
         * following Base/Limit/Attr command starts at that field's first byte
         * (datasheet p.566), rather than continuing a residual DSC position. */
        case 0x01: u->sar=data&0x3F; u->dsc=0; break;/* select descriptor + reset DSC */
        case 0x20: u->dsc=data&0x03; break;          /* set DSC (2-bit byte pointer) */
        case 0x08: case 0x0C: {                       /* write Base field (DSC bit0 = high/low) */
            bool inc = (cmd==0x0C); uint8_t tgt = u->sar & 0x3F;
            write_desc_byte(u, u->dsc&1, data);
            if ((u->dsc&1)==1){ u->dsc=0; if(inc) u->sar=(u->sar+1)&0x3F; }
            else u->dsc=1;
            mmu_refresh_fast(u,tgt);
            break; }
        case 0x09: case 0x0D: {                       /* write Limit field; 0D steps SAR */
            uint8_t tgt = u->sar & 0x3F;
            write_desc_byte(u,2,data);
            if (cmd==0x0D) u->sar=(u->sar+1)&0x3F;
            mmu_refresh_fast(u,tgt);
            break; }
        case 0x0A: case 0x0E: {                       /* write Attribute field; 0E steps SAR */
            uint8_t tgt = u->sar & 0x3F;
            write_desc_byte(u,3,data);
            if (cmd==0x0E) u->sar=(u->sar+1)&0x3F;
            mmu_refresh_fast(u,tgt);
            break; }
        case 0x0B: case 0x0F: {                       /* write full descriptor via DSC; 0F steps SAR on wrap */
            bool inc=(cmd==0x0F); uint8_t tgt = u->sar & 0x3F;
            write_desc_byte(u, u->dsc&3, data);
            uint8_t nd=(u->dsc+1)&3;
            if (nd==0 && inc) u->sar=(u->sar+1)&0x3F;
            u->dsc=nd;
            mmu_refresh_fast(u,tgt);
            break; }
        /* Violation-register reset commands (write-only). */
        case 0x11: u->viol_type=0; u->viol_captured=false; u->segt=false; break; /* reset VTR + re-arm capture */
        case 0x13: u->viol_type&=~VT_SWW; break;      /* clear Secondary Write Warning */
        case 0x14: u->viol_type&=~VT_FATL; break;     /* clear Fatal flag */
        /* Bulk attribute commands: OR a bit into every descriptor and refresh
         * each cache. Setting A_CPU/A_DMAI makes segments inaccessible until
         * reprogrammed (used to trap on unmapped access). */
        case 0x15: for(int i=0;i<64;i++){ u->desc[i].attr|=A_CPU; mmu_refresh_fast(u,(uint8_t)i);} break;  /* set all CPU-inhibit */
        case 0x16: for(int i=0;i<64;i++){ u->desc[i].attr|=A_DMAI; mmu_refresh_fast(u,(uint8_t)i);} break;  /* set all DMA-inhibit */
    }
}
