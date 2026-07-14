/* bus.c — C900 memory map, I/O dispatch, and peripherals for the classic
 * emulator: ROM/RAM, Z8010 MMU (special I/O), Z8030 SCC serial console
 * (redirected to the host terminal), Z8036 CIO stubs + 100 Hz timer, the
 * PDMAC disk doorbell + WD2010 command-block processing (DMA memcpy), and
 * interrupt aggregation. */
#include "emu.h"
#include <stdlib.h>
#include <string.h>

/* ─────────────── physical memory ───────────────
 * Only the segments that actually have a responder on the C900 answer the
 * bus; everything else floats (open bus → reads 0xFF, writes ignored). This
 * matters for boot: the BIOS probes the video board's VRAM, and when no card
 * responds there it falls back to the serial console for I/O. Backing every
 * segment with RAM would make that probe falsely succeed.
 * On a machine with no graphics card only two things respond:
 *   0x00        boot ROM (read-only, 32 KB)
 *   0x08-0x17   1 MB motherboard DRAM (RAM1M)
 * Everything else has no responder — including the graphics-card address
 * space (segs 0x37/0x3A/0x3E/0x3F) and the expansion bus (0x01-0x07,
 * 0x18-0x36). Finding nothing at the video area, the BIOS uses the serial
 * port for console I/O. */
static bool seg_populated(uint32_t a){
    uint32_t seg = (a >> 16) & 0xFF;
    return seg >= 0x08 && seg <= 0x17;             /* 1 MB motherboard DRAM (segs 8-0x17) */
}
uint8_t phys_read8(Machine *m, uint32_t a){
    a &= 0x00FFFFFF;
    if (a < ROM_SIZE) return m->rom[a];
    if (seg_populated(a)) return m->ram[a];
    return 0xFF;                                    /* open bus */
}
/* Debug write-watch (CSIM_WATCH_PHYS="lo-hi" hex, physical): logs every
 * store into [lo,hi] with the writing instruction's PC. Parsed lazily on
 * first store; zero overhead when unset beyond one predictable branch. */
static uint32_t watch_lo, watch_hi; static int watch_on = -1;
static void watch_init(void){
    const char *w = getenv("CSIM_WATCH_PHYS");
    watch_on = 0;
    if (w && sscanf(w, "%x-%x", &watch_lo, &watch_hi) == 2) watch_on = 1;
}
static void watch_hit(Machine *m, uint32_t a, uint32_t v, int bits, const char *who){
    fprintf(stderr, "[watch] %s %06X <- %0*X pc=%02X:%04X op=%04X insns=%llu\n",
            who, a, bits/4, v,
            (m->cpu.instr_start>>16)&0x7F, m->cpu.instr_start&0xFFFF,
            m->cpu.opcode[0], (unsigned long long)m->cpu.insns);
}
void phys_write8(Machine *m, uint32_t a, uint8_t v){
    a &= 0x00FFFFFF;
    if (watch_on < 0) watch_init();
    if (watch_on && a >= watch_lo && a <= watch_hi) watch_hit(m, a, v, 8, "w8 ");
    if (a < ROM_SIZE) return;                       /* ROM read-only */
    if (seg_populated(a)) m->ram[a] = v;            /* else float — ignore */
}
uint16_t phys_read16(Machine *m, uint32_t a){
    a &= 0x00FFFFFF;
    if (watch_on < 0) watch_init();
    if (watch_on && a+1 >= watch_lo && a <= watch_hi)
        fprintf(stderr, "[watch] r16 %06X -> %04X pc=%02X:%04X op=%04X insns=%llu\n",
                a, (a<ROM_SIZE-1)?(((uint16_t)m->rom[a]<<8)|m->rom[a+1]):(((uint16_t)m->ram[a]<<8)|m->ram[a+1]),
                (m->cpu.instr_start>>16)&0x7F, m->cpu.instr_start&0xFFFF,
                m->cpu.opcode[0], (unsigned long long)m->cpu.insns);
    if (a < ROM_SIZE-1) return ((uint16_t)m->rom[a]<<8) | m->rom[a+1];   /* ROM word */
    uint32_t seg = (a>>16)&0xFF;
    if (seg>=0x08 && seg<=0x17) return ((uint16_t)m->ram[a]<<8) | m->ram[a+1]; /* DRAM word */
    return ((uint16_t)phys_read8(m,a)<<8) | phys_read8(m,a+1);          /* edge / open bus */
}
void phys_write16(Machine *m, uint32_t a, uint16_t v){
    a &= 0x00FFFFFF;
    if (watch_on < 0) watch_init();
    if (watch_on && a+1 >= watch_lo && a <= watch_hi) watch_hit(m, a, v, 16, "w16");
    if (a < ROM_SIZE) return;                                           /* ROM read-only */
    uint32_t seg = (a>>16)&0xFF;
    if (seg>=0x08 && seg<=0x17){ m->ram[a]=(uint8_t)(v>>8); m->ram[a+1]=(uint8_t)v; return; }
    phys_write8(m,a,(uint8_t)(v>>8)); phys_write8(m,a+1,(uint8_t)v);    /* edge / open bus */
}

/* ─────────────── SCC (Z8030) — console on channel B ─────────────── */
/* register byte read/write. reg 0 = control (WR0/RR0), reg 8 = data. */
/* Channel-B receive interrupt pending: a character is waiting and channel B has
 * Rx interrupts enabled (WR1 D4:D3 != 0) with the master interrupt enable set
 * (WR9 D3 MIE). */
static bool scc_rxb_int(Machine *m){
    SCCChan *b = &m->scc_b;
    if (!(b->wr[9] & 0x08)) return false;   /* WR9 D3 MIE */
    if (!b->rx_avail)       return false;
    return (b->wr[1] & 0x18) != 0;          /* WR1 D4:D3 Rx int mode != disabled */
}

/* Modified interrupt vector for the Ch-B Rx-available source (SCC Tech Manual
 * Table 4-7): source code 010 replaces V3:V1 (status-low, WR9 D4=0) or V6:V4
 * (status-high). */
static uint8_t scc_vector(Machine *m){
    uint8_t base = m->scc_b.wr[2];
    if (!scc_rxb_int(m)) return base;
    uint8_t src = 0x02;                      /* Ch B RX Available */
    if (m->scc_b.wr[9] & 0x10)               /* WR9 D4: Status High */
        return (uint8_t)((base & ~0x70) | (src << 4));
    return (uint8_t)((base & ~0x0E) | (src << 1));
}

/* RR3 (read on channel A) exposes the interrupt-pending bits.  D2 = Ch B Rx IP. */
static uint8_t scc_rr3(Machine *m){
    return scc_rxb_int(m) ? 0x04 : 0x00;
}

/* The C900's SCC is a Z8030 (Z-Bus).  Register addressing is a hybrid: the
 * effective register is AD4:AD1 (passed here as addrReg), UNLESS the channel's
 * register pointer (ch->ptr, set via a WR0 write) is non-zero, in which case it
 * overrides for this one access and then auto-resets.  chan=1 → Channel A
 * (AD5 high, scc_a), chan=0 → Channel B (the console, scc_b). */
static uint8_t scc_read(Machine *m, int chan, int addrReg){
    SCCChan *ch = chan ? &m->scc_a : &m->scc_b;
    int r = ch->ptr ? ch->ptr : (addrReg & 0x0F);
    ch->ptr = 0;
    switch (r) {
    case 0: {                         /* RR0: Rx avail / Tx empty / DCD / CTS */
        uint8_t v = 0x04 | 0x40 | 0x08 | 0x20;   /* Tx empty, all sent, DCD, CTS */
        if (ch->rx_avail) v |= 0x01;
        return v;
    }
    case 8:                           /* RR8 = Rx data (reading it clears rx) */
        ch->rx_avail = false;
        return ch->rx_data;
    case 1: return 0x01;              /* RR1: all-sent, no errors */
    case 2: return chan == 0 ? scc_vector(m) : ch->wr[2];  /* RR2 (Ch B): modified vector */
    case 3: return chan == 1 ? scc_rr3(m) : 0;             /* RR3 (Ch A): IP bits */
    default: return ch->wr[r & 0x0F];
    }
}
static void scc_write(Machine *m, int chan, int addrReg, uint8_t v){
    SCCChan *ch = chan ? &m->scc_a : &m->scc_b;
    int r = ch->ptr ? ch->ptr : (addrReg & 0x0F);
    ch->ptr = 0;
    if (r == 0) {                            /* WR0: command + register pointer */
        uint8_t ptr = v & 0x07;
        uint8_t cmd = (v >> 3) & 0x07;
        ch->ptr = (cmd == 1) ? (ptr | 0x08) : ptr;  /* cmd 1 = Point High (+8) */
        /* Commands 2-7 (reset ext/status, arm-rx-int, reset-Tx-IP, error-reset,
         * reset-highest-IUS) have no state we model — we fire the Rx interrupt
         * directly off rx_avail and don't track IUS. */
        return;
    }
    if (r == 8) {                            /* WR8 = Tx data */
        if (chan == 0) {
            console_put_char(v);
            m->last_tx_insn = m->cpu.insns;
            /* The Coherent shell prompt is '#'.  Latch once the power-on tests
             * are well behind us so scripted input is fed to the shell, not to
             * some earlier boot-time read; count prompts so scripted input can
             * wait for the shell to return after each command.  The power-on
             * diagnostics and kernel banner print no '#', and boot reaches the
             * shell prompt at roughly 2-3M instructions, so 1M clears the
             * diagnostics while still latching on that first real prompt. */
            if (v == '#' && m->cpu.insns > 1000000) { m->shell_up = true; m->prompt_seq++; }
        }
        return;
    }
    ch->wr[r] = v;
    /* WR2 (vector) and WR9 (master int control) are single physical registers
     * shared by both channels — keep our two structs coherent. */
    if (r == 2 || r == 9) { m->scc_a.wr[r] = v; m->scc_b.wr[r] = v; }
}

/* deliver a byte to the console channel B receiver */
void scc_rx_console(Machine *m, uint8_t b){
    m->scc_b.rx_data = b; m->scc_b.rx_avail = true;
}

/* ─────────────── HDC/FDC command-block processing (doorbell) ───────────────
 * The disk-controller card carries both the WD2010 hard-disk MCU and the
 * Commodore floppy controller. They share one PDMAC doorbell (port 0x0500)
 * and one vectored interrupt (vector 0x80), but each owns a 16-byte SASI-style
 * command block: the hard disk at hdc_cmdblk (default 0x080000) and the floppy
 * immediately after it at hdc_cmdblk+0x10. A block is "armed" when its error
 * byte (offset 0x0C) holds 0xFF; the controller overwrites it with a completion
 * status and raises the interrupt. Both media address their data by linear
 * block number, so this is a flat-file seek/copy for either one. */
static void hdc_process(Machine *m, uint32_t cb, Disk *disk, bool is_floppy){
    if (phys_read8(m, cb + 0x0C) != 0xFF) return;    /* not armed for this block */
    uint8_t opcode = phys_read8(m, cb + 0x00);

    uint8_t lunhi = phys_read8(m, cb + 0x01);
    uint8_t mid   = phys_read8(m, cb + 0x02);
    uint8_t low   = phys_read8(m, cb + 0x03);
    uint8_t bcnt  = phys_read8(m, cb + 0x04);
    uint32_t dma  = ((uint32_t)phys_read8(m, cb+0x06)<<16) |
                    ((uint32_t)phys_read8(m, cb+0x07)<<8)  |
                     (uint32_t)phys_read8(m, cb+0x08);
    uint32_t lba  = ((uint32_t)(lunhi & 0x1F)<<16) | ((uint32_t)mid<<8) | low;

    uint8_t status;
    switch (opcode){
        case 0x00: /* TestDriveReady */
        case 0x0C: /* SetDriveParams */
        case 0x01: /* Restore */
        case 0x05: /* CheckTrackFormat */
        case 0x03: /* RequestStatus */
            /* 0x70 = "no floppy in drive" (FNOSENSE); 0x92 = HD not ready. */
            status = disk->present ? 0x00 : (is_floppy ? 0x70 : 0x92);
            break;
        case 0x04: /* FormatDisk (floppy only) — no low-level format to model */
            status = disk->present ? 0x80 : 0x70;
            break;
        case 0x0F: /* ChangeCmdBlockAddr — new base in DMA fields (hard disk) */
            if (!is_floppy) m->hdc_cmdblk = dma;
            status = 0x00;
            break;
        case 0x08: /* Read */ {
            if (!disk->present)              { status = is_floppy ? 0x70 : 0x92; break; }
            if (lba + bcnt > disk->sectors)  { status = 0x92; break; }
            uint32_t n = (uint32_t)bcnt * 512;
            fseek(disk->fp, (long)lba * 512, SEEK_SET);
            /* DMA target is always in DRAM (segs 0x08-0x17) — block-copy it.
             * A partial image (trailing sectors trimmed) reads short; zero-fill
             * the remainder so unwritten blocks read as zero, not stale RAM. */
            if (((dma>>16)&0xFF) >= 0x08 && (((dma+n-1)>>16)&0xFF) <= 0x17) {
                if (watch_on < 0) watch_init();
                if (watch_on && dma <= watch_hi && dma+n > watch_lo)
                    fprintf(stderr, "[watch] DMA %06X..%06X <- disk lba=%u insns=%llu\n",
                            dma, dma+n-1, lba, (unsigned long long)m->cpu.insns);
                size_t got = fread(m->ram + dma, 1, n, disk->fp);
                if (got < n) memset(m->ram + dma + got, 0, n - got);
            } else {
                uint8_t buf[512];
                for (uint32_t s=0; s<bcnt; s++){
                    if (fread(buf,1,512,disk->fp) != 512) memset(buf,0,512);
                    for (int i=0;i<512;i++) phys_write8(m, dma + s*512 + i, buf[i]);
                }
            }
            status = 0x80;
            break; }
        case 0x0A: /* Write */ {
            if (!disk->present)              { status = is_floppy ? 0x70 : 0x92; break; }
            if (lba + bcnt > disk->sectors)  { status = 0x92; break; }
            uint32_t n = (uint32_t)bcnt * 512;
            fseek(disk->fp, (long)lba * 512, SEEK_SET);
            if (((dma>>16)&0xFF) >= 0x08 && (((dma+n-1)>>16)&0xFF) <= 0x17) {
                fwrite(m->ram + dma, 1, n, disk->fp);
            } else {
                uint8_t buf[512];
                for (uint32_t s=0; s<bcnt; s++){
                    for (int i=0;i<512;i++) buf[i] = phys_read8(m, dma + s*512 + i);
                    fwrite(buf,1,512,disk->fp);
                }
            }
            fflush(disk->fp);
            status = 0x80;
            break; }
        default:
            status = 0x81; /* bad opcode */
            break;
    }
    phys_write8(m, cb + 0x0C, status);       /* completion sentinel */
    phys_write8(m, cb + 0x0D, lunhi);
    phys_write8(m, cb + 0x0E, mid);
    /* raise the PDMAC disk-completion vectored interrupt (vector 0x80) */
    m->disk_vi = true;
}

/* Doorbell (out 0x0500,1): service whichever command block is armed. The hard
 * disk sits at hdc_cmdblk; the floppy controller's block follows at +0x10. Both
 * are always examined — the floppy controller is a fixed part of the card, so a
 * command issued with no image attached completes with a "no floppy" status
 * rather than leaving the driver waiting for an interrupt that never comes. */
static void hdc_doorbell(Machine *m){
    hdc_process(m, m->hdc_cmdblk, &m->disk, false);
    hdc_process(m, m->hdc_cmdblk + 0x10, &m->floppy, true);
}

/* ─────────────── I/O dispatch ─────────────── */
uint16_t io_read(Machine *m, uint16_t port, bool is_byte, bool special){
    (void)is_byte;
    if (special) {
        uint8_t low = port & 0xFF;
        if (low == 0xFC || low == 0xF8) {
            return mmu_cmd_read(&m->mmu, (uint8_t)(port>>8));
        }
        return 0;
    }
    /* SCC 0x0100-0x017F, on D15:D8 */
    if ((port & 0xFF80) == 0x0100) {
        int reg = (port >> 1) & 0x0F;
        int chan = (port >> 5) & 1;           /* 1→A, 0→B */
        return (uint16_t)scc_read(m, chan, reg) << 8;
    }
    /* CIO #1 0x0000-0x007F — plain register file (read back what was written).
     * In reset all regs read 0 except MICR. */
    if ((port & 0xFF80) == 0x0000) {
        int reg = (port >> 1) & 0x3F;
        if (m->cio1_reset && reg != 0) return 0;
        return (uint16_t)m->cio1[reg] << 8;
    }
    /* CIO #2 0x0080-0x00FF (inert) */
    if ((port & 0xFF80) == 0x0080) {
        int reg = (port >> 1) & 0x3F;
        if (m->cio2_reset && reg != 0) return 0;
        return (uint16_t)m->cio2[reg] << 8;
    }
    return 0;
}

void io_write(Machine *m, uint16_t port, uint16_t data, bool is_byte, bool special){
    (void)is_byte;
    if (special) {
        uint8_t low = port & 0xFF;
        if (low == 0xFC || low == 0xF8) mmu_cmd_write(&m->mmu, (uint8_t)(port>>8), (uint8_t)data);
        return;
    }
    if ((port & 0xFF80) == 0x0100) {          /* SCC (D15:D8) */
        int reg = (port >> 1) & 0x0F;
        int chan = (port >> 5) & 1;
        scc_write(m, chan, reg, (uint8_t)(data >> 8));
        return;
    }
    if ((port & 0xFF80) == 0x0000) {          /* CIO #1 */
        int reg = (port >> 1) & 0x3F;
        uint8_t v = (uint8_t)(data >> 8);
        if (reg == 0x00) {                    /* MICR: bit0=reset */
            if (!(v & 0x01)) m->cio1_reset = false;
        }
        if (reg == 0x0C) {                    /* CT3 Command & Status */
            /* Apply the command (D7:D5) to the IP/IE/IUS bits rather than
             * storing the byte raw. */
            uint8_t cmd = (v >> 5) & 0x07;
            uint8_t cs = m->cio1[0x0C];
            switch (cmd){
                case 1: cs &= ~(0x20|0x80); break;      /* clear IP + IUS */
                case 2: cs |= 0x80; break;              /* set IUS */
                case 3: cs &= ~0x80; break;             /* clear IUS */
                case 4: cs |= 0x20; break;              /* set IP */
                case 5: cs &= ~0x20; break;             /* clear IP */
                case 6: cs |= 0x40; break;              /* set IE */
                case 7: cs &= ~0x40; break;             /* clear IE */
            }
            cs = (cs & ~0x04) | (v & 0x04);             /* GCB (gate) R/W */
            if (v & 0x02) {                             /* TCB: trigger/start */
                if (!m->ct3_running || (m->cio1[0x1E] & 0x04)) { /* REB */
                    m->ct3_running = true;
                    m->ct3_accum = 0;
                }
            }
            m->cio1[0x0C] = cs;
            return;
        }
        m->cio1[reg] = v;
        return;
    }
    if ((port & 0xFF80) == 0x0080) {          /* CIO #2 */
        int reg = (port >> 1) & 0x3F;
        uint8_t v = (uint8_t)(data >> 8);
        if (reg == 0x00 && !(v & 0x01)) m->cio2_reset = false;
        m->cio2[reg] = v;
        return;
    }
    if ((port & 0xFF00) == 0x0200) return;    /* system latch (write-only, ignore) */
    if (port >= 0x0500 && port <= 0x05FF) {   /* PDMAC disk doorbell */
        if (data != 0) hdc_doorbell(m);
        return;
    }
    /* everything else: ignore */
}

/* ─────────────── machine setup / run ─────────────── */
Machine *machine_new(void){
    decode_init();
    Machine *m = calloc(1, sizeof *m);
    m->ram = calloc(PHYS_SIZE, 1);
    m->cpu.m = m;
    m->hdc_cmdblk = 0x080000;
    m->cio1_reset = true;
    m->cio2_reset = true;
    mmu_reset(&m->mmu);
    return m;
}

int machine_load_rom(Machine *m, const char *dir){
    char hp[512], lp[512];
    snprintf(hp, sizeof hp, "%s/bios_h.bin", dir);
    snprintf(lp, sizeof lp, "%s/bios_l.bin", dir);
    FILE *fh = fopen(hp, "rb"), *fl = fopen(lp, "rb");
    if (!fh || !fl) { if(fh)fclose(fh); if(fl)fclose(fl); return -1; }
    uint8_t hi[ROM_SIZE/2], lo[ROM_SIZE/2];
    size_t nh = fread(hi, 1, sizeof hi, fh);
    size_t nl = fread(lo, 1, sizeof lo, fl);
    fclose(fh); fclose(fl);
    if (nh != nl) return -2;
    for (size_t i=0;i<nh;i++){ m->rom[2*i] = hi[i]; m->rom[2*i+1] = lo[i]; }
    return 0;   /* stock ROM — the full power-on memory test runs unmodified */
}

int machine_attach_disk(Machine *m, const char *path){
    FILE *f = fopen(path, "r+b");
    if (!f) f = fopen(path, "rb");
    if (!f) return -1;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    m->disk.fp = f;
    m->disk.sectors = (uint32_t)(sz / 512);
    /* MiniScribe 20MB geometry (612×4×17) — matches the shipped images */
    m->disk.cyls = 612; m->disk.heads = 4; m->disk.spt = 17;
    m->disk.present = true;
    return 0;
}

int machine_attach_floppy(Machine *m, const char *path){
    FILE *f = fopen(path, "r+b");
    if (!f) f = fopen(path, "rb");
    if (!f) return -1;
    m->floppy.fp = f;
    /* One fixed C900 floppy format: NFBLK linearly-addressed blocks. The image
     * may be shorter (trailing zero sectors trimmed); reads past EOF zero-fill,
     * so present the full block count regardless of file length. */
    m->floppy.sectors = FLOPPY_BLOCKS;
    m->floppy.cyls = 80; m->floppy.heads = 2; m->floppy.spt = 16;
    m->floppy.present = true;
    return 0;
}

void machine_run(Machine *m){
    CPU *c = &m->cpu;
    mmu_reset(&m->mmu);
    cpu_reset(c);
    console_init();

    uint64_t idle = 0;
    for (;;) {
        if (m->stop) break;
        if (m->insn_limit && c->insns >= m->insn_limit) break;

        /* Poll host console only every 4096 instructions — _kbhit()/read()
         * is a syscall and doing it per-instruction dominates runtime. */
        if (!m->scc_b.rx_avail) {
            /* Coherent reads the console via the SCC Rx interrupt, so it does
             * NOT poll RR0 while idle — deliver the next byte once the shell
             * prompt is up and the console has been briefly TX-quiet (the shell
             * is blocked waiting for the Rx interrupt).  Feeding a byte sets
             * rx_avail, which raises the vectored Rx interrupt below.  After a
             * carriage return we wait for a fresh '#' prompt (prompt_seq to
             * advance) before feeding the next command's first byte, so a
             * still-running command can't swallow it. */
            if (m->inq_pos < m->inq_len && m->shell_up &&
                m->prompt_seq >= m->inq_wait_seq &&
                (c->insns - m->last_tx_insn) > 300000) {
                uint8_t b = m->inq[m->inq_pos++];
                scc_rx_console(m, b);
                if (b == '\r' || b == '\n') m->inq_wait_seq = m->prompt_seq + 1;
            } else if ((m->tick_counter & 0x0FFF) == 0) {
                int ch = console_poll_char();
                if (ch == 0x1D) m->stop = true;                /* Ctrl-] quits the emulator;
                                                                * Ctrl-C (0x03) passes through */
                else if (ch >= 0) scc_rx_console(m, (uint8_t)ch);   /* serial input */
            }
        }

        m->tick_counter++;
        /* CIO #1 CT3 down-counter in continuous mode = the 100 Hz system tick.
         * We approximate the PCLK/2 period with an instruction accumulator
         * scaled from the programmed time constant, so the tick rate tracks
         * what the kernel set up (TC=30000 → ~100 Hz-equivalent cadence). */
        if (m->ct3_running && (m->cio1[0x1E] & 0x80)) {           /* CSC continuous */
            uint32_t tc = ((uint32_t)m->cio1[0x1A]<<8) | m->cio1[0x1B];
            uint32_t period = tc>>1; if (period<2000) period=2000; if (period>60000) period=60000;
            if (++m->ct3_accum >= period) { m->ct3_accum = 0; m->cio1[0x0C] |= 0x20; } /* set IP */
        }

        /* Vectored-interrupt aggregation, in daisy-chain priority order:
         * CIO #1 (CT3 timer) outranks the PDMAC disk completion. */
        bool ct3_int = (m->cio1[0x0C] & 0x20) && (m->cio1[0x0C] & 0x40)   /* CT3 IP & IE */
                    && (m->cio1[0x00] & 0x80);                            /* MICR MIE */
        bool scc_int = scc_rxb_int(m);                                    /* Ch B Rx available */
        if (ct3_int) {
            uint8_t vec = m->cio1[0x04];                                  /* CTIV */
            if (m->cio1[0x00] & 0x04) vec &= ~0x07u;                      /* CTVIS: CT3 → ctNum 0 */
            c->vi_line = true; c->vi_vector = vec; m->last_vi_disk = false;
        } else if (scc_int) {
            c->vi_line = true; c->vi_vector = scc_vector(m); m->last_vi_disk = false;
        } else if (m->disk_vi) {
            c->vi_line = true; c->vi_vector = 0x80; m->last_vi_disk = true;
        } else {
            c->vi_line = false;
        }

        cpu_step(c);

        /* The disk VI is a one-shot (cleared when serviced); the CT3 IP is
         * level and is cleared by the clock ISR writing the CS register. */
        if (c->serviced == IRQ_VI && m->last_vi_disk) m->disk_vi = false;

        if (c->halted && !c->vi_line && !m->disk_vi) {
            if (++idle > 2000000) break;     /* stuck halted with no wakeup */
        } else idle = 0;
    }
    console_shutdown();
}
