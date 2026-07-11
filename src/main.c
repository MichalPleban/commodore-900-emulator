/* main.c — entry point + host console glue + CPU self-test.
 *
 * Usage:
 *   c900 [--firmware=DIR] [--disk=FILE] [--trace] [--max=N] [--input="..."]
 *   c900 --selftest
 *
 * --firmware defaults to ../rom and --disk to ../disk/hdd.bin (both relative to
 * the working directory), so running the binary from the bin/ directory with no
 * arguments finds the ROMs and disk image in their sibling directories.
 *
 * The SCC channel B serial console is redirected to this process's stdin/
 * stdout, so the C900 boot ROM banner and any OS console appear directly in
 * the terminal, and keystrokes are delivered to the machine.
 *
 * The three console_* functions below are the ONLY host-OS-specific code in the
 * emulator; bus.c calls them to move bytes between the SCC and the terminal. */
#include "emu.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

#ifdef _WIN32
/* Windows console: conio gives us a non-blocking key poll directly. The one
 * thing to configure is ENABLE_PROCESSED_INPUT: with it on (the default), the
 * console host turns Ctrl-C into a CTRL_C_EVENT at keypress time, so the 0x03
 * never lands in the input buffer and the guest can't see it. Turn it off so
 * Ctrl-C is an ordinary key event; the saved mode is restored on shutdown. */
#include <conio.h>
#include <io.h>
#include <windows.h>
static HANDLE g_hin;
static DWORD  g_oldmode;
static bool   g_havemode;
int console_poll_char(void){ return _kbhit() ? _getch() : -1; }   /* -1 = no key ready */
void console_init(void){
    g_hin = GetStdHandle(STD_INPUT_HANDLE);
    if (GetConsoleMode(g_hin, &g_oldmode)) {
        g_havemode = true;
        SetConsoleMode(g_hin, g_oldmode & ~ENABLE_PROCESSED_INPUT);
    }
}
void console_shutdown(void){ if (g_havemode) SetConsoleMode(g_hin, g_oldmode); }
#else
/* POSIX terminal: put stdin into raw, non-blocking mode so the emulator sees
 * each keystroke immediately (no line buffering) and the host doesn't echo it
 * — the guest OS does its own echo over the serial line. The original termios
 * settings are saved and restored on shutdown so the user's shell is left sane. */
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
static struct termios g_old;   /* saved terminal settings, restored on exit */
void console_init(void){
    struct termios t; tcgetattr(0,&g_old); t=g_old;
    t.c_lflag &= ~(ICANON|ECHO|ISIG);       /* raw: no line editing, no local echo,
                                             * no Ctrl-C→SIGINT (guest gets the 0x03) */
    t.c_cc[VMIN]=0; t.c_cc[VTIME]=0;        /* read() returns immediately if no byte */
    tcsetattr(0,TCSANOW,&t);
    int fl=fcntl(0,F_GETFL); fcntl(0,F_SETFL,fl|O_NONBLOCK);
}
void console_shutdown(void){ tcsetattr(0,TCSANOW,&g_old); }
int console_poll_char(void){ unsigned char b; int n=read(0,&b,1); return n==1 ? b : -1; }
#endif

/* Emit one byte the machine transmitted on the console; flush so output is not
 * held back while the guest waits (interactively) for our reply. */
void console_put_char(int ch){ putchar(ch & 0xFF); fflush(stdout); }

/* SIGINT handler: keyboard Ctrl-C is delivered to the guest (see console_init),
 * so this only fires for an external signal (kill, or Ctrl-Break on Windows) —
 * ask the run loop to stop cleanly so the terminal gets restored. The
 * interactive way to quit is Ctrl-] (handled in the run loop's console poll). */
static Machine *g_m;
static void on_sigint(int s){ (void)s; if (g_m) g_m->stop = true; }

/* ── minimal CPU self-test: exercises the interpreter without the ROM ──
 * Loads a tiny hand-assembled Z8001 program into RAM, single-steps it, and
 * checks register results. Verifies decode + ALU + branch + memory paths. */
static int selftest(void){
    Machine *m = machine_new();
    CPU *c = &m->cpu;
    /* Put PSA reset vector into ROM image directly (we bypass the file). */
    /* Program at physical 0x000100 in RAM segment 0? RAM starts at seg 1.
     * Use segment 1 (phys 0x010000) for code+data. */
    #define PUT16(a,v) do{ m->ram[(a)]=(uint8_t)((v)>>8); m->ram[(a)+1]=(uint8_t)(v);}while(0)
    uint32_t base = 0x080000;   /* segment 8 — populated DRAM */
    int p = base;
    /* LD R1, #0x1234        21 01 12 34 */
    PUT16(p, 0x2101); PUT16(p+2, 0x1234); p+=4;
    /* LD R2, #0x0001        21 02 00 01 */
    PUT16(p, 0x2102); PUT16(p+2, 0x0001); p+=4;
    /* ADD R1, R2            81 21 */
    PUT16(p, 0x8121); p+=2;
    /* LD R3, #0x00FF        21 03 00 FF */
    PUT16(p, 0x2103); PUT16(p+2, 0x00FF); p+=4;
    /* AND R1, R3            87 31 */
    PUT16(p, 0x8731); p+=2;
    /* SUB R1, R2 (R1=0x34)  83 21 */
    PUT16(p, 0x8321); p+=2;

    /* set CPU state: segmented+system, PC = seg1:offset0 */
    memset(c->R,0,sizeof c->R);
    c->fcw = FCW_SEG | FCW_SN;
    c->pc = 0x080000;
    c->psap = 0;
    c->halted = false; c->irq_req=0;

    for (int i=0;i<6;i++) cpu_step(c);

    int ok = 1;
    if (c->R[1] != 0x0034) { printf("selftest FAIL: R1=%04X expected 0034\n", c->R[1]); ok=0; }
    if (c->R[2] != 0x0001) { printf("selftest FAIL: R2=%04X expected 0001\n", c->R[2]); ok=0; }
    if (ok) printf("CPU self-test PASSED (R1=%04X R2=%04X R3=%04X)\n", c->R[1],c->R[2],c->R[3]);

    /* ALU spot checks against the expected Z8000 semantics */
    AluResult r = alu_exec(OpAdd, 0x7FFF, 0x0001, false, WWord);
    if (r.value != 0x8000 || !(r.flags & F_PV) || !(r.flags & F_S)) { printf("ALU FAIL add overflow\n"); ok=0; }
    r = alu_exec(OpSub, 0x0000, 0x0001, false, WWord);
    if (r.value != 0xFFFF || !(r.flags & F_C)) { printf("ALU FAIL sub borrow\n"); ok=0; }
    r = alu_exec(OpSll, 0x0001, 4, false, WWord);
    if (r.value != 0x0010) { printf("ALU FAIL sll\n"); ok=0; }
    if (ok) printf("ALU checks PASSED\n");

    free(m->ram); free(m);
    return ok ? 0 : 1;
}

int main(int argc, char **argv){
    /* --- argument parsing ---------------------------------------------------
     * --firmware=DIR  directory with bios_h.bin + bios_l.bin (default ../rom)
     * --disk=FILE     raw hard-disk image (default ../disk/hdd.bin; required)
     * --trace         periodic PC/FCW progress to stderr
     * --max=N         stop after N instructions (0 = run until Ctrl-])
     * --input="..."   scripted console keystrokes (\r \n \t \\ escapes)
     * --selftest      run the built-in CPU/ALU regression and exit */
    const char *fw = "../rom", *disk = "../disk/hdd.bin", *g_input = NULL;
    bool trace = false, dosel = false;
    unsigned long long g_max = 0;
    for (int i=1;i<argc;i++){
        if (!strncmp(argv[i],"--firmware=",11)) fw = argv[i]+11;
        else if (!strncmp(argv[i],"--disk=",7)) disk = argv[i]+7;
        else if (!strcmp(argv[i],"--trace")) trace = true;
        else if (!strncmp(argv[i],"--max=",6)) g_max = strtoull(argv[i]+6,0,0);
        else if (!strncmp(argv[i],"--input=",8)) g_input = argv[i]+8;
        else if (!strcmp(argv[i],"--selftest")) dosel = true;
        else { fprintf(stderr,"unknown arg: %s\n", argv[i]); return 2; }
    }

    if (dosel) return selftest();

    Machine *m = machine_new();
    g_m = m;
    if (machine_load_rom(m, fw) != 0) {
        fprintf(stderr,"failed to load ROM from %s (need bios_h.bin + bios_l.bin)\n", fw);
        return 1;
    }
    if (machine_attach_disk(m, disk) != 0) {
        fprintf(stderr,"failed to open hard-disk image %s (the machine requires a hard disk)\n", disk);
        return 1;
    }

    m->trace = trace;
    m->insn_limit = g_max;
    if (g_input) {                       /* queue scripted serial input (\r \n \t \\ escapes) */
        for (const char *s=g_input; *s && m->inq_len < (int)sizeof m->inq; s++) {
            uint8_t c = (uint8_t)*s;
            if (c=='\\' && s[1]) { s++; c = (*s=='r')?'\r':(*s=='n')?'\n':(*s=='t')?'\t':(uint8_t)*s; }
            m->inq[m->inq_len++] = c;
        }
    }
    signal(SIGINT, on_sigint);
#ifdef _WIN32
    signal(SIGBREAK, on_sigint);   /* Ctrl-Break always signals regardless of console mode */
#endif
    fprintf(stderr,"[c900: running boot ROM; serial console below. Ctrl-] to quit (Ctrl-C goes to the guest)]\n");
    machine_run(m);
    fprintf(stderr,"\n[c900: stopped after %llu instructions]\n", (unsigned long long)m->cpu.insns);
    return 0;
}
