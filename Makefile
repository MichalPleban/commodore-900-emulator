# ─────────────────────────────────────────────────────────────────────────────
# C900 classic (instruction-level) emulator — portable Makefile.
#
# Pure C99, no external libraries.  Builds a single binary:
#   • Windows  → c900.exe
#   • Linux/macOS/other POSIX → c900   (no extension)
#
# Usage:
#   make            build the emulator (c900 / c900.exe)
#   make test       build, then run the built-in CPU + ALU self-test
#   make clean      remove the built binary
#
# On Windows the C toolchain must be 64-bit MinGW-w64 (e.g. MSYS2's
# mingw-w64-x86_64-gcc).  Override the compiler or flags on the command line,
# e.g.  make CC=clang   or   make CFLAGS="-O2 -g".
# ─────────────────────────────────────────────────────────────────────────────

CC      ?= gcc
# -flto lets the small MMU/memory accessors inline into the CPU hot loop, which
# is the difference between a slow and a "blazing fast" instruction-level core.
CFLAGS  ?= -O3 -flto -std=c99 -Wall
SRCDIR   = src
BINDIR   = bin
SRCS     = $(SRCDIR)/alu.c $(SRCDIR)/decode.c $(SRCDIR)/cpu.c \
           $(SRCDIR)/mmu.c $(SRCDIR)/bus.c $(SRCDIR)/main.c
HDRS     = $(SRCDIR)/emu.h

# GNU make sets the OS variable to "Windows_NT" on Windows (including under
# MSYS2/Git-Bash), which is how we pick the executable suffix.  Everything else
# (Linux, macOS, BSD) gets an empty suffix.
ifeq ($(OS),Windows_NT)
    EXE := .exe
else
    EXE :=
endif

TARGET = $(BINDIR)/c900$(EXE)

# RM is a GNU make built-in (defaults to "rm -f"), which works on Linux, macOS
# and under MSYS2/Git-Bash on Windows (make runs recipes through sh, which has
# rm). Override on the command line if your environment lacks it.
RM ?= rm -f

all: $(TARGET)

# The whole program is tiny, so we compile every translation unit in one gcc
# invocation (required anyway for -flto to see across files). The binary lands
# in bin/; the order-only prerequisite creates that directory first (mkdir -p
# is a no-op if it already exists, and runs through sh on every platform).
$(TARGET): $(SRCS) $(HDRS) | $(BINDIR)
	$(CC) $(CFLAGS) -o $@ $(SRCS)

$(BINDIR):
	mkdir -p $(BINDIR)

# Quick regression: exercises the CPU decode/execute and ALU without needing
# the boot ROM or a disk image.
test: $(TARGET)
	./$(TARGET) --selftest

clean:
	$(RM) $(TARGET)

.PHONY: all test clean
