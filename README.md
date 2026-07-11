# Minimal Commodore 900 emulator

Instruction-level Commodore 900 emulator, based on kevind's HDL Commodore 900 core.

## How it works

It emulates only the bare minimum necessary for software cross-development on the host machine:

- Z8001 CPU + Z8010 MMU
- 1 MB RAM
- 20 MB hard drive
- One serial line, which redirects to the host console

No graphics card and no floppy emulation at present.

## Running

Run the emulator from the `bin/` directory, since the defaults are resolved relative to the working directory:

```sh
cd bin
./c900
```

The command line accepts the following arguments:

- `--firmware=DIR` — directory holding the BIOS ROMs `bios_h.bin` and `bios_l.bin` (default `../rom`).
- `--disk=FILE` — raw hard-disk image (default `../disk/hdd.bin`). A hard disk is mandatory; the emulator exits with an error if the image can't be opened.
- `--trace` — print periodic PC/FCW progress to stderr.
- `--max=N` — stop after N instructions (0, the default, runs until Ctrl-]).
- `--input="..."` — feed scripted console keystrokes, with `\r`, `\n`, `\t`, and `\\` escapes.
- `--selftest` — run the built-in CPU/ALU regression and exit (needs neither ROM nor disk).

## Building

To build the emulator, simply type `make`. gcc is used for compilation, but since the C files are straight C99, other compilers should work too.

## Tools

BIOS dump and a hard disk image with Coherent system installed are provided so that the emulator can run without any dependencies. The software is used with permission of Robert Swartz, the copyright holder on Mark Williams software.

A Python script in the `tools/` directory can be used to examine disk images and copy files to/from them.
