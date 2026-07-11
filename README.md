# Minimal Commodore 900 emulator

Instruction-level Commodore 900 emulator, based on kevind's HDL Commodore 900 core.

## How it works

It emulates only the bare minimum necessary for software cross-development on the host machine:

- Z8001 CPU + Z8010 MMU
- 1 MB RAM
- 20 MB hard drive
- One serial line, which redirects to the host console

No graphics card and no floppy emulation at present.

## Building

To build the emulator, simply type `make`. gcc is used for compilation, but since the C files are straight C99, other compilers should work too.

## Tools

BIOS dump and a hard disk image with Coherent system installed are provided so that the emulator can run without any dependencies. The software is used with permission of Robert Swartz, the copyright holder on Mark Williams software.

A Python script in the `tools/` directory can be used to examine disk images and copy files to/from them.
