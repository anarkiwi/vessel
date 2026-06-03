# Vesselmon - a C rewrite of the BASIC Vessel monitor, built with the llvm-mos
# SDK. The C64 program is checked in (vesselmon.prg / vesselmon.d64); this
# rebuilds it and runs the host unit tests.
#
#   make             build vesselmon.prg and vesselmon.d64
#   make test        build and run the host unit tests
#   make format-check / format    enforce / apply clang-format LLVM style
#
# Requires: mos-c64-clang (llvm-mos SDK) on PATH, c1541 (VICE), a host C
# compiler, and clang-format.

MOS ?= mos-c64-clang
C1541 ?= c1541
CC ?= cc
CLANG_FORMAT ?= clang-format

SRC = vesselmon.c vesselmon_core.c vesselport_c64.c
HDR = vesselmon_core.h vesselport.h
FMT = $(SRC) $(HDR) test/vesselmon_test.c test/vesselport_host.c \
      test/vesselport_host.h

all: vesselmon.prg vesselmon.d64

vesselmon.prg: $(SRC) $(HDR)
	$(MOS) -Os -o $@ $(SRC)

vesselmon.d64: vesselmon.prg
	rm -f $@
	$(C1541) -format "vesselmon,vm" d64 $@ -write $< vesselmon

test/vesselmon_test: test/vesselmon_test.c vesselmon_core.c \
                     test/vesselport_host.c $(HDR) test/vesselport_host.h
	$(CC) -O2 -Wall -Wextra -I. -Itest -o $@ \
	    test/vesselmon_test.c vesselmon_core.c test/vesselport_host.c

test: test/vesselmon_test
	./test/vesselmon_test

format-check:
	$(CLANG_FORMAT) --style=LLVM --dry-run --Werror $(FMT)

format:
	$(CLANG_FORMAT) --style=LLVM -i $(FMT)

clean:
	rm -f test/vesselmon_test

.PHONY: all test format-check format clean
