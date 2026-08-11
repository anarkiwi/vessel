# Vesselmon - a C rewrite of the BASIC Vessel monitor, built with the llvm-mos
# SDK. The C64 program is checked in (vesselmon.prg / vesselmon.d64); this
# rebuilds it and runs the host unit tests.
#
#   make             build vesselmon.prg and vesselmon.d64
#   make test        build and run the host unit tests
#   make format-check / format    enforce / apply clang-format LLVM style
#
# Requires a container runtime (llvm-mos SDK, c1541 and clang-format run from
# pinned images) and a host C compiler for the unit tests. Set MOS, C1541 or
# CLANG_FORMAT to use host installs instead.

MOS_IMAGE ?= ghcr.io/anarkiwi/docker-mos-llvm-sdk:v23.0.1
VICE_IMAGE ?= anarkiwi/asid-vice:3.10.0.0
DOCKER_RUN := docker run --rm -u $(shell id -u):$(shell id -g) \
    -v $(CURDIR):/work -w /work

MOS ?= $(DOCKER_RUN) $(MOS_IMAGE) mos-c64-clang
# HOME must be writable by the calling uid or VICE logs errors creating its
# config, cache and state directories.
C1541 ?= $(DOCKER_RUN) -e HOME=/tmp --entrypoint c1541 $(VICE_IMAGE)
CC ?= cc
CLANG_FORMAT ?= $(DOCKER_RUN) $(MOS_IMAGE) clang-format

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
