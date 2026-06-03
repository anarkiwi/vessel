# Assemble the Vessel example programs with ACME.
#   make            assemble every example to a .prg
#   make clean      remove the .prg outputs

ACME ?= acme

EXAMPLES = echo send receive
PRGS = $(addsuffix .prg,$(EXAMPLES))

all: $(PRGS)

# Each .asm writes its own .prg via an `!to` directive and pulls in vessel.inc.
%.prg: %.asm vessel.inc
	$(ACME) $<

clean:
	rm -f $(PRGS)

.PHONY: all clean
