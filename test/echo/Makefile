# Generic 6510 assembler makefile - FTCH/HT
#
# Assumes that all files named .a, .s, i., .inc., .asm are to be considered source files

#Project specific settings
NAME = defmidi-echo
STARTADRESS = 0800
STARTFILE	= start.inc
DATEFILE	= date.inc

# Names of main files and directories
TARGETDIR = bin
SRCNAME = code.a
PRGNAME = $(NAME).prg

# Generate a comprehensive source file list
SOURCEFILES =
SOURCEFILES += $(wildcard *.a)
SOURCEFILES += $(wildcard *.asm)
SOURCEFILES += $(wildcard *.i)
SOURCEFILES += $(wildcard *.inc)
SOURCEFILES += $(wildcard *.s)
SOURCEFILES += $(wildcard *.src)


# Specific tool locations
ACME		= /usr/bin/acme
EXOMIZER	= /usr/local/bin/exomizer


#---------------------------------------------------------
# Assemble .prg file
$(TARGETDIR)/$(PRGNAME) : $(TARGETDIR) $(SRCNAME) $(SOURCEFILES) Makefile
#	@$(ASMDATE) > $(DATEFILE)							#Generate assembly time/date info
	@date "+	"'!'"pet \"%Y-%m-%d %H:%M.%S\"" > $(DATEFILE)
	@echo "	* = $$""$(STARTADRESS)" > $(STARTFILE)		#Generate start adress file
	@echo " "

	@echo " "
	@echo "*************************"
	@echo "* Assembling program... *"
	@echo " "	
#	@mkdir -p $(TARGETDIR)
	@$(ACME) -v2 --cpu 6510 -f CBM -o $(TARGETDIR)/$(PRGNAME) $(SRCNAME)
	@echo " "

	@echo "*************************"
	@echo "* Crunching program...  *"
	@echo " "
	@$(EXOMIZER) sfx 0x$(STARTADRESS) -m500 -p1 -x1 -p1 -t64 -o $(TARGETDIR)/$(PRGNAME) $(TARGETDIR)/$(PRGNAME) # > /dev/null
	@echo " "


#---------------------------------------------------------
# Create TARGETDIR directory if it doesn't exist
$(TARGETDIR) :
	@test -d "$(TARGETDIR)" || mkdir -p "$(TARGETDIR)"
	
	
#---------------------------------------------------------
clean : 
	@echo "*************************"
	@echo "* Cleaning...           *"
	@echo " "
	$(RM) -f $(STARTFILE)
	$(RM) -f $(DATEFILE)
	$(RM) -f $(TARGETDIR)/$(PRGNAME)
	rmdir $(TARGETDIR)
	@echo " "
