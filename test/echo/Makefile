# Generic 6510 assembler makefile - FTCH/HT
#
# Assumes that all files named .a, .s, i., .inc., .asm are to be considered source files

#Project specific settings
NAME = defmidi-echo
STARTADRESS = 0800
STARTFILE	= start.inc
DATEFILE	= date.inc
POSTFIX		= #.exe

# Names of main files and directories
TARGETDIR = bin
SRCNAME = code.a #$(NAME).a
#SRCNAME = 2019-02-xx-code.a #$(NAME).a
PRGNAME = $(NAME).prg
D64NAME = $(NAME).d64
LABFILE = _vicelabels.txt
VICEOUT = _viceoutput.txt

# Generate a comprehensive source file list
SOURCEFILES =
SOURCEFILES += $(wildcard *.a)
SOURCEFILES += $(wildcard *.asm)
SOURCEFILES += $(wildcard *.i)
SOURCEFILES += $(wildcard *.inc)
SOURCEFILES += $(wildcard *.s)
SOURCEFILES += $(wildcard *.src)

# Tool locations
TOOLDIR = ~/sys/c64/devtools_mac
#TOOLDIR = ../_win32tools
#TOOLDIR = ../amigatools

# Specific tool locations
ACME		= $(TOOLDIR)/acme0.93/acme$(POSTFIX)
	
PUCRUNCH	= $(TOOLDIR)/pucrunch/pucrunch$(POSTFIX)
EXOMIZER	= $(TOOLDIR)/exomizer-3/exomizer$(POSTFIX)

#8TO44		= $(TOOLDIR)/digiconv/8to44$(POSTFIX)
#ASMDATE		= $(TOOLDIR)/date/asmdate$(POSTFIX)

C1541		= $(TOOLDIR)/vice_3.1/x64sc.app/Contents/MacOS/c1541$(POSTFIX)
#VICE		= $(TOOLDIR)/vice_3.1/tools/x64$(POSTFIX)
VICE		= $(TOOLDIR)/vice_3.1/x64sc.app/Contents/MacOS/x64sc$(POSTFIX)

 

#---------------------------------------------------------
# Mekk .d64
$(TARGETDIR)/$(D64NAME) : $(TARGETDIR) $(TARGETDIR)/$(PRGNAME)
	@echo "*************************"
	@echo "* Creating .d64...      *"
	@echo " "
	@$(C1541) -format $(NAME),ht d64 $(TARGETDIR)/$(D64NAME) -attach $(TARGETDIR)/$(D64NAME) -write $(TARGETDIR)/$(PRGNAME) $(NAME)
	@echo " "

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
	@$(ACME) -v2 --cpu 6510 -f CBM --vicelabeldump $(LABFILE) -o $(TARGETDIR)/$(PRGNAME) $(SRCNAME)
	@echo " "

	@echo "*************************"
	@echo "* Crunching program...  *"
	@echo " "
	@$(EXOMIZER) sfx 0x$(STARTADRESS) -m500 -p1 -x1 -p1 -t64 -o $(TARGETDIR)/$(PRGNAME) $(TARGETDIR)/$(PRGNAME) # > /dev/null
#	@$(PUCRUNCH) -c64 -x0x$(STARTADRESS) -i0 -g0x35 -ffast $(TARGETDIR)/$(PRGNAME) $(TARGETDIR)/$(PRGNAME) > /dev/null
	@echo " "


#---------------------------------------------------------
# Create TARGETDIR directory if it doesn't exist
$(TARGETDIR) :
	@test -d "$(TARGETDIR)" || mkdir -p "$(TARGETDIR)"
	
	
#---------------------------------------------------------
# Open the .prg file on real C64 through 1541U2
xfer : $(TARGETDIR)/$(PRGNAME)
	1541u2.pl 192.168.2.64 xferscript.txt

#---------------------------------------------------------
# Open .prg file in VICE
vice : $(TARGETDIR)/$(D64NAME)
	@echo "*************************"
	@echo "* Open .d64 in vice...  *"
	@echo " "
	$(VICE) -truedrive +cart -autostart-handle-tde -moncommands $(LABFILE) $(TARGETDIR)/$(D64NAME) >>$(VICEOUT) 2>&1 & #&&
	@echo " "

#---------------------------------------------------------
clean : 
	@echo "*************************"
	@echo "* Cleaning...           *"
	@echo " "
	$(RM) -f $(STARTFILE)
	$(RM) -f $(DATEFILE)
	$(RM) -f $(TARGETDIR)/$(PRGNAME)
	$(RM) -f $(TARGETDIR)/$(D64NAME)
	$(RM) -f _*.txt
	rmdir $(TARGETDIR)
	@echo " "
	
