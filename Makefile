x86: CC=gcc
x86: PLATFORM=x86
x86: x86mbt

arm64: CC=aarch64-linux-gnu-gcc
arm64: PLATFORM=arm64
arm64: aarch64mbt

CFLAGS= -Wall -g

MKDIR= mkdir -p
OUTDIR=./Release

OBJS:= mbt_common.o \
		mbt.o

%.o: %.c
	$(CC) -c -o $@ $^

x86mbt : $(OBJS)
	$(MKDIR) $(OUTDIR)/$(PLATFORM)
	$(CC) -o $(OUTDIR)/$(PLATFORM)/mbt $^

aarch64mbt : $(OBJS)
	$(MKDIR) $(OUTDIR)/$(PLATFORM)
	$(CC) -o $(OUTDIR)/$(PLATFORM)/mbt $^
	@cp $(OUTDIR)/$(PLATFORM)/mbt ./cyw5557x/scripts_for_xavier/
	@cp $(OUTDIR)/$(PLATFORM)/mbt ./cyw5557x/scripts_for_nano/
	@cp $(OUTDIR)/$(PLATFORM)/mbt ./cyw5557x/scripts_for_rpicm4/
	@cp $(OUTDIR)/$(PLATFORM)/mbt ./cyw5557x/scripts_for_imx8/

.PHONY: clean

clean :  
	@echo "Clean All"
	@rm *.o 2> /dev/null
	@rm Release -rf 2> /dev/null
