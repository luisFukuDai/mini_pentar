PREFIX=rx-elf

DEBUG_FLAGS=

# Specify compiler to be used
CC = $(PREFIX)-gcc

# Specify Assembler to be used
AS = $(PREFIX)-as

# Specify CPU flag
CPU = -mcpu=rx600 -mlittle-endian-data -mint-register=0  

# Common compiler flags
CFLAGS = -nostartfiles -lm -g -g2  -Wall

ALL_FLAGS = $(CFLAGS) $(DEBUG_FLAGS) $(CPU) 

# ROBOT number
R = 0

# Application name
#APPNAME = 

# Specify all objects that you are going to link together
SOURCES = reset_program.S $(wildcard *.c)
OBJECTS = $(SOURCES:.c=.o) 

rx621.mot : rx621.out
	$(PREFIX)-objcopy -O srec rx621.out $@;cp rx621.mot ~/winshare

# Define ROMSTART if compiling for ROM
rx621.out:$(SOURCES) rx621.ld makefile
	$(CC) $(ALL_FLAGS) -DROMSTART  -DROBOT$(R) -T rx621.ld -Xlinker -Map -Xlinker rx621.map -o $@ 	$(SOURCES) -L. -e _PowerON_Reset libnosys.a liboptc.a liboptm.a

all:rx621.mot
rom:rx621.mot

r1: R=0 all

clean:
	rm -f *.o *.out *.mot *.map
