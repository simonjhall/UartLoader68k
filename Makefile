PREFIX=~/riscv-linux32/bin/riscv32-unknown-linux-gnu-
ARGS = -fno-rtti -fno-exceptions -march=rv32im -mabi=ilp32 -O3 -g
# ARGS = -fno-rtti -fno-exceptions -march=rv64imac -mabi=lp64  -O3 -g

ARGS += -I ../shared68k -DMACHINE_MODE

.SECONDARY:

all: uartloader.bin

clean:
	rm -f *.bin *.o *.elf *.a

%.bin: %.elf
	$(PREFIX)objcopy -O binary $^ $@

%.o: %.cpp $(HEADERS) Makefile
	$(PREFIX)c++ $< -c -o $@ -g -std=c++14 -Wall $(ARGS)

%.o: %.S
	$(PREFIX)c++ $< -c -o $@ -g

../shared68k/shared.a:
	$(MAKE) -C ../shared68k

uartloader.elf: main.o ../shared68k/shared.a
	$(PREFIX)c++ $^ -o $@ -g -static -nostartfiles -L../shared68k -l:shared.a -Wl,-Tscript_rom.lds

uartloader.bin: uartloader.elf
	$(PREFIX)objcopy $^ -O binary $@
