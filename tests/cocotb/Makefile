CC=clang-17
CC_FLAGS=--target=riscv32-none-eabi -march=rv32im_zicsr -nostdlib -T kelvin_tcm.ld

all: example_add.elf

example_add.elf: example_add.c
	$(CC) $(CC_FLAGS) -o example_add.elf example_add.c kelvin_start.S

.PHONY: clean
clean:
	rm -rf *.elf *.bin
