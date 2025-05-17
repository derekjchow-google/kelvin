# Compile with clang-17 --target=riscv32 -march=rv32im_zve32x -mabi=ilp32 -nostdlib rvv.S -o rvv.elf -Wl,--section-start=.text=0x0 -Wl,-e_start -Wl,--section-start=.bss=0x10000
# Inspect assembly with `llvm-objdump-17 -D rvv.elf`

.section .bss
.align 16
.global input_data
input_data:
    .space 32
.global output_data
output_data:
    .space 32

.section .text
.global _start
.option norelax
_start:
  li x29, 0x10020  # Input pointer
  li x30, 0x10040  # Output pointer
  li x31, 16          # vl

  # Initialize vsetvli
  vsetvli x0, x31, e8, m1, ta, ma # e8

  # Load data
  vle8.v v1, 0(x29)
  nop
  nop
  nop

  vadd.vi v2, v1, 1

  # Store data
  vse8.v v2, 0(x30)

  # Done, wait for committed!
  wfi