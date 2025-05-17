# Compile with clang-17 --target=riscv32 -march=rv32im_zve32x -mabi=ilp32 -nostdlib rvv2.S -o rvv2.elf -Wl,--section-start=.text=0x0 -Wl,-e_start -Wl,--section-start=.bss=0x10000
# Inspect assembly with `llvm-objdump-17 -D rvv2.elf`

.section .bss
.align 16
.global input_data
input_data:
    .space 64
.global output_data
output_data:
    .space 64

.section .text
.global _start
.option norelax
_start:
  la x29, input_data  # Input pointer
  #li x27, 0x10021  # Input pointer
  la x30, output_data  # Output pointer
  addi x10, x30, 16  # Output pointer
  li x31, 16       # vl
  li x28, 2        # Stride

  # Initialize vsetvli
  vsetvli x0, x31, e8, m1, ta, ma # e8

  # Load data
  vlse8.v v1, (x29), x28
  #vlse8.v v3, (x27), x28
  nop
  nop

  vadd.vi v2, v1, 8
  #vadd.vi v4, v3, 8

  # Store data
  vse8.v v2, 0(x30)
  #vse8.v v4, 0(x10)

  # Done, wait for committed!
  wfi