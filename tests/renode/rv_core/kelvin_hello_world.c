/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// A Simple kelvin program.

#include <stddef.h>
#include <stdint.h>

volatile uint8_t* uart0 = (uint8_t*)0x54000000L;
void putc(char ch) {
  *uart0 = ch;
}

char hex[] = {
  '0', '1', '2', '3',
  '4', '5', '6', '7',
  '8', '9', 'a', 'b',
  'c', 'd', 'e', 'f',
};

void print_uint32(uint32_t val) {
  putc('0');
  putc('x');
  for (int i = 7; i >= 0; --i) {
    putc(hex[(val >> (i * 4)) & 0xF]);
  }
  putc('\n');
}

void print_uint16(uint16_t val) {
  putc('0');
  putc('x');
  for (int i = 3; i >= 0; --i) {
    putc(hex[(val >> (i * 4)) & 0xF]);
  }
  putc('\n');
}

void print_uint8(uint8_t val) {
  putc('0');
  putc('x');
  for (int i = 1; i >= 0; --i) {
    putc(hex[(val >> (i * 4)) & 0xF]);
  }
  putc('\n');
}

void print_string(const char* s) {
  while (*s) {
    putc(*s);
    s++;
  }
}

int main(int argc, char *argv[]) {
  volatile uint32_t* rv_core_memory32 = (uint32_t*)0x20000000L;
  volatile uint16_t* rv_core_memory16 = (uint16_t*)0x20000000L;
  volatile uint8_t* rv_core_memory8 = (uint8_t*)0x20000000L;
  *rv_core_memory32 = 0xdeadbeef;
  print_uint32(*rv_core_memory32);
  *rv_core_memory16 = 0xb0ba;
  print_uint16(*rv_core_memory16);
  *rv_core_memory8 = 0xdd;
  print_uint8(*rv_core_memory8);

  uint32_t* our_pc_csr = (uint32_t*)0x30110L;
  print_uint32(*our_pc_csr);
  print_string("beefb0ba\n");
  print_uint32(0xb0bacafeL);
  asm volatile("wfi");
  asm volatile(".word 0x26000077");  // flushall
  return 0;
}
