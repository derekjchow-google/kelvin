# Copyright 2025 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import cocotb
import glob
import numpy as np
import os
import tqdm
import random

from cocotb.triggers import Timer, ClockCycles, RisingEdge, FallingEdge
from kelvin_test_utils.core_mini_axi_interface import AxiBurst, AxiResp,CoreMiniAxiInterface


#@cocotb.test()
async def rvv_core_mini_axi_basic_test(dut):
    """A basic rvv test."""
    core_mini_axi = CoreMiniAxiInterface(dut)
    cocotb.start_soon(core_mini_axi.clock.start())
    await core_mini_axi.init()

    with open(f"../tests/cocotb/rvv.elf", "rb") as f:
      await core_mini_axi.reset()
      print("Loading program", flush=True)
      entry_point = await core_mini_axi.load_elf(f)
      input_data = 0x00010020 # Linker not working great
      # input_data = core_mini_axi.lookup_symbol(f, "input_data")
      # output_data = core_mini_axi.lookup_symbol(f, "output_data")
      output_data = 0x00010040 # Linker not working great
      #await core_mini_axi.execute_from(entry_point)



    print(hex(entry_point), flush=True)
    print(hex(input_data), flush=True)
    print(hex(output_data), flush=True)

    print("Loading data", flush=True)
    await core_mini_axi.write(
      input_data,
      np.array([0x42, 0x42, 0x42, 0x42,
                0x41, 0x41, 0x41, 0x41,
                0x42, 0x42, 0x42, 0x42,
                0x45, 0x45, 0x45, 0x45,], dtype=np.uint8))
    await core_mini_axi.write(
      output_data,
      np.array([0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,], dtype=np.uint8))

    await core_mini_axi.execute_from(entry_point)

    # for i in range(100):
    #   await RisingEdge(dut.io_aclk)

    await core_mini_axi.wait_for_wfi()

    rdata = await core_mini_axi.read(output_data, 16)
    print("\n\n\n\n\n\n\n")
    print(rdata, flush=True)

      # await core_mini_axi.wait_for_wfi()
      # #await core_mini_axi.raise_irq()
      # #await core_mini_axi.wait_for_halted()

@cocotb.test()
async def rvv_core_mini_axi_strided(dut):
    """A basic rvv strided test."""
    core_mini_axi = CoreMiniAxiInterface(dut)
    cocotb.start_soon(core_mini_axi.clock.start())
    await core_mini_axi.init()

    with open(f"../tests/cocotb/rvv2.elf", "rb") as f:
      await core_mini_axi.reset()
      print("Loading program", flush=True)
      entry_point = await core_mini_axi.load_elf(f)
      input_data = core_mini_axi.lookup_symbol(f, "input_data")
      output_data = core_mini_axi.lookup_symbol(f, "output_data")


    print("Loading data", flush=True)
    await core_mini_axi.write(input_data, np.arange(32, dtype=np.uint8))

    await core_mini_axi.execute_from(entry_point)

    await core_mini_axi.wait_for_wfi()

    rdata = await core_mini_axi.read(output_data, 16)
    print("\n\n\n\n\n\n\n")
    print(rdata, flush=True)
  
