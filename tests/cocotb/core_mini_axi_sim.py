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
import numpy as np
import tqdm
import random

from tests.cocotb.core_mini_axi_interface import CoreMiniAxiInterface

@cocotb.test()
async def core_mini_axi_basic_write_read_memory(dut):
    """Basic test to check if TCM memory can be written and read back."""
    core_mini_axi = CoreMiniAxiInterface(dut)
    await core_mini_axi.init()
    await core_mini_axi.reset()
    cocotb.start_soon(core_mini_axi.clock.start())

    # Test reading/writing words
    await core_mini_axi.write_word(0x100, 0x42)
    await core_mini_axi.write_word(0x104, 0x43)
    rdata = (await core_mini_axi.read(0x100, 16)).view(np.uint32)
    assert (rdata[0:2] == np.array([0x42, 0x43])).all()

    # Three write/read data burst
    wdata = np.arange(48, dtype=np.uint8)
    await core_mini_axi.write(0x0, wdata)

    # Unaligned read, taking two bursts
    rdata = await core_mini_axi.read(0x8, 16)
    assert (np.arange(8, 24, dtype=np.uint8) == rdata).all()

    # # Unaligned write, taking two bursts
    wdata = np.arange(20, dtype=np.uint8)
    await core_mini_axi.write(0x204, wdata)
    rdata = await core_mini_axi.read(0x200, 32)
    assert (wdata == rdata[4:24]).all()


@cocotb.test()
async def core_mini_axi_run_wfi_in_all_slots(dut):
    """Tests the WFI instruction in each of the 4 issue slots."""
    core_mini_axi = CoreMiniAxiInterface(dut)
    cocotb.start_soon(core_mini_axi.clock.start())
    await core_mini_axi.init()

    for slot in range(0,4):
      with open(f"../tests/cocotb/wfi_slot_{slot}.elf", "rb") as f:
        await core_mini_axi.reset()
        entry_point = await core_mini_axi.load_elf(f)
        await core_mini_axi.execute_from(entry_point)

        await core_mini_axi.wait_for_wfi()
        await core_mini_axi.raise_irq()
        await core_mini_axi.wait_for_halted()

@cocotb.test()
async def core_mini_axi_slow_bready(dut):
  """Test that BVALID stays high until BREADY is presented"""
  core_mini_axi = CoreMiniAxiInterface(dut)
  await core_mini_axi.init()
  await core_mini_axi.reset()
  cocotb.start_soon(core_mini_axi.clock.start())

  wdata = np.arange(16, dtype=np.uint8)
  for i in tqdm.trange(100):
    bready_delay = random.randint(0, 50)
    await core_mini_axi.write(i*32, wdata, delay_bready=bready_delay)

  for _ in tqdm.trange(100):
    rdata = await core_mini_axi.read(i*32, 16)
    assert (wdata == rdata).all()


@cocotb.test()
async def core_mini_axi_write_read_memory_stress_test(dut):
    """Stress test reading/writing from DTCM."""
    core_mini_axi = CoreMiniAxiInterface(dut)
    await core_mini_axi.init()
    await core_mini_axi.reset()
    cocotb.start_soon(core_mini_axi.clock.start())

    # TODO(derekjchow): Write stress program to run on Kelvin

    # Range for a DTCM buffer we can read/write too.
    DTCM_START = 0x12000
    DTCM_END = 0x14000
    dtcm_model_buffer = np.zeros((DTCM_END - DTCM_START))

    for i in tqdm.trange(1000):
      start_addr = random.randint(DTCM_START, DTCM_END-2)
      end_addr = random.randint(start_addr, DTCM_END-1)
      transaction_length = end_addr - start_addr

      if random.randint(0, 1) == 1:
        wdata = np.random.randint(0, 256, transaction_length, dtype=np.uint8)
        await core_mini_axi.write(start_addr, wdata)
        dtcm_model_buffer[start_addr-DTCM_START: end_addr-DTCM_START] = wdata
      else:
        expected = dtcm_model_buffer[start_addr-DTCM_START: end_addr-DTCM_START]
        rdata = await core_mini_axi.read(start_addr, transaction_length)
        assert (expected == rdata).all()

@cocotb.test()
async def core_mini_axi_master_write_alignment(dut):
  """Test data alignment during AXI master writes"""
  core_mini_axi = CoreMiniAxiInterface(dut)
  await core_mini_axi.init()
  await core_mini_axi.reset()
  cocotb.start_soon(core_mini_axi.clock.start())

  with open("../tests/cocotb/align_test.elf", "rb") as f:
    entry_point = await core_mini_axi.load_elf(f)
    await core_mini_axi.execute_from(entry_point)

    await core_mini_axi.wait_for_halted(timeout_cycles=10000)
    assert core_mini_axi.dut.io_fault.value == 0

@cocotb.test()
async def core_mini_axi_finish_txn_before_halt_test(dut):
  core_mini_axi = CoreMiniAxiInterface(dut)
  await core_mini_axi.init()
  await core_mini_axi.reset()
  cocotb.start_soon(core_mini_axi.clock.start())

  with open("../tests/cocotb/finish_txn_before_halt.elf", "rb") as f:
    entry_point = await core_mini_axi.load_elf(f)
    await core_mini_axi.execute_from(entry_point)
    await core_mini_axi.wait_for_halted()

    assert (core_mini_axi.master_arfifo.qsize() + \
            core_mini_axi.master_rfifo.qsize() + \
            core_mini_axi.master_awfifo.qsize() + \
            core_mini_axi.master_wfifo.qsize() + \
            core_mini_axi.master_bfifo.qsize()) == 0
