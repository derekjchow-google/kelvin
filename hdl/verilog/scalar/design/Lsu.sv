// Copyright 2025 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

`ifndef HDL_VERILOG_RVV_DESIGN_RVV_SVH
`include "rvv_backend.svh"
`endif

`ifndef HDL_VERILOG_SCALAR_INC_LSU_SVH
`include "Lsu.svh"
`endif

module LsuSv #(parameter N = 4,
               parameter BUSLENB = 128,
               parameter BUSLENBYTES = BUSLENB/8,
               parameter NLSU = 2,
               type XDataT=logic [31:0],
               type XAddrT=logic [31:0],
               type XRegAddrT=logic [4:0])
(
  input clk,
  input rstn,

  input logic [N-1:0] req_valid,
  input LsuCmd [N-1:0] req_data,
  output logic [N-1:0] req_ready,

  input XDataT [N-1:0] xdata,
  input XRegAddrT [N-1:0] xaddr,

  input XDataT [N-1:0] fdata,
  input XRegAddrT [N-1:0] faddr,

  output logic rd_valid,
  output XDataT rd_data,

  output logic flt_rd_valid,
  output XDataT flt_rd_data,

  // IBus
  output logic ibus_valid,
  output XAddrT ibus_addr,
  input logic ibus_ready,
  input logic [BUSLENB-1:0] ibus_rdata,  // Arrives one cycle after valid&ready
  input logic ibus_fault_valid,
  input logic ibus_fault_write,
  input XAddrT ibus_fault_addr,
  input XAddrT ibus_fault_epc,

  // DBus
  output logic dbus_valid,
  output logic dbus_write,
  output XAddrT dbus_pc,
  output XAddrT dbus_addr,
  output XAddrT dbus_adrx,
  output logic [$clog2(BUSLENBYTES):0] dbus_size,
  output logic [BUSLENB-1:0] dbus_wdata,
  output logic [BUSLENBYTES-1:0] dbus_wmask,
  input logic dbus_ready,
  input logic[BUSLENB-1:0] dbus_rdata,  // Arrives one cycle after valid&ready

  // EBus
  output logic ebus_valid,
  output logic ebus_write,
  output XAddrT ebus_pc,
  output XAddrT ebus_addr,
  output XAddrT ebus_adrx,
  output logic [$clog2(BUSLENBYTES):0] ebus_size,
  output logic [BUSLENB-1:0] ebus_wdata,
  output logic [BUSLENBYTES-1:0] ebus_wmask,
  input logic ebus_ready,
  input logic[BUSLENB-1:0] ebus_rdata,  // Arrives one cycle after valid&ready
  output logic ebus_internal,
  input logic ebus_fault_valid,
  input logic ebus_fault_write,
  input XAddrT ebus_fault_addr,
  input XAddrT ebus_fault_epc,

  // Flush
  output logic  flush_valid,
  input  logic  flush_ready,
  output logic  flush_all,
  output logic  flush_clean,
  output logic  flush_fencei,
  output XAddrT flush_pcNext,

  // Fault
  output logic fault_valid,
  output logic fault_write,
  output XAddrT fault_addr,
  output XAddrT fault_epc,

  // RVV to LSU
  input  logic         [NLSU-1:0] lsu_valid_rvv2lsu,
  input  UOP_RVV2LSU_t [NLSU-1:0] lsu_rvv2lsu,
  output logic         [NLSU-1:0] lsu_ready_lsu2rvv,

  // LSU to RVV
  output logic     [NLSU-1:0]              lsu_valid_lsu2rvv,
  output XRegAddrT [NLSU-1:0]              lsu_addr_lsu2rvv,
  output logic     [NLSU-1:0][BUSLENB-1:0] lsu_wdata_lsu2rvv,
  output logic     [NLSU-1:0]              lsu_last_lsu2rvv,
  input  logic     [NLSU-1:0]              lsu_ready_rvv2lsu,

  // Config state
  input logic config_state_valid,
  input RVVConfigState config_state,

  output logic [1:0] store_count,
  output logic [3:0] queue_capacity,
  output logic active
);

  assign req_ready = 0;
  assign ibus_valid = 0;
  assign ibus_addr = 0;

  assign dbus_valid = 0;
  assign dbus_write = 0;
  assign dbus_pc = 0;
  assign dbus_addr = 0;
  assign dbus_adrx = 0;
  assign dbus_size = 0;
  assign dbus_wdata = 0;
  assign dbus_wmask = 0;

  assign ebus_valid = 0;
  assign ebus_write = 0;
  assign ebus_pc = 0;
  assign ebus_addr = 0;
  assign ebus_adrx = 0;
  assign ebus_size = 0;
  assign ebus_wdata = 0;
  assign ebus_wmask = 0;
  assign ebus_internal = 0;

  assign lsu_ready_lsu2rvv = 0;

  assign lsu_valid_lsu2rvv = 0;
  assign lsu_addr_lsu2rvv = 0;
  assign lsu_wdata_lsu2rvv = 0;
  assign lsu_last_lsu2rvv = 0;

  assign flush_valid = 0;
  assign flush_pcNext = 0;

  // Fault
  assign fault_valid = 0;
  assign fault_write = 0;
  assign fault_addr = 0;
  assign fault_epc = 0;

  assign store_count = 0;
  assign queue_capacity = 0;
  assign active = 0;

endmodule;
