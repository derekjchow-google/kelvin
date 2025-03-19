`ifndef LSU_INTERFACE__SV
`define LSU_INTERFACE__SV

`include "rvv_backend_define.svh"
`include "rvv_backend.svh"

interface lsu_interface (input bit clk, input bit rst_n);


// load/store unit interface
  // RVV send LSU uop to RVS
    logic             [`NUM_LSU-1:0]          uop_lsu_valid_rvv2lsu;
    UOP_RVV2LSU_t     [`NUM_LSU-1:0]          uop_lsu_rvv2lsu;
    logic             [`NUM_LSU-1:0]          uop_lsu_ready_lsu2rvv;
  // LSU feedback to RVV
    logic             [`NUM_LSU-1:0]          uop_lsu_valid_lsu2rvv;
    UOP_LSU2RVV_t     [`NUM_LSU-1:0]          uop_lsu_lsu2rvv;
    logic             [`NUM_LSU-1:0]          uop_lsu_ready_rvv2lsu;

endinterface: lsu_interface

`endif // LSU_INTERFACE__SV
