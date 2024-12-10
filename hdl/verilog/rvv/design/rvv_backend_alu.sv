// description: 
// 1. Instantiate rvv_backend_alu_unit and connect to ALU Reservation Station and ROB.
//
// feature list:
// 1. It will instantiate 2 rvv_backend_alu_unit.

`include "rvv_backend.svh"

module rvv_backend_alu
(
  pop_ex2rs,
  alu_uop_rs2ex,
  fifo_empty_rs2ex,
  fifo_1left_to_empty_rs2ex,
  
  result_valid_ex2rob,
  result_ex2rob,
  result_ready_rob2alu,
);

//
// interface signals
//
  // ALU RS to ALU unit
  output  logic       [`NUM_ALU_UOP-1:0]    pop_ex2rs;
  input   ALU_RS_t    [`NUM_ALU_UOP-1:0]    alu_uop_rs2ex;
  input   logic                             fifo_empty_rs2ex;
  input   logic                             fifo_1left_to_empty_rs2ex;

  // submit ALU result to ROB
  output  logic       [`NUM_ALU_UOP-1:0]    result_valid_ex2rob;
  output  ALU2ROB_t   [`NUM_ALU_UOP-1:0]    result_ex2rob;
  input   logic       [`NUM_ALU_UOP-1:0]    result_ready_rob2alu;

//
// internal signals
//
  // ALU RS to ALU unit
  logic               [`NUM_ALU_UOP-1:0]    alu_uop_valid_rs2ex;    

  genvar                                    i;

//
// Instantiate 2 rvv_backend_alu_unit
//
  // generate valid signals
  assign  alu_uop_valid_rs2ex[0] = !fifo_empty_rs2ex;
  assign  alu_uop_valid_rs2ex[1] = !(fifo_empty_rs2ex&fifo_1left_to_empty_rs2ex);
  
  // generate pop signals
  // it can pop alu_uop1 when it can also pop alu_uop0. Otherwise, it cannot pop alu_uop1.(That's in-ordered issue from RS) 
  assign  pop_ex2rs[0] = alu_uop_valid_rs2ex[0]&result_valid_ex2rob[0]&result_ready_rob2alu[0];
  assign  pop_ex2rs[1] = pop_ex2rs[0]&(alu_uop_valid_rs2ex[1]&result_valid_ex2rob[1]&result_ready_rob2alu[1]);  

  // instantiate
  generate
    for (i=0;i<`NUM_ALU_UOP;i=i+1) 
    begin: ALU_UNIT
      rvv_backend_alu_unit u_alu_unit[i]
        (
          // inputs
          .alu_uop_valid          (alu_uop_valid_rs2ex[i]),
          .alu_uop                (alu_uop_rs2ex[i]),
          // outputs
          .result_ex2rob_valid    (result_valid_ex2rob[i]),
          .result_ex2rob          (result_ex2rob[i])
        );
    end
  endgenerate

`ifdef ASSERT_ON
  `rvv_forbid(alu_uop_valid_rs2ex[0]&(!result_valid_ex2rob[0])) 
    else $error("rob_entry=%d. Something wrong in alu_unit0 decoding and execution.\n",alu_uop_rs2ex[0].rob_entry);

  `rvv_forbid(alu_uop_valid_rs2ex[1]&(!result_valid_ex2rob[1])) 
    else $error("rob_entry=%d. Something wrong in alu_unit1 decoding and execution.\n",alu_uop1_rs2ex[1].rob_entry);
`endif

endmodule
