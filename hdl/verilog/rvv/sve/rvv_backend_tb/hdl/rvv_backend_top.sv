`ifndef RVV_TOP__SV
`define RVV_TOP__SV

`include "rvv_backend_poke.svh"
`include "rvv_backend_tb_mod.sv"
module rvv_backend_top();

// clock & reset -----------------------------------------------------
  logic clk;
  logic rst_n;

  // Clock Generation
  parameter sim_cycle = 10;
  
  // Reset Delay Parameter
  parameter rst_delay = 5;

  always 
    begin
      #(sim_cycle/2) clk = ~clk;
    end

  //Driver reset depending on rst_delay
  initial begin
      clk = 0;
      rst_n = 0;
      repeat (rst_delay) @(posedge clk);
      rst_n = 1'b1;
      @(clk);
  end

// Instatiation ------------------------------------------------------
  rvs_interface rvs_if(clk,rst_n);
  lsu_interface lsu_if(clk,rst_n);
  vrf_interface vrf_if(clk,rst_n);

  rvv_intern_interface rvv_intern_if(clk,rst_n);
  
  rvv_backend_tb_mod test(); 
  
  rvv_backend DUT (
    .clk(clk),
    .rst_n(rst_n),
    
    .insts_valid_rvs2cq       (rvs_if.insts_valid_rvs2cq    ),
    .insts_rvs2cq             (rvs_if.insts_rvs2cq          ),
    .insts_ready_cq2rvs       (rvs_if.insts_ready_cq2rvs    ),
    
    .rt_xrf_rvv2rvs           (rvs_if.rt_xrf_rvv2rvs        ),
    .rt_xrf_valid_rvv2rvs     (rvs_if.rt_xrf_valid_rvv2rvs  ),
    .rt_xrf_ready_rvs2rvv     (rvs_if.rt_xrf_ready_rvs2rvv  ),

    .uop_lsu_valid_rvv2lsu    (lsu_if.uop_lsu_valid_rvv2lsu ),
    .uop_lsu_rvv2lsu          (lsu_if.uop_lsu_rvv2lsu       ),
    .uop_lsu_ready_lsu2rvv    (lsu_if.uop_lsu_ready_lsu2rvv ),

    .uop_lsu_valid_lsu2rvv    (lsu_if.uop_lsu_valid_lsu2rvv ),
    .uop_lsu_lsu2rvv          (lsu_if.uop_lsu_lsu2rvv       ),
    .uop_lsu_ready_rvv2lsu    (lsu_if.uop_lsu_ready_rvv2lsu ),

    
    .trap_valid_rvs2rvv       ('0                           ), // FIXME
    .trap_rvs2rvv             ('0                           ), // FIXME
    .trap_ready_rvv2rvs       (rvs_if.trap_ready_rvv2rvs    ), 

    .wr_vxsat_valid           (),
    .wr_vxsat                 (),
    .wr_vxsat_ready           (rvs_if.wr_vxsat_ready),

    .vcsr_valid               (rvs_if.vcsr_valid            ),
    .vector_csr               (rvs_if.vector_csr            ),
    .vcsr_ready               (1'b1            ) // FIXME
  );

// rvs interface -----------------------------------------------------
  assign rvs_if.rt_uop      = `RT_UOP_PATH.rt_uop;
  assign rvs_if.rt_last_uop = `RT_UOP_PATH.rt_last_uop;
  
  // ROB dataout 
  assign rvs_if.rd_valid_rob2rt = DUT.rd_valid_rob2rt;
  assign rvs_if.rd_rob2rt       = DUT.rd_rob2rt      ;
  assign rvs_if.rd_ready_rt2rob = DUT.rd_ready_rt2rob;


  // VRF retire
  always_comb begin
    for(int i=0; i<`NUM_RT_UOP; i++) begin
      rvs_if.rt_vrf_valid_rob2rt[i] = `RT_VRF_PATH.rt2vrf_write_valid[i];
      rvs_if.rt_vrf_data_rob2rt[i].uop_pc   = `RT_VRF_PATH.rob2rt_write_data[i].uop_pc;
      rvs_if.rt_vrf_data_rob2rt[i].rt_data  = `RT_VRF_PATH.rt2vrf_write_data[i].rt_data;
      rvs_if.rt_vrf_data_rob2rt[i].rt_index = `RT_VRF_PATH.rt2vrf_write_data[i].rt_index;
    end
  end
  assign rvs_if.rt_vrf_data_rob2rt[0].rt_strobe  = `RT_VRF_PATH.w_enB0;
  assign rvs_if.rt_vrf_data_rob2rt[1].rt_strobe  = `RT_VRF_PATH.w_enB1;
  assign rvs_if.rt_vrf_data_rob2rt[2].rt_strobe  = `RT_VRF_PATH.w_enB2;
  assign rvs_if.rt_vrf_data_rob2rt[3].rt_strobe  = `RT_VRF_PATH.w_enB3;

  assign rvs_if.wr_vxsat_valid[0] = `RT_VXSAT_PATH.w_valid0_chkTrap & `RT_VXSAT_PATH.w_vxsat0;
  assign rvs_if.wr_vxsat_valid[1] = `RT_VXSAT_PATH.w_valid1_chkTrap & `RT_VXSAT_PATH.w_vxsat1;
  assign rvs_if.wr_vxsat_valid[2] = `RT_VXSAT_PATH.w_valid2_chkTrap & `RT_VXSAT_PATH.w_vxsat2;
  assign rvs_if.wr_vxsat_valid[3] = `RT_VXSAT_PATH.w_valid3_chkTrap & `RT_VXSAT_PATH.w_vxsat3;

  assign rvs_if.wr_vxsat[0] = `RT_VXSAT_PATH.w_vxsat0;
  assign rvs_if.wr_vxsat[1] = `RT_VXSAT_PATH.w_vxsat1;
  assign rvs_if.wr_vxsat[2] = `RT_VXSAT_PATH.w_vxsat2;
  assign rvs_if.wr_vxsat[3] = `RT_VXSAT_PATH.w_vxsat3;

// vrf interface -----------------------------------------------------
  // For VRF value check, we need to delay a cycle to wait for writeback finished.
  always_ff @(posedge clk or negedge rst_n) begin
    if(~rst_n) begin
      vrf_if.rt_uop      <= '0;
      vrf_if.rt_last_uop <= '0;
    end else begin
      vrf_if.rt_uop      <= `RT_UOP_PATH.rt_uop;
      vrf_if.rt_last_uop <= `RT_UOP_PATH.rt_last_uop;
    end
  end

  always_comb begin: vrf_connect
    for(int i=0; i<32; i++) begin
      vrf_if.vreg[i] = `VRF_PATH.vrf_rd_data_full[i];
    end
    vrf_if.vrf_wr_wenb_full = `VRF_PATH.vrf_wr_wenb_full;
    vrf_if.vrf_wr_data_full = `VRF_PATH.vrf_wr_data_full;
  end: vrf_connect

// Internal Signals connection ---------------------------------------
  // ROB to Retire
  assign rvv_intern_if.rob2rt_write_valid = DUT.u_retire.rob2rt_write_valid;
  assign rvv_intern_if.rob2rt_write_data  = DUT.u_retire.rob2rt_write_data;
  assign rvv_intern_if.rt2rob_write_ready = DUT.u_retire.rt2rob_write_ready;

  // Decode to UOPs queue
  assign rvv_intern_if.uop_valid_de2uq  = DUT.u_decode.uop_valid_de2uq;
  
  // Disptach to each rs
  /* Dispatch unit to ALU reservation station */
  assign rvv_intern_if.rs_valid_dp2alu = DUT.u_dispatch.rs_valid_dp2alu;
  assign rvv_intern_if.rs_ready_alu2dp = DUT.u_dispatch.rs_ready_alu2dp;

  /* Dispatch unit to PMT+RDT reservation station */
  assign rvv_intern_if.rs_valid_dp2pmtrdt = DUT.u_dispatch.rs_valid_dp2pmtrdt;
  assign rvv_intern_if.rs_ready_pmtrdt2dp = DUT.u_dispatch.rs_ready_pmtrdt2dp;

  /* Dispatch unit to MUL reservation station */
  assign rvv_intern_if.rs_valid_dp2mul = DUT.u_dispatch.rs_valid_dp2mul;
  assign rvv_intern_if.rs_ready_mul2dp = DUT.u_dispatch.rs_ready_mul2dp;

  /* Dispatch unit to DIV reservation station */
  assign rvv_intern_if.rs_valid_dp2div = DUT.u_dispatch.rs_valid_dp2div;
  assign rvv_intern_if.rs_ready_div2dp = DUT.u_dispatch.rs_ready_div2dp;

  /* Dispatch unit to LSU reservation station */
  assign rvv_intern_if.rs_valid_dp2lsu = DUT.u_dispatch.rs_valid_dp2lsu;
  assign rvv_intern_if.rs_ready_lsu2dp = DUT.u_dispatch.rs_ready_lsu2dp;

  // FIFO empty/full signals
  /* CMD queue */
  assign rvv_intern_if.cmd_q_full  = DUT.u_command_queue.full;
  assign rvv_intern_if.cmd_q_empty = DUT.u_command_queue.empty;

  /* UOPs queue */
  assign rvv_intern_if.uop_q_full  = DUT.u_uop_queue.full;
  assign rvv_intern_if.uop_q_empty = DUT.u_uop_queue.empty;

  /* RS */
  assign rvv_intern_if.alu_rs_full  = DUT.u_alu_rs.full;
  assign rvv_intern_if.alu_rs_empty = DUT.u_alu_rs.empty;
  assign rvv_intern_if.mul_rs_full  = DUT.u_mul_rs.full;
  assign rvv_intern_if.mul_rs_empty = DUT.u_mul_rs.empty;
  assign rvv_intern_if.div_rs_full  = DUT.u_div_rs.full;
  assign rvv_intern_if.div_rs_empty = DUT.u_div_rs.empty;
  assign rvv_intern_if.pmtrdt_rs_full  = DUT.u_pmtrdt_rs.full;
  assign rvv_intern_if.pmtrdt_rs_empty = DUT.u_pmtrdt_rs.empty;
  assign rvv_intern_if.lsu_rs_full  = DUT.u_lsu_rs.full;
  assign rvv_intern_if.lsu_rs_empty = DUT.u_lsu_rs.empty;

  /* ROB */
  assign rvv_intern_if.rob_empty = DUT.u_rob.u_uop_valid_fifo.empty;

  /* vrf */
  assign rvv_intern_if.vrf_wr_wenb_full = `VRF_PATH.vrf_wr_wenb_full;

// Interface Coverage Collection ----------------------------------------
  rvv_interface_cov rvv_interface_cov(
    .clk(clk), 
    .rst_n(rst_n), 
    .rvv_intern_if(rvv_intern_if)
  );



endmodule: rvv_backend_top

`endif // RVV_TOP__SV
