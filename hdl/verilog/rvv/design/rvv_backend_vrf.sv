/*
description: 
1. the VRF contains 32xVLEN register file. It support 4 read ports and 4 write ports

feature list:
*/
`include "rvv.svh"

module rvv_backend_vrf(/*AUTOARG*/
   // Outputs
   vrf2dp_rd_struct,
   // Inputs
   clk, rst_n, dp2vrf_rd_struct, rt2vrf_wr_struct
   );  
// global signal
input   logic                   clk;
input   logic                   rst_n;
    
// Dispatch unit to VRF unit
// Vs_data would be return from VRF at the current cycle.
input DP2VRF_t                  dp2vrf_rd_struct;

// VRF to Dispatch read data
output VRF2DP_t                 vrf2dp_rd_struct;

// Write back to VRF
input   RT2VRF_t                rt2vrf_wr_struct;


// Wires & Regs
wire [`REGFILE_INDEX_WIDTH-1:0] rd_addr0;
wire [`REGFILE_INDEX_WIDTH-1:0] rd_addr1;
wire [`REGFILE_INDEX_WIDTH-1:0] rd_addr2;
wire [`REGFILE_INDEX_WIDTH-1:0] rd_addr3;

wire wr_valid0;
wire wr_valid1;
wire wr_valid2;
wire wr_valid3;

wire [`REGFILE_INDEX_WIDTH-1:0] wr_addr0;
wire [`REGFILE_INDEX_WIDTH-1:0] wr_addr1;
wire [`REGFILE_INDEX_WIDTH-1:0] wr_addr2;
wire [`REGFILE_INDEX_WIDTH-1:0] wr_addr3;

wire [`VLEN-1:0] wr_data0;
wire [`VLEN-1:0] wr_data1;
wire [`VLEN-1:0] wr_data2;
wire [`VLEN-1:0] wr_data3;

wire [`VLENB-1:0] wr_we0;
wire [`VLENB-1:0] wr_we1;
wire [`VLENB-1:0] wr_we2;
wire [`VLENB-1:0] wr_we3;

wire [`VLEN-1:0] wr_web0;
wire [`VLEN-1:0] wr_web1;
wire [`VLEN-1:0] wr_web2;
wire [`VLEN-1:0] wr_web3;

wire [`VLEN-1:0] vrf_addr0_old_data;
wire [`VLEN-1:0] vrf_addr1_old_data;
wire [`VLEN-1:0] vrf_addr2_old_data;
wire [`VLEN-1:0] vrf_addr3_old_data;

reg [31:0] vrf_wr_entry0;
reg [31:0] [`VLEN-1:0] vrf_wr_data0;

reg [31:0] vrf_wr_entry1;
reg [31:0] [`VLEN-1:0] vrf_wr_data1;

reg [31:0] vrf_wr_entry2;
reg [31:0] [`VLEN-1:0] vrf_wr_data2;

reg [31:0] vrf_wr_entry3;
reg [31:0] [`VLEN-1:0] vrf_wr_data3;

reg [31:0] vrf_wr_entry_full;
reg [31:0] [`VLEN-1:0] vrf_wr_data_full;

wire [31:0] [`VLEN-1:0] vrf_rd_data_full;


// DP2VRF data unpack
assign rd_addr0 = dp2vrf_rd_struct.dp2vrf_vr0_addr;
assign rd_addr1 = dp2vrf_rd_struct.dp2vrf_vr1_addr;
assign rd_addr2 = dp2vrf_rd_struct.dp2vrf_vr2_addr;
assign rd_addr3 = dp2vrf_rd_struct.dp2vrf_vr3_addr;

// RT2VRF data unpack
assign wr_valid0 = rt2vrf_wr_struct.rt2vrf_wr_valid[0];
assign wr_valid1 = rt2vrf_wr_struct.rt2vrf_wr_valid[1];
assign wr_valid2 = rt2vrf_wr_struct.rt2vrf_wr_valid[2];
assign wr_valid3 = rt2vrf_wr_struct.rt2vrf_wr_valid[3];

assign wr_addr0 = rt2vrf_wr_struct.rt2vrf_wr_data[0].rt_index;
assign wr_addr1 = rt2vrf_wr_struct.rt2vrf_wr_data[1].rt_index;
assign wr_addr2 = rt2vrf_wr_struct.rt2vrf_wr_data[2].rt_index;
assign wr_addr3 = rt2vrf_wr_struct.rt2vrf_wr_data[3].rt_index;

assign wr_data0 = rt2vrf_wr_struct.rt2vrf_wr_data[0].rt_data;
assign wr_data1 = rt2vrf_wr_struct.rt2vrf_wr_data[1].rt_data;
assign wr_data2 = rt2vrf_wr_struct.rt2vrf_wr_data[2].rt_data;
assign wr_data3 = rt2vrf_wr_struct.rt2vrf_wr_data[3].rt_data;

assign wr_we0 = rt2vrf_wr_struct.rt2vrf_wr_data[0].rt_strobe;
assign wr_we1 = rt2vrf_wr_struct.rt2vrf_wr_data[1].rt_strobe;
assign wr_we2 = rt2vrf_wr_struct.rt2vrf_wr_data[2].rt_strobe;
assign wr_we3 = rt2vrf_wr_struct.rt2vrf_wr_data[3].rt_strobe;

assign wr_web0 = {{8{wr_we0[15]}},{8{wr_we0[14]}},{8{wr_we0[13]}},{8{wr_we0[12]}},{8{wr_we0[11]}},{8{wr_we0[10]}},{8{wr_we0[9]}},{8{wr_we0[8]}},{8{wr_we0[7]}},{8{wr_we0[6]}},{8{wr_we0[5]}},{8{wr_we0[4]}},{8{wr_we0[3]}},{8{wr_we0[2]}},{8{wr_we0[1]}},{8{wr_we0[0]}}};

assign wr_web1 = {{8{wr_we1[15]}},{8{wr_we1[14]}},{8{wr_we1[13]}},{8{wr_we1[12]}},{8{wr_we1[11]}},{8{wr_we1[10]}},{8{wr_we1[9]}},{8{wr_we1[8]}},{8{wr_we1[7]}},{8{wr_we1[6]}},{8{wr_we1[5]}},{8{wr_we1[4]}},{8{wr_we1[3]}},{8{wr_we1[2]}},{8{wr_we1[1]}},{8{wr_we1[0]}}};

assign wr_web2 = {{8{wr_we2[15]}},{8{wr_we2[14]}},{8{wr_we2[13]}},{8{wr_we2[12]}},{8{wr_we2[11]}},{8{wr_we2[10]}},{8{wr_we2[9]}},{8{wr_we2[8]}},{8{wr_we2[7]}},{8{wr_we2[6]}},{8{wr_we2[5]}},{8{wr_we2[4]}},{8{wr_we2[3]}},{8{wr_we2[2]}},{8{wr_we2[1]}},{8{wr_we2[0]}}};

assign wr_web3 = {{8{wr_we3[15]}},{8{wr_we3[14]}},{8{wr_we3[13]}},{8{wr_we3[12]}},{8{wr_we3[11]}},{8{wr_we3[10]}},{8{wr_we3[9]}},{8{wr_we3[8]}},{8{wr_we3[7]}},{8{wr_we3[6]}},{8{wr_we3[5]}},{8{wr_we3[4]}},{8{wr_we3[3]}},{8{wr_we3[2]}},{8{wr_we3[1]}},{8{wr_we3[0]}}};

// Access Core
// Only write will update input

// Tmp old data in the VRF
assign vrf_addr0_old_data = vrf_rd_data_full[wr_addr0];
assign vrf_addr1_old_data = vrf_rd_data_full[wr_addr1];
assign vrf_addr2_old_data = vrf_rd_data_full[wr_addr2];
assign vrf_addr3_old_data = vrf_rd_data_full[wr_addr3];

always@(*) begin
  vrf_wr_entry0 = 32'b0;
  vrf_wr_data0 = 4096'b0;
  if (wr_valid0) begin
    vrf_wr_entry0[wr_addr0] = 1'b1;
    vrf_wr_data0[wr_addr0] = (wr_data0 & wr_web0) | (vrf_addr0_old_data & ~wr_web0);
  end
end

always@(*) begin
  vrf_wr_entry1 = 32'b0;
  vrf_wr_data1 = 4096'b0;
  if (wr_valid1) begin
    vrf_wr_entry1[wr_addr1] = 1'b1;
    vrf_wr_data1[wr_addr1] = (wr_data1 & wr_web1) | (vrf_addr1_old_data & ~wr_web1);
  end
end

always@(*) begin
  vrf_wr_entry2 = 32'b0;
  vrf_wr_data2 = 4096'b0;
  if (wr_valid2) begin
    vrf_wr_entry2[wr_addr2] = 1'b1;
    vrf_wr_data2[wr_addr2] = (wr_data2 & wr_web2) | (vrf_addr2_old_data & ~wr_web2);
  end
end

always@(*) begin
  vrf_wr_entry3 = 32'b0;
  vrf_wr_data3 = 4096'b0;
  if (wr_valid3) begin
    vrf_wr_entry3[wr_addr3] = 1'b1;
    vrf_wr_data3[wr_addr3] = (wr_data3 & wr_web3) | (vrf_addr3_old_data & ~wr_web3);
  end
end

// Mux 4 inputs to vrf_reg
integer i;
always@(*) begin
  for (i=0; i<32; i=i+1) begin
    vrf_wr_entry_full[i] = vrf_wr_entry3[i] | vrf_wr_entry2[i] | vrf_wr_entry1[i] | vrf_wr_entry0[i];
    vrf_wr_data_full[i] = vrf_wr_data3[i] | vrf_wr_data2[i] | vrf_wr_data1[i] | vrf_wr_data0[i];
  end
end

//VRF core
rvv_backend_vrf_reg vrf_reg (
  //Outputs
  .vreg(vrf_rd_data_full), 
  //Inputs
  .clk(clk), 
  .rst_n(rst_n),
  .wen(vrf_wr_entry_full), 
  .wdata(vrf_wr_data_full));

// VRF2DP data pack
assign vrf2dp_rd_struct.vrf2dp_rd0_data = vrf_rd_data_full[rd_addr0];
assign vrf2dp_rd_struct.vrf2dp_rd1_data = vrf_rd_data_full[rd_addr1];
assign vrf2dp_rd_struct.vrf2dp_rd2_data = vrf_rd_data_full[rd_addr2];
assign vrf2dp_rd_struct.vrf2dp_rd3_data = vrf_rd_data_full[rd_addr3];
assign vrf2dp_rd_struct.vrf2dp_v0_data = vrf_rd_data_full[0];


endmodule
