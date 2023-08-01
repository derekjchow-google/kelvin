module Sram_1rwm_256x288(
  input          clock,
  input          valid,
  input          write,
  input  [7:0]   addr,
  input  [287:0] wdata,
  input  [31:0]  wmask,
  output [287:0] rdata
);

reg [287:0] mem [0:255];
reg [7:0] raddr;

assign rdata = mem[raddr];

`ifdef FPGA

always @(posedge clock) begin
  for (int i = 0; i < 32; i++) begin
    if (valid & write & wmask[i]) begin
      mem[addr][i*9 +: 9] <= wdata[i*9 +: 9];
    end
  end
  if (valid & ~write) begin
    raddr <= addr;
  end
end

endmodule  // Sram_1rwm_256x288

`else  // !FPGA

Sram_1rw_256x9 u_bl00(clock, valid & (~write | wmask[0]),  write, addr, wdata[  0 +: 9], rdata[  0 +: 9]);
Sram_1rw_256x9 u_bl01(clock, valid & (~write | wmask[1]),  write, addr, wdata[  9 +: 9], rdata[  9 +: 9]);
Sram_1rw_256x9 u_bl02(clock, valid & (~write | wmask[2]),  write, addr, wdata[ 18 +: 9], rdata[ 18 +: 9]);
Sram_1rw_256x9 u_bl03(clock, valid & (~write | wmask[3]),  write, addr, wdata[ 27 +: 9], rdata[ 27 +: 9]);
Sram_1rw_256x9 u_bl04(clock, valid & (~write | wmask[4]),  write, addr, wdata[ 36 +: 9], rdata[ 36 +: 9]);
Sram_1rw_256x9 u_bl05(clock, valid & (~write | wmask[5]),  write, addr, wdata[ 45 +: 9], rdata[ 45 +: 9]);
Sram_1rw_256x9 u_bl06(clock, valid & (~write | wmask[6]),  write, addr, wdata[ 54 +: 9], rdata[ 54 +: 9]);
Sram_1rw_256x9 u_bl07(clock, valid & (~write | wmask[7]),  write, addr, wdata[ 63 +: 9], rdata[ 63 +: 9]);
Sram_1rw_256x9 u_bl08(clock, valid & (~write | wmask[8]),  write, addr, wdata[ 72 +: 9], rdata[ 72 +: 9]);
Sram_1rw_256x9 u_bl09(clock, valid & (~write | wmask[9]),  write, addr, wdata[ 81 +: 9], rdata[ 81 +: 9]);
Sram_1rw_256x9 u_bl10(clock, valid & (~write | wmask[10]), write, addr, wdata[ 90 +: 9], rdata[ 90 +: 9]);
Sram_1rw_256x9 u_bl11(clock, valid & (~write | wmask[11]), write, addr, wdata[ 99 +: 9], rdata[ 99 +: 9]);
Sram_1rw_256x9 u_bl12(clock, valid & (~write | wmask[12]), write, addr, wdata[108 +: 9], rdata[108 +: 9]);
Sram_1rw_256x9 u_bl13(clock, valid & (~write | wmask[13]), write, addr, wdata[117 +: 9], rdata[117 +: 9]);
Sram_1rw_256x9 u_bl14(clock, valid & (~write | wmask[14]), write, addr, wdata[126 +: 9], rdata[126 +: 9]);
Sram_1rw_256x9 u_bl15(clock, valid & (~write | wmask[15]), write, addr, wdata[135 +: 9], rdata[135 +: 9]);
Sram_1rw_256x9 u_bl16(clock, valid & (~write | wmask[16]), write, addr, wdata[144 +: 9], rdata[144 +: 9]);
Sram_1rw_256x9 u_bl17(clock, valid & (~write | wmask[17]), write, addr, wdata[153 +: 9], rdata[153 +: 9]);
Sram_1rw_256x9 u_bl18(clock, valid & (~write | wmask[18]), write, addr, wdata[162 +: 9], rdata[162 +: 9]);
Sram_1rw_256x9 u_bl19(clock, valid & (~write | wmask[19]), write, addr, wdata[171 +: 9], rdata[171 +: 9]);
Sram_1rw_256x9 u_bl20(clock, valid & (~write | wmask[20]), write, addr, wdata[180 +: 9], rdata[180 +: 9]);
Sram_1rw_256x9 u_bl21(clock, valid & (~write | wmask[21]), write, addr, wdata[189 +: 9], rdata[189 +: 9]);
Sram_1rw_256x9 u_bl22(clock, valid & (~write | wmask[22]), write, addr, wdata[198 +: 9], rdata[198 +: 9]);
Sram_1rw_256x9 u_bl23(clock, valid & (~write | wmask[23]), write, addr, wdata[207 +: 9], rdata[207 +: 9]);
Sram_1rw_256x9 u_bl24(clock, valid & (~write | wmask[24]), write, addr, wdata[216 +: 9], rdata[216 +: 9]);
Sram_1rw_256x9 u_bl25(clock, valid & (~write | wmask[25]), write, addr, wdata[225 +: 9], rdata[225 +: 9]);
Sram_1rw_256x9 u_bl26(clock, valid & (~write | wmask[26]), write, addr, wdata[234 +: 9], rdata[234 +: 9]);
Sram_1rw_256x9 u_bl27(clock, valid & (~write | wmask[27]), write, addr, wdata[243 +: 9], rdata[243 +: 9]);
Sram_1rw_256x9 u_bl28(clock, valid & (~write | wmask[28]), write, addr, wdata[252 +: 9], rdata[252 +: 9]);
Sram_1rw_256x9 u_bl29(clock, valid & (~write | wmask[29]), write, addr, wdata[261 +: 9], rdata[261 +: 9]);
Sram_1rw_256x9 u_bl30(clock, valid & (~write | wmask[30]), write, addr, wdata[270 +: 9], rdata[270 +: 9]);
Sram_1rw_256x9 u_bl31(clock, valid & (~write | wmask[31]), write, addr, wdata[279 +: 9], rdata[279 +: 9]);

endmodule  // Sram_1rwm_256x288

module Sram_1rw_256x9(
  input          clock,
  input          valid,
  input          write,
  input  [7:0]   addr,
  input  [8:0]   wdata,
  output [8:0]   rdata
);

  reg [8:0] mem [0:255];
  reg [7:0] raddr;

  assign rdata = mem[raddr];

  always @(posedge clock) begin
    if (valid & write) begin
      mem[addr] <= wdata;
    end
    if (valid & ~write) begin
      raddr <= addr;
    end
  end
endmodule  // Sram_1rw_256x9

`endif  // FPGA
