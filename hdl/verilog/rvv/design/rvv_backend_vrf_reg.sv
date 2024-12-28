// rvv_vrf_reg - 32 vector regsiter for V standard extention
`include "rvv_backend.svh"
`include "rvv_backend_sva.svh"
module rvv_backend_vrf_reg (/*AUTOARG*/
   // Outputs
   vreg,
   // Inputs
   wenb, wdata, clk, rst_n
   );

  output logic [31:0][`VLEN-1:0]  vreg;

  input  logic [31:0][`VLEN-1:0]  wenb; // bit en
  input  logic [31:0][`VLEN-1:0]  wdata;
  input  logic                    clk;
  input  logic                    rst_n;

`ifdef TB_SUPPORT
  logic [31:0][`VLEN-1:0]  vreg_init_data;
`endif
// -- 32 vector registers --------------------------------------------
genvar i,j;
generate
  for (i=0; i<32; i=i+1) begin
    for (j=0; j<`VLEN; j=j+1) begin
      edff #(1) vrf_unit1_reg (
`ifdef TB_SUPPORT
        .init_data(vreg_init_data[i][j]),
`endif
        .q      (vreg[i][j]),
        .en     (wenb[i][j]),
        .d      (wdata[i][j]),
        .clk    (clk),
        .rst_n  (rst_n)
        );
    `ifdef ASSERT_ON
      `rvv_forbid($isunknown(vreg[i][j]))
        else $error("VREG: data is unknow at vreg[%0d][%0d]",i,j);
    `endif //ASSERT_ON
    end //end for loop j
  end //end for loop i
endgenerate


endmodule
