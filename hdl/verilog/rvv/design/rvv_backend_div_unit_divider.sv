
`ifndef HDL_VERILOG_RVV_DESIGN_RVV_SVH
`include "rvv_backend.svh"
`endif

`ifndef DIV_DEFINE_SVH
`include "rvv_backend_div.svh"
`endif

`ifndef RVV_ASSERT__SVH
`include "rvv_backend_sva.svh"
`endif

module rvv_backend_div_unit_divider
(
  clk,
  rst_n,
  div_valid,
  opcode,
  src2_dividend,
  src1_divisor,
  result_quotient,
  result_remainder,
  result_valid,
  result_ready
);
//
// parameter
//
  parameter DIV_WIDTH = `WORD_WIDTH;

//
// interface signals
//
  // global signals
  input   logic                 clk;
  input   logic                 rst_n;

  // opcode 
  input   DIV_SIGN_SRC_e        opcode;

  // operand 
  input   logic                 div_valid;
  input   logic [DIV_WIDTH-1:0] src2_dividend;
  input   logic [DIV_WIDTH-1:0] src1_divisor;

  // result
  output  logic [DIV_WIDTH-1:0] result_quotient;
  output  logic [DIV_WIDTH-1:0] result_remainder;
  output  logic                 result_valid;
  input   logic                 result_ready;

//
// internal signals
//
  // FSM
  enum  logic [1:0] {DIV_IDLE, DIV_WORKING, DIV_PRINT} state, next_state;
  
  // number of leading zero bits
  logic [$clog2(DIV_WIDTH):0]   cnt_clzb;

  // processing round for divider
  logic [DIV_WIDTH-1:0]         quotient_r1;
  logic [DIV_WIDTH-1:0]         quotient_r2;
  logic [DIV_WIDTH-1:0]         quotient_r3;
  logic [DIV_WIDTH-1:0]         quotient_r4;
  logic [DIV_WIDTH-1:0]         remainder_r1;
  logic [DIV_WIDTH-1:0]         remainder_r2;
  logic [DIV_WIDTH-1:0]         remainder_r3;
  logic [DIV_WIDTH-1:0]         remainder_r4;
  logic [$clog2(DIV_WIDTH):0]   count_r1;
  logic [$clog2(DIV_WIDTH):0]   count_r2;
  logic [$clog2(DIV_WIDTH):0]   count_r3;
  logic [$clog2(DIV_WIDTH):0]   count_r4;

  // register signals
  logic                         dividend_en;
  logic [DIV_WIDTH-1:0]         dividend_d;
  logic [DIV_WIDTH-1:0]         dividend_q;
  logic                         divisor_en;
  logic [DIV_WIDTH-1:0]         divisor_d;
  logic [DIV_WIDTH-1:0]         divisor_q;
  logic                         quotient_en;
  logic [DIV_WIDTH-1:0]         quotient_d;
  logic [DIV_WIDTH-1:0]         quotient_q;
  logic                         remainder_en;
  logic [DIV_WIDTH-1:0]         remainder_d;
  logic [DIV_WIDTH-1:0]         remainder_q;
  logic                         count_en;
  logic [$clog2(DIV_WIDTH):0]   count_d;
  logic [$clog2(DIV_WIDTH):0]   count_q;
  logic                         q_sgn_en;
  logic                         q_sgn_d;
  logic                         q_sgn_q;
  logic                         r_sgn_en;
  logic                         r_sgn_d;
  logic                         r_sgn_q;
  
//
// register
//
  // dividend
  edff
  #(
  .WIDTH    (DIV_WIDTH)
  )
  dividend
  (
    .clk    (clk),
    .rst_n  (rst_n),
    .en     (dividend_en),
    .d      (dividend_d),
    .q      (dividend_q)
  );
  // divisor
  edff
  #(
  .WIDTH    (DIV_WIDTH)
  )
  divisor
  (
    .clk    (clk),
    .rst_n  (rst_n),
    .en     (divisor_en),
    .d      (divisor_d),
    .q      (divisor_q)
  );
  // quotient
  edff
  #(
  .WIDTH    (DIV_WIDTH)
  )
  quotient
  (
    .clk    (clk),
    .rst_n  (rst_n),
    .en     (quotient_en),
    .d      (quotient_d),
    .q      (quotient_q)
  );
  // remainder
  edff
  #(
  .WIDTH    (DIV_WIDTH)
  )
  remainder
  (
    .clk    (clk),
    .rst_n  (rst_n),
    .en     (remainder_en),
    .d      (remainder_d),
    .q      (remainder_q)
  );
  // count
  edff
  #(
  .WIDTH    ($clog2(DIV_WIDTH)+1)
  )
  count
  (
    .clk    (clk),
    .rst_n  (rst_n),
    .en     (count_en),
    .d      (count_d),
    .q      (count_q)
  );
  // record quotient sgn-bit
  edff
  #(
  .WIDTH    (1)
  )
  q_sgn
  (
    .clk    (clk),
    .rst_n  (rst_n),
    .en     (q_sgn_en),
    .d      (q_sgn_d),
    .q      (q_sgn_q)
  );
  // record remainder sgn-bit
  edff
  #(
  .WIDTH    (1)
  )
  r_sgn
  (
    .clk    (clk),
    .rst_n  (rst_n),
    .en     (r_sgn_en),
    .d      (r_sgn_d),
    .q      (r_sgn_q)
  );

//
// FSM
//
  always_ff @(posedge clk, negedge rst_n) begin
    if (rst_n=='b0) 
      state <= DIV_IDLE;
    else 
      state <= next_state;
  end
  
  // state transition
  always_comb begin
    next_state = state;

    case(state)
      DIV_IDLE: begin
        if(div_valid) begin
          if ((count_en=='b1)&(count_d=='b1))
            next_state = DIV_PRINT;
          else
            next_state = DIV_WORKING;
        end
      end
      DIV_WORKING: begin
        if(div_valid) begin
          if ((count_en=='b1)&(count_d=='b1))
            next_state = DIV_PRINT;
          else
            next_state = DIV_WORKING;
        end
      end
      DIV_PRINT: begin
        if ((div_valid)&(result_ready))
          next_state = DIV_IDLE;
        else
          next_state = DIV_PRINT;
      end
    endcase
  end

  // computational logic in every state
  always_comb begin
    // initial
    dividend_en      = 'b0;
    dividend_d       = 'b0;
    divisor_en       = 'b0;
    divisor_d        = 'b0;
    quotient_en      = 'b0;
    quotient_d       = 'b0;
    remainder_en     = 'b0;
    remainder_d      = 'b0;
    count_en         = 'b0;
    count_d          = 'b0;
    q_sgn_en         = 'b0;
    q_sgn_d          = 'b0;         
    r_sgn_en         = 'b0;
    r_sgn_d          = 'b0;
    cnt_clzb         = 'b0;
    quotient_r1      = 'b0;
    quotient_r2      = 'b0;
    quotient_r3      = 'b0;
    quotient_r4      = 'b0;
    remainder_r1     = 'b0;
    remainder_r2     = 'b0;
    remainder_r3     = 'b0;
    remainder_r4     = 'b0;
    count_r1         = 'b0;
    count_r2         = 'b0;
    count_r3         = 'b0;
    count_r4         = 'b0;
    result_quotient  = 'b0;
    result_remainder = 'b0;
    result_valid     = 'b0;

    case(state)
      DIV_IDLE: begin
        if(div_valid) begin
          // check whether divisor is 0
          if(src1_divisor=='b0) begin
            dividend_en = 'b1;
            dividend_d  = 'b0;

            divisor_en = 'b1;
            divisor_d  = 'b0;

            quotient_en = 'b1;
            quotient_d  = '1;

            remainder_en = 'b1;
            remainder_d  = src2_dividend;

            q_sgn_en = 'b1;
            q_sgn_d  = 'b0;

            r_sgn_en = 'b1;
            r_sgn_d  = 'b0;

            count_en = 'b1;
            count_d  = 'b1;
          end
          // check whether is -2^(WIDTH-1)/-1. The result will overflow.
          else if ((opcode==DIV_SIGN)&(src2_dividend=={1'b1,{(DIV_WIDTH-1){1'b0}}})&(src1_divisor=='1)) begin
            quotient_en = 'b1;
            quotient_d  = {1'b1,{(DIV_WIDTH-1){1'b0}}};

            remainder_en = 'b1;
            remainder_d  = 'b0;

            q_sgn_en = 'b1;
            q_sgn_d  = 'b0;

            r_sgn_en = 'b1;
            r_sgn_d  = 'b0;

            count_en = 'b1;
            count_d  = 'b1;
          end
          // start to initialize
          else begin
            if(opcode==DIV_SIGN) begin
              // convert signed data to unsigned data
              dividend_d  = src2_dividend[DIV_WIDTH-1] ? (~(src2_dividend)+1) : src2_dividend;
              divisor_d  = src1_divisor[DIV_WIDTH-1] ? (~(src1_divisor )+1) : src1_divisor;
              
              q_sgn_d   = src2_dividend[DIV_WIDTH-1]^src1_divisor[DIV_WIDTH-1];
              r_sgn_d   = src2_dividend[DIV_WIDTH-1];
            end
            else begin
              dividend_d  = src2_dividend;
              divisor_d  = src1_divisor;

              q_sgn_d  = 'b0;
              r_sgn_d  = 'b0;
            end

            // check whether dividend and divisor equal to last source data
            if ((dividend_d==dividend_q)&(divisor_d==divisor_q)&(q_sgn_d==q_sgn_q)&(r_sgn_d==r_sgn_q)) begin
              dividend_en = 'b0; 
              divisor_en = 'b0;

              q_sgn_en  = 'b0;
              r_sgn_en  = 'b0;
              
              count_en = 'b1;
              count_d  = 'b1;
            end
            else begin
              dividend_en = 'b1; 
              divisor_en = 'b1;

              q_sgn_en  = 'b1;
              r_sgn_en  = 'b1;

              // count leading zero bits in dividend
              if (DIV_WIDTH==32) begin
                cnt_clzb  = f_clzb32(dividend_d);
                count_en  = 'b1;
                count_d   = 'd33 - cnt_clzb;            
              end
              else if (DIV_WIDTH==16) begin
                cnt_clzb  = f_clzb16(dividend_d);
                count_en  = 'b1;
                count_d   = 'd17 - cnt_clzb;            
              end
              else if (DIV_WIDTH==8) begin
                cnt_clzb  = f_clzb8(dividend_d);
                count_en  = 'b1;
                count_d   = 'd9 - cnt_clzb;            
              end

              // initialize quotient and remainder
              quotient_en = 'b1;   
              quotient_d  = dividend_d<<cnt_clzb;

              remainder_en = 'b1;
              remainder_d  = 'b0;
            end
          end
        end
      end
      DIV_WORKING: begin
        if(div_valid) begin
          // processing
          f_div_step (remainder_q ,quotient_q ,divisor_q,remainder_r1,quotient_r1);
          f_div_step (remainder_r1,quotient_r1,divisor_q,remainder_r2,quotient_r2);
          f_div_step (remainder_r2,quotient_r2,divisor_q,remainder_r3,quotient_r3);
          f_div_step (remainder_r3,quotient_r3,divisor_q,remainder_r4,quotient_r4);
          
          count_r1 = count_q - 'd1;
          count_r2 = count_q - 'd2;
          count_r3 = count_q - 'd3;
          count_r4 = count_q - 'd4;

          if(count_r1=='d1) begin
            quotient_en = 'b1;   
            quotient_d  = quotient_r1;

            remainder_en = 'b1;
            remainder_d  = remainder_r1;
            
            count_en = 'b1;
            count_d  = 'b1;            
          end
          else if(count_r2=='d1) begin
            quotient_en = 'b1;   
            quotient_d  = quotient_r2;

            remainder_en = 'b1;
            remainder_d  = remainder_r2;
            
            count_en = 'b1;
            count_d  = 'b1;            
          end
          else if(count_r3=='d1) begin
            quotient_en = 'b1;   
            quotient_d  = quotient_r3;

            remainder_en = 'b1;
            remainder_d  = remainder_r3;
            
            count_en = 'b1;
            count_d  = 'b1;            
          end
          else begin
            quotient_en = 'b1;   
            quotient_d  = quotient_r4;

            remainder_en = 'b1;
            remainder_d  = remainder_r4;
            
            count_en = 'b1;
            count_d  = count_r4;      
          end
        end  
      end
      // get result to ouput and wait for ready signal
      DIV_PRINT: begin
        result_valid = 'b1;
        
        result_quotient  = q_sgn_q ? ((~quotient_q )+'d1) : quotient_q;
        result_remainder = r_sgn_q ? ((~remainder_q)+'d1) : remainder_q;
    
        if (result_ready) begin
          count_en     = 'b1;
          count_d      = 'b0;
        end
      end
    endcase
  end

//
// function unit
//
  // count leading zero bits
  function [1:0] f_clzb2
  (   
    input logic [1:0] src
  );

    if (src==2'b00)
      f_clzb2 = 2'b10;
    else if (src==2'b01)
      f_clzb2 = 2'b01;
    else
      f_clzb2 = 2'b00;
  endfunction

  function [2:0] f_clzb4
  (
    input logic [3:0] src
  );

    logic [1:0] hi;
    logic [1:0] lo;

    hi = f_clzb2(src[3:2]);
    lo = f_clzb2(src[1:0]);
    if ((hi[1]==1'b1)&(lo[1]==1'b1))
      f_clzb4 = 3'b100;
    else if (hi[1]==1'b0)
      f_clzb4 = {1'b0,hi};
    else
      f_clzb4 = {2'b01,lo[0]};
  endfunction

  function [3:0] f_clzb8
  (
    input logic [7:0] src
  );

    logic [2:0] hi;
    logic [2:0] lo;

    hi = f_clzb4(src[7:4]);
    lo = f_clzb4(src[3:0]);
    if ((hi[2]==1'b1)&(lo[2]==1'b1))
      f_clzb8 = 4'b1000;
    else if (hi[2]==1'b0)
      f_clzb8 = {1'b0,hi};
    else
      f_clzb8 = {2'b01,lo[1:0]};
  endfunction

  function [4:0] f_clzb16
  (
    input logic [15:0] src
  );

    logic [3:0] hi;
    logic [3:0] lo;

    hi = f_clzb8(src[15:8]);
    lo = f_clzb8(src[7:0]);
    if ((hi[3]==1'b1)&(lo[3]==1'b1))
      f_clzb16 = 5'b1_0000;
    else if (hi[3]==1'b0)
      f_clzb16 = {1'b0,hi};
    else
      f_clzb16 = {2'b01,lo[2:0]};
  endfunction

  function [5:0] f_clzb32  
  (
    input logic [31:0] src
  );
    
    logic [4:0] hi;
    logic [4:0] lo;

    hi = f_clzb16(src[31:16]);
    lo = f_clzb16(src[15:0]);
    if ((hi[4]==1'b1)&(lo[4]==1'b1))
      f_clzb32 = 6'b10_0000;
    else if (hi[4]==1'b0)
      f_clzb32 = {1'b0,hi};
    else
      f_clzb32 = {2'b01,lo[3:0]};
  endfunction

  // div step: shift and subtract
  function void f_div_step
  (
    input  logic [DIV_WIDTH-1:0] remainder_in,
    input  logic [DIV_WIDTH-1:0] quotient_in,
    input  logic [DIV_WIDTH-1:0] divisor_in,
    output logic [DIV_WIDTH-1:0] remainder_out,
    output logic [DIV_WIDTH-1:0] quotient_out
  );

    logic [DIV_WIDTH  :0] diff;
    logic [DIV_WIDTH-1:0] remainder_tmp;
    
    remainder_tmp = {remainder_in[DIV_WIDTH-2:0],quotient_in[DIV_WIDTH-1]};
    diff = {1'b0,remainder_tmp} - {1'b0,divisor_in};

    if (diff[DIV_WIDTH]) begin
      remainder_out = remainder_tmp;
      quotient_out  = {quotient_in[DIV_WIDTH-2:0],1'b0};
    end
    else begin
      remainder_out = diff[DIV_WIDTH-1:0];
      quotient_out  = {quotient_in[DIV_WIDTH-2:0],1'b1};
    end
  endfunction

endmodule
