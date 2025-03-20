// description
// the multi_fifo is a sync fifo mmodule with some specified features
// features:
//    1. multi push at a time
//    2. multi pop at a time
//    3. parameterize push/pop number with arbitrary value
//    4. output all fifo data and sort them based on read pointer
//    5. output write pointer and read pointer
// constraints:
//    1. do not support push&pop when fifo is full

module multi_fifo
(
  // global
  clk,
  rst_n,
  // push side
  push,
  datain,
  full,
  almost_full,
  // pop side
  pop,
  dataout,
  empty,
  almost_empty,
  // fifo info
  clear,
  fifo_data,
  wptr,
  rptr,
  entry_count
);
// ---parameter definition--------------------------------------------
  parameter type T      = logic [7:0];  // data structure
  parameter M           = 4;            // push signal width
  parameter N           = 4;            // pop signal width
  parameter DEPTH       = 16;           // fifo depth
  parameter POP_CLEAR   = 1'b0;         // clear data once pop
  parameter ASYNC_RSTN  = 1'b0;         // reset data
  parameter CHAOS_PUSH  = 1'b0;         // support push data disorderly
  parameter DATAOUT_REG = 1'b0;         // dataout signal register output. 
  
  localparam DEPTH_BITS = $clog2(DEPTH);

// ---port definition-------------------------------------------------
  input   logic                   clk;
  input   logic                   rst_n;
  input   logic [M-1:0]           push;         // M bits indicates M push operation(s). 
  input   T     [M-1:0]           datain;
  output  logic                   full;
  output  logic [M-1:0]           almost_full;  // almost_full[0]==1 - full
                                                // almost_full[1]==1 - The remaining free spaces <=1
                                                // almost_full[M-1]==1 - The remaining free spaces <=M-1
  input   logic [N-1:0]           pop;          // N bits indicates N pop operation(s).
  output  T     [N-1:0]           dataout;
  output  logic                   empty;
  output  logic [N-1:0]           almost_empty; // almost_empty[0]==1 - empty
                                                // almost_empty[1]==1 - The remaining quantity of valid data <= 1
                                                // almost_empty[N-1]==1 - The remaining quantity of valid data <= N-1
  input   logic                   clear;
  output  T     [DEPTH-1:0]       fifo_data;    // sort based on rptr
  output  logic [DEPTH_BITS-1:0]  wptr;         // write pointer
  output  logic [DEPTH_BITS-1:0]  rptr;         // read pointer
  output  logic [DEPTH_BITS  :0]  entry_count;  // the number of occupied entry.

// ---internal signal definition--------------------------------------
  T mem[DEPTH-1:0];

  logic                           entry_count_en; 
  logic         [DEPTH_BITS  :0]  next_entry_count;
  logic         [DEPTH_BITS  :0]  push_count;
  logic         [DEPTH_BITS  :0]  pop_count;
  logic         [DEPTH_BITS-1:0]  next_wptr;
  logic         [DEPTH_BITS-1:0]  next_rptr;
  logic         [DEPTH_BITS-1:0]  wind_rptr [DEPTH-1:0];
  logic         [DEPTH_BITS-1:0]  wind_wptr [DEPTH-1:0];

  logic         [M-1:0]           push_seq;
  T             [M-1:0]           datain_seq;
// ---code start------------------------------------------------------
  genvar  i;  
  integer l,k;
  
  // fifo status
  // valid data count
  assign next_entry_count = entry_count + push_count - pop_count;
  assign entry_count_en = (|push) | (|pop);
  cdffr #(.T(logic[DEPTH_BITS:0])) u_entry_count_reg (.q(entry_count), .c(clear), .e(entry_count_en), .d(next_entry_count), .clk(clk), .rst_n(rst_n));

  // full
  assign full = (entry_count == DEPTH);
  assign almost_full[0] = full;
  generate
    for (i=1; i<M; i++) begin : gen_almost_full
      assign almost_full[i] = (entry_count + i >= DEPTH);
    end
  endgenerate

  // empty
  assign empty = (entry_count == '0);
  assign almost_empty[0] = empty;
  generate
    for (i=1; i<N; i++) begin : gen_almost_empty
      assign almost_empty[i] = (entry_count <= i);
    end
  endgenerate

  generate
    for (i=0; i<DEPTH; i++) begin : gen_fifo_data
      assign fifo_data[i] = mem[wind_rptr[i]];
    end
  endgenerate

  // wind back rptr/wptr
  generate
    for (i=0; i<DEPTH; i++) begin : gen_wind_ptr
      assign wind_rptr[i] = rptr+i;
      assign wind_wptr[i] = wptr+i;
    end
  endgenerate
  // dataout
  always_comb begin
    pop_count = pop[0];
    for (int j=1; j<N; j++) pop_count = pop_count + pop[j];
  end
  
  assign next_rptr = rptr + pop_count;
  cdffr #(.T(logic[DEPTH_BITS-1:0])) u_rptr_reg (.q(rptr), .c(clear), .e(|pop), .d(next_rptr), .clk(clk), .rst_n(rst_n));
  
  generate
    if(DATAOUT_REG) begin
      logic [DEPTH_BITS:0] remain_count;
      assign remain_count = entry_count-pop_count;

      if (CHAOS_PUSH) begin
        for (i=0; i<N; i++) begin : gen_dataout
          always_ff @(posedge clk) begin
            if ((i<remain_count)&((|pop)|(|push_seq)))
              dataout[i] <= mem[next_rptr+i]; 
            else if ((push_seq[i-remain_count]&((i-remain_count)<M))&((|pop)|(|push_seq)))
              dataout[i] <= datain_seq[i-remain_count];
          end
        end
      end else begin
        for (i=0; i<N; i++) begin : gen_dataout
          always_ff @(posedge clk) begin
            if ((i<remain_count)&((|pop)|(|push)))
              dataout[i] <= mem[next_rptr+i]; 
            else if ((push[i-remain_count]&((i-remain_count)<M))&((|pop)|(|push)))
              dataout[i] <= datain[i-remain_count];
          end
        end
      end
    end
    else begin
      for (i=0; i<N; i++) begin : gen_dataout
        assign dataout[i] = mem[wind_rptr[i]];
      end
    end
  endgenerate

  // datain
  always_comb begin
    push_count = push[0];
    for (int j=1; j<M; j++) push_count = push_count + push[j];
  end

  assign next_wptr = wptr + push_count;
  cdffr #(.T(logic[DEPTH_BITS-1:0])) u_wptr_reg (.q(wptr), .c(clear), .e(|push), .d(next_wptr), .clk(clk), .rst_n(rst_n));

  generate
    if (CHAOS_PUSH) begin
      always_comb begin
        push_seq = '0;
        datain_seq = '0;
        l = 0;
        for (k=0; k<M; k++) begin
          if (push[k]) begin
            push_seq[l] = 1'b1;
            datain_seq[l] = datain[k];
            l++;
          end
        end
      end
    end else begin
      assign push_seq = push;
      assign datain_seq = datain;
    end
  endgenerate

  generate
  if (ASYNC_RSTN)
    if (POP_CLEAR) begin
      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
          for (int j=0; j<DEPTH; j++) begin 
            mem[j] <= '0;
          end
        else begin
          if (push_seq[0] && !full) mem[wptr] <= datain_seq[0];
          for (int j=1; j<M; j++) begin
            if (push_seq[j] && !almost_full[j]) mem[wind_wptr[j]] <= datain_seq[j];
          end
  
          if (clear) begin
            for (int j=0; j<DEPTH; j++) begin 
              mem[j] <= '0;
            end
          end else begin
            for (int j=0; j<N; j++) begin
              if (pop[j]) mem[wind_rptr[j]] <= '0;
            end
          end
        end
      end
    end else begin
      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
          for (int j=0; j<DEPTH; j++) begin 
            mem[j] <= '0;
          end
        else begin
          if (push_seq[0] && !full) mem[wptr] <= datain_seq[0];
          for (int j=1; j<M; j++) begin
            if (push_seq[j] && !almost_full[j]) mem[wind_wptr[j]] <= datain_seq[j];
          end
        end
      end
    end
  else
    if (POP_CLEAR) begin
      always_ff @(posedge clk) begin
        if (push_seq[0] && !full) mem[wptr] <= datain_seq[0];
        for (int j=1; j<M; j++) begin
          if (push_seq[j] && !almost_full[j]) mem[wind_wptr[j]] <= datain_seq[j];
        end

        if (clear) begin
          for (int j=0; j<DEPTH; j++) begin 
            mem[j] <= '0;
          end
        end else begin
          for (int j=0; j<N; j++) begin
            if (pop[j]) mem[wind_rptr[j]] <= '0;
          end
        end
      end
    end else begin
      always_ff @(posedge clk) begin
        if (push_seq[0] && !full) mem[wptr] <= datain_seq[0];
        for (int j=1; j<M; j++) begin
          if (push_seq[j] && !almost_full[j]) mem[wind_wptr[j]] <= datain_seq[j];
        end
      end
    end
  endgenerate

  `ifdef ASSERT_ON
    // test for overflow
      assert property (@(posedge clk) disable iff (!rst_n) not ( push_seq[0] && full))
        else $error("MULTI_FIFO: overflow of fifo when push_seq[0] and full");
      generate
        for (i=1; i<M; i++) begin
          assert property (@(posedge clk) disable iff (!rst_n) not ( push_seq[i] && almost_full[i]))
            else $error("MULTI_FIFO: overflow of fifo when push_seq[%d] and almost_full[%d]", i, i);
        end
      endgenerate
    // test for underflow
      assert property (@(posedge clk) disable iff (!rst_n) not ( pop[0] && empty))
        else $error("MULTI_FIFO: underflow of fifo when pop[0] and empty");
      generate
        for (i=1; i<N; i++) begin
          assert property (@(posedge clk) disable iff (!rst_n) not ( pop[i] && almost_empty[i]))
            else $error("MULTI_FIFO: underflow of fifo when pop[%d] and almost_empty[%d]", i, i);
        end
      endgenerate
  `endif

endmodule
