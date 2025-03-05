// Description:
// 1. rvv_backend_dispatch_opr_byte_type sub-module is for generating byte type for operand(s)
//    a. it is convenient for PU&RT to check if byte data shoud be updated or used for uop(s)

`ifndef HDL_VERILOG_RVV_DESIGN_RVV_SVH
`include "rvv_backend.svh"
`endif

`ifndef RVV_DISPATCH__SVH
`include "rvv_backend_dispatch.svh"
`endif

module rvv_backend_dispatch_opr_byte_type
(
    operand_byte_type,
    uop_info,
    v0_enable
);
// ---parameter definition--------------------------------------------
    localparam VLENB_WIDTH = $clog2(`VLENB);
    localparam logic [`VLENB-1:0][VLENB_WIDTH-1:0] BYTE_INDEX =
        {4'd15, 4'd14, 4'd13, 4'd12, 4'd11, 4'd10, 4'd9, 4'd8, 
         4'd7 , 4'd6 , 4'd5 , 4'd4 , 4'd3 , 4'd2 , 4'd1, 4'd0};
    //localparam logic [`VLENB-1:0][VLENB_WIDTH-1:0] BYTE_INDEX =
    //    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,
    //    16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31};

// ---port definition-------------------------------------------------
    output UOP_OPN_BYTE_TYPE_t operand_byte_type;
    input  UOP_INFO_t          uop_info;
    input  logic [`VLEN-1:0]   v0_enable;

// ---internal signal definition--------------------------------------
    EEW_e                               eew_max;
    logic  [1:0]                        eew_max_shift;

    logic  [1:0]                        vs1_eew_shift;
    logic  [`VSTART_WIDTH-1:0]          uop_vs1_start;
    logic  [`VSTART_WIDTH-1:0]          uop_vs1_end;
    logic  [`VLENB-1:0][`VL_WIDTH-1:0]  vs1_ele_index; // element index
    logic  [`VLENB-1:0]                 vs1_enable, vs1_enable_tmp;
    
    logic  [1:0]                        vs2_eew_shift;
    logic  [`VSTART_WIDTH-1:0]          uop_vs2_start;
    logic  [`VSTART_WIDTH-1:0]          uop_vs2_end;
    logic  [`VLENB-1:0][`VL_WIDTH-1:0]  vs2_ele_index; // element index
    logic  [`VLENB-1:0]                 vs2_enable, vs2_enable_tmp;

    logic  [1:0]                        vd_eew_shift;
    logic  [`VSTART_WIDTH-1:0]          uop_vd_start;
    logic  [`VSTART_WIDTH-1:0]          uop_vd_end;
    logic  [`VLENB-1:0][`VL_WIDTH-1:0]  vd_ele_index; // element index
    logic  [`VLENB-1:0]                 vd_enable, vd_enable_tmp;

// ---code start------------------------------------------------------
    // find eew_max and shift amount
    always_comb begin
      if ((uop_info.vs1_eew==EEW32)||(uop_info.vs2_eew==EEW32)||(uop_info.vd_eew==EEW32)) begin
        eew_max       = EEW32;
        eew_max_shift = 2'h2;
      end
      else if ((uop_info.vs1_eew==EEW16)||(uop_info.vs2_eew==EEW16)||(uop_info.vd_eew==EEW16)) begin
        eew_max       = EEW16;
        eew_max_shift = 2'h1;
      end
      else begin
        eew_max       = EEW8;
        eew_max_shift = 2'h0;
      end
    end

    genvar i;
// for vs1 byte type
    generate
        always_comb begin
            case (uop_info.vs1_eew)
                EEW8:   vs1_eew_shift = 2'h0;
                EEW16:  vs1_eew_shift = 2'h1;
                EEW32:  vs1_eew_shift = 2'h2;
                default:vs1_eew_shift = 2'h0;
            endcase
        end

        always_comb begin
          case({eew_max,uop_info.vs1_eew})
            {EEW32,EEW32},
            {EEW16,EEW16},
            {EEW8,EEW8}: begin
              // regular and narrowing instruction
              uop_vs1_start = uop_info.uop_index << (VLENB_WIDTH - vs1_eew_shift);
              uop_vs1_end = uop_vs1_start + (`VLENB >> eew_max_shift) - 1'b1;
            end
            {EEW32,EEW16},
            {EEW16,EEW8}: begin
              // widening instructio: EEW_vd:EEW_vs = 2:1
              uop_vs1_start = uop_info.uop_index[`UOP_INDEX_WIDTH-1:1] << (VLENB_WIDTH - vs1_eew_shift);
              uop_vs1_end = uop_info.uop_index[0] ? uop_vs1_start + (`VLENB >> (eew_max_shift-1)) - 1'b1 :
                                                    uop_vs1_start + (`VLENB >> eew_max_shift) - 1'b1 ; 
            end
            {EEW32,EEW8}: begin
              // widening instructio: EEW_vd:EEW_vs = 4:1
              uop_vs1_start = uop_info.uop_index[`UOP_INDEX_WIDTH-1:2] << (VLENB_WIDTH - vs1_eew_shift);
              case(uop_info.uop_index[1:0])
                2'd0: uop_vs1_end = uop_vs1_start + `VLENB/4 - 1;  
                2'd1: uop_vs1_end = uop_vs1_start + `VLENB/2 - 1;  
                2'd2: uop_vs1_end = uop_vs1_start + `VLENB*3/4 - 1;  
                2'd3: uop_vs1_end = uop_vs1_start + `VLENB - 1;  
                default: uop_vs1_end = 'b0;  
              endcase
            end
            default: begin
              uop_vs1_start = 'b0;
              uop_vs1_end = 'b0;
            end
          endcase
        end

        assign vs1_enable_tmp  = v0_enable[uop_vs1_start+:`VLENB]; 

        for (i=0; i<`VLENB; i++) begin : gen_vs1_byte_type
            // ele_index = uop_index * (VLEN/vs1_eew) + BYTE_INDEX[MSB:vs1_eew]
            assign vs1_enable[i] = (uop_info.vm || uop_info.ignore_vma) ? 1'b1 : vs1_enable_tmp[BYTE_INDEX[i] >> vs1_eew_shift];
            assign vs1_ele_index[i] = uop_vs1_start + (BYTE_INDEX[i] >> vs1_eew_shift);
            always_comb begin
                if (vs1_ele_index[i] >= uop_info.vl) 
                    operand_byte_type.vs1[i] = uop_info.ignore_vta ? BODY_ACTIVE 
                                                                   : TAIL;
                else if (vs1_ele_index[i] < {1'b0, uop_info.vstart}) 
                    operand_byte_type.vs1[i] = NOT_CHANGE; // prestart
                else if (vs1_ele_index[i] > uop_vd_end) 
                    operand_byte_type.vs1[i] = BODY_INACTIVE;
                else begin 
                    operand_byte_type.vs1[i] = vd_enable[i] ? BODY_ACTIVE
                                                            : BODY_INACTIVE;
                end
            end
        end
    endgenerate

// for vs2 byte type
    generate
        always_comb begin
            case (uop_info.vs2_eew)
                EEW8:   vs2_eew_shift = 2'h0;
                EEW16:  vs2_eew_shift = 2'h1;
                EEW32:  vs2_eew_shift = 2'h2;
                default:vs2_eew_shift = 2'h0;
            endcase
        end

        always_comb begin
          case({eew_max,uop_info.vs2_eew})
            {EEW32,EEW32},
            {EEW16,EEW16},
            {EEW8,EEW8}: begin
              // regular and narrowing instruction
              uop_vs2_start = uop_info.uop_index << (VLENB_WIDTH - vs2_eew_shift);
              uop_vs2_end = uop_vs2_start + (`VLENB >> eew_max_shift) - 1'b1;
            end
            {EEW32,EEW16},
            {EEW16,EEW8}: begin
              // widening instruction: EEW_vd:EEW_vs = 2:1
              uop_vs2_start = uop_info.uop_index[`UOP_INDEX_WIDTH-1:1] << (VLENB_WIDTH - vs2_eew_shift);
              uop_vs2_end = uop_info.uop_index[0] ? uop_vs2_start + (`VLENB >> (eew_max_shift-1)) - 1'b1 :
                                                    uop_vs2_start + (`VLENB >> eew_max_shift) - 1'b1 ; 
            end
            {EEW32,EEW8}: begin
              // widening instruction: EEW_vd:EEW_vs = 4:1
              uop_vs2_start = uop_info.uop_index[`UOP_INDEX_WIDTH-1:2] << (VLENB_WIDTH - vs2_eew_shift);
              case(uop_info.uop_index[1:0])
                2'd0: uop_vs2_end = uop_vs2_start + `VLENB/4 - 1;  
                2'd1: uop_vs2_end = uop_vs2_start + `VLENB/2 - 1;  
                2'd2: uop_vs2_end = uop_vs2_start + `VLENB*3/4 - 1;  
                2'd3: uop_vs2_end = uop_vs2_start + `VLENB - 1;  
                default: uop_vs2_end = 'b0;  
              endcase
            end
            default: begin
              uop_vs2_start = 'b0;
              uop_vs2_end = 'b0;
            end
          endcase
        end

        assign vs2_enable_tmp  = v0_enable[uop_vs2_start+:`VLENB]; 

        for (i=0; i<`VLENB; i++) begin : gen_vs2_byte_type
            // ele_index = uop_index * (VLEN/vs2_eew) + BYTE_INDEX[MSB:vs2_eew]
            assign vs2_enable[i] = (uop_info.vm || uop_info.ignore_vma) ? 1'b1 : vs2_enable_tmp[BYTE_INDEX[i] >> vs2_eew_shift];
            assign vs2_ele_index[i] = uop_vs2_start + (BYTE_INDEX[i] >> vs2_eew_shift);
            always_comb begin
                if (vs2_ele_index[i] >= uop_info.vl) 
                    operand_byte_type.vs2[i] = uop_info.ignore_vta ? BODY_ACTIVE 
                                                                   : TAIL; 
                else if (vs2_ele_index[i] < {1'b0, uop_info.vstart}) 
                    operand_byte_type.vs2[i] = NOT_CHANGE; // prestart
                else if (vs2_ele_index[i] > uop_vd_end) 
                    operand_byte_type.vs2[i] = BODY_INACTIVE;
                else begin 
                    operand_byte_type.vs2[i] = vd_enable[i] ? BODY_ACTIVE
                                                            : BODY_INACTIVE;
                end
            end
        end
    endgenerate

// for vd byte type
    generate
        always_comb begin
            case (uop_info.vd_eew)
                EEW8:   vd_eew_shift = 2'h0;
                EEW16:  vd_eew_shift = 2'h1;
                EEW32:  vd_eew_shift = 2'h2;
                default:vd_eew_shift = 2'h0;
            endcase
        end
        
        always_comb begin
          case({eew_max,uop_info.vd_eew})
            {EEW32,EEW32},
            {EEW16,EEW16},
            {EEW8,EEW8}: begin
              uop_vd_start = uop_info.uop_index << (VLENB_WIDTH - vd_eew_shift);
              uop_vd_end = uop_vd_start + (`VLENB >> eew_max_shift) - 1'b1;
            end
            {EEW32,EEW16},
            {EEW16,EEW8}: begin
              // narrowing instruction: EEW_vd:EEW_vs = 1:2
              uop_vd_start = uop_info.uop_index[`UOP_INDEX_WIDTH-1:1] << (VLENB_WIDTH - vd_eew_shift);
              uop_vd_end = uop_info.uop_index[0] ? uop_vd_start + (`VLENB >> (eew_max_shift-1)) - 1'b1 :
                                                   uop_vd_start + (`VLENB >> eew_max_shift) - 1'b1 ; 
            end
            default: begin
              uop_vd_start = 'b0; 
              uop_vd_end = 'b0;
            end
          endcase
        end

        assign vd_enable_tmp  = v0_enable[uop_vd_start+:`VLENB]; 

        for (i=0; i<`VLENB; i++) begin : gen_vd_byte_type
            // ele_index = uop_index * (VLEN/vd_eew) + BYTE_INDEX[MSB:vd_eew]
            assign vd_enable[i] = (uop_info.vm || uop_info.ignore_vma) ? 1'b1 : vd_enable_tmp[BYTE_INDEX[i] >> vd_eew_shift];
            assign vd_ele_index[i] = uop_vd_start + (BYTE_INDEX[i] >> vd_eew_shift);
            always_comb begin
                if (vd_ele_index[i] >= uop_info.vl) 
                    operand_byte_type.vd[i] = uop_info.ignore_vta ? BODY_ACTIVE 
                                                                  : TAIL;       
                else if (vd_ele_index[i] < {1'b0, uop_info.vstart}) 
                    operand_byte_type.vd[i] = NOT_CHANGE;     // prestart
                else if (vd_ele_index[i] > uop_vd_end) 
                    operand_byte_type.vd[i] = BODY_INACTIVE;
                else begin 
                    operand_byte_type.vd[i] = vd_enable[i] ? BODY_ACTIVE
                                                           : BODY_INACTIVE;
                end
            end
        end
    endgenerate

endmodule
