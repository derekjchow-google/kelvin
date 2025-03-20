`ifndef LSU_TRANSACTION__SV
`define LSU_TRANSACTION__SV

class lsu_transaction extends uvm_sequence_item;
  /* Tracing info */
  typedef enum {LOAD, STORE} kinds_e;
  kinds_e kind;

  int uop_pc;
  int uop_index;
  bit is_last_uop;
  bit is_indexed;

  /* info about vreg */
  // v0.t
  bit vm;

  // vd/vs3
  bit   data_vreg_valid;
  int   data_vreg_idx;
  eew_e data_vreg_eew;
  int   data_vreg_byte_start;
  int   data_vreg_byte_end;

  // vs2
  bit   vidx_vreg_valid;
  int   vidx_vreg_idx;
  eew_e vidx_vreg_eew;
  int   vidx_vreg_byte_start;
  int   vidx_vreg_byte_end;
  
  int elm_num;

  /* info about load/store address/data */
  bit  lsu_slot_addr_valid;
  int  lsu_slot_addr [`VLENB-1:0]; // byte addr for each byte data
  bit                      lsu_slot_data_valid;
  logic [`VLENB-1:0] [7:0] lsu_slot_data;
  logic [`VLENB-1:0]       lsu_slot_strobe; // byte strobe

  /* uop status */
  bit uop_rx_sent;
  bit uop_done;

  /* LSU delay */
  rand int unsigned rvv2lsu_delay;
  rand int unsigned lsu2rvv_delay;

// Constrain ----------------------------------------------------------
  constraint c_delay {
    rvv2lsu_delay dist {
      0      := 80,
      [1:10] :/ 20
    };
    lsu2rvv_delay dist {
      0      := 80,
      [1:5]  :/ 15,
      [5:20] :/ 5
    };
  }

// Auto Field ---------------------------------------------------------
  `uvm_object_utils_begin(lsu_transaction) 
    `uvm_field_enum(kinds_e,kind,UVM_ALL_ON)
    `uvm_field_int(uop_pc,UVM_ALL_ON)
    `uvm_field_int(uop_index,UVM_ALL_ON)
    `uvm_field_int(is_last_uop,UVM_ALL_ON)
    `uvm_field_int(is_indexed,UVM_ALL_ON)
    
    `uvm_field_int(vm,UVM_ALL_ON)

    `uvm_field_int(data_vreg_idx,UVM_ALL_ON)
    `uvm_field_enum(eew_e,data_vreg_eew,UVM_ALL_ON)
    `uvm_field_int(data_vreg_byte_start,UVM_ALL_ON)
    `uvm_field_int(data_vreg_byte_end  ,UVM_ALL_ON)
    `uvm_field_int(data_vreg_valid,UVM_ALL_ON)

    if(is_indexed) begin
      `uvm_field_int(vidx_vreg_idx  ,UVM_ALL_ON)
      `uvm_field_enum(eew_e, vidx_vreg_eew,UVM_ALL_ON)
      `uvm_field_int(vidx_vreg_byte_start,UVM_ALL_ON)
      `uvm_field_int(vidx_vreg_byte_end  ,UVM_ALL_ON)
      `uvm_field_int(vidx_vreg_valid,UVM_ALL_ON)
    end 
    `uvm_field_int(lsu_slot_addr_valid, UVM_ALL_ON)
    `uvm_field_sarray_int(lsu_slot_addr, UVM_ALL_ON)
    `uvm_field_int(lsu_slot_data_valid, UVM_ALL_ON)
    `uvm_field_int(lsu_slot_data, UVM_ALL_ON)
    `uvm_field_int(lsu_slot_strobe, UVM_ALL_ON)

    `uvm_field_int(uop_rx_sent,UVM_ALL_ON)
    `uvm_field_int(uop_done,UVM_ALL_ON)
    `uvm_field_int(rvv2lsu_delay,UVM_ALL_ON)
    `uvm_field_int(lsu2rvv_delay,UVM_ALL_ON)
  `uvm_object_utils_end

  extern function new(string name = "Trans");
  extern virtual function void do_print(uvm_printer printer);
endclass: lsu_transaction

function lsu_transaction::new(string name = "Trans");
  super.new(name);
  kind = LOAD;
  uop_pc = 0;
  uop_index = 0;
  is_last_uop = 0;
  is_indexed = 0;

  vm = 0;

  data_vreg_valid      = 0;
  data_vreg_idx        = 0;
  data_vreg_eew        = EEW_NONE;
  data_vreg_byte_start = 0;
  data_vreg_byte_end   = 0;

  vidx_vreg_valid      = 0;
  vidx_vreg_idx        = 0;
  vidx_vreg_eew        = EEW_NONE;
  vidx_vreg_byte_start = 0;
  vidx_vreg_byte_end   = 0;

  elm_num = 0;

  lsu_slot_addr_valid = 1'b0;
  foreach(lsu_slot_addr[i]) begin
    lsu_slot_addr[i] = 0;
  end
  lsu_slot_data_valid = 1'b0;
  lsu_slot_data       = 'x;
  lsu_slot_strobe     = 'x;

  uop_rx_sent = 1'b0;
  uop_done = 1'b0;

endfunction: new


function void lsu_transaction::do_print(uvm_printer printer);
  super.do_print(printer);
  foreach(lsu_slot_addr[i])
    printer.print_int(
      $sformatf("lsu_slot_addr[%0d]",i),
      lsu_slot_addr[i],
      32,
      UVM_HEX
    );
endfunction: do_print

`endif // LSU_TRANSACTION__SV
