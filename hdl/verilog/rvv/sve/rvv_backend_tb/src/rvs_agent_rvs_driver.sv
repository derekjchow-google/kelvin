`ifndef RVS_DRIVER__SV
`define RVS_DRIVER__SV

`include "rvv_backend_define.svh"
`include "rvv_backend.svh"
typedef class rvs_transaction;
typedef class rvs_driver;

class rvs_driver extends uvm_driver # (rvs_transaction);

  uvm_analysis_port #(rvs_transaction) inst_ap; 
  
  typedef virtual rvs_interface v_if1; 
  typedef virtual vrf_interface v_if3; 
  v_if1 rvs_if;
  v_if3 vrf_if;
  
  int             inst_tx_queue_depth = 4;
  rvs_transaction inst_tx_queue[$];
  RVVCmd          inst     [`ISSUE_LANE-1:0];
  logic           inst_vld [`ISSUE_LANE-1:0];

  bit             single_inst_mode = 0; 
  int             skip_fetch_cnt = 0;

  extern function new(string name = "rvs_driver", uvm_component parent = null); 
 
  `uvm_component_utils_begin(rvs_driver)
  `uvm_component_utils_end

  extern virtual function void build_phase(uvm_phase phase);
  extern virtual function void connect_phase(uvm_phase phase);
  extern virtual task reset_phase(uvm_phase phase);
  extern virtual task configure_phase(uvm_phase phase);
  extern virtual task run_phase(uvm_phase phase);
  extern protected virtual task tx_driver();
  extern protected virtual task inst_manage();

  extern protected virtual task rx_driver();

endclass: rvs_driver

function rvs_driver::new(string name = "rvs_driver", uvm_component parent = null);
  super.new(name, parent);
endfunction: new

function void rvs_driver::build_phase(uvm_phase phase);
  super.build_phase(phase);
  inst_ap = new("inst_ap", this);
  if(uvm_config_db#(int)::get(this, "", "inst_tx_queue_depth", inst_tx_queue_depth)) begin
    `uvm_info(get_type_name(), $sformatf("Depth of instruction queue in rvs_driver is set to %0d.", inst_tx_queue_depth), UVM_LOW)
  end
  if(uvm_config_db#(int)::get(this, "", "single_inst_mode", single_inst_mode)) begin
    `uvm_info(get_type_name(), $sformatf("single_inst_mode of rvs_driver is set to %0d.",single_inst_mode), UVM_LOW)
  end
endfunction: build_phase

function void rvs_driver::connect_phase(uvm_phase phase);
  super.connect_phase(phase);
  if(!uvm_config_db#(v_if1)::get(this, "", "rvs_if", rvs_if))
    `uvm_fatal(get_type_name(), "Fail to get rvs_if!")
  if(!uvm_config_db#(v_if3)::get(this, "", "vrf_if", vrf_if))
    `uvm_fatal(get_type_name(), "Fail to get vrf_if!")
endfunction: connect_phase

task rvs_driver::reset_phase(uvm_phase phase);
  phase.raise_objection( .obj( this ) );
  while(!rvs_if.rst_n) begin
    for(int i=0; i<`ISSUE_LANE; i++) begin
      inst[i] = '0;
      inst_vld[i] = '0;
    end
    //Reset DUT
    for(int i=0; i<`ISSUE_LANE; i++) begin
      rvs_if.insts_rvs2cq[i]         <= '0;
      rvs_if.insts_valid_rvs2cq[i]   <= '0;
    end
    for(int i=0; i<`NUM_RT_UOP; i++) begin
      rvs_if.rt_xrf_ready_rvs2rvv[i] <= '0;
    end
    @(posedge rvs_if.clk);
  end
  phase.drop_objection( .obj( this ) );

endtask: reset_phase

task rvs_driver::configure_phase(uvm_phase phase);
  super.configure_phase(phase);
endtask:configure_phase

task rvs_driver::run_phase(uvm_phase phase);
  super.run_phase(phase);
  fork 
    tx_driver();
    rx_driver();
  join
endtask: run_phase

task rvs_driver::inst_manage();
  rvs_transaction tr;
  tr = new();
  
  for(int i=0; i<`ISSUE_LANE; i++) begin
    if((rvs_if.insts_ready_cq2rvs[i]===1'b1) && inst_vld[i]) begin
      tr = inst_tx_queue.pop_front();
    end
  end

  while (inst_tx_queue.size() < inst_tx_queue_depth) begin
    if(skip_fetch_cnt != 0) begin 
      skip_fetch_cnt--;
      break;
    end
    //seq_item_port.get_next_item(tr);
    seq_item_port.try_next_item(tr);
    if(tr != null) begin
      `uvm_info(get_type_name(), $sformatf("Get item from rvs_sqr:\n%s",tr.sprint()),UVM_HIGH)
      inst_tx_queue.push_back(tr);
      `uvm_info(get_type_name(), $sformatf("Send transaction to rvs_mon:\n%s",tr.sprint()),UVM_HIGH)
      inst_ap.write(tr);
      seq_item_port.item_done(); 
      if(single_inst_mode) begin
        skip_fetch_cnt = 5;
      end else begin
        // skip_fetch_cnt = $urandom();
        skip_fetch_cnt = 0; 
      end
    end else begin
      break;
    end
  end

  for(int i=0; i<`ISSUE_LANE; i++) begin
    if(i < inst_tx_queue.size()) begin
      `uvm_info(get_type_name(), $sformatf("Assign to port inst[%d]",i),UVM_HIGH)
      inst[i].inst_pc               = inst_tx_queue[i].pc;
      assert($cast(inst[i].opcode, inst_tx_queue[i].bin_inst[6:5]));
      inst[i].bits                  = inst_tx_queue[i].bin_inst[31:7];
      inst[i].rs1                   = inst_tx_queue[i].rs_data;
      inst[i].arch_state.vl         = inst_tx_queue[i].vl;
      inst[i].arch_state.vstart     = inst_tx_queue[i].vstart;
      assert($cast(inst[i].arch_state.xrm, inst_tx_queue[i].vxrm));
      inst[i].arch_state.ma         = inst_tx_queue[i].vtype.vma;
      inst[i].arch_state.ta         = inst_tx_queue[i].vtype.vta;
      assert($cast(inst[i].arch_state.sew, inst_tx_queue[i].vtype.vsew));
      assert($cast(inst[i].arch_state.lmul, inst_tx_queue[i].vtype.vlmul));
      inst_vld[i] = 1'b1;
    end else begin
      inst[i] = inst[i];
      inst_vld[i] = 1'b0;
    end 
  end

endtask: inst_manage

task rvs_driver::tx_driver();
  rvs_transaction tr;
  tr = new();
  forever begin
    @(posedge rvs_if.clk);
    inst_manage();
    for(int i=0; i<`ISSUE_LANE; i++) begin
      rvs_if.insts_rvs2cq[i]         <= inst[i];
      rvs_if.insts_valid_rvs2cq[i]   <= inst_vld[i];
    end
  end
endtask: tx_driver

task rvs_driver::rx_driver();
  forever begin
    for(int i=0; i<`NUM_RT_UOP; i++) begin
      rvs_if.rt_xrf_ready_rvs2rvv[i] <= '1;
    end
    @(posedge rvs_if.clk);
  end
endtask: rx_driver
`endif // RVS_DRIVER__SV


