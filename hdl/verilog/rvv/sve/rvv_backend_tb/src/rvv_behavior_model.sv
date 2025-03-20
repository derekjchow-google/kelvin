`ifndef RVV_BEHAVIOR_MODEL
`define RVV_BEHAVIOR_MODEL

`include "rvv_backend.svh"

  `uvm_analysis_imp_decl(_inst)

  typedef logic [0:0]  sew1_t;
  typedef logic [7:0]  sew8_t;
  typedef logic [15:0] sew16_t;
  typedef logic [31:0] sew32_t;
  typedef logic [31:0] sew_max_t;

typedef class alu_base;
typedef class alu_processor;
typedef class lsu_processor;
typedef class pmt_processor;
typedef class rdt_processor;

class rvv_behavior_model extends uvm_component;
  typedef virtual rvs_interface v_if1;
  v_if1 rvs_if;  
  typedef virtual vrf_interface v_if3;
  v_if3 vrf_if;  

  bit ill_inst_en = 0;
  bit all_one_for_agn = 0;

  uvm_analysis_imp_inst #(rvs_transaction,rvv_behavior_model) inst_imp; 
  uvm_analysis_port #(rvs_transaction) rt_ap; 
  uvm_analysis_port #(vrf_transaction) vrf_ap;

  agnostic_e        vma;
  agnostic_e        vta;
  sew_e             vsew; 
  lmul_e            vlmul;
  logic [`XLEN-1:0] vl;
  logic [`XLEN-1:0] vstart;
  vxrm_e            vxrm;
  logic [`XLEN-1:0] vxsat;  
  logic             vxsat_valid;
  xrf_t [31:0] xrf;
  vrf_t [31:0] vrf;
  vrf_t [31:0] vrf_delay;
  vrf_t [31:0] vrf_temp;
  vrf_t [31:0] vrf_bit_strobe_temp;
  vrf_byte_t [31:0] vrf_byte_strobe_temp;

  logic [`XLEN-1:0] vlmax;
  logic [`XLEN-1:0] imm_data;

  byte mem[int unsigned];
  rvs_transaction inst_queue [$];


  int total_inst = 0;
  int executed_inst = 0;
  pmt_processor pmt_inst;
  rdt_processor rdt_inst;
      
  `uvm_component_utils(rvv_behavior_model)

  extern function new(string name = "rvv_behavior_model", uvm_component parent);
  extern virtual function void build_phase(uvm_phase phase);
  extern virtual function void connect_phase(uvm_phase phase);
  extern virtual task reset_phase(uvm_phase phase);
  extern virtual task main_phase(uvm_phase phase);
  extern virtual function void final_phase(uvm_phase phase);

  extern virtual task rx_mdl();
  extern virtual task tx_mdl();
  extern virtual task vrf_mdl();

  extern function logic [31:0] elm_fetch(oprand_type_e reg_type, int reg_idx, int elm_idx, int eew);
  extern task elm_writeback(logic [31:0] result, oprand_type_e reg_type, int reg_idx, int elm_idx, int eew);

  // imp task
  extern virtual function void write_inst(rvs_transaction inst_tr);

endclass : rvv_behavior_model

  function rvv_behavior_model::new(string name = "rvv_behavior_model", uvm_component parent);
    super.new(name, parent);
    pmt_inst = new();// create a pmt inst.
    rdt_inst = new();// create a rdt inst.
  endfunction : new

  function void rvv_behavior_model::build_phase(uvm_phase phase);
    super.build_phase(phase);
    inst_imp = new("inst_imp", this);
    rt_ap = new("rt_ap", this);
    vrf_ap = new("vrf_ap", this);
  endfunction : build_phase 

  function void rvv_behavior_model::connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    if(!uvm_config_db#(v_if1)::get(this, "", "rvs_if", rvs_if)) begin
      `uvm_fatal("MDL/NOVIF", "No virtual interface specified for this agent instance")
    end
    if(!uvm_config_db#(v_if3)::get(this, "", "vrf_if", vrf_if)) begin
      `uvm_fatal("MDL/NOVIF", "No virtual interface specified for this agent instance")
    end
    if(uvm_config_db#(bit)::get(this, "", "ill_inst_en", ill_inst_en))begin
      if(ill_inst_en) `uvm_info(get_type_name(), "Enable operating illegal instruction in reference model!", UVM_LOW)
    end
    if(uvm_config_db#(bit)::get(this, "", "all_one_for_agn", all_one_for_agn))begin
      if(all_one_for_agn) `uvm_info(get_type_name(), "Enable overwriting agnostic emelements with 1s in reference model!", UVM_LOW)
    end
  endfunction:connect_phase

  task rvv_behavior_model::reset_phase(uvm_phase phase);
    phase.raise_objection( .obj( this ) );
    while(!rvs_if.rst_n) begin
      vma         = '0;
      vta         = '0;
      vsew        = '0;
      vlmul       = '0;
      vl          = '0;
      vstart      = '0;
      vxrm        = RNU;
      vxsat       = '0;
      vxsat_valid = '0;
      for(int i=0; i<32; i++) begin
        vrf[i] = '0;
        xrf[i] = '0;
      end 
      @(posedge rvs_if.clk);
      `uvm_info("RESET_PHASE", "This is reset_phase!", UVM_LOW)
    end
    phase.drop_objection( .obj( this ) );
  endtask: reset_phase

  task rvv_behavior_model::main_phase(uvm_phase phase);
    super.main_phase(phase);
    fork
      // rx_mdl();
      tx_mdl();
      vrf_mdl();
    join 
  endtask : main_phase 

  function void rvv_behavior_model::final_phase(uvm_phase phase);
    super.final_phase(phase);
    if(inst_queue.size()>0) begin
      `uvm_error("FINAL_CHECK", "inst_queue in MDL wasn't empty!")
      foreach(inst_queue[idx]) begin
        `uvm_error("FINAL_CHECK",inst_queue[idx].sprint())
      end
    end
    uvm_config_db#(int)::set(uvm_root::get(), "", "mdl_total_inst", this.total_inst);
    uvm_config_db#(int)::set(uvm_root::get(), "", "mdl_excuted_inst", this.executed_inst);
    `uvm_info("FINAL_CHECK", $sformatf("MDL total accepted inst: %0d, executed inst: %0d, discarded %.2f%%", 
                                        this.total_inst, this.executed_inst, real'(this.total_inst - this.executed_inst)*100.0/real'(this.total_inst)), UVM_NONE)
  endfunction: final_phase 

  function void rvv_behavior_model::write_inst(rvs_transaction inst_tr);
    `uvm_info("MDL", "get a inst", UVM_HIGH)
    `uvm_info("MDL", inst_tr.sprint(), UVM_HIGH)
    inst_queue.push_back(inst_tr);
    this.total_inst++;
  endfunction

  task rvv_behavior_model::rx_mdl();
  endtask: rx_mdl

  task rvv_behavior_model::tx_mdl();
    bit fraction_lmul;
    int  eew;
    real emul;
    int elm_idx_max;
    int evl;
  
    int pc;

    bit is_widen_inst;
    bit is_widen_vs2_inst;
    bit is_narrow_inst;
    bit is_mask_producing_inst;
    bit is_mask_compare_inst;
    bit is_carry_produce_inst;
    bit is_reduction_inst;
    bit use_vm_to_cal;
    bit is_permutation_inst;

    bit vm;

    int dest_eew; real dest_emul;
    int src0_eew; real src0_emul;
    int src1_eew; real src1_emul;
    int src2_eew; real src2_emul;
    int src3_eew; real src3_emul;
    int dest_reg_idx_base, dest_reg_idx, dest_reg_elm_idx;
    int src0_reg_idx_base, src0_reg_idx, src0_reg_elm_idx;
    int src1_reg_idx_base, src1_reg_idx, src1_reg_elm_idx;
    int src2_reg_idx_base, src2_reg_idx, src2_reg_elm_idx;
    int src3_reg_idx_base, src3_reg_idx, src3_reg_elm_idx;

    logic [31:0] dest;
    logic [31:0] src0;
    logic [31:0] src1;
    logic [31:0] src2;
    logic [31:0] src3;

    logic [`NUM_RT_UOP-1:0] rt_uop;
    logic [`NUM_RT_UOP-1:0] rt_last_uop;
    rvs_transaction inst_tr;
    rvs_transaction rt_tr;

    alu_base alu_handler;
    alu_processor #( sew8_t,  sew8_t,  sew8_t) alu_08_08_08 = new();
    alu_processor #(sew16_t, sew16_t, sew16_t) alu_16_16_16 = new();
    alu_processor #(sew32_t, sew32_t, sew32_t) alu_32_32_32 = new();
    // widen                                                  
    alu_processor #(sew16_t,  sew8_t,  sew8_t) alu_16_08_08 = new();
    alu_processor #(sew16_t, sew16_t,  sew8_t) alu_16_16_08 = new();
    alu_processor #(sew32_t, sew16_t, sew16_t) alu_32_16_16 = new();
    alu_processor #(sew32_t, sew32_t, sew16_t) alu_32_32_16 = new();
    // ext                                                     
    alu_processor #(sew16_t,  sew8_t, sew32_t) alu_16_08_32 = new();
    alu_processor #(sew16_t,  sew8_t, sew16_t) alu_16_08_16 = new();
    alu_processor #(sew32_t, sew16_t, sew32_t) alu_32_16_32 = new();
    alu_processor #(sew32_t, sew16_t,  sew8_t) alu_32_16_08 = new();
    alu_processor #(sew32_t,  sew8_t, sew32_t) alu_32_08_32 = new();
    alu_processor #(sew32_t,  sew8_t, sew16_t) alu_32_08_16 = new();
    alu_processor #(sew32_t,  sew8_t,  sew8_t) alu_32_08_08 = new();
    // narrow                                               
    alu_processor #( sew8_t, sew16_t,  sew8_t) alu_08_16_08 = new();
    alu_processor #(sew16_t, sew32_t, sew16_t) alu_16_32_16 = new();
    // massk logic                                          
    alu_processor #( sew1_t,  sew1_t,  sew1_t) alu_01_01_01 = new();
    alu_processor #( sew1_t,  sew8_t,  sew8_t) alu_01_08_08 = new();
    alu_processor #( sew1_t, sew16_t, sew16_t) alu_01_16_16 = new();
    alu_processor #( sew1_t, sew32_t, sew32_t) alu_01_32_32 = new();
    // viota
    alu_processor #( sew8_t,  sew1_t,  sew1_t) alu_08_01_01 = new();
    alu_processor #(sew16_t,  sew1_t,  sew1_t) alu_16_01_01 = new();
    alu_processor #(sew32_t,  sew1_t,  sew1_t) alu_32_01_01 = new();

    lsu_processor lsu_proc = new();

    forever begin
      @(posedge rvs_if.clk);
      if(rvs_if.rst_n) begin
      rt_uop = rvs_if.rt_uop;
      rt_last_uop = rvs_if.rt_last_uop;
      while(|rt_last_uop) begin
      if(rt_last_uop[0]) begin
        // --------------------------------------------------
        // 0. Get inst and update VCSR
        if(inst_queue.size()>0) begin
          inst_tr = new("inst_tr");
          inst_tr     = inst_queue.pop_front();
          vma         = inst_tr.vma;
          vta         = inst_tr.vta;
          vsew        = inst_tr.vsew;
          vlmul       = inst_tr.vlmul;
          vl          = inst_tr.vl;
          vstart      = inst_tr.vstart;
          vxrm        = inst_tr.vxrm;
          vxsat       = '0;
          vxsat_valid = '0;

          // init
          is_widen_inst           = 0;
          is_widen_vs2_inst       = 0;
          is_narrow_inst          = 0;
          is_mask_producing_inst  = 0;
          is_mask_compare_inst    = 0;
          is_carry_produce_inst   = 0;
          is_reduction_inst       = 0;
          use_vm_to_cal           = 0;
          is_permutation_inst     = 0;

          `uvm_info("MDL",$sformatf("Start calculation:\n%s",inst_tr.sprint()),UVM_LOW)
        end else begin
          `uvm_error(get_type_name(), "Pop inst_queue while empty.")
          break;
        end
        // 0.2 Calculate & decode for this instr
        eew = 8 << vsew;
        fraction_lmul = vlmul[2];
        emul = 2.0 ** $signed(vlmul);
        vlmax       = emul * `VLEN / eew;
        elm_idx_max = fraction_lmul ?        `VLEN / eew: 
                                      emul * `VLEN / eew;
        evl = inst_tr.vl;
        `uvm_info("MDL", $sformatf("Get eew=%0d, emul=%.2f, vlmax=%0d, elm_idx_max=%0d",eew,emul,vlmax,elm_idx_max), UVM_HIGH)

        // --------------------------------------------------
        // 1. Decode - Get eew & emul
        pc = inst_tr.pc;
        is_widen_inst           = inst_tr.inst_type == ALU &&  (inst_tr.alu_inst inside {VWADDU, VWADD, VWADDU_W, VWADD_W, VWSUBU, VWSUB, VWSUBU_W, VWSUB_W, 
                                                                                         VWMUL, VWMULU, VWMULSU, VWMACCU, VWMACC, VWMACCUS, VWMACCSU});
        is_widen_vs2_inst       = inst_tr.inst_type == ALU &&  (inst_tr.alu_inst inside {VWADD_W, VWADDU_W, VWSUBU_W, VWSUB_W});
        is_narrow_inst          = inst_tr.inst_type == ALU &&  (inst_tr.alu_inst inside {VNSRL, VNSRA, VNCLIPU, VNCLIP});
        is_mask_producing_inst  = inst_tr.inst_type == ALU && ((inst_tr.alu_inst inside {VMAND, VMOR, VMXOR, VMORN, VMNAND, VMNOR, VMANDN, VMXNOR,
                                                                                         VMADC, VMSBC, 
                                                                                         VMSEQ, VMSNE, VMSLTU, VMSLT, VMSLEU, VMSLE, VMSGTU, VMSGT}) ||
                                                               (inst_tr.alu_inst inside {VMUNARY0} && inst_tr.src1_idx inside {VMSBF, VMSOF, VMSIF}));
        is_mask_compare_inst    = inst_tr.inst_type == ALU &&  (inst_tr.alu_inst inside {VMSEQ, VMSNE, VMSLTU, VMSLT, VMSLEU, VMSLE, VMSGTU, VMSGT});
        is_carry_produce_inst   = inst_tr.inst_type == ALU &&  (inst_tr.alu_inst inside {VMADC, VMSBC});
        is_reduction_inst       = inst_tr.inst_type == ALU &&  (inst_tr.alu_inst inside {VREDSUM, VREDAND, VREDOR, VREDXOR, 
                                                                                         VREDMINU,VREDMIN,VREDMAXU,VREDMAX,
                                                                                         VWREDSUMU, VWREDSUM});
        is_permutation_inst     = inst_tr.inst_type == ALU && ((inst_tr.alu_inst inside {VSMUL_VMVNRR} && inst_tr.alu_type == OPIVI && inst_tr.vm == 1) ||
                                                               (inst_tr.alu_inst inside {VCOMPRESS}) ||
                                                               (inst_tr.alu_inst inside {VSLIDEUP_RGATHEREI16, VSLIDE1UP, VSLIDE1DOWN, VSLIDEDOWN, VRGATHER}) ||
                                                               (inst_tr.alu_inst inside {VWXUNARY0} && inst_tr.src1_type == FUNC && inst_tr.src1_idx inside {VMV_X_S}) ||
                                                               (inst_tr.alu_inst inside {VWXUNARY0} && inst_tr.src2_type == FUNC && inst_tr.src2_idx inside {VMV_X_S}));
        use_vm_to_cal = inst_tr.use_vm_to_cal;
            
        `uvm_info(get_type_name(), $sformatf("permutation instruction dectect inst_tr.alu_inst = %0x , inst_tr.type = %0x \n", inst_tr.alu_inst,inst_tr.inst_type), UVM_LOW)

        if(is_permutation_inst) begin
            `uvm_info(get_type_name(), $sformatf("permutation instruction dectect\n"), UVM_LOW)
        end


        // 1.0 illegal inst check
        // vstart
        if(inst_tr.vstart >= inst_tr.vl && !(inst_tr.inst_type == ALU && inst_tr.alu_inst inside {VSMUL_VMVNRR} && inst_tr.alu_type == OPIVI)) begin
          if(inst_tr.dest_type == XRF || inst_tr.vl == 0) begin
          end else begin
            `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: ignored since vstart(%0d) >= vl(%0d).", pc, inst_tr.vstart, inst_tr.vl))
            continue;
          end
        end
        if(inst_tr.vstart >= vlmax && !(inst_tr.inst_type == ALU && inst_tr.alu_inst inside {VSMUL_VMVNRR} && inst_tr.alu_type == OPIVI || 
                                        inst_tr.inst_type == ALU && inst_tr.alu_inst inside {VMAND, VMOR, VMXOR, VMORN, VMNAND, VMNOR, VMANDN, VMXNOR})) begin
          if(inst_tr.dest_type == XRF || inst_tr.vl == 0) begin
          end else begin
            `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: ignored since vstart(%0d) >= vlmax(%0d).", pc, inst_tr.vstart, vlmax))
            continue;
          end
        end
        if(inst_tr.vl > vlmax && !(inst_tr.alu_inst inside {VSMUL_VMVNRR} && inst_tr.alu_type == OPIVI || 
                                   inst_tr.alu_inst inside {VMAND, VMOR, VMXOR, VMORN, VMNAND, VMNOR, VMANDN, VMXNOR})) begin
          if(inst_tr.dest_type == XRF || inst_tr.vl == 0) begin
          end else begin
            `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: ignored since vl(%0d) >= vlmax(%0d).", pc, inst_tr.vl, vlmax))
            continue;
          end
        end

        // OPxxx reserve check
        if(inst_tr.inst_type == ALU && inst_tr.alu_type == OPIVV && inst_tr.alu_inst inside {VRSUB, VMSGTU, VMSGT}) begin
          `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: OPIVV of %0s is ignored.",pc,inst_tr.alu_inst.name()))
          continue;
        end
        if(inst_tr.inst_type == ALU && inst_tr.alu_type == OPIVI && (inst_tr.alu_inst inside {VSUB, VSBC, VMSBC, VMINU, VMIN, VMAXU, VMAX, VMSLTU, VMSLT})) begin
          `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: OPIVI of %0s is ignored.",pc,inst_tr.alu_inst.name()))
          continue;
        end
        if(inst_tr.inst_type == ALU && inst_tr.alu_inst == VMERGE_VMVV && inst_tr.vm == 1 && inst_tr.src2_idx != 0) begin
          `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: src2_idx != 0 of vmv.v is ignored.",pc))
          continue;
        end
        if(inst_tr.inst_type == ALU && inst_tr.alu_type == OPMVX && (inst_tr.alu_inst inside {VMAND, VMOR, VMXOR, VMORN, VMNAND, VMNOR, VMANDN, VMXNOR})) begin
          `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: OPMVX of %0s is ignored.",pc,inst_tr.alu_inst.name()))
          continue;
        end
        if(inst_tr.inst_type == ALU && inst_tr.alu_type == OPMVV && (inst_tr.alu_inst inside {VWMACCUS})) begin
          `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: OPMVV of %0s is ignored.",pc,inst_tr.alu_inst.name()))
          continue;
        end
        if(inst_tr.inst_type == ALU && inst_tr.alu_type == OPMVX && (inst_tr.alu_inst inside {VWXUNARY0}) && inst_tr.src1_type == FUNC && inst_tr.src1_idx inside {VCPOP, VFIRST}) begin
          `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: OPMVX of %0s vcpop/vfirst is ignored.",pc,inst_tr.alu_inst.name()))
          continue;
        end
        if(inst_tr.inst_type == ALU && inst_tr.alu_type == OPMVX && (inst_tr.alu_inst inside {VMUNARY0})) begin
          `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: OPMVX of %0s is ignored.",pc,inst_tr.alu_inst.name()))
          continue;
        end
        if(inst_tr.inst_type == ALU && inst_tr.alu_type == OPMVX && (inst_tr.alu_inst inside {VREDSUM, VREDAND, VREDOR, VREDXOR,
                                                                                              VREDMINU,VREDMIN,VREDMAXU,VREDMAX})) begin
          `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: OPMVX of %0s is ignored.",pc,inst_tr.alu_inst.name()))
          continue;
        end

        // vm for special inst check
        if(inst_tr.inst_type == ALU && (inst_tr.alu_inst inside {VADC, VSBC}) && inst_tr.vm == 1) begin
          `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: vm == 1 of %0s is ignored.",pc,inst_tr.alu_inst.name()))
          continue;
        end
        if(inst_tr.inst_type == ALU && (inst_tr.alu_inst inside {VMAND, VMOR, VMXOR, VMORN, VMNAND, VMNOR, VMANDN, VMXNOR}) && inst_tr.vm == 0) begin
          `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: vm == 0 of %0s is ignored.",pc,inst_tr.alu_inst.name()))
          continue;
        end
        if(inst_tr.inst_type == ALU && inst_tr.alu_inst == VWXUNARY0 && inst_tr.src1_type == FUNC && inst_tr.src1_idx inside {VCPOP, VFIRST} && inst_tr.vstart !== 0) begin
          `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: vstart = %0d of vcpop/vfirst is ignored.", pc, inst_tr.vstart))
          continue;
        end
        if(inst_tr.inst_type == ALU && inst_tr.alu_inst == VMUNARY0 && inst_tr.src1_idx inside {VMSBF, VMSOF, VMSIF, VIOTA} && inst_tr.vstart !== 0) begin
          `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: vstart = %0d of vmsf/vmsof/vmsif/viota is ignored.", pc, inst_tr.vstart))
          continue;
        end
        if(inst_tr.inst_type == ALU && inst_tr.alu_inst == VMUNARY0 && inst_tr.src1_idx inside {VID} && inst_tr.src2_idx !== 0) begin
          `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: vs2 = v%0d of vid is ignored.", pc, inst_tr.src2_idx))
          continue;
        end
        if(is_reduction_inst && inst_tr.vstart !== 0) begin
          `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: vstart = %0d of reduction insts is ignored.", pc, inst_tr.vstart))
          continue;
        end
        // FIXME: vstart != 0 of vmv.s.x isn't a illegal case, it will just dispatch/retire nothing.
        //        Move this part to retire section.
        if(inst_tr.inst_type == ALU && inst_tr.alu_inst == VWXUNARY0 && inst_tr.dest_type == VRF && inst_tr.src2_type == FUNC && inst_tr.src2_idx inside {VMV_X_S} && inst_tr.src1_type == XRF && inst_tr.vstart !== 0) begin
          `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: vstart = %0d of vmv.s.x is ignored.", pc, inst_tr.vstart))
          continue;
        end

        dest_eew = eew;
        src0_eew = EEW1;
        src1_eew = eew;
        src2_eew = eew;
        src3_eew = eew;

        dest_emul = emul;
        src0_emul = EMUL1;
        src1_emul = emul;
        src2_emul = emul;
        src3_emul = emul;

        vm = inst_tr.vm; // <v0.t>

        case(inst_tr.inst_type)
          LD: begin
            // 1.1 unit-stride and constant-stride
            if(inst_tr.lsu_mop == LSU_E || inst_tr.lsu_mop == LSU_SE) begin
              dest_eew  = inst_tr.lsu_eew;
              dest_emul = inst_tr.lsu_eew * emul / eew;
              src1_eew  = EEW32; // rs1 as base address
              src1_emul = LMUL1;
            end
            // 1.2 vector indexed
            if(inst_tr.lsu_mop == LSU_UXEI || inst_tr.lsu_mop == LSU_OXEI) begin
              dest_eew  = eew;   
              dest_emul = emul;
              src1_eew  = EEW32;    // rs1 as base address
              src1_emul = LMUL1;
              src2_eew  = inst_tr.lsu_eew;  // vs2 as offset address
              src2_emul = inst_tr.lsu_eew * emul / eew;
            end
          end
          ST: begin
            // 1.1 unit-stride and constant-stride
            if(inst_tr.lsu_mop == LSU_E || inst_tr.lsu_mop == LSU_SE) begin
              src3_eew = inst_tr.lsu_eew;
              src3_emul = src3_eew * emul / eew;
              src1_eew = EEW32; // rs1 as base address
              src1_emul= LMUL1;
            end
            // 1.2 vector indexed
            if(inst_tr.lsu_mop == LSU_UXEI || inst_tr.lsu_mop == LSU_OXEI) begin
              src3_eew  = eew;   
              src3_emul = emul;
              src1_eew  = EEW32;    // rs1 as base address
              src1_emul = LMUL1;
              src2_eew  = inst_tr.lsu_eew;  // vs2 as offset address
              src2_emul = inst_tr.lsu_eew * emul / eew;
            end
          end
          ALU: begin
            // 1.1 Widen
            if(is_widen_inst) begin
              if(!(vsew inside {SEW8,SEW16})) begin
                `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: Illegal sew(%s) for widen instruction. Ignored.",pc,inst_tr.vsew.name()));
                continue;
              end else if(vlmul == LMUL8) begin
                `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: Illegal lmul(%s) for widen instruction. Ignored.",pc,inst_tr.vlmul.name()));
                continue;
              end else begin
                dest_eew = dest_eew * 2;
                dest_emul = dest_emul * 2;
              end
            end
            if(is_widen_vs2_inst) begin
              if(!(vsew inside {SEW8,SEW16})) begin
                `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: Illegal sew(%s) for widen instruction. Ignored.",pc,inst_tr.vsew.name()));
                continue;
              end else begin
                src2_eew = src2_eew * 2;
                src2_emul = src2_emul * 2;
              end
            end
            // 1.2 Narrow
            if(is_narrow_inst) begin
              if(!(vsew inside {SEW8,SEW16})) begin
                `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: Illegal sew(%s) for narrow instruction. Ignored.",pc,inst_tr.vsew.name()));
                continue;
              end else if(vlmul == LMUL8) begin
                `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: Illegal lmul(%s) for narrow instruction. Ignored.",pc,inst_tr.vlmul.name()));
                continue;
              end else begin
                src2_eew = src2_eew * 2;
                src2_emul = src2_emul * 2;
              end
            end
            if(inst_tr.alu_inst == VXUNARY0) begin
              if(inst_tr.src1_idx inside {VSEXT_VF2, VZEXT_VF2}) begin
                if(!(vsew inside {SEW16,SEW32})) begin
                  `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: Illegal sew(%s) for vext instruction. Ignored.",pc,inst_tr.vsew.name()));
                  continue;
                end else if(!(vlmul inside {LMUL1_2,LMUL1,LMUL2,LMUL4,LMUL8})) begin
                  `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: Illegal lmul(%s) for vext instruction. Ignored.",pc,inst_tr.vlmul.name()));
                  continue;
                end else begin
                  src2_eew = src2_eew / 2;
                  src2_emul = src2_emul / 2;
                end
              end 
              if(inst_tr.src1_idx inside {VSEXT_VF4, VZEXT_VF4}) begin
                if(!(vsew inside {SEW32})) begin
                  `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: Illegal sew(%s) for vext instruction. Ignored.",pc,inst_tr.vsew.name()));
                  continue;
                end else if(!(vlmul inside {LMUL1,LMUL2,LMUL4,LMUL8})) begin
                  `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: Illegal lmul(%s) for vext instruction. Ignored.",pc,inst_tr.vlmul.name()));
                  continue;
                end else begin
                  src2_eew = src2_eew / 4;
                  src2_emul = src2_emul / 4;
                end
              end
            end
            // 1.3 Mask inst
            if(inst_tr.alu_inst inside {VMAND, VMOR, VMXOR, VMORN, VMNAND, VMNOR, VMANDN, VMXNOR} || 
               inst_tr.alu_inst inside {VMUNARY0} && inst_tr.src1_idx inside {VMSBF, VMSOF, VMSIF}) begin
              dest_eew = EEW1;
              dest_emul = dest_emul * dest_eew / eew;
              src2_eew = EEW1;
              src2_emul = src2_emul * src2_eew / eew;
              src1_eew = EEW1;
              src1_emul = src1_emul * src1_eew / eew;
            end
            if(inst_tr.alu_inst inside {VMADC, VMSBC, VMSEQ, VMSNE, VMSLTU, 
                                        VMSLT, VMSLEU, VMSLE, VMSGTU, VMSGT}) begin
              dest_eew = EEW1;
              dest_emul = dest_emul * dest_eew / eew;
            end
            if(inst_tr.inst_type == ALU && inst_tr.alu_inst inside {VMUNARY0} && inst_tr.src1_idx inside {VMSBF, VMSOF, VMSIF}) begin
              dest_eew = EEW1;
              dest_emul = dest_emul * dest_eew / eew;
              src2_eew = EEW1;
              src2_emul = src2_emul * src2_eew / eew;
              src1_eew = EEW1;
              src1_emul = src1_emul * src1_eew / eew;
            end
            if(inst_tr.inst_type == ALU && inst_tr.alu_inst inside {VWXUNARY0} && (inst_tr.src1_type == FUNC && inst_tr.src1_idx inside {VMV_X_S} || 
                                                                                   inst_tr.src2_type == FUNC && inst_tr.src2_idx inside {VMV_X_S})) begin
              dest_eew = EEW1;
              dest_emul = EMUL1;
              src2_emul = EMUL1;
              src1_emul = EMUL1;
            end
            if(inst_tr.inst_type == ALU && inst_tr.alu_inst inside {VWXUNARY0} && inst_tr.src1_type == FUNC && inst_tr.src1_idx inside {VCPOP, VFIRST}) begin
              src2_eew = EEW1;
              src2_emul = src2_emul * src2_eew / eew;
              src1_eew = EEW1;
              src1_emul = src1_emul * src1_eew / eew;
            end
            if(inst_tr.inst_type == ALU && inst_tr.alu_inst inside {VMUNARY0} && inst_tr.src1_idx inside {VIOTA, VID}) begin
              src2_eew = EEW1;
              src2_emul = src2_emul * src2_eew / eew;
              src1_eew = EEW1;
              src1_emul = src1_emul * src1_eew / eew;
            end 
            if(inst_tr.inst_type == ALU && inst_tr.alu_inst inside {VSLIDEUP_RGATHEREI16} && inst_tr.alu_type inside {OPIVV}) begin
              src1_eew = EEW16;
              src1_emul = src1_emul * src1_eew / eew;
            end
            if(is_mask_producing_inst) begin
              // Special case: In this case, DUT will writeback all bits in dest.
              elm_idx_max = `VLEN;
              if(!(is_mask_compare_inst || is_carry_produce_inst)) begin
                // Special case: In this case, DUT will tread any elements > vl as tail and calculate result.
                vlmax = `VLEN;
              end else begin 
                // Special case: In this case, DUT will use vlmax==`VLEN/sew will vlmax, and calculate the result.
                vlmax = fraction_lmul ?        `VLEN / eew: 
                                        emul * `VLEN / eew;
              end
            end
            // 1.4 Reduction inst
            if(is_reduction_inst) begin
              dest_emul = EMUL1;
              src1_emul = EMUL1;
              if(inst_tr.alu_inst inside {VWREDSUM, VWREDSUMU}) begin
                dest_eew = dest_eew * 2;
                src1_eew = src1_eew * 2;
              end
            end
            //end 
            if(is_permutation_inst) begin
              if(inst_tr.inst_type == ALU && inst_tr.alu_inst == VSMUL_VMVNRR ) begin
                dest_eew  = dest_eew;
                case(inst_tr.alu_inst)
                  {VSMUL_VMVNRR}: begin 
                    if(inst_tr.alu_type == OPIVI && inst_tr.vm == 1) begin
                      if(inst_tr.src1_idx inside {0,1,3,7}) begin
                        dest_emul = inst_tr.src1_idx + 1;
                        src2_emul = inst_tr.src1_idx + 1;
                        evl = dest_emul * `VLEN / dest_eew;
                        `uvm_info("MDL/PMT",$sformatf("set evl=%0d, dest_emul=%0d, dest_eew=%0d", evl, dest_emul, dest_eew), UVM_HIGH)
                      end else begin
                        `uvm_error("pmt_processor",$sformatf("Illegal NR (%0d) in vmv<nr>r inst!",inst_tr.src1_idx))
                        continue;
                      end
                    end else begin
                      `uvm_error("pmt_processor","Illeagal vmv<nr>r inst")
                      continue;
                    end
                  end
                  default: `uvm_info("pmt_processor",$sformatf("Not invaild instruction."),UVM_LOW)
                endcase
                //src1_eew  = EEW32; // rs1 as base address
                //src1_emul = LMUL1;
              end 
            // vcompress vs1 emul == 1;
             if(inst_tr.inst_type == ALU && inst_tr.alu_inst == VCOMPRESS) begin
                 src1_emul = EMUL1;
             end
            end // is_permutation_inst
          end // ALU
          default: begin
            continue;
          end
        endcase
        // 1.4 dest is xrf
        if(inst_tr.dest_type == XRF) begin
          dest_eew = EEW32;
          dest_emul = EMUL1; 
        end

        // --------------------------------------------------
        // 2. Prepare oprand
        // 2.1 Update XRF & IMM values
        if(inst_tr.src1_type == XRF) xrf[inst_tr.src1_idx] = inst_tr.rs1_data;
        if(inst_tr.src2_type == XRF) xrf[inst_tr.src2_idx] = inst_tr.rs2_data;
        if(inst_tr.src1_type == IMM) begin 
          imm_data = $signed(inst_tr.src1_idx);
          `uvm_info("MDL", $sformatf("Got imm_data = 0x%8x(%0d) from rs1",imm_data, $signed(imm_data)), UVM_HIGH)
        end else if (inst_tr.src1_type == UIMM) begin
          imm_data = $unsigned(inst_tr.src1_idx);
          `uvm_info("MDL", $sformatf("Got uimm_data = 0x%8x(%0d) from rs1",imm_data, $unsigned(imm_data)), UVM_HIGH)
        end
        if(inst_tr.src2_type == IMM) begin 
          imm_data = $signed(inst_tr.src2_idx);
          `uvm_info("MDL", $sformatf("Got imm_data = 0x%8x(%0d) from rs2",imm_data, $signed(imm_data)), UVM_HIGH)
        end else if (inst_tr.src2_type == UIMM) begin
          imm_data = $unsigned(inst_tr.src2_idx);
          `uvm_info("MDL", $sformatf("Got uimm_data = 0x%8x(%0d) from rs2",imm_data, $unsigned(imm_data)), UVM_HIGH)
        end
          
        `uvm_info("MDL",$sformatf("Prepare done!\nelm_idx_max=%0d\ndest_eew=%0d\nsrc2_eew=%0d\nsrc1_eew=%0d\ndest_emul=%2.4f\nsrc2_emul=%2.4f\nsrc1_emul=%2.4f\n",elm_idx_max,dest_eew,src2_eew,src1_eew,dest_emul,src2_emul,src1_emul),UVM_LOW)

        // 2.2 Check VRF index
        dest_reg_idx_base = inst_tr.dest_idx;
        src2_reg_idx_base = inst_tr.src2_idx;
        src1_reg_idx_base = inst_tr.src1_idx;

        // 2.2.1 Alignment Check
        if(inst_tr.dest_type == VRF) begin
          if(dest_reg_idx_base % int'($ceil(dest_emul)) !== 0) begin
            `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: Ch32.3.4.2. Dest vrf index(%0d) is unaligned to emul(%0d). Ignored.",pc , dest_reg_idx_base, dest_emul));
            continue;
          end
        end
        if(inst_tr.src2_type == VRF) begin
          if(src2_reg_idx_base % int'($ceil(src2_emul)) !== 0) begin
            `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: Ch32.5.3. Src2 vrf index(%0d) is unaligned to emul(%0d). Ignored.",pc, src2_reg_idx_base, src2_emul));
            continue;
          end
        end
        if(inst_tr.src1_type == VRF) begin
          if(src1_reg_idx_base % int'($ceil(src1_emul)) !== 0) begin
            `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: Ch32.3.4.2. Src1 vrf index(%0d) is unaligned to emul(%0d). Ignored.",pc, src1_reg_idx_base, src1_emul));
            continue;
          end
        end
        //Whole Vector Register Move Alignment Check && vstart check 
        if(inst_tr.inst_type == ALU && inst_tr.alu_inst == VSMUL_VMVNRR && inst_tr.alu_type == OPIVI ) begin 
          if(inst_tr.vstart >= evl && (inst_tr.alu_inst inside {VSMUL_VMVNRR})) begin
            `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: VMVNRR Src2 vstart (%0d) is great or equal to evl(%0d). Ignored.",pc, inst_tr.vstart, evl));
            continue;
          end
        end
        //VRGATHEREI16 src1 esew=sew16, EMUL should align.
        if(inst_tr.inst_type == ALU && inst_tr.alu_inst == VSLIDEUP_RGATHEREI16 && inst_tr.alu_type == OPIVV ) begin 
            if(inst_tr.vstart >= evl && (inst_tr.alu_inst inside {VSLIDEUP_RGATHEREI16}) && (src1_reg_idx_base % (int'(dest_emul*(dest_eew/EEW16)) !== 0)) !== 0) begin
            `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: VRGATHER Src2 vstart (%0d) is great or equal to evl(%0d). Ignored.",pc, inst_tr.vstart, evl));
            continue;
          end
        end
        //if(inst_tr.inst_type == ALU && inst_tr.alu_inst == VSLIDEUP_RGATHEREI16 && inst_tr.alu_type == OPIVV ) begin 
        //if((inst_tr.alu_inst inside {VSLIDEUP_RGATHEREI16}) && ((src1_reg_idx_base % 2) !== 0))  begin
        //    `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: VRGATHER Src2 vstart (%0d) is great or equal to evl(%0d). Ignored.",pc, inst_tr.vstart, evl));
        //    continue;
        //  end
        //end

        // 2.2.2 Overlap Check
        // vd overlap v0.t
        if(inst_tr.dest_type == VRF && vm == 0 && dest_reg_idx_base == 0 && dest_eew != EEW1) begin
          `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: Ch32.5.3. Dest vrf index(%0d) overlap source mask register v0. Ignored.",pc,dest_reg_idx_base));
          continue;
        end
        // vd overlap vs2
        if(inst_tr.dest_type == VRF && inst_tr.src2_type == VRF && (dest_eew > src2_eew) &&
           (src2_reg_idx_base >= dest_reg_idx_base) && (src2_reg_idx_base+int'($ceil(src2_emul)) < dest_reg_idx_base+int'($ceil(dest_emul))) && 
           (dest_emul > 1 || src2_emul > 1)) begin
          `uvm_warning("MDL/INST_CHECKER", 
                       $sformatf("pc=0x%8x: Ch32.5.2. The lowest part of dest vrf(v%0d~v%0d) overlaps the src2 vrf(v%0d~v%0d) in a widen instruction. Ignored.",
                       pc, dest_reg_idx_base, dest_reg_idx_base+int'($ceil(dest_emul))-1, src2_reg_idx_base, src2_reg_idx_base+int'($ceil(src2_emul))-1));
          continue;
        end
        if(inst_tr.dest_type == VRF && inst_tr.src2_type == VRF && (dest_eew < src2_eew) &&
           (dest_reg_idx_base > src2_reg_idx_base) && (dest_reg_idx_base+int'($ceil(dest_emul)) <= src2_reg_idx_base+int'($ceil(src2_emul))) && 
           (dest_emul > 1 || src2_emul > 1)) begin
          `uvm_warning("MDL/INST_CHECKER", 
                       $sformatf("pc=0x%8x: Ch32.5.2. The dest vrf(v%0d~v%0d) overlaps the highest part of src2 vrf(v%0d~v%0d) in a narrow instruction. Ignored.",
                       pc, dest_reg_idx_base, dest_reg_idx_base+int'($ceil(dest_emul))-1, src2_reg_idx_base, src2_reg_idx_base+int'($ceil(src2_emul))-1));
          continue;
        end
        // vd overlap vs1
        if(inst_tr.dest_type == VRF && inst_tr.src1_type == VRF && (dest_eew > src1_eew) &&
           (src1_reg_idx_base >= dest_reg_idx_base) && (src1_reg_idx_base+int'($ceil(src1_emul)) < dest_reg_idx_base+int'($ceil(dest_emul))) && 
           (dest_emul > 1 || src1_emul > 1)) begin
          `uvm_warning("MDL/INST_CHECKER",
                       $sformatf("pc=0x%8x: Ch32.5.2. The lower part of dest vrf(v%0d~v%0d) overlaps the src1 vrf(v%0d~v%0d) in a widen instruction. Ignored.",
                       pc, dest_reg_idx_base, dest_reg_idx_base+int'($ceil(dest_emul))-1, src1_reg_idx_base, src1_reg_idx_base+int'($ceil(src1_emul))-1));
          continue;
        end
        if(inst_tr.dest_type == VRF && inst_tr.src1_type == VRF && (dest_eew < src1_eew) &&
           (dest_reg_idx_base > src1_reg_idx_base) && (dest_reg_idx_base+int'($ceil(dest_emul)) <= src1_reg_idx_base+int'($ceil(src1_emul))) && 
           (dest_emul > 1 || src1_emul > 1)) begin
          `uvm_warning("MDL/INST_CHECKER", 
                       $sformatf("pc=0x%8x: Ch32.5.2. The dest vrf(v%0d~v%0d) overlaps the highest part of src1 vrf(v%0d~v%0d) in a narrow instruction. Ignored.",
                       pc, dest_reg_idx_base, dest_reg_idx_base+int'($ceil(dest_emul))-1, src1_reg_idx_base, src1_reg_idx_base+int'($ceil(src1_emul))-1));
          continue;
        end
        // Special case
        if(inst_tr.inst_type == ALU && inst_tr.alu_inst == VMUNARY0 && inst_tr.src1_idx inside {VMSBF, VMSOF, VMSIF} && dest_reg_idx_base == src2_reg_idx_base) begin
          `uvm_warning("MDL/INST_CHECKER", 
                       $sformatf("pc=0x%8x: In vmsf/vmsof/vmsif, dest vrf(v%0d) can't overlap src2 vrf(v%0d). Ignored.", 
                       pc, dest_reg_idx_base, src2_reg_idx_base));
          continue;
        end
        if(inst_tr.inst_type == ALU && inst_tr.alu_inst == VMUNARY0 && inst_tr.src1_idx inside {VIOTA} && 
           src2_reg_idx_base inside {[dest_reg_idx_base:dest_reg_idx_base+int'($ceil(dest_emul))-1]}) begin
          `uvm_warning("MDL/INST_CHECKER", 
                       $sformatf("pc=0x%8x: In viota, dest vrf(%0d~%0d) can't overlap src2 vrf(%0d). Ignored.", 
                      pc, dest_reg_idx_base, dest_reg_idx_base+int'($ceil(dest_emul))-1, src2_reg_idx_base));
          continue;
        end
        if(inst_tr.inst_type == ALU && inst_tr.alu_inst == VMUNARY0 && inst_tr.src1_idx inside {VMSBF, VMSOF, VMSIF, VIOTA} && dest_reg_idx_base == 0 && vm == 0) begin
          `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: In vmsf/vmsof/vmsif/viota, dest vrf(v%0d) can't overlap v0 while vm == 0. Ignored.", pc, dest_reg_idx_base))
          continue;
        end
        // vd overlaps vs2
        if(inst_tr.inst_type == ALU && inst_tr.alu_inst == VSLIDEUP_RGATHEREI16 && inst_tr.dest_type == VRF && inst_tr.src2_type == VRF && 
           (src2_reg_idx_base >= dest_reg_idx_base) && (src2_reg_idx_base+int'($ceil(src2_emul)) <= dest_reg_idx_base+int'($ceil(dest_emul)))  
           ) begin
          `uvm_warning("MDL/INST_CHECKER",
                       $sformatf("pc=0x%8x: Ch31.16.3. The lower part of dest vrf(v%0d~v%0d) overlaps the src2 vrf(v%0d~v%0d) in a vslide instruction. Ignored.",
                       pc, dest_reg_idx_base, dest_reg_idx_base+int'($ceil(dest_emul))-1, src2_reg_idx_base, src2_reg_idx_base+int'($ceil(src2_emul))-1));
          continue;
        end
        if(inst_tr.inst_type == ALU && inst_tr.alu_inst == VSLIDEUP_RGATHEREI16 && inst_tr.dest_type == VRF && inst_tr.src2_type == VRF && 
           (dest_reg_idx_base > src2_reg_idx_base) && (dest_reg_idx_base+int'($ceil(dest_emul)) <= src2_reg_idx_base+int'($ceil(src2_emul)))  
           ) begin
          `uvm_warning("MDL/INST_CHECKER", 
                       $sformatf("pc=0x%8x: Ch31.16.3. The dest vrf(v%0d~v%0d) overlaps the highest part of src1 vrf(v%0d~v%0d) in a vslide instruction. Ignored.",
                       pc, dest_reg_idx_base, dest_reg_idx_base+int'($ceil(dest_emul))-1, src2_reg_idx_base, src2_reg_idx_base+int'($ceil(src2_emul))-1));
          continue;
       end 
        // vslide overlaps vs2
        if(inst_tr.inst_type == ALU && inst_tr.alu_inst == VSLIDE1UP && inst_tr.dest_type == VRF && inst_tr.src2_type == VRF && 
           (src2_reg_idx_base >= dest_reg_idx_base) && (src2_reg_idx_base+int'($ceil(src2_emul)) <= dest_reg_idx_base+int'($ceil(dest_emul)))  
           ) begin
          `uvm_warning("MDL/INST_CHECKER",
                       $sformatf("pc=0x%8x: Ch31.16.3. The lower part of dest vrf(v%0d~v%0d) overlaps the src2 vrf(v%0d~v%0d) in a vslide instruction. Ignored.",
                       pc, dest_reg_idx_base, dest_reg_idx_base+int'($ceil(dest_emul))-1, src2_reg_idx_base, src2_reg_idx_base+int'($ceil(src2_emul))-1));
          continue;
        end
        if(inst_tr.inst_type == ALU && inst_tr.alu_inst == VSLIDE1UP && inst_tr.dest_type == VRF && inst_tr.src2_type == VRF && 
           (dest_reg_idx_base > src2_reg_idx_base) && (dest_reg_idx_base+int'($ceil(dest_emul)) <= src2_reg_idx_base+int'($ceil(src2_emul)))  
           ) begin
          `uvm_warning("MDL/INST_CHECKER", 
                       $sformatf("pc=0x%8x: Ch31.16.3. The dest vrf(v%0d~v%0d) overlaps the highest part of src1 vrf(v%0d~v%0d) in a vslide instruction. Ignored.",
                       pc, dest_reg_idx_base, dest_reg_idx_base+int'($ceil(dest_emul))-1, src2_reg_idx_base, src2_reg_idx_base+int'($ceil(src2_emul))-1));
          continue;
       end 
       // vd overlaps vs1
        if(inst_tr.inst_type == ALU && inst_tr.alu_inst == VSLIDEUP_RGATHEREI16 && inst_tr.dest_type == VRF && inst_tr.src2_type == VRF && inst_tr.src1_type == VRF &&
           (src1_reg_idx_base >= dest_reg_idx_base) && (src1_reg_idx_base+int'($ceil(src1_emul)) <= dest_reg_idx_base+int'($ceil(dest_emul)))  
           ) begin
          `uvm_warning("MDL/INST_CHECKER",
                       $sformatf("pc=0x%8x: Ch31.16.3. The lower part of dest vrf(v%0d~v%0d) overlaps the src2 vrf(v%0d~v%0d) in a vslide instruction. Ignored.",
                       pc, dest_reg_idx_base, dest_reg_idx_base+int'($ceil(dest_emul))-1, src2_reg_idx_base, src2_reg_idx_base+int'($ceil(src2_emul))-1));
          continue;
        end
        if(inst_tr.inst_type == ALU && inst_tr.alu_inst == VSLIDEUP_RGATHEREI16 && inst_tr.dest_type == VRF && inst_tr.src2_type == VRF && inst_tr.src1_type == VRF &&
            (src1_reg_idx_base >= dest_reg_idx_base) && (src1_reg_idx_base < dest_reg_idx_base + int'($ceil(dest_emul))) 
           ) begin
          `uvm_warning("MDL/INST_CHECKER",
                       $sformatf("pc=0x%8x: Ch31.16.3. The lower part of dest vrf(v%0d~v%0d) overlaps the src2 vrf(v%0d~v%0d) in a vslide instruction. Ignored.",
                       pc, dest_reg_idx_base, dest_reg_idx_base+int'($ceil(dest_emul))-1, src2_reg_idx_base, src2_reg_idx_base+int'($ceil(src2_emul))-1));
          continue;
        end
        if(inst_tr.inst_type == ALU && inst_tr.alu_inst == VSLIDEUP_RGATHEREI16 && inst_tr.dest_type == VRF && inst_tr.src2_type == VRF && inst_tr.src1_type == VRF &&
           (dest_reg_idx_base > src1_reg_idx_base) && (dest_reg_idx_base+int'($ceil(dest_emul)) <= src1_reg_idx_base+int'($ceil(src1_emul)))  
           ) begin
          `uvm_warning("MDL/INST_CHECKER", 
                       $sformatf("pc=0x%8x: Ch31.16.3. The dest vrf(v%0d~v%0d) overlaps the highest part of src1 vrf(v%0d~v%0d) in a vslide instruction. Ignored.",
                       pc, dest_reg_idx_base, dest_reg_idx_base+int'($ceil(dest_emul))-1, src2_reg_idx_base, src2_reg_idx_base+int'($ceil(src2_emul))-1));
          continue;
      end
        if(inst_tr.inst_type == ALU && inst_tr.alu_inst == VSLIDEUP_RGATHEREI16 && inst_tr.dest_type == VRF && inst_tr.src2_type == VRF && inst_tr.src1_type == VRF &&
           (dest_reg_idx_base > src1_reg_idx_base) && (dest_reg_idx_base < src1_reg_idx_base+int'($ceil(src1_emul)))  
           ) begin
          `uvm_warning("MDL/INST_CHECKER", 
                       $sformatf("pc=0x%8x: Ch31.16.3. The dest vrf(v%0d~v%0d) overlaps the highest part of src1 vrf(v%0d~v%0d) in a vslide instruction. Ignored.",
                       pc, dest_reg_idx_base, dest_reg_idx_base+int'($ceil(dest_emul))-1, src2_reg_idx_base, src2_reg_idx_base+int'($ceil(src2_emul))-1));
          continue;
      end
// vgather vs1 emul > EMUL8
        if(inst_tr.inst_type == ALU && inst_tr.alu_inst == VSLIDEUP_RGATHEREI16 && inst_tr.dest_type == VRF && inst_tr.src2_type == VRF && inst_tr.src1_type == VRF &&
           (src1_emul > EMUL8) 
           ) begin
          `uvm_warning("MDL/INST_CHECKER", 
                       $sformatf("pc=0x%8x: Ch31.16.3. The dest vrf(v%0d~v%0d) overlaps the highest part of src1 vrf(v%0d~v%0d) in a vslide instruction. Ignored.",
                       pc, dest_reg_idx_base, dest_reg_idx_base+int'($ceil(dest_emul))-1, src2_reg_idx_base, src2_reg_idx_base+int'($ceil(src2_emul))-1));
          continue;
      end
        // vgather vd overlaps 
        if(inst_tr.inst_type == ALU && inst_tr.alu_inst == VRGATHER && inst_tr.dest_type == VRF && inst_tr.src2_type == VRF && 
           (src2_reg_idx_base >= dest_reg_idx_base) && (src2_reg_idx_base+int'($ceil(src2_emul)) <= dest_reg_idx_base+int'($ceil(dest_emul)))  
           ) begin
          `uvm_warning("MDL/INST_CHECKER",
                       $sformatf("pc=0x%8x: Ch31.16.4. The lower part of dest vrf(v%0d~v%0d) overlaps the src2 vrf(v%0d~v%0d) in a vslide instruction. Ignored.",
                       pc, dest_reg_idx_base, dest_reg_idx_base+int'($ceil(dest_emul))-1, src2_reg_idx_base, src2_reg_idx_base+int'($ceil(src2_emul))-1));
          continue;
        end
        if(inst_tr.inst_type == ALU && inst_tr.alu_inst == VRGATHER && inst_tr.dest_type == VRF && inst_tr.src2_type == VRF && 
           (dest_reg_idx_base > src2_reg_idx_base) && (dest_reg_idx_base+int'($ceil(dest_emul)) <= src2_reg_idx_base+int'($ceil(src2_emul)))  
           ) begin
          `uvm_warning("MDL/INST_CHECKER", 
                       $sformatf("pc=0x%8x: Ch31.16.4. The dest vrf(v%0d~v%0d) overlaps the highest part of src1 vrf(v%0d~v%0d) in a vslide instruction. Ignored.",
                       pc, dest_reg_idx_base, dest_reg_idx_base+int'($ceil(dest_emul))-1, src2_reg_idx_base, src2_reg_idx_base+int'($ceil(src2_emul))-1));
          continue;
        end
        if(inst_tr.inst_type == ALU && inst_tr.alu_inst == VRGATHER && inst_tr.dest_type == VRF && inst_tr.src2_type == VRF && inst_tr.src1_type == VRF &&
           (src1_reg_idx_base >= dest_reg_idx_base) && (src1_reg_idx_base+int'($ceil(src1_emul)) <= dest_reg_idx_base+int'($ceil(dest_emul)))  
           ) begin
          `uvm_warning("MDL/INST_CHECKER",
                       $sformatf("pc=0x%8x: Ch31.16.4. The lower part of dest vrf(v%0d~v%0d) overlaps the src2 vrf(v%0d~v%0d) in a vslide instruction. Ignored.",
                       pc, dest_reg_idx_base, dest_reg_idx_base+int'($ceil(dest_emul))-1, src2_reg_idx_base, src2_reg_idx_base+int'($ceil(src2_emul))-1));
          continue;
        end
        if(inst_tr.inst_type == ALU && inst_tr.alu_inst == VRGATHER && inst_tr.dest_type == VRF && inst_tr.src2_type == VRF && inst_tr.src1_type == VRF &&
           (dest_reg_idx_base > src1_reg_idx_base) && (dest_reg_idx_base+int'($ceil(dest_emul)) <= src1_reg_idx_base+int'($ceil(src1_emul)))  
           ) begin
          `uvm_warning("MDL/INST_CHECKER", 
                       $sformatf("pc=0x%8x: Ch31.16.4. The dest vrf(v%0d~v%0d) overlaps the highest part of src1 vrf(v%0d~v%0d) in a vslide instruction. Ignored.",
                       pc, dest_reg_idx_base, dest_reg_idx_base+int'($ceil(dest_emul))-1, src2_reg_idx_base, src2_reg_idx_base+int'($ceil(src2_emul))-1));
          continue;
      end

        // vcompress vd overlaps 
        if(inst_tr.inst_type == ALU && inst_tr.alu_inst == VCOMPRESS && inst_tr.dest_type == VRF && inst_tr.src2_type == VRF && inst_tr.src1_type == VRF &&
            ((src2_reg_idx_base >= dest_reg_idx_base) && (src2_reg_idx_base < dest_reg_idx_base+int'($ceil(dest_emul))) || 
            ((src2_reg_idx_base < dest_reg_idx_base) && (src2_reg_idx_base+int'($ceil(src2_emul)) > dest_reg_idx_base)) ||
            ((src1_reg_idx_base < dest_reg_idx_base) && (src1_reg_idx_base+int'($ceil(src1_emul)) > dest_reg_idx_base)) ||
            ((src1_reg_idx_base >= dest_reg_idx_base) && (src1_reg_idx_base < dest_reg_idx_base+int'($ceil(dest_emul)))))
           ) begin
          `uvm_warning("MDL/INST_CHECKER",
                       $sformatf("pc=0x%8x: Ch31.16.4. The lower part of dest vrf(v%0d~v%0d) overlaps the src2 vrf(v%0d~v%0d) in a vslide instruction. Ignored.",
                       pc, dest_reg_idx_base, dest_reg_idx_base+int'($ceil(dest_emul))-1, src2_reg_idx_base, src2_reg_idx_base+int'($ceil(src2_emul))-1));
          continue;
        end
        // vcompress vstart!=0 will discard
        if(inst_tr.inst_type == ALU && inst_tr.alu_inst == VCOMPRESS && (inst_tr.vstart != 0)
           ) begin
          continue;
      end



        vrf_temp = vrf;
        vrf_bit_strobe_temp = '0;
        `uvm_info("MDL",$sformatf("Check done!\nelm_idx_max=%0d\ndest_eew=%0d\nsrc2_eew=%0d\nsrc1_eew=%0d\nsrc0_eew=%0d\ndest_emul=%2.4f\nsrc2_emul=%2.4f\nsrc1_emul=%2.4f\nsrc0_emul=%2.4f\n",elm_idx_max,dest_eew,src2_eew,src1_eew,src0_eew,dest_emul,src2_emul,src1_emul,src0_emul),UVM_LOW)
        // --------------------------------------------------
        // 3. Operate elements
        if( is_permutation_inst == 1) begin //is_permutation_inst
        // vstart = 0 ch31.14
        //if (vstart != 0) begin
            `uvm_info("RDT_CHECKER", $sformatf("The Instructions are RDT PMT Instructions\n"),UVM_LOW)
        //endcase
              // src0 = 0;
              //rdt_inst.RDT_INIT(this,inst_tr);
              //rdt_inst.rdt_func(this,inst_tr);
              pmt_inst.PMT_INIT(this,inst_tr);
              pmt_inst.pmt_func(this,inst_tr);
        end else if(inst_tr.inst_type inside {LD, ST}) begin
          lsu_proc.exe(this, inst_tr);
        // Writeback whole vrf
        vrf = vrf_temp;
        for(int i=0; i<32; i++) begin
          for(int j=0; j<`VLENB; j++) begin
            vrf_byte_strobe_temp[i][j] = |vrf_bit_strobe_temp[i][j*8 +: 8];
          end
        end

      end else begin
        `uvm_info("rbm_test",$sformatf("vrf[31] = %0d",vrf[31]),UVM_LOW)
        for(int elm_idx=0; elm_idx<elm_idx_max; elm_idx++) begin : op_element

          // 3.0 Update elements index
          src3 = '0;
          src2 = '0;
          src1 = '0;
          src0 = '0;
          case(inst_tr.dest_type) 
            VRF: begin
              dest_reg_idx = elm_idx / (`VLEN / dest_eew) + dest_reg_idx_base;
              dest_reg_elm_idx = elm_idx % (`VLEN / dest_eew);
            end
            default: begin
              dest_reg_idx = dest_reg_idx_base;
              dest_reg_elm_idx = 0;
            end
          endcase
          case(inst_tr.src3_type)
            VRF: begin 
              src3_reg_idx = elm_idx / (`VLEN / src3_eew) + src3_reg_idx_base;
              src3_reg_elm_idx = elm_idx % (`VLEN / src3_eew);
            end
            default: begin
              src3_reg_idx = src3_reg_idx_base;
              src3_reg_elm_idx =0;
            end
          endcase
          case(inst_tr.src2_type)
            VRF: begin 
              src2_reg_idx = elm_idx / (`VLEN / src2_eew) + src2_reg_idx_base;
              src2_reg_elm_idx = elm_idx % (`VLEN / src2_eew);
            end
            default: begin
              src2_reg_idx = src2_reg_idx_base;
              src2_reg_elm_idx =0;
            end
          endcase
          case(inst_tr.src1_type)
            VRF: begin 
              src1_reg_idx = elm_idx / (`VLEN / src1_eew) + src1_reg_idx_base;
              src1_reg_elm_idx = elm_idx % (`VLEN / src1_eew);
            end
            default: begin
              src1_reg_idx = src1_reg_idx_base;
              src1_reg_elm_idx =0;
            end
          endcase
            src0_reg_idx = 0;
            src0_reg_elm_idx = elm_idx % (`VLEN / src0_eew);

          // 3.1 Fetch elements data 
          dest = elm_fetch(inst_tr.dest_type, dest_reg_idx_base, elm_idx, dest_eew);
          src3 = elm_fetch(inst_tr.src3_type, src3_reg_idx_base, elm_idx, src3_eew); 
          src2 = elm_fetch(inst_tr.src2_type, src2_reg_idx_base, elm_idx, src2_eew); 
          src1 = elm_fetch(inst_tr.src1_type, src1_reg_idx_base, elm_idx, src1_eew); 
          if(vm == 0) begin
            src0 = elm_fetch(VRF, 0, elm_idx, src0_eew);
          end else begin
            src0 = '1;
            if(inst_tr.alu_inst inside {VMERGE_VMVV})
              src0 = '1;
            if(inst_tr.alu_inst inside {VMADC, VMSBC})
              src0 = '0;
          end
          
          `uvm_info("MDL", "\n---------------------------------------------------------------------------------------------------------------------------------\n", UVM_LOW)
          `uvm_info("MDL", $sformatf("Before - element[%2d]:\n  dest(v[%2d][%2d])=0x%8h\n  src2(v[%2d][%2d])=0x%8h\n  src1(v[%2d][%2d])=0x%8h\n  src0(v[%2d][%2d])=0x%8h",
                                      elm_idx, 
                                      dest_reg_idx, dest_reg_elm_idx, dest, 
                                      src2_reg_idx, src2_reg_elm_idx, src2, 
                                      src1_reg_idx, src1_reg_elm_idx, src1, 
                                      src0_reg_idx, src0_reg_elm_idx, src0), UVM_LOW)

          // 3.2 Execute & Writeback 
          // Prepare processor handler
          case(inst_tr.inst_type)
            LD: begin 
              `uvm_fatal(get_type_name(),"Load fucntion hasn't been defined.")
            end
            ST: begin 
              `uvm_fatal(get_type_name(),"Store fucntion hasn't been defined.")
            end
            ALU: begin 
              case({dest_eew, src2_eew, src1_eew})
                { EEW8,  EEW8,  EEW8}: alu_handler = alu_08_08_08;
                {EEW16, EEW16, EEW16}: alu_handler = alu_16_16_16;
                {EEW32, EEW32, EEW32}: alu_handler = alu_32_32_32;
                // widen
                {EEW16,  EEW8,  EEW8}: alu_handler = alu_16_08_08;
                {EEW16, EEW16,  EEW8}: alu_handler = alu_16_16_08;
                {EEW32, EEW16, EEW16}: alu_handler = alu_32_16_16;
                {EEW32, EEW32, EEW16}: alu_handler = alu_32_32_16;
                //ext
                {EEW16,  EEW8, EEW32}: alu_handler = alu_16_08_32;
                {EEW16,  EEW8, EEW16}: alu_handler = alu_16_08_16;
                {EEW32, EEW16, EEW32}: alu_handler = alu_32_16_32;
                {EEW32, EEW16,  EEW8}: alu_handler = alu_32_16_08;
                {EEW32,  EEW8, EEW32}: alu_handler = alu_32_08_32;
                {EEW32,  EEW8, EEW16}: alu_handler = alu_32_08_16;
                {EEW32,  EEW8,  EEW8}: alu_handler = alu_32_08_08;
                // narrow
                { EEW8, EEW16,  EEW8}: alu_handler = alu_08_16_08;
                {EEW16, EEW32, EEW16}: alu_handler = alu_16_32_16;
                // mask logic
                { EEW1,  EEW1,  EEW1}: alu_handler = alu_01_01_01;
                { EEW1,  EEW8,  EEW8}: alu_handler = alu_01_08_08;
                { EEW1, EEW16, EEW16}: alu_handler = alu_01_16_16;
                { EEW1, EEW32, EEW32}: alu_handler = alu_01_32_32;
                // viot/vfirst/...
                { EEW8,  EEW1,  EEW1}: alu_handler = alu_08_01_01;
                {EEW16,  EEW1,  EEW1}: alu_handler = alu_16_01_01;
                {EEW32,  EEW1,  EEW1}: alu_handler = alu_32_01_01;
                default: begin
                  `uvm_error("TB_ISSUE", $sformatf("pc=0x%8x: Unsupported EEW: dest_eew=%d, src2_eew=%d, src1_eew=%d",pc, dest_eew, src2_eew, src1_eew))
                  continue;
                end
              endcase
              if(elm_idx == 0) begin
                alu_handler.reset();
              end
              alu_handler.set_vxrm(vxrm);
              alu_handler.set_elm_idx(elm_idx);
            end
          endcase

          // For vs1[0] of reduction inst.
          if(is_reduction_inst && elm_idx == 0) begin
            dest = alu_handler.exe_first_scalar(inst_tr, dest, src2, src1, src0);
            elm_writeback(dest, inst_tr.dest_type, dest_reg_idx_base, elm_idx, dest_eew);
            `uvm_info("MDL", $sformatf("Scalar - element[%2d]:\n  dest(v[%2d][%2d])=0x%8h\n  src2(v[%2d][%2d])=0x%8h\n  src1(v[%2d][%2d])=0x%8h\n  src0(v[%2d][%2d])=0x%8h",
                                        elm_idx, 
                                        dest_reg_idx, dest_reg_elm_idx, dest, 
                                        src2_reg_idx, src2_reg_elm_idx, src2, 
                                        src1_reg_idx, src1_reg_elm_idx, src1, 
                                        src0_reg_idx, src0_reg_elm_idx, src0), UVM_LOW)
          end

          if(elm_idx < vstart) begin
            // pre-start: do nothing
            if(is_mask_producing_inst) begin
              // Special case: If is mask producing operation, it will keep original values.
              `uvm_info("MDL", $sformatf("element[%2d]: pre-start, mask producing operation", elm_idx), UVM_LOW)
              elm_writeback(dest, inst_tr.dest_type, dest_reg_idx_base, elm_idx, dest_eew);
            end else begin 
              `uvm_info("MDL", $sformatf("element[%2d]: pre-start", elm_idx), UVM_LOW)
            end
          end else if(elm_idx >= vl) begin
            // tail
            if(is_mask_producing_inst) begin
              if(elm_idx < vlmax) begin
                // tail-1
                // Special case: If is mask producing operation, it will write with calculation results.
                `uvm_info("MDL", $sformatf("element[%2d]: tail, mask producing operation", elm_idx), UVM_LOW)
                if((!vm && this.vrf[0][elm_idx]) || vm || use_vm_to_cal ) begin
                  dest = alu_handler.exe(inst_tr, dest, src2, src1, src0);
                end
                elm_writeback(dest, inst_tr.dest_type, dest_reg_idx_base, elm_idx, dest_eew);
              end else begin
                // tail-2
                // Special case: If is mask producing operation, 
                //               it will keep original values, but write strobe should be 1.
                elm_writeback(dest, inst_tr.dest_type, dest_reg_idx_base, elm_idx, dest_eew);
              end
            end else begin 
              `uvm_info("MDL", $sformatf("element[%2d]: tail", elm_idx), UVM_LOW)
              if(vta == AGNOSTIC) begin
                if(all_one_for_agn) begin 
                  dest = '1;
                  elm_writeback(dest, inst_tr.dest_type, dest_reg_idx_base, elm_idx, dest_eew);
                end
              end else begin
              end
            end
          end else if(!(vm || this.vrf[0][elm_idx] || use_vm_to_cal)) begin
            // body-inactive
            if(is_mask_producing_inst) begin
              `uvm_info("MDL", $sformatf("element[%2d]: body-inactive, mask producing operation", elm_idx), UVM_LOW)
              elm_writeback(dest, inst_tr.dest_type, dest_reg_idx_base, elm_idx, dest_eew);
            end else begin
              `uvm_info("MDL", $sformatf("element[%2d]: body-inactive", elm_idx), UVM_LOW)
              if(vma == AGNOSTIC) begin
                if(all_one_for_agn) begin 
                  dest = '1;
                  elm_writeback(dest, inst_tr.dest_type, dest_reg_idx_base, elm_idx, dest_eew);
                end
              end else begin
              end
            end
          end else begin
            // body-active
            `uvm_info("MDL", $sformatf("element[%2d]: body-active", elm_idx), UVM_LOW)
            // EX
            case(inst_tr.inst_type)
              LD: begin `uvm_fatal(get_type_name(),"Load fucntion hasn't been defined.") end
              ST: begin `uvm_fatal(get_type_name(),"Store fucntion hasn't been defined.") end
              ALU: begin 
                dest = alu_handler.exe(inst_tr, dest, src2, src1, src0);
                if(vxsat === 0 && alu_handler.get_saturate() === 1) begin
                  `uvm_info("MDL", $sformatf("element[%2d]: body-active, vxsat has been set to 1.", elm_idx), UVM_LOW)
                end
                vxsat = vxsat ? vxsat : alu_handler.get_saturate();

                elm_writeback(dest, inst_tr.dest_type, dest_reg_idx_base, elm_idx, dest_eew);
             end 
            endcase
            // Write back
          end

          // Special case : Get the final value to wireback to XRF, avoiding all inactive situation. 
          if(elm_idx == elm_idx_max-1) begin
            if(inst_tr.dest_type == XRF) begin
              dest = alu_handler.get_xrf_wb_value(inst_tr);
              elm_writeback(dest, inst_tr.dest_type, dest_reg_idx_base, elm_idx, dest_eew);
            end
          end

          `uvm_info("MDL", $sformatf("After - element[%2d]:\n  dest(v[%2d][%2d])=0x%8h\n  src2(v[%2d][%2d])=0x%8h\n  src1(v[%2d][%2d])=0x%8h\n  src0(v[%2d][%2d])=0x%8h",
                                      elm_idx, 
                                      dest_reg_idx, dest_reg_elm_idx, dest, 
                                      src2_reg_idx, src2_reg_elm_idx, src2, 
                                      src1_reg_idx, src1_reg_elm_idx, src1, 
                                      src0_reg_idx, src0_reg_elm_idx, src0), UVM_LOW)
          `uvm_info("MDL", "\n---------------------------------------------------------------------------------------------------------------------------------\n", UVM_LOW)
        end : op_element 

        // Writeback whole vrf
        vrf = vrf_temp;
        for(int i=0; i<32; i++) begin
          for(int j=0; j<`VLENB; j++) begin
            vrf_byte_strobe_temp[i][j] = |vrf_bit_strobe_temp[i][j*8 +: 8];
          end
        end
      end // is_permutation_inst 


        // --------------------------------------------------
        // 4. Retire transaction gen
     if( is_permutation_inst == 1) begin //is_permutation_inst
        rt_tr   = new("rt_tr");
        rt_tr.copy(inst_tr);
        rt_tr.is_rt = 1;
        // VRF
        // if(rt_tr.dest_type == VRF && !(|vrf_bit_strobe_temp)) begin
        //   `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: Instruction with no valid vrf wirte strobe will be ignored.",pc));
        //   continue;
        // end 
        for(int i=0; i<32; i++) begin
          for(int j=0; j<`VLENB; j++) begin
            vrf_byte_strobe_temp[i][j] = |vrf_bit_strobe_temp[i][j*8 +: 8];
          end
        end


        if(rt_tr.dest_type == VRF) begin
          for(int reg_idx=dest_reg_idx_base; reg_idx<dest_reg_idx_base+int'($ceil(dest_emul)); reg_idx++) begin
            if(inst_tr.inst_type == ALU && (inst_tr.alu_inst inside {VSLIDEUP_RGATHEREI16, VSLIDE1UP, VRGATHER})) begin
              // Special case: for slideup/gather inst, we will retire all uop for now, including all-prestart uops.
              rt_tr.rt_vrf_index.push_back(reg_idx);
              rt_tr.rt_vrf_strobe.push_back(vrf_byte_strobe_temp[reg_idx]);
              rt_tr.rt_vrf_data.push_back(vrf[reg_idx]);
            end else begin
              // FIXME: All 0s in vrf wirte strobe will not be executed in DUT.
              // if(|vrf_byte_strobe_temp[reg_idx]) begin
              // All pre-start vreg will not be retired
              if(((reg_idx - dest_reg_idx_base) >= (vstart / (`VLEN / dest_eew)))   ) begin
                rt_tr.rt_vrf_index.push_back(reg_idx);
                rt_tr.rt_vrf_strobe.push_back(vrf_byte_strobe_temp[reg_idx]);
                rt_tr.rt_vrf_data.push_back(vrf[reg_idx]);
              end
            end
          end
        end
        // XRF
        if(rt_tr.dest_type == XRF) begin
          rt_tr.rt_xrf_index.push_back(rt_tr.dest_idx);
          rt_tr.rt_xrf_data.push_back(this.xrf[rt_tr.dest_idx]);
        end
        // VXSAT
        vxsat_valid = vxsat;
        rt_tr.vxsat = vxsat;
        rt_tr.vxsat_valid = vxsat_valid;



        `uvm_info("MDL",$sformatf("Complete calculation:\n%s",rt_tr.sprint()),UVM_LOW)
        rt_ap.write(rt_tr);
        this.executed_inst++;
    end //is_permutation_inst
    else begin
        rt_tr   = new("rt_tr");
        rt_tr.copy(inst_tr);
        rt_tr.is_rt = 1;
        // VRF
        // if(rt_tr.dest_type == VRF && !(|vrf_bit_strobe_temp)) begin
        //   `uvm_warning("MDL/INST_CHECKER", $sformatf("pc=0x%8x: Instruction with no valid vrf wirte strobe will be ignored.",pc));
        //   continue;
        // end 

        if(rt_tr.dest_type == VRF) begin
          for(int reg_idx=dest_reg_idx_base; reg_idx<dest_reg_idx_base+int'($ceil(dest_emul)); reg_idx++) begin
            // FIXME: All 0s in vrf wirte strobe will not be executed in DUT.
            // if(|vrf_byte_strobe_temp[reg_idx]) begin
            // All pre-start vreg will not be retired
            if((reg_idx - dest_reg_idx_base) >= (vstart / (`VLEN / dest_eew))) begin
              rt_tr.rt_vrf_index.push_back(reg_idx);
              rt_tr.rt_vrf_strobe.push_back(vrf_byte_strobe_temp[reg_idx]);
              rt_tr.rt_vrf_data.push_back(vrf_temp[reg_idx]);
            end
          end
        end
        if(rt_tr.dest_type == SCALAR) begin
          for(int reg_idx=dest_reg_idx_base; reg_idx<dest_reg_idx_base+int'($ceil(dest_emul)); reg_idx++) begin
            // All pre-start vreg will not be retired
            if((reg_idx - dest_reg_idx_base) >= (vstart / (`VLEN / dest_eew))) begin
              rt_tr.rt_vrf_index.push_back(reg_idx);
              rt_tr.rt_vrf_strobe.push_back(vrf_byte_strobe_temp[reg_idx]);
              rt_tr.rt_vrf_data.push_back(vrf_temp[reg_idx]);
            end
          end
        end
        // XRF
        if(rt_tr.dest_type == XRF) begin
          rt_tr.rt_xrf_index.push_back(rt_tr.dest_idx);
          rt_tr.rt_xrf_data.push_back(this.xrf[rt_tr.dest_idx]);
        end
        // VXSAT
        vxsat_valid = vxsat;
        rt_tr.vxsat = vxsat;
        rt_tr.vxsat_valid = vxsat_valid;

        `uvm_info("MDL",$sformatf("Complete calculation:\n%s",rt_tr.sprint()),UVM_LOW)
        rt_ap.write(rt_tr);
        this.executed_inst++;

    end // else end is_permutation_inst
        
      //`uvm_info("MDL",$sformatf("Complete calculation:\n%s",rt_tr.sprint()),UVM_LOW)
      end // if(rt_last_uop[0])
        rt_last_uop = rt_last_uop >> 1;
      end // while(|rt_last_uop)
      end // rst_n
    end // forever
    // `uvm_fatal(get_type_name()),"Im here.")
  endtask

  function logic [31:0] rvv_behavior_model::elm_fetch(oprand_type_e reg_type, int reg_idx, int elm_idx, int eew);
    logic [31:0] result;
    int bit_count;
    bit_count = eew;
    result = '0;
    case(reg_type)
      VRF: begin
        reg_idx = elm_idx / (`VLEN / eew) + reg_idx;
        elm_idx = elm_idx % (`VLEN / eew);
        // `uvm_info("MDL", $sformatf("reg_type=%0d, reg_idx=%0d, elm_idx=%0d, eew=%0d", reg_type, reg_idx, elm_idx, eew), UVM_HIGH)
        for(int i=0; i<bit_count; i++) begin
          result[i] = this.vrf[reg_idx][elm_idx*bit_count + i];
          // `uvm_info("MDL", $sformatf("elm_idx*bit_count + i=%0d", elm_idx*bit_count + i), UVM_HIGH)
          // `uvm_info("MDL", $sformatf("result[%0d]=%0d", i, result[i]), UVM_HIGH)
        end
      end
      SCALAR: begin
        for(int i=0; i<bit_count; i++) begin
          result[i] = elm_idx == 0 ? this.vrf[reg_idx][i]
                                   : this.vrf_temp[reg_idx][i]; // Always use element 0.
        end
      end
      XRF: begin
        for(int i=0; i<bit_count; i++) begin
          result[i] = this.xrf[reg_idx][i]; 
        end
      end
      UIMM,IMM: begin
        for(int i=0; i<bit_count; i++) begin
          result[i] = this.imm_data[i]; 
        end
      end
      default: result = 'x;
    endcase
    elm_fetch = result;
    // `uvm_info("MDL", $sformatf("result=%0h", result), UVM_HIGH)
  endfunction: elm_fetch

  task rvv_behavior_model::elm_writeback(logic [31:0] result, oprand_type_e reg_type, int reg_idx, int elm_idx, int eew);
    int bit_count;
    bit_count = eew;
    case(reg_type)
      VRF: begin
        reg_idx = elm_idx / (`VLEN / eew) + reg_idx;
        elm_idx = elm_idx % (`VLEN / eew);
        for(int i=0; i<bit_count; i++) begin
          this.vrf_temp[reg_idx][elm_idx*bit_count + i] = result[i];
          this.vrf_bit_strobe_temp[reg_idx][elm_idx*bit_count + i] = 1'b1;
        end
      end
      SCALAR: begin
        for(int i=0; i<bit_count; i++) begin
          this.vrf_temp[reg_idx][i] = result[i]; // Always use element 0.
          this.vrf_bit_strobe_temp[reg_idx][i] = 1'b1;
        end
      end
      XRF: begin
        for(int i=0; i<bit_count; i++) begin
          this.xrf[reg_idx][i] = result[i]; 
        end
      end
    endcase
  endtask: elm_writeback

  task rvv_behavior_model::vrf_mdl();
    vrf_transaction tr;
    int last_uop_idx_max, uop_idx_max;
    tr = new();
    forever begin
      @(posedge vrf_if.clk);
      if(vrf_if.rst_n) begin
        last_uop_idx_max = -1;
        uop_idx_max = -1;
        for(int i=0; i<`NUM_RT_UOP; i++) begin
          if(vrf_if.rt_last_uop[i] === 1'b1) last_uop_idx_max = i;
          if(vrf_if.rt_uop[i] === 1'b1) uop_idx_max = i;
        end
        if(last_uop_idx_max>=0 && last_uop_idx_max>=uop_idx_max) begin
          for(int i=0; i<32; i++) begin
              tr.vreg[i] = vrf_delay[i];
          end
          vrf_ap.write(tr);
        end
      end
      vrf_delay <= vrf;
    end
  endtask

// ALU inst part ------------------------------------------
virtual class alu_base; 
  parameter ALU_MAX_WIDTH = `VLEN;

  typedef logic unsigned [ALU_MAX_WIDTH-1:0] alu_unsigned_t;
  typedef logic signed   [ALU_MAX_WIDTH-1:0] alu_signed_t;
  
  // Config signals.
  vxrm_e vxrm;
  int elm_idx;

  // Status signals.
  bit overflow;
  bit underflow;
  int mask_count;
  bit found_first_mask;
  int first_mask_idx;
  pure virtual function alu_unsigned_t exe_first_scalar (rvs_transaction inst_tr, alu_unsigned_t dest, alu_unsigned_t src2, alu_unsigned_t src1, alu_unsigned_t src0);
  pure virtual function alu_unsigned_t exe (rvs_transaction inst_tr, alu_unsigned_t dest, alu_unsigned_t src2, alu_unsigned_t src1, alu_unsigned_t src0);
  pure virtual function alu_unsigned_t _roundoff_unsigned(alu_unsigned_t v, int unsigned d);
  pure virtual function alu_signed_t   _roundoff_signed(  alu_signed_t   v, int unsigned d);
  virtual function void reset();
    this.overflow = 1'b0;
    this.underflow = 1'b0;
    this.mask_count = 'b0;
    this.found_first_mask = 1'b0;
    this.first_mask_idx =  -'d1;
  endfunction: reset
  virtual function void set_vxrm(vxrm_e val);
    this.vxrm = val;
  endfunction: set_vxrm
  virtual function void set_elm_idx(int val);
    this.elm_idx = val;
  endfunction: set_elm_idx
  virtual function bit get_saturate();
    get_saturate = this.overflow || this.underflow;
  endfunction: get_saturate
  virtual function int get_xrf_wb_value(rvs_transaction inst_tr);
  endfunction: get_xrf_wb_value

endclass: alu_base

class alu_processor#(
  type TD = sew8_t,
  type T2 = sew8_t,
  type T1 = sew8_t,  
  type T0 = sew1_t
  ) extends alu_base;  
  
//   virtual void function reset();
//     super.reset();
//   endfunction: reset
//   virtual function void set_vxrm(vxrm_e val);
//     super.set_vxrm(vxrm_e val);
//   endfunction: set_vxrm
//   virtual function void set_elm_idx(int val);
//     super.set_elm_idx(int val);
//   endfunction: set_elm_idx
//   virtual function bit get_saturate();
//     super.get_saturate();
//   endfunction: get_saturate

  virtual function int get_xrf_wb_value(rvs_transaction inst_tr);
    super.get_xrf_wb_value(inst_tr);
    case(inst_tr.alu_inst)
      VWXUNARY0: begin
        case(inst_tr.src1_idx)
          VMV_X_S: begin
            `uvm_fatal("TB_ISSUE", "VMV_X_S should be executed in permutation processor.")
          end
          VCPOP : get_xrf_wb_value = mask_count;
          VFIRST: get_xrf_wb_value = first_mask_idx;
        endcase
      end
    endcase
  endfunction: get_xrf_wb_value

  virtual function alu_unsigned_t exe_first_scalar (rvs_transaction inst_tr, alu_unsigned_t dest, alu_unsigned_t src2, alu_unsigned_t src1, alu_unsigned_t src0);
    `uvm_info("MDL", $sformatf("sizeof(T1)=%0d, sizeof(T2)=%0d, sizeof(TD)=%0d", $size(T1), $size(T2), $size(TD)), UVM_HIGH)
    case(inst_tr.alu_inst) 
    // OPI
      VWREDSUM : exe_first_scalar = src1;
      VWREDSUMU: exe_first_scalar = src1;
    // OPM
      VREDSUM : exe_first_scalar = src1;
      VREDAND : exe_first_scalar = src1;
      VREDOR  : exe_first_scalar = src1;
      VREDXOR : exe_first_scalar = src1;
      VREDMINU: exe_first_scalar = src1;
      VREDMIN : exe_first_scalar = src1;
      VREDMAXU: exe_first_scalar = src1;
      VREDMAX : exe_first_scalar = src1;
    endcase
  endfunction: exe_first_scalar

  virtual function alu_unsigned_t exe (rvs_transaction inst_tr, alu_unsigned_t dest, alu_unsigned_t src2, alu_unsigned_t src1, alu_unsigned_t src0);
    `uvm_info("MDL", $sformatf("sizeof(T1)=%0d, sizeof(T2)=%0d, sizeof(TD)=%0d", $size(T1), $size(T2), $size(TD)), UVM_HIGH)
    overflow  = 0;
    underflow = 0;
    case(inst_tr.alu_inst) 
    // OPI
      VADD : dest = _vadd(src2, src1); 
      VSUB : dest = _vsub(src2, src1); 
      VRSUB: dest = _vrsub(src2, src1); 
  
      VADC : dest = _vadc(src2,src1,src0);
      VMADC: dest = _vmadc(src2,src1,src0);
      VSBC : dest = _vsbc(src2,src1,src0);
      VMSBC: dest = _vmsbc(src2,src1,src0);

      VAND : dest = _vmand(src2, src1); 
      VOR  : dest = _vmor(src2, src1); 
      VXOR : dest = _vmxor(src2, src1); 

      VMSEQ : dest = _vmseq(src2, src1); 
      VMSNE : dest = _vmsne(src2, src1); 
      VMSLTU: dest = _vmsltu(src2, src1); 
      VMSLT : dest = _vmslt(src2, src1); 
      VMSLEU: dest = _vmsleu(src2, src1); 
      VMSLE : dest = _vmsle(src2, src1); 
      VMSGTU: dest = _vmsgtu(src2, src1); 
      VMSGT : dest = _vmsgt(src2, src1); 

      VMINU: dest = _vminu(src2, src1); 
      VMIN : dest = _vmin(src2, src1); 
      VMAXU: dest = _vmaxu(src2, src1); 
      VMAX : dest = _vmax(src2, src1); 

      VMERGE_VMVV: dest = _vmerge(src2, src1, src0); 

      VSADDU: dest = _vsaddu(src2, src1);
      VSADD : dest = _vsadd(src2, src1);
      VSSUBU: dest = _vssubu(src2, src1);
      VSSUB : dest = _vssub(src2, src1);

      VSMUL_VMVNRR: begin
        // if(inst_tr.alu_type == OPIVI) dest = _vvmnrr(src2,src1);
        if(inst_tr.alu_type != OPIVI) dest = _vsmul(src2,src1);
      end
      
      VSSRL: dest = _vssrl(src2, src1);
      VSSRA: dest = _vssra(src2, src1);

      VNCLIPU: dest = _vnclipu(src2, src1);
      VNCLIP : dest = _vnclip(src2, src1);
      
      VWREDSUM : dest = _vwredsum(dest,src2);
      VWREDSUMU: dest = _vwredsumu(dest,src2);

    // OPM
      VWADD,
      VWADD_W:  dest = _vwadd(src2, src1);
      VWADDU,
      VWADDU_W: dest = _vwaddu(src2, src1); 
      VWSUB, 
      VWSUB_W:  dest = _vwsub(src2, src1);
      VWSUBU, 
      VWSUBU_W: dest = _vwsubu(src2, src1); 

      VXUNARY0: begin 
        if(inst_tr.src1_idx == VZEXT_VF4 || inst_tr.src1_idx == VZEXT_VF2) dest = _vzext(src2); 
        if(inst_tr.src1_idx == VSEXT_VF4 || inst_tr.src1_idx == VSEXT_VF2) dest = _vsext(src2); 
      end

      VSLL : dest = _vsll(src2, src1);
      VSRL : dest = _vsrl(src2, src1);
      VSRA : dest = _vsra(src2, src1);
      VNSRL: dest = _vsrl(src2, src1);
      VNSRA: dest = _vsra(src2, src1);
        
      VMUL    : dest = _vmul(src2, src1);
      VMULH   : dest = _vmulh(src2, src1);
      VMULHU  : dest = _vmulhu(src2, src1);
      VMULHSU : dest = _vmulhsu(src2, src1);
                
      VDIVU: dest = _vdivu(src2, src1);
      VDIV : dest = _vdiv(src2, src1);
      VREMU: dest = _vremu(src2, src1);
      VREM : dest = _vrem(src2, src1);        

      VWMUL  : dest = _vwmul(src2, src1);
      VWMULU : dest = _vwmulu(src2, src1);
      VWMULSU: dest = _vwmulsu(src2, src1);

      VMACC : dest = _vmacc(dest, src2, src1);
      VNMSAC: dest = _vnmsac(dest, src2, src1);
      VMADD : dest = _vmadd(dest, src2, src1);
      VNMSUB: dest = _vnmsub(dest, src2, src1);

      VWMACCU  : dest = _vwmaccu(dest, src2, src1);
      VWMACC   : dest = _vwmacc(dest, src2, src1);
      VWMACCUS : dest = _vwmaccus(dest, src2, src1);
      VWMACCSU : dest = _vwmaccsu(dest, src2, src1);

      VAADDU: dest = _vaaddu(src2, src1);
      VAADD : dest = _vaadd(src2, src1);
      VASUBU: dest = _vasubu(src2, src1);
      VASUB : dest = _vasub(src2, src1);

      VREDSUM : dest = _vredsum(dest,src2);
      VREDAND : dest = _vredand(dest,src2);
      VREDOR  : dest = _vredor(dest,src2);
      VREDXOR : dest = _vredxor(dest,src2);
      VREDMINU: dest = _vredminu(dest,src2);
      VREDMIN : dest = _vredmin(dest,src2);
      VREDMAXU: dest = _vredmaxu(dest,src2);
      VREDMAX : dest = _vredmax(dest,src2);

      VMAND : dest = _vmand(src2, src1); 
      VMOR  : dest = _vmor(src2, src1); 
      VMXOR : dest = _vmxor(src2, src1); 
      VMORN : dest = _vmorn(src2, src1); 
      VMNAND: dest = _vmnand(src2, src1); 
      VMNOR : dest = _vmnor(src2, src1); 
      VMANDN: dest = _vmandn(src2, src1); 
      VMXNOR: dest = _vmxnor(src2, src1); 

      VMUNARY0: begin
        case(inst_tr.src1_idx)
          VMSBF: dest = _vmsbf(src2);
          VMSOF: dest = _vmsof(src2);
          VMSIF: dest = _vmsif(src2);
          VIOTA: dest = _viota(src2);
          VID:   dest = _vid();
        endcase
      end

      VWXUNARY0: begin
        case(inst_tr.src1_idx)
          VMV_X_S: begin
            `uvm_fatal("TB_ISSUE", "VMV_X_S should be executed in permutation processor.")
          end
          VCPOP: dest = _vcpop(src2);
          VFIRST: dest = _vfirst(src2);
        endcase
      end
    endcase
    exe = dest;
    // `uvm_info("MDL", $sformatf("dest=%0d, src1=%0d, src2=%0d", exe, src1, src2), UVM_HIGH)
  endfunction : exe

  virtual function alu_unsigned_t _roundoff_unsigned(alu_unsigned_t v, int unsigned d);
    logic r;
    logic [ALU_MAX_WIDTH-1:0] v_ds1to0;
    logic [ALU_MAX_WIDTH-1:0] v_ds2to0;
    for(int i=0; i<ALU_MAX_WIDTH; i++) begin
      v_ds1to0[i] = (i>d-1) ? 1'b0 : v[i];
      v_ds2to0[i] = (i>d-2) ? 1'b0 : v[i];
    end
    case(vxrm)
    // TODO check d==0,1 condition
      RNU: r = (d == 0) ? 0 : v[d-1];
      RNE: r = (d == 0) ? 0 : ((d == 1) ? v[d-1] && v[d] : v[d-1] && (v_ds2to0 != 0 || v[d]));
      RDN: r = 0;
      ROD: r = (d == 0) ? 0 : !v[d] && (v_ds1to0 != 0);
    endcase
    _roundoff_unsigned = ($unsigned(v) >> d) + r;
    `uvm_info("MDL",$sformatf("_roundoff_unsigned: dest=0x%0x,v=0x%0x,d=0x%0x,r=0x%0x",_roundoff_unsigned,v,d,r),UVM_HIGH)
  endfunction: _roundoff_unsigned

  virtual function alu_signed_t   _roundoff_signed(  alu_signed_t   v, int unsigned d);
    logic r;
    logic [ALU_MAX_WIDTH-1:0] v_ds1to0;
    logic [ALU_MAX_WIDTH-1:0] v_ds2to0;
    for(int i=0; i<ALU_MAX_WIDTH; i++) begin
      v_ds1to0[i] = (i>d-1) ? 1'b0 : v[i];
      v_ds2to0[i] = (i>d-2) ? 1'b0 : v[i];
    end
    case(vxrm)
    // TODO check d==0,1 condition
      RNU: r = (d == 0) ? 0 : v[d-1];
      RNE: r = (d == 0) ? 0 : ((d == 1) ? v[d-1] && v[d] : v[d-1] && (v_ds2to0 != 0 || v[d]));
      RDN: r = 0;
      ROD: r = (d == 0) ? 0 : !v[d] && (v_ds1to0 != 0);
    endcase
    _roundoff_signed = ($signed(v) >>> d) + r;
    `uvm_info("MDL",$sformatf("_roundoff_signed: dest=0x%0x,v=0x%0x,d=0x%0x,r=0x%0x",_roundoff_signed,v,d,r),UVM_HIGH)
  endfunction: _roundoff_signed

  //---------------------------------------------------------------------- 
  // 31.11.1. Vector Single-Width Integer Add and Subtract
  function TD _vadd(T2 src2, T1 src1);
    _vadd = src2 + src1;
  endfunction : _vadd
  function TD _vsub(T2 src2, T1 src1);
    _vsub = src2 - src1;
  endfunction : _vsub
  function TD _vrsub(T2 src2, T1 src1);
    _vrsub = src1 - src2;
  endfunction : _vrsub

  //---------------------------------------------------------------------- 
  // 31.11.2. Vector Widening Integer Add/Subtract
  function TD _vwadd(T2 src2, T1 src1);
    logic signed [$bits(TD)-1:0] dest;
    dest = $signed(src2) + $signed(src1);
    _vwadd = dest;
  endfunction : _vwadd
  function TD _vwaddu(T2 src2, T1 src1);
    logic unsigned [$bits(TD)-1:0] dest;
    dest = $unsigned(src2) + $unsigned(src1);
    _vwaddu = dest;
  endfunction : _vwaddu
  function TD _vwsub(T2 src2, T1 src1);
    logic signed [$bits(TD)-1:0] dest;
    dest = $signed(src2) - $signed(src1);
    _vwsub = dest;
  endfunction : _vwsub
  function TD _vwsubu(T2 src2, T1 src1);
    logic signed [$bits(TD)-1:0] dest;
    dest = $unsigned(src2) - $unsigned(src1);
    _vwsubu = dest;
  endfunction : _vwsubu

  //---------------------------------------------------------------------- 
  // 31.11.3. Vector Integer Extension
  function TD _vzext(T2 src2);
    logic signed [$bits(TD)-1:0] dest;
    dest = $unsigned(src2);
    _vzext = dest;
  endfunction : _vzext
  function TD _vsext(T2 src2);
    logic unsigned [$bits(TD)-1:0] dest;
    dest = $signed(src2);
    _vsext = dest;
  endfunction : _vsext

  //---------------------------------------------------------------------- 
  // 31.11.4. Vector Integer Add-with-Carry / Subtract-with-Borrow Instructions
  function TD _vadc(T2 src2, T1 src1, T0 src0);
    _vadc = src2 + src1 + src0;
  endfunction : _vadc
  function TD _vmadc(T2 src2, T1 src1, T0 src0);
    logic [$bits(T2):0] dest;
    dest = src2 + src1 + src0;
    _vmadc = dest[$bits(T2)];
  endfunction : _vmadc
  function TD _vsbc(T2 src2, T1 src1, T0 src0);
    _vsbc = src2 - src1 - src0;
  endfunction : _vsbc
  function TD _vmsbc(T2 src2, T1 src1, T0 src0);
    logic [$bits(T2):0] dest;
    dest = src2 - src1 - src0;
    _vmsbc = dest[$bits(T2)];
  endfunction : _vmsbc

  //---------------------------------------------------------------------- 
  // 31.11.5. Vector Bitwise Logical Instructions
  // Reuse mask bit logic part.
  function TD _vminu(T2 src2, T1 src1);
    _vminu = $unsigned(src2) > $unsigned(src1) ? $unsigned(src1) : $unsigned(src2);
  endfunction : _vminu
  function TD _vmin(T2 src2, T1 src1);
    _vmin = $signed(src2) > $signed(src1) ? $signed(src1) : $signed(src2);
  endfunction : _vmin
  function TD _vmaxu(T2 src2, T1 src1);
    _vmaxu = $unsigned(src2) > $unsigned(src1) ? $unsigned(src2) : $unsigned(src1);
  endfunction : _vmaxu
  function TD _vmax(T2 src2, T1 src1);
    _vmax = $signed(src2) > $signed(src1) ? $signed(src2) : $signed(src1);
  endfunction : _vmax

  //---------------------------------------------------------------------- 
  // 31.11.6. Vector Single-Width Shift Instructions
  // 31.11.7. Vector Narrowing Integer Right Shift Instructions
  function TD _vsll(T2 src2, T1 src1);
    logic unsigned [$clog2($bits(T2))-1:0] shift_amount;
    shift_amount = src1;
    _vsll = $unsigned(src2) << shift_amount;
  endfunction : _vsll
  function TD _vsrl(T2 src2, T1 src1);
    logic unsigned [$clog2($bits(T2))-1:0] shift_amount;
    shift_amount = src1;
    _vsrl = $unsigned(src2) >> shift_amount;
  endfunction : _vsrl
  function TD _vsra(T2 src2, T1 src1);
    logic unsigned [$clog2($bits(T2))-1:0] shift_amount;
    shift_amount = src1;
    _vsra = $signed(src2) >>> shift_amount;
  endfunction : _vsra

  //---------------------------------------------------------------------- 
  // 31.11.8. Vector Integer Compare Instructions
  function TD _vmseq(T2 src2, T1 src1);
    _vmseq = (src2 === src1);
  endfunction : _vmseq
  function TD _vmsne(T2 src2, T1 src1);
    _vmsne = (src2 !== src1);
  endfunction : _vmsne
  function TD _vmsltu(T2 src2, T1 src1);
    _vmsltu = $unsigned(src2) < $unsigned(src1);
  endfunction : _vmsltu
  function TD _vmslt(T2 src2, T1 src1);
    _vmslt = $signed(src2) < $signed(src1);
  endfunction : _vmslt
  function TD _vmsleu(T2 src2, T1 src1);
    _vmsleu = $unsigned(src2) <= $unsigned(src1);
  endfunction : _vmsleu
  function TD _vmsle(T2 src2, T1 src1);
    _vmsle = $signed(src2) <= $signed(src1);
  endfunction : _vmsle
  function TD _vmsgtu(T2 src2, T1 src1);
    _vmsgtu = $unsigned(src2) > $unsigned(src1);
  endfunction : _vmsgtu
  function TD _vmsgt(T2 src2, T1 src1);
    _vmsgt = $signed(src2) > $signed(src1);
  endfunction : _vmsgt

  //---------------------------------------------------------------------- 
  // Ch31.11.10. Vector Single-Width Integer Multiply Instructions
  function TD _vmul(T2 src2, T1 src1);
    logic signed [$bits(TD)*2-1:0] dest_widen;
    logic signed [$bits(TD)*2-1:0] src2_widen;
    logic signed [$bits(TD)*2-1:0] src1_widen;
    src2_widen = $signed(src2);
    src1_widen = $signed(src1);
    dest_widen = src2_widen * src1_widen;
    _vmul = dest_widen[$bits(TD)-1:0];
  endfunction : _vmul
  function TD _vmulh(T2 src2, T1 src1);
    logic signed [$bits(TD)*2-1:0] dest_widen;
    logic signed [$bits(TD)*2-1:0] src2_widen;
    logic signed [$bits(TD)*2-1:0] src1_widen;
    src2_widen = $signed(src2);
    src1_widen = $signed(src1);
    dest_widen = src2_widen * src1_widen;
    _vmulh = dest_widen[$bits(TD)*2-1:$bits(TD)];
  endfunction : _vmulh
  function TD _vmulhu(T2 src2, T1 src1);
    logic unsigned [$bits(TD)*2-1:0] dest_widen;
    logic unsigned [$bits(TD)*2-1:0] src2_widen;
    logic unsigned [$bits(TD)*2-1:0] src1_widen;
    src2_widen = $unsigned(src2);
    src1_widen = $unsigned(src1);
    dest_widen = src2_widen * src1_widen;
    _vmulhu = dest_widen[$bits(TD)*2-1:$bits(TD)];
  endfunction : _vmulhu
  function TD _vmulhsu(T2 src2, T1 src1);
    logic signed   [$bits(TD)*2-1:0] dest_widen;
    logic signed   [$bits(TD)*2-1:0] src2_widen;
    logic unsigned [$bits(TD)*2-1:0] src1_widen;
    src2_widen = $signed(src2);
    src1_widen = $unsigned(src1);
    dest_widen = src2_widen * src1_widen;
    _vmulhsu = dest_widen[$bits(TD)*2-1:$bits(TD)];
  endfunction : _vmulhsu

  //---------------------------------------------------------------------- 
  // Ch31.11.11. Vector Integer Divide Instructions
  function TD _vdivu(T2 src2, T1 src1);
    logic unsigned [$bits(TD)-1:0] dest;
    dest = $unsigned(src2) / $unsigned(src1);
    _vdivu = (src1 == 0) ? '1 : dest;
  endfunction : _vdivu
  function TD _vdiv(T2 src2, T1 src1);
    logic signed [$bits(TD)-1:0] dest;
    dest = $signed(src2) / $signed(src1);
    _vdiv = (src1 == 0) ? '1 : dest;
  endfunction : _vdiv
  function TD _vremu(T2 src2, T1 src1);
    logic unsigned [$bits(TD)-1:0] dest;
    dest = $unsigned(src2) % $unsigned(src1);
    _vremu = (src1 == 0) ? src2 : dest;
  endfunction : _vremu
  function TD _vrem(T2 src2, T1 src1);
    logic signed [$bits(TD)-1:0] dest;
    dest = $signed(src2) % $signed(src1);
    _vrem = (src1 == 0) ? src2 : dest;
  endfunction : _vrem

  //---------------------------------------------------------------------- 
  // Ch32.11.12. Vector Widening Integer Multiply Instructions
  function TD _vwmul(T2 src2, T1 src1);
    logic signed [$bits(TD)-1:0] dest_widen;
    logic signed [$bits(TD)-1:0] src2_widen;
    logic signed [$bits(TD)-1:0] src1_widen;
    src2_widen = $signed(src2);
    src1_widen = $signed(src1);
    dest_widen = src2_widen * src1_widen;
    _vwmul = dest_widen;
  endfunction : _vwmul
  function TD _vwmulu(T2 src2, T1 src1);
    logic unsigned [$bits(TD)-1:0] dest_widen;
    logic unsigned [$bits(TD)-1:0] src2_widen;
    logic unsigned [$bits(TD)-1:0] src1_widen;
    src2_widen = $unsigned(src2);
    src1_widen = $unsigned(src1);
    dest_widen = src2_widen * src1_widen;
    _vwmulu = dest_widen;
  endfunction : _vwmulu
  function TD _vwmulsu(T2 src2, T1 src1);
    logic signed   [$bits(TD)-1:0] dest_widen;
    logic signed   [$bits(TD)-1:0] src2_widen;
    logic unsigned [$bits(TD)-1:0] src1_widen;
    src2_widen = $signed(src2);
    src1_widen = $unsigned(src1);
    dest_widen = src2_widen * src1_widen;
    _vwmulsu = dest_widen;
  endfunction : _vwmulsu

  //---------------------------------------------------------------------- 
  // Ch31.11.13. Vector Single-Width Integer Multiply-Add Instructions
  function TD _vmacc(TD dest, T2 src2, T1 src1);
    logic signed [$bits(TD)*2-1:0] dest_widen;
    logic signed [$bits(TD)*2-1:0] src2_widen;
    logic signed [$bits(TD)*2-1:0] src1_widen;
    dest_widen = $signed(dest);
    src2_widen = $signed(src2);
    src1_widen = $signed(src1);
    dest_widen = dest_widen + src2_widen * src1_widen;
    _vmacc = dest_widen;
  endfunction : _vmacc
  function TD _vnmsac(TD dest, T2 src2, T1 src1);
    logic signed [$bits(TD)*2-1:0] dest_widen;
    logic signed [$bits(TD)*2-1:0] src2_widen;
    logic signed [$bits(TD)*2-1:0] src1_widen;
    dest_widen = $signed(dest);
    src2_widen = $signed(src2);
    src1_widen = $signed(src1);
    dest_widen = dest_widen - src2_widen * src1_widen;
    _vnmsac = dest_widen;
  endfunction : _vnmsac
  function TD _vmadd(TD dest, T2 src2, T1 src1);
    logic signed [$bits(TD)*2-1:0] dest_widen;
    logic signed [$bits(TD)*2-1:0] src2_widen;
    logic signed [$bits(TD)*2-1:0] src1_widen;
    dest_widen = $signed(dest);
    src2_widen = $signed(src2);
    src1_widen = $signed(src1);
    dest_widen = src2_widen + dest_widen * src1_widen;
    _vmadd = dest_widen;
  endfunction : _vmadd
  function TD _vnmsub(TD dest, T2 src2, T1 src1);
    logic signed [$bits(TD)*2-1:0] dest_widen;
    logic signed [$bits(TD)*2-1:0] src2_widen;
    logic signed [$bits(TD)*2-1:0] src1_widen;
    dest_widen = $signed(dest);
    src2_widen = $signed(src2);
    src1_widen = $signed(src1);
    dest_widen = src2_widen - dest_widen * src1_widen;
    _vnmsub = dest_widen;
  endfunction : _vnmsub

  //---------------------------------------------------------------------- 
  // Ch31.11.14. Vector Widening Integer Multiply-Add Instructions
  function TD _vwmaccu(TD dest, T2 src2, T1 src1);
    logic unsigned [$bits(TD)-1:0] dest_widen;
    logic unsigned [$bits(TD)-1:0] src2_widen;
    logic unsigned [$bits(TD)-1:0] src1_widen;
    dest_widen = $unsigned(dest);
    src2_widen = $unsigned(src2);
    src1_widen = $unsigned(src1);
    dest_widen = dest_widen + src2_widen * src1_widen;
    _vwmaccu = dest_widen;
  endfunction : _vwmaccu
  function TD _vwmacc(TD dest, T2 src2, T1 src1);
    logic signed [$bits(TD)-1:0] dest_widen;
    logic signed [$bits(TD)-1:0] src2_widen;
    logic signed [$bits(TD)-1:0] src1_widen;
    dest_widen = $signed(dest);
    src2_widen = $signed(src2);
    src1_widen = $signed(src1);
    dest_widen = dest_widen + src2_widen * src1_widen;
    _vwmacc = dest_widen;
  endfunction : _vwmacc
  function TD _vwmaccus(TD dest, T2 src2, T1 src1);
    logic signed   [$bits(TD)-1:0] dest_widen;
    logic signed   [$bits(TD)-1:0] src2_widen;
    logic unsigned [$bits(TD)-1:0] src1_widen;
    dest_widen = $signed(dest);
    src2_widen = $signed(src2);
    src1_widen = $unsigned(src1);
    dest_widen = dest_widen + src2_widen * src1_widen;
    _vwmaccus = dest_widen;
  endfunction : _vwmaccus
  function TD _vwmaccsu(TD dest, T2 src2, T1 src1);
    logic signed   [$bits(TD)-1:0] dest_widen;
    logic unsigned [$bits(TD)-1:0] src2_widen;
    logic signed   [$bits(TD)-1:0] src1_widen;
    dest_widen = $signed(dest);
    src2_widen = $unsigned(src2);
    src1_widen = $signed(src1);
    dest_widen = dest_widen + src2_widen * src1_widen;
    _vwmaccsu = dest_widen;
  endfunction : _vwmaccsu

  //---------------------------------------------------------------------- 
  // Ch31.11.15. Vector Integer Merge Instructions
  // Ch31.11.16. Vector Integer Move Instructions
  function TD _vmerge(T2 src2, T1 src1, T0 src0);
    _vmerge = src0 ? src1 : src2;
  endfunction : _vmerge

  //---------------------------------------------------------------------- 
  // Ch31.12.1. Vector Single-Width Saturating Add and Subtract
  function TD _vsaddu(T2 src2, T1 src1);
    {overflow,_vsaddu} = $unsigned(src2) + $unsigned(src1);
    if(overflow) _vsaddu = '1;
  endfunction : _vsaddu
  function TD _vsadd(T2 src2, T1 src1);
    logic signed [$bits(TD):0] dest;
    dest = $signed(src2) + $signed(src1);
    // overflow  = dest[$bits(TD)-1] & ~src2[$bits(T2)-1] & ~src1[$bits(T1)-1]; 
    // underflow = ~dest[$bits(TD)-1] & src2[$bits(T2)-1] & src1[$bits(T1)-1]; 
    overflow  = dest[$bits(TD):$bits(TD)-1] == 2'b01;
    underflow = dest[$bits(TD):$bits(TD)-1] == 2'b10;
    if(overflow)  begin _vsadd = '1; _vsadd[$bits(TD)-1] = 1'b0; end
    else if(underflow) begin _vsadd = '0; _vsadd[$bits(TD)-1] = 1'b1; end
    else begin _vsadd = dest; end
  endfunction : _vsadd
  function TD _vssubu(T2 src2, T1 src1);
    {underflow,_vssubu} = $unsigned(src2) - $unsigned(src1);
    if(underflow) _vssubu = '0;
  endfunction : _vssubu
  function TD _vssub(T2 src2, T1 src1);
    logic signed [$bits(TD):0] dest;
    dest = $signed(src2) - $signed(src1);
    // overflow = dest[$bits(TD)-1] & ~src2[$bits(T2)-1] & src1[$bits(T1)-1]; 
    // underflow  = ~dest[$bits(TD)-1] & src2[$bits(T2)-1] & ~src1[$bits(T1)-1]; 
    overflow  = dest[$bits(TD):$bits(TD)-1] == 2'b01;
    underflow = dest[$bits(TD):$bits(TD)-1] == 2'b10;
    if(overflow)  begin _vssub = '1; _vssub[$bits(TD)-1] = 1'b0; end
    else if(underflow) begin _vssub = '0; _vssub[$bits(TD)-1] = 1'b1; end
    else begin _vssub = dest; end
  endfunction : _vssub

  //---------------------------------------------------------------------- 
  // Ch31.12.2. Vector Single-Width Averaging Add and Subtract
  function TD _vaaddu(T2 src2, T1 src1);
    logic unsigned [$bits(TD):0] dest;
    dest = $unsigned(src2) + $unsigned(src1);
    _vaaddu = _roundoff_unsigned(dest, 1);
  endfunction : _vaaddu
  function TD _vaadd(T2 src2, T1 src1);
    logic signed [$bits(TD):0] dest;
    dest = $signed(src2) + $signed(src1);
    _vaadd = _roundoff_signed(dest, 1);
  endfunction : _vaadd
  function TD _vasubu(T2 src2, T1 src1);
    logic unsigned [$bits(TD):0] dest;
    dest = $unsigned(src2) - $unsigned(src1);
    _vasubu = _roundoff_unsigned(dest, 1);
  endfunction : _vasubu
  function TD _vasub(T2 src2, T1 src1);
    logic signed [$bits(TD):0] dest;
    dest = $signed(src2) - $signed(src1);
    _vasub = _roundoff_signed(dest, 1);
  endfunction : _vasub

  //---------------------------------------------------------------------- 
  // Ch31.12.3. Vector Single-Width Fractional Multiply with Rounding and Saturation
  function TD _vsmul(T2 src2, T1 src1);
    logic signed [$bits(TD)*2-1:0] dest;
    dest = $signed(src2) * $signed(src1);
    dest = _roundoff_signed(dest, $bits(TD)-1);
    overflow  = dest[$bits(TD):$bits(TD)-1] == 2'b01;
    underflow = dest[$bits(TD):$bits(TD)-1] == 2'b10;
    if(overflow) begin _vsmul = '1; _vsmul[$bits(TD)-1] = 1'b0; end
    else if(underflow) begin _vsmul = '0; _vsmul[$bits(TD)-1] = 1'b1; end
    else begin _vsmul = dest; end
  endfunction : _vsmul

  //---------------------------------------------------------------------- 
  // Ch31.12.4. Vector Single-Width Scaling Shift Instructions
  function TD _vssrl(T2 src2, T1 src1);
    logic unsigned [$clog2($bits(T2))-1:0] shift_amount;
    shift_amount = $unsigned(src1);
    _vssrl = _roundoff_unsigned($unsigned(src2), shift_amount);
  endfunction : _vssrl
  function TD _vssra(T2 src2, T1 src1);
    logic unsigned [$clog2($bits(T2))-1:0] shift_amount;
    shift_amount = $unsigned(src1);
    _vssra = _roundoff_signed($signed(src2), shift_amount);
  endfunction : _vssra

  //---------------------------------------------------------------------- 
  // Ch31.12.5 Vector Narrowing Fixed-Point Clip Instructions
  function TD _vnclipu(T2 src2, T1 src1);
    logic unsigned [$bits(TD)*2-1:0] dest_widen;
    logic unsigned [$clog2($bits(T2))-1:0] shift_amount;
    shift_amount = $unsigned(src1);
    dest_widen = _roundoff_unsigned($unsigned(src2), shift_amount);
    `uvm_info("MDL",$sformatf("dest_widen(%0dbits) = 0x%x",$bits(dest_widen),dest_widen),UVM_HIGH)
    overflow = |dest_widen[$bits(TD)*2-1:$bits(TD)];
    `uvm_info("MDL",$sformatf("overflow = %0d",overflow),UVM_HIGH)
    if(overflow) begin _vnclipu = '1; end
    else begin _vnclipu = dest_widen; end
  endfunction
  function TD _vnclip(T2 src2, T1 src1);
    logic signed [$bits(TD)*2:0] dest_widen;
    logic unsigned [$clog2($bits(T2))-1:0] shift_amount;
    shift_amount = $unsigned(src1);
    dest_widen = _roundoff_signed($signed(src2), shift_amount);
    `uvm_info("MDL",$sformatf("dest_widen(%0dbits) = 0x%x",$bits(dest_widen),dest_widen),UVM_HIGH)
    underflow =  dest_widen[$bits(TD)*2] && (~(&dest_widen[$bits(_vnclip)*2-1:$bits(TD)]));
    overflow  = ~dest_widen[$bits(TD)*2] && |dest_widen[$bits(_vnclip)*2-1:$bits(TD)];
    if(overflow)  begin _vnclip = '1; _vnclip[$bits(TD)-1] = 1'b0; end
    else if(underflow) begin _vnclip = '0; _vnclip[$bits(TD)-1] = 1'b1; end
    else begin _vnclip = dest_widen; end
  endfunction

  //---------------------------------------------------------------------- 
  // 31.14.1. Vector Single-Width Integer Reduction Instructions
  function TD _vredsum(TD dest, T2 src2);
    logic signed [$bits(TD)-1:0] dest_temp;
    dest_temp = dest + src2;
    _vredsum = dest_temp;
  endfunction:_vredsum
  function TD _vredand(TD dest, T2 src2);
    logic signed [$bits(TD)-1:0] dest_temp;
    dest_temp = dest & src2;
    _vredand = dest_temp;
  endfunction:_vredand
  function TD _vredor(TD dest, T2 src2);
    logic signed [$bits(TD)-1:0] dest_temp;
    dest_temp = dest | src2;
    _vredor = dest_temp;
  endfunction:_vredor
  function TD _vredxor(TD dest, T2 src2);
    logic signed [$bits(TD)-1:0] dest_temp;
    dest_temp = dest ^ src2;
    _vredxor = dest_temp;
  endfunction:_vredxor
  function TD _vredminu(TD dest, T2 src2);
    logic unsigned [$bits(TD)-1:0] dest_temp;
    dest_temp = $unsigned(dest) < $unsigned(src2) ? dest : src2;
    _vredminu = dest_temp;
  endfunction:_vredminu
  function TD _vredmin(TD dest, T2 src2);
    logic signed [$bits(TD)-1:0] dest_temp;
    dest_temp = $signed(dest) < $signed(src2) ? dest : src2;
    _vredmin = dest_temp;
  endfunction:_vredmin
  function TD _vredmaxu(TD dest, T2 src2);
    logic unsigned [$bits(TD)-1:0] dest_temp;
    dest_temp = $unsigned(dest) > $unsigned(src2) ? dest : src2;
    _vredmaxu = dest_temp;
  endfunction:_vredmaxu
  function TD _vredmax(TD dest, T2 src2);
    logic signed [$bits(TD)-1:0] dest_temp;
    dest_temp = $signed(dest) > $signed(src2) ? dest : src2;
    _vredmax = dest_temp;
  endfunction:_vredmax

  //---------------------------------------------------------------------- 
  // 31.14.2. Vector Widening Integer Reduction Instructions
  function TD _vwredsum(TD dest, T2 src2);
    logic signed [$bits(TD)-1:0] dest_temp;
    dest_temp = $signed(dest) + $signed(src2);
    _vwredsum = dest_temp;
  endfunction:_vwredsum
  function TD _vwredsumu(TD dest, T2 src2);
    logic unsigned [$bits(TD)-1:0] dest_temp;
    dest_temp = $unsigned(dest) + $unsigned(src2);
    _vwredsumu = dest_temp;
  endfunction:_vwredsumu

  //---------------------------------------------------------------------- 
  // Ch31.15.1. Vector Mask-Register Logical Instructions
  function TD _vmand(T2 src2, T1 src1);
    _vmand = src2 & src1;
  endfunction : _vmand
  function TD _vmor(T2 src2, T1 src1);
    _vmor = src2 | src1;
  endfunction : _vmor
  function TD _vmxor(T2 src2, T1 src1);
    _vmxor = src2 ^ src1;
  endfunction : _vmxor
  function TD _vmorn(T2 src2, T1 src1);
    _vmorn = src2 | ~src1;
  endfunction : _vmorn
  function TD _vmnand(T2 src2, T1 src1);
    _vmnand = ~(src2 & src1);
  endfunction : _vmnand
  function TD _vmnor(T2 src2, T1 src1);
    _vmnor = ~(src2 | src1);
  endfunction : _vmnor
  function TD _vmandn(T2 src2, T1 src1);
    _vmandn = src2 & ~src1;
  endfunction : _vmandn
  function TD _vmxnor(T2 src2, T1 src1);
    _vmxnor = ~(src2 ^ src1);
  endfunction : _vmxnor 

  //---------------------------------------------------------------------- 
  // 31.15.2. Vector count population in mask vcpop.m
  function TD _vcpop(T2 src2);
    this.mask_count += src2;
    _vcpop = this.mask_count;
  endfunction: _vcpop

  //---------------------------------------------------------------------- 
  // 32.15.3. vfirst find-first-set mask bit
  function TD _vfirst(T2 src2);
    if(src2 && !this.found_first_mask) begin
      this.found_first_mask = 1'b1;
      this.first_mask_idx = this.elm_idx;
    end else begin
      this.first_mask_idx = this.first_mask_idx;
    end
    _vfirst = this.first_mask_idx;
    `uvm_info("MDL", $sformatf("found_first_mask=%0d, first_mask_idx=%0d, elm_idx=%0d", 
                                found_first_mask, first_mask_idx, elm_idx),UVM_HIGH)
  endfunction: _vfirst

  //---------------------------------------------------------------------- 
  // 32.15.4. vmsbf.m set-before-first mask bit
  function TD _vmsbf(T2 src2);
    if(src2 & ~found_first_mask) begin
      found_first_mask = 1'b1; 
    end
    if(~found_first_mask) begin
      _vmsbf = 1'b1;
    end else begin
      _vmsbf = 1'b0;
    end
  endfunction: _vmsbf

  //---------------------------------------------------------------------- 
  // 32.15.5. vmsif.m set-including-first mask bit
  function TD _vmsif(T2 src2);
    if(!found_first_mask) begin
      _vmsif = 1'b1;
    end else begin
      _vmsif = 1'b0;
    end
    if(src2 & ~found_first_mask) begin 
      found_first_mask = 1'b1; 
    end
  endfunction: _vmsif

  //---------------------------------------------------------------------- 
  // 32.15.6. vmsof.m set-only-first mask bit
  function TD _vmsof(T2 src2);
    if(src2 & ~found_first_mask) begin 
      found_first_mask = 1'b1; 
      _vmsof = 1'b1;
    end else begin
      _vmsof = 1'b0;
    end
  endfunction: _vmsof

  //---------------------------------------------------------------------- 
  // 31.15.8. Vector Iota Instruction
  function TD _viota(T2 src2);
    if(this.elm_idx == 0) begin
      this.mask_count = 0;
    end
    _viota = this.mask_count;
    this.mask_count += src2;
  endfunction: _viota

  //---------------------------------------------------------------------- 
  // 31.15.9. Vector Element Index Instruction
  function TD _vid();
    _vid = elm_idx;
  endfunction: _vid

endclass: alu_processor

// LSU inst part ------------------------------------------
class lsu_processor;  

  int dest_eew; real dest_emul;
  int src3_eew; real src3_emul;
  int src2_eew; real src2_emul;
  int src1_eew; real src1_emul;
  int src0_eew; real src0_emul;

  int seg_size; // byte size

  vrf_t [31:0] vrf_temp;

  int dest_reg_idx_base = 0;
  int src3_reg_idx_base = 0;
  int src2_reg_idx_base = 0;
  int src1_reg_idx_base = 0;
  
  sew_max_t dest;
  sew_max_t src0;
  sew_max_t src1;
  sew_max_t src2;
  sew_max_t src3;

  int lsu_nf;
  int vl, vstart, evl;
  
  int address;

  bit fraction_lmul;
  int  sew;
  real lmul;
  int elm_idx_max;
  int data_size, vidx_size; // byte size

  bit vm;

  function new();
  endfunction: new 
  
  function void exe(rvv_behavior_model rvm, ref rvs_transaction inst_tr);

    decode(inst_tr);
    `uvm_info("MDL", "LSU decode done", UVM_HIGH)

    for(int seg_idx=0; seg_idx<lsu_nf+1; seg_idx++) begin
      dest_reg_idx_base = inst_tr.dest_idx + seg_idx * int'($ceil(dest_emul));
      src3_reg_idx_base = inst_tr.src3_idx + seg_idx * int'($ceil(src3_emul));
      src2_reg_idx_base = inst_tr.src2_idx + seg_idx * int'($ceil(src2_emul));
      src1_reg_idx_base = inst_tr.src1_idx + seg_idx * int'($ceil(src1_emul));

      for(int elm_idx=0; elm_idx<elm_idx_max; elm_idx++) begin
        // fetch
        dest = rvm.elm_fetch(inst_tr.dest_type, dest_reg_idx_base, elm_idx, dest_eew);
        src3 = rvm.elm_fetch(inst_tr.src3_type, src3_reg_idx_base, elm_idx, src3_eew); 
        src2 = rvm.elm_fetch(inst_tr.src2_type, src2_reg_idx_base, elm_idx, src2_eew); 
        src1 = rvm.elm_fetch(inst_tr.src1_type, src1_reg_idx_base, elm_idx, src1_eew); 
        src0 = rvm.elm_fetch(VRF, 0, elm_idx, src0_eew);
        
        `uvm_info("MDL", "\n---------------------------------------------------------------------------------------------------------------------------------\n", UVM_LOW)

        // TODO:  fix negetive stride
        update_addr(inst_tr, seg_idx, elm_idx, data_size, src2, src1);
        if(elm_idx<vstart) begin
          // pre-start
        end else if(elm_idx >= evl) begin
          // tail
        end else if(!(vm || src0)) begin
          // body-inactive
        end else begin
          case(inst_tr.inst_type)
            LD: begin
              load_from_mem(dest, this.address, data_size, rvm.mem);
              rvm.elm_writeback(dest, inst_tr.dest_type, dest_reg_idx_base, elm_idx, dest_eew);
            end
            ST: begin
              store_to_mem(src3, this.address, data_size, rvm.mem);
            end
          endcase
        end

        `uvm_info("MDL", "\n---------------------------------------------------------------------------------------------------------------------------------\n", UVM_LOW)
      end
    end
  endfunction

  function bit decode(ref rvs_transaction inst_tr);
  
    sew = 8 << inst_tr.vsew;
    fraction_lmul = inst_tr.vlmul[2];
    lmul = 2.0 ** $signed(inst_tr.vlmul);
    elm_idx_max = fraction_lmul ?        `VLEN / sew: 
                                  lmul * `VLEN / sew;

    vl = inst_tr.vl;
    vstart = inst_tr.vstart;
    vm = inst_tr.vm;

    dest_eew  = EEW_NONE;
    src3_eew  = EEW_NONE;
    src2_eew  = EEW_NONE;
    src1_eew  = EEW_NONE;
    src0_eew  = EEW1;
    dest_emul = EMUL_NONE;
    src3_emul = EMUL_NONE;
    src2_emul = EMUL_NONE;
    src1_emul = EMUL_NONE;
    src0_emul = EMUL1;
    case(inst_tr.inst_type)
      LD: begin
        case(inst_tr.lsu_mop) 
          LSU_E   : begin
            case(inst_tr.lsu_umop)
              MASK: begin
                dest_eew  = EEW8;
                dest_emul = EMUL1;
                src2_eew  = EEW_NONE;
                src2_emul = EMUL_NONE;
                src1_eew  = EEW32;
                src1_emul = EMUL1;
                evl = int'($ceil(inst_tr.vl / 8.0));
              end
              default: begin
                dest_eew  = inst_tr.lsu_eew;
                dest_emul = dest_eew * lmul / sew;
                src2_eew  = EEW_NONE;
                src2_emul = EMUL_NONE;
                src1_eew  = EEW32;
                src1_emul = EMUL1;
                evl = inst_tr.vl;
              end
            endcase
          end
          LSU_SE  : begin
            dest_eew  = inst_tr.lsu_eew;
            dest_emul = dest_eew * lmul / sew;
            src2_eew  = EEW32;
            src2_emul = EMUL1;
            src1_eew  = EEW32;
            src1_emul = EMUL1;
            evl = inst_tr.vl;
          end
          LSU_UXEI, 
          LSU_OXEI: begin
            dest_eew  = sew;
            dest_emul = lmul;
            src2_eew  = inst_tr.lsu_eew;
            src2_emul = dest_eew * lmul / sew;
            src1_eew  = EEW32;
            src1_emul = EMUL1;
            evl = inst_tr.vl;
          end      
        endcase
        lsu_nf = inst_tr.lsu_nf;
        seg_size = (lsu_nf+1) * dest_eew / 8;
        data_size = dest_eew / 8;
        vidx_size = src2_eew / 8;
      end
      ST: begin
        case(inst_tr.lsu_mop) 
          LSU_E   : begin
            case(inst_tr.lsu_umop)
              MASK: begin
                src3_eew  = EEW8;
                src3_emul = EMUL1;
                src2_eew  = EEW_NONE;
                src2_emul = EMUL_NONE;
                src1_eew  = EEW32;
                src1_emul = EMUL1;
                evl = int'($ceil(inst_tr.vl / 8.0));
              end
              default: begin
                src3_eew  = inst_tr.lsu_eew;
                src3_emul = dest_eew * lmul / sew;
                src2_eew  = EEW_NONE;
                src2_emul = EMUL_NONE;
                src1_eew  = EEW32;
                src1_emul = EMUL1;
                evl = inst_tr.vl;
              end
            endcase
          end
          LSU_SE  : begin
            src3_eew  = inst_tr.lsu_eew;
            src3_emul = src3_eew * lmul / sew;
            src2_eew  = EEW32;
            src2_emul = EMUL1;
            src1_eew  = EEW32;
            src1_emul = EMUL1;
            evl = inst_tr.vl;
          end
          LSU_UXEI, 
          LSU_OXEI: begin
            src3_eew  = sew;
            src3_emul = lmul;
            src2_eew  = inst_tr.lsu_eew;
            src2_emul = src3_eew * lmul / sew;
            src1_eew  = EEW32;
            src1_emul = EMUL1;
            evl = inst_tr.vl;
          end      
        endcase
        lsu_nf = inst_tr.lsu_nf;
        seg_size = (lsu_nf+1) * src3_eew / 8;
        data_size = src3_eew / 8;
        vidx_size = src2_eew / 8;
      end
    endcase


    return 0;
  endfunction: decode

  function void update_addr(rvs_transaction inst_tr, int seg_idx, int elm_idx, int elm_size, sew_max_t src2, sew_max_t src1);
    case(inst_tr.lsu_mop) 
      LSU_E   : begin
        // TODO: Whole reg?
        this.address = src1 + seg_size * elm_idx + elm_size * seg_idx;
      end
      LSU_SE  : begin
        this.address = src1 + src2 * elm_idx + elm_size * seg_idx; 
      end
      LSU_UXEI, 
      LSU_OXEI: begin
        this.address = src1 + src2 + elm_size * seg_idx;
      end      
    endcase
  endfunction: update_addr

  task load_from_mem(
    output sew_max_t load_data, 
    input int address, 
    input int byte_size, 
    const ref byte mem[int unsigned]
    ); 
    
    load_data = 'x;
    for(int byte_cnt=0; byte_cnt<byte_size; byte_cnt++) begin
      load_data[byte_cnt*8 +: 8] = mem[address+byte_cnt];
    end
    `uvm_info("MDL/LSU",$sformatf("Load %0d bytes @0x%8x: 0x%x", byte_size, address, load_data), UVM_LOW)
  endtask: load_from_mem

  task store_to_mem(
    input sew_max_t store_data,
    input address, 
    input byte_size,
    ref byte mem[int unsigned]
    ); 
    
    for(int byte_cnt=0; byte_cnt<byte_size; byte_cnt++) begin
      mem[address+byte_cnt] = store_data[byte_cnt*8 +: 8];
    end
    `uvm_info("MDL/LSU",$sformatf("Store %0d bytes @0x%8x: 0x%x", byte_size, address, store_data), UVM_LOW)
  endtask: store_to_mem

endclass: lsu_processor 

// RDT inst part ---------------------------------
class rdt_processor#(
    type TD = sew8_t,
    type T1 = sew8_t,
    type T2 = sew8_t
        );
    
    int dest_reg_idx, dest_eew;
    int src0_reg_idx, src0_eew;
    int src1_reg_idx, src1_eew;
    int src2_reg_idx, src2_eew;
    bit vm;
    logic [511:0] [7:0]  vrf_sew8;
    logic [255:0] [15:0] vrf_sew16;
    logic [127:0] [31:0] vrf_sew32;
    function new();
            $display("This inst is rdt_processor\n");
    endfunction
    function rdt_func(input rvv_behavior_model rvv_behavior_model_inst, input rvs_transaction inst_tr);
        vrf_sew8 = rvv_behavior_model_inst.vrf;
        vrf_sew16 = rvv_behavior_model_inst.vrf;
        vrf_sew32 = rvv_behavior_model_inst.vrf;
        case({inst_tr.alu_inst,inst_tr.alu_type})
          {VREDSUM,OPMVV}:RDT_sum(rvv_behavior_model_inst);
          {VREDMAXU,OPMVV}:RDT_maxu(rvv_behavior_model_inst);
          {VREDMAX,OPMVV}:RDT_max(rvv_behavior_model_inst);
          {VREDMINU,OPMVV}:RDT_minu(rvv_behavior_model_inst);
          {VREDMIN,OPMVV}:RDT_min(rvv_behavior_model_inst);
          {VREDAND,OPMVV}:RDT_and(rvv_behavior_model_inst);
          {VREDOR,OPMVV}:RDT_or(rvv_behavior_model_inst);
          {VREDXOR,OPMVV}:RDT_xor(rvv_behavior_model_inst);
          {VWREDSUMU,OPIVV}:RDT_wsumu(rvv_behavior_model_inst);
          {VWREDSUM,OPIVV}:RDT_wsum(rvv_behavior_model_inst);
          default: `uvm_info("rdt_processor",$sformatf("Not invaild instruction."),UVM_LOW)
        endcase
    endfunction

    extern function void RDT_INIT(input rvv_behavior_model rvm, input rvs_transaction inst_tr);
    extern function void RDT_sum(input rvv_behavior_model rvm);
    extern function void RDT_maxu(input rvv_behavior_model rvm);
    extern function void RDT_max(input rvv_behavior_model rvm);
    extern function void RDT_minu(input rvv_behavior_model rvm);
    extern function void RDT_min(input rvv_behavior_model rvm);
    extern function void RDT_and(input rvv_behavior_model rvm);
    extern function void RDT_or(input rvv_behavior_model rvm);
    extern function void RDT_xor(input rvv_behavior_model rvm);
    extern function void RDT_wsumu(input rvv_behavior_model rvm);
    extern function void RDT_wsum(input rvv_behavior_model rvm);
    static function TD exe (rvs_transaction inst_tr, T1 src1, T2 src2);
        TD dest;
        exe = dest;
    endfunction
endclass: rdt_processor

    function void rdt_processor::RDT_INIT(input rvv_behavior_model rvm, input rvs_transaction inst_tr);
            dest_reg_idx = inst_tr.dest_idx;
            src2_reg_idx = inst_tr.src2_idx;
            src1_reg_idx = inst_tr.src1_idx;
            dest_eew = rvm.vsew;
            src2_eew= rvm.vsew;
            src1_eew = rvm.vsew;
            vm = inst_tr.vm;
            `uvm_info("RDT","rdt init",UVM_LOW)
        endfunction


    function void rdt_processor::RDT_sum(input rvv_behavior_model rvm);
            logic [31:0] acc;
          if(dest_eew == EEW8) begin
            acc = vrf_sew8[src1_reg_idx*16];
            for(int i=0;i<rvm.vlmax;i++) begin
                if(i < rvm.vstart) begin
                end
                else if(i<rvm.vl) begin
                        acc = acc + {(vm&&rvm.vrf[0][i]) ? vrf_sew8[src2_reg_idx*16+i] : 0};
                end
            end
            vrf_sew8[dest_reg_idx*16] = acc[7:0];
        end
        else if(dest_eew == EEW16) begin
            acc = vrf_sew16[src1_reg_idx*8];
            for(int i=0;i<rvm.vlmax;i++) begin
                if(i < rvm.vstart) begin
                end
                else if(i<rvm.vl) begin
                        acc = acc + {(vm&&rvm.vrf[0][i]) ? vrf_sew16[src2_reg_idx*8+i] : 0};
                end
            end
            vrf_sew16[dest_reg_idx*8] = acc[15:0];
        end
        else begin
            acc = vrf_sew32[src1_reg_idx*4];
            for(int i=0;i<rvm.vlmax;i++) begin
                if(i < rvm.vstart) begin
                end
                else if(i<rvm.vl) begin
                        acc = acc + {(vm&&rvm.vrf[0][i]) ? vrf_sew32[src2_reg_idx*4+i] : 0};
                end
            end
            vrf_sew32[dest_reg_idx*4] = acc[31:0];
        end
    endfunction

    function void rdt_processor::RDT_maxu(input rvv_behavior_model rvm);
            logic [31:0] acc;
          if(dest_eew == EEW8) begin
            acc = vrf_sew8[src1_reg_idx*16];
            for(int i=0;i<rvm.vlmax;i++) begin
                if(i < rvm.vstart) begin
                end
                else if(i<rvm.vl) begin
                        acc = (acc >= {(vm&&rvm.vrf[0][i]) ? vrf_sew8[src2_reg_idx*16+i] : 0}) ? acc :vrf_sew8[src2_reg_idx*16+i];
                end
            end
            vrf_sew8[dest_reg_idx*16] = acc[7:0];
        end
        else if(dest_eew == EEW16) begin
            acc = vrf_sew16[src1_reg_idx*8];
            for(int i=0;i<rvm.vlmax;i++) begin
                if(i < rvm.vstart) begin
                end
                else if(i<rvm.vl) begin
                        acc = (acc >= {(vm&&rvm.vrf[0][i]) ? vrf_sew16[src2_reg_idx*8+i] : 0}) ? acc :vrf_sew16[src2_reg_idx*8+i];
                end
            end
            vrf_sew16[dest_reg_idx*8] = acc[15:0];
        end
        else begin
            acc = vrf_sew32[src1_reg_idx*4];
            for(int i=0;i<rvm.vlmax;i++) begin
                if(i < rvm.vstart) begin
                end
                else if(i<rvm.vl) begin
                        acc = (acc >= {(vm&&rvm.vrf[0][i]) ? vrf_sew32[src2_reg_idx*4+i] : 0}) ? acc :vrf_sew32[src2_reg_idx*4+i];
                end
            end
            vrf_sew32[dest_reg_idx*4] = acc[31:0];
        end
    endfunction

    function void rdt_processor::RDT_max(input rvv_behavior_model rvm);
            logic [31:0] acc;
          if(dest_eew == EEW8) begin
            acc = vrf_sew8[src1_reg_idx*16];
            for(int i=0;i<rvm.vlmax;i++) begin
                if(i < rvm.vstart) begin
                end
                else if(i<rvm.vl) begin
                        acc = (signed'(acc) >= signed'({(vm&&rvm.vrf[0][i]) ? vrf_sew8[src2_reg_idx*16+i] : 0})) ? acc :vrf_sew8[src2_reg_idx*16+i];
                end
            end
            vrf_sew8[dest_reg_idx*16] = acc[7:0];
        end
        else if(dest_eew == EEW16) begin
            acc = vrf_sew16[src1_reg_idx*8];
            for(int i=0;i<rvm.vlmax;i++) begin
                if(i < rvm.vstart) begin
                end
                else if(i<rvm.vl) begin
                        acc = (signed'(acc) >= signed'({(vm&&rvm.vrf[0][i]) ? vrf_sew16[src2_reg_idx*8+i] : 0})) ? acc :vrf_sew16[src2_reg_idx*8+i];
                end
            end
            vrf_sew16[dest_reg_idx*8] = acc[15:0];
        end
        else begin
            acc = vrf_sew32[src1_reg_idx*4];
            for(int i=0;i<rvm.vlmax;i++) begin
                if(i < rvm.vstart) begin
                end
                else if(i<rvm.vl) begin
                        acc = (signed'(acc) >= signed'({(vm&&rvm.vrf[0][i]) ? vrf_sew32[src2_reg_idx*4+i] : 0})) ? acc :vrf_sew32[src2_reg_idx*4+i];
                end
            end
            vrf_sew32[dest_reg_idx*4] = acc[31:0];
        end
endfunction

    function void rdt_processor::RDT_minu(input rvv_behavior_model rvm);
            logic [31:0] acc;
          if(dest_eew == EEW8) begin
            acc = vrf_sew8[src1_reg_idx*16];
            for(int i=0;i<rvm.vlmax;i++) begin
                if(i < rvm.vstart) begin
                end
                else if(i<rvm.vl) begin
                        acc = (acc <= {(vm&&rvm.vrf[0][i]) ? vrf_sew8[src2_reg_idx*16+i] : 0}) ? acc :vrf_sew8[src2_reg_idx*16+i];
                end
            end
            vrf_sew8[dest_reg_idx*16] = acc[7:0];
        end
        else if(dest_eew == EEW16) begin
            acc = vrf_sew16[src1_reg_idx*8];
            for(int i=0;i<rvm.vlmax;i++) begin
                if(i < rvm.vstart) begin
                end
                else if(i<rvm.vl) begin
                        acc = (acc <= {(vm&&rvm.vrf[0][i]) ? vrf_sew16[src2_reg_idx*8+i] : 0}) ? acc :vrf_sew16[src2_reg_idx*8+i];
                end
            end
            vrf_sew16[dest_reg_idx*8] = acc[15:0];
        end
        else begin
            acc = vrf_sew32[src1_reg_idx*4];
            for(int i=0;i<rvm.vlmax;i++) begin
                if(i < rvm.vstart) begin
                end
                else if(i<rvm.vl) begin
                        acc = (acc <= {(vm&&rvm.vrf[0][i]) ? vrf_sew32[src2_reg_idx*4+i] : 0}) ? acc :vrf_sew32[src2_reg_idx*4+i];
                end
            end
            vrf_sew32[dest_reg_idx*4] = acc[31:0];
        end
    endfunction
    
    function void rdt_processor::RDT_min(input rvv_behavior_model rvm);
            logic [31:0] acc;
          if(dest_eew == EEW8) begin
            acc = vrf_sew8[src1_reg_idx*16];
            for(int i=0;i<rvm.vlmax;i++) begin
                if(i < rvm.vstart) begin
                end
                else if(i<rvm.vl) begin
                        acc = (signed'(acc) <= signed'({(vm&&rvm.vrf[0][i]) ? vrf_sew8[src2_reg_idx*16+i] : 0})) ? acc :vrf_sew8[src2_reg_idx*16+i];
                end
            end
            vrf_sew8[dest_reg_idx*16] = acc[7:0];
        end
        else if(dest_eew == EEW16) begin
            acc = vrf_sew16[src1_reg_idx*8];
            for(int i=0;i<rvm.vlmax;i++) begin
                if(i < rvm.vstart) begin
                end
                else if(i<rvm.vl) begin
                        acc = (signed'(acc) <= signed'({(vm&&rvm.vrf[0][i]) ? vrf_sew16[src2_reg_idx*8+i] : 0})) ? acc :vrf_sew16[src2_reg_idx*8+i];
                end
            end
            vrf_sew16[dest_reg_idx*8] = acc[15:0];
        end
        else begin
            acc = vrf_sew32[src1_reg_idx*4];
            for(int i=0;i<rvm.vlmax;i++) begin
                if(i < rvm.vstart) begin
                end
                else if(i<rvm.vl) begin
                        acc = (signed'(acc) <= signed'({(vm&&rvm.vrf[0][i]) ? vrf_sew32[src2_reg_idx*4+i] : 0})) ? acc :vrf_sew32[src2_reg_idx*4+i];
                end
            end
            vrf_sew32[dest_reg_idx*4] = acc[31:0];
        end
        endfunction

        function void rdt_processor::RDT_and(input rvv_behavior_model rvm);
            logic [31:0] acc;
          if(dest_eew == EEW8) begin
            acc = vrf_sew8[src1_reg_idx*16];
            for(int i=0;i<rvm.vlmax;i++) begin
                if(i < rvm.vstart) begin
                end
                else if(i<rvm.vl) begin
                        acc = acc && vrf_sew8[src2_reg_idx*16+i];
                end
            end
            vrf_sew8[dest_reg_idx*16] = acc[7:0];
        end
        else if(dest_eew == EEW16) begin
            acc = vrf_sew16[src1_reg_idx*8];
            for(int i=0;i<rvm.vlmax;i++) begin
                if(i < rvm.vstart) begin
                end
                else if(i<rvm.vl) begin
                        acc = acc && vrf_sew16[src2_reg_idx*8+i];
                end
            end
            vrf_sew16[dest_reg_idx*8] = acc[15:0];
        end
        else begin
            acc = vrf_sew32[src1_reg_idx*4];
            for(int i=0;i<rvm.vlmax;i++) begin
                if(i < rvm.vstart) begin
                end
                else if(i<rvm.vl) begin
                        acc = acc && vrf_sew32[src2_reg_idx*4+i];
                end
            end
            vrf_sew32[dest_reg_idx*4] = acc[31:0];
        end
    endfunction

    function void rdt_processor::RDT_or(input rvv_behavior_model rvm);
            logic [31:0] acc;
          if(dest_eew == EEW8) begin
            acc = vrf_sew8[src1_reg_idx*16];
            for(int i=0;i<rvm.vlmax;i++) begin
                if(i < rvm.vstart) begin
                end
                else if(i<rvm.vl) begin
                        acc = acc | vrf_sew8[src2_reg_idx*16+i];
                end
            end
            vrf_sew8[dest_reg_idx*16] = acc[7:0];
        end
        else if(dest_eew == EEW16) begin
            acc = vrf_sew16[src1_reg_idx*8];
            for(int i=0;i<rvm.vlmax;i++) begin
                if(i < rvm.vstart) begin
                end
                else if(i<rvm.vl) begin
                        acc = acc | vrf_sew16[src2_reg_idx*8+i];
                end
            end
            vrf_sew16[dest_reg_idx*8] = acc[15:0];
        end
        else begin
            acc = vrf_sew32[src1_reg_idx*4];
            for(int i=0;i<rvm.vlmax;i++) begin
                if(i < rvm.vstart) begin
                end
                else if(i<rvm.vl) begin
                        acc = acc | vrf_sew32[src2_reg_idx*4+i];
                end
            end
            vrf_sew32[dest_reg_idx*4] = acc[31:0];
        end
      endfunction

        function void rdt_processor::RDT_xor(input rvv_behavior_model rvm);
            logic [31:0] acc;
          if(dest_eew == EEW8) begin
            acc = vrf_sew8[src1_reg_idx*16];
            for(int i=0;i<rvm.vlmax;i++) begin
                if(i < rvm.vstart) begin
                end
                else if(i<rvm.vl) begin
                        acc = acc ^ vrf_sew8[src2_reg_idx*16+i];
                end
            end
            vrf_sew8[dest_reg_idx*16] = acc[7:0];
        end
        else if(dest_eew == EEW16) begin
            acc = vrf_sew16[src1_reg_idx*8];
            for(int i=0;i<rvm.vlmax;i++) begin
                if(i < rvm.vstart) begin
                end
                else if(i<rvm.vl) begin
                        acc = acc ^ vrf_sew16[src2_reg_idx*8+i];
                end
            end
            vrf_sew16[dest_reg_idx*8] = acc[15:0];
        end
        else begin
            acc = vrf_sew32[src1_reg_idx*4];
            for(int i=0;i<rvm.vlmax;i++) begin
                if(i < rvm.vstart) begin
                end
                else if(i<rvm.vl) begin
                        acc = acc ^ vrf_sew32[src2_reg_idx*4+i];
                end
            end
            vrf_sew32[dest_reg_idx*4] = acc[31:0];
        end
    endfunction

    function void rdt_processor::RDT_wsumu(input rvv_behavior_model rvm);
            logic [63:0] acc;
          if(src2_eew == EEW8) begin
            acc = vrf_sew8[src1_reg_idx*16];
            for(int i=0;i<rvm.vlmax;i++) begin
                if(i < rvm.vstart) begin
                end
                else if(i<rvm.vl) begin
                        acc = acc + {(vm&&rvm.vrf[0][i]) ? {8'd0,vrf_sew8[src2_reg_idx*16+i]} : 0};
                end
            end
            vrf_sew16[dest_reg_idx*16] = acc[7:0];
        end
        else if(src2_eew == EEW16) begin
            acc = vrf_sew16[src1_reg_idx*8];
            for(int i=0;i<rvm.vlmax;i++) begin
                if(i < rvm.vstart) begin
                end
                else if(i<rvm.vl) begin
                        acc = acc + {(vm&&rvm.vrf[0][i]) ? {16'd0,vrf_sew16[src2_reg_idx*8+i]} : 0};
                end
            end
            vrf_sew32[dest_reg_idx*8] = acc[31:0];
        end
        else begin
            acc = vrf_sew32[src1_reg_idx*4];
            for(int i=0;i<rvm.vlmax;i++) begin
                if(i < rvm.vstart) begin
                end
                else if(i<rvm.vl) begin
                        acc = acc + {(vm&&rvm.vrf[0][i]) ? {32'd0,vrf_sew32[src2_reg_idx*4+i]} : 0};
                end
            end
            vrf_sew32[dest_reg_idx*4] = acc[31:0];
            vrf_sew32[dest_reg_idx*4+1] = acc[63:32];
        end
    endfunction

    function void rdt_processor::RDT_wsum(input rvv_behavior_model rvm);
            logic [63:0] acc;
          if(src2_eew == EEW8) begin
            acc = vrf_sew8[src1_reg_idx*16];
            for(int i=0;i<rvm.vlmax;i++) begin
                if(i < rvm.vstart) begin
                end
                else if(i<rvm.vl) begin
                        acc = acc + {(vm&&rvm.vrf[0][i]) ? {8'hFF,vrf_sew8[src2_reg_idx*16+i]} : 0};
                end
            end
            vrf_sew16[dest_reg_idx*16] = acc[7:0];
        end
        else if(src2_eew == EEW16) begin
            acc = vrf_sew16[src1_reg_idx*8];
            for(int i=0;i<rvm.vlmax;i++) begin
                if(i < rvm.vstart) begin
                end
                else if(i<rvm.vl) begin
                        acc = acc + {(vm&&rvm.vrf[0][i]) ? {16'hFF,vrf_sew16[src2_reg_idx*8+i]} : 0};
                end
            end
            vrf_sew32[dest_reg_idx*8] = acc[31:0];
        end
        else begin
            acc = vrf_sew32[src1_reg_idx*4];
            for(int i=0;i<rvm.vlmax;i++) begin
                if(i < rvm.vstart) begin
                end
                else if(i<rvm.vl) begin
                        acc = acc + {(vm&&rvm.vrf[0][i]) ? {32'hFF,vrf_sew32[src2_reg_idx*4+i]} : 0};
                end
            end
            vrf_sew32[dest_reg_idx*4] = acc[31:0];
            vrf_sew32[dest_reg_idx*4+1] = acc[63:32];
        end
    endfunction


class pmt_processor#(
    type TD = sew8_t,
    type T2 = sew8_t,
    type T1 = sew8_t,
    type T0 = sew8_t
        );
    
    int dest_reg_idx, dest_eew;
    int src0_reg_idx, src0_eew;
    int src1_reg_idx, src1_eew;
    int src2_reg_idx, src2_eew;
    bit vm;
    logic [511:0] [7:0]  vrf_sew8;
    logic [255:0] [15:0] vrf_sew16;
    logic [127:0] [31:0] vrf_sew32;
    logic [511:0] [7:0] vrf_bit_strobe_temp_sew8;
    logic [255:0] [15:0] vrf_bit_strobe_temp_sew16;
    logic [127:0] [31:0] vrf_bit_strobe_temp_sew32;
    function new();
            $display("This inst is pmt_processor\n");
    endfunction

    function pmt_func(input rvv_behavior_model rvv_behavior_model_inst, input rvs_transaction inst_tr);
            //foreach (rvv_behavior_model_inst.vrf[i]) begin
            //        `uvm_info("pmt_processor",$sformatf("vrf[%0d] = %0d", i, rvv_behavior_model_inst.vrf[i]),UVM_LOW)
            //end 
            //rvv_behavior_model_inst.vrf[31]=128'h0123456789;
            vrf_sew8 = rvv_behavior_model_inst.vrf;
            vrf_sew16 = rvv_behavior_model_inst.vrf;
            vrf_sew32 = rvv_behavior_model_inst.vrf;
            vrf_bit_strobe_temp_sew8 = rvv_behavior_model_inst.vrf_bit_strobe_temp;
            vrf_bit_strobe_temp_sew16 = rvv_behavior_model_inst.vrf_bit_strobe_temp;
            vrf_bit_strobe_temp_sew32 = rvv_behavior_model_inst.vrf_bit_strobe_temp;
            `uvm_info("pmt_processor",$sformatf("This is in pmt_func function\n"),UVM_LOW)
            //foreach (vrf_sew8[idx8]) begin
            //    `uvm_info("pmt_processor",$sformatf("vrf_sew8[%0h] = %0h", idx8, vrf_sew8[idx8]),UVM_LOW)
            //end
            //foreach (vrf_sew16[idx16]) begin
            //    `uvm_info("pmt_processor",$sformatf("vrf_sew16[%0h] = %0h", idx16, vrf_sew16[idx16]),UVM_LOW)
            //end
            //foreach (vrf_sew32[idx32]) begin
            //    `uvm_info("pmt_processor",$sformatf("vrf_sew32[%0h] = %0h", idx32, vrf_sew32[idx32]),UVM_LOW)
            //end
            `uvm_info("pmt_processor",$sformatf("Instruction is %0x, type is %0x",inst_tr.alu_inst,inst_tr.alu_type),UVM_LOW)
            case({inst_tr.alu_inst,inst_tr.alu_type})
                    {VWXUNARY0,OPMVV}: begin
                            VMV_X_S(rvv_behavior_model_inst);
                    end
                    {VWXUNARY0,OPMVX}: VMV_S_X(rvv_behavior_model_inst);
                    {VSLIDEUP_RGATHEREI16,OPIVX}: VSLIDEUP(rvv_behavior_model_inst,rvv_behavior_model_inst.xrf[src1_reg_idx]);
                    {VSLIDEUP_RGATHEREI16,OPIVI}: VSLIDEUP(rvv_behavior_model_inst,inst_tr.src1_idx);
                    {VSLIDEDOWN,OPIVX}: VSLIDEDOWN_F(rvv_behavior_model_inst,rvv_behavior_model_inst.xrf[src1_reg_idx]);
                    {VSLIDEDOWN,OPIVI}: VSLIDEDOWN_F(rvv_behavior_model_inst,inst_tr.src1_idx);
                    {VSLIDE1UP,OPMVX}: VSLIDE1UP_F(rvv_behavior_model_inst,rvv_behavior_model_inst.xrf[src1_reg_idx]);
                    {VSLIDE1DOWN,OPMVX}: VSLIDE1DOWN_F(rvv_behavior_model_inst,rvv_behavior_model_inst.xrf[src1_reg_idx]);
                    {VRGATHER,OPIVV}: VRGATHER_VV(rvv_behavior_model_inst);
                    {VSLIDEUP_RGATHEREI16,OPIVV}: VRGATHEREI16_VV(rvv_behavior_model_inst);
                    {VRGATHER,OPIVX}: VRGATHER_VXI(rvv_behavior_model_inst,rvv_behavior_model_inst.xrf[src1_reg_idx]);
                    {VRGATHER,OPIVI}: VRGATHER_VXI(rvv_behavior_model_inst,inst_tr.src1_idx);
                    {VCOMPRESS,OPMVV}:VCOMPRESS_VM(rvv_behavior_model_inst);
                    {VSMUL_VMVNRR,OPIVI}: begin 
                        if(inst_tr.src1_idx[2:0]==3'b000) begin
                                    VMV1R(rvv_behavior_model_inst);
                                    `uvm_info("pmt_processor",$sformatf("This is in VMV1R function\n"),UVM_LOW)
                                end
                                else if(inst_tr.src1_idx[2:0]==3'b001) begin
                                    VMV2R(rvv_behavior_model_inst);
                                    `uvm_info("pmt_processor",$sformatf("This is in VMV2R function\n"),UVM_LOW)
                                end
                                else if(inst_tr.src1_idx[2:0]==3'b011) begin
                                    VMV4R(rvv_behavior_model_inst);
                                    `uvm_info("pmt_processor",$sformatf("This is in VMV4R function\n"),UVM_LOW)
                                end
                                else if(inst_tr.src1_idx[2:0]==3'b111) begin
                                    VMV8R(rvv_behavior_model_inst);
                                    `uvm_info("pmt_processor",$sformatf("This is in VMV8R function\n"),UVM_LOW)
                                end
                            else
                                `uvm_error("pmt_processor","illegal NR number in transaction!")
                    end
                    default: `uvm_info("pmt_processor",$sformatf("Not invaild instruction."),UVM_LOW)
            endcase
//            PMT_INIT(rvv_behavior_model_inst, 0);
    endfunction

    function TD exe (rvs_transaction inst_tr, T2 src2, T1 src1, T0 src0);
            $display("This class pmt_processor\n");
    endfunction 


    extern function void PMT_INIT(input rvv_behavior_model rvm, input rvs_transaction inst_tr);
    extern function void VMV_X_S(input rvv_behavior_model rvv_behavior_model_inst);
    extern function void VMV_S_X(input rvv_behavior_model rvm);
    extern function void VSLIDEUP(input rvv_behavior_model rvm, input int src1_elm_idx );
    extern function void VSLIDEDOWN_F(input rvv_behavior_model rvm, input int src1_elm_idx );
    extern function void VSLIDE1UP_F(input rvv_behavior_model rvm, input int src1_elm_data);
    extern function void VSLIDE1DOWN_F(input rvv_behavior_model rvm, input int src1_elm_idx);
    extern function void VRGATHER_VV(input rvv_behavior_model rvm);
    extern function void VRGATHEREI16_VV(input rvv_behavior_model rvm);
    extern function void VRGATHER_VXI(input rvv_behavior_model rvm, input int src1_elm_data);
    extern function void VCOMPRESS_VM(input rvv_behavior_model rvm);
    extern function void VMV1R(input rvv_behavior_model rvm);
    extern function void VMV2R(input rvv_behavior_model rvm);
    extern function void VMV4R(input rvv_behavior_model rvm);
    extern function void VMV8R(input rvv_behavior_model rvm);

endclass: pmt_processor

    function void pmt_processor::PMT_INIT(input rvv_behavior_model rvm, input rvs_transaction inst_tr);
            dest_reg_idx = inst_tr.dest_idx;
            src2_reg_idx = inst_tr.src2_idx;
            src1_reg_idx = inst_tr.src1_idx;
            dest_eew = rvm.vsew;
            src2_eew = rvm.vsew;
            src1_eew = rvm.vsew;
            vm = inst_tr.vm;
            `uvm_info("PMT","pmt init",UVM_LOW)
            `uvm_info("PMT",$sformatf("dest:%0x;src2:%0x;src1:%0x;\n ",dest_reg_idx,src2_reg_idx,src1_reg_idx),UVM_LOW)
    endfunction

    function void pmt_processor::VMV_X_S(input rvv_behavior_model rvv_behavior_model_inst);
        //rvv_behavior_model_inst.xrf[] = rvv_behavior_model_inst.vrf[][];
            case(src2_eew) 
                    SEW8: rvv_behavior_model_inst.xrf[dest_reg_idx] = {24'd0,rvv_behavior_model_inst.vrf[src2_reg_idx][7:0]};
                    SEW16: rvv_behavior_model_inst.xrf[dest_reg_idx] = {16'd0,rvv_behavior_model_inst.vrf[src2_reg_idx][15:0]};
                    SEW32: rvv_behavior_model_inst.xrf[dest_reg_idx] = {rvv_behavior_model_inst.vrf[src2_reg_idx][31:0]};

                    default: rvv_behavior_model_inst.xrf[dest_reg_idx] = 32'd0;
            endcase
            `uvm_info("pmt_processor", $sformatf("vmv.x.s x[%0d],v[%0d][0]",dest_reg_idx,src2_reg_idx), UVM_LOW);
    endfunction 

    function void pmt_processor::VMV_S_X(input rvv_behavior_model rvm);
        case(dest_eew)
            SEW8: begin 
                rvm.vrf[dest_reg_idx][7:0]=rvm.xrf[src1_reg_idx][7:0];
                rvm.vrf_bit_strobe_temp[dest_reg_idx]=128'hff;
            end
            SEW16: begin
                rvm.vrf[dest_reg_idx][15:0]=rvm.xrf[src1_reg_idx][15:0];
                rvm.vrf_bit_strobe_temp[dest_reg_idx]=128'hffff;
            end
            SEW32: begin
                rvm.vrf[dest_reg_idx][31:0]=rvm.xrf[src1_reg_idx];
                rvm.vrf_bit_strobe_temp[dest_reg_idx]=128'hffff_ffff;
            end
                default: rvm.vrf[dest_reg_idx]=16'h0;
        endcase
            `uvm_info("pmt_processor", $sformatf("vmv.s.x v[%0d][0],x[%0d]",dest_reg_idx,src1_reg_idx), UVM_LOW);
            `uvm_info("pmt_processor", $sformatf("vmv.s.x dest eew is %0x ",dest_eew), UVM_LOW);
    endfunction 

    //function void pmt_processor::VSLIDEUP_VX(input rvv_behavior_model rvm);
    //        int  i_vl,i_vstart, a;
    //        vrf_t [31:0] vrf_src;
    //        a = rvm.xrf[src1_reg_idx] * dest_eew;
    //        vrf_src = rvm.vrf;
    //        i_vstart = rvm.vstart * dest_eew;
    //        i_vl = rvm.vl * dest_eew;
    //        if ((a > i_vl) | (i_vstart > i_vl)) begin
    //            
    //        end
    //        else if(i_vstart > a) begin
    //            for(int i=i_vstart; i<i_vl; i++) begin 
    //                    if(vrf_src[0][i] && vm == 0) begin
    //                        rvm.vrf[i/dest_eew]=vrf_src[(i-a)/dest_eew];
    //                    end
    //            end
    //        end
    //        else if(i_vstart <= a) begin
    //            for(int i=a; i<i_vl; i++) begin
    //                    if(vrf_src[0][i] && vm == 0) begin
    //                        rvm.vrf[i/dest_eew]=vrf_src[(i-a)/dest_eew];
    //                    end
    //            end
    //        end
    //        else begin
    //        end
    //endfunction
    // dir : 0 -> up    1 -> down ;
    function void pmt_processor::VSLIDEUP(input rvv_behavior_model rvm, input int src1_elm_idx );
        int slide_start;
        slide_start = (rvm.vstart >= src1_elm_idx) ? rvm.vstart : src1_elm_idx;
        if(rvm.vsew == SEW8) begin 
                for(int i=slide_start; i<rvm.vl;i++)begin
                    if(i < rvm.vl) begin
                        if(rvm.vrf[0][i] & (~vm)) begin
                            vrf_sew8[dest_reg_idx*16+i] =  vrf_sew8[src2_reg_idx*16 + (i-src1_elm_idx)] ;
                            `uvm_info("pmt_processor",$sformatf("VSLIDE Des vrf_sew8[%0d] == %0x Src2 vrf_sew8[%0d]==%0x ", dest_reg_idx, vrf_sew8[dest_reg_idx*16+i], src2_reg_idx, vrf_sew8[src2_reg_idx*16 + (i-src1_elm_idx)]),UVM_LOW)
                        vrf_bit_strobe_temp_sew8[dest_reg_idx*16+i] = 8'hff;
                        end
                        else if(vm) begin
                            vrf_sew8[dest_reg_idx*16+i] =  vrf_sew8[src2_reg_idx*16 + (i-src1_elm_idx)] ;
                            `uvm_info("pmt_processor",$sformatf("VSLIDE Des vrf_sew8[%0d] == %0x Src2 vrf_sew8[%0d]==%0x ", dest_reg_idx, vrf_sew8[dest_reg_idx*16+i], src2_reg_idx, vrf_sew8[src2_reg_idx*16 + (i-src1_elm_idx)]),UVM_LOW)
                        vrf_bit_strobe_temp_sew8[dest_reg_idx*16+i] = 8'hff;
                        end
                    end 
                end 
                for (int i=rvm.vstart;i<rvm.vl;i++) begin
                    if(rvm.vrf[0][i] & (~vm)) begin
                            vrf_bit_strobe_temp_sew8[dest_reg_idx*16+i] = 8'hff;
                        end 
                        else if(vm) begin
                            vrf_bit_strobe_temp_sew8[dest_reg_idx*16+i] = 8'hff;
                        end
                end
        end
        else if(rvm.vsew == SEW16) begin 
                for(int i=slide_start; i<rvm.vl;i++)begin
                    if(i < rvm.vl) begin
                        if(rvm.vrf[0][i] & (~vm)) begin
                            vrf_sew16[dest_reg_idx*8+i] =  vrf_sew16[src2_reg_idx*8 + (i-src1_elm_idx)] ;
                            `uvm_info("pmt_processor",$sformatf("VSLIDE Des vrf_sew16[%0d] == %0x Src2 vrf_sew16[%0d]==%0x ", dest_reg_idx, vrf_sew16[dest_reg_idx*8+i], src2_reg_idx, vrf_sew16[src2_reg_idx*8 + (i-src1_elm_idx)]),UVM_LOW)
                        vrf_bit_strobe_temp_sew16[dest_reg_idx*8+i] = 16'hffff;
                        end 
                        else if(vm) begin
                            vrf_sew16[dest_reg_idx*8+i] =  vrf_sew16[src2_reg_idx*8 + (i-src1_elm_idx)] ;
                            `uvm_info("pmt_processor",$sformatf("VSLIDE Des vrf_sew16[%0d] == %0x Src2 vrf_sew16[%0d]==%0x ", dest_reg_idx, vrf_sew16[dest_reg_idx*8+i], src2_reg_idx, vrf_sew16[src2_reg_idx*8 + (i-src1_elm_idx)]),UVM_LOW)
                        vrf_bit_strobe_temp_sew16[dest_reg_idx*8+i] = 16'hffff;
                        end
                    end 
                end
                for (int i=rvm.vstart;i<rvm.vl;i++) begin
                    if(rvm.vrf[0][i] & (~vm)) begin
                            vrf_bit_strobe_temp_sew16[dest_reg_idx*8+i] = 16'hffff;
                        end 
                        else if(vm) begin
                            vrf_bit_strobe_temp_sew16[dest_reg_idx*8+i] = 16'hffff;
                        end
                end
        end
        else if(rvm.vsew == SEW32) begin 
                for(int i=slide_start; i<rvm.vl;i++)begin
                    if(i < rvm.vl) begin
                        if(rvm.vrf[0][i] & (~vm)) begin
                            vrf_sew32[dest_reg_idx*4+i] =  vrf_sew32[src2_reg_idx*4 + (i-src1_elm_idx)] ;
                            `uvm_info("pmt_processor",$sformatf("VSLIDE Des vrf_sew32[%0d] == %0x Src2 vrf_sew32[%0d]==%0x ", dest_reg_idx, vrf_sew32[dest_reg_idx*4+i], src2_reg_idx, vrf_sew32[src2_reg_idx*4 + (i-src1_elm_idx)]),UVM_LOW)
                        vrf_bit_strobe_temp_sew32[dest_reg_idx*4+i] = 32'hffffffff;
                        end 
                        else if(vm) begin
                            vrf_sew32[dest_reg_idx*4+i] =  vrf_sew32[src2_reg_idx*4 + (i-src1_elm_idx)] ;
                            `uvm_info("pmt_processor",$sformatf("VSLIDE Des vrf_sew32[%0d] == %0x Src2 vrf_sew32[%0d]==%0x ", dest_reg_idx, vrf_sew32[dest_reg_idx*4+i], src2_reg_idx, vrf_sew32[src2_reg_idx*4 + (i-src1_elm_idx)]),UVM_LOW)
                        vrf_bit_strobe_temp_sew32[dest_reg_idx*4+i] = 32'hffffffff;
                        end
                    end 
                end
                for (int i=rvm.vstart;i<rvm.vl;i++) begin
                        if(rvm.vrf[0][i] & (~vm)) begin
                        vrf_bit_strobe_temp_sew32[dest_reg_idx*4+i] = 32'hffffffff;
                        end 
                        else if(vm) begin
                        vrf_bit_strobe_temp_sew32[dest_reg_idx*4+i] = 32'hffffffff;
                        end
                end
            end 
        else begin
        end

         case(rvm.vsew)
                    SEW8:begin
                `uvm_info("pmt_processor",$sformatf("Origin VSLIDEUP FUNC Des  vrf[%0d] == %0x", dest_reg_idx, rvm.vrf[dest_reg_idx]),UVM_LOW)
                        rvm.vrf=vrf_sew8;
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew8;
                `uvm_info("pmt_processor",$sformatf("VSLIDEUP FUNC Des  vrf[%0d] == %0x, v[0] == %0x, xrf[%0d]== %0x", dest_reg_idx, rvm.vrf[dest_reg_idx], rvm.vrf[0], src1_reg_idx, src1_elm_idx),UVM_LOW)
                    end
                    SEW16: begin
                `uvm_info("pmt_processor",$sformatf("Origin VSLIDEUP FUNC Des  vrf[%0d] == %0x", dest_reg_idx, rvm.vrf[dest_reg_idx]),UVM_LOW)
                        rvm.vrf=vrf_sew16;
                `uvm_info("pmt_processor",$sformatf("VSLIDEUP FUNC Des  vrf[%0d] == %0x, v[0] == %0x", dest_reg_idx, rvm.vrf[dest_reg_idx], rvm.vrf[0]),UVM_LOW)
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew16;
                    end
                    SEW32: begin
                        rvm.vrf=vrf_sew32;
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew32;
                `uvm_info("pmt_processor",$sformatf("VSLIDEUP FUNC Des  vrf[%0d] == %0x, v[0] == %0x, xrf[%0d]== %0x", dest_reg_idx, rvm.vrf[dest_reg_idx], rvm.vrf[0], src1_reg_idx, src1_elm_idx),UVM_LOW)
                    end
                    default: begin
                        `uvm_error("pmt_processor CHECK", "NO VAILD SEW!")
                    end
        endcase
    endfunction 

    function void pmt_processor::VSLIDEDOWN_F(input rvv_behavior_model rvm, input int src1_elm_idx );

        if(rvm.vsew == SEW8) begin 
                for(int i=rvm.vstart; i<rvm.vl;i++)begin
                    if(i < rvm.vl) begin
                        if(rvm.vrf[0][i] & (~vm)) begin
                            vrf_sew8[dest_reg_idx*16+i] = ((i+src1_elm_idx) < rvm.vlmax) ? vrf_sew8[src2_reg_idx*16 + (i+src1_elm_idx)] : 0;
                            `uvm_info("pmt_processor",$sformatf("VSLIDEDOWN Des vrf_sew8[%0d] == %0x Src2 vrf_sew8[%0d]==%0x ", dest_reg_idx, vrf_sew8[dest_reg_idx*16+i], src2_reg_idx, vrf_sew8[src2_reg_idx*16 + (i+src1_elm_idx)]),UVM_LOW)
                        vrf_bit_strobe_temp_sew8[dest_reg_idx*16+i] = 8'hff;
                        end 
                        else if(vm) begin
                            vrf_sew8[dest_reg_idx*16+i] = ((i+src1_elm_idx) < rvm.vlmax) ? vrf_sew8[src2_reg_idx*16 + (i+src1_elm_idx)] : 0;
                            `uvm_info("pmt_processor",$sformatf("VSLIDEDOWN Des vrf_sew8[%0d] == %0x Src2 vrf_sew8[%0d]==%0x ", dest_reg_idx, vrf_sew8[dest_reg_idx*16+i], src2_reg_idx, vrf_sew8[src2_reg_idx*16 + (i+src1_elm_idx)]),UVM_LOW)
                        vrf_bit_strobe_temp_sew8[dest_reg_idx*16+i] = 8'hff;
                        end
                    end 
                end
                for (int i=rvm.vstart;i<rvm.vl;i++) begin
                        if(rvm.vrf[0][i]& (~vm)) begin
                    vrf_bit_strobe_temp_sew8[dest_reg_idx*16+i] = 8'hff;
                        end 
                        else if(vm) begin
                    vrf_bit_strobe_temp_sew8[dest_reg_idx*16+i] = 8'hff;
                        end
                end
        end
        else if(rvm.vsew == SEW16) begin 
                for(int i=rvm.vstart; i<rvm.vl;i++)begin
                    if(i < rvm.vl) begin
                        if(rvm.vrf[0][i] & (~vm)) begin
                            vrf_sew16[dest_reg_idx*8+i] = ((i+src1_elm_idx) < rvm.vlmax) ? vrf_sew16[src2_reg_idx*8 + (i+src1_elm_idx)] : 0;
                            `uvm_info("pmt_processor",$sformatf("VSLIDE Des vrf_sew16[%0d] == %0x Src2 vrf_sew16[%0d]==%0x ", dest_reg_idx, vrf_sew16[dest_reg_idx*8+i], src2_reg_idx, vrf_sew16[src2_reg_idx*8 + (i+src1_elm_idx)]),UVM_LOW)
                        vrf_bit_strobe_temp_sew16[dest_reg_idx*8+i] = 16'hffff;
                        end 
                        else if(vm) begin
                            vrf_sew16[dest_reg_idx*8+i] = ((i+src1_elm_idx) < rvm.vlmax) ? vrf_sew16[src2_reg_idx*8 + (i+src1_elm_idx)] : 0;
                            `uvm_info("pmt_processor",$sformatf("VSLIDE Des vrf_sew16[%0d] == %0x Src2 vrf_sew16[%0d]==%0x ", dest_reg_idx, vrf_sew16[dest_reg_idx*8+i], src2_reg_idx, vrf_sew16[src2_reg_idx*8 + (i+src1_elm_idx)]),UVM_LOW)
                        vrf_bit_strobe_temp_sew16[dest_reg_idx*8+i] = 16'hffff;
                        end
                    end 
                end
                for (int i=rvm.vstart;i<rvm.vl;i++) begin
                        if(rvm.vrf[0][i] & (~vm)) begin
                        vrf_bit_strobe_temp_sew16[dest_reg_idx*8+i] = 16'hffff;
                        end 
                        else if(vm) begin
                        vrf_bit_strobe_temp_sew16[dest_reg_idx*8+i] = 16'hffff;
                        end
                end
        end
        else if(rvm.vsew == SEW32) begin 
                for(int i=rvm.vstart; i<rvm.vl;i++)begin
                    if(i < rvm.vl) begin
                        if(rvm.vrf[0][i] & (~vm)) begin
                            vrf_sew32[dest_reg_idx*4+i] = ((i+src1_elm_idx) < rvm.vlmax) ? vrf_sew32[src2_reg_idx*4 + (i+src1_elm_idx)] : 0;
                            `uvm_info("pmt_processor",$sformatf("VSLIDE Des vrf_sew32[%0d] == %0x Src2 vrf_sew32[%0d]==%0x ", dest_reg_idx, vrf_sew32[dest_reg_idx*4+i], src2_reg_idx, vrf_sew32[src2_reg_idx*4 + (i+src1_elm_idx)]),UVM_LOW)
                        vrf_bit_strobe_temp_sew32[dest_reg_idx*4+i] = 32'hffffffff;
                        end 
                        else if(vm) begin
                            vrf_sew32[dest_reg_idx*4+i] = ((i+src1_elm_idx) < rvm.vlmax) ? vrf_sew32[src2_reg_idx*4 + (i+src1_elm_idx)] : 0;
                            `uvm_info("pmt_processor",$sformatf("VSLIDE Des vrf_sew32[%0d] == %0x Src2 vrf_sew32[%0d]==%0x ", dest_reg_idx, vrf_sew32[dest_reg_idx*4+i], src2_reg_idx, vrf_sew32[src2_reg_idx*4 + (i+src1_elm_idx)]),UVM_LOW)
                        vrf_bit_strobe_temp_sew32[dest_reg_idx*4+i] = 32'hffffffff;
                        end
                    end 
                end
                for (int i=rvm.vstart;i<rvm.vl;i++) begin
                        if(rvm.vrf[0][i] & (~vm)) begin
                        vrf_bit_strobe_temp_sew32[dest_reg_idx*4+i] = 32'hffffffff;
                        end 
                        else if(vm) begin
                        vrf_bit_strobe_temp_sew32[dest_reg_idx*4+i] = 32'hffffffff;
                        end
                end
            end 
        else begin
        end

         case(rvm.vsew)
                    SEW8:begin
                        rvm.vrf=vrf_sew8;
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew8;
                    end
                    SEW16: begin
                        rvm.vrf=vrf_sew16;
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew16;
                    end
                    SEW32: begin
                        rvm.vrf=vrf_sew32;
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew32;
                    end
                    default: begin
                        `uvm_error("VCOMPRESS_VM CHECK", "NO VAILD SEW!")
                    end
        endcase
    endfunction


    //function void pmt_processor::VSLIDEDOWN_VX(input rvv_behavior_model rvm);
    //endfunction 
    //function void pmt_processor::VSLIDEDOWN_VI(input rvv_behavior_model rvm);
    //endfunction
    function void pmt_processor::VSLIDE1UP_F(input rvv_behavior_model rvm, input int src1_elm_data);
        int slide_first_data;
        slide_first_data = (rvm.vstart >= 1) ? rvm.vstart : 1;
        if(rvm.vsew == SEW8) begin 
                `uvm_info("VSLIDE1UP_F",$sformatf("des vrf[%0d] == %0x, xrf[%0x] == %0x", dest_reg_idx, rvm.vrf[dest_reg_idx], src1_reg_idx, src1_elm_data),UVM_LOW)
                if((rvm.vstart==0) & rvm.vrf[0][0] & (~vm)) begin
                    vrf_sew8[dest_reg_idx*16] =  src1_elm_data ;
                    vrf_bit_strobe_temp_sew8[dest_reg_idx*16] = 8'hff;
                end
                else if((rvm.vstart==0)&vm) begin
                    vrf_sew8[dest_reg_idx*16] =  src1_elm_data ;
                    vrf_bit_strobe_temp_sew8[dest_reg_idx*16] = 8'hff;
                end
                for(int i=slide_first_data; i<rvm.vl;i++)begin
                    if(i < rvm.vl) begin
                        if(rvm.vrf[0][i] & (~vm)) begin
                            vrf_sew8[dest_reg_idx*16+i] =  vrf_sew8[src2_reg_idx*16 + (i-1)] ;
                            vrf_bit_strobe_temp_sew8[dest_reg_idx*16+i] = 8'hff;
                        end 
                        else if(vm) begin
                            vrf_sew8[dest_reg_idx*16+i] =  vrf_sew8[src2_reg_idx*16 + (i-1)] ;
                            vrf_bit_strobe_temp_sew8[dest_reg_idx*16+i] = 8'hff;
                        end
                    end 
                end
        end
        else if(rvm.vsew == SEW16) begin 
                if((rvm.vstart==0)&rvm.vrf[0][0] & (~vm)) begin
                    vrf_sew16[dest_reg_idx*8] = src1_elm_data ;
                    vrf_bit_strobe_temp_sew16[dest_reg_idx*8] = 16'hffff;
                end
                else if((rvm.vstart==0)&vm) begin
                    vrf_sew16[dest_reg_idx*8] = src1_elm_data ;
                    vrf_bit_strobe_temp_sew16[dest_reg_idx*8] = 16'hffff;
                end
                for(int i=slide_first_data; i<rvm.vl;i++)begin
                    if(i < rvm.vl) begin
                        if(rvm.vrf[0][i] & (~vm)) begin
                            vrf_sew16[dest_reg_idx*8+i] =  vrf_sew16[src2_reg_idx*8 + (i-1)] ;
                            vrf_bit_strobe_temp_sew16[dest_reg_idx*8+i] = 16'hffff;
                        end 
                        else if(vm) begin
                            vrf_sew16[dest_reg_idx*8+i] =  vrf_sew16[src2_reg_idx*8 + (i-1)] ;
                            vrf_bit_strobe_temp_sew16[dest_reg_idx*8+i] = 16'hffff;
                        end
                    end 
                end
        end
        else if(rvm.vsew == SEW32) begin 
                if((rvm.vstart==0)&rvm.vrf[0][0] & (~vm)) begin
                    vrf_sew32[dest_reg_idx*4] =  src1_elm_data ;
                    vrf_bit_strobe_temp_sew32[dest_reg_idx*4] = 32'hffffffff;
                end
                else if((rvm.vstart==0)&vm) begin
                    vrf_sew32[dest_reg_idx*4] =  src1_elm_data ;
                    vrf_bit_strobe_temp_sew32[dest_reg_idx*4] = 32'hffffffff;
                end
                for(int i=slide_first_data; i<rvm.vl;i++)begin
                    if(i < rvm.vl) begin
                        if(rvm.vrf[0][i] & (~vm)) begin
                            vrf_sew32[dest_reg_idx*4+i] =  vrf_sew32[src2_reg_idx*4 + (i-1)] ;
                            vrf_bit_strobe_temp_sew32[dest_reg_idx*4+i] = 32'hffffffff;
                        end 
                        else if(vm) begin
                            vrf_sew32[dest_reg_idx*4+i] =  vrf_sew32[src2_reg_idx*4 + (i-1)] ;
                            vrf_bit_strobe_temp_sew32[dest_reg_idx*4+i] = 32'hffffffff;
                        end
                    end 
                end
            end 
        else begin
        end

         case(rvm.vsew)
                    SEW8:begin
                        rvm.vrf=vrf_sew8;
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew8;
                    end
                    SEW16: begin
                `uvm_info("pmt_processor",$sformatf("Origin VCOMPRESS_VM FUNC Des  vrf[%0d] == %0x", dest_reg_idx, rvm.vrf[dest_reg_idx]),UVM_LOW)
                        rvm.vrf=vrf_sew16;
                `uvm_info("pmt_processor",$sformatf("Origin VCOMPRESS_VM FUNC Des  vrf[%0d] == %0x", dest_reg_idx, rvm.vrf[dest_reg_idx]),UVM_LOW)
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew16;
                    end
                    SEW32: begin
                        rvm.vrf=vrf_sew32;
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew32;
                    end
                    default: begin
                        `uvm_error("VCOMPRESS_VM CHECK", "NO VAILD SEW!")
                    end
        endcase

    endfunction 

    function void pmt_processor::VSLIDE1DOWN_F(input rvv_behavior_model rvm, input int src1_elm_idx);
         int slide_last_data;
        slide_last_data = rvm.vl - 1;
        if(rvm.vsew == SEW8) begin 
                for(int i=rvm.vstart; i<slide_last_data;i++)begin
                    if(i < rvm.vl) begin
                        if(rvm.vrf[0][i] && (~vm)) begin
                            vrf_sew8[dest_reg_idx*16+i] =  vrf_sew8[src2_reg_idx*16 + (i+1)] ;
                            vrf_bit_strobe_temp_sew8[dest_reg_idx*16+i] = 8'hff;
                        end 
                        else if(vm) begin
                            vrf_sew8[dest_reg_idx*16+i] =  vrf_sew8[src2_reg_idx*16 + (i+1)] ;
                            vrf_bit_strobe_temp_sew8[dest_reg_idx*16+i] = 8'hff;
                        end
                    end 
                end
                if(rvm.vrf[0][slide_last_data] & (~vm)) begin
                    vrf_sew8[dest_reg_idx*16 + slide_last_data] =  src1_elm_idx ;
                    vrf_bit_strobe_temp_sew8[dest_reg_idx*16 + slide_last_data] = 8'hff;
                end
                else if(vm) begin
                    vrf_sew8[dest_reg_idx*16 + slide_last_data] =  src1_elm_idx ;
                    vrf_bit_strobe_temp_sew8[dest_reg_idx*16 + slide_last_data] = 8'hff;
                end
        end
        else if(rvm.vsew == SEW16) begin 
                for(int i=rvm.vstart; i<slide_last_data;i++)begin
                    if(i < rvm.vl) begin
                        if(rvm.vrf[0][i] & (~vm)) begin
                            vrf_sew16[dest_reg_idx*8+i] =  vrf_sew16[src2_reg_idx*8 + (i+1)] ;
                            vrf_bit_strobe_temp_sew16[dest_reg_idx*8+i] = 16'hffff;
                        end 
                        else if(vm) begin
                            vrf_sew16[dest_reg_idx*8+i] =  vrf_sew16[src2_reg_idx*8 + (i+1)] ;
                            vrf_bit_strobe_temp_sew16[dest_reg_idx*8+i] = 16'hffff;
                        end
                    end 
                end
                if(rvm.vrf[0][slide_last_data] & (~vm)) begin
                    vrf_sew16[dest_reg_idx*8 + slide_last_data] =  src1_elm_idx ;
                    vrf_bit_strobe_temp_sew16[dest_reg_idx*8 + slide_last_data] = 16'hffff;
                end
                else if(vm) begin
                    vrf_sew16[dest_reg_idx*8 + slide_last_data] =  src1_elm_idx ;
                    vrf_bit_strobe_temp_sew16[dest_reg_idx*8 + slide_last_data] = 16'hffff;
                end
        end
        else if(rvm.vsew == SEW32) begin 
                for(int i=rvm.vstart; i<slide_last_data;i++)begin
                    if(i < rvm.vl) begin
                        if(rvm.vrf[0][i] & (~vm)) begin
                            vrf_sew32[dest_reg_idx*4+i] =  vrf_sew32[src2_reg_idx*4 + (i+1)] ;
                            vrf_bit_strobe_temp_sew32[dest_reg_idx*4+i] = 32'hffffffff;
                        end 
                        else if(vm) begin
                            vrf_sew32[dest_reg_idx*4+i] =  vrf_sew32[src2_reg_idx*4 + (i+1)] ;
                            vrf_bit_strobe_temp_sew32[dest_reg_idx*4+i] = 32'hffffffff;
                        end
                    end 
                end
                if(rvm.vrf[0][slide_last_data] & (~vm)) begin
                    vrf_sew32[dest_reg_idx*4 + slide_last_data] =  src1_elm_idx ;
                    vrf_bit_strobe_temp_sew32[dest_reg_idx*4 + slide_last_data] = 32'hffffffff;
                end
                else if(vm) begin
                    vrf_sew32[dest_reg_idx*4 + slide_last_data] =  src1_elm_idx ;
                    vrf_bit_strobe_temp_sew32[dest_reg_idx*4 + slide_last_data] = 32'hffffffff;
                end
            end 
        else begin
        end  

         case(rvm.vsew)
                    SEW8:begin
                        rvm.vrf=vrf_sew8;
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew8;
                    end
                    SEW16: begin
                        rvm.vrf=vrf_sew16;
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew16;
                    end
                    SEW32: begin
                        rvm.vrf=vrf_sew32;
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew32;
                    end
                    default: begin
                        `uvm_error("VCOMPRESS_VM CHECK", "NO VAILD SEW!")
                    end
        endcase
    endfunction
    //function void pmt_processor::VSLIDE1DOWN_VX(input rvv_behavior_model rvm);
    //endfunction
    function void pmt_processor::VRGATHER_VV(input rvv_behavior_model rvm);
        if(rvm.vsew == SEW8) begin 
                for(int i=rvm.vstart; i<rvm.vl;i++)begin
                    if(i < rvm.vl) begin
                        `uvm_info("pmt_processor",$sformatf("Index is %0x", i),UVM_LOW)
                        if(rvm.vrf[0][i] && (~vm)) begin
                            vrf_sew8[dest_reg_idx*16+i] = (vrf_sew8[src1_reg_idx*16 + i] >= rvm.vlmax) ? 0 : vrf_sew8[src2_reg_idx*16 + vrf_sew8[src1_reg_idx*16 + i]] ;
                            vrf_bit_strobe_temp_sew8[dest_reg_idx*16+i] = 8'hff;
                        end
                        else if(vm) begin
                            vrf_sew8[dest_reg_idx*16+i] = (vrf_sew8[src1_reg_idx*16 + i] >= rvm.vlmax) ? 0 : vrf_sew8[src2_reg_idx*16 + vrf_sew8[src1_reg_idx*16 + i]] ;
                            vrf_bit_strobe_temp_sew8[dest_reg_idx*16+i] = 8'hff;
                        end
                    end 
                end
        end
        else if(rvm.vsew == SEW16) begin
                for(int i=rvm.vstart; i<rvm.vl;i++)begin
                    if(i < rvm.vl) begin
                        if(rvm.vrf[0][i] && (~vm)) begin
                        `uvm_info("vgather_vv",$sformatf("vm is %0x", rvm.vrf[0][i]),UVM_LOW)
                            vrf_sew16[dest_reg_idx*8+i] = (vrf_sew16[src1_reg_idx*8 + i] >= rvm.vlmax) ? 0 : vrf_sew16[src2_reg_idx*8 + vrf_sew16[src1_reg_idx*8 + i]] ;
                            vrf_bit_strobe_temp_sew16[dest_reg_idx*8+i] = 16'hffff;
                        end
                        else if(vm) begin
                        `uvm_info("vgather_vv",$sformatf("vm is %0x", rvm.vrf[0][i]),UVM_LOW)
                            vrf_sew16[dest_reg_idx*8+i] = (vrf_sew16[src1_reg_idx*8 + i] >= rvm.vlmax) ? 0 : vrf_sew16[src2_reg_idx*8 + vrf_sew16[src1_reg_idx*8 + i]] ;
                            vrf_bit_strobe_temp_sew16[dest_reg_idx*8+i] = 16'hffff;
                        end
                    end 
                end
        end
        else begin
                for(int i=rvm.vstart; i<rvm.vl;i++)begin
                    if(i < rvm.vl) begin
                        if(rvm.vrf[0][i]& (~vm)) begin
                            vrf_sew32[dest_reg_idx*4+i] = (vrf_sew32[src1_reg_idx*4 + i] >= rvm.vlmax) ? 0 : vrf_sew32[src2_reg_idx*4 + vrf_sew32[src1_reg_idx*4 + i]] ;
                            vrf_bit_strobe_temp_sew32[dest_reg_idx*4+i] = 32'hffffffff;
                        end
                        else if(vm) begin
                            vrf_sew32[dest_reg_idx*4+i] = (vrf_sew32[src1_reg_idx*4 + i] >= rvm.vlmax) ? 0 : vrf_sew32[src2_reg_idx*4 + vrf_sew32[src1_reg_idx*4 + i]] ;
                            vrf_bit_strobe_temp_sew32[dest_reg_idx*4+i] = 32'hffffffff;
                        end
                    end 
                end
        end


        case(rvm.vsew)
                    SEW8:begin
                        rvm.vrf=vrf_sew8;
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew8;
                `uvm_info("pmt_processor",$sformatf("VGATHER Des  vrf[%0d] == %0x", dest_reg_idx, rvm.vrf[dest_reg_idx]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VGATHER SRC2  vrf[%0d] == %0x", src2_reg_idx, rvm.vrf[src2_reg_idx]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VGATHER SRC1  vrf[%0d] == %0x", src1_reg_idx, rvm.vrf[src1_reg_idx]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VGATHER V0  vrf[0] == %0x", rvm.vrf[0]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VGATHER VLMAX  %0x", rvm.vlmax),UVM_LOW)
                    end
                    SEW16: begin
                `uvm_info("pmt_processor",$sformatf("VGATHER Des  vrf[%0d] == %0x", dest_reg_idx, rvm.vrf[dest_reg_idx]),UVM_LOW)
                        rvm.vrf=vrf_sew16;
                `uvm_info("pmt_processor",$sformatf("VGATHER Des  vrf[%0d] == %0x", dest_reg_idx, rvm.vrf[dest_reg_idx]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VGATHER SRC2  vrf[%0d] == %0x", src2_reg_idx, rvm.vrf[src2_reg_idx]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VGATHER SRC1  vrf[%0d] == %0x", src1_reg_idx, rvm.vrf[src1_reg_idx]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VGATHER V0  vrf[0] == %0x", rvm.vrf[0]),UVM_LOW)
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew16;
                    end
                    SEW32: begin
                        rvm.vrf=vrf_sew32;
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew32;
                    end
                    default: begin
                        `uvm_error("VGATHER CHECK", "NO VAILD SEW!")
                    end
       endcase
   endfunction 


    function void pmt_processor::VRGATHER_VXI(input rvv_behavior_model rvm, input int src1_elm_data);
        if(rvm.vsew == SEW8) begin 
                for(int i=rvm.vstart; i<rvm.vl;i++)begin
                    if(i < rvm.vl) begin
                        if(rvm.vrf[0][i] && (~vm)) begin
                            vrf_sew8[dest_reg_idx*16+i] = (src1_elm_data >= rvm.vlmax) ? 0 : vrf_sew8[src2_reg_idx*16 + src1_elm_data] ;
                `uvm_info("pmt_processor",$sformatf("VRGATHER_VXI Des vrf_sew8[%0d] == %0x Src2 vrf_sew8[%0d]==%0x ", dest_reg_idx, vrf_sew8[dest_reg_idx*16+i], src2_reg_idx, vrf_sew8[src2_reg_idx*16 + src1_elm_data]),UVM_LOW)
                            vrf_bit_strobe_temp_sew8[dest_reg_idx*16+i] = 8'hff;
                        end
                        else if(vm) begin
                            vrf_sew8[dest_reg_idx*16+i] = (src1_elm_data >= rvm.vlmax) ? 0 : vrf_sew8[src2_reg_idx*16 + src1_elm_data] ;
                `uvm_info("pmt_processor",$sformatf("VRGATHER_VXI Des vrf_sew8[%0d] == %0x Src2 vrf_sew8[%0d]==%0x ", dest_reg_idx, vrf_sew8[dest_reg_idx*16+i], src2_reg_idx, vrf_sew8[src2_reg_idx*16 + src1_elm_data]),UVM_LOW)
                            vrf_bit_strobe_temp_sew8[dest_reg_idx*16+i] = 8'hff;
                        end
                    end 
                end
        end
        else if(rvm.vsew == SEW16) begin
                for(int i=rvm.vstart; i<rvm.vl;i++)begin
                    if(i < rvm.vl) begin
                        if(rvm.vrf[0][i] && (~vm)) begin
                            vrf_sew16[dest_reg_idx*8+i] = (src1_elm_data >= rvm.vlmax) ? 0 : vrf_sew16[src2_reg_idx*8 + src1_elm_data] ;
                `uvm_info("pmt_processor",$sformatf("VRGATHER_VXI Des vrf_sew16[%0d] == %0x Src2 vrf_sew16[%0d]==%0x ", dest_reg_idx, vrf_sew16[dest_reg_idx*8+i], src2_reg_idx, vrf_sew16[src2_reg_idx*8 + src1_elm_data]),UVM_LOW)
                            vrf_bit_strobe_temp_sew16[dest_reg_idx*8+i] = 16'hffff;
                        end
                        else if(vm) begin
                            vrf_sew16[dest_reg_idx*8+i] = (src1_elm_data >= rvm.vlmax) ? 0 : vrf_sew16[src2_reg_idx*8 + src1_elm_data] ;
                `uvm_info("pmt_processor",$sformatf("VRGATHER_VXI Des vrf_sew16[%0d] == %0x Src2 vrf_sew16[%0d]==%0x ", dest_reg_idx, vrf_sew16[dest_reg_idx*8+i], src2_reg_idx, vrf_sew16[src2_reg_idx*8 + src1_elm_data]),UVM_LOW)
                            vrf_bit_strobe_temp_sew16[dest_reg_idx*8+i] = 16'hffff;
                        end
                    end 
                end
        end
        else begin
                for(int i=rvm.vstart; i<rvm.vl;i++)begin
                    if(i < rvm.vl) begin
                        if(rvm.vrf[0][i] && (~vm)) begin
                            vrf_sew32[dest_reg_idx*4+i] = (src1_elm_data >= rvm.vlmax) ? 0 : vrf_sew32[src2_reg_idx*4 + src1_elm_data];
                `uvm_info("pmt_processor",$sformatf("VRGATHER_VXI Des vrf_sew32[%0d] == %0x Src2 vrf_sew32[%0d]==%0x ", dest_reg_idx, vrf_sew32[dest_reg_idx*4+i], src2_reg_idx, vrf_sew32[src2_reg_idx*4 + src1_elm_data]),UVM_LOW)
                            vrf_bit_strobe_temp_sew32[dest_reg_idx*4+i] = 32'hffffffff;
                        end
                        else if(vm) begin
                            vrf_sew32[dest_reg_idx*4+i] = (src1_elm_data >= rvm.vlmax) ? 0 : vrf_sew32[src2_reg_idx*4 + src1_elm_data];
                `uvm_info("pmt_processor",$sformatf("VRGATHER_VXI Des vrf_sew32[%0d] == %0x Src2 vrf_sew32[%0d]==%0x ", dest_reg_idx, vrf_sew32[dest_reg_idx*4+i], src2_reg_idx, vrf_sew32[src2_reg_idx*4 + src1_elm_data]),UVM_LOW)
                            vrf_bit_strobe_temp_sew32[dest_reg_idx*4+i] = 32'hffffffff;
                        end
                    end 
                end
       end

        case(rvm.vsew)
                    SEW8:begin
                        rvm.vrf=vrf_sew8;
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew8;
                    end
                    SEW16: begin
                `uvm_info("pmt_processor",$sformatf("Origin VCOMPRESS_VM FUNC Des  vrf[%0d] == %0x", dest_reg_idx, rvm.vrf[dest_reg_idx]),UVM_LOW)
                        rvm.vrf=vrf_sew16;
                `uvm_info("pmt_processor",$sformatf("Origin VCOMPRESS_VM FUNC Des  vrf[%0d] == %0x", dest_reg_idx, rvm.vrf[dest_reg_idx]),UVM_LOW)
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew16;
                    end
                    SEW32: begin
                        rvm.vrf=vrf_sew32;
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew32;
                    end
                    default: begin
                        `uvm_error("VCOMPRESS_VM CHECK", "NO VAILD SEW!")
                    end
       endcase

    endfunction 

        function void pmt_processor::VRGATHEREI16_VV(input rvv_behavior_model rvm);
            if(rvm.vsew == SEW8) begin 
                for(int i=rvm.vstart; i<rvm.vl;i++)begin
                    if(i < rvm.vl) begin
                        if(rvm.vrf[0][i] && (~vm)) begin
                            vrf_sew8[dest_reg_idx*16+i] = (vrf_sew16[src1_reg_idx*8 + i] >= rvm.vlmax) ? 0 : vrf_sew8[src2_reg_idx*16 + vrf_sew16[src1_reg_idx*8 + i]] ;
                `uvm_info("pmt_processor",$sformatf("VRGATHEREI16_VV Des vrf_sew8[%0d] == %0x Src2 vrf_sew8[%0d]==%0x ", dest_reg_idx, vrf_sew8[dest_reg_idx*16+i], src2_reg_idx, vrf_sew8[src2_reg_idx*16 + vrf_sew16[src1_reg_idx*8+i]]),UVM_LOW)
                            vrf_bit_strobe_temp_sew8[dest_reg_idx*16+i] = 8'hff;
                        end 
                        else if(vm) begin
                            vrf_sew8[dest_reg_idx*16+i] = (vrf_sew16[src1_reg_idx*8 + i] >= rvm.vlmax) ? 0 : vrf_sew8[src2_reg_idx*16 + vrf_sew16[src1_reg_idx*8 + i]] ;
                `uvm_info("pmt_processor",$sformatf("VRGATHEREI16_VV Des vrf_sew8[%0d] == %0x Src2 vrf_sew8[%0d]==%0x ", dest_reg_idx, vrf_sew8[dest_reg_idx*16+i], src2_reg_idx, vrf_sew8[src2_reg_idx*16 + vrf_sew16[src1_reg_idx*8+i]]),UVM_LOW)
                            vrf_bit_strobe_temp_sew8[dest_reg_idx*16+i] = 8'hff;
                        end
                    end 
                end
        end
        else if(rvm.vsew == SEW16) begin
                for(int i=rvm.vstart; i<rvm.vl;i++)begin
                    if(i < rvm.vl) begin
                        if(rvm.vrf[0][i] && (~vm)) begin
                            vrf_sew16[dest_reg_idx*8+i] = (vrf_sew16[src1_reg_idx*8 + i] >= rvm.vlmax) ? 0 : vrf_sew16[src2_reg_idx*8 + vrf_sew16[src1_reg_idx*8 + i]] ;
                `uvm_info("pmt_processor",$sformatf("VRGATHEREI16_VV Des vrf_sew16[%0d] == %0x Src2 vrf_sew16[%0d]==%0x ", dest_reg_idx, vrf_sew16[dest_reg_idx*8+i], src2_reg_idx, vrf_sew16[src2_reg_idx*8 + vrf_sew16[src1_reg_idx*8+i]]),UVM_LOW)
                            vrf_bit_strobe_temp_sew16[dest_reg_idx*8+i] = 16'hffff;
                        end
                        else if(vm) begin
                            vrf_sew16[dest_reg_idx*8+i] = (vrf_sew16[src1_reg_idx*8 + i] >= rvm.vlmax) ? 0 : vrf_sew16[src2_reg_idx*8 + vrf_sew16[src1_reg_idx*8 + i]] ;
                `uvm_info("pmt_processor",$sformatf("VRGATHEREI16_VV Des vrf_sew16[%0d] == %0x Src2 vrf_sew16[%0d]==%0x ", dest_reg_idx, vrf_sew16[dest_reg_idx*8+i], src2_reg_idx, vrf_sew16[src2_reg_idx*8 + vrf_sew16[src1_reg_idx*8+i]]),UVM_LOW)
                            vrf_bit_strobe_temp_sew16[dest_reg_idx*8+i] = 16'hffff;
                        end
                    end 
                end
        end
        else begin
                for(int i=rvm.vstart; i<rvm.vl;i++)begin
                    if(i < rvm.vl) begin
                        if(rvm.vrf[0][i] && (~vm)) begin
                            vrf_sew32[dest_reg_idx*4+i] = (vrf_sew16[src1_reg_idx*8 + i] >= rvm.vlmax) ? 0 : vrf_sew32[src2_reg_idx*4 + vrf_sew16[src1_reg_idx*8 + i]] ;
                `uvm_info("pmt_processor",$sformatf("VRGATHEREI16_VV Des vrf_sew32[%0d] == %0x Src2 vrf_sew32[%0d]==%0x ", dest_reg_idx, vrf_sew32[dest_reg_idx*4+i], src2_reg_idx, vrf_sew32[src2_reg_idx*4 + vrf_sew16[src1_reg_idx*8+i]]),UVM_LOW)
                            vrf_bit_strobe_temp_sew32[dest_reg_idx*4+i] = 32'hffffffff;
                        end
                        else if(vm) begin
                            vrf_sew32[dest_reg_idx*4+i] = (vrf_sew16[src1_reg_idx*8 + i] >= rvm.vlmax) ? 0 : vrf_sew32[src2_reg_idx*4 + vrf_sew16[src1_reg_idx*8 + i]] ;
                `uvm_info("pmt_processor",$sformatf("VRGATHEREI16_VV Des vrf_sew32[%0d] == %0x Src2 vrf_sew32[%0d]==%0x ", dest_reg_idx, vrf_sew32[dest_reg_idx*4+i], src2_reg_idx, vrf_sew32[src2_reg_idx*4 + vrf_sew16[src1_reg_idx*8+i]]),UVM_LOW)
                            vrf_bit_strobe_temp_sew32[dest_reg_idx*4+i] = 32'hffffffff;
                        end
                    end 
                end
        end
    
        case(rvm.vsew)
                    SEW8:begin
                        rvm.vrf=vrf_sew8;
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew8;
                    end
                    SEW16: begin
                `uvm_info("pmt_processor",$sformatf("Origin VCOMPRESS_VM FUNC Des  vrf[%0d] == %0x", dest_reg_idx, rvm.vrf[dest_reg_idx]),UVM_LOW)
                        rvm.vrf=vrf_sew16;
                `uvm_info("pmt_processor",$sformatf("Origin VCOMPRESS_VM FUNC Des  vrf[%0d] == %0x", dest_reg_idx, rvm.vrf[dest_reg_idx]),UVM_LOW)
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew16;
                    end
                    SEW32: begin
                        rvm.vrf=vrf_sew32;
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew32;
                    end
                    default: begin
                        `uvm_error("VCOMPRESS_VM CHECK", "NO VAILD SEW!")
                    end
       endcase
    endfunction

    //function void pmt_processor::VRGATHER_VI(input rvv_behavior_model rvm);
    //endfunction
    function void pmt_processor::VCOMPRESS_VM(input rvv_behavior_model rvm);
        int j=0;
        if(rvm.vsew == SEW8) begin
            for(int i=0; i<rvm.vl; i++) begin 
                if(i < rvm.vl) begin 
                    if(rvm.vrf[src1_reg_idx][i]) begin
                        vrf_sew8[dest_reg_idx*16+j] = vrf_sew8[src2_reg_idx*16 + i];
                        j = j+1;
                        `uvm_info("pmt_processor",$sformatf("VCOMPRESS Src vrf[%0d] == %0x", src1_reg_idx, rvm.vrf[src1_reg_idx]),UVM_LOW)
                        vrf_bit_strobe_temp_sew8[dest_reg_idx*16+i] = 8'hff;
                    end
                    else begin
                        vrf_bit_strobe_temp_sew8[dest_reg_idx*16+i] = 8'hff;
                    end
                end 
            end 
        end
        else if(rvm.vsew == SEW16) begin
            for(int i=0; i<rvm.vl; i++) begin 
                if(i < rvm.vl) begin 
                    if(rvm.vrf[src1_reg_idx][i]) begin
                `uvm_info("pmt_processor",$sformatf("VCOMPRESS Src vrf[%0d] == %0x", src1_reg_idx, rvm.vrf[src1_reg_idx]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VCOMPRESS Src2 vrf[%0d] == %0x", src2_reg_idx, vrf_sew16[src2_reg_idx*8+i]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VCOMPRESS Src2 vrf[%0d] == %0x", dest_reg_idx, vrf_sew16[dest_reg_idx*8+j]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VCOMPRESS i== %0x, j == %0x", i, j),UVM_LOW)
                        vrf_sew16[dest_reg_idx*8+j] = vrf_sew16[src2_reg_idx*8 + i];
                `uvm_info("pmt_processor",$sformatf("VCOMPRESS Src2 vrf[%0d] == %0x", src2_reg_idx, vrf_sew16[src2_reg_idx*8+i]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VCOMPRESS Src2 vrf[%0d] == %0x", dest_reg_idx, vrf_sew16[dest_reg_idx*8+j]),UVM_LOW)
                        j = j+1;
                        vrf_bit_strobe_temp_sew16[dest_reg_idx*8+i] = 16'hffff;
                    end
                    else begin
                        vrf_bit_strobe_temp_sew16[dest_reg_idx*8+i] = 16'hffff;
                    end
               end
            end  
        end
        else begin
             for(int i=0; i<rvm.vl; i++) begin 
                if(i < rvm.vl) begin 
                    if(rvm.vrf[src1_reg_idx][i]) begin
                        vrf_sew32[dest_reg_idx*4+j] = vrf_sew32[src2_reg_idx*4 + i];
                `uvm_info("pmt_processor",$sformatf("VCOMPRESS Src vrf[%0d] == %0x", src1_reg_idx, rvm.vrf[src1_reg_idx]),UVM_LOW)
                        j = j+1;
                        vrf_bit_strobe_temp_sew32[dest_reg_idx*4+i] = 32'hffffffff;
                    end
                    else begin
                        vrf_bit_strobe_temp_sew32[dest_reg_idx*4+i] = 32'hffffffff;
                    end
               end
            end 
        end 

        case(rvm.vsew)
                    SEW8:begin
                        rvm.vrf=vrf_sew8;
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew8;
                    end
                    SEW16: begin
                `uvm_info("pmt_processor",$sformatf("Origin VCOMPRESS_VM FUNC Des  vrf[%0d] == %0x", dest_reg_idx, rvm.vrf[dest_reg_idx]),UVM_LOW)
                        rvm.vrf=vrf_sew16;
                `uvm_info("pmt_processor",$sformatf("Origin VCOMPRESS_VM FUNC Des  vrf[%0d] == %0x", dest_reg_idx, rvm.vrf[dest_reg_idx]),UVM_LOW)
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew16;
                    end
                    SEW32: begin
                        rvm.vrf=vrf_sew32;
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew32;
                    end
                    default: begin
                        `uvm_error("VCOMPRESS_VM CHECK", "NO VAILD SEW!")
                    end
       endcase
                `uvm_info("pmt_processor",$sformatf("Origin VCOMPRESS_VM FUNC Des  vrf[%0d] == %0x", dest_reg_idx, rvm.vrf[dest_reg_idx]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("Origin VCOMPRESS_VM FUNC Src2  vrf[%0d] == %0x", src2_reg_idx, rvm.vrf[src2_reg_idx]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("Origin VCOMPRESS_VM FUNC Src1  vrf[%0d] == %0x", src1_reg_idx, rvm.vrf[src1_reg_idx]),UVM_LOW)
    endfunction
    function void pmt_processor::VMV1R(input rvv_behavior_model rvm);
                logic [7:0] start_reg_idx;
                logic [7:0] start_offset_idx;
                case(rvm.vsew)
                    SEW8: begin 
                        start_reg_idx = rvm.vstart/16;
                        start_offset_idx = rvm.vstart%16;
                    end
                    SEW16: begin 
                        start_reg_idx = rvm.vstart/8;
                        start_offset_idx = rvm.vstart%8;
                    end
                    SEW32: begin 
                        start_reg_idx = rvm.vstart/4;
                        start_offset_idx = rvm.vstart%4;
                    end
                    default: begin
                        `uvm_error("VMV4R CHECK", "NO VAILD SEW!")
                    end
                endcase
                `uvm_info("pmt_processor",$sformatf("Origin VMV1R FUNC Src  vrf[%0d] == %0x", src2_reg_idx, rvm.vrf[src2_reg_idx]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("Origin VMV1R FUNC Dest vrf[%0d] == %0x", dest_reg_idx, rvm.vrf[dest_reg_idx]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("start_reg_idx = %0x, start_offset_idx = %0x", start_reg_idx, start_offset_idx),UVM_LOW)
                if((rvm.vstart < 16 && (rvm.vsew == SEW8)) | (rvm.vstart < 8 && (rvm.vsew == SEW16)) | (rvm.vstart < 4 && (rvm.vsew == SEW32))  )begin
                    case(rvm.vsew)
                    SEW8: begin 
                        for(int j = 0;j < start_reg_idx*16 + start_offset_idx;j++) begin
                            vrf_bit_strobe_temp_sew8[dest_reg_idx*16+j] = 8'h00;
                        end
                        for(int i = start_reg_idx*16 + start_offset_idx;i < 16;i++) begin
                            vrf_sew8[dest_reg_idx*16+i] = vrf_sew8[src2_reg_idx*16+i];
                            vrf_bit_strobe_temp_sew8[dest_reg_idx*16+i] = 8'hff;
                        end
                    end
                    SEW16: begin
                        for(int j = 0;j < start_reg_idx*8 + start_offset_idx;j++) begin
                            vrf_bit_strobe_temp_sew16[dest_reg_idx*8+j] = 16'h0000;
                        end
                        for(int i = start_reg_idx*8 + start_offset_idx;i < 8;i++) begin
                            vrf_sew16[dest_reg_idx*8+i] = vrf_sew16[src2_reg_idx*8+i];
                            vrf_bit_strobe_temp_sew16[dest_reg_idx*8+i] = 16'hffff;
                        end
                    end
                    SEW32: begin
                        for(int j = 0;j < start_reg_idx*4 + start_offset_idx;j++) begin
                            vrf_bit_strobe_temp_sew32[dest_reg_idx*4+j] = 32'h00000000;
                        end
                        for(int i = start_reg_idx*4 + start_offset_idx;i < 4;i++) begin
                            vrf_sew32[dest_reg_idx*4+i] = vrf_sew32[src2_reg_idx*4+i];
                            vrf_bit_strobe_temp_sew32[dest_reg_idx*4+i] = 32'hffffffff;
                        end
                    end
                    default: begin
                        `uvm_error("VMV4R CHECK", "NO VAILD SEW!")
                    end
                endcase
                    //rvm.vrf[dest_reg_idx] = rvm.vrf[src2_reg_idx];
                    //rvm.vrf_bit_strobe_temp[dest_reg_idx] = 128'hffff_ffff_ffff_ffff_ffff_ffff_ffff_ffff;
                end 
                case(rvm.vsew)
                    SEW8:begin
                        rvm.vrf=vrf_sew8;
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew8;
                    end
                    SEW16: begin
                        rvm.vrf=vrf_sew16;
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew16;
                    end
                    SEW32: begin
                        rvm.vrf=vrf_sew32;
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew32;
                    end
                    default: begin
                        `uvm_error("VMV1R CHECK", "NO VAILD SEW!")
                    end
                endcase
                `uvm_info("pmt_processor",$sformatf("VMV1R FUNC Src  vrf[%0d] == %0x", src2_reg_idx, rvm.vrf[src2_reg_idx]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VMV1R FUNC Dest vrf[%0d] == %0x", dest_reg_idx, rvm.vrf[dest_reg_idx]),UVM_LOW)
    endfunction
    function void pmt_processor::VMV2R(input rvv_behavior_model rvm);
                logic [7:0] start_reg_idx;
                logic [7:0] start_offset_idx;
                case(rvm.vsew)
                    SEW8: begin 
                        start_reg_idx = rvm.vstart/16;
                        start_offset_idx = rvm.vstart%16;
                    end
                    SEW16: begin 
                        start_reg_idx = rvm.vstart/8;
                        start_offset_idx = rvm.vstart%8;
                    end
                    SEW32: begin 
                        start_reg_idx = rvm.vstart/4;
                        start_offset_idx = rvm.vstart%4;
                    end
                    default: begin
                        `uvm_error("VMV2R CHECK", "NO VAILD SEW!")
                    end
                endcase
                `uvm_info("pmt_processor",$sformatf("Origin VMV2R FUNC Src  vrf[%0d] == %0x", src2_reg_idx, rvm.vrf[src2_reg_idx]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("Origin VMV2R FUNC Src  vrf[%0d] == %0x", src2_reg_idx+1, rvm.vrf[src2_reg_idx+1]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("Origin VMV2R FUNC Dest vrf[%0d] == %0x", dest_reg_idx, rvm.vrf[dest_reg_idx]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("Origin VMV2R FUNC Dest vrf[%0d] == %0x", dest_reg_idx+1, rvm.vrf[dest_reg_idx+1]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("start_reg_idx = %0x, start_offset_idx = %0x", start_reg_idx, start_offset_idx),UVM_LOW)
                if((rvm.vstart < 32 && (rvm.vsew == SEW8)) | (rvm.vstart < 16 && (rvm.vsew == SEW16)) | (rvm.vstart < 8 && (rvm.vsew == SEW32))  )begin
                    case(rvm.vsew)
                    SEW8: begin 
                        for(int j = 0;j < start_reg_idx*16 + start_offset_idx;j++) begin
                            vrf_bit_strobe_temp_sew8[dest_reg_idx*16+j] = 8'h00;
                        end
                        for(int i = start_reg_idx*16 + start_offset_idx;i < 32;i++) begin
                            vrf_sew8[dest_reg_idx*16+i] = vrf_sew8[src2_reg_idx*16+i];
                            vrf_bit_strobe_temp_sew8[dest_reg_idx*16+i] = 8'hff;
                        end
                    end
                    SEW16: begin
                        for(int j = 0;j < start_reg_idx*8 + start_offset_idx;j++) begin
                            vrf_bit_strobe_temp_sew16[dest_reg_idx*8+j] = 16'h0000;
                        end
                        for(int i = start_reg_idx*8 + start_offset_idx;i < 16;i++) begin
                            vrf_sew16[dest_reg_idx*8+i] = vrf_sew16[src2_reg_idx*8+i];
                            vrf_bit_strobe_temp_sew16[dest_reg_idx*8+i] = 16'hffff;
                        end
                    end
                    SEW32: begin
                        for(int j = 0;j < start_reg_idx*4 + start_offset_idx;j++) begin
                            vrf_bit_strobe_temp_sew32[dest_reg_idx*4+j] = 32'h00000000;
                        end
                        for(int i = start_reg_idx*4 + start_offset_idx;i < 8;i++) begin
                            vrf_sew32[dest_reg_idx*4+i] = vrf_sew32[src2_reg_idx*4+i];
                            vrf_bit_strobe_temp_sew32[dest_reg_idx*4+i] = 32'hffffffff;
                        end
                    end
                    default: begin
                        `uvm_error("VMV4R CHECK", "NO VAILD SEW!")
                    end
                    endcase
                    //rvm.vrf[dest_reg_idx] = rvm.vrf[src2_reg_idx];
                    //rvm.vrf[dest_reg_idx+1] = rvm.vrf[src2_reg_idx+1];
                    //rvm.vrf_bit_strobe_temp[dest_reg_idx] = 128'hffff_ffff_ffff_ffff_ffff_ffff_ffff_ffff;
                    //rvm.vrf_bit_strobe_temp[dest_reg_idx+1] = 128'hffff_ffff_ffff_ffff_ffff_ffff_ffff_ffff;
                end 
            case(rvm.vsew)
                    SEW8:begin
                        rvm.vrf=vrf_sew8;
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew8;
                    end
                    SEW16: begin
                        rvm.vrf=vrf_sew16;
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew16;
                    end
                    SEW32: begin
                        rvm.vrf=vrf_sew32;
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew32;
                    end
                    default: begin
                        `uvm_error("VMV8R CHECK", "NO VAILD SEW!")
                    end
                endcase
                `uvm_info("pmt_processor",$sformatf("VMV2R FUNC Src  vrf[%0d] == %0x", src2_reg_idx, rvm.vrf[src2_reg_idx]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VMV2R FUNC Src  vrf[%0d] == %0x", src2_reg_idx+1, rvm.vrf[src2_reg_idx+1]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VMV2R FUNC Dest vrf[%0d] == %0x", dest_reg_idx, rvm.vrf[dest_reg_idx]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VMV2R FUNC Dest vrf[%0d] == %0x", dest_reg_idx+1, rvm.vrf[dest_reg_idx+1]),UVM_LOW)
    endfunction
    function void pmt_processor::VMV4R(input rvv_behavior_model rvm);
                logic [7:0] start_reg_idx;
                logic [7:0] start_offset_idx;
                case(rvm.vsew)
                    SEW8: begin 
                        start_reg_idx = rvm.vstart/16;
                        start_offset_idx = rvm.vstart%16;
                    end
                    SEW16: begin 
                        start_reg_idx = rvm.vstart/8;
                        start_offset_idx = rvm.vstart%8;
                    end
                    SEW32: begin 
                        start_reg_idx = rvm.vstart/4;
                        start_offset_idx = rvm.vstart%4;
                    end
                    default: begin
                        `uvm_error("VMV4R CHECK", "NO VAILD SEW!")
                    end
                endcase
                `uvm_info("pmt_processor",$sformatf("Origin VMV4R FUNC Src  vrf[%0d] == %0x", src2_reg_idx, rvm.vrf[src2_reg_idx]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("Origin VMV4R FUNC Src  vrf[%0d] == %0x", src2_reg_idx+1, rvm.vrf[src2_reg_idx+1]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("Origin VMV4R FUNC Src  vrf[%0d] == %0x", src2_reg_idx+2, rvm.vrf[src2_reg_idx+2]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("Origin VMV4R FUNC Src  vrf[%0d] == %0x", src2_reg_idx+3, rvm.vrf[src2_reg_idx+3]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("Origin VMV4R FUNC Dest vrf[%0d] == %0x", dest_reg_idx, rvm.vrf[dest_reg_idx]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("Origin VMV4R FUNC Dest vrf[%0d] == %0x", dest_reg_idx+1, rvm.vrf[dest_reg_idx+1]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("Origin VMV4R FUNC Dest vrf[%0d] == %0x", dest_reg_idx+2, rvm.vrf[dest_reg_idx+2]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("Origin VMV4R FUNC Dest vrf[%0d] == %0x", dest_reg_idx+3, rvm.vrf[dest_reg_idx+3]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("start_reg_idx = %0x, start_offset_idx = %0x", start_reg_idx, start_offset_idx),UVM_LOW)
                if((rvm.vstart < 64 && (rvm.vsew == SEW8)) | (rvm.vstart < 32 && (rvm.vsew == SEW16)) | (rvm.vstart < 16 && (rvm.vsew == SEW32))  )begin
                case(rvm.vsew)
                    SEW8: begin 
                        for(int j = 0;j < start_reg_idx*16 + start_offset_idx;j++) begin
                            vrf_bit_strobe_temp_sew8[dest_reg_idx*16+j] = 8'h00;
                        end
                        for(int i = start_reg_idx*16 + start_offset_idx;i < 64;i++) begin
                            vrf_sew8[dest_reg_idx*16+i] = vrf_sew8[src2_reg_idx*16+i];
                            vrf_bit_strobe_temp_sew8[dest_reg_idx*16+i] = 8'hff;
                        end
                    end
                    SEW16: begin
                        for(int j = 0;j < start_reg_idx*8 + start_offset_idx;j++) begin
                            vrf_bit_strobe_temp_sew16[dest_reg_idx*8+j] = 16'h0000;
                        end
                        for(int i = start_reg_idx*8 + start_offset_idx;i < 32;i++) begin
                            vrf_sew16[dest_reg_idx*8+i] = vrf_sew16[src2_reg_idx*8+i];
                            vrf_bit_strobe_temp_sew16[dest_reg_idx*8+i] = 16'hffff;
                        end
                    end
                    SEW32: begin
                        for(int j = 0;j < start_reg_idx*4 + start_offset_idx;j++) begin
                            vrf_bit_strobe_temp_sew32[dest_reg_idx*4+j] = 32'h00000000;
                        end
                        for(int i = start_reg_idx*4 + start_offset_idx;i < 16;i++) begin
                            vrf_sew32[dest_reg_idx*4+i] = vrf_sew32[src2_reg_idx*4+i];
                            vrf_bit_strobe_temp_sew32[dest_reg_idx*4+i] = 32'hffffffff;
                        end
                    end
                    default: begin
                        `uvm_error("VMV4R CHECK", "NO VAILD SEW!")
                    end
                endcase
                end 
                case(rvm.vsew)
                    SEW8:begin
                        rvm.vrf=vrf_sew8;
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew8;
                    end
                    SEW16: begin
                        rvm.vrf=vrf_sew16;
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew16;
                    end
                    SEW32: begin
                        rvm.vrf=vrf_sew32;
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew32;
                    end
                    default: begin
                        `uvm_error("VMV8R CHECK", "NO VAILD SEW!")
                    end
                endcase
                `uvm_info("pmt_processor",$sformatf("VMV4R FUNC Src  vrf[%0d] == %0x", src2_reg_idx, rvm.vrf[src2_reg_idx]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VMV4R FUNC Src  vrf[%0d] == %0x", src2_reg_idx+1, rvm.vrf[src2_reg_idx+1]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VMV4R FUNC Src  vrf[%0d] == %0x", src2_reg_idx+2, rvm.vrf[src2_reg_idx+2]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VMV4R FUNC Src  vrf[%0d] == %0x", src2_reg_idx+3, rvm.vrf[src2_reg_idx+3]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VMV4R FUNC Dest vrf[%0d] == %0x", dest_reg_idx, rvm.vrf[dest_reg_idx]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VMV4R FUNC Dest vrf[%0d] == %0x", dest_reg_idx+1, rvm.vrf[dest_reg_idx+1]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VMV4R FUNC Dest vrf[%0d] == %0x", dest_reg_idx+2, rvm.vrf[dest_reg_idx+2]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VMV4R FUNC Dest vrf[%0d] == %0x", dest_reg_idx+3, rvm.vrf[dest_reg_idx+3]),UVM_LOW)
    endfunction
    function void pmt_processor::VMV8R(input rvv_behavior_model rvm);
                logic [7:0] vmv8r_evl;
                logic [7:0] start_reg_idx;
                logic [7:0] start_offset_idx;
                case(rvm.vsew)
                    SEW8: begin 
                        vmv8r_evl = 128;
                        start_reg_idx = rvm.vstart/16;
                        start_offset_idx = rvm.vstart%16;
                    end
                    SEW16: begin 
                        vmv8r_evl = 64;
                        start_reg_idx = rvm.vstart/8;
                        start_offset_idx = rvm.vstart%8;
                    end
                    SEW32: begin 
                        vmv8r_evl = 32;
                        start_reg_idx = rvm.vstart/4;
                        start_offset_idx = rvm.vstart%4;
                    end
                    default: begin vmv8r_evl = 0;
                        `uvm_error("VMV8R CHECK", "NO VAILD SEW!")
                    end
                endcase
                `uvm_info("pmt_processor",$sformatf("Origin VMV8R FUNC Src  vrf[%0d] == %0x", src2_reg_idx, rvm.vrf[src2_reg_idx]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("Origin VMV8R FUNC Src  vrf[%0d] == %0x", src2_reg_idx+1, rvm.vrf[src2_reg_idx+1]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("Origin VMV8R FUNC Src  vrf[%0d] == %0x", src2_reg_idx+2, rvm.vrf[src2_reg_idx+2]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("Origin VMV8R FUNC Src  vrf[%0d] == %0x", src2_reg_idx+3, rvm.vrf[src2_reg_idx+3]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("Origin VMV8R FUNC Src  vrf[%0d] == %0x", src2_reg_idx+4, rvm.vrf[src2_reg_idx+4]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("Origin VMV8R FUNC Src  vrf[%0d] == %0x", src2_reg_idx+5, rvm.vrf[src2_reg_idx+5]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("Origin VMV8R FUNC Src  vrf[%0d] == %0x", src2_reg_idx+6, rvm.vrf[src2_reg_idx+6]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("Origin VMV8R FUNC Src  vrf[%0d] == %0x", src2_reg_idx+7, rvm.vrf[src2_reg_idx+7]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("Origin VMV8R FUNC Dest vrf[%0d] == %0x", dest_reg_idx, rvm.vrf[dest_reg_idx]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("Origin VMV8R FUNC Dest vrf[%0d] == %0x", dest_reg_idx+1, rvm.vrf[dest_reg_idx+1]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("Origin VMV8R FUNC Dest vrf[%0d] == %0x", dest_reg_idx+2, rvm.vrf[dest_reg_idx+2]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("Origin VMV8R FUNC Dest vrf[%0d] == %0x", dest_reg_idx+3, rvm.vrf[dest_reg_idx+3]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("Origin VMV8R FUNC Dest vrf[%0d] == %0x", dest_reg_idx+4, rvm.vrf[dest_reg_idx+4]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("Origin VMV8R FUNC Dest vrf[%0d] == %0x", dest_reg_idx+5, rvm.vrf[dest_reg_idx+5]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("Origin VMV8R FUNC Dest vrf[%0d] == %0x", dest_reg_idx+6, rvm.vrf[dest_reg_idx+6]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("Origin VMV8R FUNC Dest vrf[%0d] == %0x", dest_reg_idx+7, rvm.vrf[dest_reg_idx+7]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("start_reg_idx = %0x, start_offset_idx = %0x", start_reg_idx, start_offset_idx),UVM_LOW)
                if((rvm.vstart < 128 && (rvm.vsew == SEW8)) | (rvm.vstart < 64 && (rvm.vsew == SEW16)) | (rvm.vstart < 32 && (rvm.vsew == SEW32))  )begin
                case(rvm.vsew)
                    SEW8: begin 
                        for(int j = 0;j < start_reg_idx*16 + start_offset_idx;j++) begin
                            vrf_bit_strobe_temp_sew8[dest_reg_idx*16+j] = 8'h00;
                        end
                        for(int i = start_reg_idx*16 + start_offset_idx;i < 128;i++) begin
                            vrf_sew8[dest_reg_idx*16+i] = vrf_sew8[src2_reg_idx*16+i];
                            vrf_bit_strobe_temp_sew8[dest_reg_idx*16+i] = 8'hff;
                        end
                    end
                    SEW16: begin
                        for(int j = 0;j < start_reg_idx*8 + start_offset_idx;j++) begin
                            vrf_bit_strobe_temp_sew16[dest_reg_idx*8+j] = 16'h0000;
                        end
                        for(int i = start_reg_idx*8 + start_offset_idx;i < 64;i++) begin
                            vrf_sew16[dest_reg_idx*8+i] = vrf_sew16[src2_reg_idx*8+i];
                            vrf_bit_strobe_temp_sew16[dest_reg_idx*8+i] = 16'hffff;
                        end
                    end
                    SEW32: begin
                        for(int j = 0;j < start_reg_idx*4 + start_offset_idx;j++) begin
                            vrf_bit_strobe_temp_sew32[dest_reg_idx*4+j] = 32'h00000000;
                        end
                        for(int i = start_reg_idx*4 + start_offset_idx;i < 32;i++) begin
                            vrf_sew32[dest_reg_idx*4+i] = vrf_sew32[src2_reg_idx*4+i];
                            vrf_bit_strobe_temp_sew32[dest_reg_idx*4+i] = 32'hffffffff;
                        end
                    end
                    default: begin
                        `uvm_error("VMV8R CHECK", "NO VAILD SEW!")
                    end
                endcase
                end 
                case(rvm.vsew)
                    SEW8:begin
                        rvm.vrf=vrf_sew8;
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew8;
                    end
                    SEW16: begin
                        rvm.vrf=vrf_sew16;
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew16;
                    end
                    SEW32: begin
                        rvm.vrf=vrf_sew32;
                        rvm.vrf_bit_strobe_temp = vrf_bit_strobe_temp_sew32;
                    end
                    default: begin
                        `uvm_error("VMV8R CHECK", "NO VAILD SEW!")
                    end
                endcase
                `uvm_info("pmt_processor",$sformatf("VMV8R FUNC Src  vrf[%0d] == %0x", src2_reg_idx, rvm.vrf[src2_reg_idx]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VMV8R FUNC Src  vrf[%0d] == %0x", src2_reg_idx+1, rvm.vrf[src2_reg_idx+1]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VMV8R FUNC Src  vrf[%0d] == %0x", src2_reg_idx+2, rvm.vrf[src2_reg_idx+2]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VMV8R FUNC Src  vrf[%0d] == %0x", src2_reg_idx+3, rvm.vrf[src2_reg_idx+3]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VMV8R FUNC Src  vrf[%0d] == %0x", src2_reg_idx+4, rvm.vrf[src2_reg_idx+4]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VMV8R FUNC Src  vrf[%0d] == %0x", src2_reg_idx+5, rvm.vrf[src2_reg_idx+5]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VMV8R FUNC Src  vrf[%0d] == %0x", src2_reg_idx+6, rvm.vrf[src2_reg_idx+6]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VMV8R FUNC Src  vrf[%0d] == %0x", src2_reg_idx+7, rvm.vrf[src2_reg_idx+7]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VMV8R FUNC Dest vrf[%0d] == %0x", dest_reg_idx, rvm.vrf[dest_reg_idx]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VMV8R FUNC Dest vrf[%0d] == %0x", dest_reg_idx+1, rvm.vrf[dest_reg_idx+1]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VMV8R FUNC Dest vrf[%0d] == %0x", dest_reg_idx+2, rvm.vrf[dest_reg_idx+2]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VMV8R FUNC Dest vrf[%0d] == %0x", dest_reg_idx+3, rvm.vrf[dest_reg_idx+3]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VMV8R FUNC Dest vrf[%0d] == %0x", dest_reg_idx+4, rvm.vrf[dest_reg_idx+4]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VMV8R FUNC Dest vrf[%0d] == %0x", dest_reg_idx+5, rvm.vrf[dest_reg_idx+5]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VMV8R FUNC Dest vrf[%0d] == %0x", dest_reg_idx+6, rvm.vrf[dest_reg_idx+6]),UVM_LOW)
                `uvm_info("pmt_processor",$sformatf("VMV8R FUNC Dest vrf[%0d] == %0x", dest_reg_idx+7, rvm.vrf[dest_reg_idx+7]),UVM_LOW)
    endfunction


`endif // RVV_BEHAVIOR_MODEL
