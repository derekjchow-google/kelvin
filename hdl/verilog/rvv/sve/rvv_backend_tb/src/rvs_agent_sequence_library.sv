`ifndef RVS_SEQUENCER_SEQUENCE_LIBRARY__SV
`define RVS_SEQUENCER_SEQUENCE_LIBRARY__SV

`include "inst_description.svh"
typedef class rvs_transaction;

class rvs_sequencer_sequence_library extends uvm_sequence_library # (rvs_transaction);
  
  `uvm_object_utils(rvs_sequencer_sequence_library)
  `uvm_sequence_library_utils(rvs_sequencer_sequence_library)

  function new(string name = "simple_seq_lib");
    super.new(name);
    init_sequence_library();
  endfunction

endclass  

class base_sequence extends uvm_sequence #(rvs_transaction);
  static int inst_cnt = 0;
  `uvm_object_utils(base_sequence)

  function new(string name = "base_seq");
    super.new(name);
	`ifdef UVM_POST_VERSION_1_1
     set_automatic_phase_objection(1);
    `endif
  endfunction:new

  `ifdef UVM_VERSION_1_0
  virtual task pre_body();
    if (starting_phase != null)
      starting_phase.raise_objection(this);
  endtask:pre_body

  virtual task post_body();
    if (starting_phase != null)
      starting_phase.drop_objection(this);
  endtask:post_body
  `endif
  
  `ifdef UVM_VERSION_1_1
  virtual task pre_start();
    if((get_parent_sequence() == null) && (starting_phase != null))
      starting_phase.raise_objection(this, "Starting");
  endtask:pre_start

  virtual task post_start();
    if ((get_parent_sequence() == null) && (starting_phase != null))
      starting_phase.drop_objection(this, "Ending");
  endtask:post_start
  `endif

endclass

//===========================================================
// To debug testbench.
//===========================================================
class zero_seq extends base_sequence;
  `uvm_object_utils(zero_seq)
  `uvm_add_to_seq_lib(zero_seq,rvs_sequencer_sequence_library)

  function new(string name = "zero_seq");
    super.new(name);
	`ifdef UVM_POST_VERSION_1_1
     set_automatic_phase_objection(1);
    `endif
  endfunction:new

  virtual task body();
    for(int i=0; i<10; i++) begin
      req = new("req");
      start_item(req);
      assert(req.randomize() with {
        use_vlmax == 1;
        pc == inst_cnt;

        vtype.vill ==  'b0;
        vtype.rsv  ==  'b0;  
        vtype.vma  ==  'b0;
        vtype.vta  ==  'b0;
        vtype.vsew ==  SEW8;
        vtype.vlmul == LMUL1;

        inst_type == ALU;
        alu_inst == VADD;
        dest_type == VRF; dest_idx == 2;
        src1_type == VRF; src1_idx == 1;
        src2_type == VRF; src2_idx == 2;
        vm == 1; 
      });
      finish_item(req);
      inst_cnt++;
      req = new("req");
      start_item(req);
      assert(req.randomize() with {
        use_vlmax == 1;
        pc == inst_cnt;

        vtype.vill ==  'b0;
        vtype.rsv  ==  'b0;  
        vtype.vma  ==  'b0;
        vtype.vta  ==  'b1;
        vtype.vsew ==  SEW8;
        vtype.vlmul == LMUL1;

        inst_type == ALU;
        alu_inst == VADD;
        dest_type == VRF; dest_idx == 16;
        src1_type == VRF; src1_idx == 1;
        src2_type == VRF; src2_idx == 16;
        rs_data == 123;
        vm == 1; 
      });
      finish_item(req);
      inst_cnt++;
    end

  endtask
endclass: zero_seq

//===========================================================
// ALU direct test sequences
//===========================================================
//-----------------------------------------------------------
// Single instruction sequence
//-----------------------------------------------------------
class alu_smoke_vv_seq extends base_sequence;
  `uvm_object_utils(alu_smoke_vv_seq)
  `uvm_add_to_seq_lib(alu_smoke_vv_seq,rvs_sequencer_sequence_library)

  alu_inst_e alu_inst;
  bit vm;
  function new(string name = "alu_smoke_vv_seq");
    super.new(name);
	`ifdef UVM_POST_VERSION_1_1
     set_automatic_phase_objection(1);
    `endif
  endfunction:new

  virtual task body();
    req = new("req");
    start_item(req);
    assert(req.randomize() with {
      use_vlmax == 1;
      pc == inst_cnt;

      vtype.vsew ==  SEW16;
      vtype.vlmul inside {LMUL1_2, LMUL2};

      inst_type == ALU;
      alu_inst == local::alu_inst;
      dest_type == VRF; dest_idx == 16;
      src2_type == VRF; src2_idx == 8;
      src1_type == VRF; src1_idx == 2;
      vm == local::vm; 
    });
    finish_item(req);
    inst_cnt++;
  endtask

  task run_inst(alu_inst_e inst, uvm_sequencer_base sqr, bit vm = 0);
    this.alu_inst = inst;
    this.vm = vm;
    this.start(sqr);
  endtask: run_inst
endclass: alu_smoke_vv_seq

class alu_smoke_ext_seq extends base_sequence;
  `uvm_object_utils(alu_smoke_ext_seq)
  `uvm_add_to_seq_lib(alu_smoke_ext_seq,rvs_sequencer_sequence_library)
    
  alu_inst_e alu_inst;
  function new(string name = "alu_smoke_ext_seq");
    super.new(name);
	  `ifdef UVM_POST_VERSION_1_1
      set_automatic_phase_objection(1);
    `endif
  endfunction:new

  virtual task body();
    for(vext_e vext_func = vext_func.first();vext_func != vext_func.last(); vext_func = vext_func.next()) begin
      req = new("req");
      start_item(req);
      assert(req.randomize() with {
        use_vlmax == 1;
        pc == inst_cnt;

        vtype.vsew == SEW32;
        vtype.vlmul inside {LMUL1_2, LMUL2};

        inst_type == ALU;
        alu_inst == local::alu_inst;
        dest_type == VRF; dest_idx == 16;
        src2_type == VRF; src2_idx == 8;
        src1_type == FUNC; src1_idx == local::vext_func;
        vm == 0;
      });
      finish_item(req);
      inst_cnt++;
    end
  endtask
  task run_inst(alu_inst_e inst, uvm_sequencer_base sqr);
    this.alu_inst = inst;
    this.start(sqr);
  endtask: run_inst
endclass: alu_smoke_ext_seq

class alu_smoke_vx_seq extends base_sequence;
  `uvm_object_utils(alu_smoke_vx_seq)
  `uvm_add_to_seq_lib(alu_smoke_vx_seq,rvs_sequencer_sequence_library)

  alu_inst_e alu_inst;
  bit vm;
  function new(string name = "alu_smoke_vx_seq");
    super.new(name);
	`ifdef UVM_POST_VERSION_1_1
     set_automatic_phase_objection(1);
    `endif
  endfunction:new

  virtual task body();
    req = new("req");
    start_item(req);
    assert(req.randomize() with {
      use_vlmax == 1;
      pc == inst_cnt;

      vtype.vsew ==  SEW16;
      vtype.vlmul inside {LMUL1_2, LMUL2};

      inst_type == ALU;
      alu_inst == local::alu_inst;
      dest_type == VRF; dest_idx == 16;
      src2_type == VRF; src2_idx == 8;
      src1_type == XRF; src1_idx == 2;
      vm == local::vm; 
    });
    finish_item(req);
    inst_cnt++;
  endtask

  task run_inst(alu_inst_e inst, uvm_sequencer_base sqr, bit vm = 0);
    this.alu_inst = inst;
    this.vm = vm;
    this.start(sqr);
  endtask: run_inst
endclass: alu_smoke_vx_seq

class alu_smoke_vmerge_seq extends base_sequence;
  `uvm_object_utils(alu_smoke_vmerge_seq)
  `uvm_add_to_seq_lib(alu_smoke_vmerge_seq,rvs_sequencer_sequence_library)

  alu_inst_e alu_inst;
  bit vm;
  function new(string name = "alu_smoke_vmerge_seq");
    super.new(name);
	`ifdef UVM_POST_VERSION_1_1
     set_automatic_phase_objection(1);
    `endif
  endfunction:new

  virtual task body();
    req = new("req");
    // vmerge
    start_item(req);
    assert(req.randomize() with {
      use_vlmax == 1;
      pc == inst_cnt;

      vtype.vsew ==  SEW16;
      vtype.vlmul inside {LMUL1_2, LMUL2};

      inst_type == ALU;
      alu_inst == local::alu_inst;
      dest_type == VRF; dest_idx == 16;
      src2_type == VRF; src2_idx == 8;
      src1_type == VRF; src1_idx == 2;
      vm == 0; 
    });
    finish_item(req);
    inst_cnt++;
    
    // vmv.v
    start_item(req);
    assert(req.randomize() with {
      use_vlmax == 1;
      pc == inst_cnt;

      vtype.vsew ==  SEW16;
      vtype.vlmul inside {LMUL1_2, LMUL2};

      inst_type == ALU;
      alu_inst == local::alu_inst;
      dest_type == VRF; dest_idx == 16;
      src2_type == VRF; src2_idx == 0;
      src1_type == VRF; src1_idx == 2;
      vm == 1; 
    });
    finish_item(req);
    inst_cnt++;
  endtask

  task run_inst(alu_inst_e inst, uvm_sequencer_base sqr);
    this.alu_inst = inst;
    this.start(sqr);
  endtask: run_inst
endclass: alu_smoke_vmerge_seq

class alu_smoke_vmunary0_seq extends base_sequence;
  `uvm_object_utils(alu_smoke_vmunary0_seq)
  `uvm_add_to_seq_lib(alu_smoke_vmunary0_seq,rvs_sequencer_sequence_library)
    
  alu_inst_e alu_inst;
  function new(string name = "alu_smoke_vmunary0_seq");
    super.new(name);
	  `ifdef UVM_POST_VERSION_1_1
      set_automatic_phase_objection(1);
    `endif
  endfunction:new

  virtual task body();
    for(vmunary0_e vmunary0_func = vmunary0_func.first();vmunary0_func != vmunary0_func.last(); vmunary0_func = vmunary0_func.next()) begin
      req = new("req");
      start_item(req);
      assert(req.randomize() with {
        use_vlmax == 1;
        pc == inst_cnt;

        vtype.vsew == SEW16;
        vtype.vlmul inside {LMUL1_2, LMUL2};

        inst_type == ALU;
        alu_inst == local::alu_inst;
        dest_type == VRF; dest_idx == 16;
        src2_type == VRF; src2_idx == 8;
        src1_type == FUNC; src1_idx == local::vmunary0_func;
        vm == 0;
      });
      finish_item(req);
      inst_cnt++;
    end
  endtask
  task run_inst(alu_inst_e inst, uvm_sequencer_base sqr);
    this.alu_inst = inst;
    this.start(sqr);
  endtask: run_inst
endclass: alu_smoke_vmunary0_seq

//-----------------------------------------------------------
// Iterate base
//-----------------------------------------------------------
class alu_iterate_base_sequence extends base_sequence;
  `uvm_object_utils(alu_iterate_base_sequence)
  `uvm_add_to_seq_lib(alu_iterate_base_sequence,rvs_sequencer_sequence_library)
   
  bit vm;
  sew_e sew;
  lmul_e lmul;
  alu_inst_e alu_inst;
  oprand_type_e src1_type;

  test_rand_type_e rand_type = ITER;
  int inst_num = 1;

  function new(string name = "alu_iterate_base_sequence");
    super.new(name);
	  `ifdef UVM_POST_VERSION_1_1
      set_automatic_phase_objection(1);
    `endif
  endfunction:new

  task run_inst_iter(alu_inst_e inst, uvm_sequencer_base sqr, bit vm = 0);
    this.alu_inst = inst;
    this.vm = vm;
    this.rand_type = ITER;
    this.start(sqr);
  endtask: run_inst_iter
  
  task run_inst_rand(alu_inst_e inst, uvm_sequencer_base sqr, int inst_num = 50);
    this.alu_inst = inst;
    this.rand_type = RAND;
    this.inst_num = inst_num;
    this.start(sqr);
  endtask: run_inst_rand

endclass: alu_iterate_base_sequence

//-----------------------------------------------------------
// Iterate each lmul/sew for .vv/.vx/.vi
//-----------------------------------------------------------
class alu_iterate_vv_vx_vi_seq extends alu_iterate_base_sequence;
  `uvm_object_utils(alu_iterate_vv_vx_vi_seq)
  `uvm_add_to_seq_lib(alu_iterate_vv_vx_vi_seq,rvs_sequencer_sequence_library)

  function new(string name = "alu_iterate_vv_vx_vi_seq");
    super.new(name);
	  `ifdef UVM_POST_VERSION_1_1
      set_automatic_phase_objection(1);
    `endif
  endfunction:new

  virtual task body();
    if(rand_type == RAND) begin
      repeat(inst_num) begin
        req = new("req");
        start_item(req);
        assert(req.randomize() with {
          // use_vlmax == 1;
          pc == inst_cnt;

          vtype.vlmul dist {
            LMUL1_4 := 10,
            LMUL1_2 := 20,
            LMUL1   := 20,
            LMUL2   := 30,
            LMUL4   :=  5,
            LMUL8   := 15 
          };

          inst_type == ALU;
          alu_inst == local::alu_inst;

          dest_type == VRF;
          src2_type == VRF;
          src1_type inside {VRF, XRF, IMM};
        });
        finish_item(req);
        inst_cnt++;
      end // repeat(inst_num)
    end else begin
      for(lmul = lmul.first(); lmul != lmul.last(); lmul =lmul.next()) begin
        for(sew = sew.first(); sew != sew.last(); sew =sew.next()) begin
          req = new("req");
          start_item(req);
          assert(req.randomize() with {
            use_vlmax == 1;
            pc == inst_cnt;

            vtype.vsew ==  local::sew;
            vtype.vlmul == local::lmul;

            inst_type == ALU;
            alu_inst == local::alu_inst;

            dest_type == VRF;
            src2_type == VRF;
            src1_type == VRF;
            vm == local::vm;
          });
          finish_item(req);
          inst_cnt++;
        end // sew
      end // lmul
      for(lmul = lmul.first(); lmul != lmul.last(); lmul =lmul.next()) begin
        for(sew = sew.first(); sew != sew.last(); sew =sew.next()) begin
          req = new("req");
          start_item(req);
          assert(req.randomize() with {
            use_vlmax == 1;
            pc == inst_cnt;

            vtype.vsew ==  local::sew;
            vtype.vlmul == local::lmul;

            inst_type == ALU;
            alu_inst == local::alu_inst;

            dest_type == VRF;
            src2_type == VRF;
            src1_type == XRF;
            vm == local::vm;
          });
          finish_item(req);
          inst_cnt++;
        end // sew
      end // lmul
      for(lmul = lmul.first(); lmul != lmul.last(); lmul =lmul.next()) begin
        for(sew = sew.first(); sew != sew.last(); sew =sew.next()) begin
          req = new("req");
          start_item(req);
          assert(req.randomize() with {
            use_vlmax == 1;
            pc == inst_cnt;

            vtype.vsew ==  local::sew;
            vtype.vlmul == local::lmul;

            inst_type == ALU;
            alu_inst == local::alu_inst;

            dest_type == VRF;
            src2_type == VRF;
            src1_type == IMM;
            vm == local::vm;
          });
          finish_item(req);
          inst_cnt++;
        end // sew
      end // lmul
    end
  endtask: body
endclass: alu_iterate_vv_vx_vi_seq

//-----------------------------------------------------------
// Iterate each lmul/sew for .vv/.vx/.vui
//-----------------------------------------------------------
class alu_iterate_vv_vx_vui_seq extends alu_iterate_base_sequence;
  `uvm_object_utils(alu_iterate_vv_vx_vui_seq)
  `uvm_add_to_seq_lib(alu_iterate_vv_vx_vui_seq,rvs_sequencer_sequence_library)
    
  function new(string name = "alu_iterate_vv_vx_vui_seq");
    super.new(name);
	  `ifdef UVM_POST_VERSION_1_1
      set_automatic_phase_objection(1);
    `endif
  endfunction:new

  virtual task body();
    if(rand_type == RAND) begin
      repeat(inst_num) begin
        req = new("req");
        start_item(req);
        assert(req.randomize() with {
          // use_vlmax == 1;
          pc == inst_cnt;

          vtype.vlmul dist {
            LMUL1_4 := 10,
            LMUL1_2 := 20,
            LMUL1   := 20,
            LMUL2   := 30,
            LMUL4   :=  5,
            LMUL8   := 15 
          };

          inst_type == ALU;
          alu_inst == local::alu_inst;

          dest_type == VRF;
          src2_type == VRF;
          src1_type inside {VRF, XRF, UIMM};
        });
        finish_item(req);
        inst_cnt++;
      end // repeat(inst_num)
    end else begin
      for(lmul = lmul.first(); lmul != lmul.last(); lmul =lmul.next()) begin
        for(sew = sew.first(); sew != sew.last(); sew =sew.next()) begin
          req = new("req");
          start_item(req);
          assert(req.randomize() with {
            use_vlmax == 1;
            pc == inst_cnt;

            vtype.vsew ==  local::sew;
            vtype.vlmul == local::lmul;

            inst_type == ALU;
            alu_inst == local::alu_inst;

            dest_type == VRF;
            src2_type == VRF;
            src1_type == VRF;
            vm == local::vm;
          });
          finish_item(req);
          inst_cnt++;
        end // sew
      end // lmul
      for(lmul = lmul.first(); lmul != lmul.last(); lmul =lmul.next()) begin
        for(sew = sew.first(); sew != sew.last(); sew =sew.next()) begin
          req = new("req");
          start_item(req);
          assert(req.randomize() with {
            use_vlmax == 1;
            pc == inst_cnt;

            vtype.vsew ==  local::sew;
            vtype.vlmul == local::lmul;

            inst_type == ALU;
            alu_inst == local::alu_inst;

            dest_type == VRF;
            src2_type == VRF;
            src1_type == XRF;
            vm == local::vm;
          });
          finish_item(req);
          inst_cnt++;
        end // sew
      end // lmul
      for(lmul = lmul.first(); lmul != lmul.last(); lmul =lmul.next()) begin
        for(sew = sew.first(); sew != sew.last(); sew =sew.next()) begin
          req = new("req");
          start_item(req);
          assert(req.randomize() with {
            use_vlmax == 1;
            pc == inst_cnt;

            vtype.vsew ==  local::sew;
            vtype.vlmul == local::lmul;

            inst_type == ALU;
            alu_inst == local::alu_inst;

            dest_type == VRF;
            src2_type == VRF;
            src1_type == UIMM;
            vm == local::vm;
          });
          finish_item(req);
          inst_cnt++;
        end // sew
      end // lmul
    end
  endtask: body

endclass: alu_iterate_vv_vx_vui_seq

//-----------------------------------------------------------
// Iterate each lmul/sew for .vv/.vx
//-----------------------------------------------------------
class alu_iterate_vv_vx_seq extends alu_iterate_base_sequence;
  `uvm_object_utils(alu_iterate_vv_vx_seq)
  `uvm_add_to_seq_lib(alu_iterate_vv_vx_seq,rvs_sequencer_sequence_library)
    
  function new(string name = "alu_iterate_vv_vx_seq");
    super.new(name);
	  `ifdef UVM_POST_VERSION_1_1
      set_automatic_phase_objection(1);
    `endif
  endfunction:new

  virtual task body();
    if(rand_type == RAND) begin
      repeat(inst_num) begin
        req = new("req");
        start_item(req);
        assert(req.randomize() with {
          // use_vlmax == 1;
          pc == local::inst_cnt;

          vtype.vlmul dist {
            LMUL1_4 := 10,
            LMUL1_2 := 20,
            LMUL1   := 20,
            LMUL2   := 30,
            LMUL4   :=  5,
            LMUL8   := 15 
          };

          inst_type == ALU;
          alu_inst == local::alu_inst;

          dest_type == VRF;
          src2_type == VRF;
          src1_type inside {VRF, XRF};
        });
        finish_item(req);
        inst_cnt++;
      end // repeat(inst_num)
    end else begin
      for(lmul = lmul.first(); lmul != lmul.last(); lmul =lmul.next()) begin
        for(sew = sew.first(); sew != sew.last(); sew =sew.next()) begin
          req = new("req");
          start_item(req);
          assert(req.randomize() with {
            use_vlmax == 1;
            pc == local::inst_cnt;

            vtype.vsew ==  local::sew;
            vtype.vlmul == local::lmul;

            inst_type == ALU;
            alu_inst == local::alu_inst;

            dest_type == VRF;
            src2_type == VRF;
            src1_type == VRF;
            vm == local::vm;
          });
          finish_item(req);
          inst_cnt++;
        end // sew
      end // lmul
      for(lmul = lmul.first(); lmul != lmul.last(); lmul =lmul.next()) begin
        for(sew = sew.first(); sew != sew.last(); sew =sew.next()) begin
          req = new("req");
          start_item(req);
          assert(req.randomize() with {
            use_vlmax == 1;
            pc == local::inst_cnt;

            vtype.vsew ==  local::sew;
            vtype.vlmul == local::lmul;

            inst_type == ALU;
            alu_inst == local::alu_inst;

            dest_type == VRF;
            src2_type == VRF;
            src1_type == XRF;
            vm == local::vm;
          });
          finish_item(req);
          inst_cnt++;
        end // sew
      end // lmul
    end
  endtask

endclass: alu_iterate_vv_vx_seq

//-----------------------------------------------------------
// Iterate each lmul/sew for .vv/.vx
//-----------------------------------------------------------
class alu_iterate_vx_vi_seq extends alu_iterate_base_sequence;
  `uvm_object_utils(alu_iterate_vx_vi_seq)
  `uvm_add_to_seq_lib(alu_iterate_vx_vi_seq,rvs_sequencer_sequence_library)
    
  function new(string name = "alu_iterate_vx_vi_seq");
    super.new(name);
	  `ifdef UVM_POST_VERSION_1_1
      set_automatic_phase_objection(1);
    `endif
  endfunction:new

  virtual task body();
    if(rand_type == RAND) begin
      repeat(inst_num) begin
        req = new("req");
        start_item(req);
        assert(req.randomize() with {
          // use_vlmax == 1;
          pc == local::inst_cnt;

          vtype.vlmul dist {
            LMUL1_4 := 10,
            LMUL1_2 := 20,
            LMUL1   := 20,
            LMUL2   := 30,
            LMUL4   :=  5,
            LMUL8   := 15 
          };

          inst_type == ALU;
          alu_inst == local::alu_inst;

          dest_type == VRF;
          src2_type == VRF;
          src1_type inside {XRF, IMM};
        });
        finish_item(req);
        inst_cnt++;
      end // repeat(inst_num)
    end else begin
      for(lmul = lmul.first(); lmul != lmul.last(); lmul =lmul.next()) begin
        for(sew = sew.first(); sew != sew.last(); sew =sew.next()) begin
          req = new("req");
          start_item(req);
          assert(req.randomize() with {
            use_vlmax == 1;
            pc == local::inst_cnt;

            vtype.vsew ==  local::sew;
            vtype.vlmul == local::lmul;

            inst_type == ALU;
            alu_inst == local::alu_inst;

            dest_type == VRF;
            src2_type == VRF;
            src1_type == XRF;
            vm == local::vm;
          });
          finish_item(req);
          inst_cnt++;
        end // sew
      end // lmul
      for(lmul = lmul.first(); lmul != lmul.last(); lmul =lmul.next()) begin
        for(sew = sew.first(); sew != sew.last(); sew =sew.next()) begin
          req = new("req");
          start_item(req);
          assert(req.randomize() with {
            use_vlmax == 1;
            pc == local::inst_cnt;

            vtype.vsew ==  local::sew;
            vtype.vlmul == local::lmul;

            inst_type == ALU;
            alu_inst == local::alu_inst;

            dest_type == VRF;
            src2_type == VRF;
            src1_type == IMM;
            vm == local::vm;
          });
          finish_item(req);
          inst_cnt++;
        end // sew
      end // lmul
    end
  endtask

endclass: alu_iterate_vx_vi_seq

//-----------------------------------------------------------
// Iterate vzext/vsext
//-----------------------------------------------------------
class alu_iterate_ext_seq extends alu_iterate_base_sequence;
  `uvm_object_utils(alu_iterate_ext_seq)
  `uvm_add_to_seq_lib(alu_iterate_ext_seq,rvs_sequencer_sequence_library)

  function new(string name = "alu_iterate_ext_seq");
    super.new(name);
	  `ifdef UVM_POST_VERSION_1_1
      set_automatic_phase_objection(1);
    `endif
  endfunction:new

  virtual task body();;
    if(rand_type == RAND) begin
      repeat(inst_num) begin
        req = new("req");
        start_item(req);
        assert(req.randomize() with {
          // use_vlmax == 1;
          pc == local::inst_cnt;

          src1_idx inside {VZEXT_VF4, VSEXT_VF4} -> vtype.vsew inside {SEW32};
          src1_idx inside {VZEXT_VF2, VSEXT_VF2} -> vtype.vsew inside {SEW16, SEW32};
          vtype.vlmul dist {
            LMUL1_4 :=  5,
            LMUL1_2 := 10,
            LMUL1   := 30,
            LMUL2   := 40,
            LMUL4   :=  5,
            LMUL8   := 10 
          };

          inst_type == ALU;
          alu_inst == local::alu_inst;

          dest_type == VRF;
          src2_type == VRF;
          src1_type == FUNC; src1_idx inside {VZEXT_VF4, VSEXT_VF4, VZEXT_VF2, VZEXT_VF2};
        });
        finish_item(req);
        inst_cnt++;
      end // repeat(inst_num)
    end else begin
      for(lmul_e lmul = lmul.first(); lmul != lmul.last(); lmul =lmul.next()) begin
        for(sew_e sew = sew.first(); sew != sew.last(); sew =sew.next()) begin
          for(vext_e vext_func = vext_func.first();vext_func != vext_func.last(); vext_func = vext_func.next()) begin
            if((vext_func inside {VZEXT_VF4, VSEXT_VF4}) && !(sew inside {SEW32})) continue;
            if((vext_func inside {VZEXT_VF2, VZEXT_VF2}) && !(sew inside {SEW16, SEW32})) continue;
            req = new("req");
            start_item(req);
            assert(req.randomize() with {
              use_vlmax == 1;
              pc == inst_cnt;

              vtype.vsew ==  local::sew;
              vtype.vlmul == local::lmul;

              inst_type == ALU;
              alu_inst == local::alu_inst;

              dest_type == VRF; 
              src2_type == VRF;
              src1_type == FUNC; src1_idx == local::vext_func;
              vm == local::vm;
            });
            finish_item(req);
            inst_cnt++;
          end
        end
      end
    end
  endtask

endclass: alu_iterate_ext_seq

//-----------------------------------------------------------
// Iterate vmerge/vmv.v
//-----------------------------------------------------------
class alu_iterate_vmerge_seq extends alu_iterate_base_sequence;
  `uvm_object_utils(alu_iterate_vmerge_seq)
  `uvm_add_to_seq_lib(alu_iterate_vmerge_seq,rvs_sequencer_sequence_library)
    
  function new(string name = "alu_iterate_vmerge_seq");
    super.new(name);
	  `ifdef UVM_POST_VERSION_1_1
      set_automatic_phase_objection(1);
    `endif
  endfunction:new

  virtual task body();
    if(rand_type == RAND) begin
      repeat(inst_num) begin
        req = new("req");
        start_item(req);
        assert(req.randomize() with {
          // use_vlmax == 1;
          pc == local::inst_cnt;

          vtype.vlmul dist {
            LMUL1_4 := 10,
            LMUL1_2 := 20,
            LMUL1   := 20,
            LMUL2   := 30,
            LMUL4   :=  5,
            LMUL8   := 15 
          };

          inst_type == ALU;
          alu_inst == local::alu_inst;

          dest_type == VRF;
          (local::vm == 1) -> (src2_type == UNUSE);
          (local::vm == 1) -> (src2_idx  == 0);
          (local::vm == 0) -> (src2_type == VRF);
          src1_type inside {VRF, XRF, IMM};
        });
        finish_item(req);
        inst_cnt++;
      end // repeat(inst_num)
    end else begin
      for(lmul = lmul.first(); lmul != lmul.last(); lmul =lmul.next()) begin
        for(sew = sew.first(); sew != sew.last(); sew =sew.next()) begin
          req = new("req");
          start_item(req);
          assert(req.randomize() with {
            use_vlmax == 1;
            pc == inst_cnt;

            vtype.vsew ==  local::sew;
            vtype.vlmul == local::lmul;

            inst_type == ALU;
            alu_inst == local::alu_inst;

            dest_type == VRF;
            (local::vm == 1) -> (src2_type == UNUSE);
            (local::vm == 1) -> (src2_idx  == 0);
            (local::vm == 0) -> (src2_type == VRF);
            src1_type == VRF;
            vm == local::vm;
          });
          finish_item(req);
          inst_cnt++;
        end
      end
      for(lmul = lmul.first(); lmul != lmul.last(); lmul =lmul.next()) begin
        for(sew = sew.first(); sew != sew.last(); sew =sew.next()) begin
          req = new("req");
          start_item(req);
          assert(req.randomize() with {
            use_vlmax == 1;
            pc == local::inst_cnt;

            vtype.vsew ==  local::sew;
            vtype.vlmul == local::lmul;

            inst_type == ALU;
            alu_inst == local::alu_inst;

            dest_type == VRF;
            (local::vm == 1) -> (src2_type == UNUSE);
            (local::vm == 1) -> (src2_idx  == 0);
            (local::vm == 0) -> (src2_type == VRF);
            src1_type == XRF;
            vm == local::vm;
          });
          finish_item(req);
          inst_cnt++;
        end
      end
      for(lmul = lmul.first(); lmul != lmul.last(); lmul =lmul.next()) begin
        for(sew = sew.first(); sew != sew.last(); sew =sew.next()) begin
          req = new("req");
          start_item(req);
          assert(req.randomize() with {
            use_vlmax == 1;
            pc == local::inst_cnt;

            vtype.vsew ==  local::sew;
            vtype.vlmul == local::lmul;

            inst_type == ALU;
            alu_inst == local::alu_inst;

            dest_type == VRF;
            (local::vm == 1) -> (src2_type == UNUSE);
            (local::vm == 1) -> (src2_idx  == 0);
            (local::vm == 0) -> (src2_type == VRF);
            src1_type == IMM; 
            vm == local::vm;
          });
          finish_item(req);
          inst_cnt++;
        end
      end
    end
  endtask

endclass: alu_iterate_vmerge_seq

//=================================================
// LDST direct test sequence
//=================================================
class lsu_unit_stride_seq extends base_sequence;
  `uvm_object_utils(lsu_unit_stride_seq)
  `uvm_add_to_seq_lib(lsu_unit_stride_seq,rvs_sequencer_sequence_library)

  alu_inst_e alu_inst;
  function new(string name = "lsu_unit_stride_seq");
    super.new(name);
	`ifdef UVM_POST_VERSION_1_1
     set_automatic_phase_objection(1);
    `endif
  endfunction:new

  virtual task body();
    for(int i=0; i<10; i++) begin
      req = new("req");
      start_item(req);
      assert(req.randomize() with {
        pc == inst_cnt;

        vtype.vma  ==  UNDISTURB;
        vtype.vta  ==  UNDISTURB;
        vtype.vsew ==  SEW8;
        vtype.vlmul == LMUL1;

        use_vlmax == 1;
        // vl == 10;

        inst_type == LD;
        lsu_mop   == LSU_E;
        lsu_umop  == NORMAL;
        lsu_nf    == NF1;
        lsu_eew   == EEW8;

        dest_type == VRF; dest_idx == 2;
        src1_type == VRF; src1_idx == 12;
        src2_type == VRF; src2_idx == 2;

        vm == 0; 
      });
      finish_item(req);
      inst_cnt++;
    end
  endtask
endclass: lsu_unit_stride_seq

`endif // RVS_SEQUENCER_SEQUENCE_LIBRARY__SV
