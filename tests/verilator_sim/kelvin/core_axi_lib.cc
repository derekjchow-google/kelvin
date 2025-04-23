// Copyright 2025 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <systemc.h>
#include <vector>

namespace kelvin_hw_sim {

// A class wrapper for a SystemC clock.
class Clock {
 public:
  class ClockObserver {
   public:
    virtual void OnRisingEdge() {};
    virtual void OnFallingEdge() {};
  };

  void Step() {
    clock_->posedge();
    for (auto& observer : observers_) {
      observer->OnRisingEdge();
    }
    clock_->negedge();
    for (auto& observer : observers_) {
      observer->OnFallingEdge();
    }
  }

 private:
  sc_in_clk clock_;

  std::vector<ClockObserver*> observers_;
};

class AxiSlaveDriver {
 public:
 private:
  core_->io_axi_slave_read_addr_ready(tlm2axi_signals_.arready);
  core_->io_axi_slave_read_addr_valid(tlm2axi_signals_.arvalid);
  core_->io_axi_slave_read_addr_bits_addr(tlm2axi_signals_.araddr);
  core_->io_axi_slave_read_addr_bits_prot(tlm2axi_signals_.arprot);
  core_->io_axi_slave_read_addr_bits_id(tlm2axi_signals_.arid);
  core_->io_axi_slave_read_addr_bits_len(tlm2axi_signals_.arlen);
  core_->io_axi_slave_read_addr_bits_size(tlm2axi_signals_.arsize);
  core_->io_axi_slave_read_addr_bits_burst(tlm2axi_signals_.arburst);
  core_->io_axi_slave_read_addr_bits_lock(tlm2axi_signals_.arlock);
  core_->io_axi_slave_read_addr_bits_cache(tlm2axi_signals_.arcache);
  core_->io_axi_slave_read_addr_bits_qos(tlm2axi_signals_.arqos);
  core_->io_axi_slave_read_addr_bits_region(tlm2axi_signals_.arregion);
  // R
  core_->io_axi_slave_read_data_ready(tlm2axi_signals_.rready);
  core_->io_axi_slave_read_data_valid(tlm2axi_signals_.rvalid);
  core_->io_axi_slave_read_data_bits_data(tlm2axi_signals_.rdata);
  core_->io_axi_slave_read_data_bits_id(tlm2axi_signals_.rid);
  core_->io_axi_slave_read_data_bits_resp(tlm2axi_signals_.rresp);
  core_->io_axi_slave_read_data_bits_last(tlm2axi_signals_.rlast);
  // AW
  core_->io_axi_slave_write_addr_ready(tlm2axi_signals_.awready);
  core_->io_axi_slave_write_addr_valid(tlm2axi_signals_.awvalid);
  core_->io_axi_slave_write_addr_bits_addr(tlm2axi_signals_.awaddr);
  core_->io_axi_slave_write_addr_bits_prot(tlm2axi_signals_.awprot);
  core_->io_axi_slave_write_addr_bits_id(tlm2axi_signals_.awid);
  core_->io_axi_slave_write_addr_bits_len(tlm2axi_signals_.awlen);
  core_->io_axi_slave_write_addr_bits_size(tlm2axi_signals_.awsize);
  core_->io_axi_slave_write_addr_bits_burst(tlm2axi_signals_.awburst);
  core_->io_axi_slave_write_addr_bits_lock(tlm2axi_signals_.awlock);
  core_->io_axi_slave_write_addr_bits_cache(tlm2axi_signals_.awcache);
  core_->io_axi_slave_write_addr_bits_qos(tlm2axi_signals_.awqos);
  core_->io_axi_slave_write_addr_bits_region(tlm2axi_signals_.awregion);
  // W
  core_->io_axi_slave_write_data_ready(tlm2axi_signals_.wready);
  core_->io_axi_slave_write_data_valid(tlm2axi_signals_.wvalid);
  core_->io_axi_slave_write_data_bits_data(tlm2axi_signals_.wdata);
  core_->io_axi_slave_write_data_bits_last(tlm2axi_signals_.wlast);
  core_->io_axi_slave_write_data_bits_strb(tlm2axi_signals_.wstrb);
  // B
  core_->io_axi_slave_write_resp_ready(tlm2axi_signals_.bready);
  core_->io_axi_slave_write_resp_valid(tlm2axi_signals_.bvalid);
  core_->io_axi_slave_write_resp_bits_id(tlm2axi_signals_.bid);
  core_->io_axi_slave_write_resp_bits_resp(tlm2axi_signals_.bresp);
};

}  // namespace kelvin_hw_sim