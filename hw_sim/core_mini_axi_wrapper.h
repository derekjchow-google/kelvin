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

#ifndef HW_SIM_HW_CORE_MINI_AXI_WRAPPER_H_
#define HW_SIM_HW_CORE_MINI_AXI_WRAPPER_H_

#include "hw_sim/hw_primitives.h"

class CoreMiniAxiWrapper {
 public:
  explicit CoreMiniAxiWrapper(VerilatedContext* context)
    : context_(context),
      core_(context, "core"),
      clock_(context, &core_.io_aclk, &core_),
      slave_write_driver_(&clock_,
                          &core_.io_axi_slave_write_addr_valid,
                          &core_.io_axi_slave_write_addr_bits_addr,
                          &core_.io_axi_slave_write_addr_bits_prot,
                          &core_.io_axi_slave_write_addr_bits_id,
                          &core_.io_axi_slave_write_addr_bits_len,
                          &core_.io_axi_slave_write_addr_bits_size,
                          &core_.io_axi_slave_write_addr_bits_burst,
                          &core_.io_axi_slave_write_addr_bits_lock,
                          &core_.io_axi_slave_write_addr_bits_cache,
                          &core_.io_axi_slave_write_addr_bits_qos,
                          &core_.io_axi_slave_write_addr_bits_region,
                          &core_.io_axi_slave_write_addr_ready,
                          &core_.io_axi_slave_write_data_valid,
                          &core_.io_axi_slave_write_data_bits_data,
                          &core_.io_axi_slave_write_data_bits_strb,
                          &core_.io_axi_slave_write_data_bits_last,
                          &core_.io_axi_slave_write_data_ready,
                          &core_.io_axi_slave_write_resp_valid,
                          &core_.io_axi_slave_write_resp_bits_id,
                          &core_.io_axi_slave_write_resp_bits_resp,
                          &core_.io_axi_slave_write_resp_ready),
      slave_read_driver_(&clock_,
                         &core_.io_axi_slave_read_addr_valid,
                         &core_.io_axi_slave_read_addr_bits_addr,
                         &core_.io_axi_slave_read_addr_bits_prot,
                         &core_.io_axi_slave_read_addr_bits_id,
                         &core_.io_axi_slave_read_addr_bits_len,
                         &core_.io_axi_slave_read_addr_bits_size,
                         &core_.io_axi_slave_read_addr_bits_burst,
                         &core_.io_axi_slave_read_addr_bits_lock,
                         &core_.io_axi_slave_read_addr_bits_cache,
                         &core_.io_axi_slave_read_addr_bits_qos,
                         &core_.io_axi_slave_read_addr_bits_region,
                         &core_.io_axi_slave_read_addr_ready,
                         &core_.io_axi_slave_read_data_valid,
                         &core_.io_axi_slave_read_data_bits_data,
                         &core_.io_axi_slave_read_data_bits_id,
                         &core_.io_axi_slave_read_data_bits_resp,
                         &core_.io_axi_slave_read_data_bits_last,
                         &core_.io_axi_slave_read_data_ready),
        halted_(&core_.io_halted),
        wfi_(&core_.io_wfi) {}
  ~CoreMiniAxiWrapper() = default;

  void Reset() {
    core_.io_aresetn = 1;
    context_->timeInc(1);
    core_.io_aresetn = 0;
    context_->timeInc(1);
    core_.io_aresetn = 1;
    context_->timeInc(1);
  }

  void Step() {
    clock_.Step();
  }

  void Write(uint32_t addr, absl::Span<const uint8_t> data) {
    while (data.size() > 0) {
      uint32_t offset4096 = addr % 4096;
      uint32_t remainder4096 = 4096 - offset4096;
      uint32_t max_transaction_bytes = std::min(4096u, remainder4096);
      uint32_t transaction_bytes = std::min(
          static_cast<uint32_t>(data.size()), max_transaction_bytes);
      absl::Span<const uint8_t> local_data = data.subspan(0, transaction_bytes);

      std::shared_ptr<bool> transaction = slave_write_driver_.WriteTransaction(
          0, addr, local_data);
      while (!(*transaction)) {
        Step();
      }

      data.remove_prefix(transaction_bytes);
      addr += transaction_bytes;
    }
  }

  std::vector<uint8_t> Read(uint32_t addr, uint32_t len) {
    std::vector<uint8_t> result;
    result.reserve(len);
    while (result.size() < len) {
      uint32_t offset4096 = addr % 4096;
      uint32_t remainder4096 = 4096 - offset4096;
      uint32_t max_transaction_bytes = std::min(4096u, remainder4096);
      uint32_t bytes_remaining = len - result.size();
      uint32_t transaction_bytes = std::min(
          static_cast<uint32_t>(bytes_remaining), max_transaction_bytes);

      auto transaction = slave_read_driver_.ReadTransaction(
          0, addr, transaction_bytes);
      while (!(transaction->finished)) {
        Step();
      }
      std::copy(transaction->data.begin(), transaction->data.end(),
              std::back_inserter(result));

      addr += transaction_bytes;
    }
    return result;
  }

  void WriteWord(uint32_t addr, uint32_t word) {
    absl::Span<const uint8_t> data_span(
        reinterpret_cast<uint8_t*>(&word), sizeof(word));
    std::shared_ptr<bool> transaction = slave_write_driver_.WriteTransaction(
        0, addr, data_span);
    while (!(*transaction)) {
      Step();
    }
  }

  bool WaitForTermination(int timeout=10000) {
    for (int i = 0; i < timeout; i++) {
      if ((*halted_) || (*wfi_)) {
        return true;
      }
      Step();
    }
    return false;
  }

 private:
  VerilatedContext* const context_;
  VCoreMiniAxi core_;
  Clock clock_;
  AxiSlaveWriteDriver slave_write_driver_;
  AxiSlaveReadDriver slave_read_driver_;
  const uint8_t* const halted_;
  const uint8_t* const wfi_;
};

#endif  // HW_SIM_HW_CORE_MINI_AXI_WRAPPER_H_