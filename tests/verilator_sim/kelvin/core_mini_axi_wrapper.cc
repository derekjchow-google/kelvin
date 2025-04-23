#include <bitset>
#include <future>
#include <iostream>
#include <memory>
#include <optional>
#include <queue>
#include <vector>
// #include <systemc>

#include "absl/types/span.h"

#include "VCoreMiniAxi.h"

class Clock {
 public:
  class Observer {
   public:
    Observer(Clock* clock)
      : clock_(clock) {
      clock_->AddObserver(this);
    }
    virtual ~Observer() {
      clock_->RemoveObserver(this);
    }

    virtual void OnRisingEdge() {}
    virtual void OnFallingEdge() {}
   protected:
    Clock& clock() { return *clock_; }
   private:
    Clock* const clock_;
  };

  template<typename Model>
  Clock(VerilatedContext* context, uint8_t* clock, Model* model)
    : context_(context),
      clock_(clock),
      eval_function_([model](){ model->eval(); }) {}
  ~Clock() = default;

  void Step() {
    context_->timeInc(1);
    (*clock_) = 1;
    Eval();
    for (auto& observer : observers_) {
      observer->OnRisingEdge();
      Eval();
    }

    context_->timeInc(1);
    (*clock_) = 0;
    Eval();
    for (auto& observer : observers_) {
      observer->OnFallingEdge();
      Eval();
    }
  }

  void Eval() {
    eval_function_();
  }

  void AddObserver(Observer* observer) {
    observers_.push_back(observer);
  }

  void RemoveObserver(Observer* observer) {
    auto it = std::find(observers_.begin(), observers_.end(), observer);
    if (it != observers_.end()) {
      observers_.erase(it);
    }
  }

 private:
  VerilatedContext* const context_;
  uint8_t* const clock_;
  std::function<void()> eval_function_;
  std::vector<Observer*> observers_;
};

struct AxiAddr {
  uint32_t addr_bits_addr;
  uint8_t addr_bits_prot;
  uint8_t addr_bits_id;
  uint8_t addr_bits_len;
  uint8_t addr_bits_size;
  uint8_t addr_bits_burst;
  uint8_t addr_bits_lock;
  uint8_t addr_bits_cache;
  uint8_t addr_bits_qos;
  uint8_t addr_bits_region;

  static AxiAddr FromIdAddrSize(int id, uint32_t addr, uint32_t byte_length) {
    uint32_t start_addr = addr;
    uint32_t end_addr = addr + byte_length - 1;
    uint32_t start_line = start_addr / 16;
    uint32_t end_line = end_addr / 16;
    uint32_t beats = (end_line - start_line) + 1;
    uint32_t size = std::ceil(std::log2(byte_length));
    size = std::min(size, 4u);
    AxiAddr axi_addr;
    axi_addr.addr_bits_addr = addr;
    axi_addr.addr_bits_prot = 0;
    axi_addr.addr_bits_id = id;
    axi_addr.addr_bits_len = beats - 1;
    axi_addr.addr_bits_size = size;
    axi_addr.addr_bits_burst = 1;  // INCR
    axi_addr.addr_bits_lock = 0;
    axi_addr.addr_bits_cache = 0;
    axi_addr.addr_bits_qos = 0;
    axi_addr.addr_bits_region = 0;
    return axi_addr;
  }
};

struct AxiWData {
  VlWide<4> write_data_bits_data;
  uint16_t write_data_bits_strb;
  uint8_t write_data_bits_last;
};

class AxiSlaveWriteDriver : Clock::Observer {
 public:
  AxiSlaveWriteDriver(Clock* clock,
                      uint8_t* write_addr_valid,
                      uint32_t* write_addr_bits_addr,
                      uint8_t* write_addr_bits_prot,
                      uint8_t* write_addr_bits_id,
                      uint8_t* write_addr_bits_len,
                      uint8_t* write_addr_bits_size,
                      uint8_t* write_addr_bits_burst,
                      uint8_t* write_addr_bits_lock,
                      uint8_t* write_addr_bits_cache,
                      uint8_t* write_addr_bits_qos,
                      uint8_t* write_addr_bits_region,
                      const uint8_t* write_addr_ready,
                      uint8_t* write_data_valid,
                      VlWide<4>* write_data_bits_data,
                      uint16_t* write_data_bits_strb,
                      uint8_t* write_data_bits_last,
                      const uint8_t* write_data_ready,
                      const uint8_t* write_resp_valid,
                      const uint8_t* write_resp_bits_id,
                      const uint8_t* write_resp_bits_resp,
                      uint8_t* write_resp_ready)
    : Clock::Observer(clock),
      write_addr_valid_(write_addr_valid),
      write_addr_bits_addr_(write_addr_bits_addr),
      write_addr_bits_prot_(write_addr_bits_prot),
      write_addr_bits_id_(write_addr_bits_id),
      write_addr_bits_len_(write_addr_bits_len),
      write_addr_bits_size_(write_addr_bits_size),
      write_addr_bits_burst_(write_addr_bits_burst),
      write_addr_bits_lock_(write_addr_bits_lock),
      write_addr_bits_cache_(write_addr_bits_cache),
      write_addr_bits_qos_(write_addr_bits_qos),
      write_addr_bits_region_(write_addr_bits_region),
      write_addr_ready_(write_addr_ready),
      write_data_valid_(write_data_valid),
      write_data_bits_data_(write_data_bits_data),
      write_data_bits_strb_(write_data_bits_strb),
      write_data_bits_last_(write_data_bits_last),
      write_data_ready_(write_data_ready),
      write_resp_valid_(write_resp_valid),
      write_resp_bits_id_(write_resp_bits_id),
      write_resp_bits_resp_(write_resp_bits_resp),
      write_resp_ready_(write_resp_ready) {
    // Always ready to accept response
    *write_resp_ready_ = 1;
  }

  // TODO(derekjchow): Test me
  std::shared_ptr<bool> WriteTransaction(
      int id, uint32_t addr, absl::Span<const uint8_t> data) {
    // Enqueue addr
    AxiAddr axi_addr = AxiAddr::FromIdAddrSize(id, addr, data.size());
    EnqueueAddr(axi_addr);

    // Enqueue data
    while (data.size() > 0) {
      uint32_t base_addr = (addr / 16) * 16;
      uint32_t sub_addr = addr - base_addr;
      uint32_t bytes_to_write = 16 - sub_addr;
      bytes_to_write = std::min(static_cast<uint32_t>(data.size()),
                                bytes_to_write);
      absl::Span<const uint8_t> local_data = data.subspan(0, bytes_to_write);

      AxiWData axi_data;
      uint8_t* data_ptr =
          reinterpret_cast<uint8_t*>(&(axi_data.write_data_bits_data[0])) + 
          sub_addr;
      memcpy(data_ptr, local_data.data(), bytes_to_write);
      axi_data.write_data_bits_strb = 0;
      for (uint32_t i = sub_addr; i < sub_addr + bytes_to_write; i++) {
        axi_data.write_data_bits_strb |= (1 << i);
      }
      axi_data.write_data_bits_last = (bytes_to_write == data.size());
      EnqueueData(axi_data);

      data.remove_prefix(bytes_to_write);
      addr += bytes_to_write;
    }

    const auto [it, success] = outstanding_transactions_.insert({
        id, std::make_shared<bool>(false)});
    return it->second;
  }

  std::shared_ptr<bool> WriteWord(int id, uint32_t addr, uint32_t word) {
    absl::Span<const uint8_t> data_span(
        reinterpret_cast<uint8_t*>(&word), sizeof(word));
    return WriteTransaction(id, addr, data_span);
  }

  // TODO(derekjchow): Make this private
  void EnqueueAddr(const AxiAddr& addr) {
    addr_queue_.push(addr);
  }

  // TODO(derekjchow): Make this private too
  void EnqueueData(const AxiWData& data) {
    data_queue_.push(data);
  }

 private:
  void OnRisingEdge() final {
    // std::cout << "Rising edge" << std::endl;
  }

  void OnFallingEdge() final {
    // std::cout << "Falling edge" << std::endl;

    // Send Addr
    *write_addr_valid_ = !addr_queue_.empty();
    clock().Eval();
    if (!addr_queue_.empty()) {
      *write_addr_bits_addr_ = addr_queue_.front().addr_bits_addr;
      *write_addr_bits_prot_ = addr_queue_.front().addr_bits_prot;
      *write_addr_bits_id_ = addr_queue_.front().addr_bits_id;
      *write_addr_bits_len_ = addr_queue_.front().addr_bits_len;
      *write_addr_bits_size_ = addr_queue_.front().addr_bits_size;
      *write_addr_bits_burst_ = addr_queue_.front().addr_bits_burst;
      *write_addr_bits_lock_ = addr_queue_.front().addr_bits_lock;
      *write_addr_bits_cache_ = addr_queue_.front().addr_bits_cache;
      *write_addr_bits_qos_ = addr_queue_.front().addr_bits_qos;
      *write_addr_bits_region_ = addr_queue_.front().addr_bits_region;
      if (*write_addr_ready_) {
        addr_queue_.pop();
      }
      clock().Eval();
    }

    // Send Data
    *write_data_valid_ = !data_queue_.empty();
    clock().Eval();
    if (!data_queue_.empty()) {

      // TODO(derekjchow): Debug write data
      *write_data_bits_data_ = data_queue_.front().write_data_bits_data;
      *write_data_bits_strb_ = data_queue_.front().write_data_bits_strb;
      *write_data_bits_last_ = data_queue_.front().write_data_bits_last;
      if (*write_data_ready_) {
        data_queue_.pop();
      }
      clock().Eval();
    }

    // Receive Response
    if (*write_resp_valid_) {
      auto it = outstanding_transactions_.find(*write_resp_bits_id_);
      if (it != outstanding_transactions_.end()) {
        *(it->second) = true;
        outstanding_transactions_.erase(it);
      }
    }
  }

  // Signals
  // WAddr
  uint8_t* const write_addr_valid_;
  uint32_t* const write_addr_bits_addr_;
  uint8_t* const write_addr_bits_prot_;
  uint8_t* const write_addr_bits_id_;
  uint8_t* const write_addr_bits_len_;
  uint8_t* const write_addr_bits_size_;
  uint8_t* const write_addr_bits_burst_;
  uint8_t* const write_addr_bits_lock_;
  uint8_t* const write_addr_bits_cache_;
  uint8_t* const write_addr_bits_qos_;
  uint8_t* const write_addr_bits_region_;
  const uint8_t* const write_addr_ready_;
  // WData
  uint8_t* const write_data_valid_;
  VlWide<4>* const write_data_bits_data_;
  uint16_t* const write_data_bits_strb_;
  uint8_t* const write_data_bits_last_;
  const uint8_t* const write_data_ready_;
  // WResp
  const uint8_t* const write_resp_valid_;
  const uint8_t* const write_resp_bits_id_;
  const uint8_t* const write_resp_bits_resp_;
  uint8_t* const write_resp_ready_;

  std::queue<AxiAddr> addr_queue_;
  std::queue<AxiWData> data_queue_;
  std::map<uint8_t/*id*/, std::shared_ptr<bool>> outstanding_transactions_;
};

class AxiSlaveReadDriver : Clock::Observer {
 public:
  struct Transaction {
    bool finished;
    uint32_t start_addr;
    uint32_t end_addr;
    std::vector<uint8_t> data;
  };
  AxiSlaveReadDriver(Clock* clock,
                     uint8_t* read_addr_valid,
                     uint32_t* read_addr_bits_addr,
                     uint8_t* read_addr_bits_prot,
                     uint8_t* read_addr_bits_id,
                     uint8_t* read_addr_bits_len,
                     uint8_t* read_addr_bits_size,
                     uint8_t* read_addr_bits_burst,
                     uint8_t* read_addr_bits_lock,
                     uint8_t* read_addr_bits_cache,
                     uint8_t* read_addr_bits_qos,
                     uint8_t* read_addr_bits_region,
                     const uint8_t* read_addr_ready,
                     const uint8_t* read_data_valid,
                     const VlWide<4>* read_data_bits_data,
                     const uint8_t* read_data_bits_id,
                     const uint8_t* read_data_bits_resp,
                     const uint8_t* read_data_bits_last,
                     uint8_t* read_data_ready
    )
    : Clock::Observer(clock),
      read_addr_valid_(read_addr_valid),
      read_addr_bits_addr_(read_addr_bits_addr),
      read_addr_bits_prot_(read_addr_bits_prot),
      read_addr_bits_id_(read_addr_bits_id),
      read_addr_bits_len_(read_addr_bits_len),
      read_addr_bits_size_(read_addr_bits_size),
      read_addr_bits_burst_(read_addr_bits_burst),
      read_addr_bits_lock_(read_addr_bits_lock),
      read_addr_bits_cache_(read_addr_bits_cache),
      read_addr_bits_qos_(read_addr_bits_qos),
      read_addr_bits_region_(read_addr_bits_region),
      read_addr_ready_(read_addr_ready),
      read_data_valid_(read_data_valid),
      read_data_bits_data(read_data_bits_data),
      read_data_bits_id_(read_data_bits_id),
      read_data_bits_resp_(read_data_bits_resp),
      read_data_bits_last_(read_data_bits_last),
      read_data_ready_(read_data_ready) 
      {
    (*read_data_ready_) = 1;
  }

  // TODO(derekjchow): Test me
  std::shared_ptr<Transaction> ReadTransaction(int id, uint32_t addr,
                                               uint32_t byte_length) {
    // Enqueue addr
    AxiAddr axi_addr = AxiAddr::FromIdAddrSize(id, addr, byte_length);
    addr_queue_.push(axi_addr);

    // Enqueue data
    const auto [it, success] = outstanding_transactions_.insert({
        id, std::make_shared<Transaction>()});
    it->second->finished = false;
    it->second->start_addr = addr;
    it->second->end_addr = addr + byte_length;
    it->second->data.reserve(byte_length);
    return it->second;
  }

 private:
  void OnFallingEdge() final {
    // Send Addr
    *read_addr_valid_ = !addr_queue_.empty();
    clock().Eval();
    if (!addr_queue_.empty()) {
      *read_addr_bits_addr_ = addr_queue_.front().addr_bits_addr;
      *read_addr_bits_prot_ = addr_queue_.front().addr_bits_prot;
      *read_addr_bits_id_ = addr_queue_.front().addr_bits_id;
      *read_addr_bits_len_ = addr_queue_.front().addr_bits_len;
      *read_addr_bits_size_ = addr_queue_.front().addr_bits_size;
      *read_addr_bits_burst_ = addr_queue_.front().addr_bits_burst;
      *read_addr_bits_lock_ = addr_queue_.front().addr_bits_lock;
      *read_addr_bits_cache_ = addr_queue_.front().addr_bits_cache;
      *read_addr_bits_qos_ = addr_queue_.front().addr_bits_qos;
      *read_addr_bits_region_ = addr_queue_.front().addr_bits_region;
      if (*read_addr_ready_) {
        addr_queue_.pop();
      }
      clock().Eval();
    }

    // Received data
    if (*read_data_valid_) {
      auto it = outstanding_transactions_.find(*read_data_bits_id_);
      if (it == outstanding_transactions_.end()) {
        return;
      }

      // TODO(derekjchow): Should probably handle INCR mode.
      uint32_t sub_addr = it->second->start_addr % 16;
      uint32_t bytes_to_read = 16 - sub_addr;
      bytes_to_read = std::min(
          bytes_to_read, it->second->end_addr - it->second->start_addr);
      const uint8_t* read_data = reinterpret_cast<const uint8_t*>(
          &(*read_data_bits_data)[0]);
      for (uint32_t i = 0; i < bytes_to_read; i++) {
        it->second->data.push_back(read_data[i + sub_addr]);
      }
      it->second->start_addr += bytes_to_read;
      if (*read_data_bits_last_) {
        it->second->finished = true;
        outstanding_transactions_.erase(it);
      }
    }
  }

  // Signals
  // RAddr
  uint8_t* const read_addr_valid_;
  uint32_t* const read_addr_bits_addr_;
  uint8_t* const read_addr_bits_prot_;
  uint8_t* const read_addr_bits_id_;
  uint8_t* const read_addr_bits_len_;
  uint8_t* const read_addr_bits_size_;
  uint8_t* const read_addr_bits_burst_;
  uint8_t* const read_addr_bits_lock_;
  uint8_t* const read_addr_bits_cache_;
  uint8_t* const read_addr_bits_qos_;
  uint8_t* const read_addr_bits_region_;
  const uint8_t* const read_addr_ready_;
//   // RData
  const uint8_t* const read_data_valid_;
  const VlWide<4>* const read_data_bits_data;
  const uint8_t* const read_data_bits_id_;
  const uint8_t* const read_data_bits_resp_;
  const uint8_t* const read_data_bits_last_;
  uint8_t* const read_data_ready_;

  std::queue<AxiAddr> addr_queue_;
  std::map<uint8_t/*id*/, std::shared_ptr<Transaction>>
      outstanding_transactions_;
};

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
                         &core_.io_axi_slave_read_data_ready) {}
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

  // TODO(derekjchow): Refine me
  std::vector<uint8_t> Read(uint32_t addr, uint32_t len) {
    auto transaction = slave_read_driver_.ReadTransaction(0, addr, len);
    while (!(transaction->finished)) {
      Step();
    }
    return transaction->data;
  }

  void WriteWord(uint32_t addr, uint32_t word) {
    std::shared_ptr<bool> transaction = slave_write_driver_.WriteWord(
        0, addr, word);
    while (!(*transaction)) {
      Step();
    }
  }

  std::vector<uint8_t> ReadWord(uint32_t addr) {
    auto transaction = slave_read_driver_.ReadTransaction(0, addr, 4);
    while (!(transaction->finished)) {
      Step();
    }
    return transaction->data;
  }

 private:
  VerilatedContext* const context_;
  VCoreMiniAxi core_;
  Clock clock_;
  AxiSlaveWriteDriver slave_write_driver_;
  AxiSlaveReadDriver slave_read_driver_;
};

int main() {
  // TODO(derekjchow): Read and write TCM
  const std::unique_ptr<VerilatedContext> context =
      std::make_unique<VerilatedContext>();
  CoreMiniAxiWrapper wrapper(context.get());
  wrapper.Reset();
  std::cout << "Done constructor" << std::endl;
  for (int i = 0; i < 5; i++) {
    wrapper.Step();
  }

//   {
//     // Execute a write
//     wrapper.WriteWord(0, 42);

//     // Execute a read
//     auto result = wrapper.ReadWord(0);
//     uint32_t word = *reinterpret_cast<const uint32_t*>(result.data());
//     std::cout << "Final result is: " << word << std::endl;
//   }

//   {
//     // Execute a write
//     wrapper.WriteWord(16, 9001);

//     // Execute a read
//     auto result = wrapper.ReadWord(16);
//     uint32_t word = *reinterpret_cast<const uint32_t*>(result.data());
//     std::cout << "Final result is: " << word << std::endl;
//   }

//   {
//     // Execute a write
//     wrapper.WriteWord(11, 99);

//     // Execute a read
//     auto result = wrapper.ReadWord(11);
//     uint32_t word = *reinterpret_cast<const uint32_t*>(result.data());
//     std::cout << "Final result is: " << word << std::endl;
//   }



  // Execute a write
  std::vector<uint8_t> data(16);
  for (size_t i = 0; i < data.size(); i++) {
    data[i] = i;
  }
  wrapper.Write(8, absl::Span<const uint8_t>(data.data(), data.size()));

  // Execute a read
  {
    auto result = wrapper.Read(0, 32);
    for (size_t i = 0; i < result.size(); i++) {
    std::cout << i << ": " << static_cast<uint32_t>(result.data()[i])
                << std::endl;
    }
  }

  return 0;
}