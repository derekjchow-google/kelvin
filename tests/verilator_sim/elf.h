// Copyright 2024 Google LLC
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

#ifndef TESTS_VERILATOR_SIM_UTIL_H_
#define TESTS_VERILATOR_SIM_UTIL_H_

#include <cstdint>
#include <functional>
#include <string>

typedef std::function<void*(void* /* dest */, const void* /* src */,
                            size_t /* count */)>
    CopyFn;
uint32_t LoadElf(uint8_t* data, CopyFn copy_fn);
bool LookupSymbol(const uint8_t* data, const std::string& symbol_name, uint32_t* symbol_addr);

#endif  // TESTS_VERILATOR_SIM_UTIL_H_
