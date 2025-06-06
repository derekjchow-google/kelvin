# Copyright 2025 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

load("//rules:kelvin_v2.bzl", "kelvin_v2_binary")

cc_library(
    name = "hw_primitives",
    srcs = [
        "hw_primitives.cc",
    ],
    hdrs = [
        "hw_primitives.h",
    ],
    deps = [
        "//hdl/chisel/src/kelvin:core_mini_axi_cc_library_cc",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "core_mini_axi_wrapper",
    hdrs = [
        "core_mini_axi_wrapper.h",
        "mailbox.h",
    ],
    deps = [
        ":hw_primitives",
    ],
)

cc_binary(
    name = "core_mini_axi_wrapper_example",
    srcs = [
        "core_mini_axi_wrapper_example.cc",
    ],
    data = [
        "//tests/cocotb:wfi_slot_0.elf",
    ],
    deps = [
        ":core_mini_axi_wrapper",
        "//tests/verilator_sim:elf",
        "//hdl/chisel/src/kelvin:core_mini_axi_cc_library_cc",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
  name = "kelvin_simulator_headers",
  hdrs = [
    "kelvin_simulator.h",
    "mailbox.h",
  ],
)

cc_library(
  name = "core_mini_axi_simulator",
  srcs = ["core_mini_axi_simulator.cc"],
  deps = [
    ":core_mini_axi_wrapper",
    ":kelvin_simulator_headers",
  ],
)

cc_shared_library(
    name = "core_mini_axi_sim",
    deps = [
        ":core_mini_axi_simulator",
    ],
)

kelvin_v2_binary(
    name = "mailbox_example",
    srcs = [
        "mailbox_example.cc",
    ],
)

cc_binary(
    name = "core_mini_axi_simulator_example",
    srcs = [
        "core_mini_axi_simulator_example.cc",
    ],
    data = [
        ":mailbox_example.elf",
    ],
    deps = [
        ":core_mini_axi_simulator",
        "//tests/verilator_sim:elf",
        "//hdl/chisel/src/kelvin:core_mini_axi_cc_library_cc",
        "@com_google_absl//absl/types:span",
    ],
)
