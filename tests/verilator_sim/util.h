#ifndef TESTS_VERILATOR_SIM_UTIL_H_
#define TESTS_VERILATOR_SIM_UTIL_H_

#define REPEAT_1(FN) FN(0)
#define REPEAT_2(FN) REPEAT_1(FN) FN(1)
#define REPEAT_3(FN) REPEAT_2(FN) FN(2)
#define REPEAT_4(FN) REPEAT_3(FN) FN(3)
#define REPEAT_5(FN) REPEAT_4(FN) FN(4)
#define REPEAT_6(FN) REPEAT_5(FN) FN(5)
#define REPEAT_7(FN) REPEAT_6(FN) FN(6)
#define REPEAT_8(FN) REPEAT_7(FN) FN(7)
#define REPEAT(FN, N) REPEAT_(FN, N)
#define REPEAT_(FN, N) REPEAT_##N(FN)

#define STRINGIFY(x) STRINGIFY_(x)
#define STRINGIFY_(x) #x

#endif  // TESTS_VERILATOR_SIM_UTIL_H_