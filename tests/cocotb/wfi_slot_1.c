int main(int argc, char** argv) {
    int ret_val;
    asm volatile("nop; wfi");
    ret_val = *(&argc);
    return ret_val;
}
