/* DO NOT MODIFY THIS MODULE */
`timescale 1ns/100ps

module tb();

reg clk, rst, pc_en;

initial begin
    rst = 1;
    pc_en = 0;
    clk = 1;
    #4
    rst = 0;
    pc_en = 1;

    #1000

    #3    $finish;
end

SC_RISCV_32I RISCV(
   clk, rst, pc_en
);

always #2 clk <= ~clk;

endmodule


