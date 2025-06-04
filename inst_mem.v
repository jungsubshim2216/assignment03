/* MODIFY THIS MODULE ACCORDING TO THE INSTRUCTION SET */
module inst_mem(
    output [31:0] inst,
    input  [7:0] PC
);

reg [31:0] I_Memory [0:31];

wire [5:0] mem_offset;
assign mem_offset = PC[7:2];

assign inst = I_Memory[mem_offset];

initial begin
/* write clear1 instruction sequence from I_Memory[0] to I_Memory[7] */
//  I_Memory[0] = 32'h00000793;  // li a5, 0 (addi a5, x0, 0)
//  I_Memory[1] = 32'h0140006f;  // j 18 <.L2>
//  I_Memory[2] = 32'h00279713;  // slli a4, a5, 0x2
//  I_Memory[3] = 32'h00e50733;  // add a4, a0, a4
//  I_Memory[4] = 32'h00072023;  // sw zero, 0(a4)
//  I_Memory[5] = 32'h00178793;  // addi a5, a5, 1
//  I_Memory[6] = 32'hfeb7c8e3;  // blt a5, a1, 8 <.L3>
//  I_Memory[7] = 32'h00008067;  // ret
  
/* write clear2 instruction sequence from I_Memory[8] to I_Memory[14] */
  I_Memory[8] = 32'h00259593;  // slli a1, a1, 0x2
  I_Memory[9] = 32'h00b505b3;  // add a1, a0, a1
  I_Memory[10] = 32'h00c0006f; // j 34 <.L5>
  I_Memory[11] = 32'h00052023; // sw zero, 0(a0)
  I_Memory[12] = 32'h00450513; // addi a0, a0, 4
  I_Memory[13] = 32'hfeb56ce3; // bltu a0, a1, 2c <.L6>
  I_Memory[14] = 32'h00008067; // ret
  
end 

endmodule