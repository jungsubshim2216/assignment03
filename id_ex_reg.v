module id_ex_reg (
    input  wire clk, rst, flush,
    input  wire RegWrite_in, MemRW_in, MemRead_in, PCSel_in, ASel_in, BSel_in, BrU_in,
    input  wire [1:0] WBSel_in,
    input  wire [3:0] ALUSel_in,
    input  wire [7:0] pc_in, pc4_in,
    input  wire [31:0] rd1_in, rd2_in, imm_in,
    input  wire [4:0] rs1_in, rs2_in, rd_in,
    input  wire [6:0] opcode_in,
    output reg  RegWrite_out, MemRW_out, MemRead_out, PCSel_out, ASel_out, BSel_out, BrU_out,
    output reg  [1:0] WBSel_out,
    output reg  [3:0] ALUSel_out,
    output reg  [7:0] pc_out, pc4_out,
    output reg  [31:0] rd1_out, rd2_out, imm_out,
    output reg  [4:0] rs1_out, rs2_out, rd_out,
    output reg  [6:0] opcode_out
);
    always @(posedge clk or posedge rst) begin
        if (rst || flush) begin
            RegWrite_out <= 1'b0; MemRW_out <= 1'b0; MemRead_out <= 1'b0; PCSel_out <= 1'b0;
            ASel_out <= 1'b0; BSel_out <= 1'b0; BrU_out <= 1'b0;
            WBSel_out <= 2'b00;
            ALUSel_out <= 4'b0000;
            pc_out <= 8'b0; pc4_out <= 8'b0;
            rd1_out <= 32'b0; rd2_out <= 32'b0; imm_out <= 32'b0;
            rs1_out <= 5'b0; rs2_out <= 5'b0; rd_out <= 5'b0;
            opcode_out <= 7'h13;
        end else begin
            RegWrite_out <= RegWrite_in; MemRW_out <= MemRW_in; MemRead_out <= MemRead_in; PCSel_out <= PCSel_in;
            ASel_out <= ASel_in; BSel_out <= BSel_in; BrU_out <= BrU_in;
            WBSel_out <= WBSel_in; ALUSel_out <= ALUSel_in;
            pc_out <= pc_in; pc4_out <= pc4_in;
            rd1_out <= rd1_in; rd2_out <= rd2_in; imm_out <= imm_in;
            rs1_out <= rs1_in; rs2_out <= rs2_in; rd_out <= rd_in;
            opcode_out <= opcode_in;
        end
    end
endmodule