module mem_wb_reg (
    input  wire clk, rst,
    input  wire RegWrite_in,
    input  wire [1:0] WBSel_in,
    input  wire [31:0] alu_in, dmem_in,
    input  wire [7:0]  pc4_in,
    input  wire [4:0]  rd_in,
    output reg  RegWrite_out,
    output reg  [1:0] WBSel_out,
    output reg  [31:0] alu_out, dmem_out,
    output reg  [7:0]  pc4_out,
    output reg  [4:0]  rd_out
);
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            RegWrite_out <= 1'b0; WBSel_out <= 2'b0;
            alu_out <= 32'b0; dmem_out <= 32'b0;
            pc4_out <= 8'b0; rd_out <= 5'b0;
        end else begin
            RegWrite_out <= RegWrite_in; WBSel_out <= WBSel_in;
            alu_out <= alu_in; dmem_out <= dmem_in;
            pc4_out <= pc4_in; rd_out <= rd_in;
        end
    end
endmodule