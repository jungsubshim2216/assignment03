module if_id_reg (
    input  wire clk, rst, stall, flush,
    input  wire [31:0] inst_in,
    input  wire [7:0]  pc_in, pc4_in,
    output reg  [31:0] inst_out,
    output reg  [7:0]  pc_out, pc4_out
);
    always @(posedge clk or posedge rst) begin
        if (rst || flush) begin
            inst_out <= 32'h00000013; // addi x0, x0, 0 (NOP)
            pc_out   <= 8'b0;
            pc4_out  <= 8'b0;
        end else if (!stall) begin
            inst_out <= inst_in;
            pc_out   <= pc_in;
            pc4_out  <= pc4_in;
        end
    end
endmodule