module ex_mem_reg (
    input  wire        clk,
    input  wire        rst,
    input  wire        flush,
    input  wire        RegWrite_in,
    input  wire        MemRW_in,
    input  wire [1:0]  WBSel_in,
    input  wire        PCSel_in,
    input  wire        take_jalr_in,
    input  wire [7:0]  jalr_target_in,
    input  wire [31:0] alu_in,
    input  wire [31:0] rd2_in,
    input  wire [7:0]  pc4_in,
    input  wire [4:0]  rd_in,
    // ─────────── 새로 추가 ───────────
    input  wire [7:0]  pc_in,      // ← EX 단계의 PC를 이곳으로 전달

    output reg         RegWrite_out,
    output reg         MemRW_out,
    output reg  [1:0]  WBSel_out,
    output reg         PCSel_out,
    output reg         take_jalr_out,
    output reg  [7:0]  jalr_target_out,
    output reg  [31:0] alu_out,
    output reg  [31:0] rd2_out,
    output reg  [7:0]  pc4_out,
    output reg  [4:0]  rd_out,
    // ─────────── 새로 추가 ───────────
    output reg  [7:0]  pc_out       // ← MEM 단계에서 원래의 PC를 볼 수 있도록
);
    always @(posedge clk or posedge rst) begin
        if (rst || flush) begin
            RegWrite_out    <= 1'b0;
            MemRW_out       <= 1'b0;
            WBSel_out       <= 2'b00;
            PCSel_out       <= 1'b0;
            take_jalr_out   <= 1'b0;
            jalr_target_out <= 8'b0;
            alu_out         <= 32'b0;
            rd2_out         <= 32'b0;
            pc4_out         <= 8'b0;
            rd_out          <= 5'b0;
            // 새로 추가한 레지스터도 리셋
            pc_out          <= 8'b0;
        end else begin
            RegWrite_out    <= RegWrite_in;
            MemRW_out       <= MemRW_in;
            WBSel_out       <= WBSel_in;
            PCSel_out       <= PCSel_in;
            take_jalr_out   <= take_jalr_in;
            jalr_target_out <= jalr_target_in;
            alu_out         <= alu_in;
            rd2_out         <= rd2_in;
            pc4_out         <= pc4_in;
            rd_out          <= rd_in;
            // 여기에 pc_in 값을 그대로 전달
            pc_out          <= pc_in;
        end
    end
endmodule
