//────────────────────────────────────────────────────────────
// Immediate Generator  - RV32I 5-type (I,S,B,U,J)  ★ FINAL
//────────────────────────────────────────────────────────────
module ImmGen (
    input  wire [31:0] inst,      // 전체 32-bit 명령어
    input  wire [2:0]  ImmSel,    // 0:I 1:B 2:U 3:J 4:S
    output reg  [31:0] Imm
);
    always @(*) begin
        case (ImmSel)
            3'd0:  Imm = {{21{inst[31]}}, inst[30:20]};                          // I
            3'd4:  Imm = {{21{inst[31]}}, inst[30:25], inst[11:7]};              // S
            3'd1:  Imm = {{20{inst[31]}}, inst[7], inst[30:25],                  // B <<1
                                            inst[11:8], 1'b0};
            3'd2:  Imm = {inst[31:12], 12'd0};                                   // U
            3'd3:  Imm = {{12{inst[31]}}, inst[19:12], inst[20],                 // J <<1
                                            inst[30:21], 1'b0};
            default: Imm = 32'd0;
        endcase
    end
endmodule