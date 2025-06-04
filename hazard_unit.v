//────────────────────────────────────────────────────────────
// Hazard Detection Unit - load-use 1-cycle stall
//────────────────────────────────────────────────────────────
module hazard_unit (
    input  wire [4:0] IF_ID_rs1, IF_ID_rs2,
    input  wire [4:0] ID_EX_rd,
    input  wire       ID_EX_MemRead,
    output wire       stall              // ← bubble 신호 제거
);
    assign stall =
        ID_EX_MemRead &&
        ((ID_EX_rd == IF_ID_rs1) || (ID_EX_rd == IF_ID_rs2));
endmodule
