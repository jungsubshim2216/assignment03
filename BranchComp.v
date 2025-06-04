//────────────────────────────────────────────────────────────
// Branch Comparator - signed / unsigned   ★ FINAL
//────────────────────────────────────────────────────────────
module BranchComp (
    input  wire [31:0] A,
    input  wire [31:0] B,
    input  wire        BrU,   // 0: signed, 1: unsigned
    output wire        BrEq,
    output wire        BrLt
);
    assign BrEq = (A == B);

    // BrU=1 → unsigned 비교, BrU=0 → signed 비교
    assign BrLt = BrU ? (A < B) : ($signed(A) < $signed(B));
endmodule