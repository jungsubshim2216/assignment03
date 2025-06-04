module COMP32b(
    output Less,
    input [31:0] A, B,
    input uMod
);

assign Less = uMod ? (A<B) : ($signed(A) < $signed(B));

endmodule