//------------------------------------------------------------------------------
// Forwarding Unit
//------------------------------------------------------------------------------
module forwarding_unit (
    input  wire [4:0] ID_EX_rs1,
    input  wire [4:0] ID_EX_rs2,
    input  wire [4:0] EX_MEM_rd,
    input  wire        EX_MEM_RegWrite,
    input  wire [4:0] MEM_WB_rd,
    input  wire        MEM_WB_RegWrite,
    output reg  [1:0]  fwdA,
    output reg  [1:0]  fwdB
);
    always @(*) begin
        // A
        if (EX_MEM_RegWrite && (EX_MEM_rd != 5'd0) && (EX_MEM_rd == ID_EX_rs1))
            fwdA = 2'b10; // EX stage result
        else if (MEM_WB_RegWrite && (MEM_WB_rd != 5'd0) && (MEM_WB_rd == ID_EX_rs1))
            fwdA = 2'b01; // MEM/WB result
        else
            fwdA = 2'b00; // no forwarding

        // B
        if (EX_MEM_RegWrite && (EX_MEM_rd != 5'd0) && (EX_MEM_rd == ID_EX_rs2))
            fwdB = 2'b10;
        else if (MEM_WB_RegWrite && (MEM_WB_rd != 5'd0) && (MEM_WB_rd == ID_EX_rs2))
            fwdB = 2'b01;
        else
            fwdB = 2'b00;
    end
endmodule