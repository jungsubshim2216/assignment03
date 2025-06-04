//------------------------------------------------------------------------------
// Static Branch Predictor (not-taken)
//------------------------------------------------------------------------------
module branch_predictor (
    input  wire        pc_write_en,
    input  wire [7:0]  PC_curr,
    output wire [7:0]  PC_pred
);
    // �׻� PC+4 ��ȯ - not taken
    assign PC_pred = PC_curr + 8'd4;
endmodule