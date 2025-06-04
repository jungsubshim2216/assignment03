module control (
    output wire       RegWEn,
    output wire       PCSel,
    output wire       ASel,
    output wire       BSel,
    output wire [1:0] WBSel,
    output wire       MemRW,
    output wire [2:0] ImmSel,
    output wire [3:0] ALUSel,
    output wire       BrU,

    input wire [31:0] instruction,
    input wire        BrEq,
    input wire        BrLt
);

    wire [6:0] opcode = instruction[6:0];
    wire [2:0] funct3 = instruction[14:12];
    wire       funct7_5 = instruction[30];

    localparam IMM_I = 3'd0,
               IMM_B = 3'd1,
               IMM_U = 3'd2,
               IMM_J = 3'd3,
               IMM_S = 3'd4;

    assign ImmSel = (opcode==7'b0010011 || opcode==7'b0000011 || opcode==7'b1100111) ? IMM_I :      // I-type
                    (opcode==7'b0100011) ? IMM_S :                                                 // S
                    (opcode==7'b1100011) ? IMM_B :                                                 // B
                    (opcode==7'b0110111 || opcode==7'b0010111) ? IMM_U :                           // LUI, AUIPC
                    IMM_J;                                                                         // JAL

    assign ASel = (opcode==7'b1100011 || opcode==7'b1101111 || opcode==7'b0010111) ? 1'b1 : 1'b0; // pc vs RD1
    assign BSel = (opcode==7'b1100011) ? 1'b1 : 
                  (opcode==7'b0110011) ? 1'b0 :
                  1'b1;

    wire r_type = (opcode==7'b0110011);
    wire i_type = (opcode==7'b0010011);
    assign ALUSel = r_type                 ? {funct7_5, funct3} :
                    (i_type && (funct3==3'b101)) ? {instruction[30], funct3} : /* srli/srai */
                    (i_type && (funct3==3'b001)) ? {1'b0        , funct3} :    /* slli       */
                    (i_type)                     ? {1'b0        , funct3} :    /* addi,..    */
                    4'b0000; 

    assign BrU = (funct3==3'b110 || funct3==3'b111); // bltu,bgeu

    wire take_branch = (opcode==7'b1100011) && (
                        /* beq  */ (funct3==3'b000 && BrEq            ) ||
                        /* bne  */ (funct3==3'b001 && !BrEq           ) ||
                        /* blt  */ (funct3==3'b100 && BrLt            ) ||
                        /* bge  */ (funct3==3'b101 && !BrLt           ) ||
                        /* bltu */ (funct3==3'b110 && BrLt            ) ||
                        /* bgeu */ (funct3==3'b111 && !BrLt           ) );

    wire jal  = (opcode==7'b1101111);
    wire jalr = (opcode==7'b1100111);

    assign PCSel = take_branch | jal | jalr;

    assign RegWEn = ~(opcode==7'b0100011 || opcode==7'b1100011); // store, branch     

    /* WBSel : 00=ALU, 01=DMEM, 10=PC+4                                     */
    assign WBSel = (opcode==7'b0000011) ? 2'b01 :  // Load    DMEM
                   (jal | jalr        ) ? 2'b10 :  // Jump     PC+4
                   2'b00;                          //           ALU

    assign MemRW = (opcode==7'b0100011); // sw

endmodule