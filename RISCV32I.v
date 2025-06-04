//==============================================================================
// Top-level 5-stage pipelined RV32I core (modified for EX-stage branch logic)
//==============================================================================
module SC_RISCV_32I (
    input  wire clk,
    input  wire rst,
    input  wire pc_en
);
    //--------------------------------------------------------------------------
    // 0) Common Signal Declarations
    //--------------------------------------------------------------------------
    // IF Stage
    reg  [7:0]  PC;
    wire [7:0]  PC_plus4_IF;
    wire [7:0]  PC_pred_IF;
    wire [31:0] inst_IF;

    // IF/ID Register
    wire        stall_IFID, flush_IFID;
    wire [7:0]  PC_ID, PC4_ID;
    wire [31:0] inst_ID;

    // ID Stage
    wire        RegWrite_ID_ctrl, MemRW_ID_ctrl;
    wire        ASel_ID_ctrl, BSel_ID_ctrl, BrU_ID_ctrl; // BrU from Control Unit
    wire [1:0]  WBSel_ID_ctrl;
    wire [2:0]  ImmSel_ID_ctrl;
    wire [3:0]  ALUSel_ID_ctrl;
    wire        PCSel_ID_ctrl; // Control signal indicating potential control flow change
    wire [31:0] RD1_ID_raw, RD2_ID_raw, Imm_ID;
    // wire [31:0] fwd_RD1_ID, fwd_RD2_ID; // ID-stage forwarding, less critical for ID/EX inputs now
    // wire        BrEq_ID, BrLt_ID; // Moved to EX
    wire [6:0]  opcode_ID;
    wire        load_stall_ID;

    // ID/EX Register
    wire        RegWrite_EX, MemRW_EX, ID_EX_MemRead_out;
    wire        ASel_EX, BSel_EX, BrU_EX; // BrU latched from ID
    wire [1:0]  WBSel_EX;
    wire [3:0]  ALUSel_EX;
    wire        PCSel_EX;   // PCSel_ID_ctrl signal latched in EX
    wire [7:0]  PC_EX, PC4_EX;
    wire [31:0] RD1_EX, RD2_EX, Imm_EX; // RD1_EX, RD2_EX will be raw values from RF
    wire [4:0]  rs1_EX, rs2_EX, rd_EX;
    wire [6:0]  opcode_EX;

    // EX Stage
    wire [1:0]  fwdA_EX_sel, fwdB_EX_sel;
    wire [31:0] srcA_EX, srcB_EX, ALU_result_EX;
    wire        take_jalr_EX_logic; // Renamed from take_jalr_EX for clarity
    wire [7:0]  jalr_target_EX_addr;
    wire        BrEq_EX, BrLt_EX; // NEW: Branch comparison results in EX
    wire [7:0]  branch_jal_target_EX_addr; // NEW: Target for branch/JAL calculated in EX
    wire        take_branch_EX_logic;    // NEW: Branch actually taken in EX
    wire        take_jal_EX_logic;       // NEW: JAL actually taken in EX

    // EX/MEM Register
    wire        RegWrite_MEM, MemRW_MEM;
    wire [1:0]  WBSel_MEM;
    // wire        PCSel_MEM; // Replaced by more specific take_* signals
    wire        take_jalr_MEM_logic;
    wire [7:0]  jalr_target_MEM_addr;
    wire        take_branch_MEM_logic;   // NEW
    wire        take_jal_MEM_logic;      // NEW
    wire [7:0]  branch_jal_target_MEM_addr; // NEW
    wire [7:0]  PC4_MEM;
    wire [31:0] ALU_out_MEM, RD2_MEM_value;
    wire [4:0]  rd_MEM;
    wire [7:0]  PC_MEM;
    
    // MEM Stage
    wire [31:0] DMEM_out_MEM;

    // MEM/WB Register
    wire        RegWrite_WB;
    wire [1:0]  WBSel_WB;
    wire [7:0]  PC4_WB;
    wire [31:0] ALU_WB, DMEM_WB;
    wire [4:0]  rd_WB;
    wire [31:0] WB_value_WB;

    //--------------------------------------------------------------------------
    // 1) IF Stage : instruction fetch
    //--------------------------------------------------------------------------
    inst_mem IMEM (.inst(inst_IF), .PC(PC));
    assign PC_plus4_IF = PC + 8'd4;
    assign PC_pred_IF  = PC_plus4_IF;

    //--------------------------------------------------------------------------
    // 2) IF/ID Pipeline Register
    //--------------------------------------------------------------------------
    if_id_reg IF_ID (
        .clk(clk), .rst(rst),
        .stall(stall_IFID), .flush(flush_IFID),
        .inst_in(inst_IF),  .pc_in(PC), .pc4_in(PC_plus4_IF),
        .inst_out(inst_ID), .pc_out(PC_ID), .pc4_out(PC4_ID)
    );

    //--------------------------------------------------------------------------
    // 3) ID Stage : Decode / Control / Register File Read
    //--------------------------------------------------------------------------
    assign opcode_ID = inst_ID[6:0];

    // ID-Stage Forwarding Logic (original fwd_RD1_ID, fwd_RD2_ID for completeness, though not directly feeding ID/EX inputs now)
    // These could be useful if some other ID-stage component needed forwarded values.
    // For strict diagram adherence where only EX has forwarding muxes for ALU, these might be simplified/removed
    // if not used by any other ID-stage logic after moving BranchComp.
    /*
    assign fwd_RD1_ID =
            (RegWrite_EX && (rd_EX != 5'd0) && (rd_EX == inst_ID[19:15]) && ID_EX_MemRead_out == 1'b0) ? ALU_result_EX : // EX->ID (non-load)
            (RegWrite_MEM && (rd_MEM != 5'd0) && (rd_MEM == inst_ID[19:15])) ? ALU_out_MEM :   // MEM->ID forwarding
            (RegWrite_WB && (rd_WB != 5'd0) && (rd_WB == inst_ID[19:15])) ? WB_value_WB :   // WB->ID forwarding
                                                                          RD1_ID_raw;
    assign fwd_RD2_ID =
            (RegWrite_EX && (rd_EX != 5'd0) && (rd_EX == inst_ID[24:20]) && ID_EX_MemRead_out == 1'b0) ? ALU_result_EX :
            (RegWrite_MEM && (rd_MEM != 5'd0) && (rd_MEM == inst_ID[24:20])) ? ALU_out_MEM :
            (RegWrite_WB && (rd_WB != 5'd0) && (rd_WB == inst_ID[24:20])) ? WB_value_WB :
                                                                          RD2_ID_raw;
    */
    // Branch Comparator removed from ID stage

    control CTRL (
        .RegWEn(RegWrite_ID_ctrl), .PCSel(PCSel_ID_ctrl), // MODIFIED: PCSel_ID_ctrl is potential CTI
        .ASel(ASel_ID_ctrl), .BSel(BSel_ID_ctrl), .WBSel(WBSel_ID_ctrl),
        .MemRW(MemRW_ID_ctrl), .ImmSel(ImmSel_ID_ctrl),
        .ALUSel(ALUSel_ID_ctrl), .BrU(BrU_ID_ctrl), // BrU passed to EX
        .instruction(inst_ID), 
        .BrEq(1'b0), .BrLt(1'b0) // MODIFIED: BrEq/BrLt not used by control unit directly anymore for PCSel
    );

    register_file RF (
        .RD1(RD1_ID_raw), .RD2(RD2_ID_raw),
        .RR1(inst_ID[19:15]), .RR2(inst_ID[24:20]),
        .WR(rd_WB), .WD(WB_value_WB),
        .RegWrite(RegWrite_WB),
        .clk(clk), .rst(rst), .pc_en(pc_en)
    );

    ImmGen IMM (.inst(inst_ID), .ImmSel(ImmSel_ID_ctrl), .Imm(Imm_ID));

    hazard_unit HAZ (
        .IF_ID_rs1(inst_ID[19:15]), .IF_ID_rs2(inst_ID[24:20]),
        .ID_EX_rd(rd_EX), .ID_EX_MemRead(ID_EX_MemRead_out),
        .stall(load_stall_ID)
    );
    assign stall_IFID = load_stall_ID;

    //--------------------------------------------------------------------------
    // 4) ID/EX Pipeline Register
    //--------------------------------------------------------------------------
    id_ex_reg ID_EX (
        .clk(clk), .rst(rst),
        .flush(1'b0), // Actual flush for mispredicts would come from EX/MEM stage feedback to IF/ID
        .RegWrite_in(RegWrite_ID_ctrl & ~load_stall_ID),
        .MemRW_in(MemRW_ID_ctrl & ~load_stall_ID),
        .MemRead_in((opcode_ID==7'b0000011) & ~load_stall_ID), // Is it a load instruction? (LW opcode)
        .PCSel_in(PCSel_ID_ctrl & ~load_stall_ID), // MODIFIED: Pass PCSel_ID_ctrl
        .ASel_in(ASel_ID_ctrl), .BSel_in(BSel_ID_ctrl), .BrU_in(BrU_ID_ctrl), // Pass BrU
        .WBSel_in(WBSel_ID_ctrl), .ALUSel_in(ALUSel_ID_ctrl),
        .pc_in(PC_ID), .pc4_in(PC4_ID),
        .rd1_in(RD1_ID_raw), .rd2_in(RD2_ID_raw), // MODIFIED: Pass raw RF values
        .imm_in(Imm_ID),
        .rs1_in(inst_ID[19:15]), .rs2_in(inst_ID[24:20]), .rd_in(inst_ID[11:7]),
        .opcode_in(opcode_ID),
        // outputs
        .RegWrite_out(RegWrite_EX), .MemRW_out(MemRW_EX), .MemRead_out(ID_EX_MemRead_out),
        .PCSel_out(PCSel_EX), // This is the PCSel_ID_ctrl latched
        .ASel_out(ASel_EX), .BSel_out(BSel_EX), .BrU_out(BrU_EX), // BrU_EX latched
        .WBSel_out(WBSel_EX), .ALUSel_out(ALUSel_EX),
        .pc_out(PC_EX), .pc4_out(PC4_EX),
        .rd1_out(RD1_EX), .rd2_out(RD2_EX), .imm_out(Imm_EX), // RD1_EX, RD2_EX are raw
        .rs1_out(rs1_EX), .rs2_out(rs2_EX), .rd_out(rd_EX),
        .opcode_out(opcode_EX)
    );

    //--------------------------------------------------------------------------
    // 5) EX Stage : ALU / Branch & JALR Target Calc / Branch Condition
    //--------------------------------------------------------------------------
    forwarding_unit FWD_EX (
        .ID_EX_rs1(rs1_EX), .ID_EX_rs2(rs2_EX),
        .EX_MEM_rd(rd_MEM), .EX_MEM_RegWrite(RegWrite_MEM),
        .MEM_WB_rd(rd_WB), .MEM_WB_RegWrite(RegWrite_WB),
        .fwdA(fwdA_EX_sel), .fwdB(fwdB_EX_sel)
    );

    wire [31:0] opA_EX_forwarded;
    wire [31:0] opB_EX_forwarded;

    assign opA_EX_forwarded = (fwdA_EX_sel == 2'b10) ? ALU_out_MEM :
                              (fwdA_EX_sel == 2'b01) ? WB_value_WB :
                                                       RD1_EX; // Default: value from ID/EX (raw from RF)
    assign opB_EX_forwarded = (fwdB_EX_sel == 2'b10) ? ALU_out_MEM :
                              (fwdB_EX_sel == 2'b01) ? WB_value_WB :
                                                       RD2_EX; // Default: value from ID/EX (raw from RF)

    assign srcA_EX = ASel_EX ? {24'd0, PC_EX} : opA_EX_forwarded;
    assign srcB_EX = BSel_EX ? Imm_EX         : opB_EX_forwarded;

    ALU ALU0 (.ALU_o(ALU_result_EX), .A(srcA_EX), .B(srcB_EX), .ALUSel(ALUSel_EX));

    // NEW: Branch Comparator in EX
    BranchComp BCOMP_EX (
        .BrEq(BrEq_EX), .BrLt(BrLt_EX),
        .A(opA_EX_forwarded), .B(opB_EX_forwarded), .BrU(BrU_EX) // Use forwarded ALU inputs, and BrU from ID/EX
    );
    
    wire [31:0] full_target_address_temp_EX; // 중간 결과를 위한 32비트 와이어 선언
    assign full_target_address_temp_EX = PC_EX + Imm_EX; // PC_EX는 Imm_EX의 폭에 맞춰 자동 확장됨
    assign branch_jal_target_EX_addr = full_target_address_temp_EX[7:0];

    // JALR detection and target calculation
    assign jalr_target_EX_addr = ALU_result_EX[7:0];

    // NEW: Logic to determine if a branch/JAL/JALR is taken in EX
    wire is_branch_op_EX = (opcode_EX == 7'b1100011) && PCSel_EX; // e.g. BEQ, BNE, BLT, BGE (PCSel_EX indicates a CTI)
    wire is_jal_op_EX    = (opcode_EX == 7'b1101111) && PCSel_EX; // JAL
    wire is_jalr_op_EX   = (opcode_EX == 7'b1100111) && PCSel_EX && RegWrite_EX; // JALR (RegWrite ensures it's a valid op)

    // Actual decision to take the branch
    // This logic depends on how BrEq_EX, BrLt_EX, BrU_EX map to specific branch conditions
    // Simplified example: (assuming BrU for unsigned, else signed, and ALUSel_EX might imply specific test)
    // For a real design, this needs to correctly decode func3 from inst_EX or use ALUSel_EX bits
    wire branch_condition_met_EX;
    // Example logic for typical branches, assuming BrU indicates if comparison is unsigned.
    // And specific ALUSel_EX values correspond to EQ, NE, LT, GE etc.
    // This is a placeholder for more detailed decoding based on ALUSel_EX or funct3.
    // For BEQ (ALUSel for EQ): BrEq_EX
    // For BNE (ALUSel for NE): !BrEq_EX
    // For BLT (ALUSel for LT, signed): BrLt_EX && !BrU_EX
    // For BGE (ALUSel for GE, signed): !BrLt_EX && !BrU_EX
    // For BLTU (ALUSel for LT, unsigned): BrLt_EX && BrU_EX
    // For BGEU (ALUSel for GE, unsigned): !BrLt_EX && BrU_EX
    // For now, a simplified approach:
    assign branch_condition_met_EX = (BrU_EX ? BrLt_EX : BrEq_EX); // Highly simplified, needs proper decode

    assign take_branch_EX_logic = is_branch_op_EX && branch_condition_met_EX;
    assign take_jal_EX_logic    = is_jal_op_EX; // JAL is unconditional if it's a JAL opcode and PCSel_EX is active
    assign take_jalr_EX_logic   = is_jalr_op_EX;

    //--------------------------------------------------------------------------
    // 6) EX/MEM Pipeline Register
    //--------------------------------------------------------------------------
    ex_mem_reg EX_MEM (
        .clk(clk), .rst(rst), .flush(1'b0), // Flush from MEM stage if JALR mispredict or exception in MEM
        .RegWrite_in(RegWrite_EX), .MemRW_in(MemRW_EX), .WBSel_in(WBSel_EX),
        // .PCSel_in(PCSel_EX), // MODIFIED: Replaced by specific take_* signals
        .take_jalr_in(take_jalr_EX_logic),         .jalr_target_in(jalr_target_EX_addr),
        .take_branch_in(take_branch_EX_logic), // NEW
        .take_jal_in(take_jal_EX_logic),       // NEW
        .branch_jal_target_in(branch_jal_target_EX_addr), // NEW
        .alu_in(ALU_result_EX), .rd2_in(opB_EX_forwarded), // opB for SW
        .pc4_in(PC4_EX), .rd_in(rd_EX), .pc_in(PC_EX), // Pass PC_EX for potential use or debug

        .RegWrite_out(RegWrite_MEM), .MemRW_out(MemRW_MEM), .WBSel_out(WBSel_MEM),
        // .PCSel_out(PCSel_MEM), // MODIFIED
        .take_jalr_out(take_jalr_MEM_logic),       .jalr_target_out(jalr_target_MEM_addr),
        .take_branch_out(take_branch_MEM_logic), // NEW
        .take_jal_out(take_jal_MEM_logic),     // NEW
        .branch_jal_target_out(branch_jal_target_MEM_addr), // NEW
        .alu_out(ALU_out_MEM), .rd2_out(RD2_MEM_value),
        .pc4_out(PC4_MEM), .rd_out(rd_MEM), .pc_out(PC_MEM)
    );

    //--------------------------------------------------------------------------
    // 7) MEM Stage : Data memory access
    //--------------------------------------------------------------------------
    data_mem DMEM (
        .ReadData(DMEM_out_MEM),
        .ADDR(ALU_out_MEM), .WriteData(RD2_MEM_value),
        .clk(clk), .MemWrite(MemRW_MEM), .pc_en(pc_en)
    );

    //--------------------------------------------------------------------------
    // 8) MEM/WB Pipeline Register
    //--------------------------------------------------------------------------
    mem_wb_reg MEM_WB (
        .clk(clk), .rst(rst),
        .RegWrite_in(RegWrite_MEM), .WBSel_in(WBSel_MEM),
        .alu_in(ALU_out_MEM), .dmem_in(DMEM_out_MEM), .pc4_in(PC4_MEM), .rd_in(rd_MEM),
        .RegWrite_out(RegWrite_WB), .WBSel_out(WBSel_WB),
        .alu_out(ALU_WB), .dmem_out(DMEM_WB), .pc4_out(PC4_WB), .rd_out(rd_WB)
    );

    //--------------------------------------------------------------------------
    // 9) WB Stage : Write-back to Register File
    //--------------------------------------------------------------------------
    assign WB_value_WB = (WBSel_WB == 2'b00) ? ALU_WB :
                         (WBSel_WB == 2'b01) ? DMEM_WB :
                                               {24'd0, PC4_WB}; // PC+4 for JAL/JALR

    //--------------------------------------------------------------------------
    // 10) PC Update Logic & IF/ID Flush Control
    //--------------------------------------------------------------------------
    wire [7:0] nextPC_sequential = PC_pred_IF;
    // MODIFIED: Targets now come from MEM stage (originated in EX)
    // wire [7:0] nextPC_branch_or_jal_ID = branch_target_ID_addr; // Removed
    // wire [7:0] nextPC_jalr_MEM         = jalr_target_MEM_addr; // Replaced by general target

    wire [7:0] nextPC_target_from_MEM;
    assign nextPC_target_from_MEM = take_jalr_MEM_logic   ? jalr_target_MEM_addr :
                                    take_jal_MEM_logic    ? branch_jal_target_MEM_addr : // JAL uses this target
                                    take_branch_MEM_logic ? branch_jal_target_MEM_addr :
                                                            nextPC_sequential; // Should not happen if any take is true

    wire use_target_from_MEM = take_jalr_MEM_logic || take_jal_MEM_logic || take_branch_MEM_logic;

    wire [7:0] nextPC;
    assign nextPC = use_target_from_MEM ? nextPC_target_from_MEM :
                                          nextPC_sequential;

    // MODIFIED: IF/ID Flush condition
    assign flush_IFID = use_target_from_MEM && !stall_IFID; // Flush if PC changes due to CTI from EX/MEM and not stalled for other reasons
                                                        // If stalled, the CTI is for a future instruction.
                                                        // Simpler: if CTI is taken, flush regardless of current stall state,
                                                        // as the stall applies to the instruction *before* the CTI.
                                                        // The CTI itself might be stalled by RegWrite_in(& ~load_stall_ID) earlier.
                                                        // If take_*_MEM is high, it means a valid CTI is at MEM stage.
    // assign flush_IFID = use_target_from_MEM; // Flush if any control transfer instruction is retiring from MEM stage

    // Refined flush: only flush if PC actually changes from sequential due to a CTI from MEM,
    // and the pipeline is not stalled holding back that CTI's effect.
    // The take_*_MEM signals should already be 0 if their originating instruction was squashed by a load stall.
    // So, if use_target_from_MEM is true, it means a valid CTI is being processed.
    assign flush_IFID = use_target_from_MEM;


    always @(posedge clk) begin
        if (rst) begin
            PC <= 8'h20; 
        end else if (pc_en) begin
            if (!stall_IFID) begin 
                PC <= nextPC;
            end
        end
    end

    //--------------------------------------------------------------------------
    // 11) Simulation halt condition
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        if (pc_en && (PC >= 8'hF0)) begin
            $display("%t: Simulation halting, PC = %h", $time, PC);
            $stop;
        end
    end

endmodule