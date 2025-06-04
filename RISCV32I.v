//==============================================================================
// Top-level 5-stage pipelined RV32I core  (with EX→ID forwarding)
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
    wire [7:0]  PC_pred_IF; // Renamed for clarity from PC_pred
    wire [31:0] inst_IF;

    // IF/ID Register
    wire        stall_IFID, flush_IFID;
    wire [7:0]  PC_ID, PC4_ID;
    wire [31:0] inst_ID;

    // ID Stage
    wire        RegWrite_ID_ctrl, MemRW_ID_ctrl;
    wire        ASel_ID_ctrl, BSel_ID_ctrl, BrU_ID_ctrl;
    wire [1:0]  WBSel_ID_ctrl;
    wire [2:0]  ImmSel_ID_ctrl;
    wire [3:0]  ALUSel_ID_ctrl;
    wire        PCSel_ID_eff; // Effective PCselect from Control unit in ID
    wire [31:0] RD1_ID_raw, RD2_ID_raw, Imm_ID;
    wire [31:0] fwd_RD1_ID, fwd_RD2_ID; // Forwarded read data
    wire        BrEq_ID, BrLt_ID;
    wire [6:0]  opcode_ID;
    wire        load_stall_ID; // Renamed for clarity from load_stall

    // ID/EX Register
    wire        RegWrite_EX, MemRW_EX, ID_EX_MemRead_out;
    wire        ASel_EX, BSel_EX, BrU_EX;
    wire [1:0]  WBSel_EX;
    wire [3:0]  ALUSel_EX;
    wire        PCSel_EX;   // PCselect signal latched in EX
    wire [7:0]  PC_EX, PC4_EX;
    wire [31:0] RD1_EX, RD2_EX, Imm_EX;
    wire [4:0]  rs1_EX, rs2_EX, rd_EX;
    wire [6:0]  opcode_EX;

    // EX Stage
    wire [1:0]  fwdA_EX_sel, fwdB_EX_sel; // Renamed from fwdA_EX, fwdB_EX for clarity (select signals)
    wire [31:0] srcA_EX, srcB_EX, ALU_result_EX;
    wire        take_jalr_EX;
    wire [7:0]  jalr_target_EX_addr; // Renamed from jalr_target_EX for clarity
    // ---- New signals for EX stage (inputs to ex_mem_reg) ----
    wire        take_branch_EX_decision;
    wire        take_jal_EX_decision;
    wire [7:0]  branch_or_jal_target_addr_EX;

    // EX/MEM Register
    wire        RegWrite_MEM, MemRW_MEM;
    wire [1:0]  WBSel_MEM;
    wire        PCSel_MEM; // PCSel latched in MEM (used for PC update logic from MEM stage)
    wire        take_jalr_MEM;
    wire [7:0]  jalr_target_MEM_addr; // Renamed from jalr_target_MEM
    wire [7:0]  PC4_MEM;
    wire [31:0] ALU_out_MEM, RD2_MEM_value; // Renamed RD2_MEM to avoid conflict if it means register value
    wire [4:0]  rd_MEM;
    wire [7:0]  PC_MEM;
    // ---- New signals from EX/MEM Register (outputs) ----
    wire        take_branch_MEM;
    wire        take_jal_MEM;
    wire [7:0]  branch_or_jal_target_addr_MEM;
    
    // MEM Stage
    wire [31:0] DMEM_out_MEM;

    // MEM/WB Register
    wire        RegWrite_WB;
    wire [1:0]  WBSel_WB;
    wire [7:0]  PC4_WB;
    wire [31:0] ALU_WB, DMEM_WB;
    wire [4:0]  rd_WB;
    wire [31:0] WB_value_WB; // Writeback data to RF

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
    // 3) ID Stage : Decode / Control / Branch Compare / Forwarding Logic
    //--------------------------------------------------------------------------
    assign opcode_ID = inst_ID[6:0];

    assign fwd_RD1_ID =
            (RegWrite_EX && (rd_EX != 5'd0) && (rd_EX == inst_ID[19:15])) ? ALU_result_EX :
            (RegWrite_MEM && (rd_MEM != 5'd0) && (rd_MEM == inst_ID[19:15])) ? ALU_out_MEM :
            (RegWrite_WB && (rd_WB != 5'd0) && (rd_WB == inst_ID[19:15])) ? WB_value_WB :
                                                                         RD1_ID_raw;
    assign fwd_RD2_ID =
            (RegWrite_EX && (rd_EX != 5'd0) && (rd_EX == inst_ID[24:20])) ? ALU_result_EX :
            (RegWrite_MEM && (rd_MEM != 5'd0) && (rd_MEM == inst_ID[24:20])) ? ALU_out_MEM :
            (RegWrite_WB && (rd_WB != 5'd0) && (rd_WB == inst_ID[24:20])) ? WB_value_WB :
                                                                         RD2_ID_raw;

    BranchComp BCOMP (
        .BrEq(BrEq_ID), .BrLt(BrLt_ID),
        .A(fwd_RD1_ID), .B(fwd_RD2_ID), .BrU(BrU_ID_ctrl)
    );

    control CTRL (
        .RegWEn(RegWrite_ID_ctrl), .PCSel(PCSel_ID_eff),
        .ASel(ASel_ID_ctrl), .BSel(BSel_ID_ctrl), .WBSel(WBSel_ID_ctrl),
        .MemRW(MemRW_ID_ctrl), .ImmSel(ImmSel_ID_ctrl),
        .ALUSel(ALUSel_ID_ctrl), .BrU(BrU_ID_ctrl),
        .instruction(inst_ID), .BrEq(BrEq_ID), .BrLt(BrLt_ID)
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
        .flush(1'b0),
        .RegWrite_in(RegWrite_ID_ctrl & ~load_stall_ID),
        .MemRW_in(MemRW_ID_ctrl & ~load_stall_ID),
        .MemRead_in((opcode_ID==7'b0000011) & ~load_stall_ID), // LW opcode
        .PCSel_in(PCSel_ID_eff & ~load_stall_ID),
        .ASel_in(ASel_ID_ctrl), .BSel_in(BSel_ID_ctrl), .BrU_in(BrU_ID_ctrl),
        .WBSel_in(WBSel_ID_ctrl), .ALUSel_in(ALUSel_ID_ctrl),
        .pc_in(PC_ID), .pc4_in(PC4_ID),
        .rd1_in(fwd_RD1_ID), .rd2_in(fwd_RD2_ID),
        .imm_in(Imm_ID),
        .rs1_in(inst_ID[19:15]), .rs2_in(inst_ID[24:20]), .rd_in(inst_ID[11:7]),
        .opcode_in(opcode_ID),
        // outputs
        .RegWrite_out(RegWrite_EX), .MemRW_out(MemRW_EX), .MemRead_out(ID_EX_MemRead_out),
        .PCSel_out(PCSel_EX),
        .ASel_out(ASel_EX), .BSel_out(BSel_EX), .BrU_out(BrU_EX),
        .WBSel_out(WBSel_EX), .ALUSel_out(ALUSel_EX),
        .pc_out(PC_EX), .pc4_out(PC4_EX),
        .rd1_out(RD1_EX), .rd2_out(RD2_EX), .imm_out(Imm_EX),
        .rs1_out(rs1_EX), .rs2_out(rs2_EX), .rd_out(rd_EX),
        .opcode_out(opcode_EX)
    );

    //--------------------------------------------------------------------------
    // 5) EX Stage : ALU / JALR target calculation / Branch & JAL decision
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
                                                       RD1_EX;
    assign opB_EX_forwarded = (fwdB_EX_sel == 2'b10) ? ALU_out_MEM :
                              (fwdB_EX_sel == 2'b01) ? WB_value_WB :
                                                       RD2_EX;

    assign srcA_EX = ASel_EX ? {24'd0, PC_EX} : opA_EX_forwarded;
    assign srcB_EX = BSel_EX ? Imm_EX         : opB_EX_forwarded;

    ALU ALU0 (.ALU_o(ALU_result_EX), .A(srcA_EX), .B(srcB_EX), .ALUSel(ALUSel_EX));

    // JALR detection and target calculation
    assign take_jalr_EX        = PCSel_EX && (opcode_EX == 7'b1100111); // JALR opcode and PCSel asserted
    assign jalr_target_EX_addr = ALU_result_EX[7:0];

    // ---- New logic for Branch and JAL decision and target in EX ----
    wire is_branch_instr_type_EX = (opcode_EX == 7'b1100011); // Branch opcodes (BEQ, BNE, etc.)
    wire is_jal_instr_type_EX    = (opcode_EX == 7'b1101111); // JAL opcode

    // PCSel_EX is high if the control unit (in ID) decided this instruction should change PC flow
    assign take_branch_EX_decision = PCSel_EX && is_branch_instr_type_EX;
    assign take_jal_EX_decision    = PCSel_EX && is_jal_instr_type_EX;

    // ALU_result_EX holds PC_EX + Imm_EX for taken branches and JAL, which is their target address.
    // For JALR, ALU_result_EX holds RD1_EX + Imm_EX.
    assign branch_or_jal_target_addr_EX = ALU_result_EX[7:0];

    //--------------------------------------------------------------------------
    // 6) EX/MEM Pipeline Register
    //--------------------------------------------------------------------------
    ex_mem_reg EX_MEM (
        .clk(clk),
        .rst(rst),
        .flush(1'b0), // Assuming flush for EX/MEM is handled by squashing its inputs if needed upstream, or not needed here.

        // Original control/data signals
        .RegWrite_in(RegWrite_EX),
        .MemRW_in(MemRW_EX),
        .WBSel_in(WBSel_EX),
        .PCSel_in(PCSel_EX), // Pass general PCSel
        .take_jalr_in(take_jalr_EX),
        .jalr_target_in(jalr_target_EX_addr),
        .alu_in(ALU_result_EX),
        .rd2_in(opB_EX_forwarded), // Data to be written to memory for ST instructions
        .pc4_in(PC4_EX),
        .rd_in(rd_EX),
        .pc_in(PC_EX),      // EX stage PC

        // ---- Newly added inputs for ex_mem_reg ----
        .take_branch_in(take_branch_EX_decision),
        .take_jal_in(take_jal_EX_decision),
        .branch_jal_target_in(branch_or_jal_target_addr_EX),
 
        // Original outputs
        .RegWrite_out(RegWrite_MEM),
        .MemRW_out(MemRW_MEM),
        .WBSel_out(WBSel_MEM),
        .PCSel_out(PCSel_MEM),
        .take_jalr_out(take_jalr_MEM),
        .jalr_target_out(jalr_target_MEM_addr),
        .alu_out(ALU_out_MEM),
        .rd2_out(RD2_MEM_value),
        .pc4_out(PC4_MEM),
        .rd_out(rd_MEM),
        .pc_out(PC_MEM),    // Piped PC from EX

        // ---- Newly added outputs from ex_mem_reg ----
        .take_branch_out(take_branch_MEM),
        .take_jal_out(take_jal_MEM),
        .branch_jal_target_out(branch_or_jal_target_addr_MEM)
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
                                               {24'd0, PC4_WB};

    //--------------------------------------------------------------------------
    // 10) PC Update Logic & IF/ID Flush Control
    //--------------------------------------------------------------------------
    wire [31:0] branch_target_ID_full;
    wire [7:0]  branch_target_ID_addr;
    assign branch_target_ID_full = {24'd0, PC_ID} + Imm_ID;
    assign branch_target_ID_addr = branch_target_ID_full[7:0];

    wire [7:0] nextPC_sequential         = PC_pred_IF;
    wire [7:0] nextPC_target_id_resolved = branch_target_ID_addr;
    wire [7:0] nextPC_target_mem_jalr    = jalr_target_MEM_addr;
    // wire [7:0] nextPC_target_mem_branch_jal = branch_or_jal_target_addr_MEM; // Available if PC logic were to use it

    wire [7:0] nextPC;

    // PC Muxing Logic:
    // Priority: JALR from MEM, then Branch/JAL from ID (for fast redirection & flush).
    // The new signals (take_branch_MEM, take_jal_MEM, branch_or_jal_target_addr_MEM) are available
    // but not directly altering this PC mux to maintain current branch resolution strategy.
    // They could be used if branch/JAL resolution was intended to be finalized based on MEM stage info.
    assign nextPC = take_jalr_MEM ? nextPC_target_mem_jalr :                             // 1. JALR from MEM stage takes precedence
                    (PCSel_ID_eff && !load_stall_ID) ? nextPC_target_id_resolved : // 2. Branch or JAL from ID stage
                                                       nextPC_sequential;         // 3. Default sequential PC

    // IF/ID Flush condition:
    // Flush if a branch/JAL is taken in ID (PCSel_ID_eff) OR if JALR from MEM is taken.
    assign flush_IFID = (PCSel_ID_eff && !load_stall_ID) || take_jalr_MEM;

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
    // 11) Simulation halt condition (for testbench control)
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        if (pc_en && (PC >= 8'hF0)) begin
            $display("%t: Simulation halting, PC = %h", $time, PC);
            $stop;
        end
    end

endmodule
