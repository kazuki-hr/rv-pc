/**************************************************************************************************/
/**** RVSoC                                            since 2018-08-07   ArchLab. TokyoTech   ****/
/**** The Processor v0.01                                                                      ****/
/**************************************************************************************************/
`default_nettype none
/**************************************************************************************************/
`include "define.vh"

/***** 2 read 1 write port, register file                                                     *****/
/**************************************************************************************************/
module m_regfile (CLK, w_rs1, w_rs2, w_rdata1, w_rdata2, w_we, rd, w_wdata);
    input  wire        CLK;
    input  wire [ 4:0] w_rs1, w_rs2;
    output wire [31:0] w_rdata1, w_rdata2;
    input  wire        w_we;
    input  wire [ 4:0] rd;
    input  wire [31:0] w_wdata;

    reg [31:0] mem [0:31];

    assign w_rdata1 = (w_rs1 == 0) ? 0 : mem[w_rs1];
    assign w_rdata2 = (w_rs2 == 0) ? 0 : mem[w_rs2];

    always @(posedge CLK) begin
        if(w_we) begin
            if(rd!=0) mem[rd] <= w_wdata;
        end
    end

    integer i;
    initial begin
        for(i=0; i<32; i=i+1) mem[i] = 0;
`ifdef LINUX
        mem[11] = `D_INITD_ADDR + `D_START_PC;
`endif
    end
endmodule
/**************************************************************************************************/
/***** main processor                                                                         *****/
/**************************************************************************************************/
module m_RVCoreM(CLK, RST_X, w_stall, r_halt, w_insn_addr, w_data_addr, w_insn_data, w_data_data, w_is_dram_data,
                w_data_wdata, w_data_we, w_data_ctrl, w_priv, w_satp, w_mstatus, w_mtime, w_mc_mode,
                w_mtimecmp, w_wmtimecmp, w_clint_we, w_mip, w_wmip, w_plic_we, w_busy, w_pagefault,
                w_tlb_req, w_tlb_flush, w_core_pc, w_core_ir, w_core_odata, w_init_stage);
    input  wire         CLK, RST_X, w_stall;
    input  wire [127:0]  w_insn_data;
    input  wire [127:0]  w_data_data;
    input  wire         w_is_dram_data;
    input  wire [63:0]  w_wmtimecmp;
    input  wire         w_clint_we;
    input  wire [31:0]  w_wmip;
    input  wire         w_plic_we;
    input  wire         w_busy;
    input  wire [31:0]  w_pagefault;
    input  wire [2:0]   w_mc_mode;

    output reg          r_halt;         // register, set if the processor is halted
    output wire [31:0]  w_core_odata;   // constant 0
    output wire [31:0]  w_data_wdata;   // from r_data_wdata
    output wire [31:0]  w_insn_addr;    // from r_insn_addr
    output wire [2:0]   w_data_ctrl;    // from r_data_ctrl
    output wire [31:0]  w_data_addr;    // from r_mem_addr
    output wire [63:0]  w_mtime;        // from register mtime
    output wire [63:0]  w_mtimecmp;     // from register mtimecmp
    output wire [31:0]  w_priv;         // from register priv
    output wire [31:0]  w_satp;         // from register satp
    output wire [31:0]  w_mstatus;      // from register mstatus
    output wire [31:0]  w_mip;          // from register mip
    output wire [31:0]  w_core_pc;      // from register pc
    output wire [31:0]  w_core_ir;      // from r_ir
    output wire         w_init_stage;   // from r_init_stage
    output wire  [1:0]  w_tlb_req;      // from r_tlb_req
    output wire         w_data_we;      // from r_data_we, write enable for DRAM memory
    output wire         w_tlb_flush;    // from r_tlb_flush

    /***** registers and CPU architecture state ***************************************************/
    reg  [31:0] pc                 = `D_START_PC;  // Program Counter

    reg  [31:0] mstatus            = 0;            ///// CSRs
    reg  [31:0] mtvec              = 0;            //
    reg  [31:0] mscratch           = 0;            //
    reg  [31:0] mepc               = 0;            //
    reg  [31:0] mcause             = 0;            //
    reg  [31:0] mtval              = 0;            //
    reg  [31:0] mhartid            = 0;            //
    reg  [31:0] misa               = 32'h00141105; // RV32acim, Machine ISA register (MISA)
    reg  [31:0] mie                = 0;            //
    reg  [31:0] mip                = 0;            //
    reg  [31:0] medeleg            = 0;            //
    reg  [31:0] mideleg            = 0;            //
    reg  [31:0] mcounteren         = 0;            //
    reg  [31:0] stvec              = 0;            //
    reg  [31:0] sscratch           = 0;            //
    reg  [31:0] sepc               = 0;            //
    reg  [31:0] scause             = 0;            //
    reg  [31:0] stval              = 0;            //
    reg  [31:0] satp               = 0;            //
    reg  [31:0] scounteren         = 0;            //

    reg  [31:0] load_res           = 0;            // For aomic LR/SC
    reg         reserved           = 0;            // For aomic LR/SC
    reg   [1:0] priv               = `PRIV_M;      // Mode
    reg  [63:0] mtime              = 1;            // Mtime
    reg  [63:0] mtimecmp           = 1;            // Mtime

    /***** CPU Stage registers ********************************************************************/

    reg         r_tkn              = 0;            // E1:         // flag for branck taken or untaken
    reg  [31:0] r_jmp_pc           = 0;            // E1:
    reg  [31:0] r_mem_addr         = 0;            // E1:
    reg  [31:0] r_wb_data          = 0;            // E1:
    reg  [31:0] r_wb_data_csr      = 0;            // E1:


    /***** Pipeline registers ********************************************************************/
    // IfId
    reg  [31:0] IfId_ir           = 0;
    reg  [31:0] IfId_pc           = 0;
    reg         IfId_flushed      = 1;
    reg  [31:0] IfId_pending_exception = ~0;
    // IdEx
    reg  [31:0] IdEx_ir           = 0;
    reg  [31:0] IdEx_pc           = 0;
    reg  [31:0] IdEx_rrs1         = 0;
    reg  [31:0] IdEx_rrs2         = 0;
    reg  [31:0] IdEx_rcsr         = 0;
    reg   [6:0] IdEx_opcode       = `RV32_NOP & 6'h3f;
    reg   [4:0] IdEx_rd           = 0;
    reg   [4:0] IdEx_rs1          = 0;
    reg   [4:0] IdEx_rs2          = 0;
    reg   [2:0] IdEx_funct3       = 0;
    reg   [4:0] IdEx_funct5       = 0;
    reg   [6:0] IdEx_funct7       = 0;
    reg  [11:0] IdEx_funct12      = 0;
    reg  [31:0] IdEx_imm          = 0;
    reg         IdEx_regfile_we   = 0;
    reg         IdEx_op_CSR       = 0;
    reg         IdEx_use_rs2      = 0;
    reg   [6:0] IdEx_alu_f7       = 0;
    reg         IdEx_op_SC        = 0;
    reg         IdEx_op_LR        = 0;
    reg         IdEx_op_LOAD      = 0;
    reg         IdEx_op_STORE     = 0;
    reg         IdEx_op_AMO       = 0;
    reg         IdEx_op_SYS       = 0;
    reg         IdEx_op_ECALL     = 0;
    reg         IdEx_op_SRET      = 0;
    reg         IdEx_op_MRET      = 0;
    reg         IdEx_op_FENCEI    = 0;
    reg         IdEx_op_SFENCEVMA = 0;
    reg         IdEx_op_CSR_MSTA  = 0;
    reg         IdEx_op_CSR_SSTA  = 0;
    reg         IdEx_op_CSR_SATP  = 0;
    reg         IdEx_op_jmp       = 0;
    reg         IdEx_flushed      = 1;
    reg  [31:0] IdEx_pending_exception = ~0;
    // ExMem
    reg  [31:0] ExMem_ir           = 0;
    reg  [31:0] ExMem_pc           = 0;
    reg  [31:0] ExMem_rrs1         = 0;
    reg  [31:0] ExMem_rrs2         = 0;
    reg  [31:0] ExMem_rcsr         = 0;
    reg   [6:0] ExMem_opcode       = `RV32_NOP & 6'h3f;
    reg   [4:0] ExMem_rd           = 0;
    reg   [4:0] ExMem_rs1          = 0;
    reg   [4:0] ExMem_rs2          = 0;
    reg   [2:0] ExMem_funct3       = 0;
    reg   [4:0] ExMem_funct5       = 0;
    reg   [6:0] ExMem_funct7       = 0;
    reg  [11:0] ExMem_funct12      = 0;
    reg         ExMem_regfile_we   = 0;
    reg         ExMem_op_CSR       = 0;
    reg         ExMem_op_SC        = 0;
    reg         ExMem_op_LR        = 0;
    reg         ExMem_op_LOAD      = 0;
    reg         ExMem_op_STORE     = 0;
    reg         ExMem_op_AMO       = 0;
    reg         ExMem_op_SYS       = 0;
    reg         ExMem_op_SRET      = 0;
    reg         ExMem_op_MRET      = 0;
    reg         ExMem_op_CSR_MSTA  = 0;
    reg         ExMem_op_CSR_SSTA  = 0;
    reg         ExMem_op_SFENCEVMA = 0;
    reg         ExMem_op_CSR_SATP  = 0;
    reg  [31:0] ExMem_wb_data      = 0;
    reg  [31:0] ExMem_mem_addr     = 0;
    reg  [31:0] ExMem_mem_wdata    = 0;
    reg  [31:0] ExMem_wb_data_csr  = 0;
    reg         ExMem_tkn          = 0;
    reg  [31:0] ExMem_jmp_pc       = 0;
    reg         ExMem_flushed      = 1;
    reg  [31:0] ExMem_pending_exception = ~0;
    // MemWb
    reg  [31:0] MemWb_pc           = 0;
    reg  [31:0] MemWb_rrs1         = 0;
    reg  [31:0] MemWb_rrs2         = 0;
    reg  [31:0] MemWb_rcsr         = 0;
    reg   [4:0] MemWb_rd           = 0;
    reg   [4:0] MemWb_rs1          = 0;
    reg   [4:0] MemWb_rs2          = 0;
    reg   [2:0] MemWb_funct3       = 0;
    reg   [4:0] MemWb_funct5       = 0;
    reg   [6:0] MemWb_funct7       = 0;
    reg  [11:0] MemWb_funct12      = 0;
    reg         MemWb_regfile_we   = 0;
    reg         MemWb_op_CSR       = 0;
    reg         MemWb_op_SC        = 0;
    reg         MemWb_op_LR        = 0;
    reg         MemWb_op_LOAD      = 0;
    reg         MemWb_op_STORE     = 0;
    reg         MemWb_op_AMO       = 0;
    reg         MemWb_op_SYS       = 0;
    reg         MemWb_op_SRET      = 0;
    reg         MemWb_op_MRET      = 0;
    reg         MemWb_op_CSR_MSTA  = 0;
    reg         MemWb_op_CSR_SSTA  = 0;
    reg         MemWb_op_SFENCEVMA = 0;
    reg         MemWb_op_CSR_SATP  = 0;
    reg  [31:0] MemWb_wb_data      = 0;
    reg  [31:0] MemWb_mem_addr     = 0;
    reg  [31:0] MemWb_wb_data_csr  = 0;
    reg  [31:0] MemWb_mem_rdata    = 0;
    reg  [31:0] MemWb_mem_wdata    = 0;
    reg         MemWb_tkn          = 0;
    reg  [31:0] MemWb_jmp_pc       = 0;
    reg         MemWb_flushed      = 1;
    reg  [31:0] MemWb_pending_exception = ~0;

    wire        w_csr_flush;


    /***** Other Registers ************************************************************************/
    reg  [63:0] e_icount    = 1;
    reg  [31:0] r_rcsr_t    = 0;
    reg  [31:0] pc_after_intr = 0;
    reg  tlb_flush;
    reg  tkn;
    /***********************************           INI          ***********************************/

    /*If this signal is set, interrupts are
      accepted in the next cycle.*/
    reg    accept_interrupt;

    wire   w_take_exception;
    wire   w_take_interrupt;
    assign w_init_stage = accept_interrupt & (priv == `PRIV_M ? mie[11] & mstatus[3]: mie[9] & mstatus[1]) & w_tlb_req != `ACCESS_READ & w_tlb_req != `ACCESS_WRITE;

    /***********************************           IF           ***********************************/

    reg  inst_cache_we;
    reg  fetch_from_cache;
    wire [127:0] w_inst_cache_odata;
    wire w_inst_cache_hit;
    wire w_inst_cache_flush = tlb_flush | w_mc_mode == `MC_MODE_DISK | IdEx_op_FENCEI;

    m_tlb #(
        .ADDR_WIDTH(28), // for 4-word blocks
        .D_WIDTH(128),
        .ENTRY(32)
    ) inst_cache (
        .CLK(CLK),
        .RST_X(RST_X),
        .w_flush(w_inst_cache_flush),
        .w_we(inst_cache_we),
        .w_waddr(pc[31:4]),
        .w_raddr(pc[31:4]),
        .w_idata(w_insn_data),
        .w_odata(w_inst_cache_odata),
        .w_oe(w_inst_cache_hit)
    );

    wire [127:0] w_instruction128 = (fetch_from_cache? w_inst_cache_odata : w_insn_data) >> {pc[3:2], 5'b0};
    wire [31:0]  w_instruction = w_instruction128[31:0];

    assign w_insn_addr = pc;

    reg  IfId_stall;
    wire IfId_flush;
    always @ (posedge CLK) begin
        if (!RST_X | IfId_flush) begin
            IfId_ir <= `RV32_NOP;
            IfId_flushed <= 1;
            IfId_pending_exception <= ~0;
        end else if (w_pagefault == `CAUSE_FETCH_PAGE_FAULT) begin
            // do not flush the value of the PC:
            // we use it when computing the value of mtval
            IfId_pc      <= pc;
            IfId_ir      <= `RV32_NOP;
            IfId_flushed <= 0;
            IfId_pending_exception <= w_pagefault;
        end else if (!IfId_stall) begin
            IfId_ir   <= w_instruction;
            IfId_pc   <= pc;
            IfId_flushed <= 0;
            IfId_pending_exception <= ~0;
        end
    end

    /***********************************            ID        ***********************************/
    wire [31:0] w_imm;
    m_imm_gen imm_gen0(IfId_ir, w_imm);

    wire  [4:0] w_rs1     = IfId_ir[19:15];
    wire  [4:0] w_rs2     = IfId_ir[24:20];
    wire  [6:0] w_opcode  = IfId_ir[ 6: 0];
    wire  [2:0] w_funct3  = IfId_ir[14:12];
    wire  [4:0] w_funct5  = IfId_ir[31:27];
    wire  [6:0] w_funct7  = IfId_ir[31:25];
    wire [11:0] w_funct12 = IfId_ir[31:20];
    wire        w_op_priv = (w_opcode==`OPCODE_SYSTEM__ && IfId_ir[14:12]==`FUNCT3_PRIV__);
    wire        w_op_csr  = (w_opcode==`OPCODE_SYSTEM__ && IfId_ir[14:12]!=`FUNCT3_PRIV__);
    wire        w_regfile_we = (w_opcode == `OPCODE_LOAD____) | /** regfile write enable **/
                               (w_opcode == `OPCODE_LUI_____) |
                               (w_opcode == `OPCODE_AUIPC___) |
                               (w_opcode == `OPCODE_JAL_____) |
                               (w_opcode == `OPCODE_JALR____) |
                               (w_opcode == `OPCODE_OP______) |
                               (w_opcode == `OPCODE_OP_IMM__) |
                               (w_opcode == `OPCODE_AMO_____) |
                               (w_opcode == `OPCODE_SYSTEM__ && w_funct3!=`FUNCT3_PRIV__);
    wire        w_op_jmp  = (w_opcode == `OPCODE_JALR____) |
                            (w_opcode == `OPCODE_JAL_____) |
                            (w_opcode == `OPCODE_BRANCH__) |
                            (w_opcode == `OPCODE_MISC_MEM) |
                            (w_opcode == `OPCODE_SYSTEM__);

    wire        w_use_rs2 = (w_opcode == `OPCODE_OP______) |
                            (w_opcode == `OPCODE_STORE___) |
                            (w_opcode == `OPCODE_BRANCH__) |
                            (w_opcode == `OPCODE_AMO_____);

    wire [31:0] w_rrs1, w_rrs2;
    m_regfile regs(CLK, w_rs1, w_rs2, w_rrs1, w_rrs2, MemWb_regfile_we, MemWb_rd, MemWb_wb_data);

    wire IdEx_stall;
    wire IdEx_flush;

    wire fw_WBtoID1;
    wire fw_WBtoID2;

    forward_unit fwd_WBtoID1(w_rs1, MemWb_rd, MemWb_regfile_we, fw_WBtoID1);
    forward_unit fwd_WBtoID2(w_rs2, MemWb_rd, MemWb_regfile_we, fw_WBtoID2);

    always @(posedge CLK) begin
        if (!IdEx_stall) begin
            IdEx_ir          <= IfId_ir;
            IdEx_pc          <= IfId_pc;
            IdEx_rrs1        <= (fw_WBtoID1) ? MemWb_wb_data : w_rrs1;
            IdEx_rrs2        <= (fw_WBtoID2) ? MemWb_wb_data : w_rrs2;
            IdEx_rcsr        <= r_rcsr_t;
            IdEx_opcode      <= (IdEx_flush) ?  `OPCODE_OP_IMM__ : w_opcode;
            IdEx_rd          <= IfId_ir[11: 7];
            IdEx_rs1         <= (IdEx_flush) ? 5'h0 : w_rs1;    // flush to prevent uneccesary forwarding
            IdEx_rs2         <= (IdEx_flush) ? 5'h0 : w_rs2;
            IdEx_funct3      <= (IdEx_flush) ? 3'h0 : w_funct3;
            IdEx_funct5      <= (IdEx_flush) ? 5'h0 : w_funct5;
            IdEx_funct7      <= (IdEx_flush) ? 7'h0 : w_funct7;
            IdEx_funct12     <= (IdEx_flush) ? 12'h0 :w_funct12;
            IdEx_imm         <= w_imm;
            IdEx_regfile_we  <= (IdEx_flush) ? 1'b0 : w_regfile_we;
            IdEx_use_rs2     <= (IdEx_flush) ? 1'b0 : w_use_rs2;
            IdEx_op_CSR      <= (IdEx_flush) ? 1'b0 : w_op_csr;
            IdEx_alu_f7      <= (IdEx_flush) ? 7'h0 :
                                (w_opcode==`OPCODE_OP_IMM__) ?
                                ((w_funct3==`FUNCT3_ADD___) ? 0 : w_funct7 & 7'h20) : w_funct7;

            IdEx_op_SC       <= (IdEx_flush) ? 1'b0 : (w_opcode==`OPCODE_AMO_____ & w_funct5==`FUNCT5_SC______);
            IdEx_op_LR       <= (IdEx_flush) ? 1'b0 : (w_opcode==`OPCODE_AMO_____ & w_funct5==`FUNCT5_LR______);
            IdEx_op_LOAD     <= (IdEx_flush) ? 1'b0 : (w_opcode==`OPCODE_LOAD____);
            IdEx_op_STORE    <= (IdEx_flush) ? 1'b0 : (w_opcode==`OPCODE_STORE___);
            IdEx_op_AMO      <= (IdEx_flush) ? 1'b0 : (w_opcode==`OPCODE_AMO_____ &
                                                       w_funct5!=`FUNCT5_LR______ &
                                                       w_funct5!=`FUNCT5_SC______);
            IdEx_op_SYS      <= (IdEx_flush) ? 1'b0 : (w_opcode==`OPCODE_SYSTEM__);
            IdEx_op_ECALL    <= (IdEx_flush) ? 1'b0 : (w_op_priv && w_funct12==`FUNCT12_ECALL_);
            IdEx_op_SRET     <= (IdEx_flush) ? 1'b0 : (w_op_priv && w_funct12==`FUNCT12_SRET__);
            IdEx_op_MRET     <= (IdEx_flush) ? 1'b0 : (w_op_priv && w_funct12==`FUNCT12_MRET__);
            IdEx_op_SFENCEVMA<= (IdEx_flush) ? 1'b0 : (w_op_priv && w_funct7==`FUNCT7_SFENCE_VMA); // ignore rs1 and rs2
            IdEx_op_FENCEI   <= (IdEx_flush) ? 1'b0 : (w_opcode==`OPCODE_MISC_MEM && w_funct3==`FUNCT3_FENCEI);
            IdEx_op_CSR_MSTA <= (IdEx_flush) ? 1'b0 : (w_op_csr  && w_funct12==`CSR_MSTATUS);
            IdEx_op_CSR_SSTA <= (IdEx_flush) ? 1'b0 : (w_op_csr  && w_funct12==`CSR_SSTATUS);
            IdEx_op_CSR_SATP <= (IdEx_flush) ? 1'b0 : (w_op_csr  && w_funct12==`CSR_SATP);
            IdEx_op_jmp      <= (IdEx_flush) ? 1'b0 :  w_op_jmp;
            IdEx_flushed     <= IfId_flushed | IdEx_flush;
            IdEx_pending_exception <= (IdEx_flush & !tkn) ? ~0 : IfId_pending_exception;
        end
    end


    /***********************************           EX          ***********************************/


    wire [31:0] w_alu_im_rslt;
    wire        w_alu_b_rslt;
    wire [31:0] w_alu_c_rslt;
    wire        w_use_imm = (IdEx_opcode == `OPCODE_OP_IMM__);
    wire        fw_MEMtoEX1;
    wire        fw_MEMtoEX2;
    wire        fw_WBtoEX1;
    wire        fw_WBtoEX2;
    wire        w_mem_load = ExMem_op_LOAD | ExMem_op_LR | ExMem_op_AMO;
    wire        w_load_use_hazard1 = fw_MEMtoEX1  & w_mem_load;
    wire        w_load_use_hazard2 = (fw_MEMtoEX2 & IdEx_use_rs2) & w_mem_load;
    wire        w_ex_hazard  = w_load_use_hazard1 | w_load_use_hazard2;
    reg         r_load_data_avail = 0;  // for resolving data hazard
    wire        w_hazard_stall = (w_ex_hazard) & !r_load_data_avail;
    /*In case of a load-use data hazard, put data read from memory in this register.
      This cannot be a pipeline register, because for forwarding to work, we have to
      stall all of the EX, MEM, and WB stages.*/
    reg [31:0] r_mem_rdata_for_hazard;
    reg  r_EX_new_inst = 0;

    wire [31:0] w_MEM_wb_data_nonload; // from MEM

    wire [31:0] w_alu_in1  = (w_load_use_hazard1) ? r_mem_rdata_for_hazard :
                             (fw_MEMtoEX1)        ? w_MEM_wb_data_nonload  :
                             (fw_WBtoEX1)         ? MemWb_wb_data  :
                                                    IdEx_rrs1;
    wire [31:0] w_alu_in2  = (w_use_imm)          ? IdEx_imm :
                             (w_load_use_hazard2) ? r_mem_rdata_for_hazard :
                             (fw_MEMtoEX2)        ? w_MEM_wb_data_nonload  :
                             (fw_WBtoEX2)         ? MemWb_wb_data  :
                                                    IdEx_rrs2;

    wire        w_aluim_en = (IdEx_opcode==`OPCODE_OP______ || w_use_imm) & !w_hazard_stall & r_EX_new_inst;
    wire [31:0] w_mem_addr = (IdEx_op_LOAD || IdEx_op_STORE) ? w_alu_in1+IdEx_imm :
                             (IdEx_op_AMO | IdEx_op_LR | IdEx_op_SC) ? w_alu_in1 : 0;
    wire [31:0] w_mem_wdata = (w_load_use_hazard2) ? r_mem_rdata_for_hazard :
                              (fw_MEMtoEX2)        ? ExMem_wb_data :
                              (fw_WBtoEX2)         ? MemWb_wb_data :
                                                     IdEx_rrs2;
    wire        w_alu_busy;
    wire        w_EX_mul_or_div = (IdEx_opcode == `OPCODE_OP______ & IdEx_funct7[0]);
    wire        w_ex_busy = w_alu_busy | w_hazard_stall | (w_EX_mul_or_div & r_EX_new_inst);
    reg  [31:0] w_EX_wb_data;
    reg  [31:0] wb_data_csr;
    reg  [31:0] jmp_pc;
    reg         illegal_inst;

    forward_unit fwd_MEMtoEx1(IdEx_rs1, ExMem_rd, ExMem_regfile_we, fw_MEMtoEX1);
    forward_unit fwd_MEMtoEx2(IdEx_rs2, ExMem_rd, ExMem_regfile_we, fw_MEMtoEX2);
    forward_unit fwd_WBtoEx1(IdEx_rs1, MemWb_rd, MemWb_regfile_we, fw_WBtoEX1);
    forward_unit fwd_WBtoEx2(IdEx_rs2, MemWb_rd, MemWb_regfile_we, fw_WBtoEX2);


    m_alu_im ALU_IM(CLK, (RST_X & !IdEx_flush), w_aluim_en, w_alu_in1, w_alu_in2, IdEx_funct3, IdEx_alu_f7, w_alu_im_rslt,
                    w_alu_busy);
    m_alu_b  ALU_B(w_alu_in1, w_alu_in2, IdEx_funct3, w_alu_b_rslt);
    m_alu_c  ALU_C(IdEx_rcsr, w_alu_in1, IdEx_imm, IdEx_funct3, w_alu_c_rslt);


    always @ (*) begin
        tkn = 0;
        w_EX_wb_data = 0;
        wb_data_csr = 0;
        jmp_pc = 0;
        illegal_inst = 0;
        case(IdEx_opcode)
            `OPCODE_LUI_____ : begin w_EX_wb_data  = IdEx_imm;       end
            `OPCODE_AUIPC___ : begin w_EX_wb_data  = IdEx_pc + IdEx_imm;  end
            `OPCODE_OP______ : begin w_EX_wb_data  = w_alu_im_rslt;  end
            `OPCODE_OP_IMM__ : begin w_EX_wb_data  = w_alu_im_rslt;  end
            `OPCODE_LOAD____ : begin end
            `OPCODE_STORE___ : begin end
            `OPCODE_JAL_____ : begin
                tkn          = 1;
                w_EX_wb_data = IdEx_pc + 4;
                jmp_pc       = IdEx_pc + IdEx_imm;
            end
            `OPCODE_JALR____ : begin
                tkn          = !w_hazard_stall;
                w_EX_wb_data = IdEx_pc + 4;
                jmp_pc       = w_alu_in1 + IdEx_imm;
            end
            `OPCODE_BRANCH__ : begin
                tkn          = !w_hazard_stall & w_alu_b_rslt;
                jmp_pc       = IdEx_pc + IdEx_imm;
            end
            `OPCODE_AMO_____ : begin end
            `OPCODE_MISC_MEM : begin
             if (IdEx_funct3==`FUNCT3_FENCEI) begin
                tkn = 1;
                jmp_pc = IdEx_pc + 4;end
             end
            `OPCODE_SYSTEM__ : begin
            if(IdEx_funct3 == `FUNCT3_PRIV__) begin
                case (IdEx_funct12)
                    `FUNCT12_ECALL_ : begin end
                    `FUNCT12_EBREAK : begin end
                    `FUNCT12_URET__ : begin
                            // next instruction to execute after the pipeline is flushed
                            jmp_pc    = IdEx_rcsr;
                        end
                        `FUNCT12_SRET__ : begin
                            jmp_pc    = IdEx_rcsr;
                        end
                        `FUNCT12_MRET__ : begin
                            jmp_pc    = IdEx_rcsr;
                        end
                        `FUNCT12_WFI___ : begin end
                        default: begin
                            if(IdEx_funct7==`FUNCT7_SFENCE_VMA) begin
                                tkn = 1;
                                jmp_pc = IdEx_pc + 4;
                            end
                        end
                    endcase
                end else begin
                    wb_data_csr = w_alu_c_rslt;
                    // next instruction to execute after the pipeline is flushed
                    jmp_pc      = IdEx_pc + 4;
                end
            end
            default : begin
                tkn       = 0;
                jmp_pc    = 0;
                illegal_inst = 1;
                $write("UNKNOWN OPCODE DETECT!!\n");
                $write("TC:%08d PC:%08x OPCODE=%7b, ir=%8x\n", mtime[31:0], IdEx_pc, IdEx_opcode, IdEx_ir);
                $write("Simulation Stopped...\n");
                $finish();
            end
        endcase
    end

    // for ALU controll
    reg r_hazard_stall_prev = 0;
    always @ (posedge CLK) begin
        r_hazard_stall_prev <= w_hazard_stall;
        if (!IdEx_stall) begin
            r_EX_new_inst <= 1;
        end else if (r_hazard_stall_prev & !w_hazard_stall) begin
            // hazard resolved
            r_EX_new_inst <= 0;
        end else if (w_hazard_stall) begin
            r_EX_new_inst <= 1;
        end else begin
            r_EX_new_inst <= 0;
        end
    end

    reg ExMem_stall;
    wire ExMem_flush;

    always @(posedge CLK) begin
        if (!ExMem_stall) begin
            ExMem_ir          <= IdEx_ir;
            ExMem_pc          <= IdEx_pc;
            ExMem_rrs1        <= IdEx_rrs1;
            ExMem_rrs2        <= IdEx_rrs2;
            ExMem_rcsr        <= IdEx_rcsr;
            ExMem_rd          <= IdEx_rd;
            ExMem_rs1         <= IdEx_rs1;
            ExMem_rs2         <= IdEx_rs2;
            ExMem_funct3      <= IdEx_funct3;
            ExMem_funct5      <= IdEx_funct5;
            ExMem_funct7      <= IdEx_funct7;
            ExMem_funct12     <= IdEx_funct12;
            ExMem_regfile_we  <= (ExMem_flush) ? 1'b0 : IdEx_regfile_we;
            ExMem_wb_data     <= (IdEx_op_SYS && IdEx_funct3 != `FUNCT3_PRIV__) ? IdEx_rcsr : w_EX_wb_data;
            ExMem_wb_data_csr <= wb_data_csr;
            ExMem_mem_addr    <= w_mem_addr; // memory address for Load, Store and Atomic
            ExMem_mem_wdata   <= w_mem_wdata;
            ExMem_op_SC       <= (ExMem_flush) ? 1'b0 : IdEx_op_SC;
            ExMem_op_LR       <= (ExMem_flush) ? 1'b0 : IdEx_op_LR;
            ExMem_op_LOAD     <= (ExMem_flush) ? 1'b0 : IdEx_op_LOAD;
            ExMem_op_STORE    <= (ExMem_flush) ? 1'b0 : IdEx_op_STORE;
            ExMem_op_AMO      <= (ExMem_flush) ? 1'b0 : IdEx_op_AMO;
            ExMem_op_SYS      <= (ExMem_flush) ? 1'b0 : IdEx_op_SYS;
	        ExMem_op_SRET     <= (ExMem_flush) ? 1'b0 : IdEx_op_SRET;
	        ExMem_op_MRET     <= (ExMem_flush) ? 1'b0 : IdEx_op_MRET;
            ExMem_op_SFENCEVMA<= (ExMem_flush) ? 1'b0 : IdEx_op_SFENCEVMA;
            ExMem_op_CSR      <= (ExMem_flush) ? 1'b0 : IdEx_op_CSR;
            ExMem_op_CSR_MSTA <= (ExMem_flush) ? 1'b0 : IdEx_op_CSR_MSTA;
            ExMem_op_CSR_SSTA <= (ExMem_flush) ? 1'b0 : IdEx_op_CSR_SSTA;
            ExMem_op_CSR_SATP <= (ExMem_flush) ? 1'b0 : IdEx_op_CSR_SATP;
            ExMem_tkn         <= (ExMem_flush) ? 0 : (tkn | IdEx_op_CSR | IdEx_op_MRET | IdEx_op_SRET);
            ExMem_jmp_pc      <= jmp_pc;
            ExMem_flushed     <= IdEx_flushed | ExMem_flush;
            ExMem_pending_exception <= (ExMem_flush)   ? ~0 :
                                       (IdEx_op_ECALL) ? `CAUSE_USER_ECALL + priv:
                                       (illegal_inst)  ? `CAUSE_ILLEGAL_INSTRUCTION:
                                                       IdEx_pending_exception;
        end
    end

    /***********************************           Mem           ***********************************/

    wire MemWb_stall;
    wire MemWb_flush;

    reg  data_cache_we;
    reg  replace_cach_entry;
    reg  load_from_cache;
    wire [127:0] w_data_cache_odata;
    wire w_data_cache_hit;
    wire w_data_cache_flush = tlb_flush | w_mc_mode == `MC_MODE_DISK;
    wire [31:0]  w_mask_width = (ExMem_funct3 == 0) ? {24'h0, 8'hff} :
                                (ExMem_funct3 == 1) ? {16'h0, 16'hffff} :
                                32'hffffffff;
    wire [127:0] w_cach_wdata_mask = ~({96'h0, w_mask_width} << {ExMem_mem_addr[3:0], 3'b0});
    wire [127:0] w_data_cache_wdata = (replace_cach_entry) ?
                  (w_data_cache_odata & w_cach_wdata_mask) | (({96'h0, w_data_wdata} << {ExMem_mem_addr[3:0], 3'b0}) & ~w_cach_wdata_mask)
                  : w_data_data;

    m_tlb #(
        .ADDR_WIDTH(28), // for 4-word blocks
        .D_WIDTH(128),
        .ENTRY(32)
    ) data_cache (
        .CLK(CLK),
        .RST_X(RST_X),
        .w_flush(w_data_cache_flush),
        .w_we(data_cache_we),
        .w_waddr(ExMem_mem_addr[31:4]),
        .w_raddr(ExMem_mem_addr[31:4]),
        .w_idata(w_data_cache_wdata),
        .w_odata(w_data_cache_odata),
        .w_oe(w_data_cache_hit)
    );

    wire [127:0] w_odata_t1 = (load_from_cache ?  w_data_cache_odata : w_data_data) >> {ExMem_mem_addr[3:0], 3'b0};
    wire [31:0] w_odata_t2 = (!load_from_cache & !w_is_dram_data) ? w_data_data[31:0] : w_odata_t1[31:0];

    wire [31:0] w_ld_lb = {{24{w_odata_t2[ 7]&(~ExMem_funct3[2])}}, w_odata_t2[ 7:0]};
    wire [31:0] w_ld_lh = {{16{w_odata_t2[15]&(~ExMem_funct3[2])}}, w_odata_t2[15:0]};

    wire [31:0] w_mem_rdata = (ExMem_funct3[1:0]==0) ? w_ld_lb :
                              (ExMem_funct3[1:0]==1) ? w_ld_lh : w_odata_t2;


    wire fw_WBtoMem;
    wire w_datamem_pagefault = (w_pagefault == `CAUSE_LOAD_PAGE_FAULT |
                                w_pagefault == `CAUSE_STORE_PAGE_FAULT);


    always@(posedge CLK) begin
        if (!MemWb_stall & !w_datamem_pagefault & !w_take_interrupt) begin
            if(ExMem_op_LR) begin
                reserved <= 1;
            end else if (ExMem_op_SC) begin
                reserved <= 0;
            end
        end
        /*load_res should only be updated when reserved is set:
          this is done to match the value of load_res with simrv*/
        if (!MemWb_stall & ExMem_op_LR) begin
            load_res <= ExMem_mem_addr;
        end
    end

    wire w_sc_success = (ExMem_mem_addr==load_res) && reserved;

    wire [31:0] w_alu_a_rslt;
    reg  [31:0] r_alu_a_rslt;
    reg  [31:0] r_amo_mem_rdata;
    m_alu_a ALU_A (ExMem_mem_wdata, r_amo_mem_rdata, ExMem_funct5, w_alu_a_rslt);


    assign w_MEM_wb_data_nonload =  (ExMem_op_AMO) ? r_amo_mem_rdata :
                                    (ExMem_op_SC)  ? !w_sc_success :
                                                     ExMem_wb_data;


    always @(posedge CLK) begin
        if (!MemWb_stall) begin
            MemWb_pc          <= ExMem_pc;
            MemWb_rrs1        <= ExMem_rrs1;
            MemWb_rrs2        <= ExMem_rrs2;
            MemWb_rcsr        <= ExMem_rcsr;
            MemWb_rd          <= ExMem_rd;
            MemWb_rs1         <= ExMem_rs1;
            MemWb_rs2         <= ExMem_rs2;
            MemWb_funct3      <= ExMem_funct3;
            MemWb_funct5      <= ExMem_funct5;
            MemWb_funct7      <= ExMem_funct7;
            MemWb_funct12     <= ExMem_funct12;
            MemWb_regfile_we  <= (MemWb_flush | w_datamem_pagefault) ? 1'b0 : ExMem_regfile_we;
            MemWb_op_SC       <= (MemWb_flush) ? 1'b0 : ExMem_op_SC;
            MemWb_op_LR       <= (MemWb_flush) ? 1'b0 : ExMem_op_LR;
            MemWb_op_LOAD     <= (MemWb_flush) ? 1'b0 : ExMem_op_LOAD;
            MemWb_op_STORE    <= (MemWb_flush) ? 1'b0 : ExMem_op_STORE;
            MemWb_op_AMO      <= (MemWb_flush) ? 1'b0 : ExMem_op_AMO;
            MemWb_op_SYS      <= (MemWb_flush) ? 1'b0 : ExMem_op_SYS;
            MemWb_op_SRET     <= (MemWb_flush) ? 1'b0 : ExMem_op_SRET;
	        MemWb_op_MRET     <= (MemWb_flush) ? 1'b0 : ExMem_op_MRET;
            MemWb_op_SFENCEVMA<= (MemWb_flush) ? 1'b0 : ExMem_op_SFENCEVMA;
            MemWb_op_CSR      <= (MemWb_flush) ? 1'b0 : ExMem_op_CSR;
            MemWb_op_CSR_MSTA <= (MemWb_flush) ? 1'b0 : ExMem_op_CSR_MSTA;
            MemWb_op_CSR_SSTA <= (MemWb_flush) ? 1'b0 : ExMem_op_CSR_SSTA;
            MemWb_op_CSR_SATP <= (MemWb_flush) ? 1'b0 : ExMem_op_CSR_SATP;
            MemWb_wb_data     <= ((ExMem_op_LOAD | ExMem_op_LR) & w_ex_hazard) ? r_mem_rdata_for_hazard :
                                 (ExMem_op_LOAD | ExMem_op_LR) ? w_mem_rdata :
                                                        w_MEM_wb_data_nonload;
            MemWb_wb_data_csr <= ExMem_wb_data_csr;
            MemWb_mem_addr    <= ExMem_mem_addr;
            MemWb_mem_wdata   <= ExMem_mem_wdata;
            MemWb_tkn         <= (MemWb_flush) ? 0 : ExMem_tkn;
            MemWb_jmp_pc      <= ExMem_jmp_pc;
            MemWb_flushed     <= ExMem_flushed | MemWb_flush;
            MemWb_pending_exception <=  (MemWb_flush)        ? ~0                    :
                                         w_datamem_pagefault ? w_pagefault           :
                                                               ExMem_pending_exception;
            pc_after_intr     <= (ExMem_flushed | MemWb_flush) ? pc_after_intr :
                                 (ExMem_tkn) ?                   ExMem_jmp_pc  :
                                                                 ExMem_pc + 4;

        end
    end


    assign w_data_wdata = (ExMem_op_STORE | ExMem_op_SC) ?  ExMem_mem_wdata : r_alu_a_rslt;




    /***********************************           WB           ***********************************/
    assign w_csr_flush = MemWb_op_CSR | MemWb_op_MRET | MemWb_op_SRET;

        /*******************************     CSRs  and Exception   ***********************************/

    reg [31:0] inst_cnt = 0;
    always @ (posedge CLK) begin
`ifdef REAL_MTIME
        if(RST_X) mtime <= mtime + 1;
        if (!MemWb_flushed && !MemWb_stall) inst_cnt <= inst_cnt + 1;
`else
        if (!MemWb_flushed && !MemWb_stall) mtime <= mtime + 1;
`endif
    end


    wire [31:0] pending_interrupts = mip & mie;
    wire [31:0] enable_interrupts  = (pending_interrupts) ? (priv == `PRIV_M) ? ((mstatus & `MSTATUS_MIE) ? ~mideleg : 0) :
                                                            (priv == `PRIV_S) ? ((mstatus & `MSTATUS_SIE) ? (~0) : ~mideleg) :
                                                            (priv == `PRIV_U) ? ~0 : 0 : 0;
    wire [31:0] w_interrupt_mask = pending_interrupts & enable_interrupts;
    wire [31:0] w_irq_t          = w_interrupt_mask & (~w_interrupt_mask+1);

    /* We assume that interrupts are taken between instructions in the
       EX stage and MEM stage, and when the latter instruction has left
       the MEM stage.
    */
    reg    r_accept_interrupt_prev = 0;

    assign w_take_interrupt = (w_interrupt_mask != 0) & r_accept_interrupt_prev & !w_busy;
    assign w_take_exception = MemWb_pending_exception != ~0 & !w_take_interrupt;
    reg [31:0] irq_num;

    always@(*) begin
        case (w_irq_t)
            //32'h00000001: irq_num = 0;  //reserved
            32'h00000002: irq_num = 1;
            //32'h00000004: irq_num = 2;  //reserved
            32'h00000008: irq_num = 3;
            //32'h00000010: irq_num = 4;  //reserved
            32'h00000020: irq_num = 5;
            //32'h00000040: irq_num = 6;  //reserved
            32'h00000080: irq_num = 7;
            //32'h00000100: irq_num = 8;  //reserved
            32'h00000200: irq_num = 9;
            //32'h00000400: irq_num = 10; //reserved
            32'h00000800: irq_num = 11;
            /*32'h00001000: irq_num = 12;
            32'h00002000: irq_num = 13;
            32'h00004000: irq_num = 14;    //reserved
            32'h00008000: irq_num = 15;*/
            default:      irq_num = 32;
        endcase
    end

    wire [31:0] cause = (w_take_interrupt) ? (`CAUSE_INTERRUPT | irq_num) : MemWb_pending_exception;
    wire w_deleg = (priv > `PRIV_S) ? 0 :
                   (w_take_interrupt) ? (mideleg & w_irq_t) != 0 : ((medeleg >> cause) & 1 != 0);

    always @ (posedge CLK) begin
        if (!RST_X || r_halt) begin
            pc <= `D_START_PC;
        end else if (w_take_interrupt) begin
            pc <= (w_deleg) ? stvec : mtvec;
        end else if (w_take_exception) begin
            pc <= (w_deleg) ? stvec : mtvec;
        end else if (w_csr_flush) begin
            pc <= MemWb_jmp_pc;
        end else if (tkn) begin
            pc <= jmp_pc;
        end else if (!IfId_stall) begin
            pc <= pc + 4;
        end
    end

    reg [31:0] next_tval;

    always @ ( * ) begin
        if (w_take_exception) begin
            if (MemWb_pending_exception == `CAUSE_FETCH_PAGE_FAULT) begin
                next_tval = MemWb_pc;
            end else if (MemWb_pending_exception == `CAUSE_LOAD_PAGE_FAULT |
                         MemWb_pending_exception == `CAUSE_STORE_PAGE_FAULT) begin
                next_tval = MemWb_mem_addr;
            end else begin
                next_tval = 0;
            end
        end else begin
            next_tval = 0;
        end
    end


    /***********************************           CSR          ***********************************/
    wire [11:0] w_csr_addr = (w_funct3  != `FUNCT3_PRIV__ )  ? w_funct12 :   // from ID stage
                             (w_funct12 == `FUNCT12_ECALL_)  ? `CSR_MTVEC:
                             (w_funct12 == `FUNCT12_URET__)  ? `CSR_UEPC :
                             (w_funct12 == `FUNCT12_SRET__)  ? `CSR_SEPC :
                             (w_funct12 == `FUNCT12_MRET__)  ? `CSR_MEPC : 0;

    wire [31:0] w_sstatus_t = (mstatus | 32'h6000) & 32'h000de133;
    wire [31:0] w_mstatus_t = (mstatus | 32'h6000);

    always@(*) begin /***** read CSR register, for OF stage  *****/
        case(w_csr_addr)
            12'h3A0         : r_rcsr_t = 0;
            12'h3B0         : r_rcsr_t = 0;
            `CSR_FFLAGS     : r_rcsr_t = 0;
            `CSR_FRM        : r_rcsr_t = 0;
            `CSR_FCSR       : r_rcsr_t = 0;
            `CSR_SIE        : r_rcsr_t = mie & mideleg;
            `CSR_STVEC      : r_rcsr_t = stvec;
            `CSR_SCOUNTEREN : r_rcsr_t = scounteren;
            `CSR_SSCRATCH   : r_rcsr_t = sscratch;
            `CSR_SEPC       : r_rcsr_t = sepc;
            `CSR_SCAUSE     : r_rcsr_t = scause;
            `CSR_STVAL      : r_rcsr_t = stval;
            `CSR_SIP        : r_rcsr_t = mip & mideleg;
            `CSR_SATP       : r_rcsr_t = satp;

            `CSR_MEDELEG    : r_rcsr_t = medeleg;
            `CSR_MIDELEG    : r_rcsr_t = mideleg;
            `CSR_MIE        : r_rcsr_t = mie;
            `CSR_MTVEC      : r_rcsr_t = mtvec;
            `CSR_MCOUNTEREN : r_rcsr_t = mcounteren;
            `CSR_MSCRATCH   : r_rcsr_t = mscratch;
            `CSR_MEPC       : r_rcsr_t = mepc;
            `CSR_MCAUSE     : r_rcsr_t = mcause;
            `CSR_MTVAL      : r_rcsr_t = mtval;
            `CSR_MIP        : r_rcsr_t = mip;
            `CSR_MISA       : r_rcsr_t = misa | 32'h40000000;

            `CSR_MCYCLE     : r_rcsr_t = mtime[31:0];
            `CSR_MINSTRET   : r_rcsr_t = mtime[31:0];
            `CSR_CYCLE      : r_rcsr_t = mtime[31:0];
            `CSR_INSTRET    : r_rcsr_t = mtime[31:0];
`ifdef SIM_MODE
            `CSR_TIME       : r_rcsr_t = mtime[31:0]+ !IdEx_flushed + !ExMem_flushed + !MemWb_flushed;
`else
            `CSR_TIME       : r_rcsr_t = mtime[31:0];
`endif
            `CSR_MCYCLEH    : r_rcsr_t = mtime[63:32];
            `CSR_MINSTRETH  : r_rcsr_t = mtime[63:32];
            `CSR_CYCLEH     : r_rcsr_t = mtime[63:32];
            `CSR_INSTRETH   : r_rcsr_t = mtime[63:32];
            `CSR_TIMEH      : r_rcsr_t = mtime[63:32];

            `CSR_SSTATUS    : r_rcsr_t = (w_sstatus_t[31:13]==3 | w_sstatus_t[31:15]==3) ? (w_sstatus_t | 32'h80000000) : w_sstatus_t;
            `CSR_MSTATUS    : r_rcsr_t = (w_mstatus_t[31:13]==3 | w_mstatus_t[31:15]==3) ? (w_mstatus_t | 32'h80000000) : w_mstatus_t;
            `CSR_MHARTID    : r_rcsr_t = mhartid;
            default         : r_rcsr_t = 0;
        endcase
    end



    reg[31:0] next_mip;
    always @ (*) begin
        next_mip = mip;
        if(mtime > `ENABLE_TIMER && (mtimecmp + 1 < (mtime + !MemWb_flushed)) && accept_interrupt && !ExMem_flushed) begin
            next_mip = mip | `MIP_MTIP;
        end
        if (w_clint_we) begin
            next_mip = mip & ~`MIP_MTIP;
        end else if (w_plic_we) begin
            next_mip = w_wmip;
        end
    end

    always @ (posedge CLK) begin
        if (w_clint_we) begin
            mtimecmp    <= w_wmtimecmp;
        end
    end

    wire [31:0] w_sstatus   = (mstatus & ~`SSTATUS_MASK) | (MemWb_wb_data_csr & `SSTATUS_MASK);

    wire [31:0] w_sstatus_t1 = (mstatus & ~`MSTATUS_SPIE) | (((mstatus >> `PRIV_S) & 1) << `MSTATUS_SPIE_SHIFT);
    wire [31:0] w_sstatus_t2 = (w_sstatus_t1  & ~`MSTATUS_SPP) | (priv << `MSTATUS_SPP_SHIFT);
    wire [31:0] w_sstatus_t3 = (w_sstatus_t2  & ~`MSTATUS_SIE);
    wire [31:0] w_mstatus_t1 = (mstatus & ~`MSTATUS_MPIE) | (((mstatus >> `PRIV_M) & 1) << `MSTATUS_MPIE_SHIFT);
    wire [31:0] w_mstatus_t2 = (w_mstatus_t1  & ~`MSTATUS_MPP) | (priv << `MSTATUS_MPP_SHIFT);
    wire [31:0] w_mstatus_t3 = (w_mstatus_t2  & ~`MSTATUS_MIE);

    always@(posedge CLK) begin /***** write CSR registers *****/
        if(!w_busy) begin
            if(w_take_interrupt) begin
                if(w_deleg) begin
                    scause  <= cause;
                    /*When an interrupt occurs when an instruction that
                      caused exception is in the MEM stage,
                      restart that instruction after the interrupt is handled.
                    */
                    sepc    <= (MemWb_pending_exception != ~0) ? MemWb_pc :
                                pc_after_intr;
                    stval   <= next_tval;
                    mstatus <= w_sstatus_t3;
                    priv    <= `PRIV_S;
                end else begin
                    mcause  <= cause;
                    mepc    <= (MemWb_pending_exception != ~0) ? MemWb_pc :
                                pc_after_intr;
                    mtval   <= next_tval;
                    mstatus <= w_mstatus_t3;
                    priv    <= `PRIV_M;
                end
            end
            else if(w_take_exception) begin
                if(w_deleg) begin
                    scause  <= cause;
                    sepc    <= MemWb_pc;
                    stval   <= next_tval;
                    mstatus <= w_sstatus_t3;
                    priv    <= `PRIV_S;
                end else begin
                    mcause  <= cause;
                    mepc    <= MemWb_pc;
                    mtval   <= next_tval;
                    mstatus <= w_mstatus_t3;
                    priv    <= `PRIV_M;
                end
            end
            else if(MemWb_op_MRET | MemWb_op_SRET) begin
                case (MemWb_funct12)
                    `FUNCT12_SRET__ : begin
                        mstatus <= (((mstatus & ~(1<<`PRIV_S)) | (mstatus[5] << `PRIV_S)) | 32'h20) & ~32'h100;
                        priv    <= mstatus[8];
                    end
                    `FUNCT12_MRET__ : begin
                        mstatus <= (((mstatus & ~(1 << `PRIV_M))
                                     | (mstatus[`MSTATUS_MPIE_SHIFT] << `PRIV_M))
                                    | `MSTATUS_MPIE) & ~`MSTATUS_MPP;
                        priv    <= mstatus[`MSTATUS_MPP_SHIFT+1:`MSTATUS_MPP_SHIFT];
                    end
                endcase
            end
            else if(MemWb_op_CSR) begin
                case(MemWb_funct12)
                    `CSR_SEPC       : sepc       <= MemWb_wb_data_csr & ~1;
                    `CSR_SCAUSE     : scause     <= MemWb_wb_data_csr;
                    `CSR_STVAL      : stval      <= MemWb_wb_data_csr;
                    `CSR_MEPC       : mepc       <= MemWb_wb_data_csr & ~1;
                    `CSR_MCAUSE     : mcause     <= MemWb_wb_data_csr;
                    `CSR_MTVAL      : mtval      <= MemWb_wb_data_csr;
                    `CSR_SIP        : mip        <= (mip & ~mideleg) | (MemWb_wb_data_csr & mideleg);
                    `CSR_MIP        : mip        <= (mip & ~`WCSR_MASK4) | (MemWb_wb_data_csr & `WCSR_MASK4);
                    `CSR_MSTATUS    : mstatus    <= (mstatus & ~`MASK_STATUS) | (MemWb_wb_data_csr & `MASK_STATUS);
                    `CSR_SSTATUS    : mstatus    <= (mstatus & ~`MASK_STATUS) | (w_sstatus & `MASK_STATUS);
                endcase
            end else begin
                    mip <= next_mip;
            end
    end else begin
                mip <= next_mip;
        end
    end


    always @ (posedge CLK) begin
        if (!w_busy) begin
            if (MemWb_op_CSR) begin
                case(MemWb_funct12)
                    `CSR_STVEC      : stvec      <= MemWb_wb_data_csr & ~3;
                    `CSR_SCOUNTEREN : scounteren <= MemWb_wb_data_csr & 5;
                    `CSR_SSCRATCH   : sscratch   <= MemWb_wb_data_csr;
                    `CSR_MTVEC      : mtvec      <= MemWb_wb_data_csr & ~3;
                    `CSR_MCOUNTEREN : mcounteren <= MemWb_wb_data_csr & 5;
                    `CSR_MSCRATCH   : mscratch   <= MemWb_wb_data_csr;
                    `CSR_SIE        : mie        <= (mie & ~mideleg) | (MemWb_wb_data_csr & mideleg);
                    `CSR_MEDELEG    : medeleg    <= (medeleg & ~`WCSR_MASK1) | (MemWb_wb_data_csr & `WCSR_MASK1);
                    `CSR_MIDELEG    : mideleg    <= (mideleg & ~`WCSR_MASK2) | (MemWb_wb_data_csr & `WCSR_MASK2);
                    `CSR_MIE        : mie        <= (mie & ~`WCSR_MASK3) | (MemWb_wb_data_csr & `WCSR_MASK3);
                    `CSR_SATP       : satp       <= MemWb_wb_data_csr;
                endcase
            end
        end
    end


    /***********************************       TLB FLUSH!       ***********************************/
    wire [31:0] w_data_t = (MemWb_funct12==`CSR_MSTATUS) ? MemWb_wb_data_csr : w_sstatus;
    wire [31:0] w_mod    = w_data_t ^ mstatus;

    assign w_tlb_flush   = tlb_flush & !w_busy; // generate this signal to flush TLBs

    always@(*) begin
        if(w_take_exception || w_interrupt_mask != 0) tlb_flush = 1;
        else if(MemWb_op_SRET || MemWb_op_MRET) tlb_flush = 1;
        else if(MemWb_op_CSR_MSTA) begin
            //if((w_mod & (`MSTATUS_MPRV | `MSTATUS_SUM | `MSTATUS_MXR)) != 0 ||
            //   ((mstatus & `MSTATUS_MPRV) && (w_mod & `MSTATUS_MPP) != 0)) begin
                tlb_flush = 1;
            //end else tlb_flush = 0;*/
        end
        else if(MemWb_op_CSR_SSTA) begin
            //if((w_mod & (`MSTATUS_MPRV | `MSTATUS_SUM | `MSTATUS_MXR)) != 0 ||
            //   ((mstatus & `MSTATUS_MPRV) && (w_mod & `MSTATUS_MPP) != 0)) begin
                tlb_flush = 1;
            //end else tlb_flush = 0;*/
        end else if (MemWb_op_CSR_SATP | IdEx_op_SFENCEVMA) tlb_flush = 1;
        else tlb_flush = 0;
    end


    /*************************     Memory Accesses    ************************************/

    reg    r_fetch_page_fault  = 0;
    reg    hazard_data_load;
    reg    data_wen;
    reg    data_ren;
    reg    inst_ren;

    assign w_data_we = data_wen;

/*********************************************/
    localparam  IDLE            = 0;
    localparam  STORE           = 1;
    localparam  LOAD            = 2;
    localparam  INST_READ       = 3;
    localparam  LOAD_USE_HAZARD = 4;
    localparam  AMO_LOAD        = 5;
    localparam  AMO_ALU         = 6;
    localparam  AMO_STORE       = 7;
    localparam  INTERRUPT       = 8;

    wire w_interrupt_ok = !ExMem_op_CSR & !ExMem_op_SRET & !ExMem_op_MRET;

    reg [3:0] mem_access_state = IDLE;
    reg [3:0] next_state;

    always @ (*) begin
        data_wen = 0;
        data_ren = 0;
        inst_ren = 0;
        inst_cache_we = 0;
        fetch_from_cache = 1;
        data_cache_we = 0;
        replace_cach_entry = 0;
        load_from_cache = 0;
        hazard_data_load = 0;
        IfId_stall = 1;
        ExMem_stall = 1;
        accept_interrupt = 0;
        next_state = mem_access_state;
        if (!RST_X) begin
            next_state = IDLE;
        end else if (w_take_interrupt & mem_access_state != INTERRUPT) begin
            IfId_stall = 0;
            ExMem_stall = 0;
            next_state = INTERRUPT;
        end else if (w_pagefault != ~0 | w_take_exception | w_csr_flush) begin
        // wait for one cycle
            next_state = IDLE;
            ExMem_stall = 0;
            IfId_stall  = 0;
        end else begin
            case (mem_access_state)
                IDLE: begin
                    /* wait for EX to finish, in order to let forwarding work*/
                    if (!w_ex_busy & !w_busy) begin
                        if (ExMem_op_AMO) begin
                            data_ren = 1;
                            next_state = AMO_LOAD;
                        end else if (ExMem_op_STORE | (ExMem_op_SC & w_sc_success)) begin
                            data_wen = 1;
                            next_state = STORE;
                        end else if (ExMem_op_LOAD | ExMem_op_LR) begin
                            if (w_data_cache_hit) begin
                                next_state = IDLE;
                                ExMem_stall = 0;
                                if (w_inst_cache_hit) begin
                                    IfId_stall = 0;
                                end
                                load_from_cache = 1;
                                accept_interrupt = w_interrupt_ok;
                            end else begin
                                data_ren = 1;
                                next_state = LOAD;
                            end
                        end else if (!r_fetch_page_fault) begin
                            if (IdEx_op_jmp & !w_inst_cache_hit) begin
                                next_state = IDLE;
				                ExMem_stall = 0;
                                accept_interrupt = w_interrupt_ok;
                            end else if (w_inst_cache_hit) begin
                                next_state = IDLE;
                                IfId_stall = 0;
                                ExMem_stall = 0;
                                accept_interrupt = w_interrupt_ok;
                            end else begin
                                inst_ren = 1;
                                next_state = INST_READ;
                            end
                        end else begin
                            ExMem_stall = 0;
                        end
                    end else if (w_ex_hazard) begin
                        if (w_data_cache_hit & ExMem_op_LOAD) begin
                            next_state = LOAD_USE_HAZARD;
                            hazard_data_load = 1;
                            load_from_cache = 1;
                        end else begin
                            data_ren = 1;
                            if (ExMem_op_AMO) begin
                                next_state = AMO_LOAD;
                            end else begin
                                next_state = LOAD;
                            end
                        end
                    end
                end
                STORE: begin
                    if (!w_busy) begin
                        next_state = IDLE;
                        if (w_inst_cache_hit) begin
                            IfId_stall = 0;
                        end
                        if (w_data_cache_hit) begin
                            replace_cach_entry = 1;
                            data_cache_we = 1;
                        end
                        ExMem_stall = 0;
			            accept_interrupt = w_interrupt_ok;
                    end else begin
                        data_wen = 1;
                    end
                end
                LOAD: begin
                    if (!w_busy) begin
                        if (w_ex_hazard) begin
                            next_state = LOAD_USE_HAZARD;
                        end else begin
                            if (w_inst_cache_hit) begin
                                IfId_stall = 0;
                            end
                            /* If the data is from sources other than memory(devices, clint, etc.),
                               do not write it into the cache*/
                            data_cache_we = w_is_dram_data;
                            next_state = IDLE;
                            ExMem_stall = 0;
			                accept_interrupt = w_interrupt_ok;
                        end
                        hazard_data_load = 1;
                    end else begin
                        data_ren = 1;
                    end
                end
                INST_READ: begin
                    if (!w_busy) begin
                        next_state  = IDLE;
                        IfId_stall  = 0;
                        ExMem_stall = 0;
                        fetch_from_cache = 0;
                        inst_cache_we = 1;
                        accept_interrupt = w_interrupt_ok;
                    end else begin
                        inst_ren = 1;
                    end
                end
                LOAD_USE_HAZARD: begin
                    next_state = IDLE;
                    if (!w_ex_busy) begin
                        ExMem_stall = 0;
                    end
                    accept_interrupt = w_interrupt_ok;
                end
                AMO_LOAD: begin
                    if (!w_busy) begin
                        next_state = AMO_ALU;
                        hazard_data_load = 1;
                    end else begin
                        data_ren = 1;
                    end
                end
                AMO_ALU: begin
                // wait for the ALU to compute the result
                    next_state = AMO_STORE;
                end
                AMO_STORE: begin
                    next_state = STORE;
                    data_wen = 1;
                end
                INTERRUPT : begin
                    if (!w_busy) begin
                        next_state = IDLE;
                    end
                end
                default: ;
            endcase
        end
    end

    always @ (posedge CLK) begin
        mem_access_state <= next_state;
        if (!IdEx_stall) begin
            r_load_data_avail<= 0;
        end if (next_state == LOAD_USE_HAZARD || mem_access_state == AMO_ALU) begin
            r_load_data_avail <= 1;
        end

        if (hazard_data_load) begin
            r_mem_rdata_for_hazard <= w_mem_rdata;
        end

        if (mem_access_state == AMO_LOAD & !w_busy) begin
            r_amo_mem_rdata <= w_mem_rdata;
        end
        if (w_pagefault == `CAUSE_FETCH_PAGE_FAULT) begin
            r_fetch_page_fault <= 1;
        end else if (w_take_exception) begin
            r_fetch_page_fault <= 0;
        end

        if (mem_access_state == AMO_ALU) begin
            r_alu_a_rslt <= w_alu_a_rslt;
        end

    	if (!w_busy) begin
            r_accept_interrupt_prev <= accept_interrupt;
    	end
    end


/************************************************/

    assign w_data_addr =  ExMem_mem_addr;

    assign w_tlb_req  = (!RST_X)   ? 2'h3 :
                        (data_wen) ? `ACCESS_WRITE :
                        (data_ren) ? `ACCESS_READ  :
                        (inst_ren) ? `ACCESS_CODE  : 2'h3;

    assign w_data_ctrl = (!RST_X) ? 0 : (ExMem_op_AMO) ? 3'h2 : ExMem_funct3;

    assign w_satp     = satp;

    assign w_priv     = priv;
    assign w_mstatus  = mstatus;
    assign w_mtime    = mtime;
    assign w_mtimecmp = mtimecmp;
    assign w_mip      = mip;

    /***********************************    Pipeline Control    ***********************************/

    // IfId
    assign IfId_flush = tkn | w_take_interrupt | w_take_exception | w_csr_flush;
    // IdEx
    assign IdEx_stall = ExMem_stall;
    assign IdEx_flush = tkn | w_take_exception | w_take_interrupt | (IfId_stall & !IdEx_stall & IfId_pending_exception == ~0) | w_csr_flush;
    // ExMem
    assign ExMem_flush = w_take_exception | w_take_interrupt | w_csr_flush;
    // MemWb
    assign MemWb_stall = ExMem_stall;
    assign MemWb_flush = w_take_exception | w_take_interrupt | w_csr_flush;


    /**********************************************************************************************/
    initial r_halt = 0;

    assign w_core_pc = pc;
    assign w_core_ir = IfId_ir;
    assign w_core_odata = inst_cnt;
endmodule
/**************************************************************************************************/

// forwarding unit
module forward_unit(
    input  wire [4:0] rs,
    input  wire [4:0] rd,
    input  wire regwrite,
    output wire forward
    );

    assign forward = (rs == rd) & (rs != 0) & regwrite;
endmodule

/**************************************************************************************************/
/* 32bit-32cycle divider (signed or unsigned)                                                     */
/**************************************************************************************************/
module m_div_unit(CLK, RST_X, w_init, w_signed, w_dividend, w_divisor, w_rslt, w_busy);
    input  wire         CLK, RST_X;
    input  wire         w_init, w_signed;
    input  wire [31:0]  w_dividend, w_divisor;
    output wire [63:0]  w_rslt;
    output wire         w_busy;

    reg         r_sign_dividend=0, r_sign_divisor=0;

    wire [31:0] w_uint_dividend, w_uint_divisor;
    wire [63:0] w_uint_rslt;

    m_div_unit_core divcore(CLK, RST_X, w_init, w_uint_dividend,
                            w_uint_divisor, w_uint_rslt, w_busy);

    assign w_uint_dividend  = (w_signed & w_dividend[31]) ? ~w_dividend + 1 : w_dividend;
    assign w_uint_divisor   = (w_signed & w_divisor[31])  ? ~w_divisor  + 1 : w_divisor;
    assign w_rslt[63:32]    =
        (~w_signed)?                   w_uint_rslt[63:32]     :
        ({r_sign_dividend, r_sign_divisor} == 2'b00) ?  w_uint_rslt[63:32]     :
        ({r_sign_dividend, r_sign_divisor} == 2'b01) ?  w_uint_rslt[63:32]     :
        ({r_sign_dividend, r_sign_divisor} == 2'b10) ? ~w_uint_rslt[63:32] + 1 :
                                                       ~w_uint_rslt[63:32] + 1;
    assign w_rslt[31: 0]    =
        (~w_signed)?                     w_uint_rslt[31: 0]   :
        ({r_sign_dividend, r_sign_divisor} == 2'b00) ?  w_uint_rslt[31: 0]     :
        ({r_sign_dividend, r_sign_divisor} == 2'b01) ? ~w_uint_rslt[31: 0] + 1 :
        ({r_sign_dividend, r_sign_divisor} == 2'b10) ? ~w_uint_rslt[31: 0] + 1 :
                                                        w_uint_rslt[31: 0];

    always @(posedge CLK) begin
      if(!RST_X) begin
          r_sign_dividend   <= 0;
          r_sign_divisor    <= 0;
      end else begin
          r_sign_dividend   <= (w_init) ? w_dividend[31] : r_sign_dividend;
          r_sign_divisor    <= (w_init) ? w_divisor[31] : r_sign_divisor;
      end
    end
endmodule

/**************************************************************************************************/
module m_div_unit_core(CLK, RST_X, w_init, w_dividend, w_divisor, r_rslt, w_busy);
    input  wire        CLK, RST_X;
    input  wire        w_init;
    input  wire [31:0] w_dividend, w_divisor;
    output reg  [63:0] r_rslt;
    output wire        w_busy;

    reg  [31:0] r_divisor=0;
    reg   [5:0] r_count=0;

    wire [32:0] w_differ;

    reg    r_busy   = 0;
    assign w_busy   = r_busy;
    assign w_differ = r_rslt[63:31] - {1'b0, r_divisor};

    always @(posedge CLK) begin
        if(!RST_X) begin
            r_divisor   <= 0;
            r_rslt      <= 0;
            r_count     <= 0;
            r_busy      <= 0;
        end else if(w_init) begin
            r_divisor   <= w_divisor;
            r_rslt      <= {32'h0, w_dividend};
            r_count     <= 0;
            r_busy      <= 1;
        end else if (r_count < 32) begin
            r_divisor   <= r_divisor;
            r_rslt      <= (w_differ[32]) ? {r_rslt[62:0], 1'h0} :
                                            {w_differ[31:0], r_rslt[30:0], 1'h1};
            r_count     <= r_count + 1;
            if (r_count == 31) begin
                r_busy <= 0;
            end
        end
    end
endmodule

/***** ALU module                                                                             *****/
/**************************************************************************************************/
module m_alu_im(CLK, RST_X, w_le, w_in1, w_in2, w_funct3, w_funct7, r_rslt, w_busy);
    input  wire         CLK, RST_X;
    input  wire         w_le;
    input  wire [31:0]  w_in1, w_in2;
    input  wire  [2:0]  w_funct3;
    input  wire  [6:0]  w_funct7;
    output reg  [31:0]  r_rslt;
    output wire         w_busy;

    reg  r_div_first = 1;

    wire signed [31:0] w_sin1 = w_in1;
    wire signed [31:0] w_sin2 = w_in2;

    wire  [4:0] w_shamt  = w_in2[4:0];

    reg  [31:0] r_in1    = 0;
    reg  [31:0] r_sin1   = 0;
    reg  [31:0] r_in2    = 0;
    reg  [31:0] r_sin2   = 0;
    wire [63:0] w_mul_SS = {{32{r_sin1[31]}},r_sin1} * {{32{r_sin2[31]}},r_sin2};
    wire [63:0] w_mul_SU = {{32{r_sin1[31]}},r_sin1} * {32'b0, r_in2};
    wire [63:0] w_mul_UU = {32'b0, r_in1} * {32'b0, r_in2};
    reg  [63:0] r_mul_SS = 0;
    reg  [63:0] r_mul_SU = 0;
    reg  [63:0] r_mul_UU = 0;
    reg         r_mul_state = 0;
    reg         r_le = 0;
    wire w_mul_req          =   (w_funct7[0] &&
                                (w_funct3 == `FUNCT3_MUL___ || w_funct3 == `FUNCT3_MULH__ ||
                                 w_funct3 == `FUNCT3_MULHU_ || w_funct3 == `FUNCT3_MULHSU));

    reg  r_mul_busy = 0;

    always @(posedge CLK) begin
        if (!RST_X) begin
            r_mul_state <= 0;
            r_mul_busy  <= 0;
        end
        else if (r_mul_state == 0 & w_mul_req & w_le) begin
            r_in1  <= w_in1;
            r_sin1 <= w_sin1;
            r_in2  <= w_in2;
            r_sin2 <= w_sin2;
            r_mul_state <= 1;
            r_mul_busy  <= 1;
        end else if (r_mul_state == 1) begin
            r_mul_SS   <= w_mul_SS;
            r_mul_SU   <= w_mul_SU;
            r_mul_UU   <= w_mul_UU;
            r_mul_state <= 0;
            r_mul_busy  <= 0;
        end
    end

    wire        w_divunit_busy;
    wire [63:0] w_divunit_rslt;

    wire w_div_req          =   (w_funct7[0] &&
                                (w_funct3 == `FUNCT3_DIV___ || w_funct3 == `FUNCT3_DIVU__ ||
                                 w_funct3 == `FUNCT3_REM___ || w_funct3 == `FUNCT3_REMU__));
    wire w_div_init         =   w_div_req && w_le && !w_divunit_busy;
    wire w_div_signed       =   (w_funct3 == `FUNCT3_DIV___ || w_funct3 == `FUNCT3_REM___);

    m_div_unit divunit(CLK, RST_X, w_div_init,
                        w_div_signed, w_in1, w_in2, w_divunit_rslt, w_divunit_busy);
    assign w_busy = w_divunit_busy | r_mul_busy;

    wire [63:0] w_div_rslt =    (w_in2 == 0) ? {w_in1, 32'hffffffff} :
                                (w_in2==32'hffffffff &&
                                (w_funct3 == `FUNCT3_DIV___ || w_funct3 == `FUNCT3_REM___))
                                ? {32'h0, w_in1} : w_divunit_rslt;

    // For Div unit
    always@(posedge CLK) begin
        if(w_div_init) begin
            r_div_first <= 0;
        end
        else if(!w_le) begin
            r_div_first <= 1;
        end
        if(w_le && w_div_req && !w_busy) begin
            if(w_funct3 == `FUNCT3_DIV___ || w_funct3 == `FUNCT3_DIVU__) begin
                if(r_rslt == w_div_rslt[31:0]) begin
                end
                else begin
                    $write("CAUTION! DIV FAIL! %x/%x true:%x unit:%x %x\n",
                            w_in1, w_in2, r_rslt, w_div_rslt[63:32], w_div_rslt[31:0]);
                    $finish();
                end
            end
            if(w_funct3 == `FUNCT3_REM___ || w_funct3 == `FUNCT3_REMU__) begin
                if(r_rslt == w_div_rslt[63:32]) begin
                end
                else begin
                    $write("CAUTION! REM FAIL! %x/%x true:%x unit:%x %x\n",
                            w_in1, w_in2, r_rslt, w_div_rslt[63:32], w_div_rslt[31:0]);
                    $finish();
                end
            end
        end
    end

    always@(*) begin
        if(!w_funct7[0]) begin
            case (w_funct3)
                `FUNCT3_ADD___ : r_rslt = (w_funct7) ? w_in1 - w_in2 : w_in1 + w_in2;
                `FUNCT3_SLL___ : r_rslt = w_in1 << w_shamt;
                `FUNCT3_SLT___ : r_rslt = {31'b0, w_sin1 < w_sin2};
                `FUNCT3_SLTU__ : r_rslt = {31'b0, w_in1 < w_in2};
                `FUNCT3_XOR___ : r_rslt = w_in1 ^ w_in2;
                `FUNCT3_SRL___ : begin
                                    if(w_funct7[5]) r_rslt = w_sin1 >>> w_shamt;
                                    else            r_rslt = w_in1 >> w_shamt;
                                end
                `FUNCT3_OR____ : r_rslt = w_in1 | w_in2;
                `FUNCT3_AND___ : r_rslt = w_in1 & w_in2;
                default        : r_rslt = 0;
            endcase
        end else begin
            case (w_funct3)
                `FUNCT3_MUL___ : r_rslt = r_mul_SS[31:0];
                `FUNCT3_MULH__ : r_rslt = r_mul_SS[63:32];
                `FUNCT3_MULHSU : r_rslt = r_mul_SU[63:32];
                `FUNCT3_MULHU_ : r_rslt = r_mul_UU[63:32];

                `FUNCT3_DIV___ : begin
                                    if(w_in2==32'hffffffff) r_rslt = w_in1;
                                    else if(w_in2==0)       r_rslt = 32'hffffffff;
                                    else                    r_rslt = w_div_rslt[31:0];
                                end
                `FUNCT3_DIVU__ : begin
                                    if(w_in2==0)    r_rslt = 32'hffffffff;
                                    else            r_rslt = w_div_rslt[31:0];
                                end
                `FUNCT3_REM___ : begin
                                    if(w_in2==32'hffffffff) r_rslt = 0;
                                    else if(w_in2==0)       r_rslt = w_in1;
                                    else                    r_rslt = w_div_rslt[63:32];
                                end
                `FUNCT3_REMU__ : begin
                                    if(w_in2==0)    r_rslt = w_in1;
                                    else            r_rslt = w_div_rslt[63:32];
                                end
                default        : r_rslt = 0;
            endcase
        end
    end
endmodule

/***** ALU for Branch                                                                         *****/
/**************************************************************************************************/
module m_alu_b(w_in1, w_in2, w_funct3, r_rslt);
    input wire [31:0]  w_in1, w_in2;
    input wire  [2:0]  w_funct3;
    output reg         r_rslt;

    wire signed [31:0] w_sin1 = w_in1;
    wire signed [31:0] w_sin2 = w_in2;

    always@(*) begin
        case(w_funct3)
            `FUNCT3_BEQ___ : r_rslt = w_in1 == w_in2;
            `FUNCT3_BNE___ : r_rslt = w_in1 != w_in2;
            `FUNCT3_BLT___ : r_rslt = w_sin1 < w_sin2;
            `FUNCT3_BGE___ : r_rslt = w_sin1 >= w_sin2;
            `FUNCT3_BLTU__ : r_rslt = w_in1 < w_in2;
            `FUNCT3_BGEU__ : r_rslt = w_in1 >= w_in2;
            default        : r_rslt = 0;
        endcase
    end
endmodule

/***** ALU for Atomic                                                                         *****/
/**************************************************************************************************/
module m_alu_a(w_in1, w_in2, w_funct5, r_rslt);
    input wire [31:0]  w_in1, w_in2;
    input wire  [4:0]  w_funct5;
    output reg [31:0]  r_rslt;

    wire signed [31:0] w_sin1 = w_in1;
    wire signed [31:0] w_sin2 = w_in2;

    always@(*) begin
        case(w_funct5)
            `FUNCT5_LR______ : r_rslt = 0;
            `FUNCT5_SC______ : r_rslt = w_in1;
            `FUNCT5_AMO_SWAP : r_rslt = w_in1;
            `FUNCT5_AMO_ADD_ : r_rslt = w_in1 + w_in2;
            `FUNCT5_AMO_AND_ : r_rslt = w_in1 & w_in2;
            `FUNCT5_AMO_OR__ : r_rslt = w_in1 | w_in2;
            `FUNCT5_AMO_XOR_ : r_rslt = w_in1 ^ w_in2;
            `FUNCT5_AMO_MIN_ : r_rslt = (w_sin1 < w_sin2) ? w_in1 : w_in2;
            `FUNCT5_AMO_MINU : r_rslt = (w_in1 < w_in2) ? w_in1 : w_in2;
            `FUNCT5_AMO_MAX_ : r_rslt = (w_sin1 > w_sin2) ? w_in1 : w_in2;
            `FUNCT5_AMO_MAXU : r_rslt = (w_in1 > w_in2) ? w_in1 : w_in2;
            default          : r_rslt = 0;
        endcase
    end
endmodule

/***** ALU for CSR                                                                            *****/
/**************************************************************************************************/
module m_alu_c(w_rcsr, w_rrs1, w_imm, w_funct3, r_rslt);
    input wire [31:0] w_rcsr, w_rrs1, w_imm;
    input wire  [2:0] w_funct3;
    output reg [31:0] r_rslt;

    always@(*) begin
        case(w_funct3)
            `FUNCT3_CSRRW_ : r_rslt = w_rrs1;
            `FUNCT3_CSRRS_ : r_rslt = w_rcsr | w_rrs1;
            `FUNCT3_CSRRC_ : r_rslt = w_rcsr & (~w_rrs1);
            `FUNCT3_CSRRWI : r_rslt = w_imm;
            `FUNCT3_CSRRSI : r_rslt = w_rcsr | w_imm;
            `FUNCT3_CSRRCI : r_rslt = w_rcsr & (~w_imm);
            default        : r_rslt = 0;
        endcase
    end
endmodule

/***** Imm generateor                                                                         *****/
/**************************************************************************************************/
module m_imm_gen(w_inst, r_imm);
    input  wire [31:0]  w_inst;
    output reg  [31:0]  r_imm;

    wire [6:0] opcode = w_inst[6:0];
    wire [31:0] imm_I = { {21{w_inst[31]}}, w_inst[30:25], w_inst[24:20] };
    wire [31:0] imm_S = { {21{w_inst[31]}}, w_inst[30:25], w_inst[11:8], w_inst[7] };
    wire [31:0] imm_B = { {20{w_inst[31]}}, w_inst[7],w_inst[30:25] ,w_inst[11:8], 1'b0 };
    wire [31:0] imm_U = { w_inst[31:12], 12'b0 };
    wire [31:0] imm_J = { {12{w_inst[31]}}, w_inst[19:12], w_inst[20], w_inst[30:25], w_inst[24:21], 1'b0 };
    wire [31:0] imm_C = { 27'b0, w_inst[19:15] };

    always @(*) begin
        case (opcode)
            `OPCODE_OP_IMM__ : r_imm = imm_I;
            `OPCODE_STORE___ : r_imm = imm_S;
            `OPCODE_LOAD____ : r_imm = imm_I;
            `OPCODE_LUI_____ : r_imm = imm_U;
            `OPCODE_BRANCH__ : r_imm = imm_B;
            `OPCODE_AUIPC___ : r_imm = imm_U;
            `OPCODE_JAL_____ : r_imm = imm_J;
            `OPCODE_JALR____ : r_imm = imm_I;
            `OPCODE_SYSTEM__ : r_imm = imm_C;
            default          : r_imm = 0;
        endcase
    end
endmodule
/**************************************************************************************************/

/***** ALU for I                                                                              *****/
/**************************************************************************************************/
module m_alu_i (w_in1, w_in2, w_funct3, w_funct7, r_rslt);
    input wire [31:0]   w_in1, w_in2;
    input wire  [2:0]   w_funct3;
    input wire  [6:0]   w_funct7;
    output reg [31:0]   r_rslt;

    wire signed [31:0] w_sin1 = w_in1;
    wire signed [31:0] w_sin2 = w_in2;

    wire [4:0] w_shamt = w_in2[4:0];

    always@(*) begin
        case (w_funct3)
            `FUNCT3_ADD___ : r_rslt = (w_funct7) ? w_in1 - w_in2 : w_in1 + w_in2;
            `FUNCT3_SLL___ : r_rslt = w_in1 << w_shamt;
            `FUNCT3_SLT___ : r_rslt = {31'b0, w_sin1 < w_sin2};
            `FUNCT3_SLTU__ : r_rslt = {31'b0, w_in1 < w_in2};
            `FUNCT3_XOR___ : r_rslt = w_in1 ^ w_in2;
            `FUNCT3_SRL___ : begin
                                if(w_funct7[5]) r_rslt = w_sin1 >>> w_shamt;
                                else            r_rslt = w_in1   >> w_shamt;
                            end
            `FUNCT3_OR____ : r_rslt = w_in1 | w_in2;
            `FUNCT3_AND___ : r_rslt = w_in1 & w_in2;
            default        : begin
                $write("ILLEGAL INSTRUCTION! in alu_i\n");
                r_rslt = 0;
                $finish();
            end
        endcase
    end
endmodule

/**************************************************************************************************/
module m_decomp(w_ic, r_iw);
    input  wire [31:0] w_ic;
    output reg  [31:0] r_iw;

    // C0
    wire [ 4:0] w_c0_rs1   ={2'b01, w_ic[9:7]};
    wire [ 4:0] w_c0_rs2   ={2'b01, w_ic[4:2]};
    wire [ 4:0] w_c0_rd    ={2'b01, w_ic[4:2]};
    wire [31:0] w_c0_uimm1 ={25'b0, w_ic[5], w_ic[12:10], w_ic[6], 2'b0};
    wire [31:0] w_c0_uimm2 ={24'b0, w_ic[6:5], w_ic[12:10], 3'b0};
    wire [31:0] w_c0_nzuimm={22'b0, w_ic[10:7], w_ic[12:11], w_ic[5], w_ic[6], 2'b0};

    // C1
    wire [ 1:0] w_c1_funct1=w_ic[11:10];
    wire [ 2:0] w_c1_funct2={w_ic[12], w_ic[6:5]};
    wire [ 4:0] w_c1_rs1   ={2'b01, w_ic[9:7]};
    wire [ 4:0] w_c1_rs2   ={2'b01, w_ic[4:2]};
    wire [ 4:0] w_c1_rd    ={2'b01, w_ic[9:7]};
    wire [31:0] w_c1_nzimm ={{27{w_ic[12]}}, w_ic[6:2]};
    wire [ 4:0] w_c1_shamt =w_c1_nzimm[4:0];
    wire [31:0] w_c1_imm1  ={{21{w_ic[12]}}, w_ic[8], w_ic[10:9], w_ic[6], w_ic[7], w_ic[2], w_ic[11], w_ic[5:3],1'b0};
    wire [31:0] w_c1_imm2  ={{25{w_ic[12]}}, w_ic[6:5], w_ic[2], w_ic[11:10], w_ic[4:3], 1'b0};
    wire [31:0] w_c1_imm3  ={{27{w_ic[12]}}, w_ic[6:2]};
    wire [31:0] w_c1_imm4  ={{23{w_ic[12]}}, w_ic[4:3], w_ic[5], w_ic[2], w_ic[6], 4'b0};

    // C2
    wire [ 4:0] w_c2_rs2   =w_ic[6:2];
    wire [ 4:0] w_c2_rd    =w_ic[11:7];
    wire [31:0] w_c2_uimm1 ={23'b0, w_ic[4:2], w_ic[12], w_ic[6:5], 3'b0};
    wire [31:0] w_c2_uimm2 ={24'b0, w_ic[3:2], w_ic[12], w_ic[6:4], 2'b0};
    wire [31:0] w_c2_uimm3 ={22'b0, w_ic[9:7], w_ic[12:10], 3'b0};
    wire [31:0] w_c2_uimm4 ={23'b0, w_ic[8:7], w_ic[12:9], 2'b0};
    wire [31:0] w_c2_nzuimm={26'b0, w_ic[12], w_ic[6:2]};
    wire [ 4:0] w_c2_shamt =w_c2_nzuimm[4:0];

    always @(*) begin
        case ({w_ic[1:0], w_ic[15:13]})
            // C0
            {2'b00, 3'b000}: r_iw={w_c0_nzuimm[11:0],  5'd2, `FUNCT3_ADD___, w_c0_rd, `OPCODE_OP_IMM__};                      // C.ADDI4SPN
            {2'b00, 3'b001}: r_iw={w_c0_uimm2[11:0], w_c0_rs1, `FUNCT3_LD____, w_c0_rd, `OPCODE_LOAD_FP_};                    // C.FLD
            {2'b00, 3'b010}: r_iw={w_c0_uimm1[11:0], w_c0_rs1, `FUNCT3_LW____, w_c0_rd, `OPCODE_LOAD____};                    // C.LW
            {2'b00, 3'b011}: r_iw={w_c0_uimm1[11:0], w_c0_rs1, `FUNCT3_LW____, w_c0_rd, `OPCODE_LOAD_FP_};                    // C.FLW
            {2'b00, 3'b101}: r_iw={w_c0_uimm1[11:5], w_c0_rs2, w_c0_rs1, `FUNCT3_SD____, w_c0_uimm1[4:0], `OPCODE_STORE_FP};  // C.FSD
            {2'b00, 3'b110}: r_iw={w_c0_uimm1[11:5], w_c0_rs2, w_c0_rs1, `FUNCT3_SW____, w_c0_uimm1[4:0], `OPCODE_STORE___};  // C.SW
            {2'b00, 3'b111}: r_iw={w_c0_uimm1[11:5], w_c0_rs2, w_c0_rs1, `FUNCT3_SW____, w_c0_uimm1[4:0], `OPCODE_STORE_FP};  // C.FSW
            // C1
            {2'b01, 3'b000}: r_iw={w_c1_nzimm[11:0], w_ic[11:7], `FUNCT3_ADD___, w_ic[11:7], `OPCODE_OP_IMM__};               // C.ADDI
            {2'b01, 3'b001}: r_iw={w_c1_imm1[20], w_c1_imm1[10:1], w_c1_imm1[11], w_c1_imm1[19:12], 5'd1, `OPCODE_JAL_____};  // C.JAL
            {2'b01, 3'b010}: r_iw={w_c1_imm3[11:0], 5'd0, `FUNCT3_ADD___, w_ic[11:7], `OPCODE_OP_IMM__};                      // C.LI
            {2'b01, 3'b011}:
              begin
                  if (w_ic[11:7] == 2) r_iw={w_c1_imm4[11:0], 5'd2, `FUNCT3_ADD___, 5'd2, `OPCODE_OP_IMM__};                    // C.ADDI16SP
                  else               r_iw={w_c1_nzimm[19:0], w_ic[11:7], `OPCODE_LUI_____};                                     // C.LUI
              end
            {2'b01, 3'b101}: r_iw={w_c1_imm1[20], w_c1_imm1[10:1], w_c1_imm1[11], w_c1_imm1[19:12], 5'd0, `OPCODE_JAL_____};  // C.J
            {2'b01, 3'b110}: r_iw={w_c1_imm2[12], w_c1_imm2[10:5], 5'd0, w_c1_rs1, `FUNCT3_BEQ___, w_c1_imm2[4:1], w_c1_imm2[11], `OPCODE_BRANCH__}; // C.BEQZ
            {2'b01, 3'b111}: r_iw={w_c1_imm2[12], w_c1_imm2[10:5], 5'd0, w_c1_rs1, `FUNCT3_BNE___, w_c1_imm2[4:1], w_c1_imm2[11], `OPCODE_BRANCH__}; // C.BNEZ
            {2'b01, 3'b100}:
              begin
                  case (w_c1_funct1)
                      2'd0: r_iw={w_c1_nzimm[11:0], w_c1_rd, `FUNCT3_SRL___, w_c1_rd, `OPCODE_OP_IMM__};                        // C.SRLI
                      2'd1: r_iw={2'b01, w_c1_nzimm[9:0], w_c1_rd, `FUNCT3_SRL___, w_c1_rd, `OPCODE_OP_IMM__};                  // C.SRAI
                      2'd2: r_iw={w_c1_nzimm[11:0], w_c1_rd, `FUNCT3_AND___, w_c1_rd, `OPCODE_OP_IMM__};                        // C.ANDI
                      2'd3:
                        begin
                            case (w_c1_funct2)
                                3'd0: r_iw={7'b0100000, w_c1_rs2, w_c1_rd, `FUNCT3_ADD___, w_c1_rd, `OPCODE_OP______};            // C.SUB
                                3'd1: r_iw={7'b0, w_c1_rs2, w_c1_rd, `FUNCT3_XOR___, w_c1_rd, `OPCODE_OP______};                  // C.XOR
                                3'd2: r_iw={7'b0, w_c1_rs2, w_c1_rd, `FUNCT3_OR____, w_c1_rd, `OPCODE_OP______};                  // C.OR
                                3'd3: r_iw={7'b0, w_c1_rs2, w_c1_rd, `FUNCT3_AND___, w_c1_rd, `OPCODE_OP______};                  // C.AND
                                default: r_iw=`ILLEGAL_INST;
                            endcase
                        end
                  endcase
              end
            // C2
            {2'b10, 3'b000}: r_iw={7'b0, w_c2_shamt, w_c2_rd, `FUNCT3_SLL___, w_c2_rd, `OPCODE_OP_IMM__};                     // C.SLLI
            {2'b10, 3'b001}: r_iw={w_c2_uimm1[11:0],  5'd2, `FUNCT3_LD____, w_c2_rd, `OPCODE_LOAD_FP_};                       // C.FLDSP
            {2'b10, 3'b010}: r_iw={w_c2_uimm2[11:0],  5'd2, `FUNCT3_LW____, w_c2_rd, `OPCODE_LOAD____};                       // C.LWSP
            {2'b10, 3'b011}: r_iw={w_c2_uimm2[11:0],  5'd2, `FUNCT3_LW____, w_c2_rd, `OPCODE_LOAD_FP_};                       // C.FLWSP
            {2'b10, 3'b101}: r_iw={w_c2_uimm3[11:5], w_c2_rs2, 5'd2, `FUNCT3_SD____, w_c2_uimm3[4:0], `OPCODE_STORE_FP};      // C.FSDSP
            {2'b10, 3'b110}: r_iw={w_c2_uimm4[11:5], w_c2_rs2, 5'd2, `FUNCT3_SW____, w_c2_uimm4[4:0], `OPCODE_STORE___};      // C.SWSP
            {2'b10, 3'b111}: r_iw={w_c2_uimm4[11:5], w_c2_rs2, 5'd2, `FUNCT3_SW____, w_c2_uimm4[4:0], `OPCODE_STORE_FP};      // C.FSWSP
            {2'b10, 3'b100}:
              /*              begin
               if (w_ic[12]) begin
               if (w_c2_rd == 0 && w_c2_rs2 == 0) r_iw={`FUNCT12_EBREAK, 13'b0, `OPCODE_SYSTEM__};                       // C.EBREAK
               if (w_c2_rd != 0 && w_c2_rs2 == 0) r_iw={12'b0, w_c2_rd, 3'b0, 5'd1, `OPCODE_JALR____};                   // C.JALR
               if (w_c2_rd != 0 && w_c2_rs2 != 0) r_iw={7'b0, w_c2_rs2, w_c2_rd, `FUNCT3_ADD___, w_c2_rd, `OPCODE_OP______};  // C.ADD
                end else begin
               if (w_c2_rd != 0 && w_c2_rs2 == 0) r_iw={12'b0, w_c2_rd, 8'b0, `OPCODE_JALR____};                         // C.JR
               if (w_c2_rd != 0 && w_c2_rs2 != 0) r_iw={7'b0, w_c2_rs2, 5'd0, `FUNCT3_ADD___, w_c2_rd, `OPCODE_OP______};// C.MV
                end
            end*/
              if (w_c2_rs2 == 0) begin
                  if (w_c2_rd == 0) r_iw={`FUNCT12_EBREAK, 13'b0, `OPCODE_SYSTEM__}; // C.EBREAK
                  else if (w_ic[12]) r_iw={12'b0, w_c2_rd, 3'b0, 5'd1, `OPCODE_JALR____}; // C.JALR
                  else r_iw={12'b0, w_c2_rd, 8'b0, `OPCODE_JALR____}; // C.JR
              end
              else begin
                  if(w_ic[12]) r_iw={7'b0, w_c2_rs2, w_c2_rd, `FUNCT3_ADD___, w_c2_rd, `OPCODE_OP______};  // C.ADD
                  else r_iw={7'b0, w_c2_rs2, 5'd0, `FUNCT3_ADD___, w_c2_rd, `OPCODE_OP______};// C.MV
              end

            // 32bit
            {2'b11, 3'b000}: r_iw=w_ic;
            {2'b11, 3'b001}: r_iw=w_ic;
            {2'b11, 3'b010}: r_iw=w_ic;
            {2'b11, 3'b011}: r_iw=w_ic;
            {2'b11, 3'b100}: r_iw=w_ic;
            {2'b11, 3'b101}: r_iw=w_ic;
            {2'b11, 3'b110}: r_iw=w_ic;
            {2'b11, 3'b111}: r_iw=w_ic;
        endcase
    end
endmodule
/**************************************************************************************************/
