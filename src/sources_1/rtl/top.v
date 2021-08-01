/**************************************************************************************************/
/**** RVSoC (Mini Kuroda/RISC-V)                       since 2018-08-07   ArchLab. TokyoTech   ****/
/**** top module and simulation module v0.01                                                   ****/
/**************************************************************************************************/
`default_nettype none
/**************************************************************************************************/
`include "define.vh"

/**************************************************************************************************/
module m_topsim();
    reg CLK = 0;
    reg RST_X;

    initial forever #50 CLK = ~CLK;

    initial begin
        #50  RST_X = 0;
        #300 RST_X = 1;
    end

    wire w_halt;

    wire w_finish;

    wire [31:0] w_insn_data, w_insn_addr;
    wire [31:0] w_data_data, w_data_wdata, w_data_addr;
    wire        w_data_we;
    wire [2:0]  w_data_ctrl;

    wire [31:0] w_priv, w_satp, w_mstatus;
    wire [63:0] w_mtime, w_mtimecmp, w_wmtimecmp;
    wire        w_clint_we;
    wire [31:0] w_mip, w_wmip;
    wire        w_plic_we;
    wire        w_busy;
    wire [31:0] w_pagefault;
    wire [1:0]  w_tlb_req;
    wire        w_tlb_flush;
    wire        w_txd;
    wire        w_rxd;
    wire        w_init_done;
    wire        w_init_stage;

    wire        CORE_RST_X = RST_X & w_init_done;

    wire [15:0] ddr2_dq;
    wire [1:0]  ddr2_dqs_n;
    wire [1:0]  ddr2_dqs_p;
    wire [12:0] ddr2_addr;
    wire [2:0]  ddr2_ba;
    wire        ddr2_ras_n;
    wire        ddr2_cas_n;
    wire        ddr2_we_n;
    wire        ddr2_ck_p;
    wire        ddr2_ck_n;
    wire        ddr2_cke;
    wire        ddr2_cs_n;
    wire [1:0]  ddr2_dm;
    wire        ddr2_odt;

    wire [7:0]  w_uart_data;
    wire        w_uart_we;

    wire [15:0] w_led;

    /**********************************************************************************************/

    reg       r_uart_busy  = 0;
    reg [3:0] r_uart_count = 0;
    reg [3:0] r_uart_cycle = 0;
    reg [7:0] r_uart_data  = 0;

`ifdef USE_UART
    always@(posedge CLK) begin
        if (w_init_done) begin
            if (!r_uart_busy && !w_txd) begin
                r_uart_busy  <= 1;
                r_uart_data  <= 0;
                r_uart_count <= 0;
                r_uart_cycle <= 1;
            end else if (r_uart_busy && (r_uart_count == 8) && w_txd) begin
                r_uart_count <= 0;
                r_uart_busy  <= 0;
                r_uart_cycle <= 0;
                $write("%c", r_uart_data);
                $fflush();
            end else if (r_uart_busy && (r_uart_cycle == 8)) begin
                r_uart_count <= r_uart_count + 1;
                r_uart_cycle <= 1;
            end else if (r_uart_busy && (r_uart_cycle == 4) && (r_uart_count > 0)) begin
                r_uart_cycle <= r_uart_cycle + 1;
                r_uart_data  <= {w_txd,r_uart_data[7:1]};
            end else if (r_uart_busy) begin
                r_uart_cycle <= r_uart_cycle + 1;
            end
        end
    end
`else
    always@(posedge CLK) begin
        if(w_uart_we) begin $write("%c", w_uart_data); $fflush(); end
    end
`endif

    /**********************************************************************************************/
    m_mmu mmu(
        .CLK            (CLK),
        .RST_X          (RST_X),
        .w_insn_addr    (w_insn_addr),
        .w_data_addr    (w_data_addr),
        .w_data_wdata   (w_data_wdata),
        .w_data_we      (w_data_we),
        .w_data_ctrl    (w_data_ctrl),
        .w_insn_data    (w_insn_data),
        .w_data_data    (w_data_data),
        .r_finish       (w_finish),
        .w_priv         (w_priv),
        .w_satp         (w_satp),
        .w_mstatus      (w_mstatus),
        .w_mtime        (w_mtime),
        .w_mtimecmp     (w_mtimecmp),
        .w_wmtimecmp    (w_wmtimecmp),
        .w_clint_we     (w_clint_we),
        .w_mip          (w_mip),
        .w_wmip         (w_wmip),
        .w_plic_we      (w_plic_we),
        .w_proc_busy    (w_busy),
        .w_pagefault    (w_pagefault),
        .w_tlb_req      (w_tlb_req),
        .w_tlb_flush    (w_tlb_flush),
        .w_txd          (w_txd),
        .w_rxd          (w_rxd),
        .w_init_done    (w_init_done),
        // input clk, rst (active-low)
        .mig_clk        (1'b0),
        .mig_rst_x      (1'b0),
        // memory interface ports
        .ddr2_dq        (ddr2_dq),
        .ddr2_dqs_n     (ddr2_dqs_n),
        .ddr2_dqs_p     (ddr2_dqs_p),
        .ddr2_addr      (ddr2_addr),
        .ddr2_ba        (ddr2_ba),
        .ddr2_ras_n     (ddr2_ras_n),
        .ddr2_cas_n     (ddr2_cas_n),
        .ddr2_we_n      (ddr2_we_n),
        .ddr2_ck_p      (ddr2_ck_p),
        .ddr2_ck_n      (ddr2_ck_n),
        .ddr2_cke       (ddr2_cke),
        .ddr2_cs_n      (ddr2_cs_n),
        .ddr2_dm        (ddr2_dm),
        .ddr2_odt       (ddr2_odt),
        // output clk, rst (active-low)
        .o_clk          (),
        .o_rst_x        (),
        .w_uart_data    (w_uart_data),
        .w_uart_we      (w_uart_we),
        .w_led          (w_led),
        .w_init_stage   (w_init_stage),
        .w_checksum     (),
        .w_debug_btnd   (1'b0)
    );

    m_RVCoreM p(
        .CLK            (CLK),
        .RST_X          (CORE_RST_X),
        .w_stall        (1'b0),
        .r_halt         (w_halt),
        .w_insn_addr    (w_insn_addr),
        .w_data_addr    (w_data_addr),
        .w_insn_data    (w_insn_data),
        .w_data_data    (w_data_data),
        .w_data_wdata   (w_data_wdata),
        .w_data_we      (w_data_we),
        .w_data_ctrl    (w_data_ctrl),
        .w_priv         (w_priv),
        .w_satp         (w_satp),
        .w_mstatus      (w_mstatus),
        .w_mtime        (w_mtime),
        .w_mtimecmp     (w_mtimecmp),
        .w_wmtimecmp    (w_wmtimecmp),
        .w_clint_we     (w_clint_we),
        .w_mip          (w_mip),
        .w_wmip         (w_wmip),
        .w_plic_we      (w_plic_we),
        .w_busy         (w_busy),
        .w_pagefault    (w_pagefault),
        .w_tlb_req      (w_tlb_req),
        .w_tlb_flush    (w_tlb_flush),
        .w_core_pc      (),
        .w_core_ir      (),
        .w_core_odata   (),
        .w_init_stage   (w_init_stage)
    );

    /**********************************************************************************************/
    always@(posedge CLK) if (w_halt) begin $write("HALT detect! at PC:%x\n", p.pc); $finish(); end
    always@(posedge CLK) if (w_finish & !r_uart_busy) begin
        $write("FINISH!\n");
`ifdef DEBUG
        $fclose(fp);
`endif
        $finish();
    end

    // LOAD
    integer i,j;
    //integer k;
    reg  [7:0] mem_bbl [0:`BBL_SIZE-1];
    reg  [7:0] mem_mc[0:`MC_MEM_SIZE-1];
    reg  [7:0] mem_disk[0:`DISK_SIZE-1];
    initial begin
    #1
`ifdef LINUX
        /*$write("Load micro-ctrl program: %s\n", `MC_PROG);
        $readmemh(`MC_PROG, mem_mc);
        j=0;

        for(i=0;i<`MC_MEM_SIZE;i=i+4) begin
            mmu.mc.LCMEM.mem1.mem[j]=mem_mc[i];
            mmu.mc.LCMEM.mem2.mem[j]=mem_mc[i+1];
            mmu.mc.LCMEM.mem3.mem[j]=mem_mc[i+2];
            mmu.mc.LCMEM.mem4.mem[j]=mem_mc[i+3];
            j=j+1;
        end*/
        $write("Load image file: %s\n", `IMAGE_FILE);
        $readmemh(`IMAGE_FILE, mem_disk);
        //j=`BBL_SIZE/4;
        j=`BBL_SIZE;

        /*for(i=0;i<`DISK_SIZE;i=i+4) begin
            mmu.idbmem.idbmem.mem1.mem[j]=mem_disk[i];
            mmu.idbmem.idbmem.mem2.mem[j]=mem_disk[i+1];
            mmu.idbmem.idbmem.mem3.mem[j]=mem_disk[i+2];
            mmu.idbmem.idbmem.mem4.mem[j]=mem_disk[i+3];
            j=j+1;
        end*/
        for(i=0;i<`DISK_SIZE;i=i+1) begin
            mmu.idbmem.idbmem.mem[j]=mem_disk[i];
            j=j+1;
        end
`endif
        $write("Running %s\n", {`HEX_DIR,`HEXFILE});
        $readmemh({`HEX_DIR,`HEXFILE}, mem_bbl);
        j=0;

        /*for(i=0;i<`BBL_SIZE;i=i+4) begin
            mmu.idbmem.idbmem.mem1.mem[j]=mem_bbl[i];
            mmu.idbmem.idbmem.mem2.mem[j]=mem_bbl[i+1];
            mmu.idbmem.idbmem.mem3.mem[j]=mem_bbl[i+2];
            mmu.idbmem.idbmem.mem4.mem[j]=mem_bbl[i+3];
            j=j+1;
        end*/
        for(i=0;i<`BBL_SIZE;i=i+1) begin
            mmu.idbmem.idbmem.mem[j]=mem_bbl[i];
            j=j+1;
        end
        $write("-------------------------------------------------------------------\n");
    end
`ifdef DEBUG
    integer fp;
    initial begin
        fp=$fopen("log.txt","w");
        if(fp == 0)begin
            $display("File Open Error!!!!!");
            $finish();
        end
    end
`endif

    reg [31:0] r_cnt = 0;
    always@(posedge CLK) begin
        r_cnt <= r_cnt + 1;
        if(p.mtime > `TIMEOUT) begin
            $write("Simulation Time out...\n");
`ifdef MEM_DUMP
            $fclose(fp);
            fp=$fopen("final_mem.txt","w");
            if(fp == 0)begin
                $display("File Open Error!!!!! in final_mem!");
                $finish();
            end
            j=0;
            for(i=0;i<`BBL_SIZE;i=i+4) begin
                $fwrite(fp,"%x\n",mmu.idbmem.mem1.mem[j]);
                $fwrite(fp,"%x\n",mmu.idbmem.mem2.mem[j]);
                $fwrite(fp,"%x\n",mmu.idbmem.mem3.mem[j]);
                $fwrite(fp,"%x\n",mmu.idbmem.mem4.mem[j]);
                j=j+1;
            end
            $fclose(fp);
`endif
            $finish();
        end

`ifdef DEBUG
        // EXEC STATE
        if(p.state == `S_FIN && !p.w_busy) begin
            if(p.mtime % 1000000 == 0) begin
                $write("MTIME=%09d %d\n", p.mtime[31:0], p.mtimecmp);
                $fflush();
            end
`ifdef TRACE
            $fwrite(fp, "%08d %08x %08x\n", p.mtime[31:0], p.r_cpc, p.r_ir);
            // REGSTATE
            for(i=0;i<4;i=i+1) begin
                for(j=0;j<8;j=j+1) begin
                    $fwrite(fp, "%08x",p.regs.mem[i*8+j]);
                    if(j==7)    $fwrite(fp, "\n");
                    else        $fwrite(fp, " ");
                end
            end
            // CSR STATE
            #1
            $fwrite(fp, "%08x %08x %08x %08x %08x %08x %08x %08x \n", p.mstatus, p.mtvec, p.mscratch, p.mepc, p.mcause, p.mtval, p.mhartid, p.misa);
            $fwrite(fp, "%08x %08x %08x %08x %08x %08x %08x %08x \n", p.mie, p.mip, p.medeleg, p.mideleg, p.mcounteren, p.stvec, p.sscratch, p.sepc);
            $fwrite(fp, "%08x %08x %08x %08x %08x %08x %08x %08x \n", p.scause, p.stval, p.satp, p.scounteren, p.load_res, p.pending_exception, p.pending_tval, {30'h0,p.priv});
            
            //TLB
            for(i=0;i<4;i=i+1) begin
                $fwrite(fp, "%08x %08x ", (mmu.TLB_inst_r.r_valid[i]) ? {mmu.TLB_inst_r.mem[i][39:22], i[1:0], 12'b0} : 32'hffffffff, (mmu.TLB_inst_r.r_valid[i]) ? {mmu.TLB_inst_r.mem[i][21:0], 10'b0} : 32'hffffffff);
            end
            $fwrite(fp,"\n");
            for(i=0;i<4;i=i+1) begin
                $fwrite(fp, "%08x %08x ", (mmu.TLB_data_r.r_valid[i]) ? {mmu.TLB_data_r.mem[i][39:22], i[1:0], 12'b0} : 32'hffffffff, (mmu.TLB_data_r.r_valid[i]) ? {mmu.TLB_data_r.mem[i][21:0], 10'b0} : 32'hffffffff);
            end
            $fwrite(fp,"\n");
            for(i=0;i<4;i=i+1) begin
                $fwrite(fp, "%08x %08x ", (mmu.TLB_data_w.r_valid[i]) ? {mmu.TLB_data_w.mem[i][39:22], i[1:0], 12'b0} : 32'hffffffff, (mmu.TLB_data_w.r_valid[i]) ? {mmu.TLB_data_w.mem[i][21:0], 10'b0} : 32'hffffffff);
            end
            $fwrite(fp,"\n");
`endif
`ifdef PC_TRACE
            if(p.mtime % 1000 == 0) begin
                $fwrite(fp, "%08d %08x\n", (p.mtime[31:0]/1000), p.r_cpc);
                $fflush(fp);
            end
`endif

        end
`endif
    end
`ifdef MIDDLE
    initial begin
        #300
        `include `REGFILE
    end
`endif

    /**********************************************************************************************/

    wire init_txd, tx_ready;

    reg [7:0]   uartdata = 0;
    reg         uartwe   = 0;
    reg [16:0]  imemaddr = 0;

    always@(posedge CLK) begin
        if(r_cnt > 10 && tx_ready && !uartwe) begin
            uartdata <= mem_bbl[imemaddr];
            imemaddr <= imemaddr + 1;
            uartwe   <= 1;
        end else begin
            uartwe   <= 0;
            uartdata <= 0;
        end
    end

    UartTx UartTx_init(CLK, RST_X, uartdata, uartwe, init_txd, tx_ready);

    assign w_rxd = init_txd;

endmodule
/**************************************************************************************************/
/**** Byte unit BRAM Main Memory module with LATENCY for simulation (1-port)                   ****/
/**************************************************************************************************/
module m_bu_mem #(parameter MEM_SIZE = `MEM_SIZE)
            (CLK, w_addr, w_odata, w_we, w_le, w_wdata, w_ctrl, w_stall);
    input  wire             CLK;
    input  wire [`XLEN-1:0] w_addr;
    output wire     [127:0] w_odata;
    input  wire             w_we, w_le;
    input  wire [`XLEN-1:0] w_wdata;
    input  wire       [2:0] w_ctrl;
    output wire             w_stall;

    reg   [2:0] r_ctrl  = 3;
    reg  [31:0] r_addr  = 0;
    reg  [31:0] r_cnt   = 0;
    reg         r_le    = 0;

    reg   [7:0] mem [0:MEM_SIZE-1];
    reg [127:0] r_odata = 0;

    wire [`XLEN-1:0] w_maddr =  (`LATENCY==0) ? w_addr : 
                                (w_we) ? w_addr : (r_cnt == `LATENCY) ? r_addr  : 0;
    wire       [2:0] w_mctrl =  (`LATENCY==0) ? w_ctrl :
                                (w_we) ? w_ctrl : (r_cnt == `LATENCY) ? r_ctrl  : 0;

    assign w_stall = (r_cnt != 0);

    always@(posedge CLK) begin
        if(w_le && r_cnt == 0) begin
            r_addr <= w_addr;
            r_ctrl <= w_ctrl;
            r_le   <= w_le;
            if(`LATENCY != 0) begin
                r_cnt  <= 1;
            end
        end
        else if(r_cnt == `LATENCY) begin
            r_cnt = 0;
        end
        else if(r_cnt != 0) begin
            r_cnt = r_cnt + 1;
        end
    end

    // Select Write DATA
    wire [7:0] w_data1, w_data2, w_data3, w_data4;
    assign {w_data4, w_data3, w_data2, w_data1} =   (w_mctrl[1:0] == 0) ? {4{w_wdata[7:0]}} :
                                                    (w_mctrl[1:0] == 1) ? {2{w_wdata[15:0]}} : w_wdata;

    wire [7:0] o_data1, o_data2, o_data3, o_data4;
    //assign w_odata = {o_data4, o_data3, o_data2, o_data1};
    assign w_odata = r_odata;

    // Check Write Enable
    wire WE1 = w_we && ((w_mctrl[1:0]==2)   || (w_mctrl[1:0]==1 && w_maddr[1] == 0)
                                            || (w_mctrl[1:0]==0 && w_maddr[1:0]==0));
    wire WE2 = w_we && ((w_mctrl[1:0]==2)   || (w_mctrl[1:0]==1 && w_maddr[1] == 0)
                                            || (w_mctrl[1:0]==0 && w_maddr[1:0]==1));
    wire WE3 = w_we && ((w_mctrl[1:0]==2)   || (w_mctrl[1:0]==1 && w_maddr[1] == 1)
                                            || (w_mctrl[1:0]==0 && w_maddr[1:0]==2));
    wire WE4 = w_we && ((w_mctrl[1:0]==2)   || (w_mctrl[1:0]==1 && w_maddr[1] == 1)
                                            || (w_mctrl[1:0]==0 && w_maddr[1:0]==3));

    wire [31:0] w_maddr_tw = {w_maddr[31:2], 2'b0};
    wire [31:0] w_maddr_tr = {w_maddr[31:4], 4'b0};


    always@(posedge CLK) begin
        if(WE1) mem[w_maddr_tw  ] <= w_data1;
        if(WE2) mem[w_maddr_tw+1] <= w_data2;
        if(WE3) mem[w_maddr_tw+2] <= w_data3;
        if(WE4) mem[w_maddr_tw+3] <= w_data4;
        r_odata <= {mem[w_maddr_tr+15], mem[w_maddr_tr+14], mem[w_maddr_tr+13], mem[w_maddr_tr+12],
                    mem[w_maddr_tr+11], mem[w_maddr_tr+10], mem[w_maddr_tr+ 9], mem[w_maddr_tr+ 8],
                    mem[w_maddr_tr+ 7], mem[w_maddr_tr+ 6], mem[w_maddr_tr+ 5], mem[w_maddr_tr+ 4],
                    mem[w_maddr_tr+ 3], mem[w_maddr_tr+ 2], mem[w_maddr_tr+ 1], mem[w_maddr_tr   ]};
    end

endmodule
/**************************************************************************************************/

/**** BRAM Wrapper for simulation (1-port)                                                     ****/
/**************************************************************************************************/
module m_dram_sim#(parameter MEM_SIZE = `MEM_SIZE)
            (CLK, w_addr, w_odata, w_we, w_le, w_wdata, w_ctrl, w_stall, w_mtime);
    input  wire             CLK;
    input  wire [`XLEN-1:0] w_addr;
    output wire [`XLEN-1:0] w_odata;
    input  wire             w_we, w_le;
    input  wire [`XLEN-1:0] w_wdata;
    input  wire       [2:0] w_ctrl;
    output wire             w_stall;
    input  wire      [31:0] w_mtime;

    reg   [1:0] r_cache_state = 0;

    reg  [31:0] r_addr = 0;
    reg   [2:0] r_ctrl = 0;
    reg [127:0] r_odata = 0;

    // evaluation
    reg         e_test;
    reg  [31:0] e_data;
    reg  [63:0] e_cnt = 0;
    reg  [63:0] e_hit = 0;

    // DRAM
    wire        w_dram_stall;
    wire        w_dram_le;
    wire  [2:0] w_dram_ctrl = (w_we) ? w_ctrl : 3'h2;
    wire [31:0] w_dram_addr = (w_we) ? w_addr : r_addr;
    wire[127:0] w_dram_odata;

    // Cache
    wire        w_cache_oe;
    wire        w_cache_clr   = (r_cache_state == 2'b11 && w_cache_oe);
    wire        w_cache_we    = (r_cache_state == 2'b10 && !w_dram_stall);
    wire [31:0] w_cache_addr  = (r_cache_state == 2'b00) ? w_addr : r_addr;
    wire[127:0] w_cache_idata = w_dram_odata;
    wire[127:0] w_cache_odata;

    wire [31:0] w_odata_t = (r_odata >> {r_addr[3:0], 3'b0});

    always@(posedge CLK) begin
        if(r_cache_state == 2'b01 && !w_cache_oe) begin
            r_cache_state <= 2'b10;
            e_test <= w_cache_oe;
            e_data <= w_cache_odata;
        end
        else if(r_cache_state == 2'b11 || (r_cache_state == 2'b01 && w_cache_oe)
                || (r_cache_state == 2'b10 && !w_dram_stall)) begin
            r_odata <= (r_cache_state == 2'b01) ? w_cache_odata : w_dram_odata;
            r_cache_state <= 2'b00;
            // Cache hit count
            if(r_cache_state == 2'b01 && w_cache_oe) begin 
                e_hit <= e_hit + 1;
            end
            // CACHE DATA CHECK
            if(r_cache_state!=2'b11 && e_test) begin
                if(e_data != w_dram_odata) begin
                    $write("%d: CACHE DATA WRONG!!, ADDR:%x DATA%x %x\n",
                            w_mtime, r_addr, w_dram_odata, e_data);
                    $finish();
                end
            end
        end
        else if(w_we) begin
            r_cache_state   <= 2'b11;
            r_addr          <= w_addr;
            r_ctrl          <= w_ctrl;
        end
        else if(w_le) begin
            r_cache_state   <= 2'b01;
            r_addr          <= w_addr;
            r_ctrl          <= w_ctrl;
            e_cnt           <= e_cnt + 1;
        end
`ifdef DEBUG
        if(w_mtime == 70000000) begin
            $write("cnt = %d, hit = %d\n", e_cnt, e_hit);
            $write("hit rate = %f\n", 1.0*e_hit/e_cnt);
        end
`endif
    end

    m_dram_cache#(28,128,`CACHE_SIZE/16) cache(CLK, 1'b1, 1'b0, w_cache_clr, w_cache_we, w_cache_addr[31:4],
                                w_cache_idata, w_cache_odata, w_cache_oe);

    assign w_dram_le = (r_cache_state == 2'b01 && !w_cache_oe);
    assign w_stall = w_dram_stall || (r_cache_state != 2'b00);

    assign w_odata =    (r_ctrl[1:0]==0) ? ((r_ctrl[2]) ? {24'h0, w_odata_t[7:0]} :
                                            {{24{w_odata_t[7]}}, w_odata_t[7:0]}) :
                        (r_ctrl[1:0]==1) ? ((r_ctrl[2]) ? {16'h0, w_odata_t[15:0]} :
                                            {{16{w_odata_t[15]}}, w_odata_t[15:0]}) :
                        w_odata_t;

    m_bu_mem#(MEM_SIZE) idbmem(CLK, w_dram_addr, w_dram_odata,
                                w_we, w_dram_le, w_wdata, w_dram_ctrl, w_dram_stall);

`ifdef DEBUG
    initial $write("Cache Size: %d KB\n", `CACHE_SIZE/1024);
`endif
endmodule
/**************************************************************************************************/
