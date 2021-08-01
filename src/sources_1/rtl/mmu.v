/**************************************************************************************************/
/**** RVSoC (Mini Kuroda/RISC-V)                       since 2018-08-07   ArchLab. TokyoTech   ****/
/**** Memory Controller v0.01                                                                  ****/
/**************************************************************************************************/
`default_nettype none
/**************************************************************************************************/
`include "define.vh"

/**************************************************************************************************/
module m_mmu(
    input  wire         CLK, clk_50mhz, clk_100mhz, RST_X,
    input  wire [31:0]  w_insn_addr, w_data_addr,
    input  wire [31:0]  w_data_wdata,
    input  wire         w_data_we,
    input  wire  [2:0]  w_data_ctrl,
    output wire [127:0]  w_insn_data,
    output wire [127:0]  w_data_data,
    output wire         w_is_dram_data,
    output reg          r_finish,
    input  wire [31:0]  w_priv, w_satp, w_mstatus,
    input  wire [63:0]  w_mtime, w_mtimecmp,
    output wire [63:0]  w_wmtimecmp,
    output wire         w_clint_we,
    input  wire [31:0]  w_mip,
    output wire [31:0]  w_wmip,
    output wire         w_plic_we,
    output wire         w_proc_busy,
    output wire [31:0]  w_pagefault,
    input  wire  [1:0]  w_tlb_req,
    input  wire         w_tlb_flush,
    output wire         w_txd,
    input  wire         w_rxd,
    output wire         w_init_done,
    input  wire         mig_clk,
    input  wire         mig_rst_x,
    inout  wire [15:0]  ddr2_dq,
    inout  wire  [1:0]  ddr2_dqs_n,
    inout  wire  [1:0]  ddr2_dqs_p,
    output wire [12:0]  ddr2_addr,
    output wire  [2:0]  ddr2_ba,
    output wire         ddr2_ras_n,
    output wire         ddr2_cas_n,
    output wire         ddr2_we_n,
    output wire         ddr2_ck_p,
    output wire         ddr2_ck_n,
    output wire         ddr2_cke,
    output wire         ddr2_cs_n,
    output wire  [1:0]  ddr2_dm,
    output wire         ddr2_odt,
    output wire         o_clk,
    output wire         o_rst_x,
    output wire  [7:0]  w_uart_data,
    output wire         w_uart_we,
    output wire [15:0]  w_led,
    input  wire         w_init_stage,
    output wire [31:0]  w_checksum,
    input  wire         w_debug_btnd,
    input  wire  [1:0]  w_baud,
    input  wire         w_crs_dv_phy,
    output wire  [1:0]  w_txd_phy,
    output wire         w_txen_phy,
    output wire         w_clkin_phy,
    input  wire  [1:0]  w_rxd_phy,
    input  wire         w_rxerr_phy,
    input  wire         sd_cd,
    output wire         sd_rst,
    output wire         sd_sclk,
    inout  wire         sd_cmd,
    inout  wire [ 3:0]  sd_dat,
    output wire         w_init_start,
    output wire [ 3:0]  vga_red,
    output wire [ 3:0]  vga_green,
    output wire [ 3:0]  vga_blue,
    output wire         vga_h_sync,
    output wire         vga_v_sync,
    inout  wire         usb_ps2_clk,
    inout  wire         usb_ps2_data,
`ifdef CH559_USB
    input  wire        ch559_rx,
`else
    inout  wire        pmod_ps2_clk,
    inout  wire        pmod_ps2_data,
`endif
    output wire [2:0]   w_mc_mode,
    output wire         w_tlb_busy,
    output wire         w_dram_busy,
    output wire         w_ps2_kbd_we,
    output wire [ 7:0]  w_ps2_kbd_data,
    output wire         w_ps2_mouse_we,
    output wire [ 7:0]  w_ps2_mouse_data
);

    initial r_finish = 0;

    /***** Address translation ********************************************************************/
    reg  [31:0] physical_addr       = 0;
    reg         page_walk_fail      = 0;

    // Page walk state
    reg  [2:0]  r_pw_state          = 0;
    reg         r_tlb_busy          = 0;

    // Page table entry
    reg  [31:0] L1_pte              = 0;
    reg  [31:0] L0_pte              = 0;

    /***** Other Registers ************************************************************************/
    // Device checker
    reg   [3:0] r_dev               = 0;
    reg   [3:0] r_virt              = 0;

    // TO HOST
    reg  [31:0] r_tohost            = 0;

    // PLIC
    reg  [31:0] plic_served_irq     = 0;
    reg  [31:0] plic_pending_irq    = 0;
    reg  [31:0] r_plic_odata        = 0;

    // CLINT
    reg  [31:0] r_clint_odata       = 0;

    /***** Micro Controller ***********************************************************************/
    reg   [2:0] r_mc_mode           = 0;
    reg  [31:0] r_mc_qnum           = 0;
    reg  [31:0] r_mc_qsel           = 0;

    reg  [31:0] r_mc_done           = 0;
    assign w_mc_mode = r_mc_mode;

    /***** Keyboard Input *************************************************************************/
    reg   [3:0] r_consf_head        = 0;  // Note!!
    reg   [3:0] r_consf_tail        = 0;  // Note!!
    reg   [4:0] r_consf_cnts        = 0;  // Note!!
    reg         r_consf_en;
`ifdef SIM_MODE
    initial #1  r_consf_en <= 1;
`else
    initial #1  r_consf_en <= 0;
`endif

    reg   [7:0] cons_fifo [0:15];
    initial begin
        cons_fifo[0] = 8'h72;
        cons_fifo[1] = 8'h6f;
        cons_fifo[2] = 8'h6f;
        cons_fifo[3] = 8'h74;
        cons_fifo[4] = 8'hd;
        cons_fifo[5] = 8'h74;
        cons_fifo[6] = 8'h6f;
        cons_fifo[7] = 8'h70;
        cons_fifo[8] = 8'hd;
    end

    /**********************************************************************************************/
    // dram data
    wire [31:0] w_dram_odata;


    wire [31:0] w_mc_addr;
    wire [31:0] w_mc_wdata;
    wire        w_mc_we;
    wire  [2:0] w_mc_ctrl;
    wire  [1:0] w_mc_aces;

    wire  w_mode_is_mc  = r_mc_mode != `MC_MODE_CPU;
    wire  w_mode_is_cpu = r_mc_mode == `MC_MODE_CPU;

    wire [31:0] w_mem_wdata = (w_mode_is_mc) ? w_mc_wdata  : w_data_wdata;
    wire        w_mem_we    = (w_mode_is_mc) ? w_mc_we     : w_data_we;


    /***********************************        Page walk       ***********************************/
    wire        w_iscode        = (w_tlb_req == `ACCESS_CODE);
    wire        w_isread        = (w_tlb_req == `ACCESS_READ);
    wire        w_iswrite       = (w_tlb_req == `ACCESS_WRITE);
    reg         r_iscode        = 0;
    reg         r_isread        = 0;
    reg         r_iswrite       = 0;
    wire [31:0] v_addr          = w_iscode ? w_insn_addr : w_data_addr;

    // Level 1
    wire [31:0] vpn1            = {22'b0, v_addr[31:22]};
    wire [31:0] L1_pte_addr     = {w_satp[19:0], 12'b0} + {vpn1, 2'b0};
    wire  [2:0] L1_xwr          = w_mstatus[19] ? (L1_pte[3:1] | L1_pte[5:3]) : L1_pte[3:1];
    wire [31:0] L1_paddr        = {L1_pte[29:10], 12'h0};
    wire [31:0] L1_p_addr       = {L1_paddr[31:22], v_addr[21:0]};
    wire        L1_write        = !L1_pte[6] || (!L1_pte[7] && w_iswrite);
    wire        L1_success      = !(L1_xwr ==2 || L1_xwr == 6 || !L1_pte[0] ||
                                   (L1_xwr != 0 && ((w_priv == `PRIV_S && (L1_pte[4] && !w_mstatus[18])) ||
                                                    (w_priv == `PRIV_U && !L1_pte[4]) ||
                                                    (L1_xwr[w_tlb_req] == 0))));

    // Level 0
    wire [31:0] vpn0            = {22'b0, v_addr[21:12]};
    wire [31:0] L0_pte_addr     = {L1_pte[29:10], 12'b0} + {vpn0, 2'b0};
    wire  [2:0] L0_xwr          = w_mstatus[19] ? (L0_pte[3:1] | L0_pte[5:3]) : L0_pte[3:1];
    wire [31:0] L0_paddr        = {L0_pte[29:10], 12'h0};
    wire [31:0] L0_p_addr       = {L0_paddr[31:12], v_addr[11:0]};
    wire        L0_write        = !L0_pte[6] || (!L0_pte[7] && w_iswrite);
    wire        L0_success      = !(L0_xwr ==2 || L0_xwr == 6 || !L0_pte[0] || !L1_success ||
                                    (w_priv == `PRIV_S && (L0_pte[4] && !w_mstatus[18])) ||
                                    (w_priv == `PRIV_U && !L0_pte[4]) ||
                                    (L0_xwr[w_tlb_req] == 0));

    // update pte
    wire [31:0] L1_pte_write    = L1_pte | `PTE_A_MASK | (w_iswrite ? `PTE_D_MASK : 0);
    wire [31:0] L0_pte_write    = L0_pte | `PTE_A_MASK | (w_iswrite ? `PTE_D_MASK : 0);
    wire        w_pte_we        = (r_pw_state==5) && (((L1_xwr != 0 && L1_success) && L1_write) ||
                                        ((L0_xwr != 0 && L0_success) && L0_write));
    wire [31:0] w_pte_waddr     = (L1_xwr != 0 && L1_success) ? L1_pte_addr : L0_pte_addr;
    wire [31:0] w_pte_wdata     = (L1_xwr != 0 && L1_success) ? L1_pte_write : L0_pte_write;

    assign w_pagefault          = !page_walk_fail ? ~32'h0 : (r_iscode) ? `CAUSE_FETCH_PAGE_FAULT :
                                    (r_isread) ? `CAUSE_LOAD_PAGE_FAULT : `CAUSE_STORE_PAGE_FAULT;

    reg  [31:0] r_tlb_addr = 0;
    reg   [2:0] r_tlb_use  = 0;
    wire [21:0] w_tlb_inst_r_addr, w_tlb_data_r_addr, w_tlb_data_w_addr;
    wire        w_tlb_inst_r_oe, w_tlb_data_r_oe, w_tlb_data_w_oe;
    wire        w_use_tlb = (w_mode_is_cpu && (w_iscode || w_isread || w_iswrite)
                                          && (!(w_priv == `PRIV_M || w_satp[31] == 0)));
    wire        w_tlb_hit = ((w_iscode && w_tlb_inst_r_oe) ||
                            (w_isread && w_tlb_data_r_oe)  ||
                            (w_iswrite && w_tlb_data_w_oe));

    // PAGE WALK state
    always@(posedge CLK) begin
        if(r_pw_state == 0) begin
            // PAGE WALK START
            if(!w_dram_busy && w_use_tlb) begin
                // tlb miss
                if(!w_tlb_hit) begin
                    r_pw_state <= 1;
                    r_tlb_busy <= 1;
                end
                else begin
                    r_pw_state <= 7;
                    r_tlb_busy <= 1;
                    case ({w_iscode, w_isread, w_iswrite})
                        3'b100 : r_tlb_addr <= {w_tlb_inst_r_addr[21:2], w_insn_addr[11:0]};
                        3'b010 : r_tlb_addr <= {w_tlb_data_r_addr[21:2], w_data_addr[11:0]};
                        3'b001 : r_tlb_addr <= {w_tlb_data_w_addr[21:2], w_data_addr[11:0]};
                        default: r_tlb_addr <= 0;
                    endcase
                    r_tlb_use <= {w_iscode, w_isread, w_iswrite};
                end
                {r_iscode, r_isread, r_iswrite} <= {w_iscode, w_isread, w_iswrite};
            end
        end
        // Level 1
        else if(r_pw_state == 1 && !w_dram_busy) begin
            L1_pte      <= w_dram_odata;
            r_pw_state  <= 2;
        end
        else if(r_pw_state == 2) begin
            r_pw_state  <= 3;
        end
        // Level 0
        else if(r_pw_state == 3 && !w_dram_busy) begin
            L0_pte      <= w_dram_odata;
            r_pw_state  <= 4;
        end
        // Success?
        else if(r_pw_state == 4) begin
            if(!L1_pte[0]) begin
                physical_addr   <= 0;
                page_walk_fail  <= 1;
                r_tlb_busy      <= 0;
            end
            else if(L1_xwr) begin
                physical_addr   <= (L1_success) ? L1_p_addr : 0;
                page_walk_fail  <= (L1_success) ? 0 : 1;
                r_tlb_busy      <= L1_success;
            end
            else if(!L0_pte[0]) begin
                physical_addr   <= 0;
                page_walk_fail  <= 1;
                r_tlb_busy      <= 0;
            end
            else if(L0_xwr) begin
                physical_addr   <= (L0_success) ? L0_p_addr : 0;
                page_walk_fail  <= (L0_success) ? 0 : 1;
                r_tlb_busy      <= L0_success;
            end
            r_pw_state  <= 5;
        end
        // Update pte
        else if(r_pw_state == 5) begin
            r_pw_state      <= 0;
            physical_addr   <= 0;
            page_walk_fail  <= 0;
        end
        else if(r_pw_state == 7) begin
            r_pw_state <= 0;
            r_tlb_use <= 0;
            r_tlb_busy <= 0;
            //$write("hoge!, %x, %x\n", page_walk_fail, r_tlb_use);
        end
    end

    /***********************************           TLB          ***********************************/
    wire        w_tlb_inst_r_we   = (r_pw_state == 5 && !page_walk_fail && w_iscode);
    wire        w_tlb_data_r_we   = (r_pw_state == 5 && !page_walk_fail && w_isread);
    wire        w_tlb_data_w_we   = (r_pw_state == 5 && !page_walk_fail && w_iswrite);
    wire [21:0] w_tlb_wdata       = {physical_addr[31:12], 2'b0};

    m_tlb#(20, 22, `TLB_SIZE) TLB_inst_r (CLK, 1'b1, w_tlb_flush, w_tlb_inst_r_we,
                                            w_insn_addr[31:12], w_insn_addr[31:12], w_tlb_wdata,
                                            w_tlb_inst_r_addr, w_tlb_inst_r_oe);

    m_tlb#(20, 22, `TLB_SIZE) TLB_data_r (CLK, 1'b1, w_tlb_flush, w_tlb_data_r_we,
                                            w_data_addr[31:12], w_data_addr[31:12], w_tlb_wdata,
                                            w_tlb_data_r_addr, w_tlb_data_r_oe);

    m_tlb#(20, 22, `TLB_SIZE) TLB_data_w (CLK, 1'b1, w_tlb_flush, w_tlb_data_w_we,
                                            w_data_addr[31:12], w_data_addr[31:12], w_tlb_wdata,
                                            w_tlb_data_w_addr, w_tlb_data_w_oe);

    /***********************************          Memory        ***********************************/
    reg  [31:0] r_tlb_pte_addr = 0;
    reg         r_tlb_acs = 0;
    always@(*)begin
        case (r_pw_state)
            0:      begin r_tlb_pte_addr <= L1_pte_addr;    r_tlb_acs = 1; end
            2:      begin r_tlb_pte_addr <= L0_pte_addr;    r_tlb_acs = 1; end
            5:      begin r_tlb_pte_addr <= w_pte_waddr;    r_tlb_acs = 1; end
            default:begin r_tlb_pte_addr <= 0;              r_tlb_acs = 0; end
        endcase
    end

    //wire w_tlb_i_use = w_tlb_inst_r_oe && w_iscode;
    //wire w_tlb_r_use = w_tlb_data_r_oe && w_isread;
    //wire w_tlb_w_use = w_tlb_data_w_oe && w_iswrite;
    /*wire [31:0] w_tlb_data_addr = (w_tlb_w_use) ?   {w_tlb_data_w_addr[21:2], w_data_addr[11:0]} :
                                                    {w_tlb_data_r_addr[21:2], w_data_addr[11:0]};*/


    wire [31:0] w_insn_paddr =  (w_priv == `PRIV_M || w_satp[31] == 0) ? w_insn_addr :
                                r_tlb_addr;

    wire [31:0] w_mem_paddr  =  (w_mode_is_mc)           ? w_mc_addr     :
                                (w_priv == `PRIV_M || w_satp[31] == 0)  ? w_data_addr   : r_tlb_addr;
//                                (r_pw_state == 5)                       ? r_tlb_addr    :
//                                (r_tlb_acs)                             ? r_tlb_pte_addr:
//                                                                          w_data_addr;

    wire [2:0]  w_mem_ctrl   =  (w_mode_is_mc)                        ? w_mc_ctrl         :
                                (w_priv == `PRIV_M || w_satp[31] == 0)  ? w_data_ctrl       :
                                (r_tlb_use[1:0]!=0)                     ? w_data_ctrl       :
                                (r_pw_state == 0)                       ? `FUNCT3_LW____    :
                                (r_pw_state == 2)                       ? `FUNCT3_LW____    :
                                (r_pw_state == 5)                       ? `FUNCT3_SW____    :
                                w_data_ctrl;

    wire  [3:0] w_dev       = w_mem_paddr[31:28];// & 32'hf0000000;
    wire  [3:0] w_virt      = w_mem_paddr[27:24];// & 32'h0f000000;
    wire [27:0] w_offset    = w_mem_paddr & 28'h7ffffff;

    //always@(posedge CLK) w_virt <= w_mem_paddr & 32'h0f000000;

    wire [31:0] w_dram_wdata    = (r_pw_state == 5) ? w_pte_wdata : w_mem_wdata;
    wire        w_dram_we       = (w_mem_we && !w_tlb_busy
                                    && (w_dev == `MEM_BASE_TADDR || w_dev == 0));

    wire [31:0] w_dram_addr =   (w_mode_is_mc)              ? w_mc_addr         :
                                (w_iscode && !w_tlb_busy)   ? w_insn_paddr      :
                                (w_priv == `PRIV_M || w_satp[31] == 0) ? w_data_addr :
                                (r_tlb_acs && !w_tlb_hit)   ? r_tlb_pte_addr    : w_mem_paddr;

    wire [2:0]  w_dram_ctrl =   (w_mode_is_mc)              ? (w_mem_ctrl)      :
                                (w_iscode && !w_tlb_busy)   ? `FUNCT3_LW____    : w_mem_ctrl;
    assign      w_insn_data =   w_dram_odata128;

    wire        w_dram_aces = (w_dram_addr[31:28] == 8 || w_dram_addr[31:28] == 0);

    wire        w_dram_le   =
                    (w_dram_busy)  ? 0 :
                    (!w_dram_aces) ? 0 :
                    (w_mode_is_mc) ? (w_mc_aces==`ACCESS_READ && w_mc_addr[31:28] != 0) :
                    (w_priv == `PRIV_M || w_satp[31] == 0) ? (w_iscode || w_isread) :
                    (r_tlb_use[2:1]!=0) ? 1 :
                    (w_tlb_busy && !w_tlb_hit && (r_pw_state == 0 || r_pw_state==2)) ? 1 : 0;


    /****************** RVCoreM memory map **********************
    0x40000000 +------------------------+
               | Console registers      |
    0x41000000 +------------------------+
               | Disk registers         |
    0x42000000 +------------------------+
               | Ethernet registers     |
    0x43000000 +------------------------+
               | Keyboard registers     |
    0x44000000 +------------------------+
               | Mouse registers        |
    0x45000000 +------------------------+
               | Simple framebuffer     |
    0x50000000 +------------------------+
               | PLIC                   |
    0x60000000 +------------------------+
               | CLINT                  |
    0x70000000 +------------------------+


    /***********************************         Console        ***********************************/
    wire        w_cons_we   = (w_mode_is_mc) ? (w_mem_we && w_mem_paddr[31:12] == 20'h4000a) :
                            (w_mem_we && !w_tlb_busy && w_dev == `VIRTIO_BASE_TADDR && w_virt == 0);
    wire [31:0] w_cons_data;
    wire [31:0] w_cons_addr = (w_mode_is_mc) ? w_mem_paddr : {4'b0, w_offset};
    wire        w_cons_req;
    wire [31:0] w_cons_irq;
    wire        w_cons_irq_oe;
    wire [31:0] w_cons_qnum;
    wire [31:0] w_cons_qsel;

    /***********************************           Disk         ***********************************/
    wire        w_disk_we   = (w_mode_is_mc) ? (w_mem_we && w_mem_paddr[31:12] == 20'h4000b) :
                            (w_mem_we && !w_tlb_busy && w_dev == `VIRTIO_BASE_TADDR && w_virt == 1);
    wire [31:0] w_disk_data;
    wire [31:0] w_disk_addr = (w_mode_is_mc) ? w_mem_paddr : {4'b0, w_offset};
    wire        w_disk_req;
    wire [31:0] w_disk_irq;
    wire        w_disk_irq_oe;
    wire [31:0] w_disk_qnum;
    wire [31:0] w_disk_qsel;

    /***********************************      Ethernet MAC     ***********************************/
    wire        w_ether_we   = (w_mode_is_mc) ? (w_mem_we && (w_mem_paddr[31:12] == 20'h4000d || w_mem_paddr[31:12] == 20'h4000e)) :
                            (w_mem_we && !w_tlb_busy && w_dev == `VIRTIO_BASE_TADDR && w_virt == 2);
    wire [31:0] w_ether_data;
    wire [31:0] w_ether_addr = (w_mode_is_mc) ? w_mem_paddr : {4'b0, w_offset};
    wire        w_ether_send_req, w_ether_recv_req;
    wire [31:0] w_ether_irq;
    wire        w_ether_irq_oe;
    wire [31:0] w_ether_qnum;
    wire [31:0] w_ether_qsel;
    reg  plic_ether_en = 0;

    /***********************************  　Keyboard     ***********************************/

    wire        w_keybrd_we   = (w_mode_is_mc) ? (w_mem_we && w_mem_paddr[31:12] == 20'h40010 ) :
                            (w_mem_we && !w_tlb_busy && w_dev == `VIRTIO_BASE_TADDR && w_virt == 3);
    wire [31:0] w_keybrd_data;
    wire [31:0] w_keybrd_addr = (w_mode_is_mc) ? w_mem_paddr : {4'b0, w_offset};
    wire        w_keybrd_req;
    wire [31:0] w_keybrd_irq;
    wire        w_keybrd_irq_oe;
    wire [31:0] w_keybrd_qnum;
    wire [31:0] w_keybrd_qsel;

    /***********************************  　Mouse    ***********************************/

    wire        w_mouse_we   = (w_mode_is_mc) ? (w_mem_we && w_mem_paddr[31:12] == 20'h40011 ) :
                            (w_mem_we && !w_tlb_busy && w_dev == `VIRTIO_BASE_TADDR && w_virt == 4);
    wire [31:0] w_mouse_data;
    wire [31:0] w_mouse_addr = (w_mode_is_mc) ? w_mem_paddr : {4'b0, w_offset};
    wire        w_mouse_req;
    wire [31:0] w_mouse_irq;
    wire        w_mouse_irq_oe;
    wire [31:0] w_mouse_qnum;
    wire [31:0] w_mouse_qsel;


    /***********************************      Simple Framebuffer     ***********************************/
    wire        w_fb_we = (w_mem_we && !w_tlb_busy && w_dev == `VIRTIO_BASE_TADDR && w_virt == 5);
    wire [31:0] w_fb_data;
    wire [31:0] w_fb_waddr = {4'b0, w_offset};
    wire [`FB_ADDR_WIDTH-1: 0] w_fb_raddr;
    wire [`FB_PIX_WIDTH-1: 0] w_fb_wdata0;
    wire [`FB_PIX_WIDTH-1: 0] w_fb_wdata1;
    wire [`FB_PIX_WIDTH-1: 0] w_fb_rdata;
    localparam BPC = `FB_PIX_WIDTH/3;   // Bits per Color
    wire [11:0] w_pix_data = {w_fb_rdata[BPC*3-1:BPC*2], {(4-BPC){1'b0}}, w_fb_rdata[BPC*2-1:BPC], {(4-BPC){1'b0}}, w_fb_rdata[BPC-1:0], {(4-BPC){1'b0}}};

    color_converter c0(.i_data(w_mem_wdata[15: 0]), .o_data(w_fb_wdata0));
    color_converter c1(.i_data(w_mem_wdata[31:16]), .o_data(w_fb_wdata1));

    wire pix_clk;
    clk_wiz_3 m_clkgen3 (.clk_in1(clk_100mhz), .reset(), .clk_out1(pix_clk), .locked());

    framebuf  fb0 (
        .i_wclk(CLK),
        .i_we(w_fb_we),
        .i_waddr(w_fb_waddr[`FB_ADDR_WIDTH-1: 0]),
        .i_wdata({w_fb_wdata1, w_fb_wdata0}),
        .i_rclk(pix_clk),
        .i_raddr(w_fb_raddr),
        .o_rdata(w_fb_rdata)
        );

    VGA vga (
        .pix_clk(pix_clk),
        .frame_pix(w_pix_data),
        .VGA_H_SYNC(vga_h_sync),
        .VGA_V_SYNC(vga_v_sync),
        .VGA_RED(vga_red),
        .VGA_BLUE(vga_blue),
        .VGA_GREEN(vga_green),
        .frame_addr(w_fb_raddr)
        );



    /***********************************         SD Card       ***********************************/

    wire [40:0] sdcram_addr;
    wire        sdcram_ren;
    wire [ 3:0] sdcram_wen;
    wire [31:0] sdcram_wdata;
    wire [31:0] sdcram_rdata;
    wire        sdcram_busy;
    wire [ 2:0] sdcram_state;
    wire [ 2:0] sdi_state;
    wire [ 5:0] sdc_state;

    wire [40:0] ctrl_sdcram_addr;
    wire        ctrl_sdcram_ren;
    wire [ 3:0] ctrl_sdcram_wen;
    wire [31:0] ctrl_sdcram_wdata;

    wire [40:0] loader_sdcram_addr;
    wire        loader_sdcram_ren;

    // SD card program loader

    wire [31:0] loader_addr;
    wire [31:0] loader_data;
    wire        loader_we;
    wire        loader_done;


    SDPLOADER sp(
        .i_clk(CLK),
        .i_rst_x(RST_X),
        .o_addr(loader_addr),
        .o_data(loader_data),
        .o_we(loader_we),
        .o_done(loader_done),
        .sdcram_addr(loader_sdcram_addr),
        .sdcram_ren(loader_sdcram_ren),
        .sdcram_rdata(sdcram_rdata),
        .sdcram_busy(sdcram_busy),
        .w_dram_busy(w_dram_busy)
    );

    assign sdcram_addr  = (loader_done) ? ctrl_sdcram_addr  : loader_sdcram_addr ;
    assign sdcram_ren   = (loader_done) ? ctrl_sdcram_ren   : loader_sdcram_ren  ;
    assign sdcram_wen   = (loader_done) ? ctrl_sdcram_wen   : 0                  ;
    assign sdcram_wdata = (loader_done) ? ctrl_sdcram_wdata : 0                  ;


    sdcram#(
        .CACHE_DEPTH(2),
        .BLOCK_NUM(8),
        .POLLING_CYCLES(1024)
    )sdcram_0(
        .i_sys_clk(CLK),
        .i_sys_rst(!RST_X),
        .i_sd_clk(clk_50mhz),
        .i_sd_rst(!RST_X),

        // for user interface
        .i_sdcram_addr(sdcram_addr),
        .i_sdcram_ren(sdcram_ren),
        .i_sdcram_wen(sdcram_wen),
        .i_sdcram_wdata(sdcram_wdata),
        .o_sdcram_rdata(sdcram_rdata),
        .o_sdcram_busy(sdcram_busy),

        // for debug
        .sdcram_state(sdcram_state),
        .sdi_state(sdi_state),
        .sdc_state(sdc_state),

        // for sd
        .sd_cd(sd_cd),
        .sd_rst(sd_rst),
        .sd_sclk(sd_sclk),
        .sd_cmd(sd_cmd),
    	.sd_dat(sd_dat)
    );


    // SDCRAM Controller

    wire [31:0] sdctrl_addr;
    wire        sdctrl_ren;
    wire        sdctrl_wen;
    wire [31:0] sdctrl_wdata;
    wire [31:0] sdctrl_rdata;
    wire        sdctrl_busy;


    assign sdctrl_addr  = w_mc_addr - 32'h90000000 + `BIN_BBL_SIZE;
    assign sdctrl_wdata = w_mc_wdata;

    wire  w_sd_access         = w_mode_is_mc & (w_mc_addr[31:28] >= 9);
    wire  w_sd_read_req       = w_sd_access & (w_mc_aces == `ACCESS_READ);
    wire  w_sd_write_req      = w_sd_access & w_mem_we;

    reg   r_prev_sd_read_req  = 0;
    reg   r_prev_sd_write_req = 0;
    always @ (posedge CLK) begin
        r_prev_sd_read_req  <= w_sd_read_req;
        r_prev_sd_write_req <= w_sd_write_req;
    end


    assign sdctrl_ren = !r_prev_sd_read_req & w_sd_read_req & !sdctrl_busy;
    assign sdctrl_wen = !r_prev_sd_write_req & w_sd_write_req & !sdctrl_busy;

    sdcram_controller sdcon(
        .i_clk(CLK),
        .i_rst_x(RST_X),
        .i_addr(sdctrl_addr),
        .o_rdata(sdctrl_rdata),
        .i_wdata(sdctrl_wdata),
        .i_ren(sdctrl_ren),
        .i_wen(sdctrl_wen),
        .o_busy(sdctrl_busy),
        .sdcram_addr(ctrl_sdcram_addr),
        .sdcram_ren(ctrl_sdcram_ren),
        .sdcram_wen(ctrl_sdcram_wen),
        .sdcram_wdata(ctrl_sdcram_wdata),
        .sdcram_rdata(sdcram_rdata),
        .sdcram_busy(sdcram_busy)
    );

    /***********************************          OUTPUT        ***********************************/
    reg  [31:0] r_data_data = 0;
    reg         is_dram_data;
    always@(*) begin
        is_dram_data = 0;
        case (r_dev)
            `CLINT_BASE_TADDR : r_data_data = r_clint_odata;
            `PLIC_BASE_TADDR  : r_data_data = r_plic_odata;
            `VIRTIO_BASE_TADDR:
            case (r_virt)
                0: r_data_data = w_cons_data;
                1: r_data_data = w_disk_data;
                2: r_data_data = w_ether_data;
                3: r_data_data = w_keybrd_data;
                4: r_data_data = w_mouse_data;
                5: r_data_data = w_fb_data;
                default: r_data_data = 0;
            endcase
            default: begin
                r_data_data = w_dram_odata;
                is_dram_data = 1;
            end
        endcase
    end
    assign w_data_data = (is_dram_data) ? w_dram_odata128 : {96'h0, r_data_data};
    assign w_is_dram_data = is_dram_data;
    /*assign      w_data_data =   (r_dev == `CLINT_BASE_TADDR)    ? r_clint_odata     :
                                (r_dev == `PLIC_BASE_TADDR)     ? r_plic_odata      :
                                (r_dev == `VIRTIO_BASE_TADDR)   ?
                                    ((r_virt == 0) ? w_cons_data : w_disk_data) : w_dram_odata;*/

    /***********************************          VirtIO        ***********************************/

    wire        w_uart_valid;
    wire  [7:0] w_uart_recvdata;
    wire        w_uart_req = r_consf_en && (w_mtime > 64'd61000000) && ((w_mtime & 64'h3ffff) == 0)
                            && w_mode_is_cpu && w_init_stage;
    reg         r_uart_valid = 0;
    reg   [7:0] r_uart_recvdata  = 0;
    always@(posedge CLK) begin
        r_uart_valid   <= w_uart_valid;
        r_uart_recvdata  <= w_uart_recvdata;
    end

    always@(posedge CLK) begin
        if(r_mc_done) begin
            r_mc_mode <= `MC_MODE_CPU;
            r_mc_done <= 0;
            if(r_mc_mode==`MC_MODE_KEY) begin
`ifdef SIM_MODE
                r_consf_en <= 1;
                r_consf_head <= (r_consf_head < 8) ? r_consf_head + 1 : r_consf_head;
`else
                r_consf_en <= (r_consf_cnts<=1) ? 0 : 1;
                r_consf_head <= r_consf_head + 1;
                r_consf_cnts <= r_consf_cnts - 1;
`endif
            end
        end

        if(w_cons_req) begin
            r_mc_mode <= `MC_MODE_CONS;
            r_mc_qnum <= w_cons_qnum;
            r_mc_qsel <= w_cons_qsel;
        end else if(w_uart_req) begin
            r_mc_mode <= `MC_MODE_KEY;
            r_mc_qnum <= w_cons_qnum;
            r_mc_qsel <= 0;
        end else if(w_disk_req) begin
            r_mc_mode <= `MC_MODE_DISK;
            r_mc_qnum <= w_disk_qnum;
            r_mc_qsel <= w_disk_qsel;
        end else if(w_ether_send_req) begin
            r_mc_mode <= `MC_MODE_ETHER_SEND;
            r_mc_qnum <= w_ether_qnum;
            r_mc_qsel <= w_ether_qsel;
        end else if(w_ether_recv_req) begin
            r_mc_mode <= `MC_MODE_ETHER_RECV;
            r_mc_qnum <= w_ether_qnum;
            r_mc_qsel <= 0;
        end else if(w_keybrd_req) begin
            r_mc_mode <= `MC_MODE_KEYBRD;
            r_mc_qnum <= w_keybrd_qnum;
            r_mc_qsel <= w_keybrd_qsel;
        end else if(w_mouse_req) begin
            r_mc_mode <= `MC_MODE_MOUSE;
            r_mc_qnum <= w_mouse_qnum;
            r_mc_qsel <= w_mouse_qsel;
        end


        if(r_uart_valid) begin
            if(r_consf_cnts < 16) begin
                cons_fifo[r_consf_tail] <= r_uart_recvdata;
                r_consf_tail            <= r_consf_tail + 1;
                r_consf_cnts            <= r_consf_cnts + 1;
                r_consf_en              <= 1;
            end
        end

        if(r_tohost[31:16]==`CMD_POWER_OFF) begin
            r_mc_done <= 1;
        end

    end



    reg  [31:0] r_mc_arg = 0;
    always@(*) begin
        case (w_mem_paddr[31:12])
            20'h40009: begin
                case (w_mem_paddr[3:0])
                    4: r_mc_arg = r_mc_qnum;
                    8: r_mc_arg = r_mc_qsel;
                    default: r_mc_arg = r_mc_mode;
                endcase
            end
            20'h4000a: r_mc_arg = w_cons_data;
            20'h4000b: r_mc_arg = w_disk_data;
            20'h4000c: r_mc_arg = cons_fifo[r_consf_head];
            20'h4000d: r_mc_arg = w_ether_data;
            20'h4000f: r_mc_arg = w_ether_data;
            20'h40010: r_mc_arg = w_keybrd_data;
            20'h40011: r_mc_arg = w_mouse_data;
            default:   begin
                if (w_mem_paddr[31:28] >= 9) begin
                    r_mc_arg = sdctrl_rdata;
                end else begin
                    r_mc_arg = w_dram_odata;
                end
            end
        endcase
    end

    wire [31:0] w_mc_arg = r_mc_arg;


    wire [31:0] w_virt_irq      = (w_uart_req)      ? w_cons_irq :
                                  (w_disk_irq_oe)   ? w_disk_irq :
                                  (w_ether_irq_oe)  ? w_ether_irq :
                                  (w_keybrd_irq_oe) ? w_keybrd_irq :
                                  (w_mouse_irq_oe)  ? w_mouse_irq :
                                                      w_cons_irq;

    wire        w_virt_irq_oe   = w_cons_irq_oe | w_disk_irq_oe | w_uart_req | w_ether_irq_oe | w_keybrd_irq_oe | w_mouse_irq_oe;

    m_RVuc mc(CLK, w_mode_is_mc, (w_dram_busy | sdctrl_busy), w_mc_addr, w_mc_arg, w_mc_wdata,
                w_mc_we, w_mc_ctrl, w_mc_aces);

    m_console   console(CLK, 1'b1, w_cons_we, w_cons_addr, w_mem_wdata, plic_pending_irq, w_cons_data,
                        w_cons_irq, w_cons_irq_oe, r_mc_mode, w_cons_req, w_cons_qnum, w_cons_qsel, w_uart_req);

    m_disk      disk(CLK, 1'b1, w_disk_we, w_disk_addr, w_mem_wdata, plic_pending_irq, w_disk_data,
                        w_disk_irq, w_disk_irq_oe, r_mc_mode, w_disk_req, w_disk_qnum, w_disk_qsel);


    m_ether     ether(CLK, clk_50mhz, RST_X, w_ether_we, w_ether_addr, w_mem_wdata, plic_pending_irq, w_ether_data,
                        w_ether_irq, w_ether_irq_oe, r_mc_mode, w_ether_send_req, w_ether_recv_req, w_ether_qnum, w_ether_qsel,
                        w_crs_dv_phy, w_txd_phy, w_txen_phy, w_rxd_phy, w_rxerr_phy, w_clkin_phy, w_mtime, w_init_stage, plic_ether_en);

    m_keyboard  keybrd(CLK, RST_X, w_keybrd_we, w_keybrd_addr, w_mem_wdata, plic_pending_irq, w_keybrd_data,
                        w_keybrd_irq, w_keybrd_irq_oe, r_mc_mode, w_keybrd_req, w_keybrd_qnum, w_keybrd_qsel, w_mtime, w_init_stage,
`ifdef CH559_USB
                        ch559_rx,
`else
                        pmod_ps2_clk, pmod_ps2_data,
`endif
                        w_ps2_kbd_we, w_ps2_kbd_data);

    m_mouse  mouse(CLK, RST_X, w_mouse_we, w_mouse_addr, w_mem_wdata, plic_pending_irq, w_mouse_data,
                        w_mouse_irq, w_mouse_irq_oe, r_mc_mode, w_mouse_req, w_mouse_qnum, w_mouse_qsel, w_mtime, w_init_stage,
                        usb_ps2_clk, usb_ps2_data, w_ps2_mouse_we, w_ps2_mouse_data);


    /***********************************           PLIC         ***********************************/
    wire [31:0] w_plic_pending_irq_nxt  =   w_virt_irq_oe ? w_virt_irq : plic_pending_irq;
    wire [31:0] w_plic_mask             =   w_plic_pending_irq_nxt & ~plic_served_irq;
    wire [31:0] w_plic_served_irq_nxt   =   (w_virt_irq_oe) ? plic_served_irq :
                                          (w_isread) ? plic_served_irq | w_mask_t :
                                          plic_served_irq & ~(1 << (w_data_wdata-1));

    wire [31:0]     w_plic_claim;
    wire[31:0]      w_mask_t = w_plic_mask & (~w_plic_mask+1);

    reg [31:0]      r_plic_claim = 0;

    always @ (*) begin
        case (w_mask_t)
            32'h00000001: r_plic_claim = 1 ;   // console
            32'h00000002: r_plic_claim = 2 ;   // disk
            32'h00000004: r_plic_claim = 3 ;   // ethernet
            32'h00000008: r_plic_claim = 4 ;   // keyboard
            32'h00000010: r_plic_claim = 5 ;   // mouse
            default: r_plic_claim = 0;
        endcase
    end

    wire w_plic_aces = (w_dev == `PLIC_BASE_TADDR && !w_tlb_busy &&
            ((w_isread && w_plic_mask != 0) || (w_iswrite && w_offset == `PLIC_HART_BASE+4)));

    reg  [31:0] r_wmip = 0;
    reg         r_plic_we = 0;

    reg  [31:0] r_plic_pending_irq_t    = 0;
    reg  [31:0] r_plic_served_irq_t     = 0;

    reg         r_virt_irq_oe_t = 0;
    reg         r_plic_aces_t   = 0;

    wire [31:0] w_plic_mask_nxt = r_plic_pending_irq_t & ~r_plic_served_irq_t;

    always@(posedge CLK) begin
        if(!w_tlb_busy) begin
            //r_plic_we   <= (w_virt_irq_oe || w_plic_aces);
            r_virt_irq_oe_t         <= w_virt_irq_oe;
            r_plic_aces_t           <= w_plic_aces;
            r_plic_pending_irq_t    <= w_plic_pending_irq_nxt;
            r_plic_served_irq_t     <= w_plic_served_irq_nxt;
        end
    end

    assign w_plic_we    = (r_virt_irq_oe_t || r_plic_aces_t);//r_plic_we;
    assign w_wmip       = (w_plic_mask_nxt) ? w_mip | (`MIP_MEIP | `MIP_SEIP) :
                          w_mip & ~(`MIP_MEIP | `MIP_SEIP);

//    assign w_plic_we = (w_virt_irq_oe || w_plic_aces);
//    assign w_wmip    = (w_plic_mask_nxt) ? w_mip | (`MIP_MEIP | `MIP_SEIP) :
                        //w_mip & ~(`MIP_MEIP | `MIP_SEIP);

    /***********************************          CLINT         ***********************************/
    assign w_wmtimecmp  = (w_dev == `CLINT_BASE_TADDR && w_offset==28'h4000 && w_data_we != 0) ?
                                {w_mtimecmp[63:32], w_data_wdata} :
                          (w_dev == `CLINT_BASE_TADDR && w_offset==28'h4004 && w_data_we != 0) ?
                                {w_data_wdata, w_mtimecmp[31:0]} : 0;
    assign w_clint_we   = (w_mode_is_cpu &&w_dev == `CLINT_BASE_TADDR && w_data_we != 0);

    /***********************************           BUSY         ***********************************/
    assign w_tlb_busy = //w_use_tlb & r_pw_state != 7
                    !(w_use_tlb)                            ? 0 :
                    (r_pw_state == 7)                       ? 0 : 1;
/*                    (r_pw_state != 0)                       ? 1 :
                    ((w_iscode && w_tlb_inst_r_oe) ||
                    (w_isread && w_tlb_data_r_oe)  ||
                    (w_iswrite && w_tlb_data_w_oe))         ? 0 : 1;*/

    /*wire w_mc_busy =    (r_mc_done)                                                 ? 0 :
                        (r_mc_mode != 0 || w_cons_req || w_uart_req || w_disk_req)   ? 1 : 0;*/
    wire w_mc_busy =    (w_mode_is_mc) ? 1 : 0;


    wire w_tx_ready;
    assign w_proc_busy = r_tlb_busy || w_mc_busy || w_dram_busy || !w_tx_ready;
    /**********************************************************************************************/
    // PLIC, CLINT ACCESS
    reg         r_uart_we = 0;
    reg   [7:0] r_uart_data = 0;

    always@(posedge CLK) begin
        r_dev   <= w_dev;
        r_virt  <= w_virt;

        if (w_dev == `PLIC_BASE_TADDR && !w_tlb_busy && w_iswrite && w_offset == 28'h2000) begin
            plic_ether_en <= (w_data_wdata & (1 << `VIRTIO_ETHER_IRQ)) != 0;
        end


        /*********************************         TOHOST         *********************************/
        // OUTPUT CHAR
        if(r_tohost[31:16]==`CMD_PRINT_CHAR) begin
            r_uart_we   <= 1;
            r_uart_data <= r_tohost[7:0];
        end else begin
            r_uart_we   <= 0;
            r_uart_data <= 0;
        end
        // Finish Simulation
        if(r_tohost[31:16]==`CMD_POWER_OFF) begin
            if(w_mode_is_cpu) r_finish = 1;
            else begin
                r_tohost <= 0;
            end
        end
        else begin
            r_tohost <= (w_mem_paddr==`TOHOST_ADDR && w_mem_we) ? w_mem_wdata   :
                        (r_tohost[31:16]==`CMD_PRINT_CHAR)      ? 0             : r_tohost;
        end

        /**********************************         PLIC         **********************************/
        if(w_plic_aces) begin
            r_plic_odata    <= r_plic_claim;
            plic_served_irq <= w_plic_served_irq_nxt;
        end

        if(w_virt_irq_oe) begin
            plic_pending_irq    <= w_virt_irq;
        end


        /*********************************          CLINT         *********************************/
        r_clint_odata <=    (w_offset==28'hbff8) ? w_mtime[31:0] :
                            (w_offset==28'hbffc) ? w_mtime[63:32] :
                            (w_offset==28'h4000) ? w_mtimecmp[31:0] :
                            (w_offset==28'h4004) ? w_mtimecmp[63:32] : 0;
    end
    /**********************************************************************************************/

    wire w_cons_txd;
    UartTx UartTx0(CLK, RST_X, r_uart_data, r_uart_we, w_cons_txd, w_tx_ready);
    serialc serialc0(CLK, RST_X, w_rxd, w_uart_recvdata, w_uart_valid);


    assign w_uart_data = r_uart_data;
    assign w_uart_we   = r_uart_we;

    wire [31:0]  w_pl_init_addr;
    wire [31:0]  w_pl_init_data;
    wire         w_pl_init_we;
    wire         w_pl_init_done;

    assign       w_pl_init_addr = loader_addr;
    assign       w_pl_init_data = loader_data;
    assign       w_pl_init_we   = loader_we;
    assign       w_pl_init_done = loader_done;

    /**********************************************************************************************/

    reg  [31:0]  r_checksum = 0;
    always@(posedge CLK) begin
        r_checksum <= (!RST_X)                      ? 0                             :
                      (!w_init_done & w_pl_init_we) ? r_checksum + w_pl_init_data   :
                      r_checksum;
    end

    assign w_checksum = r_checksum;

    /**********************************************************************************************/

    wire w_debug_txd;
    wire w_rec_done;
    m_debug_key debug_KEY(CLK, RST_X, w_debug_btnd, w_debug_txd, w_uart_we, w_uart_data, w_mtime[31:0], w_rec_done);

    assign w_txd = (w_rec_done) ? w_debug_txd : w_cons_txd;

/**************************************************************************************************/
    reg          r_bbl_done   = 0;
    reg          r_dtree_done = 0;
    reg          r_disk_done  = 1;
    reg  [31:0]  r_initaddr  = 0;
    reg  [31:0]  r_initaddr3 = (16*1024*1024); /* initial address of Device Tree */

    // Zero init
    wire w_zero_we;
    reg  r_zero_we;
    reg  r_zero_done        = 1;
    reg  [31:0]  r_zeroaddr = 0;

`ifdef SIM_MODE
    reg  [2:0] r_init_state = 4;
`else
    reg  [2:0] r_init_state = 0;
    always@(posedge CLK) begin
        r_init_state <= (!RST_X) ? 0 :
                      (r_init_state == 0)              ? 2 :
                      //(r_init_state == 1 & r_zero_done)  ? 2 :
                      (r_init_state == 2 & r_bbl_done)   ? 3 :
                      (r_init_state == 3 & r_dtree_done) ? 4 :
                      r_init_state;
    end
`endif

    assign w_init_start = (r_initaddr != 0);

    assign w_init_done = (r_init_state == 4);

    always@(posedge CLK) begin
        if(w_pl_init_we & (r_init_state == 2))      r_initaddr      <= r_initaddr + 4;
        if(r_initaddr  >= (9*1024*1024))            r_bbl_done      <= 1;
        if(w_pl_init_we & (r_init_state == 3))      r_initaddr3     <= r_initaddr3 + 4;
        if(r_initaddr3 >= (16*1024*1024 + 4*1024))  r_dtree_done    <= 1;
    end

    // Zero init
    always@(posedge CLK) begin
        if(!w_dram_busy & !r_zero_done) r_zero_we <= 1;
        if(r_zero_we) begin
            r_zero_we    <= 0;
            r_zeroaddr <= r_zeroaddr + 4;
        end
        if(r_zeroaddr >= `MEM_SIZE) r_zero_done <= 1;
    end

`ifdef SIM_MODE
    assign w_zero_we = 0;
`else
    assign w_zero_we = r_zero_we;
`endif
    /**********************************************************************************************/
    wire [31:0]  w_dram_addr_t  = w_dram_addr & 32'h7ffffff;
    wire [31:0]  w_dram_addr_t2 =
                    (r_init_state == 1) ? r_zeroaddr     :
                    (r_init_state == 2) ? r_initaddr     :
                    (r_init_state == 3) ? r_initaddr3    : w_dram_addr_t;

    wire [31:0]  w_dram_wdata_t   =   (r_init_state == 1) ? 32'b0 :
                                    (r_init_state == 4) ? w_dram_wdata : w_pl_init_data;
    wire         w_dram_we_t      =   (w_pte_we || w_dram_we) && !w_dram_busy;
    wire [2:0]   w_dram_ctrl_t  = (!w_init_done) ? `FUNCT3_SW____ : w_dram_ctrl;

    reg  [31:0] r_addr = 0;
    reg  [2 :0] r_ctrl = 0;
    always @ (posedge CLK) begin
        if (w_dram_we_t | w_dram_le) begin
            r_addr <= w_dram_addr_t2;
            r_ctrl <= w_dram_ctrl_t;
        end
    end
    wire [127:0] w_dram_odata128;

    wire[127:0] w_odata_t1 = (w_dram_odata128 >> {r_addr[3:0], 3'b0});
    wire [31:0] w_odata_t2 = w_odata_t1[31:0];

    wire [31:0] w_ld_lb = {{24{w_odata_t2[ 7]&(~r_ctrl[2])}}, w_odata_t2[ 7:0]};
    wire [31:0] w_ld_lh = {{16{w_odata_t2[15]&(~r_ctrl[2])}}, w_odata_t2[15:0]};

    assign w_dram_odata = (r_ctrl[1:0]==0) ? w_ld_lb :
                    (r_ctrl[1:0]==1) ? w_ld_lh : w_odata_t2;

/**************************************************************************************************/

`ifdef SIM_MODE
    m_dram_sim#(`MEM_SIZE) idbmem(CLK, w_dram_addr_t2, w_dram_odata, w_dram_we_t, w_dram_le,
                                    w_dram_wdata_t, w_dram_ctrl_t, w_dram_busy, w_mtime[31:0]);
`else
    wire calib_done;
    DRAM_conRV dram_con (
                                // user interface ports
                               .i_rd_en(w_dram_le),
                               .i_wr_en(w_zero_we || w_pl_init_we || w_dram_we_t),
                               .i_addr(w_dram_addr_t2),
                               .i_data(w_dram_wdata_t),
                               .o_data(w_dram_odata128),
                               .o_busy(w_dram_busy),
                               .i_ctrl(w_dram_ctrl_t),
                               // input clk, rst (active-low)
                               .mig_clk(mig_clk),
                               .mig_rst_x(mig_rst_x),
                               // memory interface ports
                               .ddr2_dq(ddr2_dq),
                               .ddr2_dqs_n(ddr2_dqs_n),
                               .ddr2_dqs_p(ddr2_dqs_p),
                               .ddr2_addr(ddr2_addr),
                               .ddr2_ba(ddr2_ba),
                               .ddr2_ras_n(ddr2_ras_n),
                               .ddr2_cas_n(ddr2_cas_n),
                               .ddr2_we_n(ddr2_we_n),
                               .ddr2_ck_p(ddr2_ck_p),
                               .ddr2_ck_n(ddr2_ck_n),
                               .ddr2_cke(ddr2_cke),
                               .ddr2_cs_n(ddr2_cs_n),
                               .ddr2_dm(ddr2_dm),
                               .ddr2_odt(ddr2_odt),
                               // output clk, rst (active-low)
                               .o_clk(o_clk),
                               .o_rst_x(o_rst_x),
                               // other
                               .o_init_calib_complete(calib_done)
                               );
`endif

    assign w_led = (w_proc_busy << 12) | (r_mc_mode << 8)
                    | ({w_pl_init_done, r_disk_done, r_bbl_done, r_zero_done} << 4) | r_init_state;

endmodule
/**************************************************************************************************/
/*** Simple Direct Mapped Cache for TLB                                                         ***/
/**************************************************************************************************/
module m_tlb#(parameter ADDR_WIDTH = 20, D_WIDTH = 20, ENTRY = 4)
            (CLK, RST_X, w_flush, w_we, w_waddr, w_raddr, w_idata, w_odata, w_oe);
    input  wire                     CLK, RST_X;
    input  wire                     w_flush, w_we;
    input  wire [ADDR_WIDTH-1:0]    w_waddr, w_raddr;
    input  wire    [D_WIDTH-1:0]    w_idata;
    output wire    [D_WIDTH-1:0]    w_odata;
    output wire                     w_oe;             //output enable
    reg                               [ENTRY-1:0]   r_valid = 0;
    reg  [(ADDR_WIDTH-$clog2(ENTRY)+D_WIDTH)-1:0]   mem [0:ENTRY-1];
    integer i;
    initial for(i=0; i<ENTRY; i=i+1) mem[i] = 0;
    // READ
    wire              [$clog2(ENTRY)-1:0]   w_ridx;
    wire [(ADDR_WIDTH-$clog2(ENTRY))-1:0]   w_rtag;
    assign {w_rtag, w_ridx} = w_raddr;
    wire [ENTRY-1:0] w_ridx_v = ({{(ENTRY-1){1'b0}},{1'b1}} << w_ridx);
    wire w_tagmatch = (mem[w_ridx][(ADDR_WIDTH-$clog2(ENTRY)+D_WIDTH)-1:D_WIDTH] == w_rtag);
    assign w_odata  = mem[w_ridx][D_WIDTH-1:0];
    //assign w_oe     = (w_tagmatch && r_valid[w_ridx]);
    assign w_oe     = (w_tagmatch && (r_valid & w_ridx_v));
    // WRITE
    wire              [$clog2(ENTRY)-1:0]   w_widx;
    wire [(ADDR_WIDTH-$clog2(ENTRY))-1:0]   w_wtag;
    assign {w_wtag, w_widx} = w_waddr;
    wire [ENTRY-1:0] w_widx_v = ({{(ENTRY-1){1'b0}},{1'b1}} << w_widx);
    always  @(posedge  CLK)  begin
        // FLUSH
        if (!RST_X || w_flush) begin
            r_valid <= 0;
        end
        if (w_we) begin
            mem[w_widx] <= {w_wtag, w_idata};
            //r_valid[w_widx] <= 1;
            r_valid <= r_valid | w_widx_v;
        end
    end
endmodule // m_tlb
/**************************************************************************************************/
