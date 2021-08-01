/**************************************************************************************************/
/**** RVSoC (Mini Kuroda/RISC-V)                       since 2018-08-07   ArchLab. TokyoTech   ****/
/**** Memory Module v0.02                                                                      ****/
/**************************************************************************************************/
`default_nettype none
/**************************************************************************************************/
`include "define.vh"

/**** DRAM Main Memory module for implementation                                               ****/
/**************************************************************************************************/
`ifndef SIM_MODE
/**** DRAM Controller with Cache                                                               ****/
/**************************************************************************************************/
module DRAM_con#(
              parameter DDR2_DQ_WIDTH   = 16,
              parameter DDR2_DQS_WIDTH  = 2,
              parameter DDR2_ADDR_WIDTH = 13,
              parameter DDR2_BA_WIDTH   = 3,
              parameter DDR2_DM_WIDTH   = 2,
              parameter APP_ADDR_WIDTH  = 27,
              parameter APP_CMD_WIDTH   = 3,
              parameter APP_DATA_WIDTH  = 128,  // Note
              parameter APP_MASK_WIDTH  = 16)
    (
     // input clk, rst (active-low)
     input  wire                         mig_clk,
     input  wire                         mig_rst_x,
     // memory interface ports
     inout  wire [DDR2_DQ_WIDTH-1 : 0]   ddr2_dq,
     inout  wire [DDR2_DQS_WIDTH-1 : 0]  ddr2_dqs_n,
     inout  wire [DDR2_DQS_WIDTH-1 : 0]  ddr2_dqs_p,
     output wire [DDR2_ADDR_WIDTH-1 : 0] ddr2_addr,
     output wire [DDR2_BA_WIDTH-1 : 0]   ddr2_ba,
     output wire                         ddr2_ras_n,
     output wire                         ddr2_cas_n,
     output wire                         ddr2_we_n,
     output wire [0:0]                   ddr2_ck_p,
     output wire [0:0]                   ddr2_ck_n,
     output wire [0:0]                   ddr2_cke,
     output wire [0:0]                   ddr2_cs_n,
     output wire [DDR2_DM_WIDTH-1 : 0]   ddr2_dm,
     output wire [0:0]                   ddr2_odt,
     // output clk, rst (active-low)
     output wire                         o_clk,
     output wire                         o_rst_x,
     // user interface ports
     input  wire                         i_rd_en,
     input  wire                         i_wr_en,
     input  wire [31:0]                  i_addr,
     input  wire [31:0]                  i_data,
     output wire                         o_init_calib_complete,
     output wire [31:0]                  o_data,
     output wire                         o_busy,
     input  wire [2:0]                   i_ctrl);

    /***** store output data to registers in posedge clock cycle *****/

    reg   [1:0] r_cache_state = 0;

    reg  [31:0] r_addr = 0;
    reg   [2:0] r_ctrl = 0;
    reg [127:0] r_o_data = 0;

    // DRAM
    wire        w_dram_stall;
    wire        w_dram_le;
    wire  [2:0] w_dram_ctrl = (i_wr_en) ? i_ctrl : 3'h2;
    wire [31:0] w_dram_addr = (i_wr_en) ? i_addr : r_addr;
    wire[127:0] w_dram_odata;

    // Cache
    wire        c_oe;
    wire        c_clr   = (r_cache_state == 2'b11 && c_oe);
    wire        c_we    = (r_cache_state == 2'b10 && !w_dram_stall);
    wire [31:0] c_addr  = (r_cache_state == 2'b00) ? i_addr : r_addr;
    wire[127:0] c_idata = w_dram_odata;
    wire[127:0] c_odata;

    wire[127:0] r_odata_t2  = (r_o_data >> {r_addr[3:0], 3'b0});
    wire [31:0] r_o_data_t  = r_odata_t2[31:0];

    always@(posedge o_clk) begin
        if(r_cache_state == 2'b01 && !c_oe) begin
            r_cache_state <= 2'b10;
        end
        else if(r_cache_state == 2'b11 || (r_cache_state == 2'b01 && c_oe)
                || (r_cache_state == 2'b10 && !w_dram_stall)) begin
            r_cache_state <= 2'b00;
            r_o_data <= (r_cache_state == 2'b01) ? c_odata : w_dram_odata;
        end
        else if(i_wr_en) begin
            r_cache_state <= 2'b11;
            r_addr <= i_addr;
            r_ctrl <= i_ctrl;
        end
        else if(i_rd_en) begin
            r_cache_state <= 2'b01;
            r_addr <= i_addr;
            r_ctrl <= i_ctrl;
        end
    end

    m_dram_cache#(28,128,`CACHE_SIZE/16) cache(o_clk, 1'b1, 1'b0, c_clr, c_we,
                                c_addr[31:4], c_idata, c_odata, c_oe);

    assign w_dram_le = (r_cache_state == 2'b01 && !c_oe);
    assign o_busy = w_dram_stall || r_cache_state != 0;

    assign o_data = (r_ctrl[1:0]==0) ?  ((r_ctrl[2]) ? {24'h0, r_o_data_t[7:0]} :
                                        {{24{r_o_data_t[7]}}, r_o_data_t[7:0]}) :
                    (r_ctrl[1:0]==1) ?  ((r_ctrl[2]) ? {16'h0, r_o_data_t[15:0]} :
                                        {{16{r_o_data_t[15]}}, r_o_data_t[15:0]}) :
                    r_o_data_t;

    DRAM_Wrapper dram (
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
               // user interface ports
               .i_rd_en(w_dram_le),
               .i_wr_en(i_wr_en),
               .i_addr(w_dram_addr),
               .i_data(i_data),
               .o_init_calib_complete(o_init_calib_complete),
               .o_data(w_dram_odata),
               .o_busy(w_dram_stall),
               .i_ctrl(w_dram_ctrl)
               );

endmodule
/**************************************************************************************************/
module DRAM_Wrapper #(
              parameter DDR2_DQ_WIDTH   = 16,
              parameter DDR2_DQS_WIDTH  = 2,
              parameter DDR2_ADDR_WIDTH = 13,
              parameter DDR2_BA_WIDTH   = 3,
              parameter DDR2_DM_WIDTH   = 2,
              parameter APP_ADDR_WIDTH  = 27,
              parameter APP_CMD_WIDTH   = 3,
              parameter APP_DATA_WIDTH  = 128,  // Note
              parameter APP_MASK_WIDTH  = 16)
    (
     // input clk, rst (active-low)
     input  wire                         mig_clk,
     input  wire                         mig_rst_x,
     // memory interface ports
     inout  wire [DDR2_DQ_WIDTH-1 : 0]   ddr2_dq,
     inout  wire [DDR2_DQS_WIDTH-1 : 0]  ddr2_dqs_n,
     inout  wire [DDR2_DQS_WIDTH-1 : 0]  ddr2_dqs_p,
     output wire [DDR2_ADDR_WIDTH-1 : 0] ddr2_addr,
     output wire [DDR2_BA_WIDTH-1 : 0]   ddr2_ba,
     output wire                         ddr2_ras_n,
     output wire                         ddr2_cas_n,
     output wire                         ddr2_we_n,
     output wire [0:0]                   ddr2_ck_p,
     output wire [0:0]                   ddr2_ck_n,
     output wire [0:0]                   ddr2_cke,
     output wire [0:0]                   ddr2_cs_n,
     output wire [DDR2_DM_WIDTH-1 : 0]   ddr2_dm,
     output wire [0:0]                   ddr2_odt,
     // output clk, rst (active-low)
     output wire                         o_clk,
     output wire                         o_rst_x,
     // user interface ports
     input  wire                         i_rd_en,
     input  wire                         i_wr_en,
     input  wire [31:0]                  i_addr,
     input  wire [31:0]                  i_data,
     output wire                         o_init_calib_complete,
     output wire [127:0]                 o_data,
     output wire                         o_busy,
     input  wire [2:0]                   i_ctrl);

    /***** store output data to registers in posedge clock cycle *****/

    wire [127:0]w_o_data;
    wire        w_o_busy;

    reg  [127:0]r_o_data = 0;
    reg         r_o_busy = 0;
    always @(posedge o_clk) begin
        r_o_data <= w_ctrl_data;
        r_o_busy <= w_o_busy;
    end

    assign o_data = r_o_data;
    assign o_busy = r_o_busy || r_le || r_we;

    /***** select load data by i_ctrl *****/

    reg  [2:0]  r_ctrl  = 0;
    reg  [31:0] r_iaddr = 0;
    wire [127:0]w_ctrl_data;

    reg  [31:0] r_wdata = 0;
    reg         r_le = 0;
    reg         r_we = 0;

    always @(posedge o_clk) begin
        //if(i_rd_en) begin
        r_ctrl  <= i_ctrl;
        r_iaddr <= i_addr;
        //end
        r_le <= i_rd_en;
        r_we <= i_wr_en;
        r_wdata <= i_data;
    end
    assign w_ctrl_data = w_o_data;

    wire [31:0] w_ctrl_iaddr = (r_we) ? {r_iaddr[31:2],2'b0} : {r_iaddr[31:4],4'b0};

    wire [3:0]  w_data_mask = (r_ctrl[1:0] == 0) ? (4'b0001 << r_iaddr[1:0]) :
                            (r_ctrl[1:0] == 1) ? (4'b0011 << {r_iaddr[1], 1'b0}) : 4'b1111;

    wire [31:0] w_data =    (r_ctrl[1:0] == 0) ? {4{r_wdata[7:0]}} :
                            (r_ctrl[1:0] == 1) ? {2{r_wdata[15:0]}} : r_wdata;

    DRAM_con_witout_cache dram_con_witout_cache (
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
               // user interface ports
               .i_rd_en(r_le),
               .i_wr_en(r_we),
               .i_addr(w_ctrl_iaddr),
               .i_data(w_data),
               .o_init_calib_complete(o_init_calib_complete),
               .o_data(w_o_data),
               .o_busy(w_o_busy),
               .i_mask(~w_data_mask)
               );

endmodule

/**** DRAM Controller with Cache                                                               ****/
/**************************************************************************************************/
module DRAM_conRV#(
              parameter DDR2_DQ_WIDTH   = 16,
              parameter DDR2_DQS_WIDTH  = 2,
              parameter DDR2_ADDR_WIDTH = 13,
              parameter DDR2_BA_WIDTH   = 3,
              parameter DDR2_DM_WIDTH   = 2,
              parameter APP_ADDR_WIDTH  = 27,
              parameter APP_CMD_WIDTH   = 3,
              parameter APP_DATA_WIDTH  = 128,  // Note
              parameter APP_MASK_WIDTH  = 16)
    (
     // input clk, rst (active-low)
     input  wire                         mig_clk,
     input  wire                         mig_rst_x,
     // memory interface ports
     inout  wire [DDR2_DQ_WIDTH-1 : 0]   ddr2_dq,
     inout  wire [DDR2_DQS_WIDTH-1 : 0]  ddr2_dqs_n,
     inout  wire [DDR2_DQS_WIDTH-1 : 0]  ddr2_dqs_p,
     output wire [DDR2_ADDR_WIDTH-1 : 0] ddr2_addr,
     output wire [DDR2_BA_WIDTH-1 : 0]   ddr2_ba,
     output wire                         ddr2_ras_n,
     output wire                         ddr2_cas_n,
     output wire                         ddr2_we_n,
     output wire [0:0]                   ddr2_ck_p,
     output wire [0:0]                   ddr2_ck_n,
     output wire [0:0]                   ddr2_cke,
     output wire [0:0]                   ddr2_cs_n,
     output wire [DDR2_DM_WIDTH-1 : 0]   ddr2_dm,
     output wire [0:0]                   ddr2_odt,
     // output clk, rst (active-low)
     output wire                         o_clk,
     output wire                         o_rst_x,
     // user interface ports
     input  wire                         i_rd_en,
     input  wire                         i_wr_en,
     input  wire [31:0]                  i_addr,
     input  wire [31:0]                  i_data,
     output wire                         o_init_calib_complete,
     output wire [127:0]                  o_data,
     output wire                         o_busy,
     input  wire [2:0]                   i_ctrl
     );

    /***** store output data to registers in posedge clock cycle *****/

    reg  [31:0] r_addr  = 0;
    reg   [2:0] r_ctrl  = 0;

    reg         r_we    = 0;
    reg  [31:0] r_wdata = 0;
    reg         r_rd    = 0;

    always @(posedge o_clk) begin
        if((i_rd_en || i_wr_en) && !o_busy) begin
            r_ctrl  <= i_ctrl;
            r_addr  <= i_addr;
            r_wdata <= i_data;
        end
        r_we    <= i_wr_en;
        r_rd    <= i_rd_en;
    end

    wire[127:0] w_dram_odata;
    assign o_data = w_dram_odata;
    wire [3:0]  w_mask =    (r_ctrl[1:0] == 0) ? (4'b0001 << r_addr[1:0]) :
                            (r_ctrl[1:0] == 1) ? (4'b0011 << {r_addr[1], 1'b0}) : 4'b1111;
    wire [31:0] w_wdata =   (r_ctrl[1:0] == 0) ? {4{r_wdata[ 7:0]}} :
                            (r_ctrl[1:0] == 1) ? {2{r_wdata[15:0]}} : r_wdata;

    //wire        w_addr = (r_we) ? r_addr : i_addr;

    wire w_busy;
    assign o_busy = w_busy | r_we | r_rd;

    DRAM_conX dram (
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
               // user interface ports
               .i_rd_en(r_rd),
               .i_wr_en(r_we),
               .i_addr(r_addr),
               .i_data(w_wdata),
               .o_init_calib_complete(o_init_calib_complete),
               .o_data(w_dram_odata),
               .o_busy(w_busy),
               .i_mask(~w_mask)
               );

endmodule

/**************************************************************************************************/
/****Cache Controller                                                                          ****/
/**************************************************************************************************/
module cach_controller (
    input  wire                         clk,
    input  wire                         rst_x,
    input  wire                         i_rd_en,
    input  wire                         i_wr_en,
    input  wire [31:0]                  i_addr,
    input  wire [31:0]                  i_data,
    output wire [127:0]                 o_data,
    output wire                         o_busy,
    input  wire [3:0]                   i_mask,
    // cach
    input  wire                         cach_hit,
    output reg                          cach_we,
    output reg  [31:0]                  cach_addr,
    output reg  [127:0]                 cach_wdata,
    input  wire [127:0]                 cach_rdata,
    //dram
    input  wire                         dram_stall,
    output reg                          dram_le,
    output wire                         dram_we,
    input  wire [127:0]                 dram_rdata,
    output wire [31:0]                  dram_wdata,
    output wire [31:0]                  dram_addr
    );


    reg  [31:0] r_addr = 0;
    reg  [31:0] r_wdata = 0;
    reg [127:0] r_o_data = 0;

    // write buffer
    reg  [31:0] write_buf  = 0;
    reg         wbuf_full  = 0;
    reg         wbuf_we;


    wire [31:0]  write_mask = ~{{8{i_mask[3]}}, {8{i_mask[2]}}, {8{i_mask[1]}}, {8{i_mask[0]}}};
    wire [127:0] cach_rdata_mask = ~{{96'h0, write_mask} << {r_addr[3:2], 5'b0}};
    wire [127:0] cach_wdata_masked = (r_wdata & write_mask) << {r_addr[3:2], 5'b0};

    localparam  IDLE = 0;
    localparam  READ = 1;
    localparam  CACH_WRITE = 2;
    localparam  WBUF_WAIT = 3;
    localparam  READ_MISS = 4;
    localparam  WRITE_MISS = 5;

    reg [2:0]   r_state = IDLE;
    reg [2:0]   next_state = 0;
    reg [127:0] next_odata;
    // combinational logic

    always @ (*) begin
        next_state = r_state;
        next_odata = r_o_data;
        wbuf_we    = 0;
        dram_le    = 0;
        cach_we    = 0;
        cach_addr  = r_addr;
        cach_wdata = dram_rdata;
        case (r_state)
            IDLE: begin
                if (i_wr_en) begin
                    if (!wbuf_full) begin
                        wbuf_we = 1;
                        next_state = CACH_WRITE;
                    end else begin
                        next_state = WBUF_WAIT;
                    end
                end else if (i_rd_en) begin
                    next_state = READ;
                end
                cach_addr = i_addr;
            end
            READ: begin
                if (cach_hit) begin
                    next_state = IDLE;
                    next_odata = cach_rdata;
                end else begin
                    if (!dram_stall) begin
                        next_state = READ_MISS;
                        dram_le    = 1;
                    end
                end
            end
            READ_MISS: begin
                if (!dram_stall) begin
                    next_state = IDLE;
                    next_odata = dram_rdata;
                    cach_we = 1;
                    cach_wdata = dram_rdata;
                end
            end
            WBUF_WAIT: begin
                if (!wbuf_full) begin
                    wbuf_we = 1;
                    next_state = CACH_WRITE;
                end
            end
            CACH_WRITE: begin
                if (cach_hit) begin
                    next_state = IDLE;
                    cach_we = 1;
                    cach_wdata = (cach_rdata & cach_rdata_mask) | cach_wdata_masked;
                end else begin
`ifdef FETCH_BLOCK_ON_WRITE_MISS
                    if (!dram_stall) begin
                        next_state = WRITE_MISS;
                        dram_le = 1;
                    end
`else
                    next_state = IDLE;
`endif
                end
            end
            WRITE_MISS: begin
                if (!dram_stall) begin
                    next_state = IDLE;
                    cach_we = 1;
                    cach_wdata = (dram_rdata & cach_rdata_mask) | cach_wdata_masked;
                end
            end
            default:;
        endcase
    end

    always @ (posedge clk) begin
        r_state <= (!rst_x) ? IDLE : next_state;
        r_o_data <= next_odata;
        if ((i_wr_en || i_rd_en) && r_state == IDLE) begin
            r_addr <= i_addr;
            r_wdata <= i_data;
        end
    end

    always @ (posedge clk) begin
        if (wbuf_we) begin
            wbuf_full  <= 1;
        end else if (wbuf_full & !dram_stall) begin
            wbuf_full <= 0;
        end
    end

    assign dram_we = (!wbuf_full) & wbuf_we;
    assign dram_addr = (wbuf_we && r_state == IDLE) ? i_addr : r_addr;
    assign dram_wdata = (r_state == IDLE) ? i_data : r_wdata;

    assign o_busy = r_state != IDLE;

    assign o_data = r_o_data;

endmodule //cach_controller

/**************************************************************************************************/
/**** DRAM Controller with Cache                                                               ****/
/**************************************************************************************************/
module DRAM_conX#(
              parameter DDR2_DQ_WIDTH   = 16,
              parameter DDR2_DQS_WIDTH  = 2,
              parameter DDR2_ADDR_WIDTH = 13,
              parameter DDR2_BA_WIDTH   = 3,
              parameter DDR2_DM_WIDTH   = 2,
              parameter APP_ADDR_WIDTH  = 27,
              parameter APP_CMD_WIDTH   = 3,
              parameter APP_DATA_WIDTH  = 128,  // Note
              parameter APP_MASK_WIDTH  = 16)
    (
     // input clk, rst (active-low)
     input  wire                         mig_clk,
     input  wire                         mig_rst_x,
     // memory interface ports
     inout  wire [DDR2_DQ_WIDTH-1 : 0]   ddr2_dq,
     inout  wire [DDR2_DQS_WIDTH-1 : 0]  ddr2_dqs_n,
     inout  wire [DDR2_DQS_WIDTH-1 : 0]  ddr2_dqs_p,
     output wire [DDR2_ADDR_WIDTH-1 : 0] ddr2_addr,
     output wire [DDR2_BA_WIDTH-1 : 0]   ddr2_ba,
     output wire                         ddr2_ras_n,
     output wire                         ddr2_cas_n,
     output wire                         ddr2_we_n,
     output wire [0:0]                   ddr2_ck_p,
     output wire [0:0]                   ddr2_ck_n,
     output wire [0:0]                   ddr2_cke,
     output wire [0:0]                   ddr2_cs_n,
     output wire [DDR2_DM_WIDTH-1 : 0]   ddr2_dm,
     output wire [0:0]                   ddr2_odt,
     // output clk, rst (active-low)
     output wire                         o_clk,
     output wire                         o_rst_x,
     // user interface ports
     input  wire                         i_rd_en,
     input  wire                         i_wr_en,
     input  wire [31:0]                  i_addr,
     input  wire [31:0]                  i_data,
     output wire                         o_init_calib_complete,
     output wire[127:0]                  o_data,
     output wire                         o_busy,
     input  wire [3:0]                   i_mask);

    /***** store output data to registers in posedge clock cycle *****/


    // DRAM
    wire        w_dram_stall;
    wire        w_dram_le;
    wire        w_dram_we;
    wire [31:0] w_dram_addr;
    wire [31:0] w_dram_idata;
    wire[127:0] w_dram_odata;

    // Cache
    wire        c_oe;
    wire        c_clr;
    wire        c_we;
    wire [31:0] c_addr;
    wire[127:0] c_idata;
    wire[127:0] c_odata;

    cach_controller cach_controller(
        .clk(o_clk),
        .rst_x(o_rst_x),
        .i_rd_en(i_rd_en),
        .i_wr_en(i_wr_en),
        .i_addr(i_addr),
        .i_data(i_data),
        .o_data(o_data),
        .o_busy(o_busy),
        .i_mask(i_mask),
        .cach_hit(c_oe),
        .cach_we(c_we),
        .cach_addr(c_addr),
        .cach_wdata(c_idata),
        .cach_rdata(c_odata),
        .dram_stall(w_dram_stall),
        .dram_le(w_dram_le),
        .dram_we(w_dram_we),
        .dram_rdata(w_dram_odata),
        .dram_wdata(w_dram_idata),
        .dram_addr(w_dram_addr)
        );

    m_dram_cache#(28,128,`CACHE_SIZE/16) cache(o_clk, 1'b1, 1'b0, c_clr, c_we,
                                c_addr[31:4], c_idata, c_odata, c_oe);

    DRAM_Wrapper2 dram (
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
               // user interface ports
               .i_rd_en(w_dram_le),
               .i_wr_en(w_dram_we),
               .i_addr(w_dram_addr),
               .i_data(w_dram_idata),
               .o_init_calib_complete(o_init_calib_complete),
               .o_data(w_dram_odata),
               .o_busy(w_dram_stall),
               .i_mask(i_mask)
               );

endmodule
/**************************************************************************************************/
/**************************************************************************************************/
module DRAM_Wrapper2 #(
              parameter DDR2_DQ_WIDTH   = 16,
              parameter DDR2_DQS_WIDTH  = 2,
              parameter DDR2_ADDR_WIDTH = 13,
              parameter DDR2_BA_WIDTH   = 3,
              parameter DDR2_DM_WIDTH   = 2,
              parameter APP_ADDR_WIDTH  = 27,
              parameter APP_CMD_WIDTH   = 3,
              parameter APP_DATA_WIDTH  = 128,  // Note
              parameter APP_MASK_WIDTH  = 16)
    (
     // input clk, rst (active-low)
     input  wire                         mig_clk,
     input  wire                         mig_rst_x,
     // memory interface ports
     inout  wire [DDR2_DQ_WIDTH-1 : 0]   ddr2_dq,
     inout  wire [DDR2_DQS_WIDTH-1 : 0]  ddr2_dqs_n,
     inout  wire [DDR2_DQS_WIDTH-1 : 0]  ddr2_dqs_p,
     output wire [DDR2_ADDR_WIDTH-1 : 0] ddr2_addr,
     output wire [DDR2_BA_WIDTH-1 : 0]   ddr2_ba,
     output wire                         ddr2_ras_n,
     output wire                         ddr2_cas_n,
     output wire                         ddr2_we_n,
     output wire [0:0]                   ddr2_ck_p,
     output wire [0:0]                   ddr2_ck_n,
     output wire [0:0]                   ddr2_cke,
     output wire [0:0]                   ddr2_cs_n,
     output wire [DDR2_DM_WIDTH-1 : 0]   ddr2_dm,
     output wire [0:0]                   ddr2_odt,
     // output clk, rst (active-low)
     output wire                         o_clk,
     output wire                         o_rst_x,
     // user interface ports
     input  wire                         i_rd_en,
     input  wire                         i_wr_en,
     input  wire [31:0]                  i_addr,
     input  wire [31:0]                  i_data,
     output wire                         o_init_calib_complete,
     output wire [127:0]                 o_data,
     output wire                         o_busy,
     input  wire [3:0]                   i_mask);

    /***** store output data to registers in posedge clock cycle *****/

    wire [127:0]w_o_data;
    wire        w_o_busy;

    reg  [127:0]r_o_data = 0;
    reg         r_o_busy = 0;
    always @(posedge o_clk) begin
        r_o_data <= w_ctrl_data;
        r_o_busy <= w_o_busy;
    end

    assign o_data = r_o_data;
    assign o_busy = r_o_busy || r_le || r_we;

    /***** select load data by i_ctrl *****/

    reg  [3:0]  r_mask  = 0;
    reg  [31:0] r_iaddr = 0;
    wire [127:0]w_ctrl_data;

    reg  [31:0] r_wdata = 0;
    reg         r_le = 0;
    reg         r_we = 0;

    always @(posedge o_clk) begin
        //if(i_rd_en) begin
        r_mask  <= i_mask;
        r_iaddr <= i_addr;
        //end
        r_le <= i_rd_en;
        r_we <= i_wr_en;
        r_wdata <= i_data;
    end
    assign w_ctrl_data = w_o_data;

    wire [31:0] w_ctrl_iaddr = (r_we) ? {r_iaddr[31:2],2'b0} : {r_iaddr[31:4],4'b0};

    DRAM_con_witout_cache dram_con_witout_cache (
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
               // user interface ports
               .i_rd_en(r_le),
               .i_wr_en(r_we),
               .i_addr(w_ctrl_iaddr),
               .i_data(r_wdata),
               .o_init_calib_complete(o_init_calib_complete),
               .o_data(w_o_data),
               .o_busy(w_o_busy),
               .i_mask(r_mask)
               );

endmodule
/**************************************************************************************************/
`endif

/**************************************************************************************************/

/*** Single-port RAM with synchronous read                                                      ***/
module m_bram#(parameter WIDTH=32, ENTRY=256)(CLK, w_we, w_addr, w_idata, r_odata);
  input  wire                     CLK, w_we;
  input  wire [$clog2(ENTRY)-1:0] w_addr;
  input  wire         [WIDTH-1:0] w_idata;
  output reg          [WIDTH-1:0] r_odata;

  reg          [WIDTH-1:0]  mem [0:ENTRY-1];

  integer i;
  initial for (i=0;i<ENTRY;i=i+1) mem[i]=0;

  always  @(posedge  CLK)  begin
    if (w_we) mem[w_addr] <= w_idata;
    r_odata <= mem[w_addr];
  end
endmodule
/**************************************************************************************************/

/*** Dual-port RAM with synchronous read                                                        ***/
module m_bram2#(parameter WIDTH=32, ENTRY=256)
                (CLK, w_we, w_raddr, w_waddr, w_idata, w_odata1, w_odata2);
    input  wire                     CLK, w_we;
    input  wire [$clog2(ENTRY)-1:0] w_raddr, w_waddr;
    input  wire         [WIDTH-1:0] w_idata;
    output wire         [WIDTH-1:0] w_odata1, w_odata2;

    reg  [$clog2(ENTRY)-1:0] r_addr, r_addr2;

    reg          [WIDTH-1:0] mem [0:ENTRY-1];

    always  @(posedge  CLK)  begin
        if (w_we) mem[w_waddr] <= w_idata;
        r_addr2 <= w_waddr;
        r_addr <= w_raddr;
    end
    assign w_odata1 = mem[r_addr];
    assign w_odata2 = mem[r_addr2];
endmodule
/**************************************************************************************************/
/*** Single-port RAM with synchronous read with colum access                                    ***/
module m_col_bram#(parameter WIDTH=32, ENTRY=256)(CLK, w_we, w_addr, w_idata, w_odata);
    input  wire                     CLK;
    input  wire               [3:0] w_we;
    input  wire [$clog2(ENTRY)-1:0] w_addr;
    input  wire         [WIDTH-1:0] w_idata;
    output wire         [WIDTH-1:0] w_odata;

    //initial $readmemh(`MEMFILE, mem);

    (* ram_style = "block" *) reg [WIDTH-1:0] mem[0:ENTRY-1];
    reg [$clog2(ENTRY)-1:0] addr=0;
    always @(posedge CLK) begin
        if (w_we[0]) mem[w_addr][ 7: 0] <= w_idata[ 7: 0];
        if (w_we[1]) mem[w_addr][15: 8] <= w_idata[15: 8];
        if (w_we[2]) mem[w_addr][23:16] <= w_idata[23:16];
        if (w_we[3]) mem[w_addr][31:24] <= w_idata[31:24];
        addr <= w_addr;
    end
    assign w_odata = mem[addr];
endmodule
/**************************************************************************************************/
`ifndef SIM_MODE
module AsyncFIFO #(
			       parameter DATA_WIDTH  = 512,
			       parameter ADDR_WIDTH  = 8) // FIFO_DEPTH = 2^ADDR_WIDTH
    (
     input  wire                    wclk,
	 input  wire                    rclk,
     input  wire                    i_wrst_x,
     input  wire                    i_rrst_x,
	 input  wire                    i_wen,
	 input  wire [DATA_WIDTH-1 : 0] i_data,
     input  wire                    i_ren,
	 output wire [DATA_WIDTH-1 : 0] o_data,
	 output wire                    o_empty,
	 output wire                    o_full);

    reg  [DATA_WIDTH-1 : 0] afifo[(2**ADDR_WIDTH)-1 : 0];
    reg  [ADDR_WIDTH : 0]   waddr;
    reg  [ADDR_WIDTH : 0]   raddr;

    reg  [ADDR_WIDTH : 0]   raddr_gray1;
    reg  [ADDR_WIDTH : 0]   raddr_gray2;

    reg  [ADDR_WIDTH : 0]   waddr_gray1;
    reg  [ADDR_WIDTH : 0]   waddr_gray2;

    wire [DATA_WIDTH-1 : 0] data;

    wire [ADDR_WIDTH : 0]   raddr_gray;
    wire [ADDR_WIDTH : 0]   waddr_gray;

    wire [ADDR_WIDTH : 0]   raddr2;
    wire [ADDR_WIDTH : 0]   waddr2;

    genvar genvar_i;

    // output signals
    assign o_data  = data;
    assign o_empty = (raddr == waddr2);
    assign o_full  = (waddr[ADDR_WIDTH] != raddr2[ADDR_WIDTH]) &&
                     (waddr[ADDR_WIDTH-1 : 0] == raddr2[ADDR_WIDTH-1 : 0]);

    // binary code to gray code
    assign raddr_gray = raddr[ADDR_WIDTH : 0] ^ {1'b0, raddr[ADDR_WIDTH : 1]};
    assign waddr_gray = waddr[ADDR_WIDTH : 0] ^ {1'b0, waddr[ADDR_WIDTH : 1]};

    // gray code to binary code
    generate
	    for (genvar_i = 0; genvar_i <= ADDR_WIDTH; genvar_i = genvar_i + 1) begin
		    assign raddr2[genvar_i] = ^raddr_gray2[ADDR_WIDTH : genvar_i];
		    assign waddr2[genvar_i] = ^waddr_gray2[ADDR_WIDTH : genvar_i];
	    end
    endgenerate

    // double flopping read address before using it in write clock domain
    always @(posedge wclk) begin
	    if (!i_wrst_x) begin
		    raddr_gray1 <= 0;
		    raddr_gray2 <= 0;
	    end else begin
		    raddr_gray1 <= raddr_gray;
		    raddr_gray2 <= raddr_gray1;
	    end
    end

    // double flopping write address before using it in read clock domain
    always @(posedge rclk) begin
	    if (!i_rrst_x) begin
		    waddr_gray1 <= 0;
		    waddr_gray2 <= 0;
	    end else begin
		    waddr_gray1 <= waddr_gray;
		    waddr_gray2 <= waddr_gray1;
	    end
    end

    // read
    assign data = afifo[raddr[ADDR_WIDTH-1 : 0]];
    always @(posedge rclk) begin
	    if (!i_rrst_x) begin
		    raddr <= 0;
	    end else if (i_ren) begin
		    raddr <= raddr + 1;
	    end
    end

    // write
    always @(posedge wclk) begin
	    if (!i_wrst_x) begin
		    waddr <= 0;
	    end else if (i_wen) begin
		    afifo[waddr[ADDR_WIDTH-1 : 0]] <= i_data;
		    waddr <= waddr + 1;
	    end
    end

endmodule
`endif
/**************************************************************************************************/
/*** Simple Direct Mapped Cache Sync CLK for DRAM                                               ***/
/**************************************************************************************************/
module m_dram_cache#(parameter ADDR_WIDTH = 30, D_WIDTH = 32, ENTRY = 1024)
    (CLK, RST_X, w_flush, w_clr, w_we, w_addr, w_idata, w_odata, w_oe);
    input  wire                     CLK, RST_X;
    input  wire                     w_flush, w_we, w_clr;
    input  wire [ADDR_WIDTH-1:0]    w_addr;
    input  wire    [D_WIDTH-1:0]    w_idata;
    output wire    [D_WIDTH-1:0]    w_odata;
    output wire                     w_oe;             //output enable

    // index and tag
    reg  [$clog2(ENTRY)-1:0]                r_idx = 0;
    reg  [(ADDR_WIDTH - $clog2(ENTRY))-1:0] r_tag = 0;

    // index and tag
    wire                [$clog2(ENTRY)-1:0] w_idx;
    wire [(ADDR_WIDTH - $clog2(ENTRY))-1:0] w_tag;
    assign {w_tag, w_idx} = w_addr;

    wire                                            w_mwe = w_clr | w_we | !RST_X | w_flush;
    wire                      [$clog2(ENTRY)-1:0]   w_maddr = w_idx;
    wire [ADDR_WIDTH - $clog2(ENTRY) + D_WIDTH:0]   w_mwdata = w_we ? {1'b1, w_tag, w_idata} : 0;
    wire [ADDR_WIDTH - $clog2(ENTRY) + D_WIDTH:0]   w_modata;

    wire                                            w_mvalid;
    wire                     [$clog2(ENTRY)-1:0]    w_midx;
    wire      [(ADDR_WIDTH - $clog2(ENTRY))-1:0]    w_mtag;
    wire                           [D_WIDTH-1:0]    w_mdata;
    assign {w_mvalid, w_mtag, w_mdata} = w_modata;


    m_bram#((ADDR_WIDTH - $clog2(ENTRY) + D_WIDTH)+1, ENTRY)
        mem(CLK, w_mwe, w_maddr, w_mwdata, w_modata);

    assign w_odata  = w_mdata;
    assign w_oe     = (w_mvalid && w_mtag == r_tag);

    always  @(posedge  CLK)  begin
        r_tag <= w_tag;
        r_idx <= w_idx;
    end
endmodule // DMC
/**************************************************************************************************/
