
module sdcram_controller(
    input  wire        i_clk,
    input  wire        i_rst_x,
    input  wire [31:0] i_addr,
    output wire [31:0] o_rdata,
    input  wire [31:0] i_wdata,
    input  wire        i_ren,
    input  wire        i_wen,
    output wire        o_busy,
    // sdcram interface
    output wire [40:0] sdcram_addr,
    output wire        sdcram_ren,
    output wire [ 3:0] sdcram_wen,
    output wire [31:0] sdcram_wdata,
    input  wire [31:0] sdcram_rdata,
    input  wire        sdcram_busy
    );



    reg [31:0] r_addr   = 0;
    reg [31:0] r_rdata  = 0;
    reg [31:0] r_wdata  = 0;
    reg        r_sd_ren = 0;
    reg        r_sd_wen = 0;


    assign sdcram_ren  = r_sd_ren;
    assign sdcram_addr = {9'h0, r_addr};

    assign sdcram_wen   = {4{r_sd_wen}};
    assign sdcram_wdata = r_wdata;


    assign o_rdata = r_rdata;


    localparam  IDLE          = 0;
    localparam  WAIT_READ     = 1;
    localparam  WAIT_WRITE    = 2;
    localparam  SEND          = 3;
    localparam  WAIT_COMPLETE = 4;

    reg [2:0] r_state = IDLE;
    assign o_busy = (r_state != IDLE);

    always @ (posedge i_clk) begin
        if (!i_rst_x) begin
            r_addr   <= 0;
            r_rdata  <= 0;
            r_sd_ren <= 0;
            r_sd_wen <= 0;
            r_state  <= IDLE;
        end else begin
            case(r_state)
            IDLE: begin
                if (i_ren) begin
                    r_addr  <= i_addr;
                    r_state <= WAIT_READ;
                end else if (i_wen) begin
                    r_addr  <= i_addr;
                    r_state <= WAIT_WRITE;
                    r_wdata <= i_wdata;
                end
            end
            WAIT_READ: begin
                if (!sdcram_busy) begin
                    r_state  <= SEND;
                    r_sd_ren <= 1;
                end
            end
            WAIT_WRITE: begin
                if (!sdcram_busy) begin
                    r_state  <= SEND;
                    r_sd_wen <= 1;
                end
            end
            SEND: begin
                r_state <= WAIT_COMPLETE;
            end
            WAIT_COMPLETE: begin
                if (!sdcram_busy) begin
                    r_state  <= IDLE;
                    r_sd_ren <= 0;
                    r_sd_wen <= 0;
                    r_rdata  <= sdcram_rdata;
                end
            end
            endcase
        end
    end

endmodule

module SDPLOADER(
    input  wire        i_clk,
    input  wire        i_rst_x,
    output wire [31:0] o_addr,
    output wire [31:0] o_data,
    output wire        o_we,
    output wire        o_done,
    // sdcram interface
    output wire [40:0] sdcram_addr,
    output wire        sdcram_ren,
    input  wire [31:0] sdcram_rdata,
    input  wire        sdcram_busy,
    input  wire        w_dram_busy
    );



    reg [31:0] r_addr   = 0;
    reg [31:0] r_data   = 0;
    reg        r_we     = 0;
    reg        r_done   = 0;
    reg        r_sd_ren = 0;

    assign sdcram_ren = r_sd_ren;
    assign sdcram_addr = {9'h0, r_addr};

    assign o_addr = r_addr;
    assign o_data = r_data;

    assign o_done = r_done;

    localparam  IDLE      = 0;
    localparam  SEND      = 1;
    localparam  SD_WAIT   = 2;
    localparam  DRAM_WAIT = 3;

    reg [1:0] r_state = IDLE;
    assign o_we = r_state==DRAM_WAIT & !w_dram_busy;

    always @ (posedge i_clk) begin
        if (!i_rst_x) begin
            r_addr  <= 0;
            r_data  <= 0;
            r_we    <= 0;
            r_done  <= 0;
            r_state <= IDLE;
        end else begin
            case(r_state)
            IDLE: begin
                if (r_addr >= `BIN_BBL_SIZE) begin
                    r_done <= 1;
                end else if (!sdcram_busy & !r_done) begin
                    r_state  <= SEND;
                    r_sd_ren <= 1;
                end
            end
            SEND: begin
                r_state <= SD_WAIT;
            end
            SD_WAIT: begin
                if (!sdcram_busy) begin
                    r_state  <= DRAM_WAIT;
                    r_sd_ren <= 0;
                    r_data   <= sdcram_rdata;
                end
            end
            DRAM_WAIT: begin
                if (!w_dram_busy) begin
                    r_state <= IDLE;
                    r_addr <= r_addr + 4;
                end
            end
            endcase
        end
    end

endmodule



////////////////////////////////////////////////////////////////////////////////
// TokyoTech ArchLab
// Written by Kanamori
// 2020/10/02
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// SDCRAM (SD card RAM)
// Signal busy lasts at least one cycle
////////////////////////////////////////////////////////////////////////////////
module sdcram#(
    parameter CACHE_DEPTH = 8,
    parameter BLOCK_NUM = 1,
    parameter POLLING_CYCLES = 1024
    )(
    input  wire        i_sys_clk,
    input  wire        i_sys_rst,
    input  wire        i_sd_clk,
    input  wire        i_sd_rst,

    // for user interface
    input  wire [40:0] i_sdcram_addr,
    input  wire        i_sdcram_ren,
    input  wire [ 3:0] i_sdcram_wen,
    input  wire [31:0] i_sdcram_wdata,
    output wire [31:0] o_sdcram_rdata,
    output wire        o_sdcram_busy,

    // for debug
    output wire [ 2:0] sdcram_state,
    output wire [ 2:0] sdi_state,
    output wire [ 4:0] sdc_state,

    // for sd
    input  wire        sd_cd,
    output wire        sd_rst,
    output wire        sd_sclk,
    output wire        sd_cmd,
	inout  wire [ 3:0] sd_dat
    );

////////////////////////////////////////////////////////////////////////////////
//  SD interface, access sd card and cache ram
////////////////////////////////////////////////////////////////////////////////
    wire        sdi_i_req_en;
    wire        sdi_i_req_rw;
    wire [40:0] sdi_i_req_adr;
    wire        sdi_o_req_ack;

    wire        sdi_i_rep_ready;
    wire        sdi_o_rep_en;
    wire        sdi_o_rep_rw;
    wire [40:0] sdi_o_rep_adr;

    wire [40:0] sdi_cache_adr;
    wire [31:0] sdi_cache_din;
    wire [ 3:0] sdi_cache_wen;
    wire        sdi_cache_en;
    wire [31:0] sdi_cache_dout;

    sd_interface #(
        .BLOCK_NUM(BLOCK_NUM))
    sdi(
        .i_clk(i_sd_clk), // Max 50MHz.
        .i_rst(i_sd_rst),
        // for request
        .i_req_en(sdi_i_req_en), // HIGH indicate request is enable.
        .i_req_rw(sdi_i_req_rw), // Read (0) or Write (1)
        .i_req_adr(sdi_i_req_adr), // block address,
        //.o_req_ready(sdi_o_req_ready),
        .o_req_ack(sdi_o_req_ack),
        // for reply
        .i_rep_ready(sdi_i_rep_ready),
        .o_rep_en(sdi_o_rep_en), // HIGH indicate reply is enable.
        .o_rep_rw(sdi_o_rep_rw),
        .o_rep_adr(sdi_o_rep_adr), // block address
        // for debug
        .sdi_state(sdi_state),
        .sdc_state(sdc_state),
        // for cache
        .o_cache_adr(sdi_cache_adr),
        .o_cache_din(sdi_cache_din),
        .o_cache_we(sdi_cache_wen),
        .o_cache_en(sdi_cache_en),
        .i_cache_dout(sdi_cache_dout),
        // for sd
        .sd_cd(sd_cd),
        .sd_rst(sd_rst),
        .sd_sclk(sd_sclk),
        .sd_cmd(sd_cmd),
        .sd_dat(sd_dat)
    );

////////////////////////////////////////////////////////////////////////////////
//  send request from cache controller to sd interface and reply
////////////////////////////////////////////////////////////////////////////////
    wire        w_req_i_wen;
    wire [41:0] w_req_i_data;
    wire        w_req_i_ren;
    wire [41:0] w_req_o_data;
    wire        w_req_o_empty;
    wire        w_req_o_full;

    AsyncFIFO #(
        .DATA_WIDTH(1+41),
        .ADDR_WIDTH(2))
    req_fifo(
         .wclk(i_sys_clk),
    	 .rclk(i_sd_clk),
         .i_wrst_x(!i_sys_rst),
         .i_rrst_x(!i_sd_rst),
    	 .i_wen(w_req_i_wen),
    	 .i_data(w_req_i_data),
         .i_ren(w_req_i_ren),
    	 .o_data(w_req_o_data),
    	 .o_empty(w_req_o_empty),
    	 .o_full(w_req_o_full)
    );
    assign sdi_i_req_en = !w_req_o_empty;
    assign sdi_i_req_rw = w_req_o_data[41];
    assign sdi_i_req_adr= w_req_o_data[40:0];
    assign w_req_i_ren  = sdi_o_req_ack;

    wire        w_ack_i_wen;
    wire [41:0] w_ack_i_data;
    wire        w_ack_i_ren;
    wire [41:0] w_ack_o_data;
    wire        w_ack_o_empty;
    wire        w_ack_o_full;

    AsyncFIFO #(
        .DATA_WIDTH(1+41),
        .ADDR_WIDTH(2))
    ack_fifo(
         .wclk(i_sd_clk),
         .rclk(i_sys_clk),
         .i_wrst_x(!i_sd_rst),
         .i_rrst_x(!i_sys_rst),
         .i_wen(w_ack_i_wen),
         .i_data(w_ack_i_data),
         .i_ren(w_ack_i_ren),
         .o_data(w_ack_o_data),
         .o_empty(w_ack_o_empty),
         .o_full(w_ack_o_full)
    );

    assign sdi_i_rep_ready = !w_ack_o_full;
    assign w_ack_i_wen     = sdi_o_rep_en;
    assign w_ack_i_data    = {sdi_o_rep_rw, sdi_o_rep_adr};
////////////////////////////////////////////////////////////////////////////////
//  ram for cache
////////////////////////////////////////////////////////////////////////////////
    wire [40:0] cc_cache_adr;
    wire [ 3:0] cc_cache_wen;
    wire [31:0] cc_cache_din;
    wire [31:0] cc_cache_dout;
    wire        cc_cache_en;

    xilinx_true_dual_port_read_first_byte_write_2_clock_ram #(
      .NB_COL(4),                           // Specify number of columns (number of bytes)
      .COL_WIDTH(8),                        // Specify column width (byte width, typically 8 or 9)
      .RAM_DEPTH(128 * CACHE_DEPTH * BLOCK_NUM),                     // Specify RAM depth (number of entries)
      .RAM_PERFORMANCE("LOW_LATENCY"), // Select "HIGH_PERFORMANCE" or "LOW_LATENCY"
      .INIT_FILE("")                        // Specify name/location of RAM initialization file if using one (leave blank if not)
    ) sdcram_buf (
      .addra( cc_cache_adr[8+$clog2(CACHE_DEPTH)+$clog2(BLOCK_NUM):2]),   // Port A address bus, width determined from RAM_DEPTH
      .addrb(sdi_cache_adr[8+$clog2(CACHE_DEPTH)+$clog2(BLOCK_NUM):2]),   // Port B address bus, width determined from RAM_DEPTH
      .dina(cc_cache_din),     // Port A RAM input data, width determined from NB_COL*COL_WIDTH
      .dinb(sdi_cache_din),     // Port B RAM input data, width determined from NB_COL*COL_WIDTH
      .clka(i_sys_clk),     // Port A clock
      .clkb(i_sd_clk),     // Port B clock
      .wea(cc_cache_wen),       // Port A write enable, width determined from NB_COL
      .web(sdi_cache_wen),       // Port B write enable, width determined from NB_COL
      .ena(cc_cache_en),       // Port A RAM Enable, for additional power savings, disable port when not in use
      .enb(sdi_cache_en),       // Port B RAM Enable, for additional power savings, disable port when not in use
      .rsta(i_sys_rst),     // Port A output reset (does not affect memory contents)
      .rstb(i_sd_rst),     // Port B output reset (does not affect memory contents)
      .regcea(0), // Port A output register enable
      .regceb(0), // Port B output register enable
      .douta(cc_cache_dout),   // Port A RAM output data, width determined from NB_COL*COL_WIDTH
      .doutb(sdi_cache_dout)    // Port B RAM output data, width determined from NB_COL*COL_WIDTH
    );

////////////////////////////////////////////////////////////////////////////////
//  cache controller
////////////////////////////////////////////////////////////////////////////////
    localparam INIT = 3'b000;
    localparam IDLE = 3'b001;
    localparam WRITE_BLOCK = 3'b010;
    localparam READ_BLOCK = 3'b011;
    localparam SET_TAG = 3'b100;
    localparam WAIT = 3'b101;
    localparam POLLING = 3'b110;
    localparam CLEAN_TAG = 3'b111;

    reg  [ 8:0] r_states = {3{INIT}};
    wire [ 2:0] w_state = r_states[2:0];

    reg         r_req_wen = 0;
    reg  [41:0] r_req_data;
    reg  [41:0] r_rep_data;
    reg         r_ct_dirty = 0;
    reg  [40:0] r_i_sdcram_addr = 0;
    reg  [31:0] r_i_sdcram_wdata = 0;
    reg  [ 3:0] r_i_sdcram_wen = 0;
    // for polling
    reg  [$clog2(POLLING_CYCLES)-1:0] r_pcnt = 0;
    reg  [$clog2(CACHE_DEPTH)-1   :0] r_pblk = 0;
    reg  [40-$clog2(CACHE_DEPTH)-9-$clog2(BLOCK_NUM):0] r_ptag = 0;

    wire w_cont_ready = (w_state != INIT) && (w_state != WAIT);

    wire [40:0] ct_i_addr;
    wire        ct_i_dirty;
    wire        ct_i_wen;
    wire        ct_o_valid;
    wire        ct_o_hit;
    wire        ct_o_dirty;
    wire [40:0] ct_o_addr;
    wire        ct_o_ready;

    // write first, if write enable, then valid and hit
    cache_tags#(
        .DEPTH(CACHE_DEPTH),
        .BLOCK_NUM(BLOCK_NUM)
    )ct_0(
        .i_clk(i_sys_clk),
        .i_rst(i_sys_rst),
        .i_data(ct_i_addr),
        .i_dirty(ct_i_dirty),
        .i_wen(ct_i_wen),
        .o_valid(ct_o_valid),
        .o_hit(ct_o_hit),
        .o_dirty(ct_o_dirty),
        .o_data(ct_o_addr),
        .o_ready(ct_o_ready)
    );

    wire [9+$clog2(BLOCK_NUM)-1:0] w_blk_zeros = 0;
    assign ct_i_addr  = ((w_state == POLLING) || (w_state == CLEAN_TAG)) ? {r_ptag, r_pblk, w_blk_zeros} :
                         (w_state == SET_TAG) ? r_i_sdcram_addr : i_sdcram_addr;
    assign ct_i_dirty = (w_state == CLEAN_TAG) ? 0 : r_ct_dirty;
    assign ct_i_wen   = (w_state == SET_TAG) || (w_state == CLEAN_TAG);
    wire  ct_state   = {ct_o_valid, ct_o_hit, ct_o_dirty};

    assign cc_cache_adr = r_i_sdcram_addr;
    assign cc_cache_wen = ((w_state == SET_TAG) & (ct_o_hit & ct_o_valid)) ? r_i_sdcram_wen : 0;
    assign cc_cache_din = r_i_sdcram_wdata;
    assign cc_cache_en  = (w_state != WAIT);

    assign o_sdcram_rdata = cc_cache_dout;
    assign o_sdcram_busy  = (w_state != IDLE);

    assign sdcram_state = w_state;

    wire [40:0] w_new_sd_addr = {ct_i_addr[40:9+$clog2(BLOCK_NUM)], w_blk_zeros};
    wire [40:0] w_old_sd_addr = {ct_o_addr[40:9+$clog2(BLOCK_NUM)], w_blk_zeros};

    wire        w_sdi_ready   = !w_req_o_full;
    wire        w_sdi_ack     = !w_ack_o_empty;

    assign w_req_i_data = r_req_data;
    assign w_req_i_wen  = r_req_wen;
    assign w_ack_i_ren  = w_sdi_ack & (w_state==WAIT);

    always @ (posedge i_sys_clk) begin
        if(i_sys_rst) begin
            r_states <= {3{INIT}};
        end
        else begin
        case(w_state)
        INIT: begin
            if(w_sdi_ready & ct_o_ready) begin
                r_states   <= {3{IDLE}};
            end
            r_req_wen  <= 0;
            r_req_data <= 0;
            r_rep_data <= 0;
            r_ct_dirty <= 0;
            r_i_sdcram_addr <= 0;
            r_i_sdcram_wdata <= 0;
            r_i_sdcram_wen <= 0;
        end
        IDLE: begin
            r_ct_dirty <= (i_sdcram_wen) ? 1 : 0;
            r_i_sdcram_addr <= i_sdcram_addr;
            r_i_sdcram_wdata<= i_sdcram_wdata;
            r_i_sdcram_wen  <= i_sdcram_wen;
            // Read
            if(i_sdcram_ren) begin
                case({ct_o_valid, ct_o_hit, ct_o_dirty})
                3'b000: begin // 1.read from sd
                    r_states <= {IDLE, SET_TAG, READ_BLOCK};
                end
                3'b001: begin // 1.read from sd
                    r_states <= {IDLE, SET_TAG, READ_BLOCK};
                end
                3'b010: begin // 1.read from sd
                    r_states <= {IDLE, SET_TAG, READ_BLOCK};
                end
                3'b011: begin // 1.read from sd
                    r_states <= {IDLE, SET_TAG, READ_BLOCK};
                end
                3'b100: begin // 1.read from sd
                    r_states <= {IDLE, SET_TAG, READ_BLOCK};
                end
                3'b101: begin // 1. write back to sd, 2. read from sd
                    r_states <= {SET_TAG, READ_BLOCK, WRITE_BLOCK};
                end
                3'b110: begin // cache hit
                    r_states <= {{2{IDLE}}, SET_TAG};
                end
                3'b111: begin // cache hit
                    r_states <= {{2{IDLE}}, SET_TAG};
                end
                endcase
                r_pcnt <= 0;
            end
            // write
            else if(i_sdcram_wen!=0) begin
                case({ct_o_valid, ct_o_hit, ct_o_dirty})
                3'b000: begin // read from sd
                    r_states <= {IDLE, SET_TAG, READ_BLOCK};
                end
                3'b001: begin // read from sd
                    r_states <= {IDLE, SET_TAG, READ_BLOCK};
                end
                3'b010: begin // read from sd
                    r_states <= {IDLE, SET_TAG, READ_BLOCK};
                end
                3'b011: begin // read from sd
                    r_states <= {IDLE, SET_TAG, READ_BLOCK};
                end
                3'b100: begin // 1. read from sd,
                    r_states <= {IDLE, SET_TAG, READ_BLOCK};
                end
                3'b101: begin // 1. write back to sd, 2. read from sd, 3.write to cache
                    r_states <= {SET_TAG, READ_BLOCK, WRITE_BLOCK};
                end
                3'b110: begin // 1. write to cache before update tag
                    r_states <= {IDLE, IDLE, SET_TAG};
                end
                3'b111: begin // 1. write to cache
                    r_states <= {{2{IDLE}}, SET_TAG};
                end
                endcase
                r_pcnt <= 0;
            end
            // write back
            else if(r_pcnt == (POLLING_CYCLES-1)) begin
                r_pcnt   <= 0;
                r_pblk   <= r_pblk + 1;
                r_ptag   <= 0;
                r_states <= {IDLE, CLEAN_TAG, POLLING};
            end
            else begin
                r_pcnt <= r_pcnt + 1;
            end

        end
        WRITE_BLOCK: begin
            if(w_sdi_ready) begin
                r_states[2:0] <= WAIT;
                r_req_wen <= 1;
                r_req_data<= {1'b1, w_old_sd_addr};
            end
        end
        READ_BLOCK: begin
            if(w_sdi_ready) begin
                r_states[2:0] <= WAIT;
                r_req_wen  <= 1;
                r_req_data <= {1'b0, w_new_sd_addr};
            end
        end
        WAIT: begin
            r_req_wen  <= 0;
            r_req_data <= 0;
            if(w_sdi_ack) begin
                r_states   <= {IDLE, r_states[8:3]};
                r_rep_data <= w_ack_o_data;
            end
        end
        SET_TAG: begin
            r_states  <= {IDLE, r_states[8:3]};
        end
        POLLING: begin
            // not necessary to wirte back
            if(!(ct_o_valid & ct_o_dirty)) begin
                r_states <= {3{IDLE}};
            end
            else if(w_sdi_ready)begin
                r_states[2:0] <= WAIT;
                r_ptag        <= w_old_sd_addr[40:9+$clog2(CACHE_DEPTH)+$clog2(BLOCK_NUM)];
                r_req_wen     <= 1;
                r_req_data    <= {1'b1, w_old_sd_addr};
            end
        end
        CLEAN_TAG: begin
            r_states <= {IDLE, r_states[8:3]};
        end
        endcase
        end
    end
endmodule

////////////////////////////////////////////////////////////////////////////////
module cache_tags#(
    parameter DEPTH = 8,
    parameter BLOCK_NUM = 1
    )(
    input  wire        i_clk,
    input  wire        i_rst,
    input  wire [40:0] i_data,
    input  wire        i_dirty,
    input  wire        i_wen,
    output wire        o_valid,
    output wire        o_hit,
    output wire        o_dirty,
    output wire [40:0] o_data,
    output wire        o_ready
    );

    localparam  WIDTH = 41-9-$clog2(DEPTH)-$clog2(BLOCK_NUM);

    // valid(1bit) + dirty(1bit) + tag(WIDTH bit)
    reg             valids [DEPTH-1:0];
    reg             dirtys [DEPTH-1:0];
    reg [WIDTH-1:0] tags   [DEPTH-1:0];

    wire [$clog2(DEPTH)-1:0] i_tag_adr = i_data[9+$clog2(DEPTH)+$clog2(BLOCK_NUM)-1:9+$clog2(BLOCK_NUM)];
    wire [WIDTH-1:0]         i_tag     = i_data[40:9+$clog2(DEPTH)+$clog2(BLOCK_NUM)];
    wire [8+$clog2(BLOCK_NUM):0] i_offset  = i_data[8+$clog2(BLOCK_NUM):0];

    wire [WIDTH-1:0] tag   = tags  [i_tag_adr];
    wire             valid = valids[i_tag_adr];
    wire             dirty = dirtys[i_tag_adr];

    assign o_valid = (i_wen) ? 1 : valid;
    assign o_hit   = (i_wen) ? 1 : (i_tag==tag);
    assign o_dirty = (i_wen) ? i_dirty : dirty;
    assign o_data  = {tag, i_tag_adr, i_offset};

    reg [$clog2(DEPTH):0] r_rst_cnt = 0;
    reg                   r_rst_det = 0;

    wire w_rst_fin = r_rst_cnt[$clog2(DEPTH)];
    wire w_wen = i_wen | r_rst_det;

    wire [$clog2(DEPTH)-1:0] w_tag_adr = (r_rst_det) ? r_rst_cnt[$clog2(DEPTH)-1:0] : i_tag_adr;

    assign o_ready = !r_rst_det;

    always @ (posedge i_clk) begin
        r_rst_det <= (i_rst) ? 1 : (w_rst_fin) ? 0 : r_rst_det;
        r_rst_cnt <= (i_rst) ? 0 : (r_rst_det) ? r_rst_cnt + 1 : 0;
        if(w_wen) begin
            valids[w_tag_adr] <= (r_rst_det) ? 0 : 1;
            dirtys[w_tag_adr] <= (r_rst_det) ? 0 : i_dirty;
            tags  [w_tag_adr] <= (r_rst_det) ? 0 : i_tag;
        end
    end
endmodule

////////////////////////////////////////////////////////////////////////////////
// wrapper of sd_controller,
////////////////////////////////////////////////////////////////////////////////
module sd_interface#(
    parameter BLOCK_NUM = 1
    )(
    input  wire        i_clk, // Max 50MHz.
    input  wire        i_rst,
    // for request queue
    input  wire        i_req_en, // HIGH indicate request is enable.
    input  wire        i_req_rw, // Read (0) or Write (1)
    input  wire [40:0] i_req_adr, // block address,
    //output wire        o_req_ready,
    output wire        o_req_ack,
    // for reply queue
    input  wire        i_rep_ready,
    output wire        o_rep_en, // HIGH indicate reply is enable.
    output wire        o_rep_rw,
    output wire [40:0] o_rep_adr, // block address
    // for debug
    output wire [ 2:0] sdi_state,
    output wire [ 4:0] sdc_state,

    // for cache
    output wire [40:0] o_cache_adr,
    output wire [31:0] o_cache_din,
    output wire [ 3:0] o_cache_we,
    output wire        o_cache_en,
    input  wire [31:0] i_cache_dout,

    // for sd
    input  wire        sd_cd,
    output wire        sd_rst,
    output wire        sd_sclk,
    output wire        sd_cmd,
	inout  wire [ 3:0] sd_dat
    );

////////////////////////////////////////////////////////////////////////////////
    assign sd_dat[1] = 1;
    assign sd_dat[2] = 1;
    assign sd_rst    = 0;

    wire        sdc_rd;
    wire [ 7:0] sdc_dout;
    wire        sdc_byte_available;
    wire        sdc_wr;
    wire [ 7:0] sdc_din;
    wire        sdc_ready_for_next_byte;
    wire        sdc_ready;
    wire [40:0] sdc_address;
    wire [ 4:0] sdc_state;

    sd_controller sdc(
        .i_clk(i_clk),
        .i_rst(i_rst),
        .o_ready(sdc_ready),

        .i_ren(sdc_rd),
        .o_data(sdc_dout),
        .o_data_en(sdc_byte_available),

        .i_wen(sdc_wr),
        .i_data(sdc_din),
        .o_data_ready(sdc_ready_for_next_byte),

        .i_blk_num(BLOCK_NUM),
        .i_adr(sdc_address[40:9]),

        .o_state(sdc_state),

        .cs(sd_dat[3]),
        .mosi(sd_cmd), // Connect to SD_CMD.
        .miso(sd_dat[0]), // Connect to SD_DAT[0].
        .sclk(sd_sclk)
    );

////////////////////////////////////////////////////////////////////////////////

    localparam  BLOCK_SIZE = 512;

    localparam  SDC_INIT = 0;
    localparam  SDC_IDLE = 1;
    localparam  SDC_READ = 2;
    localparam  SDC_WRITE= 3;
    localparam  SDC_REPLY= 4;

    reg  [ 2:0] r_state   = SDC_INIT;

    reg         r_sdc_rd;
    reg         r_sdc_wr;

    reg         r_rep_en;
    reg         r_rep_rw;
    reg  [40:0] r_rep_adr;

    reg  [40:0] r_o_cache_adr = 0;
    reg  [ 3:0] r_o_cache_we  = 0;

    assign sdc_rd  = r_sdc_rd;
    assign sdc_din =  w_sdc_din[7:0];
    assign sdc_wr  = r_sdc_wr;
    assign sdc_address = r_rep_adr;

    assign o_req_ack = (r_state == SDC_IDLE) & i_req_en;

    assign o_rep_en  = r_rep_en;
    assign o_rep_rw  = r_rep_rw;
    assign o_rep_adr = r_rep_adr;

    assign o_cache_adr = r_o_cache_adr;
    assign o_cache_we  = (sdc_byte_available) ? r_o_cache_we : 0;
    assign o_cache_din = {4{sdc_dout}};
    assign o_cache_en  = (r_state == SDC_READ) | (r_state == SDC_WRITE);

    assign sdi_state = r_state;

    wire [31:0] w_sdc_din = {i_cache_dout >> {r_o_cache_adr[1:0], 3'b000}};

    always @ (posedge i_clk) begin
        if(i_rst) begin
            r_state <= SDC_INIT;
        end
        else begin
        case(r_state)
        SDC_INIT:begin
            if(sdc_ready) begin
                r_state <= SDC_IDLE;
            end
            r_sdc_rd  <= 0;
            r_sdc_wr  <= 0;

            r_rep_en  <= 0;
            r_rep_rw  <= 0;
            r_rep_adr <= 0;

            r_o_cache_adr <= 0;
            r_o_cache_we  <= 0;
        end
        SDC_IDLE:begin
            // 0 - Read request, 1 - Write request
            if(i_req_en) begin
                r_state  <= (i_req_rw) ? SDC_WRITE : SDC_READ;
                r_sdc_rd <= !i_req_rw;
                r_sdc_wr <=  i_req_rw;

                r_rep_adr <= i_req_adr;
                r_rep_rw  <= i_req_rw;

                r_o_cache_adr <= i_req_adr;
                r_o_cache_we  <= (i_req_rw) ? 0 : 4'b0001;
            end
            r_rep_en <= 0;
        end
        SDC_READ:begin
            if(sdc_byte_available) begin
                r_o_cache_adr <= r_o_cache_adr + 1;
                r_o_cache_we  <= {r_o_cache_we[2:0], r_o_cache_we[3]};
                r_sdc_rd <= 0;
            end
            else if(sdc_ready & !r_sdc_rd) begin
                r_state <= SDC_REPLY;
            end
        end
        SDC_WRITE:begin
            if(sdc_ready_for_next_byte) begin
                r_o_cache_adr <= r_o_cache_adr + 1;
                r_sdc_wr <= 0;
            end
            else if(sdc_ready & !r_sdc_wr) begin
                r_state <= SDC_REPLY;
            end
        end
        SDC_REPLY: begin
            if(i_rep_ready) begin
                r_rep_en <= 1;
                r_state  <= SDC_IDLE;
            end
            else begin
                r_rep_en <= 0;
            end
            r_o_cache_we <= 0;
        end
        endcase
        end
    end

endmodule // sd_interface




////////////////////////////////////////////////////////////////////////////////
//  Xilinx True Dual Port RAM Byte Write, Write First Dual Clock RAM
//  This code implements a parameterizable true dual port memory (both ports can read and write).
//  The behavior of this RAM is when data is written, the new memory contents at the write
//  address are presented on the output port.

module xilinx_true_dual_port_write_first_byte_write_2_clock_ram #(
  parameter NB_COL = 4,                           // Specify number of columns (number of bytes)
  parameter COL_WIDTH = 9,                        // Specify column width (byte width, typically 8 or 9)
  parameter RAM_DEPTH = 1024,                     // Specify RAM depth (number of entries)
  parameter RAM_PERFORMANCE = "LOW_LATENCY", // Select "HIGH_PERFORMANCE" or "LOW_LATENCY"
  parameter INIT_FILE = ""                        // Specify name/location of RAM initialization file if using one (leave blank if not)
) (
  input wire [clogb2(RAM_DEPTH-1)-1:0] addra,   // Port A address bus, width determined from RAM_DEPTH
  input wire [clogb2(RAM_DEPTH-1)-1:0] addrb,   // Port B address bus, width determined from RAM_DEPTH
  input wire [(NB_COL*COL_WIDTH)-1:0] dina,   // Port A RAM input data
  input wire [(NB_COL*COL_WIDTH)-1:0] dinb,   // Port B RAM input data
  input wire clka,                            // Port A clock
  input wire clkb,                            // Port B clock
  input wire [NB_COL-1:0] wea,                // Port A write enable
  input wire [NB_COL-1:0] web,                // Port B write enable
  input wire ena,                             // Port A RAM Enable, for additional power savings, disable BRAM when not in use
  input wire enb,                             // Port B RAM Enable, for additional power savings, disable BRAM when not in use
  input wire rsta,                            // Port A output reset (does not affect memory contents)
  input wire rstb,                            // Port B output reset (does not affect memory contents)
  input wire regcea,                          // Port A output register enable
  input wire regceb,                          // Port B output register enable
  output wire [(NB_COL*COL_WIDTH)-1:0] douta, // Port A RAM output data
  output wire [(NB_COL*COL_WIDTH)-1:0] doutb  // Port B RAM output data
);

  reg [(NB_COL*COL_WIDTH)-1:0] BRAM [RAM_DEPTH-1:0];
  reg [(NB_COL*COL_WIDTH)-1:0] ram_data_a = {(NB_COL*COL_WIDTH){1'b0}};
  reg [(NB_COL*COL_WIDTH)-1:0] ram_data_b = {(NB_COL*COL_WIDTH){1'b0}};

  // The following code either initializes the memory values to a specified file or to all zeros to match hardware
  generate
    if (INIT_FILE != "") begin: use_init_file
      initial
        $readmemh(INIT_FILE, BRAM, 0, RAM_DEPTH-1);
    end else begin: init_bram_to_zero
      integer ram_index;
      initial
        for (ram_index = 0; ram_index < RAM_DEPTH; ram_index = ram_index + 1)
          BRAM[ram_index] = {(NB_COL*COL_WIDTH){1'b0}};
    end
  endgenerate

  generate
  genvar i;
     for (i = 0; i < NB_COL; i = i+1) begin: byte_write
       always @(posedge clka)
         if (ena)
           if (wea[i]) begin
             BRAM[addra][(i+1)*COL_WIDTH-1:i*COL_WIDTH] <= dina[(i+1)*COL_WIDTH-1:i*COL_WIDTH];
             ram_data_a[(i+1)*COL_WIDTH-1:i*COL_WIDTH] <= dina[(i+1)*COL_WIDTH-1:i*COL_WIDTH];
           end else begin
             ram_data_a[(i+1)*COL_WIDTH-1:i*COL_WIDTH] <= BRAM[addra][(i+1)*COL_WIDTH-1:i*COL_WIDTH];
           end

       always @(posedge clkb)
         if (enb)
           if (web[i]) begin
             BRAM[addrb][(i+1)*COL_WIDTH-1:i*COL_WIDTH] <= dinb[(i+1)*COL_WIDTH-1:i*COL_WIDTH];
             ram_data_b[(i+1)*COL_WIDTH-1:i*COL_WIDTH] <= dinb[(i+1)*COL_WIDTH-1:i*COL_WIDTH];
           end else begin
             ram_data_b[(i+1)*COL_WIDTH-1:i*COL_WIDTH] <= BRAM[addrb][(i+1)*COL_WIDTH-1:i*COL_WIDTH];
           end
     end
  endgenerate

  //  The following code generates HIGH_PERFORMANCE (use output register) or LOW_LATENCY (no output register)
  generate
    if (RAM_PERFORMANCE == "LOW_LATENCY") begin: no_output_register

      // The following is a 1 clock cycle read latency at the cost of a longer clock-to-out timing
       assign douta = ram_data_a;
       assign doutb = ram_data_b;

    end else begin: output_register

      // The following is a 2 clock cycle read latency with improve clock-to-out timing

      reg [(NB_COL*COL_WIDTH)-1:0] douta_reg = {(NB_COL*COL_WIDTH){1'b0}};
      reg [(NB_COL*COL_WIDTH)-1:0] doutb_reg = {(NB_COL*COL_WIDTH){1'b0}};

      always @(posedge clka)
        if (rsta)
          douta_reg <= {(NB_COL*COL_WIDTH){1'b0}};
        else if (regcea)
          douta_reg <= ram_data_a;

      always @(posedge clkb)
        if (rstb)
          doutb_reg <= {(NB_COL*COL_WIDTH){1'b0}};
        else if (regceb)
          doutb_reg <= ram_data_b;

      assign douta = douta_reg;
      assign doutb = doutb_reg;

    end
  endgenerate

  //  The following function calculates the address width based on specified RAM depth
  function integer clogb2;
    input integer depth;
      for (clogb2=0; depth>0; clogb2=clogb2+1)
        depth = depth >> 1;
  endfunction

endmodule

// The following is an instantiation template for xilinx_true_dual_port_write_first_byte_write_2_clock_ram
/*
  //  Xilinx True Dual Port RAM Byte Write Write-First Dual Clock RAM
  xilinx_true_dual_port_write_first_byte_write_2_clock_ram #(
    .NB_COL(4),                           // Specify number of columns (number of bytes)
    .COL_WIDTH(9),                        // Specify column width (byte width, typically 8 or 9)
    .RAM_DEPTH(1024),                     // Specify RAM depth (number of entries)
    .RAM_PERFORMANCE("HIGH_PERFORMANCE"), // Select "HIGH_PERFORMANCE" or "LOW_LATENCY"
    .INIT_FILE("")                        // Specify name/location of RAM initialization file if using one (leave blank if not)
  ) your_instance_name (
    .addra(addra),   // Port A address bus, width determined from RAM_DEPTH
    .addrb(addrb),   // Port B address bus, width determined from RAM_DEPTH
    .dina(dina),     // Port A RAM input data, width determined from NB_COL*COL_WIDTH
    .dinb(dinb),     // Port B RAM input data, width determined from NB_COL*COL_WIDTH
    .clka(clka),     // Port A clock
    .clkb(clkb),     // Port B clock
    .wea(wea),       // Port A write enable, width determined from NB_COL
    .web(web),       // Port B write enable, width determined from NB_COL
    .ena(ena),       // Port A RAM Enable, for additional power savings, disable port when not in use
    .enb(enb),       // Port B RAM Enable, for additional power savings, disable port when not in use
    .rsta(rsta),     // Port A output reset (does not affect memory contents)
    .rstb(rstb),     // Port B output reset (does not affect memory contents)
    .regcea(regcea), // Port A output register enable
    .regceb(regceb), // Port B output register enable
    .douta(douta),   // Port A RAM output data, width determined from NB_COL*COL_WIDTH
    .doutb(doutb)    // Port B RAM output data, width determined from NB_COL*COL_WIDTH
  );
*/

////////////////////////////////////////////////////////////////////////////////


//  Xilinx True Dual Port RAM Byte Write Read First Dual Clock RAM
//  This code implements a parameterizable true dual port memory (both ports can read and write).
//  The behavior of this RAM is when data is written, the prior memory contents at the write
//  address are presented on the output port.

module xilinx_true_dual_port_read_first_byte_write_2_clock_ram #(
  parameter NB_COL = 4,                           // Specify number of columns (number of bytes)
  parameter COL_WIDTH = 9,                        // Specify column width (byte width, typically 8 or 9)
  parameter RAM_DEPTH = 1024,                     // Specify RAM depth (number of entries)
  parameter RAM_PERFORMANCE = "HIGH_PERFORMANCE", // Select "HIGH_PERFORMANCE" or "LOW_LATENCY"
  parameter INIT_FILE = ""                        // Specify name/location of RAM initialization file if using one (leave blank if not)
) (
  input wire [clogb2(RAM_DEPTH-1)-1:0] addra,   // Port A address bus, width determined from RAM_DEPTH
  input wire [clogb2(RAM_DEPTH-1)-1:0] addrb,   // Port B address bus, width determined from RAM_DEPTH
  input wire [(NB_COL*COL_WIDTH)-1:0] dina,   // Port A RAM input data
  input wire [(NB_COL*COL_WIDTH)-1:0] dinb,   // Port B RAM input data
  input wire clka,                            // Port A clock
  input wire clkb,                            // Port B clock
  input wire [NB_COL-1:0] wea,                // Port A write enable
  input wire [NB_COL-1:0] web,                // Port B write enable
  input wire ena,                             // Port A RAM Enable, for additional power savings, disable port when not in use
  input wire enb,                             // Port B RAM Enable, for additional power savings, disable port when not in use
  input wire rsta,                            // Port A output reset (does not affect memory contents)
  input wire rstb,                            // Port B output reset (does not affect memory contents)
  input wire regcea,                          // Port A output register enable
  input wire regceb,                          // Port B output register enable
  output wire [(NB_COL*COL_WIDTH)-1:0] douta, // Port A RAM output data
  output wire [(NB_COL*COL_WIDTH)-1:0] doutb  // Port B RAM output data
);

  reg [(NB_COL*COL_WIDTH)-1:0] BRAM [RAM_DEPTH-1:0];
  reg [(NB_COL*COL_WIDTH)-1:0] ram_data_a = {(NB_COL*COL_WIDTH){1'b0}};
  reg [(NB_COL*COL_WIDTH)-1:0] ram_data_b = {(NB_COL*COL_WIDTH){1'b0}};

  // The following code either initializes the memory values to a specified file or to all zeros to match hardware
  generate
    if (INIT_FILE != "") begin: use_init_file
      initial
        $readmemh(INIT_FILE, BRAM, 0, RAM_DEPTH-1);
    end else begin: init_bram_to_zero
      integer ram_index;
      initial
        for (ram_index = 0; ram_index < RAM_DEPTH; ram_index = ram_index + 1)
          BRAM[ram_index] = {(NB_COL*COL_WIDTH){1'b0}};
    end
  endgenerate

  always @(posedge clka)
    if (ena) begin
      ram_data_a <= BRAM[addra];
    end

  always @(posedge clkb)
    if (enb) begin
      ram_data_b <= BRAM[addrb];
    end

  generate
  genvar i;
     for (i = 0; i < NB_COL; i = i+1) begin: byte_write
       always @(posedge clka)
         if (ena)
           if (wea[i])
             BRAM[addra][(i+1)*COL_WIDTH-1:i*COL_WIDTH] <= dina[(i+1)*COL_WIDTH-1:i*COL_WIDTH];
       always @(posedge clkb)
         if (enb)
           if (web[i])
             BRAM[addrb][(i+1)*COL_WIDTH-1:i*COL_WIDTH] <= dinb[(i+1)*COL_WIDTH-1:i*COL_WIDTH];
end
  endgenerate

  //  The following code generates HIGH_PERFORMANCE (use output register) or LOW_LATENCY (no output register)
  generate
    if (RAM_PERFORMANCE == "LOW_LATENCY") begin: no_output_register

      // The following is a 1 clock cycle read latency at the cost of a longer clock-to-out timing
       assign douta = ram_data_a;
       assign doutb = ram_data_b;

    end else begin: output_register

      // The following is a 2 clock cycle read latency with improve clock-to-out timing

      reg [(NB_COL*COL_WIDTH)-1:0] douta_reg = {(NB_COL*COL_WIDTH){1'b0}};
      reg [(NB_COL*COL_WIDTH)-1:0] doutb_reg = {(NB_COL*COL_WIDTH){1'b0}};

      always @(posedge clka)
        if (rsta)
          douta_reg <= {(NB_COL*COL_WIDTH){1'b0}};
        else if (regcea)
          douta_reg <= ram_data_a;

      always @(posedge clkb)
        if (rstb)
          doutb_reg <= {(NB_COL*COL_WIDTH){1'b0}};
        else if (regceb)
          doutb_reg <= ram_data_b;

      assign douta = douta_reg;
      assign doutb = doutb_reg;

    end
  endgenerate

  //  The following function calculates the address width based on specified RAM depth
  function integer clogb2;
    input integer depth;
      for (clogb2=0; depth>0; clogb2=clogb2+1)
        depth = depth >> 1;
  endfunction

endmodule

// The following is an instantiation template for xilinx_true_dual_port_read_first_byte_write_2_clock_ram
/*
  //  Xilinx True Dual Port RAM Byte Write Read First Dual Clock RAM
  xilinx_true_dual_port_read_first_byte_write_2_clock_ram #(
    .NB_COL(4),                           // Specify number of columns (number of bytes)
    .COL_WIDTH(9),                        // Specify column width (byte width, typically 8 or 9)
    .RAM_DEPTH(1024),                     // Specify RAM depth (number of entries)
    .RAM_PERFORMANCE("HIGH_PERFORMANCE"), // Select "HIGH_PERFORMANCE" or "LOW_LATENCY"
    .INIT_FILE("")                        // Specify name/location of RAM initialization file if using one (leave blank if not)
  ) your_instance_name (
    .addra(addra),   // Port A address bus, width determined from RAM_DEPTH
    .addrb(addrb),   // Port B address bus, width determined from RAM_DEPTH
    .dina(dina),     // Port A RAM input data, width determined from NB_COL*COL_WIDTH
    .dinb(dinb),     // Port B RAM input data, width determined from NB_COL*COL_WIDTH
    .clka(clka),     // Port A clock
    .clkb(clkb),     // Port B clock
    .wea(wea),       // Port A write enable, width determined from NB_COL
    .web(web),       // Port B write enable, width determined from NB_COL
    .ena(ena),       // Port A RAM Enable, for additional power savings, disable port when not in use
    .enb(enb),       // Port B RAM Enable, for additional power savings, disable port when not in use
    .rsta(rsta),     // Port A output reset (does not affect memory contents)
    .rstb(rstb),     // Port B output reset (does not affect memory contents)
    .regcea(regcea), // Port A output register enable
    .regceb(regceb), // Port B output register enable
    .douta(douta),   // Port A RAM output data, width determined from NB_COL*COL_WIDTH
    .doutb(doutb)    // Port B RAM output data, width determined from NB_COL*COL_WIDTH
  );
*/
