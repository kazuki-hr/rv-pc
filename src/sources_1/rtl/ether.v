/**************************************************************************************************/
/**** RVSoC (Mini Kuroda/RISC-V)                       since 2018-08-07   ArchLab. TokyoTech   ****/
/**** The Ethernet Module v0.01                                                                    ****/
/**************************************************************************************************/
`default_nettype none
/**************************************************************************************************/
`include "define.vh"

`define MAC_ADDR 48'haabbccddeeff

`define ETHER_REG_BASE            32'h4000d800
`define ETHER_REG_SEND_BUSY       5'h0
`define ETHER_REG_SEND_PACKET_LEN 5'h4
`define ETHER_REG_CMD_SEND        5'h8
`define ETHER_REG_RECV_PACKET_LEN 5'hc
`define ETHER_REG_RECV_DONE       5'h10
`define ETHER_REG_NUM             5

/**************************************************************************************************/
module m_ether(CLK, clk_50mhz, RST_X, w_we, w_addr_t, w_idata, w_iirq, w_odata,
                w_oirq, w_oeirq, w_mode, w_send_req, w_recv_req, w_qnum, w_qsel, crs_dv_phy,
                txd_phy, txen_phy, rxd_phy, rxerr_phy, clkin_phy, w_mtime, w_init_stage, int_en);
    input  wire         CLK, clk_50mhz, RST_X;
    input  wire         w_we;
    input  wire [31:0]  w_addr_t, w_idata, w_iirq;
    output wire [31:0]  w_odata, w_oirq;
    output wire         w_oeirq;
    input  wire [2:0]   w_mode;   // 0:CPU, 1:CONS-w_req, 2: DISK_REQ
    output wire         w_send_req;    // Micro-Ctrl Request
    output wire         w_recv_req;
    output wire [31:0]  w_qnum, w_qsel;
    input  wire         crs_dv_phy;
    output wire [1:0]   txd_phy;
    output wire         txen_phy;
    input  wire [1:0]   rxd_phy;
    input  wire         rxerr_phy;
    output wire         clkin_phy;
    input  wire [63:0]  w_mtime;
    input  wire         w_init_stage;
    input  wire         int_en;

    wire [13:0] w_addr = w_addr_t[13:0]; // Note, use just lower 14-bit, not verified
    reg  [13:0] r_addr = 0;

    reg  [31:0] MagicValue              = 32'h74726976;
    reg  [31:0] Version                 = 32'h2;
    reg  [31:0] DeviceID                = 32'h1;
    reg  [31:0] VendorID                = 32'hffff;
    reg  [63:0] DeviceFeatures          = 1 << 32 | 1 << 5;  //VIRTIO_F_VERSION_1, VIRTIO_NET_F_MAC
    reg  [31:0] QueueNumMax             = `ETHER_QUEUE_NUM_MAX;
    reg  [31:0] ConfigGeneration        = 0;

    reg   [7:0] Config [0:255];
    reg  [31:0] DeviceFeaturesSel       = 0;
    reg  [31:0] DriverFeatures          = 0;
    reg  [31:0] DriverFeaturesSel       = 0;
    reg  [31:0] InterruptStatus         = 0;
    reg  [31:0] InterruptAcknowledge    = 0;
    reg  [31:0] Status                  = 1;
    reg  [31:0] QueueSel                = 0;
    reg  [31:0] QueueNum                = 0;

    reg  [31:0] Queue[0:(`ETHER_QUEUE_NUM_MAX * 9) -1];
    integer i;
    initial begin
        for(i=0;i<`ETHER_QUEUE_NUM_MAX * 9;i=i+1) Queue[i] = 0;
    end

    reg  [31:0] rdata = 0;
    reg         r_oeirq = 0;
    reg  [31:0] r_oirq  = 0;

    assign w_odata = rdata;
    assign w_oeirq = r_oeirq;
    always @ (posedge CLK) begin
        r_oeirq <= (w_mode==`MC_MODE_CPU && w_we && w_addr==32'h64) | w_recv_req;
    end

    wire [31:0] w_irqmask = 1 << (`VIRTIO_ETHER_IRQ-1);
    wire [31:0] w_oirq1   = w_iirq |  w_irqmask;
    wire [31:0] w_oirq2   = w_iirq & ~w_irqmask;
    always @ (posedge CLK) begin
        r_oirq <= (w_recv_req) ? w_oirq1 : (w_addr==32'h64) ? w_oirq2 : w_oirq1;
    end
    assign w_oirq = r_oirq;

    assign w_send_req  = (w_mode==`MC_MODE_CPU && w_we && w_addr == 32'h50 && w_idata == 1);
    assign w_recv_req  = (w_mode==`MC_MODE_CPU && pending && w_init_stage && (w_mtime > 64'd61000000) && ((w_mtime & 64'h3ffff) == 0) && int_en);
    assign w_qnum = QueueNum;
    assign w_qsel = (w_recv_req) ? 0 : w_idata ;

    reg [31:0] ether_regs [0:`ETHER_REG_NUM-1];
    initial begin
        for(i=0;i<`ETHER_REG_NUM;i=i+1) ether_regs[i] = 0;
    end

    wire [$clog2(`ETHER_REG_NUM)+1:0] w_ether_reg_addr = w_addr[$clog2(`ETHER_REG_NUM)+1:0];



    wire[11:0] w_send_buf_addr;
    wire[7:0]  send_buf_data;

    xilinx_simple_dual_port_2_clock_ram #(
    .RAM_WIDTH(8),                       // Specify RAM data width
    .RAM_DEPTH(`ETHER_MTU+22),                     // Specify RAM depth (number of entries)
    .RAM_PERFORMANCE("LOW_LATENCY"), // Select "HIGH_PERFORMANCE" or "LOW_LATENCY"
    .INIT_FILE("")                        // Specify name/location of RAM initialization file if using one (leave blank if not)
    ) send_buf (
    .addra(w_addr[11:0]),   // Write address bus, width determined from RAM_DEPTH
    .addrb(w_send_buf_addr),   // Read address bus, width determined from RAM_DEPTH
    .dina(w_idata[7:0]),     // RAM input data, width determined from RAM_WIDTH
    .clka(CLK),     // Clock
    .clkb(clk_50mhz),
    .wea(w_mode == `MC_MODE_ETHER_SEND & w_addr[13:12] == 2'b10 & w_we),       // Write enable
    .enb(1),	     // Read Enable, for additional power savings, disable when not in use
    .rstb(!RST_X),     // Output reset (does not affect memory contents)
    .regceb(0), // Output register enable
    .doutb(send_buf_data)    // RAM output data, width determined from RAM_WIDTH
    );


    reg  [7:0] recv_buf_wdata;
    wire [7:0] recv_buf_rdata;
    reg [11:0] recv_addr = 0;
    reg recv_buf_we      = 0;


    xilinx_simple_dual_port_2_clock_ram #(
    .RAM_WIDTH(8),                       // Specify RAM data width
    .RAM_DEPTH(`ETHER_MTU+22),                     // Specify RAM depth (number of entries)
    .RAM_PERFORMANCE("LOW_LATENCY"), // Select "HIGH_PERFORMANCE" or "LOW_LATENCY"
    .INIT_FILE("")                        // Specify name/location of RAM initialization file if using one (leave blank if not)
    ) recv_buf (
    .addra(recv_addr),   // Write address bus, width determined from RAM_DEPTH
    .addrb(w_addr[11:0]),   // Read address bus, width determined from RAM_DEPTH
    .dina(recv_buf_wdata),     // RAM input data, width determined from RAM_WIDTH
    .clka(clk_50mhz),     // Clock
    .clkb(CLK),
    .wea(recv_buf_we),       // Write enable
    .enb(1),	     // Read Enable, for additional power savings, disable when not in use
    .rstb(!RST_X),     // Output reset (does not affect memory contents)
    .regceb(0), // Output register enable
    .doutb(recv_buf_rdata)    // RAM output data, width determined from RAM_WIDTH
    );


    reg pending = 0;

    always@(posedge CLK) begin
        // Micro-Ctrl Access

        /*
        memory map (Micro-Ctrl)
        0x4000d000 +------------------------+
                   | QueueState             |
        0x4000d800 +------------------------+
                   | Ethernet registers     |
        0x4000e000 +------------------------+
                   | Send buffers           |
        0x4000f000 +------------------------+
                   | Receive buffers        |
        0x40010000 +------------------------+
        */

        if(w_mode==`MC_MODE_ETHER_SEND || w_mode==`MC_MODE_ETHER_RECV) begin
            if (w_addr[13:11] == 3'b011) begin
                // Ethernet registers
                if (w_we) begin
                    case (w_ether_reg_addr)
                        `ETHER_REG_CMD_SEND: begin ether_regs[`ETHER_REG_CMD_SEND >> 2]  <= w_idata; end
                        `ETHER_REG_SEND_PACKET_LEN: begin ether_regs[`ETHER_REG_SEND_PACKET_LEN >> 2] <= w_idata; end
                        `ETHER_REG_RECV_DONE: begin ether_regs[`ETHER_REG_RECV_DONE >> 2] <= w_idata; end
                        default: ;
                    endcase
                end else begin
                    // clear write-only registers
                    ether_regs[`ETHER_REG_CMD_SEND >> 2]  <= 0;
                    ether_regs[`ETHER_REG_RECV_DONE >> 2] <= 0;
                end
                rdata <= ether_regs[w_ether_reg_addr >> 2];
            end else if (w_addr[13:12] == 2'b01) begin
                // QueueState
                rdata <= Queue[w_addr[$clog2(`ETHER_QUEUE_NUM_MAX*9)+1:2]];
                if(w_we) begin
                    Queue[w_addr[$clog2(`ETHER_QUEUE_NUM_MAX*9)+1:2]] <= w_idata;
                end
            end else if(w_addr[13:12] == 2'b11) begin
                // Receive buffers
                rdata <= recv_buf_rdata;
            end
        end

        // CPU Access
        else if(w_mode==`MC_MODE_CPU) begin
            // READ
            case (w_addr)
                32'h0   : rdata <= MagicValue;
                32'h4   : rdata <= Version;
                32'h8   : rdata <= DeviceID;
                32'hc   : rdata <= VendorID;
                32'h10  : rdata <= (DeviceFeaturesSel == 0)? DeviceFeatures[31:0]: DeviceFeatures[63:32];
                32'h34  : rdata <= QueueNumMax;
                32'h44  : rdata <= Queue[QueueSel*9];               // Ready
                32'h60  : rdata <= InterruptStatus;
                32'h70  : rdata <= Status;
                32'hfc  : rdata <= ConfigGeneration;
                32'h100 : rdata <= `MAC_ADDR >> 40 & 8'hff;
                32'h101 : rdata <= `MAC_ADDR >> 32 & 8'hff;
                32'h102 : rdata <= `MAC_ADDR >> 24 & 8'hff;
                32'h103 : rdata <= `MAC_ADDR >> 16 & 8'hff;
                32'h104 : rdata <= `MAC_ADDR >> 8  & 8'hff;
                32'h105 : rdata <= `MAC_ADDR       & 8'hff;
                default : rdata <= 0;
            endcase

            // WRITE
            if(w_we) begin
                case (w_addr)
                    32'h14  : DeviceFeaturesSel         <= w_idata;
                    32'h20  : DriverFeatures            <= w_idata;
                    32'h24  : DriverFeaturesSel         <= w_idata;
                    32'h30  : QueueSel                  <= w_idata;
                    32'h38  : QueueNum                  <= w_idata;
                    32'h44  : Queue[QueueSel*9]         <= w_idata;  // Ready
                    32'h50  : begin
                        Queue[QueueSel*9+1] <= w_idata;              // Notify
                        InterruptStatus <= InterruptStatus | 1;
                    end
                    32'h64  : begin
                        InterruptAcknowledge <= w_idata;
                        InterruptStatus <= InterruptStatus & ~w_idata;
                    end
                    32'h70  : Status                    <= w_idata;
                    32'h80  : Queue[QueueSel*9+2]       <= w_idata;  // DescLow
                    32'h90  : Queue[QueueSel*9+4]       <= w_idata;  // AvailLow
                    32'ha0  : Queue[QueueSel*9+6]       <= w_idata;  // UsedLow
                endcase
            end
        end
        if (w_recv_req) begin
            InterruptStatus <= InterruptStatus | 1;
        end
    end

    assign clkin_phy = clk_50mhz;


    // Ethernet send


    reg txen_mac = 0;

    wire mac_ready;
    reg s0 = 0, s1 = 0;
    reg mac_ready_sync = 0;
    reg txen_mac_sync = 0;

    //clock domain crossing
    always@(posedge CLK) begin
        s0 <= mac_ready;
        mac_ready_sync <= s0;
    end


    localparam  SEND_IDLE          = 0;
    localparam  SEND_WAIT_ACK      = 1;
    localparam  SEND_WAIT_COMPLETE = 2;

    reg [1:0] r_send_state = SEND_IDLE;

    wire        w_cmd_send = ether_regs[`ETHER_REG_CMD_SEND >> 2][0];
    wire [11:0] send_buf_len = ether_regs[`ETHER_REG_SEND_PACKET_LEN >> 2];
    wire        send_busy = (r_send_state != SEND_IDLE);

    always @ (posedge CLK) begin
        ether_regs[`ETHER_REG_SEND_BUSY >> 2] <= {31'h0, send_busy};
    end

    always @ (posedge CLK) begin
        if (!RST_X) begin
            r_send_state <= SEND_IDLE;
            txen_mac <= 0;
        end else begin
            case (r_send_state)
                SEND_IDLE: begin
                    if (w_cmd_send) begin
                        r_send_state <= SEND_WAIT_ACK;
                        txen_mac <= 1;
                    end
                end
                SEND_WAIT_ACK: begin
                    if (!mac_ready_sync) begin  //acked
                        txen_mac <= 0;
                        r_send_state <= SEND_WAIT_COMPLETE;
                    end
                end
                SEND_WAIT_COMPLETE: begin
                    if (mac_ready_sync) begin
                        r_send_state <= SEND_IDLE;
                    end
                end
            endcase
        end
    end

    always@(posedge clk_50mhz) begin
        s1 <= txen_mac;
        txen_mac_sync <= s1;
    end


    ether_send es0(!RST_X, clk_50mhz, crs_dv_phy, send_buf_data, txen_mac_sync,
                    send_buf_len, txd_phy, w_send_buf_addr, txen_phy, mac_ready);


    // Ethernet receive

    wire       dv, done;
    reg        avail_sync = 0;
    reg        avail = 0;
    reg        pending_sync = 0;
    reg        t0 = 0, t1 = 0;

    always@(posedge CLK) begin
        t0 <= avail;
        avail_sync <= t0;
    end

    always@(posedge CLK) begin
        if (avail_sync) begin
            pending <= 1;
        end else if (ether_regs[`ETHER_REG_RECV_DONE>>2]) begin
            pending <= 0;
        end
    end


    always@(posedge clk_50mhz) begin
        t1 <= pending;
        pending_sync <= t1;
    end

    wire [7:0] recv_data;

    localparam  RECV_IDLE     = 0;
    localparam  RECV_DATA     = 1;
    localparam  RECV_WAIT_ACK = 2;

    reg [1:0] r_recv_state = RECV_IDLE;

    always @ (posedge clk_50mhz) begin
        if (!RST_X) begin
            r_recv_state <= RECV_IDLE;
            recv_addr <= 0;
            recv_buf_we <= 0;
            avail <= 0;
        end else begin
            case (r_recv_state)
                RECV_IDLE: begin
                    if (!pending_sync & dv) begin
                        recv_buf_wdata <= recv_data;
                        recv_buf_we <= 1;
                        r_recv_state <= RECV_DATA;
                    end
                end
                RECV_DATA: begin
                    if (dv) begin
                        recv_buf_wdata <= recv_data;
                        recv_addr <= recv_addr + 1;
                        recv_buf_we <= 1;
                    end else begin
                        if (done) begin
                            ether_regs[`ETHER_REG_RECV_PACKET_LEN >> 2] <= recv_addr+1;
                            avail    <= 1;
                            r_recv_state <= RECV_WAIT_ACK;
                        end
                        recv_buf_we <= 0;
                    end
                end
                RECV_WAIT_ACK: begin
                    if (pending_sync) begin
                        avail <= 0;
                        r_recv_state <= RECV_IDLE;
                        recv_addr <= 0;
                    end
                end
            endcase

        end
    end


    ether_recv er0(!RST_X, clk_50mhz, rxd_phy, crs_dv_phy, rxerr_phy, recv_data, dv, done);

endmodule // Ether
/**************************************************************************************************/

module ether_send (rst, clk_50mhz, crs_dv_phy, buf_data, txen_mac, buf_len, txd_phy, buf_addr, txen_phy, ready);
    input wire rst;
    input wire clk_50mhz;
    input wire crs_dv_phy;
    input wire[7:0] buf_data;
    input wire txen_mac;
    input wire[11:0] buf_len;
    output wire[1:0] txd_phy;
    output reg[11:0] buf_addr;
    output reg txen_phy = 0;
    output wire ready;

    reg[2:0] bit_cnt = 0;
    reg[7:0] buffer;
    reg[11:0] len;
    reg[31:0] col_waitcnt = 0;


    localparam  IDLE           = 0;
    localparam  SEND           = 1;
    localparam  COLLISION_WAIT = 2;

    assign ready = (r_state == IDLE);

    reg [1:0]   r_state = IDLE;

    always @ (posedge clk_50mhz) begin
        if (rst) begin
            r_state <= IDLE;
            txen_phy <= 0;
        end else begin
            case (r_state)
                IDLE: begin
                    if (txen_mac) begin
                        bit_cnt <= 3'd0;
                        txen_phy <= 0;
                        buf_addr <= 0;
                        len <= buf_len;
                        r_state <= SEND;
                    end
                end
                SEND: begin
                    if (crs_dv_phy) begin
                        col_waitcnt <= 0;
                        r_state <= COLLISION_WAIT;
                    end else if (bit_cnt == 3'd6) begin
                        buf_addr <= buf_addr + 1;
                        bit_cnt <= 0;
                        if (buf_addr == len) begin
                            r_state <= IDLE;
                            txen_phy <= 0;
                        end else begin
                            buffer <= (buf_addr < 7) ? 8'h55: (buf_addr == 7) ? 8'hd5 :buf_data;
                            txen_phy <= 1;
                        end
                    end else begin
                        bit_cnt <= bit_cnt + 2;
                    end
                end
                COLLISION_WAIT: begin
                    if (col_waitcnt == 32'hffff) begin
                        bit_cnt <= 0;
                        buf_addr <= 0;
                        r_state <= SEND;
                    end else begin
                        col_waitcnt <= col_waitcnt + 1;
                    end
                    txen_phy <= 0;
                end
            endcase
        end
    end



    wire[7:0] lsb2;
    assign lsb2 = txen_phy? (buffer >> (bit_cnt)): 8'b0;
    assign txd_phy = lsb2[1:0];

endmodule


module ether_recv(rst, clk_50mhz, rxd_phy, crs_dv_phy, rxerr_phy, data, dv, done);
    input  wire        rst;
    input  wire        clk_50mhz;
    input  wire  [1:0] rxd_phy;
    input  wire        crs_dv_phy;
    input  wire        rxerr_phy;
    output wire  [7:0] data;
    output reg         dv = 0;      //data valid
    output reg         done = 0;

    parameter STATE_IDLE = 3'b0, STATE_RECV1 = 3'b1, STATE_RECV2 = 3'b10;

    reg [2:0] state = STATE_IDLE;
    reg [7:0] buffer = 0;
    reg [2:0] bit_cnt = 0;
    reg [1:0] rxd_prev = 0;
    reg       dv_prev = 0;

    assign data = buffer;

    always @(posedge clk_50mhz) begin
        if (rst) begin
            state <= STATE_IDLE;
            dv    <= 0;
            done  <= 0;
        end
        if (state == STATE_IDLE) begin
            if (crs_dv_phy && rxd_phy == 2'b11) begin   // end of SFD
                state <= STATE_RECV1;
                bit_cnt <= 0;
                done <= 0;
            end
        end else if (state == STATE_RECV1) begin
            if (crs_dv_phy) begin
                if (bit_cnt == 0) begin
                    buffer[1:0] <= rxd_phy;
                end else if (bit_cnt == 2) begin
                    buffer[3:2] <= rxd_phy;
                end else if (bit_cnt == 4) begin
                    buffer[5:4] <= rxd_phy;
                end else begin
                    buffer[7:6] <= rxd_phy;
                end
                if (bit_cnt == 6) begin
                    bit_cnt <= 0;
                    dv <= 1;
                end else begin
                    bit_cnt <= bit_cnt + 2;
                    dv <= 0;
                end
            end else begin
                state <= STATE_RECV2;
                rxd_prev <= rxd_phy;
                dv <= 0;
            end
        end else if (state == STATE_RECV2) begin
            if (crs_dv_phy) begin
                if (bit_cnt == 0) begin
                    buffer[3:0] <= {rxd_phy, rxd_prev};
                    bit_cnt <= 4;
                end else begin      //  bit_cnt should be 4
                    buffer[7:4] <= {rxd_phy, rxd_prev};
                    bit_cnt <= 0;
                    dv <= 1;
                end
            end else begin
                rxd_prev <= rxd_phy;
                dv <= 0;
                if (!dv_prev && !crs_dv_phy) begin
                    state <= STATE_IDLE;
                    done <= 1;
                end
            end
            dv_prev <= crs_dv_phy;
        end
    end

endmodule


//  Xilinx Simple Dual Port 2 Clock RAM
//  This code implements a parameterizable SDP dual clock memory.
//  If a reset or enable is not necessary, it may be tied off or removed from the code.

module xilinx_simple_dual_port_2_clock_ram #(
  parameter RAM_WIDTH = 36,                       // Specify RAM data width
  parameter RAM_DEPTH = 512,                      // Specify RAM depth (number of entries)
  parameter RAM_PERFORMANCE = "HIGH_PERFORMANCE", // Select "HIGH_PERFORMANCE" or "LOW_LATENCY"
  parameter INIT_FILE = ""                        // Specify name/location of RAM initialization file if using one (leave blank if not)
) (
  input wire [clogb2(RAM_DEPTH-1)-1:0] addra, // Write address bus, width determined from RAM_DEPTH
  input wire [clogb2(RAM_DEPTH-1)-1:0] addrb, // Read address bus, width determined from RAM_DEPTH
  input wire [RAM_WIDTH-1:0] dina,          // RAM input data
  input wire clka,                          // Write clock
  input wire clkb,                          // Read clock
  input wire wea,                           // Write enable
  input wire enb,                           // Read Enable, for additional power savings, disable when not in use
  input wire rstb,                          // Output reset (does not affect memory contents)
  input wire regceb,                        // Output register enable
  output wire [RAM_WIDTH-1:0] doutb         // RAM output data
);

  reg [RAM_WIDTH-1:0] BRAM [RAM_DEPTH-1:0];
  reg [RAM_WIDTH-1:0] ram_data = {RAM_WIDTH{1'b0}};

  // The following code either initializes the memory values to a specified file or to all zeros to match hardware
  generate
    if (INIT_FILE != "") begin: use_init_file
      initial
        $readmemh(INIT_FILE, BRAM, 0, RAM_DEPTH-1);
    end else begin: init_bram_to_zero
      integer ram_index;
      initial
        for (ram_index = 0; ram_index < RAM_DEPTH; ram_index = ram_index + 1)
          BRAM[ram_index] = {RAM_WIDTH{1'b0}};
    end
  endgenerate

  always @(posedge clka)
    if (wea)
      BRAM[addra] <= dina;

  always @(posedge clkb)
    if (enb)
      ram_data <= BRAM[addrb];

  //  The following code generates HIGH_PERFORMANCE (use output register) or LOW_LATENCY (no output register)
  generate
    if (RAM_PERFORMANCE == "LOW_LATENCY") begin: no_output_register

      // The following is a 1 clock cycle read latency at the cost of a longer clock-to-out timing
       assign doutb = ram_data;

    end else begin: output_register

      // The following is a 2 clock cycle read latency with improve clock-to-out timing

      reg [RAM_WIDTH-1:0] doutb_reg = {RAM_WIDTH{1'b0}};

      always @(posedge clkb)
        if (rstb)
          doutb_reg <= {RAM_WIDTH{1'b0}};
        else if (regceb)
          doutb_reg <= ram_data;

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

// The following is an instantiation template for xilinx_simple_dual_port_2_clock_ram
/*
//  Xilinx Simple Dual Port 2 Clock RAM
  xilinx_simple_dual_port_2_clock_ram #(
    .RAM_WIDTH(18),                       // Specify RAM data width
    .RAM_DEPTH(1024),                     // Specify RAM depth (number of entries)
    .RAM_PERFORMANCE("HIGH_PERFORMANCE"), // Select "HIGH_PERFORMANCE" or "LOW_LATENCY"
    .INIT_FILE("")                        // Specify name/location of RAM initialization file if using one (leave blank if not)
  ) your_instance_name (
    .addra(addra),    // Write address bus, width determined from RAM_DEPTH
    .addrb(addrb),    // Read address bus, width determined from RAM_DEPTH
    .dina(dina),      // RAM input data, width determined from RAM_WIDTH
    .clka(clka),      // Write clock
    .clkb(clkb),      // Read clock
    .wea(wea),        // Write enable
    .enb(enb),        // Read Enable, for additional power savings, disable when not in use
    .rstb(rstb),      // Output reset (does not affect memory contents)
    .regceb(regceb),  // Output register enable
    .doutb(doutb)     // RAM output data, width determined from RAM_WIDTH
  ); */
