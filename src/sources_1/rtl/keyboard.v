/**************************************************************************************************/
/**** RVSoc (Mini Kuroda/RISC-V)                       since 2018-08-07   ArchLab. TokyoTech   ****/
/**** The Console Module v0.01                                                                 ****/
/**************************************************************************************************/
`default_nettype none
/**************************************************************************************************/
`include "define.vh"

`define VIRTIO_INPUT_CFG_UNSET     8'h00
`define VIRTIO_INPUT_CFG_ID_NAME   8'h01
`define VIRTIO_INPUT_CFG_ID_SERIAL 8'h02
`define VIRTIO_INPUT_CFG_ID_DEVIDS 8'h03
`define VIRTIO_INPUT_CFG_PROP_BITS 8'h10
`define VIRTIO_INPUT_CFG_EV_BITS   8'h11
`define VIRTIO_INPUT_CFG_ABS_INFO  8'h12

`define VIRTIO_INPUT_EV_SYN 8'h00
`define VIRTIO_INPUT_EV_KEY 8'h01
`define VIRTIO_INPUT_EV_REL 8'h02
`define VIRTIO_INPUT_EV_ABS 8'h03
`define VIRTIO_INPUT_EV_REP 8'h14

/**************************************************************************************************/
module m_keyboard(CLK, RST_X, w_we, w_addr_t, w_idata, w_iirq, w_odata,
                w_oirq, w_oeirq, w_mode, w_keybrd_req, w_qnum, w_qsel, w_mtime,
                w_init_stage,
`ifdef CH559_USB
                ch559_rx,
`else
                ps2_clk, ps2_data,
`endif
                w_kbd_we, w_kbd_data);
    input  wire         CLK, RST_X;
    input  wire         w_we;
    input  wire [31:0]  w_addr_t, w_idata, w_iirq;
    output wire [31:0]  w_odata, w_oirq;
    output wire         w_oeirq;
    input  wire  [2:0]  w_mode;   // 0:CPU, 1:CONS-w_req, 2: DISK_REQ
    output wire         w_keybrd_req;
    output wire [31:0]  w_qnum, w_qsel;
    input  wire [63:0]  w_mtime;
    input  wire         w_init_stage;
`ifdef CH559_USB
    input  wire         ch559_rx;
`else
    inout  wire         ps2_clk, ps2_data;
`endif
    output wire         w_kbd_we;
    output wire [7:0]   w_kbd_data;
    wire       w_ps2_we;
    wire [7:0] w_ps2_data;
    assign w_kbd_we = w_ps2_we;
    assign w_kbd_data = w_ps2_data;

    wire [11:0] w_addr = w_addr_t[11:0]; // Note, use just lower12-bit, not verified

    reg  [31:0] MagicValue              = 32'h74726976;
    reg  [31:0] Version                 = 32'h2;
    reg  [31:0] DeviceID                = 32'h12;
    reg  [31:0] VendorID                = 32'hffff;
    reg  [31:0] DeviceFeatures          = 32'h1;
    reg  [31:0] QueueNumMax             = `KEYBRD_QUEUE_NUM_MAX;
    reg  [31:0] ConfigGeneration        = 0;

    reg  [31:0] DeviceFeaturesSel       = 0;
    reg  [31:0] DriverFeatures          = 0;
    reg  [31:0] DriverFeaturesSel       = 0;
    reg  [31:0] InterruptStatus         = 0;
    reg  [31:0] InterruptAcknowledge    = 0;
    reg  [31:0] Status                  = 1;
    reg  [31:0] QueueSel                = 0; /* Note */
    reg  [31:0] QueueNum                = 0;

    reg  [7:0]  select                  = 0;
    reg  [7:0]  subsel                  = 0;
    reg  [7:0] id_name [0:14];
    initial begin
        id_name[ 0] = "v";
        id_name[ 1] = "i";
        id_name[ 2] = "r";
        id_name[ 3] = "t";
        id_name[ 4] = "i";
        id_name[ 5] = "o";
        id_name[ 6] = "_";
        id_name[ 7] = "k";
        id_name[ 8] = "e";
        id_name[ 9] = "y";
        id_name[10] = "b";
        id_name[11] = "o";
        id_name[12] = "a";
        id_name[13] = "r";
        id_name[14] = "d";
    end

    reg  [31:0] Queue[0:(`KEYBRD_QUEUE_NUM_MAX * 9) -1];
    integer i;
    initial begin
        for(i=0;i<`KEYBRD_QUEUE_NUM_MAX * 9;i=i+1) Queue[i] = 0;
    end

    reg  [31:0] r_rdata = 0;
    reg         r_oeirq = 0;
    reg  [31:0] r_oirq  = 0;

    assign w_odata = r_rdata;
    assign w_oeirq = r_oeirq;
    always @ (posedge CLK) begin
        r_oeirq <= (w_mode==`MC_MODE_CPU && w_we && w_addr==32'h64) | w_keybrd_req;
    end

    wire [31:0] w_irqmask = 1 << (`VIRTIO_KEYBRD_IRQ-1);
    wire [31:0] w_oirq1   = w_iirq |  w_irqmask;
    wire [31:0] w_oirq2   = w_iirq & ~w_irqmask;
    always @ (posedge CLK) begin
        r_oirq <= (w_keybrd_req) ? w_oirq1 : (w_addr==32'h64) ? w_oirq2 : w_oirq1;
    end
    assign w_oirq = r_oirq;

    wire   w_key_recv;
    assign w_keybrd_req = w_key_recv ;

    assign w_qnum = QueueNum;
    assign w_qsel = (w_key_recv) ? 0 : w_idata;

    wire       w_ps2_we;
    wire [7:0] w_ps2_data;

    reg        r_ps2_we;
    reg  [7:0] r_ps2_data;

    always @ (posedge CLK) begin
        r_ps2_we <= w_ps2_we;
        r_ps2_data <= w_ps2_data;
    end

    reg   [3:0] r_head        = 0;
    reg   [3:0] r_tail        = 0;
    reg   [4:0] r_cnts        = 0;
    reg         r_en          = 0;
    reg   [15:0] fifo [0:15];

    reg key_release = 0;

    assign w_key_recv = r_en && (w_mtime > 64'd61000000) && ((w_mtime & 64'h3ffff) == 0)
                            && (w_mode == `MC_MODE_CPU) && w_init_stage;

`ifdef CH559_USB
    serialc s0(CLK, RST_X, ch559_rx, w_ps2_data, w_ps2_we);
`else
    m_ps2interface ps2interface(
        .ps2_clk(ps2_clk),
        .ps2_data(ps2_data),
        .CLK(CLK),
        .RST(!RST_X),
        .tx_data(0),
        .tx_en(0),
        .rx_data(w_ps2_data),
        .rx_en(w_ps2_we),
        .busy(),
        .err()
        );
`endif

    reg [2:0] r_mode_prev = 0;
    always @ (posedge CLK) begin
        r_mode_prev <= w_mode;
    end

    always@(posedge CLK) begin
        if (w_ps2_we) begin
            if (w_ps2_data == 8'hf0) begin
                key_release = 1;
            end else begin
                if(r_cnts < 16) begin
                    fifo[r_tail] <= (key_release) ? {8'h0, w_ps2_data}: {8'h1, w_ps2_data};
                    r_tail            <= r_tail + 1;
                    r_cnts            <= r_cnts + 1;
                    r_en              <= 1;
                end
                key_release <= 0;
            end
        end

        if (w_mode == `MC_MODE_CPU && r_mode_prev == `MC_MODE_KEYBRD) begin
            r_en <= (r_cnts<=1) ? 0 : 1;
            r_head <= r_head + 1;
            r_cnts <= r_cnts - 1;
        end
        // Micro-Ctrl Access
        if(w_mode==`MC_MODE_KEYBRD) begin
            if (w_addr == 12'h0ffc) begin
                r_rdata <= {16'h0, fifo[r_head]};
            end else begin
                r_rdata <= Queue[w_addr[$clog2(`CONSOLE_QUEUE_NUM_MAX*9)+1:2]];
                if(w_we) begin
                    Queue[w_addr[$clog2(`CONSOLE_QUEUE_NUM_MAX*9)+1:2]] <= w_idata;
                end
            end
        end

        // CPU Access
        else if(w_mode==`MC_MODE_CPU) begin
            // READ
            case (w_addr)
                32'h0   : r_rdata <= MagicValue;
                32'h4   : r_rdata <= Version;
                32'h8   : r_rdata <= DeviceID;
                32'hc   : r_rdata <= VendorID;
                32'h10  : r_rdata <= DeviceFeatures;
                32'h34  : r_rdata <= QueueNumMax;
                32'h44  : r_rdata <= Queue[QueueSel*9];               // Ready
                32'h60  : r_rdata <= InterruptStatus;
                32'h70  : r_rdata <= Status;
                32'hfc  : r_rdata <= ConfigGeneration;
                32'h102 :   case (select)                 // size
                                `VIRTIO_INPUT_CFG_ID_NAME: r_rdata <= 15;   // length of "virtio_keyboard"
                                `VIRTIO_INPUT_CFG_EV_BITS:
                                case (subsel)
                                    `VIRTIO_INPUT_EV_KEY: r_rdata <= 128/8;
                                    `VIRTIO_INPUT_EV_REP: r_rdata <= 1;
                                    default: r_rdata <= 0;
                                endcase
                                default: r_rdata <= 0;
                            endcase
                default :   if (32'h108 <= w_addr && w_addr < 32'h188) begin   //  union u
                                if (select == `VIRTIO_INPUT_CFG_ID_NAME) begin
                                    r_rdata <= id_name[w_addr - 32'h108];
                                end else begin
                                    r_rdata <= 32'hff;
                                end
                            end else begin
                                r_rdata <= 0;
                            end
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
                    32'h100 : select                    <= w_idata[7:0];
                    32'h101 : subsel                    <= w_idata[7:0];
                endcase
            end
        end
        if(w_key_recv) begin
            InterruptStatus <= InterruptStatus | 1;
        end
    end



endmodule // Keyboard
/**************************************************************************************************/
