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
module m_mouse(CLK, RST_X, w_we, w_addr_t, w_idata, w_iirq, w_odata,
                w_oirq, w_oeirq, w_mode, w_mouse_req, w_qnum, w_qsel, w_mtime,
                w_init_stage, ps2_clk, ps2_data, w_ms_we, w_ms_data);
    input  wire         CLK, RST_X;
    input  wire         w_we;
    input  wire [31:0]  w_addr_t, w_idata, w_iirq;
    output wire [31:0]  w_odata, w_oirq;
    output wire         w_oeirq;
    input  wire  [2:0]  w_mode;   // 0:CPU, 1:CONS-w_req, 2: DISK_REQ
    output wire         w_mouse_req;
    output wire [31:0]  w_qnum, w_qsel;
    input  wire [63:0]  w_mtime;
    input  wire         w_init_stage;
    inout  wire         ps2_clk, ps2_data;
    output wire         w_ms_we ;
    output wire [7:0]   w_ms_data ;
    wire w_ps2_avail;
    wire [7:0] w_rx_data;
    assign w_ms_we = w_ps2_avail;
    assign w_ms_data = w_rx_data;

    wire [11:0] w_addr = w_addr_t[11:0]; // Note, use just lower12-bit, not verified

    reg  [31:0] MagicValue              = 32'h74726976;
    reg  [31:0] Version                 = 32'h2;
    reg  [31:0] DeviceID                = 32'h12;
    reg  [31:0] VendorID                = 32'hffff;
    reg  [31:0] DeviceFeatures          = 32'h1;
    reg  [31:0] QueueNumMax             = `MOUSE_QUEUE_NUM_MAX;
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
    reg  [7:0] id_name [0:11];
    initial begin
        id_name[ 0] = "v";
        id_name[ 1] = "i";
        id_name[ 2] = "r";
        id_name[ 3] = "t";
        id_name[ 4] = "i";
        id_name[ 5] = "o";
        id_name[ 6] = "_";
        id_name[ 7] = "m";
        id_name[ 8] = "o";
        id_name[ 9] = "u";
        id_name[10] = "s";
        id_name[11] = "e";
    end

    reg  [31:0] Queue[0:(`MOUSE_QUEUE_NUM_MAX * 9) -1];
    integer i;
    initial begin
        for(i=0;i<`MOUSE_QUEUE_NUM_MAX * 9;i=i+1) Queue[i] = 0;
    end

    reg  [31:0] r_rdata = 0;
    reg         r_oeirq = 0;
    reg  [31:0] r_oirq  = 0;

    assign w_odata = r_rdata;
    assign w_oeirq = r_oeirq;
    always @ (posedge CLK) begin
        r_oeirq <= (w_mode==`MC_MODE_CPU && w_we && w_addr==32'h64) | w_mouse_req;
    end

    wire [31:0] w_irqmask = 1 << (`VIRTIO_MOUSE_IRQ-1);
    wire [31:0] w_oirq1   = w_iirq |  w_irqmask;
    wire [31:0] w_oirq2   = w_iirq & ~w_irqmask;
    always @ (posedge CLK) begin
        r_oirq <= (w_mouse_req) ? w_oirq1 : (w_addr==32'h64) ? w_oirq2 : w_oirq1;
    end
    assign w_oirq = r_oirq;

    assign w_odata = r_rdata;

    wire   w_mouse_recv;
    assign w_mouse_req = w_mouse_recv ;

    assign w_qnum = QueueNum;
    assign w_qsel = (w_mouse_recv) ? 0 : w_idata;

    reg   [3:0] r_head        = 0;
    reg   [3:0] r_tail        = 0;
    reg   [4:0] r_cnts        = 0;
    reg         r_en          = 0;
    reg   [66:0] fifo [0:15];


    assign w_mouse_recv = r_en && (w_mtime > 64'd61000000) && ((w_mtime & 64'h3ffff) == 0)
                            && (w_mode == `MC_MODE_CPU) && w_init_stage;



    wire w_ps2_busy;
    reg  r_ps2_we = 0;
    reg  [7:0] r_tx_data;

    m_ps2interface ps(
        .ps2_clk(ps2_clk),
        .ps2_data(ps2_data),
        .CLK(CLK),
        .RST(!RST_X),
        .tx_data(r_tx_data),
        .tx_en(r_ps2_we),
        .rx_data(w_rx_data),
        .rx_en(w_ps2_avail),
        .busy(w_ps2_busy),
        .err()
    );

    reg [15:0] r_mouse_data = 0;
    reg [1:0]  r_cnt3 = 0;

    reg [2:0] r_mode_prev = 0;
    always @ (posedge CLK) begin
        r_mode_prev <= w_mode;
    end


    // mouse controller state machine
    localparam IDLE = 0;
    localparam SET_SAMPLE_RATE = 1;
    localparam SEND_SAMPLE_RATE = 2;
    localparam ENABLE_REPORTING = 3;
    localparam RECEIVE_DATA = 4;
    localparam WAIT_ACK = 5;

    reg [2:0] state = IDLE;
    reg [2:0] next_state; // valid only in state WAIT_ACK

    always@(posedge CLK)begin
        case(state)
            IDLE: begin
                if (w_mtime == 64'd61000000 || (w_rx_data == 8'haa & w_ps2_avail)) begin
                    state <= SET_SAMPLE_RATE;
                end
                r_ps2_we <= 0;
            end
            SET_SAMPLE_RATE: begin
                if (!w_ps2_busy) begin
                    r_ps2_we <= 1;
                    r_tx_data <= 8'hf3;
                    state <= WAIT_ACK;
                    next_state <= SEND_SAMPLE_RATE;
                end
            end
            SEND_SAMPLE_RATE: begin
                 if (!w_ps2_busy) begin
                    r_ps2_we <= 1;
                    r_tx_data <= `MOUSE_SAMPLE_RATE;
                    state <= WAIT_ACK;
                    next_state <= ENABLE_REPORTING;
               end
            end
            ENABLE_REPORTING: begin
                if (!w_ps2_busy) begin
                    r_ps2_we <= 1;
                    r_tx_data <= 8'hf4;
                    state <= WAIT_ACK;
                    next_state <= RECEIVE_DATA;
               end
            end
            RECEIVE_DATA: begin
                r_ps2_we <= 0;
            end
            WAIT_ACK: begin
                if (w_rx_data == 8'hfa & w_ps2_avail) begin
                    state <= next_state;
                end
                r_ps2_we <= 0;
            end
        endcase
    end

    reg x_ov = 0, y_ov = 0;
    reg x_sign = 0, y_sign = 0;
    reg lbtn = 0, rbtn = 0;
    reg signed [31:0] dx = 0, dy = 0;
    reg [6:0] sample_cnt = 0;
    reg [2:0] btn_state = 0;

    always @ (posedge CLK) begin
        if (w_ps2_avail & state == RECEIVE_DATA) begin
            case (r_cnt3)
                0 : begin
                    x_ov   <= w_rx_data[6];
                    y_ov   <= w_rx_data[7];
                    x_sign <= w_rx_data[4];
                    y_sign <= w_rx_data[5];
                    lbtn   <= w_rx_data[0];
                    rbtn   <= w_rx_data[1];
                    //if (sample_cnt == 0) begin
                        if(r_cnts < 16) begin
                            fifo[r_tail] <= {1'b0, rbtn, lbtn, dx, dy};
                            r_tail            <= r_tail + 1;
                            r_cnts            <= r_cnts + 1;
                            r_en              <= 1;
                        end
                        sample_cnt <= 0;
                        dx <= 0;
                        dy <= 0;
                    //end
                end
                1 : begin
                    dx <= {{24{x_sign}}, (x_ov) ? 8'hff : w_rx_data} + dx;
                end
                2 : begin
                    dy <= {{24{y_sign}}, (y_ov) ? 8'hff : w_rx_data} + dy;
                    sample_cnt <= sample_cnt + 1;
                end
            endcase
            r_cnt3 <= (r_cnt3 == 2'h2) ? 0 : r_cnt3 + 1;
        end

        if (w_mode == `MC_MODE_CPU & r_mode_prev == `MC_MODE_MOUSE) begin
            r_en <= (r_cnts<=1) ? 0 : 1;
            r_head <= r_head + 1;
            r_cnts <= r_cnts - 1;
        end
    end

    always@(posedge CLK) begin

        // Micro-Ctrl Access
        if(w_mode==`MC_MODE_MOUSE) begin
            if (w_addr == 12'hff0) begin
                r_rdata <= {29'h0, btn_state};
                if (w_we) begin
                    btn_state <= w_idata[2:0];
                end
            end else if (w_addr == 12'hff4) begin
                r_rdata <= {29'h0, fifo[r_head][66:64]};
            end else if (w_addr == 12'hff8) begin
                r_rdata <= fifo[r_head][31:0];
            end else if (w_addr == 12'hffc) begin
                r_rdata <= fifo[r_head][63:32];
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
                                `VIRTIO_INPUT_CFG_ID_NAME: r_rdata <= 12;   // length of "virtio_mouse"
                                `VIRTIO_INPUT_CFG_EV_BITS:
                                case (subsel)
                                    `VIRTIO_INPUT_EV_KEY: r_rdata <= 512/8;
                                    `VIRTIO_INPUT_EV_REL: r_rdata <= 2;
                                    default: r_rdata <= 0;
                                endcase
                                default: r_rdata <= 0;
                            endcase
                default :   if (12'h108 <= w_addr && w_addr < 12'h188) begin   //  union u
                                case (select)
                                    `VIRTIO_INPUT_CFG_ID_NAME: r_rdata <= id_name[w_addr - 12'h108];
                                    `VIRTIO_INPUT_CFG_EV_BITS:
                                    case (subsel)
                                        `VIRTIO_INPUT_EV_KEY:
                                        if (w_addr == 12'h12a) begin
                                            r_rdata <= 7;
                                        end else begin
                                            r_rdata <= 0;
                                        end
                                        `VIRTIO_INPUT_EV_REL:
                                        if (w_addr == 12'h108) begin
                                            r_rdata <= 7;
                                        end else begin
                                            r_rdata <= 0;
                                        end
                                        default: r_rdata <= 0;
                                    endcase
                                endcase
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
        if(w_mouse_recv) begin
            InterruptStatus <= InterruptStatus | 1;
        end
    end



endmodule // Mouse
/**************************************************************************************************/
