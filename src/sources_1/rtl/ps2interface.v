`define CYCLE_100US 100 * 100// 100 * (clock frequency in MHz)


module m_ps2interface(
    input  wire       CLK,
    input  wire       RST,
    inout  wire       ps2_clk,
    inout  wire       ps2_data,
    input  wire [7:0] tx_data,
    input  wire       tx_en,
    output reg  [7:0] rx_data,
    output reg        rx_en,
    output wire       busy,
    output reg        err
    );

    wire ps2_clk_clean, ps2_data_clean;
    debouncer debouncer0(CLK, ps2_clk, ps2_clk_clean);
    debouncer debouncer1(CLK, ps2_data, ps2_data_clean);

    reg ps2_clk_h = 1, ps2_data_h = 1;
    assign ps2_clk  = (ps2_clk_h) ? 1'bz : 1'b0;
    assign ps2_data = (ps2_data_h) ? 1'bz : 1'b0;

    reg  ps2_clk_prev = 0;
    always @ (posedge CLK) begin
        ps2_clk_prev <= ps2_clk_clean;
    end
    wire ps2_clk_posedge = !ps2_clk_prev & ps2_clk_clean;
    wire ps2_clk_negedge = ps2_clk_prev & !ps2_clk_clean;

    reg [2:0] bitcnt8 = 0;
    reg parity = 0;
    reg [7:0] tx_buf = 0;
    reg [$clog2(`CYCLE_100US)-1:0] cnt100us = 0;

    localparam  IDLE         = 0;
    localparam  RX_DATA      = 1;
    localparam  RX_PARITY    = 2;
    localparam  RX_STOP      = 3;
    localparam  TX_INIT      = 4;
    localparam  TX_START     = 5;
    localparam  TX_DATA      = 6;
    localparam  TX_PARITY    = 7;
    localparam  TX_STOP      = 8;
    localparam  TX_WAIT_ACK  = 9;
    localparam  TX_WAIT_IDLE = 10;

    reg [3:0] state = IDLE;

    always @ (posedge CLK) begin
        if (RST) begin
            state <= IDLE;
            rx_en <= 0;
            err   <= 0;
            ps2_clk_h <= 1;
            ps2_data_h <= 1;
        end else begin
            case (state)
                IDLE: begin
                    if (ps2_clk_negedge) begin
                    //  ignore start bit
                        state <= RX_DATA;
                        bitcnt8 <= 0;
                        parity <= 0;
                    end else if (tx_en) begin
                        tx_buf <= tx_data;
                        bitcnt8 <= 0;
                        parity <= 0;
                        state <= TX_INIT;
                        cnt100us <= 0;
                        ps2_clk_h <= 0;
                    end
                end
                RX_DATA: begin
                    if (ps2_clk_negedge) begin
                        rx_data <= {ps2_data_clean, rx_data[7:1]};
                        parity  <= ps2_data_clean ^ parity;
                        if (bitcnt8 == 7) begin
                            state <= RX_PARITY;
                        end else begin
                            bitcnt8 <= bitcnt8 + 1;
                        end
                    end
                end
                RX_PARITY: begin
                    if (ps2_clk_negedge) begin
                        if (parity ^ ps2_data_clean) begin
                            //parity ok
                            rx_en <= 1;
                        end else begin
                            err <= 1;
                        end
                        state <= RX_STOP;
                    end
                end
                RX_STOP: begin
                    if (ps2_clk_negedge) begin
                        //stop bit arrived
                        state <= IDLE;
                    end
                    rx_en <= 0;
                    err   <= 0;
                end
                TX_INIT: begin
                    //wait for 100 microsec
                    if (cnt100us == `CYCLE_100US) begin
                        state <= TX_START;
                        ps2_data_h <= 0;
                    end
                    cnt100us <= cnt100us + 1;
                end
                TX_START: begin
                    //start bit
                    ps2_clk_h <= 1;
                    state <= TX_DATA;
                end
                TX_DATA: begin
                    if (ps2_clk_negedge) begin
                        ps2_data_h <= tx_buf[0];
                        tx_buf <= {1'b0, tx_buf[7:1]};
                        parity <= tx_buf[0] ^ parity;
                        if (bitcnt8 == 7) begin
                            state <= TX_PARITY;
                        end else begin
                            bitcnt8 <= bitcnt8 + 1;
                        end
                    end
                end
                TX_PARITY: begin
                    if (ps2_clk_negedge) begin
                        ps2_data_h <= ~parity;
                        state <= TX_STOP;
                    end
                end
                TX_STOP: begin
                    if (ps2_clk_negedge) begin
                        //stop bit
                        ps2_data_h <= 1;
                        state <= TX_WAIT_ACK;
                    end
                end
                TX_WAIT_ACK: begin
                    if (ps2_clk_negedge) begin
                        if (ps2_data_clean) begin
                            err <= 1;
                        end
                        state <= TX_WAIT_IDLE;
                    end
                end
                TX_WAIT_IDLE: begin
                    if (ps2_clk_clean & ps2_clk_clean) begin
                        state <= IDLE;
                        err <= 0;
                    end
                end
                default: ;
            endcase

        end
    end

    assign busy = (state != IDLE);

endmodule

module debouncer#(
    parameter WAIT_NUM = 16
    )(
    input  wire CLK,
    input  wire dirty,
    output reg  clean = 0
    );


    reg [$clog2(WAIT_NUM)-1:0] cnt = 0;
    reg t0, dirty_sync;

    always @ (posedge CLK) begin
        t0 <= dirty;
        dirty_sync <= t0;
    end

    always @ (posedge CLK) begin
        if (dirty_sync == clean) begin
            cnt <= 0;
        end else begin
            if (cnt == WAIT_NUM-1) begin
                clean <= dirty_sync;
            end
            cnt <= cnt + 1;
        end
    end


endmodule
