////////////////////////////////////////////////////////////////////////////////
// 41bit address space
// adjust sd card ver 2.00 or later
////////////////////////////////////////////////////////////////////////////////

module sd_controller(
    input  wire        i_clk,
    input  wire        i_rst,

    output wire        o_ready,

    input  wire        i_ren,
    output wire [ 7:0] o_data,
    output wire        o_data_en,

    input  wire        i_wen,
    input  wire [ 7:0] i_data,
    output wire        o_data_ready,

    input  wire [31:0] i_blk_num,
    input  wire [31:0] i_adr,

    output wire [ 4:0] o_state,

    // sd
    output reg  cs,
    output wire mosi,
    input  wire miso,
    output wire sclk
);

    // for init
    localparam RST = 0;
    localparam INIT = 1;
    localparam CMD0 = 2;
    localparam CMD55 = 3;
    localparam CMD41 = 4;
    localparam POLL_CMD = 5;
    localparam CMD58 = 6;
    localparam CHECK_CCS = 7;
    localparam CMD8 = 8;

    localparam SEND_CMD_R7 = 9;
    localparam RECEIVE_R7_WAIT = 10;
    localparam RECEIVE_R7 = 11;
    // for general
    localparam IDLE = 16;
    localparam SEND_CMD = 17;
    localparam RECEIVE_BYTE_WAIT = 18;
    localparam RECEIVE_BYTE = 19;
    localparam RECEIVE_BLOCK_WAIT = 20;
    localparam RECEIVE_BLOCK = 21;
    localparam RECEIVE_CRC = 22;
    localparam SEND_STOP_CMD = 23;
    // for write
    localparam SEND_BLOCK_INIT = 24;
    localparam SEND_BLOCK = 25;
    localparam SEND_BLOCK_WAIT = 26;
    localparam SEND_STOP_TOKEN = 27;
    localparam STOP_TOKEN_WAIT = 28;

    localparam WRITE_DATA_SIZE = 515;

    reg [ 4:0] state = RST;
    reg [ 4:0] return_state;
    reg        sclk_sig = 0;
    reg [55:0] cmd_out;
    reg [ 7:0] recv_data;
    reg [39:0] recv_data_R7;
    reg        cmd_mode = 1;
    reg [ 7:0] data_sig = 8'hFF;
    reg [ 9:0] byte_counter;
    reg [13:0] bit_counter;
    reg [31:0] block_counter;
    reg [26:0] boot_counter = 27'd100_000_000;
    reg [ 7:0] dout = 0;
    reg        byte_available = 0;
    reg        ready_for_next_byte = 0;

    assign o_ready = (state == IDLE);
    assign o_data  = dout;
    assign o_data_en = byte_available;
    assign o_data_ready = ready_for_next_byte;
    assign o_state = state;

    assign sclk = sclk_sig;
    assign mosi = cmd_mode ? cmd_out[55] : data_sig[7];

    always @(posedge i_clk) begin
        if(i_rst == 1) begin
            state <= RST;
            sclk_sig <= 0;
            boot_counter <= 27'd100_000_000;
        end
        else begin
            case(state)
                RST: begin
                    if(boot_counter == 0) begin
                        sclk_sig <= 0;
                        cmd_out <= {56{1'b1}};
                        byte_counter <= 0;
                        byte_available <= 0;
                        ready_for_next_byte <= 0;
                        cmd_mode <= 1;
                        bit_counter <= 160;
                        block_counter <= 0;
                        cs <= 1;
                        state <= INIT;
                        dout <= 0;
                        byte_available <= 0;
                        ready_for_next_byte <= 0;
                    end
                    else begin
                        boot_counter <= boot_counter - 1;
                    end
                end
                INIT: begin
                    if(bit_counter == 0) begin
                        cs <= 0;
                        state <= CMD0;
                    end
                    else begin
                        bit_counter <= bit_counter - 1;
                        sclk_sig <= ~sclk_sig;
                    end
                end
                CMD0: begin
                    cmd_out <= 56'hFF_40_00_00_00_00_95;
                    bit_counter <= 55;
                    return_state <= CMD8;
                    state <= SEND_CMD;
                end
                CMD8: begin
                    cmd_out <= 56'hFF_48_00_00_01_AA_87;
                    bit_counter <= 55;
                    return_state <= CMD41;
                    state <= SEND_CMD_R7;
                end
                CMD55: begin
                    cmd_out <= 56'hFF_77_00_00_00_00_65;
                    bit_counter <= 55;
                    return_state <= CMD41;
                    state <= SEND_CMD;
                end
                CMD41: begin
                    cmd_out <= 56'hFF_69_40_FF_80_00_17;
                    bit_counter <= 55;
                    return_state <= POLL_CMD;
                    state <= SEND_CMD;
                end
                POLL_CMD: begin
                    if(recv_data[0] == 0) begin
                        state <= CMD58;
                    end
                    else begin
                        state <= CMD55;
                    end
                end
                CMD58: begin
                    cmd_out <= 56'hFF_7A_00_00_00_00_FD;
                    bit_counter <= 55;
                    return_state <= CHECK_CCS;
                    state <= SEND_CMD_R7;
                end
                CHECK_CCS: begin
                    if(recv_data_R7[32] == 0) begin
                        state <= IDLE;
                    end
                    else begin
                        state <= CMD58;
                    end
                end
                IDLE: begin
                    if(i_ren == 1 & i_blk_num != 0) begin
                        block_counter <= i_blk_num;
                        cmd_out <= {16'hFF_52, i_adr, 8'hFF};
                        bit_counter <= 55;
                        return_state <= RECEIVE_BLOCK_WAIT;
                        state <= SEND_CMD;
                    end
                    else if(i_wen == 1 & i_blk_num != 0) begin
                        block_counter <= i_blk_num;
                        cmd_out <= {16'hFF_59, i_adr, 8'hFF};
                        bit_counter <= 55;
                        return_state <= SEND_BLOCK_INIT;
                        state <= SEND_CMD;
                    end
                    else begin
                        state <= IDLE;
                    end
                end
                SEND_CMD: begin
                    if (sclk_sig == 1) begin
                        if (bit_counter == 0) begin
                            state <= RECEIVE_BYTE_WAIT;
                        end
                        else begin
                            bit_counter <= bit_counter - 1;
                            cmd_out <= {cmd_out[54:0], 1'b1};
                        end
                    end
                    sclk_sig <= ~sclk_sig;
                end
                SEND_CMD_R7: begin
                    if (sclk_sig == 1) begin
                        if (bit_counter == 0) begin
                            state <= RECEIVE_R7_WAIT;
                        end
                        else begin
                            bit_counter <= bit_counter - 1;
                            cmd_out <= {cmd_out[54:0], 1'b1};
                        end
                    end
                    sclk_sig <= ~sclk_sig;
                end
                RECEIVE_BYTE_WAIT: begin
                    if (sclk_sig == 1) begin
                        if (miso == 0) begin
                            recv_data <= 0;
                            bit_counter <= 6;
                            state <= RECEIVE_BYTE;
                        end
                    end
                    sclk_sig <= ~sclk_sig;
                end
                RECEIVE_BYTE: begin
                    byte_available <= 0;
                    if (sclk_sig == 1) begin
                        recv_data <= {recv_data[6:0], miso};
                        if (bit_counter == 0) begin
                            state <= return_state;
                        end
                        else begin
                            bit_counter <= bit_counter - 1;
                        end
                    end
                    sclk_sig <= ~sclk_sig;
                end
                RECEIVE_R7_WAIT: begin
                    if (sclk_sig == 1) begin
                        if (miso == 0) begin
                            recv_data_R7 <= 0;
                            bit_counter <= 38;
                            state <= RECEIVE_R7;
                        end
                    end
                    sclk_sig <= ~sclk_sig;
                end
                RECEIVE_R7: begin
                    byte_available <= 0;
                    if (sclk_sig == 1) begin
                        recv_data_R7 <= {recv_data_R7[38:0], miso};
                        if (bit_counter == 0) begin
                            state <= return_state;
                        end
                        else begin
                            bit_counter <= bit_counter - 1;
                        end
                    end
                    sclk_sig <= ~sclk_sig;
                end
                RECEIVE_BLOCK_WAIT: begin
                    if(sclk_sig == 1 && miso == 0) begin
                        bit_counter <= (512 * 8)-1;
                        block_counter <= block_counter - 1;
                        return_state <= (block_counter == 1) ? SEND_STOP_CMD : RECEIVE_BLOCK_WAIT;
                        state <= RECEIVE_BLOCK;
                    end
                    sclk_sig <= ~sclk_sig;
                end
                RECEIVE_BLOCK: begin
                    if(sclk_sig == 1) begin
                        recv_data <= {recv_data[6:0], miso};
                        if(bit_counter[2:0]==3'b000) begin
                            dout <= {recv_data[6:0], miso};
                            byte_available <= 1;
                            if(bit_counter == 0) begin
                                bit_counter <= (2 * 8)-1;
                                state <= RECEIVE_CRC;
                            end
                            else begin
                                bit_counter <= bit_counter - 1;
                            end
                        end
                        else begin
                            byte_available <= 0;
                            bit_counter <= bit_counter - 1;
                        end
                    end
                    else begin
                        byte_available <= 0;
                    end
                    sclk_sig <= ~sclk_sig;
                end
                RECEIVE_CRC: begin
                    if(sclk_sig == 1) begin
                        if(bit_counter == 0) begin
                            state <= return_state;
                        end
                        else begin
                            bit_counter <= bit_counter - 1;
                        end
                    end
                    sclk_sig <= ~sclk_sig;
                    byte_available <= 0;
                end
                SEND_STOP_CMD: begin
                    cmd_out <= 56'hFF_4C_00_00_00_00_FF;
                    bit_counter <= 55;
                    return_state <= STOP_TOKEN_WAIT;//IDLE;
                    state <= SEND_CMD;
                end
                SEND_BLOCK_INIT: begin
                    cmd_mode <= 0;
                    ready_for_next_byte <= 0;
                    state <= SEND_BLOCK;
                    data_sig <= 8'hFC;
                    bit_counter <= WRITE_DATA_SIZE * 8 - 1;
                end
                SEND_BLOCK: begin
                    if(sclk_sig == 1) begin
                        if(bit_counter[2:0] == 3'b000) begin
                            if(bit_counter == 0) begin
                                state <= RECEIVE_BYTE_WAIT;
                                return_state <= SEND_BLOCK_WAIT;
                            end
                            else if(bit_counter <= 16) begin
                                data_sig <= 8'hFC;
                            end
                            else begin
                                data_sig <= i_data;
                                ready_for_next_byte <= 1;
                            end
                        end
                        else begin
                            data_sig <= {data_sig[6:0], 1'b1};
                        end
                        bit_counter <= bit_counter - 1;
                    end
                    else begin
                        ready_for_next_byte <= 0;
                    end
                    sclk_sig <= ~sclk_sig;
                end
                SEND_BLOCK_WAIT: begin
                    if (sclk_sig == 1) begin
                        if (miso == 1) begin
                            cmd_out <= 56'hFD_FF_FF_FF_FF_FF_FF;
                            bit_counter <= 15;
                            state <= (block_counter == 1) ? SEND_STOP_TOKEN : SEND_BLOCK_INIT;
                            block_counter <= block_counter - 1;
                            cmd_mode <= 1;
                        end
                    end
                    sclk_sig = ~sclk_sig;
                end
                SEND_STOP_TOKEN: begin
                    if (sclk_sig == 1) begin
                        if (bit_counter == 0) begin
                            state <= STOP_TOKEN_WAIT;
                        end
                        else begin
                            bit_counter <= bit_counter - 1;
                            cmd_out <= {cmd_out[54:0], 1'b1};
                        end
                    end
                    sclk_sig <= ~sclk_sig;
                end
                STOP_TOKEN_WAIT: begin
                    if (sclk_sig == 1) begin
                        if (miso == 1) begin
                            state <= IDLE;
                            cmd_mode <= 1;
                        end
                    end
                    sclk_sig = ~sclk_sig;
                end
            endcase
        end
    end
endmodule
