`include "define.vh"

module framebuf (
    // for framebuffer write
    input  wire i_wclk,
    input  wire i_we,
    input  wire [`FB_ADDR_WIDTH-1:0] i_waddr,
    input  wire [`FB_PIX_WIDTH*2-1:0]  i_wdata,
    // for framebuffer read
    input  wire i_rclk,
    input  wire [`FB_ADDR_WIDTH-1:0] i_raddr,
    output wire [`FB_PIX_WIDTH-1:0]  o_rdata
);

    (* ram_style = "block" *) reg [`FB_PIX_WIDTH*2-1: 0] mem[0: (`FRAME_WIDTH*`FRAME_HEIGHT/(`RESIZE_RATE * `RESIZE_RATE * 2))-1];
    initial begin
        $readmemh("fbinit.mem", mem);
    end

    // write two pixels at once
    always @ (posedge i_wclk) begin
        if (i_we) begin
            mem[i_waddr >> 2] <= i_wdata;
        end
    end

    reg [`FB_ADDR_WIDTH-1:0] r_raddr;

    always @ (posedge i_rclk) begin
        r_raddr <= i_raddr;
    end

    assign o_rdata = (r_raddr[0]) ? mem[r_raddr >> 1][2*`FB_PIX_WIDTH-1: `FB_PIX_WIDTH]:
                                    mem[r_raddr >> 1][`FB_PIX_WIDTH-1: 0];


endmodule

module color_converter(
    input  wire [15:0] i_data,
    output wire [`FB_PIX_WIDTH-1:0]  o_data
    );

    wire [4:0] red   = i_data[15:11];
    wire [5:0] green = i_data[10:5];
    wire [4:0] blue  = i_data[4:0];

    assign o_data = {red[4:2], green[5:3], blue[4:2]};

endmodule

module VGA #(
    parameter H_FP = 16,
    parameter H_PW = 96,
    parameter H_MAX = 800,
    parameter V_FP = 10,
    parameter V_PW = 2,
    parameter V_MAX = 521,
    parameter BITS_WIDTH = 12,
    parameter PIX_WIDTH = 12,
    parameter VGABIT_WIDTH = 4
)
(
    input   wire    pix_clk,
    input   wire [PIX_WIDTH-1:0] frame_pix,
    output  reg     VGA_H_SYNC,
    output  reg     VGA_V_SYNC,
    output  reg [VGABIT_WIDTH-1:0]  VGA_RED,
    output  reg [VGABIT_WIDTH-1:0]  VGA_BLUE,
    output  reg [VGABIT_WIDTH-1:0]  VGA_GREEN,
    output  wire [`FB_ADDR_WIDTH-1:0] frame_addr
);


    // counters
    reg [BITS_WIDTH - 1:0] h_cnt;
    reg [BITS_WIDTH - 1:0] v_cnt;


    // syncronization signal
    reg h_sync;
    reg v_sync;


    // pix data is valid
    wire valid;

    // colors
    wire [BITS_WIDTH - 1:0] cnt_bg_h;
    wire [BITS_WIDTH - 1:0] cnt_bg_v;
    reg [VGABIT_WIDTH-1:0] bg_red;
    reg [VGABIT_WIDTH-1:0] bg_blue;
    reg [VGABIT_WIDTH-1:0] bg_green;


    initial begin
    	h_cnt <= 0;
    	v_cnt <= 0;
    end


    // horizon counter

    always @(posedge pix_clk) begin
        if(h_cnt == (H_MAX - 1)) begin
            h_cnt <= 0;
        end else begin
            h_cnt <= h_cnt + 1;
        end
    end

    // vertical counter
    always @(posedge pix_clk) begin
        if(h_cnt == (H_MAX - 1) && v_cnt == (V_MAX - 1)) begin
            v_cnt <= 0;
        end else if(h_cnt == (H_MAX - 1)) begin
            v_cnt <= v_cnt + 1;
        end
    end

    // horizontal sync.
    always @(posedge pix_clk) begin
        if( (h_cnt >= (H_FP + `FRAME_WIDTH - 1)) && (h_cnt < (H_FP + `FRAME_WIDTH + H_PW - 1)) ) begin
            h_sync <= 1;
        end else begin
            h_sync <= 0;
        end
    end

    // vertical sync.
    always @(posedge pix_clk) begin
        if( (v_cnt >= (V_FP + `FRAME_HEIGHT - 1)) && (v_cnt < (V_FP + `FRAME_HEIGHT + V_PW - 1)) ) begin
            v_sync <= 1;
        end else begin
            v_sync <= 0;
        end
    end

    // validation
    assign valid = ((h_cnt < `FRAME_WIDTH) && (v_cnt < `FRAME_HEIGHT));

    reg [`FB_ADDR_WIDTH-1 :0] x = 0, y = 0;

    assign frame_addr = x + y * `FRAME_WIDTH / `RESIZE_RATE;
    // pixel address counter
    reg[$clog2(`RESIZE_RATE): 0] xwait = 0, ywait = 0;

    always @(posedge pix_clk) begin
        if (valid) begin
            if (xwait == `RESIZE_RATE-1) begin
                if (x == `FRAME_WIDTH/`RESIZE_RATE -1) begin
                    x <= 0;
                end else begin
                    x <= x + 1;
                end
            end
            xwait <= (xwait == `RESIZE_RATE-1) ? 0 : xwait + 1;
            if ((x == `FRAME_WIDTH/`RESIZE_RATE -1) && xwait == `RESIZE_RATE-1) begin
                if (ywait == `RESIZE_RATE-1) begin
                    if (y == `FRAME_HEIGHT/`RESIZE_RATE -1) begin
                        y <= 0;
                    end else begin
                        y <= y + 1;
                    end
                end
                ywait <= (ywait == `RESIZE_RATE-1) ? 0 : ywait + 1;
            end
        end
    end


    always @(posedge pix_clk) begin
        if (valid)  begin
        	bg_red <= frame_pix[PIX_WIDTH-1:PIX_WIDTH-VGABIT_WIDTH];
        	bg_green <= frame_pix[PIX_WIDTH-VGABIT_WIDTH-1: PIX_WIDTH-2*VGABIT_WIDTH];
        	bg_blue <= frame_pix[PIX_WIDTH-2*VGABIT_WIDTH-1: PIX_WIDTH-3*VGABIT_WIDTH];
        end else begin
            bg_red <= 0;
            bg_green <= 0;
            bg_blue <= 0;
        end
    end



    always @(posedge pix_clk) begin
        VGA_BLUE <= bg_blue;
        VGA_RED <= bg_red;
        VGA_GREEN <= bg_green;
        VGA_H_SYNC <= h_sync;
        VGA_V_SYNC <= v_sync;
    end
endmodule
