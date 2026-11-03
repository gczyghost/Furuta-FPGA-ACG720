// Measured OID updates DO after rising SCK (up to 0.42 us on this bench).
// Capture on falling SCK, 2 us after launch at 250 kHz. Use bits 17:6
// as its 12-bit count; undocumented status bits are NOT invented as parity.
module encoder_pendulum #(parameter integer HALF_CLKS=100, GAP_CLKS=250)(
    input wire clk,rst,start,miso, input wire [31:0] clock_stamp,
    output wire sck,csn, output reg busy,done,valid,
    output reg [11:0] angle, output reg [31:0] stamp,
    output reg [17:0] raw_frame
);
    reg launch,second_read;reg [17:0] first;
    wire [17:0] rx; wire unused_mosi,spi_busy,finished;
    wire signed [11:0] difference=rx[17:6]-first[17:6];
    // 250 kHz pair separation < 90 us: 4 counts plus ceil(35*90us*4096/tau).
    // More conservative than the firmware's 0.75 ms allowance.
    spi_master #(.BITS(18),.HALF_CLKS(HALF_CLKS),.CPHA(1),.GAP_CLKS(GAP_CLKS)) spi
      (clk,rst,launch,18'b0,miso,sck,csn,unused_mosi,spi_busy,finished,rx);
    always @(posedge clk) begin
      if(rst) begin busy<=0;done<=0;valid<=0;angle<=0;stamp<=0;raw_frame<=0;launch<=0;second_read<=0;first<=0;end
      else begin
        launch<=0;done<=0;
        if(start&&!busy) begin busy<=1;valid<=0;second_read<=0;launch<=1;stamp<=clock_stamp;end
        if(busy&&finished) begin
          if(!second_read) begin first<=rx;second_read<=1;launch<=1;end
          else begin
            raw_frame<=rx;angle<=rx[17:6];
            valid<=first!=0 && rx!=0 && first!=18'h3ffff && rx!=18'h3ffff && difference<=7 && difference>=-7;
            busy<=0;done<=1;
          end
        end
      end
    end
endmodule
