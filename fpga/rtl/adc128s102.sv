// 50 MHz / (2*3) = 8.333 MHz, inside the S102 8..16 MHz conversion clock range.
// Commands select the NEXT conversion: discard -> IN0 -> IN1, never label
// the currently shifted command as the currently returned channel.
module adc128s102 #(parameter integer HALF_CLKS=3, CS_SETUP_CLKS=2*HALF_CLKS, NEG_EDGE_CAPTURE=0)(
    input wire clk,rst,start,dout, input wire [31:0] clock_stamp,
    output wire sclk,csn,din, output reg busy,done,valid,
    output reg [11:0] ia_raw,ib_raw, output reg [31:0] ia_stamp,ib_stamp
);
    reg launch;reg [15:0] tx;reg [1:0] index;reg format_ok;
    wire [15:0] rx;wire spi_busy,finished;
    // Default50MHz:60/60ns SCLK,capture80ns afterfall.
    // Actual25MHz:HALF1 gives40/40ns SCLK; NEG_EDGE_CAPTURE captures
    // 60ns afterpriorfall and20ns beforenextfall; shift onnextposedge.
    // CS_SETUP4 gives160ns lead,180ns tophysicalfirstcapture.
    // Devicehold4ns is typical, not a minimum; do not rely on it.
    spi_master #(.BITS(16),.HALF_CLKS(HALF_CLKS),.CPHA(0),.GAP_CLKS(5),.SYNC_INPUT(0),
      .CS_SETUP_CLKS(CS_SETUP_CLKS),.RX_DELAY_CLKS(1),.CAPTURE_NEGEDGE(NEG_EDGE_CAPTURE)) spi
      (clk,rst,launch,tx,dout,sclk,csn,din,spi_busy,finished,rx);
    always @(posedge clk) begin
      if(rst) begin busy<=0;done<=0;valid<=0;ia_raw<=0;ib_raw<=0;ia_stamp<=0;ib_stamp<=0;launch<=0;tx<=0;index<=0;format_ok<=0;end
      else begin
        launch<=0;done<=0;
        if(start&&!busy) begin busy<=1;valid<=0;tx<=0;launch<=1;index<=0;format_ok<=1;end
        if(busy&&finished) begin
          case(index)
            0: begin tx<=16'h0800; index<=1;launch<=1;ia_stamp<=clock_stamp;end
            1: begin ia_raw<=rx[11:0];ib_stamp<=clock_stamp;format_ok<=rx[15:12]==0;
                     tx<=0;index<=2;launch<=1;end
            2: begin ib_raw<=rx[11:0];
                     valid<=format_ok && rx[15:12]==0 && ia_raw>0 && ia_raw<4095 && rx[11:0]>0 && rx[11:0]<4095;
                     busy<=0;done<=1;end
          endcase
        end
      end
    end
endmodule
