// Telemetry-only encoder velocity, separate from the control observer.
// Reject discontinuous timestamps; Q32 filter state retains sub-Q16 updates.
module sensor_velocity #(parameter integer SAMPLE_CLKS=25000)(
 input wire clk,rst,sample,valid,reverse,input wire [13:0] angle,input wire [31:0] stamp,
 output reg signed [31:0] raw_velocity,filtered_velocity,output reg velocity_valid,
 output reg [31:0] velocity_stamp);
 reg seen;reg [13:0] previous;reg [31:0] previous_stamp;
 reg [2:0] state;reg signed [14:0] delta;reg signed [63:0] product,filter_state,filter_delta;
 reg signed [31:0] raw_next;reg [31:0] pending_stamp;
 reg divide_start,divide_negative;reg [63:0] divide_numerator;wire divide_done;wire [63:0] divide_quotient;
 udiv64 filter_divider(clk,rst||sample,divide_start,divide_numerator,32'd21,,divide_done,divide_quotient);
 wire [13:0] modulo_delta=angle-previous;
 wire signed [14:0] signed_delta=modulo_delta[13] ? $signed({1'b0,modulo_delta})-15'sd16384:$signed({1'b0,modulo_delta});
 always @(posedge clk)begin
  if(rst)begin seen<=0;previous<=0;previous_stamp<=0;state<=0;delta<=0;product<=0;filter_state<=0;filter_delta<=0;
   raw_velocity<=0;filtered_velocity<=0;velocity_valid<=0;velocity_stamp<=0;pending_stamp<=0;raw_next<=0;
   divide_start<=0;divide_negative<=0;divide_numerator<=0;end
  else begin
   divide_start<=0;
   if(sample)begin
   previous<=angle;previous_stamp<=stamp;pending_stamp<=stamp;seen<=valid;velocity_valid<=0;
   if(valid&&seen&&(stamp-previous_stamp)==SAMPLE_CLKS)begin delta<=reverse?-signed_delta:signed_delta;state<=1;end
   else begin state<=0;filter_state<=0;end
  end else case(state)
   // 2*pi * 65536 * 2000 / 16384 = 50265.482457...
   1:begin product<=delta*64'sd52707178533;state<=2;end
   2:begin raw_next<=product>>>20;state<=3;end
   3:begin filter_delta<=$signed(raw_next)*64'sd65536-filter_state;state<=4;end
   // Preserve signed truncation toward zero without a wide combinational '/'.
   4:begin divide_numerator<=filter_delta[63]?-$unsigned(filter_delta):$unsigned(filter_delta);
      divide_negative<=filter_delta[63];divide_start<=1;state<=5;end
   5:if(divide_done)begin filter_delta<=divide_negative?-$signed(divide_quotient):$signed(divide_quotient);state<=6;end
   6:begin filter_state<=filter_state+filter_delta;state<=7;end
   7:begin raw_velocity<=raw_next;filtered_velocity<=filter_state>>>16;velocity_stamp<=pending_stamp;velocity_valid<=1;state<=0;end
  endcase
  end
 end
endmodule
