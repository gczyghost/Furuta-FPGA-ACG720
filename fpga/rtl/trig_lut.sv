module trig_lut #(parameter FILE="rom/sine_q16.hex")(
 input wire clk,rst,start,input wire [15:0] phase,
 output reg done, output reg signed [31:0] sine,cosine);
 reg signed [31:0] rom[0:1023];
 reg signed [31:0] s0,s1,c0,c1; reg [5:0] frac;reg pending;
 wire [9:0] si=phase[15:6],sj=si+10'd1,ci=si+10'd256,cj=si+10'd257;
 initial $readmemh(FILE,rom);
 always @(posedge clk) begin
   if(rst) begin done<=0;pending<=0;sine<=0;cosine<=65536;s0<=0;s1<=0;c0<=0;c1<=0;frac<=0;end
   else begin
     done<=pending;pending<=start;
     if(start) begin s0<=rom[si];s1<=rom[sj];c0<=rom[ci];c1<=rom[cj];frac<=phase[5:0];end
     if(pending) begin sine<=s0+((64'(s1-s0)*$signed({1'b0,frac}))>>>6);
                         cosine<=c0+((64'(c1-c0)*$signed({1'b0,frac}))>>>6);end
   end
 end
endmodule
