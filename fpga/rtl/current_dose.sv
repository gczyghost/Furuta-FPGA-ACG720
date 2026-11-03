// Original v27: 15 complete seconds + current partial second, explicit long mode.
// Strict lifetime quota remains available. Threshold stays 1 A^2 s (Q32).
module current_dose(input wire clk,rst,enable,rolling,add,
 input wire [63:0] increment,output reg done,fault,output reg [63:0] dose);
 // Separate scalar banks keep physical reset on every stored bit. Do not infer
 // a reset-emulated RAM with per-word valid flags and two asynchronous reads.
 wire [1023:0] bank_data;reg [3:0] index;reg [10:0] ticks;
 wire rotate=rolling&&ticks==1999;
 wire [3:0] next_index=index+1'b1;
 reg pending,rotate_latched,rolling_latched;
 reg [63:0] base,increment_latched;
 wire [63:0] next_dose=base+increment_latched;
 wire [63:0] expired=bank_data[next_index*64+:64];
 wire [63:0] bucket_sum=bank_data[index*64+:64]+increment_latched;
 genvar g;
 generate for(g=0;g<16;g=g+1)begin:bank
  reg [63:0] value;
  assign bank_data[g*64+:64]=value;
  always @(posedge clk)begin
   if(rst||!enable)value<=0;
   else if(pending&&rolling_latched)begin
    if(rotate_latched&&next_index==g)value<=increment_latched;
    else if(!rotate_latched&&index==g)value<=bucket_sum;
   end
  end
 end endgenerate
 always @(posedge clk)begin
  if(rst||!enable)begin
   done<=0;fault<=0;dose<=0;index<=0;ticks<=0;pending<=0;base<=0;increment_latched<=0;rotate_latched<=0;rolling_latched<=0;
  end else begin
   done<=0;
   if(add&&!fault&&!pending)begin
    base<=dose-(rotate?expired:64'd0);increment_latched<=increment;
    rotate_latched<=rotate;rolling_latched<=rolling;pending<=1;
   end
   if(pending)begin
    pending<=0;
    dose<=next_dose;done<=1;
    if(next_dose>=64'd4294967296)fault<=1;
    if(rolling_latched)begin
     if(rotate_latched)begin index<=next_index;ticks<=0;end
     else ticks<=ticks+1'b1;
    end
   end
  end
 end
endmodule

