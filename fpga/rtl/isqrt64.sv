// Restoring square root, 32 iterations. Root is ceil(sqrt(x)) so vector
// saturation never overshoots because a norm was rounded down.
module isqrt64(input wire clk,rst,start,input wire [63:0] x,
 output reg busy,done,output reg [31:0] root);
 reg [63:0] rad;reg [65:0] remn;reg [31:0] acc;reg [5:0] count;
 reg [65:0] shifted,trial,next_rem;reg [31:0] next_acc;
 always @* begin
   shifted=(remn<<2)|{64'b0,rad[63:62]};trial=({34'b0,acc}<<2)|66'd1;
   next_rem=shifted>=trial ? shifted-trial:shifted;
   next_acc=(acc<<1)|(shifted>=trial);
 end
 always @(posedge clk) begin
   if(rst) begin busy<=0;done<=0;root<=0;rad<=0;remn<=0;acc<=0;count<=0;end
   else begin
     done<=0;
     if(start&&!busy) begin busy<=1;rad<=x;remn<=0;acc<=0;count<=0;end
     else if(busy) begin
       remn<=next_rem;acc<=next_acc;rad<=rad<<2;count<=count+1'b1;
       if(count==31) begin root<=next_acc+((next_rem!=0)&&(next_acc!=32'hffffffff));busy<=0;done<=1;end
     end
   end
 end
endmodule
