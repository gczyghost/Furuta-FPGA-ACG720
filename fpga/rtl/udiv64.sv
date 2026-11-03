// Unsigned 64/32 restoring divider. No combinational '/' on runtime operands.
module udiv64(input wire clk,rst,start,input wire [63:0] numerator,input wire [31:0] denominator,
 output reg busy,done,output reg [63:0] quotient);
 reg [63:0] num,q;reg [31:0] den;reg [32:0] remn;reg [6:0] count;
 wire [32:0] trial={remn[31:0],num[63]};wire take=trial>={1'b0,den};
 always @(posedge clk) begin
   if(rst) begin busy<=0;done<=0;quotient<=0;num<=0;q<=0;den<=1;remn<=0;count<=0;end
   else begin
     done<=0;
     if(start&&!busy) begin busy<=1;num<=numerator;den<=denominator;q<=0;remn<=0;count<=0;end
     else if(busy) begin
       num<=num<<1;remn<=take ? trial-den:trial;q<={q[62:0],take};count<=count+1'b1;
       if(count==63) begin quotient<=den==0 ? 64'hffffffffffffffff:{q[62:0],take};busy<=0;done<=1;end
     end
   end
 end
endmodule
