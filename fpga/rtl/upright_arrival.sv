// v27 UprightMotion::Arrival: 500 ms speed RMS then 500 ms good dwell.
// RAM is cleared by actual sequential writes, avoiding inferred RAM reset
// assumptions. Clearing takes 500 clocks (10 us), not 500 control samples.
module upright_arrival(input wire clk,rst,begin_motion,enable,start,ended,
 input wire signed [31:0] q_error,beta,w,
 output wire arrived,output wire ready,output reg done);
 reg [63:0] squares[0:499],read_square,square,sum;
 reg [8:0] index,clear_index;reg [9:0] count,dwell;
 reg clearing,half;reg [1:0] state;reg end_sample,position_good;
 assign ready=!clearing;
 assign arrived=enable&&!clearing&&dwell>=500;
 always @(posedge clk)begin
   read_square<=squares[index];
   if(!rst&&clearing)squares[clear_index]<=0;
   else if(!rst&&state==2)squares[index]<=square;
 end
 always @(posedge clk)begin
  if(rst||begin_motion)begin clearing<=1;clear_index<=0;index<=0;count<=0;dwell<=0;sum<=0;state<=0;half<=0;done<=0;end
  else begin
   done<=0;
   if(clearing)begin
     if(clear_index==499)begin clearing<=0;clear_index<=0;end
     else clear_index<=clear_index+1'b1;
   end else case(state)
     0:if(enable&&start)begin
       half<=!half;
       if(!half)begin
         square<=64'($signed(w))*64'($signed(w));
         end_sample<=ended;position_good<=q_error>=-2287&&q_error<=2287&&beta>=-2287&&beta<=2287;state<=1;
       end else done<=1;
     end
     1:begin sum<=sum+square-read_square;count<=count<500?count+1'b1:count;state<=2;end
     2:begin
       index<=index==499?0:index+1'b1;
       if(end_sample&&position_good&&count==500&&sum<64'd48318382080)dwell<=dwell<500?dwell+1'b1:dwell;
       else dwell<=0;
       done<=1;state<=0;
     end
   endcase
  end
 end
endmodule
