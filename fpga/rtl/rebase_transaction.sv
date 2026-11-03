// Experimental owner-side transaction preparation; not connected to board RTL.
// All recipients consume the combinational commit at the SAME rising edge.
// owner_idle must cover every recipient; sample_start covers every new control
// computation. The owner retries a cancelled request at a later idle boundary.
module rebase_transaction(input wire clk,rst,enable,stop,fault,
 input wire request,owner_idle,sample_start,command_pending,reverse,
 input wire signed [31:0] goal,
 output wire busy,commit,output reg complete,cancelled,rejected,
 output reg signed [31:0] shift_q16,shift_counts,shift_turns);
 import fixed_math::*;
 reg [2:0] state;
 reg negative,reverse_latched;reg [31:0] magnitude,accumulated;reg [4:0] turns;
 // Derived from the original FP32 branch and roundf, exhaustively checked over
 // every Q16 goal within +/-64 rad by rebase_reference_boundaries.cpp.
 function automatic [31:0] threshold(input [4:0] index);
  case(index)
   0:threshold=205953;1:threshold=617663;2:threshold=1029438;3:threshold=1441212;
   4:threshold=1852987;5:threshold=2264762;6:threshold=2676537;7:threshold=3088312;
   8:threshold=3500086;9:threshold=3911861;default:threshold=32'hffffffff;
  endcase
 endfunction
 wire abort_request=!enable||stop||fault||sample_start||command_pending;
 assign busy=state!=0;
 assign commit=state==3&&owner_idle&&!abort_request&&!rst;
 always @(posedge clk)begin
  if(rst)begin
   state<=0;negative<=0;reverse_latched<=0;magnitude<=0;accumulated<=0;turns<=0;
   complete<=0;cancelled<=0;rejected<=0;shift_q16<=0;shift_counts<=0;shift_turns<=0;
  end else begin
   complete<=0;cancelled<=0;rejected<=0;
   if(abort_request)begin
    cancelled<=busy;state<=0;shift_q16<=0;shift_counts<=0;shift_turns<=0;
   end else case(state)
    0:if(request&&owner_idle)begin
     negative<=goal[31];reverse_latched<=reverse;magnitude<=goal[31]?(~goal+32'd1):goal;
     accumulated<=0;turns<=0;shift_q16<=0;shift_counts<=0;shift_turns<=0;state<=1;
    end
    1:begin
     // Proposed frame bound, not a substitute for trajectory/current protection.
     if(magnitude>32'd4194304)begin rejected<=1;complete<=1;state<=0;end
     else if(magnitude<32'd205953)begin complete<=1;state<=0;end
     else state<=2;
    end
    2:if(turns<10&&magnitude>=threshold(turns))begin
     accumulated<=accumulated+32'd411775;turns<=turns+1'b1;
    end else begin
     shift_q16<=negative?-$signed(accumulated):$signed(accumulated);
     shift_counts<=(negative^reverse_latched)?-($signed({1'b0,turns})*32'sd16384):($signed({1'b0,turns})*32'sd16384);
     shift_turns<=negative?-$signed({1'b0,turns}):$signed({1'b0,turns});state<=3;
    end
    3:if(commit)begin complete<=1;state<=0;end
    default:begin rejected<=1;state<=0;end
   endcase
  end
 end
endmodule
