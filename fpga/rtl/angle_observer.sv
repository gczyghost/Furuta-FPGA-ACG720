// Q24 internal states preserve sub-Q16 motion between samples.
module angle_observer #(parameter integer PERIODIC_POSITION=0)(input wire clk,rst,start,input wire signed [31:0] angle,
 output reg done,output reg signed [31:0] position,velocity,input wire slow,
 input wire rebase,input wire signed [31:0] shift,output wire idle,output reg rebase_ack,input wire restart);
 reg initialized,slow_latched;reg [3:0] state;
 reg signed [63:0] position_fine,velocity_fine,predicted,latched,residual;
 reg signed [63:0] dt_product,position_product,velocity_product,next_position,next_velocity;
 wire signed [63:0] shift_fine=$signed(shift)*64'sd256;
 assign idle=state==0&&!start;
 function automatic signed [63:0] clip_fine(input signed [63:0] x);
  clip_fine=x>64'sd549755813632 ? 64'sd549755813632:x< -64'sd549755813888 ? -64'sd549755813888:x;
 endfunction
 // Exact signed remainder, one input bit per clock. rem_mag < TAU at
 // every boundary, so trial < 2*TAU fits in 28 bits. This replaces the
 // 157-level combinational remainder cone without changing Q24 rounding.
 reg [63:0] wrap_bits;
 reg [26:0] rem_mag;
 reg [6:0] wrap_count;
 reg wrap_negative;
 reg [1:0] wrap_target;
 wire [27:0] trial={rem_mag,wrap_bits[63]};
 wire [27:0] reduced=trial>=28'd105414400 ? trial-28'd105414400:trial;
 wire signed [63:0] signed_rem=wrap_negative ? -$signed({37'd0,rem_mag}):$signed({37'd0,rem_mag});
 wire signed [63:0] wrapped=signed_rem>=64'sd52707200 ? signed_rem-64'sd105414400:
                           signed_rem< -64'sd52707200 ? signed_rem+64'sd105414400:signed_rem;
 task automatic begin_wrap(input signed [63:0] x,input [1:0] target);
  begin
   wrap_bits<=x[63] ? -x:x;rem_mag<=0;wrap_negative<=x[63];
   wrap_count<=0;wrap_target<=target;state<=6;
  end
 endtask
 always @(posedge clk)begin
  if(rst)begin
   wrap_bits<=0;rem_mag<=0;wrap_count<=0;wrap_negative<=0;wrap_target<=0;
   initialized<=0;done<=0;position<=0;velocity<=0;state<=0;slow_latched<=0;
   position_fine<=0;velocity_fine<=0;predicted<=0;latched<=0;residual<=0;
   dt_product<=0;position_product<=0;velocity_product<=0;next_position<=0;next_velocity<=0;rebase_ack<=0;
  end else begin
   done<=0;rebase_ack<=0;
   if(rebase&&idle)begin
    position<=position-shift;position_fine<=position_fine-shift_fine;
    predicted<=predicted-shift_fine;latched<=latched-shift_fine;next_position<=next_position-shift_fine;
    rebase_ack<=1;
   end else case(state)
    0:if(start)begin
      if(!initialized||(restart===1'b1))begin
       latched<=$signed(angle)*64'sd256;
       begin_wrap($signed(angle)*64'sd256,0);
      end else begin
       dt_product<=velocity_fine*64'sd2147484;
       latched<=$signed(angle)*64'sd256;slow_latched<=slow;state<=1;
      end
    end
    1:begin predicted<=clip_fine(position_fine+(dt_product>>>32));state<=2;end
    2:begin begin_wrap(latched-predicted,1);end
    3:begin
     position_product<=residual*(slow_latched ? 64'sd3991:64'sd11259);
     velocity_product<=residual*(slow_latched ? 64'sd125372:64'sd1060333);state<=4;
    end
    4:begin
     next_position<=clip_fine(predicted+(position_product>>>16));
     next_velocity<=clip_fine(velocity_fine+(velocity_product>>>16));state<=5;
    end
    // Both observer instances use the same schedule, including continuous
    // arm position. motor_control requires their done pulses to coincide.
    5:begin begin_wrap(next_position,2);end
    6:begin
     rem_mag<=reduced[26:0];wrap_bits<={wrap_bits[62:0],1'b0};
     if(wrap_count==63)state<=7;else wrap_count<=wrap_count+1'b1;
    end
    7:case(wrap_target)
     0:begin
      position_fine<=PERIODIC_POSITION!=0?wrapped:latched;
      position<=(PERIODIC_POSITION!=0?wrapped:latched)>>>8;
      velocity<=0;velocity_fine<=0;initialized<=1;done<=1;state<=0;
     end
     1:begin residual<=wrapped;state<=3;end
     2:begin
      position_fine<=PERIODIC_POSITION!=0?wrapped:next_position;
      position<=(PERIODIC_POSITION!=0?wrapped:next_position)>>>8;
      velocity_fine<=next_velocity;velocity<=next_velocity>>>8;done<=1;state<=0;
     end
    endcase
   endcase
  end
 end
endmodule
