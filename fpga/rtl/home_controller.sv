// Experimental idle-only coordinate translation; not integrated.
// ArmZero::HomeController, Q16 SI, 2 kHz. The caller owns motion/thermal
// qualification and STOP. begin_move initializes a shortest-path reference;
// terminal selects reset(goal,goal), moveSeconds=0 with inherited state.
module home_controller(input wire clk,rst,enable,start,begin_move,terminal,
 input wire signed [31:0] q,a,w,v,requested,seed_integral,
 input wire [10:0] seed_settled,
 output reg busy,done,output reg signed [31:0] command,reference_q,reference_w,reference_acc,
 output wire settled,input wire capture_enable,output wire signed [31:0] planned_goal,input wire rebase,input wire signed [31:0] rebase_shift,output reg rebase_ack,
 output wire signed [63:0] integral_state,input wire precise_seed,input wire signed [63:0] precise_integral,output wire [10:0] settled_state) /* synthesis syn_dspstyle = "logic" */;
 import fixed_math::*;
 reg signed [31:0] wrap_input,wrap_signed;
 reg [31:0] wrap_mag,wrap_multiple,wrap_rem;
 reg [45:0] wrap_product /* synthesis syn_dspstyle = "dsp" */;
 reg wrap_sign;reg [1:0] wrap_target;reg [5:0] wrap_resume;
 wire signed [31:0] wrapped=wrap_signed>PI?wrap_signed-TAU:wrap_signed< -PI?wrap_signed+TAU:wrap_signed;
 reg [5:0] state;reg [31:0] tick,duration_ticks;
 reg [10:0] rest_ticks;reg is_terminal,negative;
 reg signed [31:0] origin,goal,delta,xq,xa,xw,xv,vel_scale,acc_scale;
 reg signed [31:0] delta_abs,requested_difference;reg [31:0] trajectory_tick;
 reg signed [31:0] s,s2,s3,s4,s5,pcurve,vcurve,acurve,eq,ea,ew,peq,pea,ff,friction,raw;
 reg signed [63:0] integral;
 assign integral_state=integral;assign settled_state=rest_ticks;
 reg past_trajectory;reg signed [31:0] settle_error;
 reg replan;reg signed [31:0] capture_base,capture_coupled,capture_scale;
 reg signed [31:0] mul_a,mul_b;
 reg signed [63:0] mul_product /* synthesis syn_dspstyle = "dsp" */;
 wire signed [31:0] mul_value=sat64(mul_product>>>16);
 always @*begin
   mul_a=0;mul_b=0;
   case(state)
    32:begin mul_a=delta_abs;mul_b=6000;end
    2:begin mul_a=delta_abs;mul_b=2000;end
    4:begin mul_a=$signed(duration_ticks);mul_b=$signed(duration_ticks);end
    36:begin mul_a=delta_abs;mul_b=4000000;end
    8:begin mul_a=s;mul_b=s;end
    9:begin mul_a=s2;mul_b=s;end
    10:begin mul_a=s3;mul_b=s;end
    11:begin mul_a=s4;mul_b=s;end
    13:begin mul_a=delta;mul_b=pcurve;end
    14:begin mul_a=vel_scale;mul_b=vcurve;end
    15:begin mul_a=acc_scale;mul_b=acurve;end
    21:begin mul_a=capture_scale;mul_b=capture_coupled;end
   endcase
 end
 always @(posedge clk)mul_product<=mul_a*mul_b;
 reg signed [63:0] integral_step;
 wire signed [63:0] next_integral=integral+integral_step;
 wire signed [63:0] clipped_integral=next_integral>64'sd84443352006656 ? 64'sd84443352006656:
    next_integral< -64'sd84443352006656 ? -64'sd84443352006656:next_integral;
 reg div_start;reg [63:0] numerator;reg [31:0] denominator;
 wire div_busy,div_done;wire [63:0] quotient;
 udiv64 divider(clk,rst||!enable,div_start,numerator,denominator,div_busy,div_done,quotient);
 assign settled=enable&&rest_ticks>=1000;
 assign planned_goal=goal;
 always @(posedge clk)begin
  if(rst||!enable)begin
   wrap_input<=0;wrap_signed<=0;wrap_mag<=0;wrap_multiple<=0;wrap_rem<=0;wrap_product<=0;wrap_sign<=0;wrap_target<=0;wrap_resume<=0;
   rebase_ack<=0;busy<=0;done<=0;state<=0;tick<=0;rest_ticks<=0;integral<=0;command<=0;
   reference_q<=0;reference_w<=0;reference_acc<=0;div_start<=0;
   past_trajectory<=0;settle_error<=0;duration_ticks<=6000;origin<=0;goal<=0;delta<=0;is_terminal<=0;replan<=0;
  end else begin
   rebase_ack<=0;done<=0;div_start<=0;
   case(state)
    0:if(start)begin
      busy<=1;xq<=q;wrap_input<=a;wrap_target<=0;state<=41;xw<=w;xv<=v;
      if(begin_move)begin
        tick<=0;is_terminal<=terminal;integral<=(precise_seed===1'b1) ? precise_integral:$signed(seed_integral)*64'sd4294967296;rest_ticks<=seed_settled;replan<=0;
        origin<=terminal?requested:q;requested_difference<=requested-q;wrap_resume<=38;
      end else if(replan)begin replan<=0;wrap_resume<=1;end
      else wrap_resume<=6;
    end
    38:begin if(is_terminal)begin delta<=0;state<=39;end
      else begin wrap_input<=requested_difference;wrap_target<=1;state<=41;end end
    39:begin goal<=is_terminal?origin:origin+delta;state<=1;end
    40:begin numerator<={16'd0,trajectory_tick,16'd0};denominator<=duration_ticks;div_start<=1;state<=7;end
    1:begin delta_abs<=absq(delta);state<=32;end
    32:state<=33;
    33:begin duration_ticks<=is_terminal?0:6000+(mul_product>>16);state<=2;end
    2:if(is_terminal)begin vel_scale<=0;acc_scale<=0;state<=6;end
      else state<=34;
    34:begin numerator<=mul_product;denominator<=duration_ticks;div_start<=1;state<=3;end
    3:if(div_done)begin vel_scale<=delta[31]?-$signed(quotient[31:0]):$signed(quotient[31:0]);state<=4;end
    4:state<=35;
    35:begin denominator<=mul_product[31:0];state<=36;end
    36:state<=37;
    37:begin numerator<=mul_product;div_start<=1;state<=5;end
    5:if(div_done)begin acc_scale<=delta[31]?-$signed(quotient[31:0]):$signed(quotient[31:0]);state<=6;end
    6:begin
      tick<=tick+1'b1;
      if(capture_enable&&absq(xa)>22938)begin
        capture_base<=reference_q-xq-mul(9830,xw);
        capture_coupled<=mul(26214,xa)+mul(3277,xv);
        capture_scale<=clip((39322-absq(xa))*4,0,65536);state<=21;
      end else if(is_terminal||tick>=1000+duration_ticks)begin reference_q<=goal;reference_w<=0;reference_acc<=0;state<=16;end
      else if(tick<=1000)begin reference_q<=origin;reference_w<=0;reference_acc<=0;state<=16;end
      else begin trajectory_tick<=tick-1000;state<=40;end
    end
    7:if(div_done)begin s<=quotient[31:0];state<=8;end
    8:state<=24;
    24:begin s2<=mul_value;state<=9;end
    9:state<=25;
    25:begin s3<=mul_value;state<=10;end
    10:state<=26;
    26:begin s4<=mul_value;state<=11;end
    11:state<=27;
    27:begin s5<=mul_value;state<=12;end
    12:begin pcurve<=10*s3-15*s4+6*s5;vcurve<=30*s2-60*s3+30*s4;acurve<=60*s-180*s2+120*s3;state<=13;end
    13:state<=28;
    28:begin reference_q<=origin+mul_value;state<=14;end
    14:state<=29;
    29:begin reference_w<=mul_value;state<=15;end
    15:state<=30;
    30:begin reference_acc<=mul_value;state<=16;end
    16:begin
      eq<=xq-reference_q;ea<=xa+mul(941,reference_acc);ew<=xw-reference_w;
      numerator<=64'(absq(reference_w))<<16;denominator<=absq(reference_w)+2621;negative<=reference_w[31];
      if(reference_w==0)begin friction<=0;state<=18;end
      else begin div_start<=1;state<=17;end
    end
    17:if(div_done)begin friction<=negative?-$signed(quotient[31:0]):$signed(quotient[31:0]);state<=18;end
    18:begin peq<=eq+mul(98,ew);pea<=ea+mul(98,xv);ff<=mul(888,reference_acc)+mul(116,reference_w)+mul(1610,friction);state<=19;end
    19:begin past_trajectory<=is_terminal||tick>1000+duration_ticks;settle_error<=goal-xq;integral_step<=$signed(peq)*64'sd6442451;raw<=ff-mul(163840,peq)+mul(16384,pea)-mul(13107,ew)-$signed(integral>>>32);state<=20;end
    20:begin
      if(absq(peq)<16384&&absq(pea)<7864&&absq(ew)<131072&&(absq(raw)<58982||raw[31]==peq[31]))integral<=clipped_integral;
      if(past_trajectory&&absq(settle_error)<=571&&absq(xa)<=571&&absq(xw)<=3276&&absq(xv)<=3276)
        rest_ticks<=rest_ticks<1000?rest_ticks+1'b1:rest_ticks;
      else rest_ticks<=0;
      command<=clip(raw,-65536,65536);busy<=0;done<=1;state<=0;
    end
    21:state<=31;
    31:begin
      // EnabledSession::captureDown: hold the previous reference softly,
      // reset home trajectory to measured position, retain the requested goal.
      command<=clip(capture_base+mul_value,-7864,7864);
      origin<=xq;wrap_input<=goal-xq;wrap_target<=2;state<=41;tick<=0;is_terminal<=0;replan<=1;
      reference_q<=xq;reference_w<=0;reference_acc<=0;integral<=0;rest_ticks<=0;
    end
    41:begin wrap_sign<=wrap_input[31];wrap_mag<=wrap_input[31]?-$unsigned(wrap_input):$unsigned(wrap_input);state<=42;end
    42:begin wrap_product<=wrap_mag*14'd10430;state<=43;end
    43:begin wrap_multiple<=wrap_product[45:32]*19'd411775;state<=44;end
    44:begin wrap_rem<=wrap_mag-wrap_multiple;state<=45;end
    45:begin wrap_rem<=wrap_rem>=32'd411775 ? wrap_rem-32'd411775:wrap_rem;state<=46;end
    46:begin wrap_signed<=wrap_sign?-$signed(wrap_rem):$signed(wrap_rem);state<=47;end
    47:begin
      case(wrap_target)
       0:begin xa<=wrapped;state<=wrap_resume;end
       1:begin delta<=wrapped;state<=39;end
       2:begin delta<=wrapped;goal<=xq+wrapped;state<=48;end
      endcase
    end
    48:begin busy<=0;done<=1;state<=0;end
   endcase
   // Idle translation writes only coordinate state. Other state is idle already;
   // do not route the global commit through every controller register enable.
   if(rebase&&!busy&&!start)begin
    rebase_ack<=1;origin<=origin-rebase_shift;goal<=goal-rebase_shift;
    reference_q<=reference_q-rebase_shift;xq<=xq-rebase_shift;
   end
  end
 end
endmodule
