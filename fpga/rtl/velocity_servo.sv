// v27 VelocityController, 1 kHz reference/feedback in a 2 kHz current loop.
// High precision reference states avoid losing small acceleration increments.
// Entry, lease, physical path limits and handoff belong to the motion owner.
module velocity_servo #(parameter integer OWNER_IDLE_REBASE=0)(input wire clk,rst,enable,start,begin_motion,end_requested,
 input wire signed [15:0] target_rpm,input wire signed [31:0] origin,seed_integral,q,beta,w,v,
 output reg busy,done,fault,finished,output reg signed [31:0] command,reference_q,reference_beta,reference_w,reference_v,
 output wire signed [31:0] integral_out,input wire rebase,input wire signed [31:0] rebase_shift,output reg rebase_ack,
 input wire finite_mode) /* synthesis syn_dspstyle = "logic" */;
 // Registered exact normalization of the predicted angular error.
 reg [31:0] wrap_mag,wrap_multiple,wrap_rem;
 reg [45:0] wrap_product /* synthesis syn_dspstyle = "dsp" */;
 reg wrap_sign;reg signed [31:0] wrap_signed;
 import fixed_math::*;
 reg [5:0] state;reg half,ending;reg signed [15:0] rpm;
 reg [15:0] finite_tick,finite_ramp_ticks;
 reg finite_down,finite_negative,finite_complete;
 // Time-share a bounded exact multiplier for finite reference scaling.
 // Preserve the original low-64-bit product and signed >>>16 rounding,
 // without a combinational 64x32 multiply followed by *1000 in one clock.
 reg [63:0] finite_multiply_sum,finite_multiply_shift,finite_acc_scaled;
 reg [31:0] finite_multiply_factor;reg [4:0] finite_multiply_bit;
 reg finite_multiply_derivative;reg signed [63:0] finite_speed_product;
 reg signed [31:0] finite_u,finite_u2,finite_u3,finite_remaining,finite_remaining2,finite_poly,finite_shape,finite_derivative;
 wire [15:0] rpm_magnitude=rpm[15]?-$unsigned(rpm):$unsigned(rpm);
 // Tramp=max(1.5,1.875*|rpm|*pi/30/6), rounded up to a 1ms tick.
 // The shared reference/feedback path avoids a second set of 15 large ROMs.
 function automatic [15:0] speed_ramp_ticks(input [15:0] magnitude);
   case(magnitude)
    50:speed_ramp_ticks=1637;60:speed_ramp_ticks=1964;80:speed_ramp_ticks=2618;
    100:speed_ramp_ticks=3273;120:speed_ramp_ticks=3927;150:speed_ramp_ticks=4909;
    180:speed_ramp_ticks=5891;200:speed_ramp_ticks=6545;
    default:speed_ramp_ticks=1500;
   endcase
 endfunction
 reg signed [63:0] position,speed,acceleration,jerk,beta_product,next_beta,previous_beta,integral;
 reg signed [31:0] xq,xb,xw,xv,w_square,acc_ff,friction,ff,eq,eb,ew,ev,peq,peb,feedback,raw;
 reg [6:0] bad_ticks;
 reg [127:0] gain_rom[0:22],gain_read,gain_left,gain_right;reg [4:0] gain_address;
 reg [1:0] column;reg signed [31:0] gain[0:3],gain_diff,gain_fraction;
 reg signed [31:0] tanh_rom[0:512],tanh_read,tanh_left,tanh_value;reg [9:0] tanh_address;reg [7:0] tanh_fraction;
 reg div_start,beta_negative;reg [63:0] numerator;reg [31:0] denominator;
 reg signed [31:0] mul_a,mul_b;
 reg signed [63:0] mul_product /* synthesis syn_dspstyle = "dsp" */;
 wire signed [31:0] mul_value=sat64(mul_product>>>16);
 always @*begin
   mul_a=0;mul_b=0;
   case(state)
    5:begin mul_a=$signed(speed>>>16);mul_b=$signed(speed>>>16);end
    17:begin mul_a=gain_diff;mul_b=gain_fraction;end
    20:begin
      mul_a=gain[column];
      case(column)0:mul_b=peq;1:mul_b=peb;2:mul_b=ew;3:mul_b=ev;endcase
    end
    43:begin mul_a=finite_u;mul_b=finite_u;end
    44:begin mul_a=finite_u2;mul_b=finite_u;end
    53:begin mul_a=finite_remaining;mul_b=finite_remaining;end
    45:begin mul_a=finite_u3;mul_b=finite_poly;end
    56:begin mul_a=finite_u2;mul_b=finite_remaining2;end
   endcase
 end
 always @(posedge clk)mul_product<=mul_a*mul_b;
 wire div_done;wire [63:0] quotient;
 reg signed [63:0] requested_speed,jerk_difference,jerk_raw;
 reg signed [63:0] integral_step,integral_next;
 reg signed [31:0] speed_abs,tracking_scaled,seed_latched;reg signed [63:0] beta_delta;
 wire signed [31:0] tracking_speed=tracking_scaled>98304?tracking_scaled:98304;
 reg tracking_bad;
 assign integral_out=integral>>>32;
 function automatic signed [63:0] clip64(input signed [63:0] x,lo,hi);clip64=x<lo?lo:x>hi?hi:x;endfunction
 function automatic signed [63:0] dt(input signed [63:0] x);
  reg signed [63:0] product;begin product=x*64'sd4294967;dt=product>>>32;end
 endfunction
 function automatic [63:0] abs64(input signed [63:0] x);abs64=x<0?-x:x;endfunction
 initial begin $readmemh("rom/servo_gains_q16.hex",gain_rom);$readmemh("rom/servo_tanh_q16.hex",tanh_rom);end
 always @(posedge clk)begin gain_read<=gain_rom[gain_address];tanh_read<=tanh_rom[tanh_address];end
 udiv64 divider(clk,rst||!enable,div_start,numerator,denominator,,div_done,quotient);
 always @(posedge clk)begin
  if(rst||!enable)begin
   requested_speed<=0;jerk_difference<=0;jerk_raw<=0;wrap_mag<=0;wrap_multiple<=0;wrap_rem<=0;wrap_product<=0;wrap_sign<=0;wrap_signed<=0;
   rebase_ack<=0;busy<=0;done<=0;fault<=0;finished<=0;command<=0;state<=0;half<=0;ending<=0;rpm<=0;
   position<=0;speed<=0;acceleration<=0;previous_beta<=0;integral<=0;bad_ticks<=0;div_start<=0;
   seed_latched<=0;beta_delta<=0;reference_q<=0;reference_beta<=0;reference_w<=0;reference_v<=0;gain_address<=0;tanh_address<=0;
   finite_tick<=0;finite_ramp_ticks<=1500;finite_down<=0;finite_negative<=0;finite_complete<=0;
   finite_multiply_sum<=0;finite_multiply_shift<=0;finite_acc_scaled<=0;finite_multiply_factor<=0;
   finite_multiply_bit<=0;finite_multiply_derivative<=0;finite_speed_product<=0;
   finite_u<=0;finite_u2<=0;finite_u3<=0;finite_remaining<=0;finite_remaining2<=0;finite_poly<=0;finite_shape<=0;finite_derivative<=0;
  end else if(rebase&&OWNER_IDLE_REBASE==0)begin
   done<=0;rebase_ack<=0;
   if(busy)begin fault<=1;command<=0;end
   else begin rebase_ack<=1;position<=position-$signed(rebase_shift)*64'sd65536;reference_q<=reference_q-rebase_shift;end
  end else begin
   done<=0;div_start<=0;rebase_ack<=0;
   case(state)
    // In the integrated owner, commit requires !servo_busy and no start.
    // Keep the transaction local to idle instead of gating every work register.
    0:if(OWNER_IDLE_REBASE!=0&&rebase)begin
      rebase_ack<=1;position<=position-$signed(rebase_shift)*64'sd65536;reference_q<=reference_q-rebase_shift;
    end else if(start&&!fault)begin
      if(target_rpm>200||target_rpm< -200)begin fault<=1;command<=0;done<=1;end
      else if(!begin_motion&&half)begin half<=0;done<=1;end
      else begin
        half<=1;busy<=1;rpm<=target_rpm;ending<=end_requested;xq<=q;xb<=beta;xw<=w;xv<=v;
        if(begin_motion)begin position<=$signed(origin)*64'sd65536;speed<=0;acceleration<=0;previous_beta<=0;
          seed_latched<=seed_integral;bad_ticks<=0;finished<=0;finite_tick<=0;finite_complete<=0;end
        state<=begin_motion?27:37;
      end
    end
    27:begin integral<=$signed(clip(seed_latched,-7864,7864))*64'sd4294967296;state<=37;end
    // Preserve signed64bit arithmetic and saturation, isolate the rpm multiplier.
    37:begin requested_speed<=ending?0:$signed(rpm)*64'sd449767923;state<=(finite_mode===1'b1) ? 40:38;end
    40:begin finite_ramp_ticks<=speed_ramp_ticks(rpm_magnitude);state<=41;end
    41:begin
      if(finite_tick<65535)finite_tick<=finite_tick+1'b1;
      if(finite_tick<1500)begin speed<=0;acceleration<=0;state<=4;end
      else if(finite_tick<1500+finite_ramp_ticks)begin
        numerator<=64'(finite_tick-1500)<<16;denominator<=finite_ramp_ticks;div_start<=1;finite_down<=0;state<=42;
      end else if(finite_tick<2500+finite_ramp_ticks)begin speed<=requested_speed;acceleration<=0;state<=4;end
      else if(finite_tick<2500+2*finite_ramp_ticks)begin
        numerator<=64'(finite_tick-2500-finite_ramp_ticks)<<16;denominator<=finite_ramp_ticks;div_start<=1;finite_down<=1;state<=42;
      end else begin speed<=0;acceleration<=0;finite_complete<=1;state<=4;end
    end
    42:if(div_done)begin finite_u<=quotient[31:0];state<=43;end
    // The same saturated Q16 operation as mul(), using the registered DSP
    // already shared by feedback. Each consumer follows its launch state.
    43:begin finite_remaining<=65536-finite_u;state<=51;end
    51:begin finite_u2<=mul_value;state<=44;end
    44:begin finite_poly<=655360-15*finite_u+6*finite_u2;state<=52;end
    52:begin finite_u3<=mul_value;state<=53;end
    53:state<=54;
    54:begin finite_remaining2<=mul_value;state<=45;end
    45:state<=55;
    55:begin finite_shape<=mul_value;state<=56;end
    56:state<=57;
    57:begin finite_derivative<=30*mul_value;state<=46;end
    46:begin
      finite_multiply_sum<=0;finite_multiply_shift<=abs64(requested_speed);finite_multiply_factor<=$unsigned(finite_shape);
      finite_multiply_bit<=0;finite_multiply_derivative<=0;state<=48;
    end
    48:begin
      if(finite_multiply_factor[0])finite_multiply_sum<=finite_multiply_sum+finite_multiply_shift;
      finite_multiply_shift<=finite_multiply_shift<<1;finite_multiply_factor<=finite_multiply_factor>>1;
      if(finite_multiply_bit==31)state<=49;else finite_multiply_bit<=finite_multiply_bit+1'b1;
    end
    49:begin
      if(!finite_multiply_derivative)begin
        finite_speed_product<=requested_speed[63]?-$signed(finite_multiply_sum):$signed(finite_multiply_sum);
        finite_multiply_sum<=0;finite_multiply_shift<=abs64(requested_speed);finite_multiply_factor<=$unsigned(finite_derivative);
        finite_multiply_bit<=0;finite_multiply_derivative<=1;state<=48;
      end else begin finite_acc_scaled<=finite_multiply_sum>>16;state<=50;end
    end
    50:begin
      speed<=finite_down?requested_speed-(finite_speed_product>>>16):finite_speed_product>>>16;
      numerator<=finite_acc_scaled*1000;
      denominator<=finite_ramp_ticks;div_start<=1;finite_negative<=rpm[15]^finite_down;state<=47;
    end
    47:if(div_done)begin acceleration<=finite_negative?-$signed(quotient):$signed(quotient);state<=4;end
    38:begin jerk_difference<=requested_speed-speed;state<=39;end
    39:begin jerk_raw<=4*jerk_difference-4*acceleration;state<=1;end
    1:begin jerk<=clip64(jerk_raw,-64'sd17179869184,64'sd17179869184);state<=2;end
    2:begin acceleration<=clip64(acceleration+dt(jerk),-64'sd8589934592,64'sd8589934592);state<=3;end
    3:begin speed<=speed+dt(acceleration);state<=4;end
    4:begin position<=position+dt(speed);state<=5;end
    5:begin
      reference_q<=position>>>16;reference_w<=speed>>>16;
      beta_product<=acceleration*64'sd64215288;acc_ff<=(acceleration*64'sd57730879)>>>48;state<=24;
    end
    24:begin w_square<=mul_value;state<=6;end
    6:begin
      numerator<=abs64(beta_product>>>32)<<24;denominator<=16777216+((64'sd45623061*$signed(w_square))>>>24);
      beta_negative<=!acceleration[63];div_start<=1;state<=7;
    end
    7:if(div_done)begin next_beta<=beta_negative?-$signed(quotient):$signed(quotient);state<=28;end
    28:begin beta_delta<=next_beta-previous_beta;state<=8;end
    8:begin
      speed_abs<=absq(reference_w);
      reference_beta<=next_beta>>>16;reference_v<=(beta_delta*1000)>>>16;previous_beta<=next_beta;
      if(absq(reference_w)>=26214)begin tanh_value<=65536;state<=12;end
      else begin tanh_address<=(absq(reference_w)*5)>>8;tanh_fraction<=(absq(reference_w)*5)&255;state<=9;end
    end
    9:begin tanh_address<=tanh_address+1'b1;state<=10;end
    10:begin tanh_left<=tanh_read;state<=11;end
    11:begin tanh_value<=tanh_left+(($signed(tanh_read-tanh_left)*$signed({1'b0,tanh_fraction}))>>>8);state<=12;end
    12:begin
      friction<=speed[63]?-tanh_value:tanh_value;
      gain_address<=speed_abs>=1441137?21:speed_abs>>16;
      gain_fraction<=speed_abs>=1441137?64881:speed_abs&65535;state<=13;
    end
    13:begin ff<=acc_ff+((64'sd140385915*$signed(friction))>>>32);gain_address<=gain_address+1'b1;state<=14;end
    14:begin gain_left<=gain_read;state<=15;end
    15:begin gain_right<=gain_read;column<=0;state<=16;end
    16:begin gain_diff<=$signed(gain_right[column*32+:32])-$signed(gain_left[column*32+:32]);state<=17;end
    17:state<=25;
    25:begin gain[column]<=$signed(gain_left[column*32+:32])+mul_value;
      if(column==3)state<=18;else begin column<=column+1'b1;state<=16;end
    end
    18:begin tracking_scaled<=mul(13107,speed_abs);eq<=xq-reference_q;eb<=xb-reference_beta;ew<=xw-reference_w;ev<=xv-reference_v;state<=19;end
    19:begin peq<=eq+mul(98,ew);peb<=eb+mul(98,ev);column<=0;state<=30;end
    30:begin wrap_sign<=peb[31];wrap_mag<=peb[31]?-$unsigned(peb):$unsigned(peb);state<=31;end
    31:begin wrap_product<=wrap_mag*14'd10430;state<=32;end
    32:begin wrap_multiple<=wrap_product[45:32]*19'd411775;state<=33;end
    33:begin wrap_rem<=wrap_mag-wrap_multiple;state<=34;end
    34:begin wrap_rem<=wrap_rem>=32'd411775 ? wrap_rem-32'd411775:wrap_rem;state<=35;end
    35:begin wrap_signed<=wrap_sign?-$signed(wrap_rem):$signed(wrap_rem);state<=36;end
    36:begin peb<=wrap_signed>PI?wrap_signed-TAU:wrap_signed< -PI?wrap_signed+TAU:wrap_signed;state<=20;end
    20:begin integral_step<=$signed(peq)*64'sd1503239;state<=26;end
    26:begin
      feedback<=column==0?mul_value:feedback+mul_value;
      if(column==3)state<=21;else begin column<=column+1'b1;state<=20;end
    end
    21:begin
      integral_next<=integral-integral_step;tracking_bad<=absq(peq)>32768||absq(ew)>tracking_speed;
      raw<=ff-feedback-integral_out;state<=22;
    end
    22:begin
      if(absq(peq)<16384&&absq(peb)<9830&&absq(ew)<131072&&absq(ev)<196608&&(absq(raw)<34079||raw[31]!=peq[31]))
        integral<=clip64(integral_next,-64'sd33775622815744,64'sd33775622815744);
      bad_ticks<=tracking_bad?bad_ticks+1'b1:0;
      if(tracking_bad&&bad_ticks>=99)fault<=1;state<=23;
    end
    23:begin
      command<=fault?0:clip(ff-feedback-integral_out,-65536,65536);
      finished<=(finite_mode===1'b1) ? finite_complete:ending&&abs64(speed)<64'd21474836&&abs64(acceleration)<64'd42949673;
      done<=1;busy<=0;state<=0;
    end
   endcase
   // Defensive fault for a violated integration contract; no frame ACK.
   // The default standalone interface retains its original busy-rebase stall.
   if(OWNER_IDLE_REBASE!=0&&rebase&&busy)begin fault<=1;command<=0;rebase_ack<=0;end
  end
 end
endmodule
