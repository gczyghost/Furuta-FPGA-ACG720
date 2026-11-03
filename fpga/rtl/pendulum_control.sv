// Modes: 1 bounded current, 2 down-position hold, 3 frozen swing + quiet
// hold, 4 upright hold only. No automatic retry or swing after capture loss.
module pendulum_control(input wire clk,rst,start,enable,input wire [2:0] mode,
 input wire signed [31:0] q,alpha,w,v,iq,iq_set,
 output reg busy,done,fault,output reg [7:0] reason,
 output reg signed [31:0] command,alpha_ref,current_cap,input wire long_run,
 input wire resume_hold,input wire signed [31:0] resume_integral,resume_command,
 output wire signed [31:0] hold_integral,input wire voltage_saturated,
 input wire begin_session,input wire signed [31:0] session_command,input wire continuous_session);
 import fixed_math::*;
 reg signed [31:0] wrap_input,wrap_signed;
 reg [31:0] wrap_mag,wrap_multiple,wrap_rem;
 reg [45:0] wrap_product /* synthesis syn_dspstyle = "dsp" */;
 reg wrap_sign;reg [1:0] wrap_target;
 wire signed [31:0] wrapped=wrap_signed>PI?wrap_signed-TAU:wrap_signed< -PI?wrap_signed+TAU:wrap_signed;
 reg [383:0] rom[0:1000];reg signed [31:0] voltage_caps[0:1023];
 reg [383:0] row;
 reg [31:0] tick; reg [5:0] state;
 reg [10:0] row_index;reg [9:0] voltage_index;
 reg signed [31:0] xq,xa,xw,xv,xi,beta,raw,correction,cap,voltage_cap;
 // Retain16 fractional bits below the existing Q16 integral output.
 // Same gain,1ms update gate and +/-7864Q16 clamp; no output-cycle change.
 reg signed [31:0] integral_accumulator;
 wire signed [31:0] integral=integral_accumulator>>>16;
 reg signed [31:0] recovery,blend,quiet,recovery_u,damped,priority_factor,severity,desired;
 assign hold_integral=integral;
 reg signed [31:0] aq,ab,aw,av,sev0,sev1,sev2,sev3,pri0,pri1;
 reg integral_allowed;reg signed [31:0] integral_product;
 reg hold_active;reg [15:0] tracking_ticks;
 reg signed [31:0] e0,e1,e2,e3,e4,p0,p1,p2,p3,p4,sum01,sum23,ff0,feedforward;
 reg signed [31:0] h0,h1,h2,h3,h4,s0,s1,s2,s3,d0,d1,d2,d3,d4,di0,di1,hold_u,mix_delta;
 reg signed [63:0] h0_product,h1_product,h2_product,h3_product,h4_product,s0_product,s1_product,s2_product,s3_product,d0_product,d1_product,d2_product,d3_product,d4_product,di0_product;
 reg signed [31:0] command_delta,command_step,command_next,slew_step;
 reg signed [63:0] time_a,time_b,time_c,time_d,time_ab,time_cd;
 wire signed [63:0] time_x=di0;
 reg saturated_latched;
 reg signed [31:0] severity_left,severity_right,mix_product;
 wire signed [31:0] r0=row[31:0],r1=row[63:32],r2=row[95:64],r3=row[127:96],r4=row[159:128];
 wire signed [31:0] k0=row[191:160],k1=row[223:192],k2=row[255:224],k3=row[287:256],k4=row[319:288];
 wire signed [31:0] derivative=row[351:320],reference_alpha=row[383:352];
 function automatic signed [31:0] maxq(input signed [31:0] a,b); maxq=a>b ? a:b;endfunction
 initial begin $readmemh("rom/boost_q16.hex",rom);$readmemh("rom/voltage_cap_q16.hex",voltage_caps);end

 // Exact constant product: 4294967 = 2^22+2^17-2^15+2^11+2^8+2^6-2^3-1.
 // Balanced signed64 sums avoid a new multiplier without changing Q32 precision.
 // All intermediates fit signed64 for the complete signed32 input domain.
 function automatic signed [63:0] integral_time_product(input signed [31:0] value);
  reg signed [63:0] x,a,b,c,d;
  begin
   x=value;
   a=(x<<<22)+(x<<<17);b=(x<<<11)-(x<<<15);
   c=(x<<<8)+(x<<<6);d=-(x<<<3)-x;
   integral_time_product=(a+b)+(c+d);
  end
 endfunction
 always @(posedge clk) begin
  if(rst) begin time_a<=0;time_b<=0;time_c<=0;time_d<=0;time_ab<=0;time_cd<=0;wrap_input<=0;wrap_signed<=0;wrap_mag<=0;wrap_multiple<=0;wrap_rem<=0;wrap_product<=0;wrap_sign<=0;wrap_target<=0;busy<=0;done<=0;fault<=0;reason<=0;command<=0;alpha_ref<=0;current_cap<=0;
    severity_left<=0;severity_right<=0;mix_product<=0;saturated_latched<=0;slew_step<=3277;
    aq<=0;ab<=0;aw<=0;av<=0;sev0<=0;sev1<=0;sev2<=0;sev3<=0;pri0<=0;pri1<=0;
    tick<=0;state<=0;row_index<=0;voltage_index<=0;row<=0;
    xq<=0;xa<=0;xw<=0;xv<=0;xi<=0;beta<=0;raw<=0;correction<=0;cap<=0;voltage_cap<=0;
    integral_allowed<=0;integral_product<=0;
    integral_accumulator<=0;recovery<=0;blend<=0;quiet<=0;recovery_u<=0;damped<=0;priority_factor<=0;severity<=0;desired<=0;hold_active<=0;tracking_ticks<=0;
    e0<=0;e1<=0;e2<=0;e3<=0;e4<=0;p0<=0;p1<=0;p2<=0;p3<=0;p4<=0;sum01<=0;sum23<=0;ff0<=0;feedforward<=0;
    h0<=0;h1<=0;h2<=0;h3<=0;h4<=0;s0<=0;s1<=0;s2<=0;s3<=0;d0<=0;d1<=0;d2<=0;d3<=0;d4<=0;di0<=0;di1<=0;hold_u<=0;mix_delta<=0;end
  else begin
    done<=0;
    if(!enable) begin
      tick<=0;state<=0;busy<=0;command<=0;integral_accumulator<=0;recovery<=0;blend<=0;hold_active<=0;tracking_ticks<=0;
    end else case(state)
      0: if(start&&!fault) begin
        // Preserve the 1kHz parity and post-capture phase indefinitely without
        // wrapping back into Swing. Finite diagnostic modes retain deadlines.
        busy<=1;tick<=continuous_session===1'b1&&(mode==3||mode==4)&&tick>=6001?6000:tick+1; xq<=q;xa<=alpha;xw<=w;xv<=v;xi<=iq;beta<=alpha-PI;saturated_latched<=voltage_saturated===1'b1;
        row_index<=tick<1000 ? 0:tick>=3000 ? 1000:(tick-1000)>>1;
        voltage_index<=absq(w)>=4190208 ? 1023:(absq(w)+4095)>>12;
        if(abs_gt(q,183501)||abs_gt(w,917504)||abs_gt(v,2293760)||(!(continuous_session===1'b1&&(mode==3||mode==4))&&tick>=((mode==3||mode==4)?(long_run?1220000:260000):40000))) begin
          fault<=1;reason<=8'h21;command<=0;busy<=0;done<=1;
        end else if(mode==4 && !upright_le(alpha,39322)) begin
          fault<=1;reason<=8'h22;command<=0;busy<=0;done<=1;
        end else state<=1;
      end
      1:begin wrap_input<=beta;wrap_target<=0;state<=32;end
      24: begin aq<=absq(xq);ab<=absq(beta);aw<=absq(xw);av<=absq(xv);row<=rom[row_index];voltage_cap<=voltage_caps[voltage_index];state<=2;end
      2: begin
        sev0<=mul(aq,3754936);sev1<=mul(ab,3754936);sev2<=mul(aw,163840);sev3<=mul(av,163840);pri0<=mul(ab,1872457);pri1<=mul(av,81920);
        if(mode==1) begin raw<=clip(iq_set,-5243,5243);cap<=5243;alpha_ref<=xa;state<=12;end
        else if(mode==2 || (mode==3&&tick<=1000)) begin
          raw<=-mul(131072,xq)-mul(7864,xw);cap<=7864;alpha_ref<=0;state<=12;
        end else if(mode==3||mode==4) begin
          if(mode==3) begin
            e0<=xq-r0;wrap_input<=xa-r1;wrap_target<=1;e2<=xw-r2;e3<=xv-r3;e4<=mul(21627,xi)-r4;
            ff0<=mul(209,derivative);
            alpha_ref<=reference_alpha; cap<=tick<2600 ? 104858:65536;
          end else begin correction<=0;alpha_ref<=PI;cap<=65536;end
          state<=mode==3?32:14;
        end else begin fault<=1;reason<=8'h23;command<=0;busy<=0;done<=1;state<=0;end
      end
      14: begin
        severity_left<=maxq(sev0,sev1);severity_right<=maxq(sev2,sev3);
        if(mode==3) begin p0<=mul(k0,e0);p1<=mul(k1,e1);p2<=mul(k2,e2);p3<=mul(k3,e3);p4<=mul(k4,e4);end
        else begin p0<=mul(196608,xq);p1<=-mul(786432,beta);p2<=mul(58982,xw);p3<=-mul(62259,xv);p4<=-mul(9830,xi);end
        feedforward<=add(r4,ff0);state<=15;
      end
      15: begin sum01<=p0+p1;sum23<=p2+p3;state<=16;end
      16: begin correction<=mode==3 ? -sum01-sum23-p4:sum01+sum23+p4;state<=3;end
      3: begin
        if(mode==3) raw<=mul(198594,add(feedforward,correction));
        else raw<=correction;
        h0_product<=32'sd196608*xq;h1_product<=32'sd786432*beta;h2_product<=32'sd58982*xw;h3_product<=32'sd62259*xv;h4_product<=32'sd9830*xi;
        s0_product<=32'sd229376*xq;s1_product<=32'sd917504*beta;s2_product<=32'sd68813*xw;s3_product<=32'sd72090*xv;
        d0_product<=32'sd131072*xq;d1_product<=32'sd917504*beta;d2_product<=32'sd52429*xw;d3_product<=32'sd78643*xv;d4_product<=32'sd19661*xi;
        di0_product<=32'sd22938*xq;
        if((mode==4 || tick>=2700) && absq(xq)<19661&&absq(beta)<9830&&absq(xw)<131072&&absq(xv)<196608) hold_active<=1;
        severity<=maxq(severity_left,severity_right);
        if(mode==3&&tick>5600&&absq(beta)>39322) begin fault<=1;reason<=8'h22;end
        wrap_input<=xa-alpha_ref;wrap_target<=2;
        if(tracking_ticks>=100) begin fault<=1;reason<=8'h24;end
        state<=32;
      end
      26:begin h0<=sat64(h0_product>>>16);h1<=-sat64(h1_product>>>16);h2<=sat64(h2_product>>>16);h3<=-sat64(h3_product>>>16);h4<=-sat64(h4_product>>>16);s0<=sat64(s0_product>>>16);s1<=-sat64(s1_product>>>16);s2<=sat64(s2_product>>>16);s3<=-sat64(s3_product>>>16);d0<=sat64(d0_product>>>16);d1<=-sat64(d1_product>>>16);d2<=sat64(d2_product>>>16);d3<=-sat64(d3_product>>>16);d4<=-sat64(d4_product>>>16);di0<=sat64(di0_product>>>16);state<=4;end
      4: begin
        desired<=clip((severity-65536)/2,0,65536);
        quiet<=(h0+h1)+(h2+h3)+h4;
        recovery_u<=(s0+s1)+(s2+s3);
        damped<=(d0+d1)+(d2+d3)+d4;
        // Reuse existing states4/5/6; no extra cycles or altered sample/update gate.
        time_a<=(time_x<<<22)+(time_x<<<17);time_b<=(time_x<<<11)-(time_x<<<15);
        time_c<=(time_x<<<8)+(time_x<<<6);time_d<=-(time_x<<<3)-time_x;
        priority_factor<=clip(maxq(pri0,pri1)-65536,0,65536);state<=5;
      end
      5: begin
        time_ab<=time_a+time_b;time_cd<=time_c+time_d;
        // Outer loop runs every second current tick, exactly 1 ms.
        if(hold_active && !tick[0]) begin
          recovery<=clip(recovery+(desired>recovery ? ratio32(desired-recovery,286331153):ratio32(desired-recovery,14316558)),0,65536);
          blend<=clip(blend+164,0,65536);
        end
        recovery_u<=recovery_u+mul(priority_factor,damped-recovery_u);state<=6;
      end
      6: begin
        di1<=sat64((time_ab+time_cd)>>>16);
        hold_u<=quiet+mul(recovery,recovery_u-quiet)+integral;
        if(hold_active)cap<=65536+mul(39322,recovery);
        state<=17;
      end
      17: begin mix_delta<=hold_u-raw;state<=18;end
      18: begin mix_product<=mul(blend,mix_delta);state<=25;end
      25: begin if(hold_active)raw<=add(raw,mix_product);state<=7;end
      7: begin
        if(hold_active&&!tick[0]&&absq(beta)<9830&&absq(xq)<16384&&absq(xw)<131072&&absq(xv)<196608&&
           ((!saturated_latched&&absq(hold_u)<cap-5243) || (hold_u[31]!=xq[31]))) integral_allowed<=1;else integral_allowed<=0;
        integral_product<=mul(di1,blend);state<=22;
      end
      22:begin if(integral_allowed)integral_accumulator<=clip(add(integral_accumulator,integral_product),-32'sd515375104,32'sd515375104);state<=12;end
      12: begin
        current_cap<=cap<voltage_cap ? cap:voltage_cap;
        state<=23;
      end
      23:begin raw<=clip(raw,-current_cap,current_cap);state<=13;end
      13: begin
        command_delta<=raw-command;
        // v27 profile4: 100 + 60*recovery A/s, sampled at 2 kHz.
        slew_step<=hold_active?3277+mul(1966,recovery):3277;state<=19;
      end
      19:begin command_step<=clip(command_delta,-slew_step,slew_step);state<=20;end
      20:begin command_next<=command+command_step;state<=21;end
      21:begin
        command<=fault ? 0:clip(command_next,-current_cap,current_cap);
        busy<=0;done<=1;state<=0;
      end
      32:begin wrap_sign<=wrap_input[31];wrap_mag<=wrap_input[31]?-$unsigned(wrap_input):$unsigned(wrap_input);state<=33;end
      33:begin wrap_product<=wrap_mag*14'd10430;state<=34;end
      34:begin wrap_multiple<=wrap_product[45:32]*19'd411775;state<=35;end
      35:begin wrap_rem<=wrap_mag-wrap_multiple;state<=36;end
      36:begin wrap_rem<=wrap_rem>=32'd411775 ? wrap_rem-32'd411775:wrap_rem;state<=37;end
      37:begin wrap_signed<=wrap_sign?-$signed(wrap_rem):$signed(wrap_rem);state<=38;end
      38:begin
       case(wrap_target)
        0:begin beta<=wrapped;state<=24;end
        1:begin e1<=wrapped;state<=14;end
        2:begin tracking_ticks<=absq(wrapped)>78643 ? tracking_ticks+1'b1:0;state<=26;end
       endcase
      end
    endcase
    if(resume_hold&&enable)begin integral_accumulator<=clip(resume_integral,-7864,7864)<<<16;command<=resume_command;end
    // Explicit sessionTransition semantics, not a reset of PI or dose state.
    if((begin_session===1'b1)&&enable)command<=clip(session_command,-7864,7864);
  end
 end
endmodule
