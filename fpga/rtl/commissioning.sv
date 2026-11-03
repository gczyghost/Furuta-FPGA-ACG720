// R1 commissioning at 2 kHz. No PI feedback until current signs/angle are known.
// Results are candidates only; the existing stopped write/commit protocol applies them.
module commissioning(input wire clk,rst,enable,input wire [2:0] mode,
 input wire sample,input wire [13:0] raw,input wire [11:0] adc_a,adc_b,
 input wire signed [31:0] offset_a,offset_b,gain_a,gain_b,
 input wire [5:0] poles,
 output reg finished,failed,output reg [7:0] error,
 output wire [31:0] status,output reg signed [31:0] result_a,result_b,
 output reg [15:0] result_zero,output reg result_reverse,
 output reg update,output reg [15:0] du,dv,dw,
 input wire [31:0] sample_stamp);
 import fixed_math::*;
 reg signed [31:0] sample_a,sample_b,absolute_gain_a,absolute_gain_b;
 reg active,valid;reg [2:0] kind;reg [15:0] count;
 reg [4:0] state;reg [13:0] previous;reg signed [31:0] position,origin,delta;
 reg signed [31:0] ca,cb,pa[0:2],pb[0:2],endpos[0:2],sumpos,window_min,window_max;
 reg signed [31:0] apa[0:2],apb[0:2],pa0x5,pb0x7,pb1x5,pa1x7;
 reg [31:0] phase_acc;reg [15:0] phase;reg trig_start;
 wire trig_done;wire signed [31:0] sn,cs;
 reg signed [31:0] av,bv,va,vb,vc,amplitude,forward,back,travel_f,travel_b;
 wire signed [31:0] lo=va<vb ? (va<vc?va:vc):(vb<vc?vb:vc);
 integer j;
 assign status={error,count,3'b0,kind,valid,active};
 trig_lut trig(clk,rst,trig_start,phase,trig_done,sn,cs);
 function automatic [15:0] duty(input signed [31:0] volts);
   reg signed [31:0] d;begin d=ratio32(clip(volts,0,131072),178956971);duty=d[15:0];end
 endfunction
 task automatic reject(input [7:0] why);
 begin active<=0;valid<=0;finished<=1;failed<=1;error<=why;du<=0;dv<=0;dw<=0;update<=1;state<=0;end
 endtask
 always @(posedge clk) begin
  if(rst) begin
   sample_a<=0;sample_b<=0;absolute_gain_a<=0;absolute_gain_b<=0;
   active<=0;valid<=0;kind<=0;count<=0;state<=0;finished<=0;failed<=0;error<=0;
   result_a<=0;result_b<=0;result_zero<=0;result_reverse<=0;update<=0;du<=0;dv<=0;dw<=0;
   previous<=0;position<=0;origin<=0;delta<=0;ca<=0;cb<=0;sumpos<=0;window_min<=0;window_max<=0;
   phase_acc<=0;phase<=0;trig_start<=0;av<=0;bv<=0;va<=0;vb<=0;vc<=0;amplitude<=0;
   forward<=0;back<=0;travel_f<=0;travel_b<=0;
   for(j=0;j<3;j=j+1)begin pa[j]<=0;pb[j]<=0;endpos[j]<=0;end
  end else begin
   finished<=0;update<=0;trig_start<=0;
   if(!enable) begin
    if(active)begin valid<=0;error<=8'h46;end
    active<=0;state<=0;du<=0;dv<=0;dw<=0;
   end else if(!active && !finished && !valid && !failed && state==0) begin
    active<=1;kind<=mode;count<=0;previous<=raw;position<=$signed({1'b0,raw});origin<=$signed({1'b0,raw});
    phase_acc<=0;sumpos<=0;error<=0;
    for(j=0;j<3;j=j+1)begin pa[j]<=0;pb[j]<=0;endpos[j]<=0;end
   end else if(active) begin
    case(state)
     0: if(sample)begin
      // While invalid, candidate registers retain the last raw sample for fault diagnosis.
      result_b<=sample_stamp;result_a<={4'b0,adc_a,4'b0,adc_b};
      delta<=$signed({1'b0,raw})-$signed({1'b0,previous});previous<=raw;
      sample_a<=($signed({1'b0,adc_a})<<<16)-offset_a;sample_b<=($signed({1'b0,adc_b})<<<16)-offset_b;
      absolute_gain_a<=absq(gain_a);absolute_gain_b<=absq(gain_b);state<=1;
      if(adc_a<16||adc_a>4079||adc_b<16||adc_b>4079)reject(8'h40);
     end
     1: begin
      ca<=mul(sample_a,absolute_gain_a);cb<=mul(sample_b,absolute_gain_b);
      if(delta>8192)delta<=delta-16384;else if(delta< -8192)delta<=delta+16384;
      state<=2;
     end
     2: begin
      position<=position+delta;state<=3;
      // Polarity-independent protection, including worst-case reconstructed third phase.
      if(absq(ca)>22938||absq(cb)>22938||absq(ca)+absq(cb)>32768)reject(8'h41);
      else if(absq(delta)>32)reject(8'h42);
     end
     3: begin
      // Match ESP32 D's 1.2 rad travel envelope (16384 counts/turn).
      // Free-rotor mapping needs motion; per-sample jump/current guards remain active.
      if(absq(position-origin)>(kind==5?3130:4096))reject(8'h42);
      else if(kind==5)begin
       // 50 ms pulses, 350 ms off; use 5..30 ms plateau (50 samples).
       if(count<800)j=0;else if(count<1600)j=1;else j=2;
       if((count-j*800)>=10 && (count-j*800)<60)begin pa[j]<=pa[j]+ca;pb[j]<=pb[j]+cb;end
       du<=0;dv<=0;dw<=0;
       if((count-j*800)<100)begin
        if(j==0)du<=3277;else if(j==1)dv<=3277;else dw<=3277; // 1.2 V / 24 V
       end
       update<=1;
       if(count==2399)state<=10;else begin count<=count+1'b1;state<=0;end
      end else if(kind==6 && poles==11)begin
       // Original v27 conditioning -1 turn/3 s, +2 turns/4 s, -2 turns/4 s.
       if(count<2000)phase_acc<=0;
       else if(count<8000)phase_acc<=phase_acc-32'd715828;
       else if(count<10000)phase_acc<=0;
       else if(count<18000)phase_acc<=phase_acc+32'd1073742;
       else if(count<20000)phase_acc<=0;
       else if(count<28000)phase_acc<=phase_acc-32'd1073742;
       else phase_acc<=0;
       phase<=phase_acc[31:16];amplitude<=count<400 ? count*188:75366; // ramp to 1.15 V
       if(count==9488||count==19488||count==29488)begin sumpos<=position;window_min<=position;window_max<=position;end
       else if((count>9488&&count<10000)||(count>19488&&count<20000)||(count>29488&&count<30000))begin
        sumpos<=sumpos+position;
        if(position<window_min)window_min<=position;if(position>window_max)window_max<=position;
       end
       if(count==9999||count==19999||count==29999)begin
        if(count==9999)endpos[0]<=(sumpos+position)>>>9;
        else if(count==19999)endpos[1]<=(sumpos+position)>>>9;
        else endpos[2]<=(sumpos+position)>>>9;
       end
       if((count==9999||count==19999||count==29999) &&
          (window_max-window_min>16 || position-window_min>16 || window_max-position>16))reject(8'h43);
       else begin trig_start<=1;state<=4;end
      end else reject(8'h44);
     end
     4: if(trig_done)begin av<=mul(amplitude,cs);bv<=mul(amplitude,sn);state<=5;end
     5: begin va<=av;vb<=-mul(32768,av)+mul(56756,bv);vc<=-mul(32768,av)-mul(56756,bv);state<=6;end
     6: begin du<=duty(va-lo);dv<=duty(vb-lo);dw<=duty(vc-lo);update<=1;
       if(count==29999)state<=11;else begin count<=count+1'b1;state<=0;end
     end
     10: begin
      du<=0;dv<=0;dw<=0;update<=1;
      for(j=0;j<3;j=j+1)begin apa[j]<=absq(pa[j]);apb[j]<=absq(pb[j]);end
      state<=14;
     end
     14:begin pa0x5<=apa[0]*5;pb0x7<=apb[0]*7;pb1x5<=apb[1]*5;pa1x7<=apa[1]*7;state<=15;end
     15: begin
      du<=0;dv<=0;dw<=0;update<=1;
      if(apa[0]<114700||apb[1]<114700 ||
         pa0x5<pb0x7 || pb1x5<pa1x7 ||
         pa[0][31]==pa[1][31] || pa[0][31]==pa[2][31] ||
         pb[1][31]==pb[0][31] || pb[1][31]==pb[2][31] ||
         apa[1]<32768||apa[2]<32768||apb[0]<32768||apb[2]<32768)reject(8'h45);
      else begin result_a<=pa[0][31] ? -absq(gain_a):absq(gain_a);result_b<=pb[1][31] ? -absq(gain_b):absq(gain_b);
       valid<=1;active<=0;finished<=1;state<=0;end
     end
     11: begin forward<=endpos[1]-endpos[0];back<=endpos[1]-endpos[2];state<=12;end
     12: begin travel_f<=absq(forward)*11;travel_b<=absq(back)*11;state<=13;end
     13: begin
      du<=0;dv<=0;dw<=0;update<=1;
      if(forward[31]!=back[31]||absq(travel_f-32768)>1000||absq(travel_b-32768)>1000||
         absq(travel_f-travel_b)>1400||absq(endpos[0]-endpos[2])>91)reject(8'h44);
      else begin result_reverse<=forward[31];result_zero<=forward[31] ? -endpos[2]*44:endpos[2]*44;
       valid<=1;active<=0;finished<=1;state<=0;end
     end
    endcase
   end
  end
 end
endmodule
