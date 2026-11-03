// Frozen v27 drop_0, exact integer half-steps precomputed at 2 kHz.
// One wide ROM port avoids the verified dual-read synthesis discrepancy.
module controlled_drop(input wire clk,rst,start,enable,input wire reverse,
 input wire signed [31:0] q,a,w,v,
 output reg done,fault,output reg [7:0] reason,
 output reg signed [31:0] command,reference_a,output wire slow_observer,
 output reg home_active,output wire home_settled,output wire signed [63:0] home_integral_state,output wire [10:0] home_settled_state,
 input wire begin_drop,input wire signed [31:0] inherited_command,output wire signed [31:0] current_cap);
 import fixed_math::*;
 reg signed [31:0] wrap_input,wrap_signed,settled_phase;
 reg [31:0] wrap_mag,wrap_multiple,wrap_rem;
 reg [45:0] wrap_product /* synthesis syn_dspstyle = "dsp" */;
 reg wrap_sign;reg [1:0] wrap_target;
 wire signed [31:0] wrapped=wrap_signed>PI?wrap_signed-TAU:wrap_signed< -PI?wrap_signed+TAU:wrap_signed;
 reg [383:0] rom[0:2208],left_row;
 reg [4:0] state;reg [13:0] tick;reg [11:0] index;
 reg signed [31:0] node[0:11];integer i;
 reg signed [31:0] xq,xa,xw,xv,eq,ea,ew,ev,peq,pea,p0,p1,p2,p3,ff,raw,integral,cap;
 reg [7:0] tracking;
 reg signed [31:0] limited,command_delta,command_step,command_next;
 reg [10:0] settled_ticks;
 reg [15:0] home_ticks; // Separate 20 s terminal Home lease at 2 kHz.
 reg home_start,home_initialized;wire home_done,home_rest;
 wire signed [31:0] home_command;
 home_controller home(clk,rst,enable&&home_active,home_start,!home_initialized,1'b1,
   xq,xa,xw,xv,32'sd0,integral,settled_ticks,,home_done,home_command,,,,home_rest,1'b1,,1'b0,32'sd0,,home_integral_state,1'b0,64'sd0,home_settled_state);
 assign home_settled=enable&&home_active&&home_rest&&!fault;
 assign current_cap=!enable ? 0:cap;
 assign slow_observer=enable&&tick>=1208;
 initial $readmemh("rom/drop_2khz_q16.hex",rom);
 function automatic signed [31:0] directed(input signed [31:0] x);
  directed=reverse ? -x:x;
 endfunction
 always @(posedge clk) begin
  if(rst)begin cap<=0;wrap_input<=0;wrap_signed<=0;settled_phase<=0;wrap_mag<=0;wrap_multiple<=0;wrap_rem<=0;wrap_product<=0;wrap_sign<=0;wrap_target<=0;state<=0;tick<=0;home_ticks<=0;done<=0;fault<=0;reason<=0;command<=0;reference_a<=0;integral<=0;tracking<=0;index<=0;home_active<=0;settled_ticks<=0;home_start<=0;home_initialized<=0;end
  else begin
   done<=0;home_start<=0;
   if(!enable)begin state<=0;tick<=0;home_ticks<=0;command<=0;integral<=0;tracking<=0;home_active<=0;settled_ticks<=0;home_initialized<=0;end
   else case(state)
    0:if(start&&!fault)begin
      xq<=q;xa<=a;xw<=w;xv<=v;index<=tick>=2208?2208:tick[11:0];
      cap<=tick<1205?104858:65536;
      if(tick<10000)tick<=tick+1'b1; // Saturate: terminal settling must not wrap the trajectory clock.
      if(home_active&&home_ticks<40000)home_ticks<=home_ticks+1'b1;
      if(absq(q)>183501||absq(w)>917504||absq(v)>2293760||(!home_active&&tick>=10000)||(home_active&&home_ticks>=40000))begin fault<=1;reason<=8'h21;command<=0;done<=1;end
      // v27 transfers AFTER the sample at duration + 0.3 s (tick 1805).
      // The drop goal is exactly zero; reset(goal,goal), moveSeconds=0:
      // reference position/velocity/acceleration are all zero. Integral and
      // settled dwell are inherited, not reset at the controller boundary.
      else if(home_active)begin home_start<=1;state<=9;end
      else state<=1;
    end
    1:begin left_row<=rom[index];state<=2;end
    2:begin
      for(i=0;i<12;i=i+1)node[i]<=$signed(left_row[i*32+:32]);
      state<=3;
    end
    3:begin eq<=xq-directed(node[0]);wrap_input<=xa-directed(node[1]);wrap_target<=0;ew<=xw-directed(node[2]);ev<=xv-directed(node[3]);ff<=directed(node[4]);reference_a<=directed(node[10]);state<=16;end
    4:begin peq<=eq+mul(98,ew);pea<=ea+mul(98,ev);state<=5;end
    5:begin p0<=mul(node[5],peq);p1<=mul(node[6],pea);p2<=mul(node[7],ew);p3<=mul(node[8],ev);state<=6;end
    6:begin raw<=ff-p0-p1-p2-p3-integral;wrap_input<=xa-reference_a;wrap_target<=1;
      if(!home_active&&tracking>=100)begin fault<=1;reason<=8'h24;end
      state<=16;
    end
    7:begin
      if(absq(peq)<16384&&absq(pea)<7864&&absq(ew)<131072&&(absq(raw)<mul(58982,cap)||(raw[31]==peq[31])))begin
        integral<=clip(integral+mul(98,peq),-19661,19661);
      end
      state<=8;
    end
    8:begin
      limited<=clip(raw,-cap,cap);state<=10;
    end
    10:begin command_delta<=limited-command;state<=11;end
    11:begin command_step<=clip(command_delta,-3277,3277);state<=12;end
    12:begin command_next<=command+command_step;wrap_input<=xa;wrap_target<=2;state<=16;end
    13:begin
      command<=fault?0:clip(command_next,-cap,cap);
      if(!home_active)begin
        if(tick>=1206&&absq(xq)<=571&&absq(settled_phase)<=800&&absq(xw)<=9830&&absq(xv)<=13107)
          settled_ticks<=settled_ticks<1000?settled_ticks+1'b1:settled_ticks;
        else settled_ticks<=0;
        if(tick>=1806)home_active<=1;
      end
      done<=1;state<=0;
    end
    9:if(home_done)begin
      raw<=home_command;reference_a<=0;tracking<=0;home_initialized<=1;state<=8;
    end
    16:begin wrap_sign<=wrap_input[31];wrap_mag<=wrap_input[31]?-$unsigned(wrap_input):$unsigned(wrap_input);state<=17;end
    17:begin wrap_product<=wrap_mag*14'd10430;state<=18;end
    18:begin wrap_multiple<=wrap_product[45:32]*19'd411775;state<=19;end
    19:begin wrap_rem<=wrap_mag-wrap_multiple;state<=20;end
    20:begin wrap_rem<=wrap_rem>=32'd411775 ? wrap_rem-32'd411775:wrap_rem;state<=21;end
    21:begin wrap_signed<=wrap_sign?-$signed(wrap_rem):$signed(wrap_rem);state<=22;end
    22:begin
     case(wrap_target)
      0:begin ea<=wrapped;state<=4;end
      1:begin tracking<=!home_active&&absq(wrapped)>22938 ? tracking+1'b1:0;state<=7;end
      2:begin settled_phase<=wrapped;state<=13;end
     endcase
    end
   endcase
   // ESP32 changes trajectory/mode at DROP while retaining iqRef; the next
   // current tick slews from this command, not from an implicit zero.
   if((begin_drop===1'b1)&&enable)command<=inherited_command;
  end
 end
endmodule
