// Original v27 down QSTEP / upright UPSTEP table feedback. Single physical
// ROM read address, left/right rows captured on separate clocks. The owner
// qualifies entry, bounds the full path and transfers goal/integral on finish.
module planned_motion(input wire clk,rst,enable,start,begin_motion,
 input wire [3:0] requested_bank,input wire signed [31:0] angle,origin,seed_integral,q,a,w,v,
 output reg busy,done,fault,finished,output reg signed [31:0] command,goal,reference_q,reference_a,
 output wire signed [31:0] integral_out,output wire reference_ended,output reg [10:0] settled_ticks) /* synthesis syn_dspstyle = "logic" */;
 // Registered exact normalization of the predicted angular error.
 reg [31:0] wrap_mag,wrap_multiple,wrap_rem;
 reg [45:0] wrap_product /* synthesis syn_dspstyle = "dsp" */;
 reg wrap_sign;reg signed [31:0] wrap_signed;
 reg signed [31:0] wrap_input,tracking_error,settled_phase;reg [1:0] wrap_target;
 wire signed [31:0] wrapped=wrap_signed>fixed_math::PI?wrap_signed-fixed_math::TAU:wrap_signed< -fixed_math::PI?wrap_signed+fixed_math::TAU:wrap_signed;
 import fixed_math::*;import motion_metadata::*;
 reg [139:0] rom[0:MOTION_ROWS-1],read_row,left_row,right_row;
 reg [13:0] read_address,right_address;reg [5:0] state;reg [3:0] bank;reg [2:0] column;
 reg [31:0] tick,elapsed,index,period,base,count,duration;
 reg [63:0] div10_product;reg [3:0] tenth;
 reg signed [31:0] mul_a,mul_b;
 reg signed [63:0] mul_product /* synthesis syn_dspstyle = "dsp" */;
 wire signed [31:0] mul_value=sat64(mul_product>>>16);
 reg signed [31:0] scale,frac,diff,node[0:6],xq,xa,xw,xv,from,request_angle;
 reg signed [31:0] rq,ra,rw,rv,ff,eq,ea,ew,ev,peq,pea,feedback,raw,cap;
 reg signed [63:0] integral;
 reg signed [31:0] seed_latched;
 reg signed [31:0] fp0,fp1,fp2,fp3;
 reg [7:0] tracking;
 reg div_start;reg [63:0] numerator;reg [31:0] denominator;
 wire div_done;wire [63:0] quotient;
 wire upright=bank>=5;
 wire signed [63:0] increment=$signed(peq)*(upright?-64'sd1503239:64'sd6442451);
 reg signed [63:0] integral_step,next_integral;
 wire signed [63:0] limit_integral=upright?64'sd33775622815744:64'sd84443352006656;
 wire signed [63:0] clipped_integral=next_integral>limit_integral?limit_integral:next_integral< -limit_integral?-limit_integral:next_integral;
 assign integral_out=integral>>>32;
 assign reference_ended=elapsed>=duration;
 function automatic signed [31:0] unpack_node(input [139:0] row,input [2:0] c);
   unpack_node=$signed(row[c*20+:20]);
 endfunction
 function automatic [31:0] tenth_fraction(input [3:0] t);
  case(t)
   0:tenth_fraction=0;1:tenth_fraction=6553;2:tenth_fraction=13107;3:tenth_fraction=19660;4:tenth_fraction=26214;
   5:tenth_fraction=32768;6:tenth_fraction=39321;7:tenth_fraction=45875;8:tenth_fraction=52428;9:tenth_fraction=58982;
   default:tenth_fraction=0;
  endcase
 endfunction
 initial $readmemh("rom/motion_q16_p20.hex",rom);
 always @(posedge clk) read_row<=rom[read_address];
 always @*begin
   mul_a=0;mul_b=0;
   if(state==8)begin mul_a=diff;mul_b=frac;end
   else if(state==9)begin mul_a=scale;mul_b=node[column];end
 end
 always @(posedge clk)mul_product<=mul_a*mul_b;
 udiv64 divider(clk,rst||!enable,div_start,numerator,denominator,,div_done,quotient);
 always @(posedge clk)begin
  if(rst||!enable)begin
   wrap_input<=0;tracking_error<=0;settled_phase<=0;wrap_target<=0;wrap_mag<=0;wrap_multiple<=0;wrap_rem<=0;wrap_product<=0;wrap_sign<=0;wrap_signed<=0;
   busy<=0;done<=0;fault<=0;finished<=0;command<=0;goal<=0;reference_q<=0;reference_a<=0;
   integral<=0;seed_latched<=0;tick<=0;tracking<=0;state<=0;div_start<=0;read_address<=0;settled_ticks<=0;
  end else begin
   done<=0;div_start<=0;
   case(state)
    0:if(start&&!fault)begin
      busy<=1;xq<=q;xa<=a;xw<=w;xv<=v;
      if(begin_motion)begin
        bank<=requested_bank;from<=origin;goal<=origin+angle;request_angle<=angle;
        base<=motion_base(requested_bank);count<=motion_count(requested_bank);
        period<=motion_step_ticks(requested_bank);duration<=motion_duration_ticks(requested_bank);
        seed_latched<=seed_integral;
        tick<=0;tracking<=0;finished<=0;settled_ticks<=0;
        if(requested_bank>8||angle==0||absq(angle)>motion_goal_q16(requested_bank)+2)begin fault<=1;command<=0;busy<=0;done<=1;end
        else begin numerator<=64'(absq(angle))<<16;denominator<=motion_goal_q16(requested_bank);div_start<=1;state<=28;end
      // v27 executes upright motion at 1 kHz inside the 2 kHz current loop.
      end else if(upright&&tick[0])begin
        tick<=tick+1'b1;finished<=tick>=duration+800;busy<=0;done<=1;
      end else state<=2;
    end
    // Split incoming sign inversion from saturation; divider runs in parallel.
    28:begin integral<=$signed(clip(seed_latched,bank>=5?-7864:-19661,bank>=5?7864:19661))*64'sd4294967296;state<=1;end
    1:if(div_done)begin scale<=request_angle[31]?-$signed(quotient[31:0]):$signed(quotient[31:0]);state<=2;end
    2:begin
      elapsed<=tick;tick<=tick+1;div10_product<=64'(tick)*64'hcccccccd;
      cap<=upright&&tick<duration?104858:65536;state<=24;
    end
    // Exact unsigned division by ten via reciprocal multiply, then a 10-entry
    // fraction table. Avoid large combinational / and % networks in synthesis.
    24:begin index<=period==2?elapsed>>1:div10_product>>35;tenth<=elapsed-(div10_product>>35)*10;state<=25;end
    25:begin frac<=period==2?(elapsed[0]?32768:0):tenth_fraction(tenth);state<=3;end
    3:begin
      read_address<=base+(index>=count?count-1:index);
      right_address<=base+(index+1>=count?count-1:index+1);state<=4;
    end
    4:begin read_address<=right_address;state<=5;end
    5:begin left_row<=read_row;state<=6;end
    6:begin right_row<=read_row;column<=0;state<=7;end
    7:begin diff<=unpack_node(right_row,column)-unpack_node(left_row,column);state<=8;end
    8:state<=26;
    26:begin node[column]<=unpack_node(left_row,column)+mul_value;
      if(column==6)begin column<=0;state<=9;end else begin column<=column+1'b1;state<=7;end
    end
    9:state<=27;
    27:begin
      case(column)
       0:rq<=from+mul_value;1:ra<=mul_value;2:rw<=mul_value;3:rv<=mul_value;4:ff<=mul_value;
       5:reference_q<=from+mul_value;6:reference_a<=mul_value;
      endcase
      if(column==6)state<=10;else begin column<=column+1'b1;state<=9;end
    end
    10:begin eq<=xq-rq;wrap_input<=xa-ra;wrap_target<=0;ew<=xw-rw;ev<=xv-rv;state<=30;end
    11:begin peq<=eq+mul(98,ew);pea<=ea+mul(98,ev);wrap_input<=ea+mul(98,ev);wrap_target<=1;state<=upright?30:12;end
    30:begin wrap_sign<=wrap_input[31];wrap_mag<=wrap_input[31]?-$unsigned(wrap_input):$unsigned(wrap_input);state<=31;end
    31:begin wrap_product<=wrap_mag*14'd10430;state<=32;end
    32:begin wrap_multiple<=wrap_product[45:32]*19'd411775;state<=33;end
    33:begin wrap_rem<=wrap_mag-wrap_multiple;state<=34;end
    34:begin wrap_rem<=wrap_rem>=32'd411775 ? wrap_rem-32'd411775:wrap_rem;state<=35;end
    35:begin wrap_signed<=wrap_sign?-$signed(wrap_rem):$signed(wrap_rem);state<=36;end
    36:case(wrap_target)
      0:begin ea<=wrapped;state<=11;end
      1:begin pea<=wrapped;state<=12;end
      2:begin tracking_error<=wrapped;state<=37;end
      3:begin settled_phase<=wrapped;state<=14;end
    endcase
    37:begin wrap_input<=xa;wrap_target<=3;state<=30;end
    12:begin
      integral_step<=increment;
      fp0<=upright?mul(124518,peq):mul(163840,peq);
      fp1<=upright?mul(655360,pea):mul(16384,pea);
      fp2<=upright?mul(45875,ew):mul(13107,ew);
      fp3<=mul(55706,ev);state<=38;
    end
    38:begin feedback<=upright?-fp0+fp1-fp2+fp3:fp0-fp1+fp2;state<=13;end
    13:begin next_integral<=integral+integral_step;raw<=ff-feedback-integral_out;wrap_input<=xa-reference_a;wrap_target<=2;state<=upright?14:30;end
    14:begin
      if(absq(peq)<16384&&absq(pea)<(upright?9830:7864)&&absq(ew)<131072&&(!upright||absq(ev)<196608)&&
         (absq(raw)<(upright?34079:58982)||(upright?(raw[31]!=peq[31]):(raw[31]==peq[31]))))integral<=clipped_integral;
      if(!upright)begin
        tracking<=absq(tracking_error)>22938 ? tracking+1'b1:0;
        if(tracking>=100)fault<=1;
      end
      state<=15;
    end
    15:begin
      command<=fault?0:clip(upright?ff-feedback-integral_out:raw,-cap,cap);
      finished<=elapsed>=duration+(upright?800:600);busy<=0;done<=1;state<=0;
      if(!upright&&elapsed>=duration&&absq(xq-goal)<=571&&absq(settled_phase)<=800&&absq(xw)<=9830&&absq(xv)<=13107)
        settled_ticks<=settled_ticks<1000?settled_ticks+1'b1:settled_ticks;
      else settled_ticks<=0;
    end
   endcase
  end
 end
endmodule
