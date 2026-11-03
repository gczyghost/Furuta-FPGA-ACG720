// Experimental resource study only. Not included by either enabled build.
// Same fixed_math operation order as foc_core, one registered multiplier.
module foc_core(input wire clk,rst,start,enable,
 input wire signed [31:0] ia,ib,iq_ref,
 input wire [15:0] phase_sample,phase_output,
 output reg busy,done,output reg signed [31:0] id,iq,ud,uq,
 output reg [15:0] duty_u,duty_v,duty_w) /* synthesis syn_dspstyle = "logic" */;
 import fixed_math::*;
 localparam signed [31:0] LIMIT=884736;
 reg [6:0] state;
 reg signed [31:0] a,b,twice_b,clarke,beta,refq,sd,cd,so,co;
 reg signed [31:0] pd0,pd1,pq0,pq1,parkd,parkq,dd,dq,fd,fq;
 reg signed [31:0] ed,eq,zd,zq,kpd,kpq,rd,rq,scale;
 reg signed [31:0] awd,awq,kid,kiq,bd,bq,izd,izq;
 reg signed [31:0] ip0,ip1,ip2,ip3,alpha_v,beta_v,sv_a,sv_b;
 reg signed [31:0] va,vb,vc,minimum,su,sv,sw,cu,cv,cw,du_scaled,dv_scaled,dw_scaled;
 reg signed [31:0] mul_a,mul_b;
 reg signed [63:0] product /* synthesis syn_dspstyle = "dsp" */;
 wire signed [31:0] value=sat64(product>>>16);
 reg [63:0] square_d,square_q,magnitude;
 reg [15:0] sample_phase,out_phase,trig_phase;
 reg trig_start,sqrt_start,div_start;
 wire trig_done,sqrt_done,div_done;
 wire signed [31:0] ts,tc;
 wire [31:0] norm;wire [63:0] ratio;
 trig_lut trig(clk,rst,trig_start,trig_phase,trig_done,ts,tc);
 isqrt64 sq(clk,rst,sqrt_start,magnitude,,sqrt_done,norm);
 udiv64 dv(clk,rst,div_start,{16'b0,LIMIT,16'b0},norm,,div_done,ratio);
 always @*begin
  mul_a=0;mul_b=0;
  case(state)
   3:begin mul_a=clarke;mul_b=37837;end
   8:begin mul_a=a;mul_b=cd;end
   10:begin mul_a=beta;mul_b=sd;end
   12:begin mul_a=beta;mul_b=cd;end
   14:begin mul_a=a;mul_b=sd;end
   18:begin mul_a=54613;mul_b=dd;end
   20:begin mul_a=54613;mul_b=dq;end
   24:begin mul_a=458752;mul_b=ed;end
   26:begin mul_a=458752;mul_b=eq;end
   29:begin mul_a=rd;mul_b=rd;end
   31:begin mul_a=rq;mul_b=rq;end
   37:begin mul_a=rd;mul_b=scale;end
   39:begin mul_a=rq;mul_b=scale;end
   42:begin mul_a=192250;mul_b=ed;end
   44:begin mul_a=192250;mul_b=eq;end
   46:begin mul_a=3277;mul_b=awd;end
   48:begin mul_a=3277;mul_b=awq;end
   52:begin mul_a=ud;mul_b=co;end
   54:begin mul_a=uq;mul_b=so;end
   56:begin mul_a=ud;mul_b=so;end
   58:begin mul_a=uq;mul_b=co;end
   61:begin mul_a=32768;mul_b=alpha_v;end
   63:begin mul_a=56756;mul_b=beta_v;end
   69:begin mul_a=cu;mul_b=178956971;end
   71:begin mul_a=cv;mul_b=178956971;end
   73:begin mul_a=cw;mul_b=178956971;end
  endcase
 end
 always @(posedge clk)product<=mul_a*mul_b;
 always @(posedge clk)begin
  if(rst)begin
   state<=0;busy<=0;done<=0;id<=0;iq<=0;ud<=0;uq<=0;duty_u<=0;duty_v<=0;duty_w<=0;
   zd<=0;zq<=0;trig_start<=0;sqrt_start<=0;div_start<=0;
   a<=0;b<=0;twice_b<=0;clarke<=0;beta<=0;refq<=0;sd<=0;cd<=0;so<=0;co<=0;
   pd0<=0;pd1<=0;pq0<=0;pq1<=0;parkd<=0;parkq<=0;dd<=0;dq<=0;fd<=0;fq<=0;
   ed<=0;eq<=0;kpd<=0;kpq<=0;rd<=0;rq<=0;scale<=0;awd<=0;awq<=0;kid<=0;kiq<=0;bd<=0;bq<=0;izd<=0;izq<=0;
   ip0<=0;ip1<=0;ip2<=0;ip3<=0;alpha_v<=0;beta_v<=0;sv_a<=0;sv_b<=0;
   va<=0;vb<=0;vc<=0;minimum<=0;su<=0;sv<=0;sw<=0;cu<=0;cv<=0;cw<=0;du_scaled<=0;dv_scaled<=0;dw_scaled<=0;
   square_d<=0;square_q<=0;magnitude<=0;sample_phase<=0;out_phase<=0;trig_phase<=0;
  end else begin
   done<=0;trig_start<=0;sqrt_start<=0;div_start<=0;
   case(state)
    0:begin
     if(!enable)begin zd<=0;zq<=0;duty_u<=0;duty_v<=0;duty_w<=0;ud<=0;uq<=0;end
     if(start)begin a<=ia;b<=ib;refq<=iq_ref;sample_phase<=phase_sample;out_phase<=phase_output;busy<=1;state<=1;end
    end
    1:begin twice_b<=add(b,b);state<=2;end
    2:begin clarke<=add(a,twice_b);state<=3;end
    3:state<=4;
    4:begin beta<=value;trig_phase<=sample_phase;trig_start<=1;state<=5;end
    5:if(trig_done)begin sd<=ts;cd<=tc;trig_phase<=out_phase;trig_start<=1;state<=6;end
    6:if(trig_done)begin so<=ts;co<=tc;state<=8;end
    8:state<=9;9:begin pd0<=value;state<=10;end
    10:state<=11;11:begin pd1<=value;state<=12;end
    12:state<=13;13:begin pq0<=value;state<=14;end
    14:state<=15;15:begin pq1<=value;state<=16;end
    16:begin parkd<=add(pd0,pd1);parkq<=add(pq0,-pq1);state<=17;end
    17:begin dd<=parkd-id;dq<=parkq-iq;state<=18;end
    18:state<=19;19:begin fd<=value;state<=20;end
    20:state<=21;21:begin fq<=value;state<=22;end
    22:begin id<=add(id,fd);iq<=add(iq,fq);state<=23;end
    23:begin ed<=-id;eq<=refq-iq;state<=24;end
    24:state<=25;25:begin kpd<=value;state<=26;end
    26:state<=27;27:begin kpq<=value;state<=28;end
    28:begin rd<=add(kpd,zd);rq<=add(kpq,zq);state<=29;end
    29:state<=30;30:begin square_d<=product;state<=31;end
    31:state<=32;32:begin square_q<=product;state<=33;end
    33:begin magnitude<=square_d+square_q;state<=34;end
    34:begin sqrt_start<=1;state<=35;end
    35:if(sqrt_done)begin
      if(norm>LIMIT)begin div_start<=1;state<=36;end
      else begin ud<=rd;uq<=rq;state<=41;end
    end
    36:if(div_done)begin scale<=$signed(ratio[31:0]);state<=37;end
    37:state<=38;38:begin ud<=value;state<=39;end
    39:state<=40;40:begin uq<=value;state<=41;end
    41:begin awd<=ud-rd;awq<=uq-rq;state<=42;end
    42:state<=43;43:begin kid<=value;state<=44;end
    44:state<=45;45:begin kiq<=value;state<=46;end
    46:state<=47;47:begin bd<=value;state<=48;end
    48:state<=49;49:begin bq<=value;state<=50;end
    50:begin izd<=add(kid,bd);izq<=add(kiq,bq);state<=51;end
    51:begin zd<=enable?clip(add(zd,izd),-LIMIT,LIMIT):0;zq<=enable?clip(add(zq,izq),-LIMIT,LIMIT):0;state<=52;end
    52:state<=53;53:begin ip0<=value;state<=54;end
    54:state<=55;55:begin ip1<=value;state<=56;end
    56:state<=57;57:begin ip2<=value;state<=58;end
    58:state<=59;59:begin ip3<=value;state<=60;end
    60:begin alpha_v<=add(ip0,-ip1);beta_v<=add(ip2,ip3);state<=61;end
    61:state<=62;62:begin sv_a<=value;state<=63;end
    63:state<=64;64:begin sv_b<=value;state<=65;end
    65:begin va<=alpha_v;vb<=add(-sv_a,sv_b);vc<=add(-sv_a,-sv_b);state<=66;end
    66:begin minimum<=va<vb?(va<vc?va:vc):(vb<vc?vb:vc);state<=67;end
    67:begin su<=va-minimum;sv<=vb-minimum;sw<=vc-minimum;state<=68;end
    68:begin cu<=clip(su,0,24*65536);cv<=clip(sv,0,24*65536);cw<=clip(sw,0,24*65536);state<=69;end
    69:state<=70;70:begin du_scaled<=product>>>32;state<=71;end
    71:state<=72;72:begin dv_scaled<=product>>>32;state<=73;end
    73:state<=74;74:begin dw_scaled<=product>>>32;state<=75;end
    75:begin
     duty_u<=enable?(du_scaled>=65535?16'hffff:du_scaled[15:0]):0;
     duty_v<=enable?(dv_scaled>=65535?16'hffff:dv_scaled[15:0]):0;
     duty_w<=enable?(dw_scaled>=65535?16'hffff:dw_scaled[15:0]):0;
     if(!enable)begin ud<=0;uq<=0;end
     done<=1;busy<=0;state<=0;
    end
   endcase
  end
 end
endmodule
