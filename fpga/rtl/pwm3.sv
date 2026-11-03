// Atomic three-phase update at the carrier boundary, 25 kHz at 50 MHz.
// The motor module accepts 3PWM; complementary/dead-time logic is in its driver.
module pwm3 #(parameter integer PERIOD=2000, REGISTERED_OUTPUT=0)(input wire clk,rst,enable,update,
 input wire [15:0] du,dv,dw,output wire u,v,w,output wire boundary);
 localparam integer WIDTH=$clog2(PERIOD)<1?1:$clog2(PERIOD);
 reg [WIDTH-1:0] count,pu,pv,pw,au,av,aw;
 reg ru,rv,rw;
 wire [47:0] tu=du*PERIOD,tv=dv*PERIOD,tw=dw*PERIOD;
 wire [WIDTH-1:0] nu=tu[16+:WIDTH],nv=tv[16+:WIDTH],nw=tw[16+:WIDTH];
 assign boundary=count==0;
 // Top-level uses registered pins: raw permission clears all phase registers
 // on the same edge as EN. Standalone users can retain asynchronous gating.
 assign u=REGISTERED_OUTPUT ? ru : enable && ru;
 assign v=REGISTERED_OUTPUT ? rv : enable && rv;
 assign w=REGISTERED_OUTPUT ? rw : enable && rw;
 always @(posedge clk) begin
   if(rst) begin count<=0;pu<=0;pv<=0;pw<=0;au<=0;av<=0;aw<=0;ru<=0;rv<=0;rw<=0;end
   else begin
     count<=count==PERIOD-1 ? 0:count+1'b1;
     if(!enable) begin pu<=0;pv<=0;pw<=0;au<=0;av<=0;aw<=0;ru<=0;rv<=0;rw<=0;end
     else begin
       if(update) begin pu<=nu;pv<=nv;pw<=nw;end
       if(count==PERIOD-1) begin
        au<=update ? nu:pu;av<=update ? nv:pv;aw<=update ? nw:pw;
        ru<=(update?nu:pu)!=0;rv<=(update?nv:pv)!=0;rw<=(update?nw:pw)!=0;
       end else begin ru<=count+1<au;rv<=count+1<av;rw<=count+1<aw;end
     end
   end
 end
endmodule
