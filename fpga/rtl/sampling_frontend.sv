module sampling_frontend #(parameter integer SENSOR_HALF=100, ADC_HALF=3, SENSOR_GAP=250, ADC_NEGEDGE=0, ADC_CS_SETUP=2*ADC_HALF)(
 input wire clk,rst,tick, input wire [31:0] stamp,
 input wire as_miso,pend_miso,adc_dout,
 output wire as_sck,as_mosi,as_csn,pend_sck,pend_csn,adc_sclk,adc_csn,adc_din,
 output reg snapshot,valid,busy, output reg [7:0] error,
 output wire [13:0] arm_raw,output wire [11:0] pend_raw,ia_raw,ib_raw,
 output wire [31:0] arm_stamp,pend_stamp,ia_stamp,ib_stamp
);
 wire adone,pdone,cdone,av,pv,cv;wire ab,pb,cb;wire [15:0] diag;wire [7:0] ae;wire [17:0] pr;
 reg [2:0] finished;reg launch;
 encoder_as5048 #(.HALF_CLKS(SENSOR_HALF),.GAP_CLKS(SENSOR_GAP)) asenc(clk,rst,launch,as_miso,stamp,as_sck,as_mosi,as_csn,ab,adone,av,arm_raw,arm_stamp,diag,ae);
 encoder_pendulum #(.HALF_CLKS(SENSOR_HALF),.GAP_CLKS(SENSOR_GAP)) pend(clk,rst,launch,pend_miso,stamp,pend_sck,pend_csn,pb,pdone,pv,pend_raw,pend_stamp,pr);
 adc128s102 #(.HALF_CLKS(ADC_HALF),.NEG_EDGE_CAPTURE(ADC_NEGEDGE),.CS_SETUP_CLKS(ADC_CS_SETUP)) adc(clk,rst,launch,adc_dout,stamp,adc_sclk,adc_csn,adc_din,cb,cdone,cv,ia_raw,ib_raw,ia_stamp,ib_stamp);
 always @(posedge clk) begin
   if(rst) begin snapshot<=0;valid<=0;busy<=0;error<=0;finished<=0;launch<=0;end
   else begin
     snapshot<=0;launch<=0;
     if(tick) begin
       if(busy) begin error<=8'h01;valid<=0;snapshot<=1;end
       else begin launch<=1;busy<=1;finished<=0;end
     end
     if(adone) finished[0]<=1;
     if(pdone) finished[1]<=1;
     if(cdone) finished[2]<=1;
     if(busy && (&finished)) begin
       snapshot<=1;valid<=av&&pv&&cv;busy<=0;
       error<=!av ? 8'h02:!pv ? 8'h03:!cv ? 8'h04:0;
     end
   end
 end
endmodule
