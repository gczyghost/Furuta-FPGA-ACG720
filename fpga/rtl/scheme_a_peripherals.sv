// Related native PLL clocks. Every crossing remains subject to ordinary STA.
// Reset asserts in both domains; each domain releases on its own clock.
module scheme_a_reset(input wire clk,assert_reset,output wire rst);
 reg [1:0] release_sync=0;
 always @(posedge clk or posedge assert_reset)
  if(assert_reset)release_sync<=0;else release_sync<={release_sync[0],1'b1};
 assign rst=assert_reset||!release_sync[1];
endmodule

module scheme_a_sensors(
 input wire core_clk,fast_clk,rst,tick,input wire [31:0] stamp,
 input wire as_miso,pend_miso,adc_dout,
 output wire as_sck,as_mosi,as_csn,pend_sck,pend_csn,adc_sclk,adc_csn,adc_din,
 output wire snapshot,valid,busy,output wire [7:0] error,
 output wire [13:0] arm_raw,output wire [11:0] pend_raw,ia_raw,ib_raw,
 output wire [31:0] arm_stamp,pend_stamp,ia_stamp,ib_stamp,
 output wire transport_fault
);
 wire cr,fr,start_fast,tick_busy,tick_overrun,unused_tick;
 scheme_a_reset rc(core_clk,rst,cr),rf(fast_clk,rst,fr);
 scheme_a_mailbox #(.WIDTH(1)) requests(core_clk,cr,tick,1'b1,tick_busy,tick_overrun,
  fast_clk,fr,start_fast,unused_tick);
 wire done_fast,valid_fast,busy_fast,reply_busy,reply_overrun;
 wire [7:0] error_fast;wire [13:0] arm_fast;wire [11:0] pend_fast,ia_fast,ib_fast;
 wire [31:0] as_at_fast,pend_at_fast,ia_at_fast,ib_at_fast;
 // This related-clock registered bus has no timing exception. Data coherence
 // depends on routed setup/hold, unlike the independent-phase mailbox protocol.
 reg [31:0] stamp_fast;
 always @(posedge fast_clk)stamp_fast<=stamp;
 sampling_frontend sensors(fast_clk,fr,start_fast,stamp_fast,as_miso,pend_miso,adc_dout,
  as_sck,as_mosi,as_csn,pend_sck,pend_csn,adc_sclk,adc_csn,adc_din,
  done_fast,valid_fast,busy_fast,error_fast,arm_fast,pend_fast,ia_fast,ib_fast,
  as_at_fast,pend_at_fast,ia_at_fast,ib_at_fast);
 scheme_a_mailbox #(.WIDTH(187)) replies(fast_clk,fr,done_fast,
  {valid_fast,error_fast,arm_fast,pend_fast,ia_fast,ib_fast,as_at_fast,pend_at_fast,ia_at_fast,ib_at_fast},
  reply_busy,reply_overrun,core_clk,cr,snapshot,
  {valid,error,arm_raw,pend_raw,ia_raw,ib_raw,arm_stamp,pend_stamp,ia_stamp,ib_stamp});
 reg [1:0] busy_sync,overrun_sync;
 always @(posedge core_clk or posedge cr)begin
  if(cr)begin busy_sync<=0;overrun_sync<=0;end
  else begin busy_sync<={busy_sync[0],busy_fast};overrun_sync<={overrun_sync[0],reply_overrun};end
 end
 assign busy=tick_busy||busy_sync[1];
 assign transport_fault=tick_overrun||overrun_sync[1];
endmodule

module scheme_a_uart(
 input wire core_clk,fast_clk,rst,rx,
 output wire rx_valid,rx_error,output wire [7:0] rx_data,
 input wire tx_start,input wire [7:0] tx_data,output wire tx,tx_busy,
 output wire transport_fault
);
 wire cr,fr,rv,re;wire [7:0] rd;
 scheme_a_reset rc(core_clk,rst,cr),rf(fast_clk,rst,fr);
 uart_rx #(.DIV(109)) receiver(fast_clk,fr,rx,rv,re,rd);
 wire receive_busy,receive_overrun,received,received_error;
 scheme_a_mailbox #(.WIDTH(9)) incoming(fast_clk,fr,rv||re,{re,rd},receive_busy,receive_overrun,
  core_clk,cr,received,{received_error,rx_data});
 reg [1:0] overrun_sync;
 always @(posedge core_clk or posedge cr)
  if(cr)overrun_sync<=0;else overrun_sync<={overrun_sync[0],receive_overrun};
 assign rx_valid=received&&!received_error&&!overrun_sync[1];
 assign rx_error=(received&&received_error)||overrun_sync[1];
 // TX acknowledgement means serial completion, not merely mailbox capture.
 // Producer busy stays asserted from acceptance through the stop bit.
 reg request,acknowledge,tx_overrun;
 reg [1:0] ack_sync,request_sync;
 reg [7:0] held;
 assign tx_busy=request!=ack_sync[1];
 always @(posedge core_clk or posedge cr)begin
  if(cr)begin request<=0;held<=0;ack_sync<=0;tx_overrun<=0;end
  else begin
   ack_sync<={ack_sync[0],acknowledge};
   if(tx_start)begin
    if(tx_busy)tx_overrun<=1;
    else begin held<=tx_data;request<=~request;end
   end
  end
 end
 reg launch,active,saw_busy;wire serial_busy;
 uart_tx #(.DIV(109)) transmitter(fast_clk,fr,launch,held,tx,serial_busy);
 always @(posedge fast_clk or posedge fr)begin
  if(fr)begin request_sync<=0;acknowledge<=0;launch<=0;active<=0;saw_busy<=0;end
  else begin
   request_sync<={request_sync[0],request};launch<=0;
   if(!active&&request_sync[1]!=acknowledge)begin launch<=1;active<=1;saw_busy<=0;end
   if(active&&serial_busy)saw_busy<=1;
   if(active&&saw_busy&&!serial_busy)begin acknowledge<=request_sync[1];active<=0;end
  end
 end
 assign transport_fault=overrun_sync[1]||tx_overrun;
endmodule

module scheme_a_pwm(
 input wire core_clk,fast_clk,rst,enable,update,
 input wire [15:0] du,dv,dw,output wire u,v,w,boundary,transport_fault
);
 // Permission removal flushes BOTH mailbox endpoints, including while a clock
 // is stopped. Fresh permission cannot replay a queued pre-STOP PWM triple.
 wire inhibit=rst||!enable;wire cr,fr,received,busy,overrun;
 wire [15:0] fu,fv,fw;
 scheme_a_reset rc(core_clk,inhibit,cr),rf(fast_clk,inhibit,fr);
 scheme_a_mailbox #(.WIDTH(48)) triples(core_clk,cr,update&&enable,{du,dv,dw},busy,overrun,
  fast_clk,fr,received,{fu,fv,fw});
 reg fresh;
 always @(posedge fast_clk or posedge fr)
  if(fr)fresh<=0;else if(received)fresh<=1;
 wire pu,pv,pw;
 pwm3 #(.PERIOD(2000),.REGISTERED_OUTPUT(1)) carrier(fast_clk,fr,
  enable&&(fresh||received),received,fu,fv,fw,pu,pv,pw,boundary);
 // Source permission bypasses transport and carrier boundaries for shutdown.
 assign u=enable&&!rst&&fresh&&pu;
 assign v=enable&&!rst&&fresh&&pv;
 assign w=enable&&!rst&&fresh&&pw;
 reg failed;
 always @(posedge core_clk)begin
  if(rst)failed<=0;
  else if(enable&&update&&busy&&!cr)failed<=1;
 end
 assign transport_fault=failed;
endmodule
