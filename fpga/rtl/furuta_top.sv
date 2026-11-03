// R1 perfboard. HARD_DISABLE=1 is the delivered, synthesis-default interlock.
// Simulations may set it to 0; configuration and a fresh explicit ARM are
// still mandatory. ACG720 S0-S4 select one explicitly owned local command producer.
module furuta_top #(parameter integer CLOCK_HZ=50000000, HARD_DISABLE=1, QUIET_SAMPLES=100, MULTITURN=0,
 STAMP_HZ=CLOCK_HZ, SENSOR_HALF=100, SENSOR_GAP=250, ADC_HALF=3, ADC_NEGEDGE=0, ADC_CS_SETUP=2*ADC_HALF, EXTERNAL_CLOCK=0)(
 input wire CLK_50M,UART_RX,AS_MISO,AS_PWM,PEND_DO,ADC_DOUT,
 output wire UART_TX,PWM_U,PWM_V,PWM_W,EN_REQ,ENC_OE,
 output wire AS_SCK,AS_MOSI,AS_CSN,PEND_SCK,PEND_CSN,ADC_CSN,ADC_DIN,ADC_SCLK,
 input wire [4:0] KEY_N, input wire CLOCK_READY,CLOCK_FAULT);
 localparam integer POWER_WIDTH=$clog2(CLOCK_HZ/100)<1?1:$clog2(CLOCK_HZ/100);
 localparam integer TICK_WIDTH=$clog2(CLOCK_HZ/2000)<1?1:$clog2(CLOCK_HZ/2000);
 reg [POWER_WIDTH-1:0] power_count=0;reg rst=1,clock_started=0;
 // Registered release avoids a counter comparator on the external OE path.
 always @(posedge CLK_50M) if(EXTERNAL_CLOCK!=0&&!clock_started&&CLOCK_READY!==1'b1)begin power_count<=0;rst<=1;end
 else if(rst) begin
  clock_started<=1;
  if(power_count==CLOCK_HZ/100-1)rst<=0;
  else power_count<=power_count+1'b1;
 end
 reg enc_oe=0;
 always @(posedge CLK_50M)enc_oe<=!rst;
 assign ENC_OE=enc_oe;
 reg [31:0] stamp=0;reg [TICK_WIDTH-1:0] tick_count=0;
 reg [1:0] stamp_phase=0;
 wire tick=!rst&&tick_count==CLOCK_HZ/2000-1;
 // At25MHz this is2*elapsed_core_cycles; at40MHz floor(5*cycles/4), modulo2^32.
 // All sensor/completion/telemetry timestamps share it; no scaling of wrapped packets.
 always @(posedge CLK_50M)begin
  if(CLOCK_HZ==25000000&&STAMP_HZ==50000000)stamp<=stamp+32'd2;
  else if(CLOCK_HZ==40000000&&STAMP_HZ==50000000)begin stamp_phase<=stamp_phase+1'b1;stamp<=stamp+(stamp_phase==3?32'd2:32'd1);end
  else stamp<=stamp+1'b1;
  if(rst||tick)tick_count<=0;else tick_count<=tick_count+1'b1;
 end
 wire snapshot,valid,sample_busy,clear_control;wire [7:0] sample_error;
 wire [13:0] arm_raw;wire [11:0] pend_raw,ia_raw,ib_raw;
 wire [31:0] as_at,pend_at,ia_at,ib_at;
 sampling_frontend #(.SENSOR_HALF(SENSOR_HALF),.SENSOR_GAP(SENSOR_GAP),.ADC_HALF(ADC_HALF),.ADC_NEGEDGE(ADC_NEGEDGE),.ADC_CS_SETUP(ADC_CS_SETUP)) sensors(CLK_50M,rst||clear_control,tick,stamp,AS_MISO,PEND_DO,ADC_DOUT,
  AS_SCK,AS_MOSI,AS_CSN,PEND_SCK,PEND_CSN,ADC_SCLK,ADC_CSN,ADC_DIN,
  snapshot,valid,sample_busy,sample_error,arm_raw,pend_raw,ia_raw,ib_raw,as_at,pend_at,ia_at,ib_at);
 wire rx_valid,rx_error,cmd;wire [7:0] rx_data,op,addr,seq;wire [31:0] value;
 uart_rx #(.DIV((CLOCK_HZ+230400)/460800)) rx(CLK_50M,rst,UART_RX,rx_valid,rx_error,rx_data);
 wire host_cmd;wire [7:0] host_op,host_addr,host_seq;wire [31:0] host_value;
 command_link #(.BYTE_TIMEOUT(CLOCK_HZ/100)) link(CLK_50M,rst,rx_valid,rx_error,rx_data,host_cmd,host_op,host_addr,host_seq,host_value);
 wire ack_valid;wire [7:0] ack_result,ack_addr,ack_seq;wire [31:0] ack_value;
 wire core_enable;
 wire run,fault,calibrated,ready,control_done,control_fault,control_busy;
 wire core_control_fault;wire [7:0] core_control_reason,reason,control_reason;
 assign control_fault=core_control_fault||(EXTERNAL_CLOCK!=0&&CLOCK_FAULT);
 // Existing0x10 denotes an unavailable/deadline-failed control pipeline.
 assign control_reason=(EXTERNAL_CLOCK!=0&&CLOCK_FAULT)?8'h10:core_control_reason;
 wire [2:0] mode;
 wire signed [31:0] offset_a,offset_b,gain_a,gain_b,iq_set,ia,ib,id,iq,ud,uq,iq_ref,q,alpha,w,v;
 wire [13:0] arm_zero;wire [11:0] pend_down;wire [15:0] electrical_zero;wire [5:0] poles;wire reverse;
 wire cal_start,cal_finished,cal_failed,cal_update,cal_reverse;
 wire [7:0] cal_error;wire [31:0] cal_status,cal_a,cal_b;wire [15:0] cal_zero,cu,cv,cw;
 wire commissioning_mode=mode==5||mode==6;
 wire drop_dose_ok,long_run,home_active,home_settled;wire [63:0] dose_value;
 wire [1:0] motion_kind;wire [3:0] motion_bank;wire signed [31:0] motion_origin,motion_goal;
 wire motion_end,motion_resume,motion_finished,motion_arrived;
 wire [31:0] protocol_epoch;wire [3:0] protocol_eligibility;
 wire signed [31:0] telemetry_cap,telemetry_reference_q,telemetry_reference_w;
 wire telemetry_reference_valid;
 wire rebase_commit,rebase_inhibit,keep_session,finite_speed;wire signed [31:0] rebase_shift,frame_turns;
 // Exact unsigned modulo-2^32 age comparison. For a sub-65536 bound,
 // a fresh age has zero high difference after the low-half borrow.
 function automatic age_fresh(input [31:0] now_stamp,then_stamp);
  reg [16:0] low_difference;
  reg [3:0] gen_borrow,prop_borrow;
  reg [4:0] borrow;
  reg [15:0] low_result;
  integer j;
  reg [15:0] high_mismatch0,high_mismatch1;
  reg [3:0] high_clear0,high_clear1;
  reg high_eq0,high_eq1;
  reg [15:0] prefix1,prefix2,prefix4,prefix8,high_carry;
  reg [31:0] full_difference;
  begin
   for(j=0;j<4;j=j+1)begin
    gen_borrow[j]=now_stamp[j*4+:4]<then_stamp[j*4+:4];
    prop_borrow[j]=now_stamp[j*4+:4]==then_stamp[j*4+:4];
   end
   borrow[0]=1'b0;
   borrow[1]=gen_borrow[0];
   borrow[2]=gen_borrow[1]|(prop_borrow[1]&gen_borrow[0]);
   borrow[3]=gen_borrow[2]|(prop_borrow[2]&gen_borrow[1])|(prop_borrow[2]&prop_borrow[1]&gen_borrow[0]);
   borrow[4]=gen_borrow[3]|(prop_borrow[3]&gen_borrow[2])|(prop_borrow[3]&prop_borrow[2]&gen_borrow[1])|(prop_borrow[3]&prop_borrow[2]&prop_borrow[1]&gen_borrow[0]);
   for(j=0;j<4;j=j+1)low_result[j*4+:4]=now_stamp[j*4+:4]-then_stamp[j*4+:4]-{3'b0,borrow[j]};
   low_difference={borrow[4],low_result};
   // Speculate both high relations independently; actual borrow only selects below.
   // Four parallel Boolean prefix levels; no serial 16-bit increment.
   // Independent of low-half borrow until the existing terminal select.
   prefix1=then_stamp[31:16] & ((then_stamp[31:16]<<1)|16'h0001);
   prefix2=prefix1 & ((prefix1<<2)|16'h0003);
   prefix4=prefix2 & ((prefix2<<4)|16'h000f);
   prefix8=prefix4 & ((prefix4<<8)|16'h00ff);
   high_carry={prefix8[14:0],1'b1};
   high_mismatch0=now_stamp[31:16]^then_stamp[31:16];
   high_mismatch1=high_mismatch0^high_carry;
   for(j=0;j<4;j=j+1)begin
    high_clear0[j]=~(|high_mismatch0[j*4+:4]);
    high_clear1[j]=~(|high_mismatch1[j*4+:4]);
   end
   high_eq0=(high_clear0[0]&high_clear0[1])&(high_clear0[2]&high_clear0[3]);
   high_eq1=(high_clear1[0]&high_clear1[1])&(high_clear1[2]&high_clear1[3]);
   full_difference=now_stamp-then_stamp;
   if(STAMP_HZ/1000>0&&STAMP_HZ/1000<65536)
    age_fresh=(low_difference[16] ? high_eq1:high_eq0)&&low_difference[15:0]<STAMP_HZ/1000;
   else age_fresh=full_difference<STAMP_HZ/1000;
  end
 endfunction
 wire sample_fresh=valid && age_fresh(stamp,as_at) && age_fresh(stamp,pend_at) && age_fresh(stamp,ia_at) && age_fresh(stamp,ib_at);
 supervisor #(.HEARTBEAT_CLKS(CLOCK_HZ*4/5),.SAMPLE_CLKS(CLOCK_HZ/1000),.QUIET_SAMPLES(QUIET_SAMPLES),.MULTITURN(MULTITURN)) guard(
  CLK_50M,rst,cmd,op,addr,seq,value,snapshot,sample_fresh,control_done,control_fault,sample_error,control_reason,
  ia,ib,q,alpha,w,v,run,fault,clear_control,calibrated,reason,mode,offset_a,offset_b,gain_a,gain_b,iq_set,
  arm_zero,pend_down,electrical_zero,poles,reverse,ready,ack_valid,ack_result,ack_addr,ack_seq,ack_value,
  cal_finished,cal_failed,cal_error,cal_status,cal_a,cal_b,cal_zero,cal_reverse,cal_start,drop_dose_ok,long_run,dose_value,home_active,home_settled,motion_kind,motion_bank,motion_origin,motion_end,motion_resume,motion_finished,motion_arrived,motion_goal,rebase_commit,rebase_shift,rebase_inhibit,keep_session,finite_speed,protocol_epoch,protocol_eligibility,core_enable);
 wire [15:0] du,dv,dw;
 // guard.rebase_inhibit includes raw snapshot regardless of freshness.
 motor_control #(.MULTITURN(MULTITURN),.EXTERNAL_SAMPLE_INHIBIT(1)) core(CLK_50M,rst||clear_control,snapshot&&sample_fresh&&calibrated,core_enable,mode,
  arm_raw,arm_zero,pend_raw,pend_down,ia_raw,ib_raw,offset_a,offset_b,gain_a,gain_b,iq_set,poles,reverse,electrical_zero,
  control_busy,control_done,core_control_fault,core_control_reason,du,dv,dw,ia,ib,id,iq,ud,uq,iq_ref,q,alpha,w,v,drop_dose_ok,long_run,dose_value,home_active,home_settled,motion_kind,motion_bank,motion_origin,motion_end,motion_resume,motion_finished,motion_arrived,motion_goal,rebase_inhibit,rebase_commit,rebase_shift,frame_turns,keep_session,finite_speed,telemetry_cap,telemetry_reference_q,telemetry_reference_w,telemetry_reference_valid);
 commissioning calibration_engine(CLK_50M,rst||clear_control||cal_start,run&&!fault&&commissioning_mode,
  mode,snapshot&&sample_fresh,arm_raw,ia_raw,ib_raw,offset_a,offset_b,gain_a,gain_b,poles,
  cal_finished,cal_failed,cal_error,cal_status,cal_a,cal_b,cal_zero,cal_reverse,cal_update,cu,cv,cw,ia_at);
 wire [15:0] pwm_u=commissioning_mode?cu:du,pwm_v=commissioning_mode?cv:dv,pwm_w=commissioning_mode?cw:dw;
 // Register the complete permission, including timestamp checks. Fault/STOP
 // takes at most one additional 20 ns system cycle, never a PWM carrier cycle.
 // This removes wide timestamp subtractors from the physical EN/PWM pin path.
 wire output_permit=!HARD_DISABLE && !rst && run && !fault && !control_fault &&
    !(commissioning_mode&&(cal_failed||cal_finished||!cal_status[0])) && !(snapshot&&!sample_fresh);
 reg output_enable=0;
 always @(posedge CLK_50M) begin
  if(rst)output_enable<=0;
  else output_enable<=output_permit;
 end
 assign EN_REQ=!HARD_DISABLE && output_enable;
 wire boundary;
 pwm3 #(.PERIOD(CLOCK_HZ/25000),.REGISTERED_OUTPUT(1)) pwm(CLK_50M,rst,output_permit,commissioning_mode?cal_update:control_done,pwm_u,pwm_v,pwm_w,PWM_U,PWM_V,PWM_W,boundary);
 wire tx_busy,tx_start;wire [7:0] tx_data;
 localparam integer TELEMETRY_WORDS=MULTITURN!=0?30:17;
 wire [31:0] motion_status={20'd0,(mode==4&&motion_kind==3&&!motion_arrived),keep_session,home_active,home_settled,(finite_speed&&(mode==0&&motion_kind==3||mode==4&&motion_kind==3&&!motion_arrived)),motion_end,motion_arrived,motion_finished,(mode==0&&motion_kind!=0),1'b0,motion_kind};
 wire [543:0] legacy_values={stamp,4'b0,ia_raw,4'b0,ib_raw,20'b0,pend_raw,sample_fresh,7'b0,sample_error,2'b0,arm_raw,16'b0,pwm_w,pwm_u,pwm_v,uq,ud,iq_ref,iq,ib,ia,v,w,alpha,q,
   8'hf1,(MULTITURN!=0?8'h04:8'h02),reason,calibrated,mode,(HARD_DISABLE!=0),ready,fault,run};
 wire [TELEMETRY_WORDS*32-1:0] telemetry_values;
 generate if(MULTITURN!=0)begin: full_telemetry
   wire signed [31:0] sensor_raw,sensor_filtered;wire sensor_valid;wire [31:0] sensor_at;
   sensor_velocity #(.SAMPLE_CLKS(STAMP_HZ/2000)) sensor_speed(CLK_50M,rst||clear_control,snapshot,sample_fresh,reverse,arm_raw,as_at,
     sensor_raw,sensor_filtered,sensor_valid,sensor_at);
   reg [31:0] completed_at;reg have_completed;
   reg [959:32] completed_values;
   wire completed_fresh=have_completed&&!clear_control&&age_fresh(stamp,completed_at);
   // bits0..4 describe the frozen control sample; bits8..11 are live eligibility.
   wire [31:0] sample_flags={27'd0,1'b1,1'b1,1'b1,(sensor_valid&&sensor_at==as_at),telemetry_reference_valid};
   wire [31:0] live_flags={20'd0,protocol_eligibility,3'd0,completed_fresh,
      (completed_fresh ? completed_values[27*32+:4]:4'd0)};
   always @(posedge CLK_50M)begin
     if(rst||clear_control)begin completed_values<=0;completed_at<=0;have_completed<=0;end
     else if(control_done)begin
       completed_at<=as_at;have_completed<=1;
       completed_values<={sensor_raw,protocol_epoch,sample_flags,as_at,as_at,sensor_filtered,
         telemetry_reference_q,telemetry_reference_w,dose_value[47:16],telemetry_cap,
         motion_status,motion_goal,frame_turns,legacy_values[543:32]};
     end
   end
   // Keep faults/STOP visible even if no new valid control sample can complete.
   assign telemetry_values={completed_values[959:928],protocol_epoch,live_flags,completed_values[863:32],legacy_values[31:0]};
 end else begin assign telemetry_values=legacy_values;end endgenerate
 wire [4:0] local_keys,local_held;
 offline_keys buttons(CLK_50M,rst,tick,KEY_N,local_keys,local_held);
 wire local_request,local_request_ready,local_response,local_response_ok;
 wire [7:0] local_op,local_addr;wire [31:0] local_value,local_response_value;
 wire local_offline,local_abort,local_link_failed,local_activity,local_stop,local_failed;
 wire [7:0] local_state;
 wire local_invalidate;
  wire host_ack;wire [7:0] host_result,host_ack_addr,host_ack_seq;wire [31:0] host_ack_value;
 offline_link ownership(CLK_50M,rst,tick,local_keys[0],local_keys[1],local_stop,run,local_activity,
   host_cmd,host_op,host_addr,host_seq,host_value,cmd,op,addr,seq,value,
   ack_valid,ack_result,ack_addr,ack_seq,ack_value,
   host_ack,host_result,host_ack_addr,host_ack_seq,host_ack_value,
   local_request,local_op,local_addr,local_value,local_request_ready,
   local_response,local_response_ok,local_response_value,local_offline,local_abort,local_link_failed,local_invalidate);
 reg [5:0] local_observe_count=0;
 wire local_observe=tick&&local_observe_count==39;
 always @(posedge CLK_50M)begin
   if(rst||local_observe)local_observe_count<=0;
   else if(tick)local_observe_count<=local_observe_count+1'b1;
 end
 // Coherent whole-row boundary; delay the observation strobe by the same edge.
  // Live fault/reset cancellation below bypasses this informational snapshot.
  reg [959:0] local_row;
  reg local_observe_q;
  reg local_fault_seen;
  wire local_fault_now=fault||control_fault||(snapshot&&!sample_fresh);
  assign local_invalidate=local_fault_now&&!local_fault_seen;
  always @(posedge CLK_50M)begin
    if(rst)begin local_row<=0;local_observe_q<=0;local_fault_seen<=0;end
    else begin local_row<=telemetry_values;local_observe_q<=local_observe;local_fault_seen<=local_fault_now;end
  end
 offline_sequence #(.CLOCK_HZ(STAMP_HZ)) local_sequence(CLK_50M,rst,tick,local_observe_q,local_row,
   local_offline,local_abort,local_link_failed,local_keys,
   local_request_ready,local_response,local_response_ok,local_response_value,
   local_request,local_op,local_addr,local_value,local_stop,local_activity,local_failed,local_state);
 telemetry #(.PERIOD(CLOCK_HZ/(50*TELEMETRY_WORDS)),.WORDS(TELEMETRY_WORDS)) tel(CLK_50M,rst,
  telemetry_values,tx_busy,tx_start,tx_data,host_ack,host_result,host_ack_addr,host_ack_seq,host_ack_value);
 uart_tx #(.DIV((CLOCK_HZ+230400)/460800)) tx(CLK_50M,rst,tx_start,tx_data,UART_TX,tx_busy);
 // AS_PWM is physically routed on R1 but not used as a fallback for bad SPI.
endmodule
