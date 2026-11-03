module supervisor #(parameter integer HEARTBEAT_CLKS=40000000, SAMPLE_CLKS=50000, QUIET_SAMPLES=100, MULTITURN=0)(
 input wire clk,rst,cmd,input wire [7:0] op,addr,seq,input wire [31:0] value,
 input wire sample,valid,control_done,control_fault,input wire [7:0] sample_error,control_reason,
 input wire signed [31:0] ia,ib,q,alpha,w,v,
 output reg run,fault,clear_control,calibrated,output reg [7:0] reason,output reg [2:0] mode,
 output reg signed [31:0] offset_a,offset_b,gain_a,gain_b,iq_set /* synthesis syn_preserve = 1 */,
 output reg [13:0] arm_zero, output reg [11:0] pend_down,
 output reg [15:0] electrical_zero,output reg [5:0] poles,output reg reverse,
 output wire ready,
 output reg ack_valid,output reg [7:0] ack_result,ack_addr,ack_seq,output reg [31:0] ack_value,
 input wire cal_finished,cal_failed,input wire [7:0] cal_error,
 input wire [31:0] cal_status,cal_a,cal_b,input wire [15:0] cal_zero,input wire cal_reverse,
 output reg cal_start,input wire drop_dose_ok,output reg long_run,input wire [63:0] dose_value,input wire home_active,home_settled,
 output reg [1:0] motion_kind,output reg [3:0] motion_bank,output reg signed [31:0] motion_origin,
 output reg motion_end,motion_resume,input wire motion_finished,motion_arrived,input wire signed [31:0] motion_goal,
 input wire rebase_commit,input wire signed [31:0] rebase_shift,output wire rebase_inhibit,output reg keep_session,output reg finite_speed,
 output wire [31:0] protocol_epoch,output wire [3:0] protocol_eligibility,output reg core_enable /* synthesis syn_preserve = 1 */);
 wire eval_cmd;wire [7:0] eval_op,eval_addr,eval_seq;wire [31:0] eval_value;
 import fixed_math::*;
 import motion_metadata::*;
 function automatic bit speed_recipe_valid(input signed [31:0] rpm);
   case(rpm)
    -200,-180,-150,-120,-100,-80,-60,-50,-40,-30,-20,-15,-10,-5,-1,
    1,5,10,15,20,30,40,50,60,80,100,120,150,180,200:speed_recipe_valid=1;
    default:speed_recipe_valid=0;
   endcase
 endfunction
 // Magnitude limits without a 32-bit negate/mux ahead of the comparison.
 // Bounds here are positive and below INT_MAX, including trajectory tables.
 function automatic bit mag_le(input signed [31:0] x,limit);
   mag_le=x>=-limit&&x<=limit;
 endfunction
 function automatic bit mag_lt(input signed [31:0] x,limit);
   mag_lt=x> -limit&&x<limit;
 endfunction
 // Downward orientation is equivalent at 0 and +/-TAU after reverse drop.
 // Parallel intervals retain original physical thresholds and reject angles
 // outside the single-wrap domain used by the downstream controllers.
 function automatic bit down_le(input signed [31:0] x,limit);
   down_le=mag_le(x,limit)||(x>=TAU-limit&&x<=TAU+limit)||
     (x>=-TAU-limit&&x<=-TAU+limit);
 endfunction
 function automatic bit down_lt(input signed [31:0] x,limit);
   down_lt=mag_lt(x,limit)||(x>TAU-limit&&x<TAU+limit)||
     (x>-TAU-limit&&x<-TAU+limit);
 endfunction
 reg [31:0] hb_age,sample_age;reg hb_seen,seq_seen;reg [7:0] last_seq,config_mask;reg [7:0] healthy;

 reg [9:0] down_quiet,up_quiet;
 reg [20:0] run_ticks;
 reg [31:0] reset_epoch;
 reg motion_pending,pending_absolute;reg [1:0] pending_kind;reg [3:0] pending_bank;
 reg signed [31:0] pending_value,held_goal;
 localparam signed [31:0] START_LIMIT=MULTITURN!=0?TAU:182243;
 localparam signed [31:0] ENDPOINT_LIMIT=MULTITURN!=0?4194304:182243;
 // Accepted banks 0..4 require down_qualified (STOP or mode 0).
 // Accepted banks 5..8 require up_qualified (running mode 3/4).
 // Decode the command bank before arithmetic, avoiding a mode-dependent
 // mux in the subtract/range path. Existing qualification remains below.
 // Enabled v27 QSTEP/FAST starts at HomeController.goal, not noisy q.
 // Stopped entry still uses the measured starting coordinate.
 wire signed [31:0] down_origin=run&&mode==0&&motion_kind==0?motion_goal:q;
 wire signed [31:0] requested_origin=eval_addr>=5?held_goal:down_origin;
 wire direct_use_goal=run&&mode==0&&motion_kind==0;
 wire signed [31:0] direct_q_delta=$signed(eval_value)-q;
 wire signed [31:0] direct_goal_delta=$signed(eval_value)-motion_goal;
 wire signed [31:0] down_delta=$signed(eval_value)-down_origin;
 wire signed [31:0] up_delta=$signed(eval_value)-held_goal;
 wire signed [31:0] absolute_delta=eval_addr>=5?up_delta:down_delta;
 function automatic signed [31:0] phase_correction(input signed [31:0] delta);
   phase_correction=delta>PI+1 ? -TAU:delta< -PI-1 ? TAU:0;
 endfunction
 function automatic bit phase_delta_le(input signed [31:0] delta,limit);
   phase_delta_le=delta>PI+1 ? (delta>=TAU-limit&&delta<=TAU+limit):
     delta< -PI-1 ? (delta>=-TAU-limit&&delta<=-TAU+limit):mag_le(delta,limit);
 endfunction
 // MULTITURN absolute commands name a physical phase in [-PI,PI].
 // START_LIMIT and target guards keep this single wrap inside [-3PI,3PI].
 wire signed [31:0] down_angle=eval_op==8'h27 ?
   (MULTITURN!=0?down_delta+phase_correction(down_delta):down_delta):$signed(eval_value);
 wire signed [31:0] up_angle=eval_op==8'h27 ?
   (MULTITURN!=0?up_delta+phase_correction(up_delta):up_delta):$signed(eval_value);
 wire signed [31:0] requested_angle=eval_addr>=5?up_angle:down_angle;
 wire signed [31:0] requested_endpoint=eval_op==8'h27 ? $signed(eval_value)+(MULTITURN!=0?phase_correction(absolute_delta):0):requested_origin+$signed(eval_value);
 wire direct_q_nonzero=direct_q_delta!=0&&direct_q_delta!=TAU&&direct_q_delta!= -TAU;
 wire direct_goal_nonzero=direct_goal_delta!=0&&direct_goal_delta!=TAU&&direct_goal_delta!= -TAU;
 wire down_nonzero=direct_use_goal?direct_goal_nonzero:direct_q_nonzero;
 wire up_nonzero=up_delta!=0&&up_delta!=TAU&&up_delta!= -TAU;
 wire requested_nonzero=eval_op==8'h27 ? (MULTITURN!=0 ? (eval_addr>=5?up_nonzero:down_nonzero):absolute_delta!=0):eval_value!=0;
 // Select after each origin's full admission arithmetic; no extra cycle.
 wire direct_q_range=bank_range(eval_op==8'h27 ? direct_q_delta:$signed(eval_value),eval_addr[3:0],eval_op==8'h27&&MULTITURN!=0);
 wire direct_goal_range=bank_range(eval_op==8'h27 ? direct_goal_delta:$signed(eval_value),eval_addr[3:0],eval_op==8'h27&&MULTITURN!=0);
 wire down_range=direct_use_goal?direct_goal_range:direct_q_range;
 wire up_range=bank_range(eval_op==8'h27 ? up_delta:$signed(eval_value),eval_addr[3:0],eval_op==8'h27&&MULTITURN!=0);
 wire requested_range=eval_addr>=5?up_range:down_range;
 wire origin_in_range=eval_addr>=5?mag_le(held_goal,START_LIMIT):(direct_use_goal?mag_le(motion_goal,START_LIMIT):mag_le(q,START_LIMIT));
 wire endpoint_in_range=eval_op==8'h27 ? mag_le($signed(eval_value),ENDPOINT_LIMIT):eval_addr>=5?mag_le(held_goal+$signed(eval_value),ENDPOINT_LIMIT):(direct_use_goal?mag_le(motion_goal+$signed(eval_value),ENDPOINT_LIMIT):mag_le(q+$signed(eval_value),ENDPOINT_LIMIT));
 function automatic bit bank_range(input signed [31:0] delta,input [3:0] bank,input bit phased);
 begin
   // Disjoint bank groups; parallel OR avoids a serial priority result mux.
   bank_range=
     ((bank==0||bank==5)&&(phased?phase_delta_le(delta,32'sd34315):mag_le(delta,32'sd34315)))||
     ((bank==1||bank==6)&&(phased?phase_delta_le(delta,32'sd102944):mag_le(delta,32'sd102944)))||
     ((bank==2)&&(phased?phase_delta_le(delta,32'sd194449):mag_le(delta,32'sd194449)))||
     ((bank==3||bank==7)&&(phased?phase_delta_le(delta,32'sd205887):mag_le(delta,32'sd205887)))||
     ((bank==4||bank==8)&&(phased?phase_delta_le(delta,32'sd411775):mag_le(delta,32'sd411775)))||
     ((bank>8)&&(phased?phase_delta_le(delta,0):mag_le(delta,0)));
 end
 endfunction
 wire signed [31:0] home_delta=MULTITURN!=0?wrap($signed(eval_value)-q):$signed(eval_value)-q;
 wire signed [31:0] pending_origin=pending_kind>=2?held_goal:down_origin;
 wire signed [31:0] pending_absolute_delta=pending_value-pending_origin;
 wire signed [31:0] pending_angle=pending_absolute?
   (MULTITURN!=0?pending_absolute_delta+phase_correction(pending_absolute_delta):pending_absolute_delta):pending_value;
 wire signed [31:0] pending_endpoint=pending_absolute?pending_value+(MULTITURN!=0?phase_correction(pending_absolute_delta):0):pending_origin+pending_value;
 // Compute each origin's admission in parallel, select after arithmetic.
 // The existing origin/angle registers and dispatch cycle are unchanged.
 wire pending_use_goal=run&&mode==0&&motion_kind==0;
 wire signed [31:0] pending_q_delta=pending_value-q;
 wire signed [31:0] pending_goal_delta=pending_value-motion_goal;
 wire signed [31:0] pending_held_delta=pending_value-held_goal;
 wire pending_q_range=bank_range(pending_absolute?pending_q_delta:pending_value,pending_bank,pending_absolute&&MULTITURN!=0);
 wire pending_goal_range=bank_range(pending_absolute?pending_goal_delta:pending_value,pending_bank,pending_absolute&&MULTITURN!=0);
 wire pending_held_range=bank_range(pending_absolute?pending_held_delta:pending_value,pending_bank,pending_absolute&&MULTITURN!=0);
 wire pending_range=pending_kind>=2?pending_held_range:pending_use_goal?pending_goal_range:pending_q_range;
 wire pending_origin_ok=pending_kind>=2?mag_le(held_goal,START_LIMIT):
   pending_use_goal?mag_le(motion_goal,START_LIMIT):mag_le(q,START_LIMIT);
 wire pending_relative_endpoint_ok=pending_kind>=2?mag_le(held_goal+pending_value,ENDPOINT_LIMIT):
   pending_use_goal?mag_le(motion_goal+pending_value,ENDPOINT_LIMIT):mag_le(q+pending_value,ENDPOINT_LIMIT);
 wire pending_endpoint_ok=pending_absolute?mag_le(pending_endpoint,ENDPOINT_LIMIT):pending_relative_endpoint_ok;
 // Accepted absolute values are within +/-PI and origins within +/-TAU.
 // A single phase correction puts their endpoint within +/-3PI, hence
 // strictly inside the unchanged +/-64 rad endpoint bound. Home's guarded
 // delta similarly lies within +/-3PI and its wrap within +/-(PI+1).
 wire motion_link_ok=calibrated&&!fault&&!control_fault&&hb_seen&&hb_age<HEARTBEAT_CLKS&&sample_age<SAMPLE_CLKS&&!(sample&&!valid);
 wire down_qualified=down_quiet>=600&&dose_value<64'd2791728742&&(!run||mode==0&&motion_kind==0&&home_settled);
 wire servo_settling=mode==4&&motion_kind==3&&!motion_arrived;
 wire up_qualified=run&&(mode==3||mode==4)&&!servo_settling&&up_quiet>=600&&dose_value<64'd2791728742;
 // ESP32 VelocityController::begin uses <=0.5, independently of UPSTEP's
 // 0.65 threshold. Recheck both command admission and pending dispatch.
 wire servo_qualified=up_qualified&&dose_value<=64'd2147483648;

 // Single execution transaction, three extra50MHz edges. No reusable permit.
 reg [1:0] tx_phase;
 reg [7:0] tx_tag,tx_completed_tag;
 reg [1:0] tx_kind,tx_owner_kind;
 reg [2:0] tx_owner_mode;
 reg [3:0] tx_bank;
 reg tx_absolute,tx_use_goal,tx_arrived,tx_home_settled;
 reg signed [31:0] tx_value,tx_q,tx_goal,tx_held,tx_origin;
 reg [9:0] tx_down_quiet,tx_up_quiet;
 reg [63:0] tx_dose;
 reg signed [31:0] tx_dq,tx_dg,tx_dh,tx_da,tx_endpoint;
 reg tx_result;
 reg signed [31:0] tx_angle;
 wire tx_down=tx_down_quiet>=600&&tx_dose<64'd2791728742&&tx_owner_mode==0&&tx_owner_kind==0&&tx_home_settled;
 wire tx_up=(tx_owner_mode==3||tx_owner_mode==4)&&!(tx_owner_mode==4&&tx_owner_kind==3&&!tx_arrived)&&tx_up_quiet>=600&&tx_dose<64'd2791728742;
 wire tx_servo=tx_up&&tx_dose<=64'd2147483648;
 wire tx_range=tx_kind>=2?bank_range(tx_absolute?tx_dh:tx_value,tx_bank,tx_absolute&&MULTITURN!=0):
   tx_use_goal?bank_range(tx_absolute?tx_dg:tx_value,tx_bank,tx_absolute&&MULTITURN!=0):bank_range(tx_absolute?tx_dq:tx_value,tx_bank,tx_absolute&&MULTITURN!=0);
 wire tx_origin_ok=mag_le(tx_origin,START_LIMIT);
 wire tx_endpoint_ok=mag_le(tx_endpoint,ENDPOINT_LIMIT);
 wire tx_cancel=!run||fault||control_fault||!motion_link_ok||
   (cmd&&op==2)||rebase_commit||(mode!==tx_owner_mode)||(motion_kind!==tx_owner_kind);
 // One real UART producer; one slot spans the bounded execution window.
 reg [55:0] command_slot;reg command_used,command_overflow;
 reg [1:0] rx_phase;reg [55:0] rx_payload;
 reg rx_run,rx_ready,rx_down,rx_up,rx_link,rx_idle,rx_origin_ok,rx_q_ok;
 reg [2:0] rx_owner_mode;reg [1:0] rx_owner_kind;
 reg signed [31:0] rx_origin,rx_q,rx_delta,rx_endpoint,rx_angle;
 reg rx_good,rx_veto,rx_boundary,tx_admitted;
 reg rx_finish,rx_home;reg signed[31:0] rx_boundary_goal;
 wire receive_idle=tx_phase==0&&rx_phase==0;
 wire dispatch_queued=receive_idle&&command_used;
 wire dispatch_valid=receive_idle&&(command_used||cmd);
 wire [55:0] dispatch_payload=command_used?command_slot:{op,addr,seq,value};
 wire [7:0] dispatch_op=dispatch_payload[55:48];
 wire [7:0] dispatch_addr=dispatch_payload[47:40];
 wire [31:0] dispatch_value=dispatch_payload[31:0];
 wire rx_start=dispatch_valid&&(dispatch_op==8'h25||dispatch_op==8'h27);
 wire eval_queued=dispatch_queued;
 wire eval_direct=receive_idle&&!command_used;
 assign eval_cmd=(rx_phase==3)||(dispatch_valid&&!rx_start);
 wire [55:0] eval_payload=rx_phase==3?rx_payload:dispatch_payload;
 assign {eval_op,eval_addr,eval_seq,eval_value}=eval_payload;
 wire queue_push=cmd&&!eval_direct;
 wire raw_heartbeat=cmd&&op==1&&value==32'h48424154&&command_used==0&&(!seq_seen||seq==last_seq+8'd1);
 wire rx_absolute=rx_payload[55:48]==8'h27;
 wire [3:0] rx_bank=rx_payload[43:40];
 wire signed[31:0] rx_value=$signed(rx_payload[31:0]);
 wire rx_cancel=fault||control_fault||(sample&&!valid)||(cmd&&op==2)||rebase_commit||
   (mode!==rx_owner_mode)||(motion_kind!==rx_owner_kind)||
   (rx_run&&!run)||!motion_link_ok;
 always @(posedge clk)begin
   if(rst)begin command_used<=0;command_overflow<=0;end
   else begin
     if(dispatch_queued)command_used<=0;
     if(queue_push)begin
       if(!command_used||dispatch_queued)begin command_slot<={op,addr,seq,value};command_used<=1;end
       else command_overflow<=1; // Unsupported extra producer: retain prior slot.
     end
   end
 end
 reg [9:0] drop_quiet;
 reg drop_pending,drop_reverse;
 wire commissioning_mode=mode==5||mode==6;
 wire sequence_ok=!seq_seen||eval_seq==last_seq+8'd1;
 wire soft_reset_accepted=eval_cmd&&sequence_ok&&eval_op==8'h2a&&eval_addr==0&&eval_value==32'h52535431&&!run&&config_mask==0&&!(sample&&!valid);
 assign protocol_epoch=reset_epoch;
 // Informational eligibility only; the command path rechecks all conditions.
 assign protocol_eligibility={servo_settling,(!run&&config_mask==0&&!(sample&&!valid)),
   (MULTITURN!=0&&keep_session&&motion_link_ok&&servo_qualified&&!motion_pending&&!drop_pending),
   (MULTITURN!=0&&keep_session&&motion_link_ok&&run&&!motion_pending&&!drop_pending&&
    (servo_qualified||mode==0&&motion_kind==3&&!motion_end&&!finite_speed))};
 // A normal completed sample may request a rebase. Only transitions inhibit it.
 // Block the entire preparation if commands/snapshots or lease boundaries race.
 // No frame request is generated during enabled Drop. Inhibit the whole
 // continuous-session Drop phase, rather than routing Home completion
 // through the atomic commit fanout. The subsequent Home retries normally.
 assign rebase_inhibit=cmd||(command_used!=0)||(tx_phase!=0)||(rx_phase!=0)||(keep_session&&mode==7)||eval_cmd||sample||motion_pending||drop_pending||motion_resume||clear_control||cal_start||
   fault||control_fault||!calibrated||!hb_seen||hb_age>=HEARTBEAT_CLKS-1||sample_age>=SAMPLE_CLKS-1||
   (control_done&&run&&(!keep_session&&run_ticks>=(long_run?1220000:260000)||mode==0&&motion_kind!=0&&motion_finished));
 assign ready=calibrated && healthy>=QUIET_SAMPLES && hb_seen && hb_age<HEARTBEAT_CLKS && !fault && !control_fault && !(sample&&!valid) && sample_age<SAMPLE_CLKS;
 // Predecode from final next state on the SAME edge as run/fault/mode.
 // Existing RHS expressions still read old state; assignment priority unchanged.
 reg run_next,fault_next;reg [2:0] mode_next;
 always @(posedge clk) begin
   run_next=run;fault_next=fault;mode_next=mode;
   if(rst) begin rx_phase<=0;rx_veto<=0;rx_boundary<=0;tx_admitted<=0;tx_phase<=0;tx_tag<=0;tx_completed_tag<=0;finite_speed<=0;keep_session<=0;cal_start<=0;ack_valid<=0;ack_result<=0;ack_addr<=0;ack_seq<=0;ack_value<=0;run_next=0;fault_next=0;reason<=0;mode_next=0;clear_control<=0;calibrated<=0;
     offset_a<=0;offset_b<=0;gain_a<=0;gain_b<=0;iq_set<=0;arm_zero<=0;pend_down<=0;electrical_zero<=0;poles<=0;reverse<=0;
     hb_age<=HEARTBEAT_CLKS;sample_age<=0;hb_seen<=0;seq_seen<=0;last_seq<=0;config_mask<=0;healthy<=0;drop_quiet<=0;drop_pending<=0;drop_reverse<=0;long_run<=0;
     motion_kind<=0;motion_bank<=0;motion_origin<=0;motion_end<=0;motion_resume<=0;motion_pending<=0;pending_absolute<=0;pending_kind<=0;pending_bank<=0;pending_value<=0;held_goal<=0;down_quiet<=0;up_quiet<=0;run_ticks<=0;reset_epoch<=0;end
   else begin
     clear_control<=0;ack_valid<=0;cal_start<=0;
     motion_resume<=0;
     if(!run||fault||control_fault)begin keep_session<=0;motion_pending<=0;run_ticks<=0;held_goal<=0;end
     else if(control_done)begin
       // ENA1 is a live leased user session; finite ARM/LONG retain deadlines.
       if(!keep_session&&run_ticks<21'h1fffff)run_ticks<=run_ticks+1'b1;
       if(!keep_session&&run_ticks>=(long_run?1220000:260000))begin tx_phase<=0;rx_veto<=1;run_next=0;fault_next=1;reason<=8'h21;end
       if(motion_pending)begin
         if(tx_phase==0)begin
           if(motion_link_ok&&!rebase_commit&&!(cmd&&op==2))begin
             tx_admitted<=1;tx_phase<=1;tx_tag<=tx_tag+1'b1;
             tx_kind<=pending_kind;tx_bank<=pending_bank;tx_absolute<=pending_absolute;tx_use_goal<=pending_use_goal;
             tx_value<=pending_value;tx_q<=q;tx_goal<=motion_goal;tx_held<=held_goal;tx_origin<=pending_origin;
             tx_owner_mode<=mode;tx_owner_kind<=motion_kind;tx_arrived<=motion_arrived;tx_home_settled<=home_settled;
             tx_down_quiet<=down_quiet;tx_up_quiet<=up_quiet;tx_dose<=dose_value;
           end else motion_pending<=0;
         end
       end else if(rx_phase!=0&&rx_run&&!rx_boundary)begin
         rx_boundary<=1;rx_finish<=mode==0&&motion_kind!=0&&motion_finished;
         rx_home<=keep_session&&mode==7&&home_active&&home_settled;rx_boundary_goal<=motion_goal;
         if(motion_link_ok&&!rebase_commit&&!(cmd&&op==2))begin
           tx_admitted<=0;tx_phase<=1;tx_tag<=tx_tag+1'b1;motion_pending<=1;
           tx_kind<=rx_payload[47:40]<5?1:2;tx_bank<=rx_bank;tx_absolute<=rx_absolute;tx_use_goal<=pending_use_goal;
           tx_value<=rx_value;tx_q<=q;tx_goal<=motion_goal;tx_held<=held_goal;
           tx_origin<=rx_payload[47:40]>=5?held_goal:down_origin;
           tx_owner_mode<=mode;tx_owner_kind<=motion_kind;tx_arrived<=motion_arrived;tx_home_settled<=home_settled;
           tx_down_quiet<=down_quiet;tx_up_quiet<=up_quiet;tx_dose<=dose_value;
         end
       end else if(mode==0&&motion_kind!=0&&motion_finished)begin
         motion_resume<=1;
         if(motion_kind==1)begin motion_kind<=0;iq_set<=motion_goal;end
         else begin mode_next=4;held_goal<=motion_goal;iq_set<=0;end
       end else if(keep_session&&mode==7&&home_active&&home_settled)begin
         mode_next=0;motion_kind<=0;iq_set<=0;motion_resume<=1;
       end
     end
     if(tx_phase==1&&!tx_cancel)begin
       tx_phase<=2;tx_dq<=tx_value-tx_q;tx_dg<=tx_value-tx_goal;tx_dh<=tx_value-tx_held;
       tx_da<=tx_value-tx_origin;tx_endpoint<=tx_absolute?tx_value:tx_origin+tx_value;
     end
     if(tx_phase==2&&!tx_cancel)begin
       tx_phase<=3;
       tx_result<=mag_le(tx_q,START_LIMIT)&&tx_origin_ok&&
          (tx_kind==3||((tx_absolute&&MULTITURN!=0)||tx_endpoint_ok)&&tx_range)&&
          ((tx_kind==1&&tx_down)||(tx_kind==2&&tx_up)||(tx_kind==3&&tx_servo));
       tx_angle<=tx_kind==3?tx_value:tx_absolute?(MULTITURN!=0?tx_da+phase_correction(tx_da):tx_da):tx_value;
     end
     if(tx_phase==3&&!tx_cancel)begin
       tx_phase<=0;motion_pending<=0;tx_completed_tag<=tx_tag;
       if(tx_result&&tx_admitted)begin mode_next=0;motion_kind<=tx_kind;motion_bank<=tx_bank;motion_origin<=tx_origin;iq_set<=tx_angle;motion_end<=0;end
     end
     if(tx_phase!=0&&(tx_cancel!==1'b0))begin tx_phase<=0;motion_pending<=0;end
     if(!motion_link_ok)begin down_quiet<=0;up_quiet<=0;end
     else if(control_done)begin
       if(down_le(alpha,800)&&mag_le(w,9830)&&mag_le(v,13107))down_quiet<=down_quiet<600?down_quiet+1'b1:down_quiet;else down_quiet<=0;
       if(run&&(mode==3||mode==4)&&mag_le(q-held_goal,2287)&&upright_le(alpha,1143)&&mag_le(w,19661)&&mag_le(v,19661))
         up_quiet<=up_quiet<600?up_quiet+1'b1:up_quiet;else up_quiet<=0;
     end
     if(!run||fault||control_fault)long_run<=0;
     // Switch controllers only at a completed current-loop boundary.
     if(!run||fault||control_fault)drop_pending<=0;
     else if(drop_pending&&control_done)begin
       drop_pending<=0;
       if(!servo_settling&&drop_quiet>=600&&drop_dose_ok&&!(sample&&!valid))begin mode_next=7;iq_set<=drop_reverse?-65536:65536;end
     end
     if(!run||fault||servo_settling||(mode!=3&&mode!=4)||(sample&&!valid))drop_quiet<=0;
     else if(control_done)begin
       if(mag_lt(q,1966)&&upright_lt(alpha,1966)&&mag_lt(w,19661)&&mag_lt(v,19661))
         drop_quiet<=drop_quiet<600?drop_quiet+1'b1:drop_quiet;
       else drop_quiet<=0;
     end
     if(hb_age<HEARTBEAT_CLKS) hb_age<=hb_age+1'b1;
     if(sample_age<SAMPLE_CLKS) sample_age<=sample_age+1'b1;
     if(sample) begin
       sample_age<=0;
       if(!valid) begin healthy<=0;if(run) begin run_next=0;fault_next=1;reason<=sample_error;end end
     end
     if(control_done&&!run) begin
       if(calibrated && mag_lt(ia,3277) && mag_lt(ib,3277) && mag_lt(q,MULTITURN!=0?TAU+1:196608) && mag_lt(w,9830) && mag_lt(v,19661))
         healthy<=healthy<QUIET_SAMPLES ? healthy+1'b1:healthy;
       else healthy<=0;
     end
     if(rx_start)begin
       rx_phase<=1;rx_payload<=dispatch_payload;rx_veto<=0;rx_boundary<=0;
       rx_run<=run;rx_ready<=ready;rx_down<=down_qualified;rx_up<=up_qualified;rx_link<=motion_link_ok;
       rx_idle<=!motion_pending&&!drop_pending;rx_owner_mode<=mode;rx_owner_kind<=motion_kind;
       rx_origin<=dispatch_addr>=5?held_goal:down_origin;rx_q<=q;
       rx_origin_ok<=mag_le(dispatch_addr>=5?held_goal:down_origin,START_LIMIT);rx_q_ok<=mag_le(q,START_LIMIT);
     end
     if(rx_phase!=0&&(rx_cancel!==1'b0))rx_veto<=1;
     if(rx_phase==1)begin
       rx_phase<=2;rx_delta<=rx_value-rx_origin;rx_endpoint<=rx_origin+rx_value;
     end
     if(rx_phase==2)begin
       rx_phase<=3;
       rx_good<=rx_link&&rx_idle&&rx_payload[47:40]<9&&
         (rx_absolute?(MULTITURN!=0?(rx_delta!=0&&rx_delta!=TAU&&rx_delta!= -TAU):rx_delta!=0):rx_value!=0)&&
         (MULTITURN==0||!rx_absolute||mag_le(rx_value,PI))&&
         bank_range(rx_absolute?rx_delta:rx_value,rx_bank,rx_absolute&&MULTITURN!=0)&&rx_q_ok&&rx_origin_ok&&
         (rx_absolute&&MULTITURN!=0||mag_le(rx_absolute?rx_value:rx_endpoint,ENDPOINT_LIMIT))&&
         ((rx_payload[47:40]<5&&rx_down&&(rx_run||rx_ready))||(rx_payload[47:40]>=5&&rx_up));
       rx_angle<=rx_absolute?(MULTITURN!=0?rx_delta+phase_correction(rx_delta):rx_delta):rx_value;
     end
     if(rx_phase==3)begin
       rx_phase<=0;
       if(rx_veto||rx_cancel||!rx_good||!sequence_ok)begin
         if(rx_boundary||control_done)begin
           tx_phase<=0;motion_pending<=0;
           if(run&&!fault&&!control_fault&&!rx_veto&&!rx_cancel)begin
             if(rx_boundary?rx_finish:(mode==0&&motion_kind!=0&&motion_finished))begin
               motion_resume<=1;
               if(rx_owner_kind==1)begin motion_kind<=0;iq_set<=rx_boundary?rx_boundary_goal:motion_goal;end
               else begin mode_next=4;held_goal<=rx_boundary?rx_boundary_goal:motion_goal;iq_set<=0;end
             end else if(rx_boundary?rx_home:(keep_session&&mode==7&&home_active&&home_settled))begin mode_next=0;motion_kind<=0;iq_set<=0;motion_resume<=1;end
           end
         end
       end
     end
     if(eval_cmd) begin
       ack_valid<=1;ack_result<=8'h82;ack_addr<=eval_addr;ack_seq<=eval_seq;ack_value<=1;
      if(sequence_ok || eval_op==2 || (eval_op==4&&!run&&eval_value==32'h53594e43)) begin
       ack_value<=2;
       if(sequence_ok) begin seq_seen<=1;last_seq<=eval_seq;end
       case(eval_op)
         1: if(eval_value==32'h48424154) begin if(!eval_queued)begin hb_seen<=1;hb_age<=0;end ack_result<=8'h81;ack_value<=eval_value;end
         2: begin ack_result<=8'h81;ack_value<=0;end
         4: if(!run&&eval_value==32'h53594e43) begin seq_seen<=1;last_seq<=eval_seq;ack_result<=8'h81;ack_value<=eval_value;end
         3: if(!run && eval_value==32'h434c4541 && !(sample&&!valid)) begin fault_next=0;reason<=0;healthy<=0;clear_control<=1;ack_result<=8'h81;ack_value<=eval_value;end
         8'h12: if(eval_addr<36) begin
           ack_result<=8'h81;
           case(eval_addr)
             0:ack_value<=offset_a;1:ack_value<=offset_b;2:ack_value<=gain_a;3:ack_value<=gain_b;
             4:ack_value<={18'b0,arm_zero};5:ack_value<={20'b0,pend_down};6:ack_value<={16'b0,electrical_zero};
             7:ack_value<={23'b0,reverse,2'b0,poles};
             8:ack_value<=32'h43414c31;9:ack_value<=cal_status;10:ack_value<=cal_a;11:ack_value<=cal_b;
             12:ack_value<={16'b0,cal_zero};13:ack_value<={31'b0,cal_reverse};
             14:ack_value<=32'h54524331; // TRC1: last commissioning sample, frozen on exit
             15:ack_value<=cal_a;16:ack_value<=cal_b;
             17:ack_value<=32'h44525031; // DRP1: controlled drop and 130 s hold limit
             18:ack_value<={29'd0,drop_pending,drop_dose_ok,(drop_quiet>=600)};
             19:ack_value<=32'h4c4f4e31; // LON1: explicit 610 s maximum + original rolling dose
             20:ack_value<=dose_value[47:16]; // unsigned Q16 sum-of-phases A^2 s
             21:ack_value<={31'd0,long_run};
             22:ack_value<=32'h484f4d31; // HOM1: terminal home handoff
             23:ack_value<=run&&!fault&&mode==7 ? {30'd0,home_settled,home_active}:0;
             24:ack_value<=32'h484f4d32; // HOM2: bounded shortest-path home / down-position move
             25:ack_value<=run&&!fault&&mode==0&&motion_kind==0 ? {30'd0,home_settled,home_active}:0;
             26:ack_value<=MULTITURN!=0?32'h4d4f5432:32'h4d4f5431; // MOT2: phase targets and atomic frame telemetry
             27:ack_value<=run&&!fault ? {24'd0,motion_pending,motion_end,motion_arrived,motion_finished,(mode==0&&motion_kind!=0),1'b0,motion_kind}:0;
             28:ack_value<=motion_goal;
             29:ack_value<=MULTITURN!=0?32'h454e4131:0; // ENA1 continuous enabled session
             30:ack_value<={31'd0,keep_session};
             31:ack_value<=32'h44543230; // DT20: separate 20 s terminal Drop Home lease
             32:ack_value<=MULTITURN!=0?32'h53563237:0; // SV27: leased continuous velocity owner
             33:ack_value<=32'h52535431; // RST1: explicit stopped control-domain reset
             34:ack_value<=reset_epoch;
             35:ack_value<=MULTITURN!=0?32'h53504431:0; // SPD1: thirty bounded speed recipes
           endcase
         end
         8'h2a: if(soft_reset_accepted)begin tx_phase<=0;rx_veto<=1;
           finite_speed<=0;
           // Preserve the committed calibration bank and its physical zeros.
           // Reset sensor/control engines, leases, qualification and motion;
           // UART/ACK remain alive so host can verify the epoch and resynchronize.
           run_next=0;fault_next=0;reason<=0;mode_next=0;iq_set<=0;clear_control<=1;
           keep_session<=0;long_run<=0;healthy<=0;down_quiet<=0;up_quiet<=0;drop_quiet<=0;
           hb_seen<=0;hb_age<=HEARTBEAT_CLKS;sample_age<=SAMPLE_CLKS;run_ticks<=0;
           seq_seen<=0;last_seq<=0;drop_pending<=0;drop_reverse<=0;cal_start<=0;
           motion_kind<=0;motion_bank<=0;motion_origin<=0;motion_end<=0;motion_resume<=0;
           motion_pending<=0;pending_absolute<=0;pending_kind<=0;pending_bank<=0;pending_value<=0;held_goal<=0;
           reset_epoch<=reset_epoch+1'b1;ack_result<=8'h81;ack_value<=eval_value;
         end
         8'h24: if(!run&&ready&&(eval_addr==0||MULTITURN!=0&&eval_addr==1)&&mag_le($signed(eval_value),MULTITURN!=0?PI:183501)&&mag_le(q,MULTITURN!=0?TAU:183501)&&
                      (MULTITURN!=0||home_delta>=-PI&&home_delta<PI)&&down_le(alpha,6553)&&mag_le(w,9830)&&mag_le(v,19661))begin
           mode_next=0;motion_kind<=0;motion_end<=0;iq_set<=$signed(eval_value);run_next=1;long_run<=eval_addr==1;keep_session<=eval_addr==1;healthy<=0;ack_result<=8'h81;ack_value<=eval_value;
         end
         8'h29: if(MULTITURN!=0&&keep_session&&motion_link_ok&&run&&mode==0&&motion_kind==0&&home_settled&&
              eval_addr==3&&eval_value==32'h53574731&&mag_lt(q,1966)&&down_lt(alpha,800)&&mag_lt(w,9830)&&mag_lt(v,13107)&&
              dose_value<64'd2791728742&&!motion_pending&&!drop_pending)begin
           mode_next=3;iq_set<=0;held_goal<=0;ack_result<=8'h81;ack_value<=eval_value;
         end
         8'h25,8'h27: if(rx_phase==3&&rx_good&&!rx_veto&&!rx_cancel)begin
           if(rx_run)begin
             if(rx_boundary||control_done)tx_admitted<=1;
             else begin motion_pending<=1;pending_kind<=eval_addr<5?1:2;pending_bank<=eval_addr[3:0];pending_value<=eval_value;pending_absolute<=eval_op==8'h27;end
           end else begin mode_next=0;motion_kind<=1;motion_bank<=eval_addr[3:0];motion_origin<=rx_q;iq_set<=rx_angle;motion_end<=0;run_next=1;long_run<=0;healthy<=0;end
           ack_result<=8'h81;ack_value<=eval_value;
         end
         8'h2b: if(MULTITURN!=0&&keep_session&&motion_link_ok&&run&&!motion_pending&&!drop_pending&&
             servo_qualified&&eval_addr==0&&speed_recipe_valid($signed(eval_value)))begin
           motion_pending<=1;pending_kind<=3;pending_bank<=0;pending_value<=eval_value;pending_absolute<=0;
           finite_speed<=1;ack_result<=8'h81;ack_value<=eval_value;
         end
         8'h26: if(motion_link_ok&&run&&!motion_pending&&!drop_pending&&!(finite_speed&&mode==0&&motion_kind==3))begin
           if(eval_addr==1&&eval_value==0&&mode==0&&motion_kind==3)begin motion_end<=1;ack_result<=8'h81;ack_value<=eval_value;end
           else if(eval_addr==0&&mag_le($signed(eval_value),200)&&
               (servo_qualified||mode==0&&motion_kind==3&&!motion_end))begin
             if(mode==0&&motion_kind==3)iq_set<=eval_value;
             else begin motion_pending<=1;pending_kind<=3;pending_bank<=0;pending_value<=eval_value;pending_absolute<=0;finite_speed<=0;end
             ack_result<=8'h81;ack_value<=eval_value;
           end
         end
         8'h10: if(!run && eval_addr<8 &&
           ((eval_addr<2 && eval_value>=33554432 && eval_value<=234881024) ||
            ((eval_addr==2||eval_addr==3) && absq($signed(eval_value))>=32 && mag_le($signed(eval_value),256)) ||
            (eval_addr==4&&eval_value<16384)||(eval_addr==5&&eval_value<4096)||(eval_addr==6&&eval_value<65536)||
            (eval_addr==7&&(eval_value&32'hfffffec0)==0&&eval_value[5:0]>=1&&eval_value[5:0]<=32))) begin
           ack_result<=8'h81;ack_value<=eval_value;
           calibrated<=0;healthy<=0;config_mask[eval_addr]<=1;
           case(eval_addr)
            0: offset_a<=eval_value;1:offset_b<=eval_value;2:gain_a<=eval_value;3:gain_b<=eval_value;
            4:arm_zero<=eval_value[13:0];5:pend_down<=eval_value[11:0];6:electrical_zero<=eval_value[15:0];
            7:begin poles<=eval_value[5:0];reverse<=eval_value[8];end
           endcase
         end
         8'h11: if(!run && eval_value==32'hc011ca1b && (&config_mask) &&
           offset_a>=33554432 && offset_a<=234881024 && offset_b>=33554432 && offset_b<=234881024 &&
           absq(gain_a)>=32 && mag_le(gain_a,256) && absq(gain_b)>=32 && mag_le(gain_b,256) && poles>=1&&poles<=32) begin
             calibrated<=1;clear_control<=1;healthy<=0;config_mask<=0;ack_result<=8'h81;ack_value<=eval_value;
           end
         8'h21: if(!run && ready && (eval_addr==5||eval_addr==6) && eval_value==32'h43414c31 &&
            mag_lt(alpha,9830) && mag_lt(w,9830) && mag_lt(v,19661) && (eval_addr==5||poles==11)) begin
             mode_next=eval_addr[2:0];run_next=1;long_run<=0;healthy<=0;cal_start<=1;ack_result<=8'h81;ack_value<=eval_value;
           end
         8'h22: if(run&&!fault&&!control_fault&&!drop_pending&&!servo_settling&&calibrated&&(mode==3||mode==4)&&eval_addr==7&&
             (eval_value==32'h44525031||eval_value==32'h4452504e)&&drop_quiet>=600&&drop_dose_ok&&
             hb_seen&&hb_age<HEARTBEAT_CLKS&&sample_age<SAMPLE_CLKS&&!(sample&&!valid))begin
               drop_pending<=1;drop_reverse<=eval_value==32'h4452504e;ack_result<=8'h81;ack_value<=eval_value;
             end
         8'h20: if(!run && ready && eval_addr>=1&&eval_addr<=4 &&
              ((eval_addr==1&&mag_le($signed(eval_value),5243)) ||
               (eval_addr==2&&mag_lt(alpha,9830)) ||
               (eval_addr==3&&mag_lt(q,3277)&&down_lt(alpha,2621)&&mag_lt(w,19661)&&mag_lt(v,19661)) ||
               (eval_addr==4&&upright_lt(alpha,9830)))) begin
             mode_next=eval_addr[2:0];iq_set<=eval_value;run_next=1;long_run<=0;healthy<=0;ack_result<=8'h81;ack_value<=eval_value;
           end
         8'h23: if(!run&&ready&&eval_addr==3&&eval_value==32'h4c4f4e31&&
             mag_lt(q,3277)&&down_lt(alpha,2621)&&mag_lt(w,19661)&&mag_lt(v,19661))begin
             mode_next=3;iq_set<=0;run_next=1;long_run<=1;healthy<=0;ack_result<=8'h81;ack_value<=eval_value;
           end
       endcase
      end
     end
     if(cmd&&op==2)begin rx_veto<=1;tx_phase<=0;finite_speed<=0;keep_session<=0;run_next=0;healthy<=0;drop_pending<=0;motion_pending<=0;motion_end<=0;motion_kind<=0;long_run<=0;end
     if(raw_heartbeat)begin hb_seen<=1;hb_age<=0;end
     if(run && commissioning_mode && cal_finished) begin
       tx_phase<=0;rx_veto<=1;run_next=0;healthy<=0;
       if(cal_failed)begin fault_next=1;reason<=cal_error;end
     end
     if(run && commissioning_mode && (absq(alpha)>22938 || absq(v)>131072 || absq(w)>131072 || absq(q)>196608)) begin
       tx_phase<=0;rx_veto<=1;run_next=0;fault_next=1;healthy<=0;reason<=8'h42;
     end
     // Fault checks override arm/heartbeat on the same clock. No auto-restart.
     if(run && (hb_age>=HEARTBEAT_CLKS || sample_age>=SAMPLE_CLKS || control_fault)) begin
       tx_phase<=0;rx_veto<=1;run_next=0;fault_next=1;healthy<=0;reason<=control_fault ? control_reason:hb_age>=HEARTBEAT_CLKS ? 8'h31:8'h32;
     end
     if(sample&&!valid) healthy<=0;
     // Invalid acquisition wins simultaneous new control/watchdog failures,
     // matching the reference's sensor validation before control execution.
     if(run&&sample&&!valid)begin tx_phase<=0;rx_veto<=1;run_next=0;fault_next=1;reason<=sample_error;healthy<=0;end
     // STOP and later faults preserve the original reason. Only a valid,
     // sequence-qualified explicit CLEAR while stopped may clear this latch.
     if(fault&&!soft_reset_accepted&&!(eval_cmd&&sequence_ok&&eval_op==3&&!run&&eval_value==32'h434c4541&&!(sample&&!valid)))begin
       fault_next=1;reason<=reason;
     end
     // A qualified commit excludes all command/transition writes. Put its
     // coordinate mux last so commit does not traverse the command priority chain.
     if(rebase_commit===1'b1 && !rebase_inhibit && run)begin
       held_goal<=held_goal-rebase_shift;motion_origin<=motion_origin-rebase_shift;
       if(mode==0&&motion_kind==0)iq_set<=iq_set-rebase_shift;
     end
   end
   if(!run||fault||control_fault)tx_phase<=0;
   run<=run_next;fault<=fault_next;mode<=mode_next;
   core_enable<=run_next&&!fault_next&&mode_next!=5&&mode_next!=6;
 end
endmodule
