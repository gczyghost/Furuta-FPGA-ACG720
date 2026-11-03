// Sensor-to-duty pipeline. No peripheral pin access. Reset is explicit;
// invalid/stale snapshots must be rejected by the supervisor before start.
module motor_control #(parameter integer MULTITURN=0, EXTERNAL_SAMPLE_INHIBIT=0)(input wire clk,rst,start,enable,input wire [2:0] mode,
 input wire [13:0] arm_raw,arm_zero,input wire [11:0] pend_raw,pend_down,ia_raw,ib_raw,
 input wire signed [31:0] offset_a,offset_b,gain_a,gain_b,iq_set,
 input wire [5:0] poles,input wire reverse,input wire [15:0] electrical_zero,
 output reg busy,done,fault,output reg [7:0] reason,
 output wire [15:0] du,dv,dw,
 output reg signed [31:0] ia,ib,output wire signed [31:0] id,iq,ud,uq,iq_ref,
 output wire signed [31:0] q,alpha,w,v,output wire drop_dose_ok,input wire long_run,output wire [63:0] dose_value,output wire home_active,home_settled,
 input wire [1:0] motion_kind,input wire [3:0] motion_bank,input wire signed [31:0] motion_origin,
 input wire motion_end,motion_resume,output wire motion_finished,motion_arrived,output wire signed [31:0] motion_goal, input wire rebase_inhibit, output wire rebase_commit, output wire signed [31:0] rebase_shift, output reg signed [31:0] frame_turns,input wire keep_session,input wire finite_speed,output wire signed [31:0] telemetry_cap,telemetry_reference_q,telemetry_reference_w,output wire telemetry_reference_valid);
 import fixed_math::*;
 reg signed [31:0] frame_distance;
 reg signed [31:0] phase_input,phase_signed,upright_phase;
 reg [31:0] phase_mag,phase_multiple,phase_rem;
 reg [45:0] phase_product /* synthesis syn_dspstyle = "dsp" */;
 reg phase_sign;
 reg [4:0] state;reg [13:0] previous;reg initialized;
 reg signed [31:0] turn_counts,position,pend_angle;
 reg signed [11:0] pend_count;
 reg signed [31:0] pend_product;
 reg signed [13:0] step_delta; reg obs_start,outer_start,foc_start;
 wire arm_done,pend_done,outer_done,outer_fault,outer_busy,foc_done,foc_busy;
 wire signed [31:0] drop_cap;
 wire [7:0] outer_reason;wire signed [31:0] ar,cap;
 reg [15:0] phase;wire [63:0] dose;wire dose_done,dose_fault;reg [63:0] mean_square;reg [63:0] sum_square;reg [5:0] sustained;
 wire [15:0] calculated_phase;
 commutation_phase phase_calc(arm_raw,poles,reverse,electrical_zero,calculated_phase);
 reg signed [31:0] peak;reg [13:0] raw_latched;
 wire signed [14:0] crossing=$signed({1'b0,arm_raw})-$signed({1'b0,previous});
 wire signed [11:0] pend_diff=pend_raw-pend_down;
 wire signed [13:0] initial_diff=arm_raw-arm_zero;
 wire signed [31:0] phase_c=add(ia,ib);
 reg div_start,subtract_rms,obs_ready;reg [63:0] numerator;reg [31:0] denominator;
 reg signed [31:0] count_position,cphase;reg signed [63:0] position_product;
 reg [63:0] sa,sb,sc,partial_sum;
 wire div_done,div_busy;wire [63:0] quotient;
 wire [63:0] updated_mean_square=subtract_rms?mean_square-quotient:mean_square+quotient;
 udiv64 protection_div(clk,rst,div_start,numerator,denominator,div_busy,div_done,quotient);
 wire slow_observer,drop_slow,normal_done,normal_fault,drop_done,drop_fault;
 reg previous_enable,observer_restart_pending;reg [2:0] previous_mode;
 reg signed [31:0] previous_command;
 reg [63:0] previous_voltage_square;
 wire session_entry=enable&&(!previous_enable||(mode==3&&previous_mode!=3&&previous_mode!=4));
 wire drop_entry=enable&&mode==7&&(!previous_enable||previous_mode!=7);
 wire voltage_saturated=previous_voltage_square>64'd706438905200;
 // The ESP32 resets observer initialization at begin/sessionTransition,
 // but not at DROP or fast-motion handoff. Consume only at a sensor start,
 // preserving arm unwrap, control PI, rolling dose and all fault latches.
 always @(posedge clk)begin
   if(rst)begin previous_enable<=0;previous_mode<=0;previous_command<=0;observer_restart_pending<=0;previous_voltage_square<=0;end
   else begin
     previous_enable<=enable;previous_mode<=mode;
     if(enable)previous_command<=iq_ref;
     previous_voltage_square<=64'($signed(ud))*64'($signed(ud))+64'($signed(uq))*64'($signed(uq));
     if(obs_start)observer_restart_pending<=0;
     if(session_entry)observer_restart_pending<=1;
   end
 end
 wire drop_home_active,drop_home_settled,move_done,move_settled;
 wire signed [31:0] move_command,move_goal,move_reference,plan_reference;
 wire move_busy,plan_busy,servo_busy,move_rebase_ack,servo_rebase_ack;
 reg move_initialized,move_fault,move_arrived;reg [15:0] move_ticks;
 // Initial-arrival deadline is not a cumulative lifetime unsettled budget.
 // Live move_settled still controls eligibility and telemetry independently.
 wire home_deadline_exempt=(keep_session===1'b1)&&(move_settled||move_arrived);
 reg signed [31:0] move_slewed;
 wire is_home=mode==0&&motion_kind==0;
 wire is_plan=mode==0&&(motion_kind==1||motion_kind==2);
 wire is_servo=mode==0&&motion_kind==3;
 // v27 mode-specific protection persists through active -> settling -> hold.
 // The 14 rad/s limit remains unchanged outside this explicit owner.
 wire servo_guard=(MULTITURN!=0)&&enable&&(is_servo||(mode==4&&motion_kind==3&&!motion_arrived));
 wire signed [31:0] servo_reference_w;
 reg signed [63:0] servo_target_product,servo_margin_product /* synthesis syn_dspstyle = "logic" */;
 reg signed [31:0] servo_guard_base,servo_speed_limit;
 wire signed [31:0] servo_target_w=servo_target_product>>>16;
 always @(posedge clk)begin
   if(rst||!servo_guard)begin servo_target_product<=0;servo_guard_base<=0;servo_margin_product<=0;servo_speed_limit<=65536;end
   else begin
     servo_target_product<=is_servo&&!motion_end?$signed(iq_set[15:0])*64'sd449767923:64'sd0;
     servo_guard_base<=max_abs(servo_reference_w,servo_target_w);
     // ceil(1.1 * Q16 value) plus ceil(0.3 * 65536). At 200rpm
     // coefficient quantization adds <0.0002rad/s, never a global relaxation.
     servo_margin_product<=$signed(servo_guard_base)*64'sd72090+64'sd65535;
     servo_speed_limit<=(servo_margin_product>>>16)+19661>65536?(servo_margin_product>>>16)+19661:65536;
   end
 end
 wire plan_done,plan_fault,plan_finished,plan_ended,servo_done,servo_fault,servo_finished;
 wire signed [31:0] plan_command,plan_goal,plan_integral,servo_command,servo_goal,servo_integral,normal_integral;
 wire [10:0] plan_settled;
 reg stage_entered,stage_initialized,home_terminal,arrival_active,arrival_begin;
 reg [1:0] arrival_kind;
 reg [15:0] arrival_ticks;
 reg signed [31:0] stage_slewed,last_motion_goal,terminal_integral,resume_integral,resume_command,hold_goal;
 reg [10:0] terminal_settled;
 reg drop_seed_valid;reg signed [63:0] precise_terminal;
 wire signed [63:0] drop_precise_integral,move_precise_integral;
 wire [10:0] drop_settled_state,move_settled_state;
 wire stage_done=is_servo?servo_done:plan_done;
 wire signed [31:0] stage_command=is_servo?servo_command:plan_command;
 wire signed [31:0] stage_limit=is_plan&&motion_kind==2&&!plan_ended?104858:65536;
 reg signed [31:0] stage_delta,stage_step,stage_sum,move_delta,move_step,move_sum;
 reg [2:0] stage_valid,move_valid;
 wire stage_ready=stage_valid[2],move_ready=move_valid[2];
 wire signed [31:0] stage_next=clip(stage_sum,-stage_limit,stage_limit);
 // Final command filtering completes before FOC starts. Preserve the original
 // signed 32-bit operations and limits, adding only system-clock pipeline stages.
 always @(posedge clk)begin
   if(rst||!enable)begin stage_valid<=0;move_valid<=0;end
   else begin
     stage_valid<={stage_valid[1:0],stage_done&&(is_plan||is_servo)};
     move_valid<={move_valid[1:0],move_done&&is_home};
     stage_delta<=stage_command-stage_slewed;stage_step<=clip(stage_delta,-3277,3277);stage_sum<=stage_slewed+stage_step;
     move_delta<=move_command-move_slewed;move_step<=clip(move_delta,-3277,3277);move_sum<=move_slewed+move_step;
   end
 end
 assign motion_finished=is_plan?plan_finished:is_servo?servo_finished:1'b0;
 assign motion_goal=last_motion_goal;
 wire [7:0] normal_reason,drop_reason;wire signed [31:0] normal_command,drop_command,drop_ref;
 assign drop_dose_ok=dose<64'd2147483648;
 assign dose_value=dose;
 current_dose dose_counter(clk,rst,enable,long_run,state==8&&div_done,quotient,dose_done,dose_fault,dose);
 wire arm_idle,arm_rebase_ack,rebase_busy,rebase_complete,rebase_cancelled,rebase_rejected;
 wire signed [31:0] rebase_counts,rebase_turns;
 reg rebase_previous,rebase_home_previous,rebase_servo_previous;
 reg [2:0] rebase_mode;reg [1:0] rebase_kind;reg rebase_enabled;
 // Board integration inhibits on every raw snapshot, a superset of start.
 // Omit its redundant freshness-dependent path only for that explicit wiring.
 wire rebase_sample_start=EXTERNAL_SAMPLE_INHIBIT!=0 ? 1'b0:start;
 wire frame_idle=!busy&&arm_idle&&!obs_start&&!outer_start&&!foc_start&&!rebase_sample_start&&
   !foc_busy&&!outer_busy&&!move_busy&&!plan_busy&&!servo_busy;
 wire signed [31:0] frame_goal=!enable?q:is_home?move_goal:is_servo?servo_goal:last_motion_goal;
 wire frame_request=done&&(frame_goal>205952||frame_goal< -205952)&&
   (!enable||is_home&&move_settled||mode==4&&motion_arrived||
    is_servo&&(servo_goal>3276800||servo_goal< -3276800));
 wire frame_transition=rebase_busy&&(mode!=rebase_mode||motion_kind!=rebase_kind||enable!=rebase_enabled);
 wire signed [31:0] frame_reference=is_home?(move_initialized?move_reference:q):
   is_plan?(stage_initialized?plan_reference:motion_origin):
   is_servo?(stage_initialized?servo_goal:motion_origin):mode==4?hold_goal:32'sd0;
 always @(posedge clk)begin
   if(rst)begin rebase_mode<=0;rebase_kind<=0;rebase_enabled<=0;end
   else if(frame_request&&frame_idle&&!rebase_busy)begin rebase_mode<=mode;rebase_kind<=motion_kind;rebase_enabled<=enable;end
 end
 // The board owner must inhibit pending commands/transitions and any snapshot.
 // A changed mode/run state also cancels preparation inside the core.
 rebase_transaction frame_prepare(clk,rst,(MULTITURN!=0)&&!fault&&mode!=5&&mode!=6,1'b0,fault,
   frame_request,frame_idle,rebase_sample_start,(rebase_inhibit===1'b1)||frame_transition,reverse,frame_goal,
   rebase_busy,rebase_commit,rebase_complete,rebase_cancelled,rebase_rejected,rebase_shift,rebase_counts,rebase_turns);
 angle_observer arm_obs(clk,rst,obs_start,position,arm_done,q,w,slow_observer,rebase_commit,rebase_shift,arm_idle,arm_rebase_ack,observer_restart_pending);
 angle_observer #(.PERIODIC_POSITION(1)) pend_obs(clk,rst,obs_start,pend_angle,pend_done,alpha,v,slow_observer,1'b0,32'sd0,,,observer_restart_pending);
 pendulum_control outer(clk,rst,outer_start&&mode!=7&&mode!=0,
   enable&&(mode!=7&&mode!=0||mode==0&&motion_kind>=2),mode==0?3'd4:mode,q-hold_goal,alpha,w,v,iq,iq_set,
   outer_busy,normal_done,normal_fault,normal_reason,normal_command,ar,cap,long_run,motion_resume&&mode==4,resume_integral,resume_command,normal_integral,voltage_saturated,session_entry,previous_command,keep_session);
 controlled_drop dropper(clk,rst,outer_start&&mode==7,enable&&mode==7,iq_set[31],q,alpha,w,v,drop_done,drop_fault,drop_reason,drop_command,drop_ref,drop_slow,drop_home_active,drop_home_settled,drop_precise_integral,drop_settled_state,drop_entry,previous_command,drop_cap);
 home_controller mover(clk,rst,enable&&is_home,outer_start&&is_home,!move_initialized,home_terminal,
   q,alpha,w,v,iq_set,terminal_integral,terminal_settled,move_busy,move_done,move_command,move_reference,,,move_settled,1'b1,move_goal,rebase_commit&&enable&&is_home,rebase_shift,move_rebase_ack,
   move_precise_integral,drop_seed_valid&&home_terminal,precise_terminal,move_settled_state);
 planned_motion planner(clk,rst,enable&&is_plan,outer_start&&is_plan,!stage_initialized,motion_bank,iq_set,motion_origin,
   motion_kind==2?-normal_integral:32'sd0,q,motion_kind==2?upright_phase:alpha,w,v,
   plan_busy,plan_done,plan_fault,plan_finished,plan_command,plan_goal,plan_reference,,plan_integral,plan_ended,plan_settled);
 velocity_servo #(.OWNER_IDLE_REBASE(1)) servo(clk,rst,enable&&is_servo,outer_start&&is_servo,!stage_initialized,motion_end,$signed(iq_set[15:0]),motion_origin,
   -normal_integral,q,upright_phase,w,v,servo_busy,servo_done,servo_fault,servo_finished,servo_command,servo_goal,,servo_reference_w,,servo_integral,rebase_commit&&enable&&is_servo,rebase_shift,servo_rebase_ack,finite_speed);
 upright_arrival arrival(clk,rst||!enable,arrival_begin,enable&&arrival_active&&(arrival_kind==2||mode==4),outer_start,
   mode==4||is_plan&&plan_ended,q-last_motion_goal,upright_phase,w,motion_arrived,,);
 assign telemetry_cap=!enable||fault ? 0:mode==7 ? drop_cap:mode==0 ? (is_home ? 65536:stage_limit):cap;
 assign telemetry_reference_valid=enable&&!fault&&(is_servo||mode==4&&motion_kind==3&&!motion_arrived);
 assign telemetry_reference_q=is_servo ? servo_goal:last_motion_goal;
 assign telemetry_reference_w=is_servo ? servo_reference_w:0;
 assign slow_observer=(enable&&mode==0&&motion_kind<2)||drop_slow;
 assign home_active=is_home?enable&&move_initialized:drop_home_active;
 // Each source already includes its mutually exclusive Home/Drop enable.
 reg settled_stage;
 always @(posedge clk)begin
   if(rst||!enable)settled_stage<=0;
   else settled_stage<=move_settled||drop_home_settled;
 end
 assign home_settled=enable&&(is_home||mode==7)&&settled_stage;
 wire is_normal=mode!=0&&mode!=7;
 // Decode mutually exclusive owners in parallel. In particular, do not
 // cascade a mode-selected done mux into a second fault/reason mux.
 assign outer_done=(is_home&&move_ready)||((is_plan||is_servo)&&stage_ready)||
                   (mode==7&&drop_done)||(is_normal&&normal_done);
 wire completed_home_fault=is_home&&(move_ready||!enable)&&move_fault;
 wire completed_plan_fault=is_plan&&(stage_ready||!enable)&&plan_fault;
 wire completed_servo_fault=is_servo&&(stage_ready||!enable)&&servo_fault;
 wire completed_drop_fault=mode==7&&(drop_done||!enable)&&drop_fault;
 wire completed_normal_fault=is_normal&&(normal_done||!enable)&&normal_fault;
 wire completed_outer_fault=completed_home_fault||completed_plan_fault||completed_servo_fault||completed_drop_fault||completed_normal_fault;
 wire [7:0] completed_outer_reason=({8{completed_home_fault}}&8'h21)|
   ({8{completed_plan_fault||completed_servo_fault}}&8'h24)|
   ({8{completed_drop_fault}}&drop_reason)|({8{completed_normal_fault}}&normal_reason);
 assign outer_fault=is_home?move_fault:is_plan?plan_fault:is_servo?servo_fault:mode==7?drop_fault:normal_fault;
 assign outer_reason=is_home?8'h21:is_plan||is_servo?8'h24:mode==7?drop_reason:normal_reason;
 assign iq_ref=is_home?(drop_seed_valid&&!move_initialized?resume_command:move_slewed):is_plan||is_servo?stage_slewed:mode==7?drop_command:normal_command;
 always @(posedge clk)begin
   if(rst||!enable)begin
     drop_seed_valid<=0;precise_terminal<=0;
     stage_entered<=0;stage_initialized<=0;stage_slewed<=0;last_motion_goal<=0;home_terminal<=0;terminal_integral<=0;terminal_settled<=0;
     resume_integral<=0;resume_command<=0;hold_goal<=0;arrival_active<=0;arrival_begin<=0;arrival_kind<=0;arrival_ticks<=0;
   end else begin
     arrival_begin<=0;
     if(outer_start&&arrival_active&&!motion_arrived&&(arrival_kind==2||mode==4)&&arrival_ticks<65535)arrival_ticks<=arrival_ticks+1'b1;
     if(!is_plan&&!is_servo)begin stage_entered<=0;stage_initialized<=0;end
     else begin
       if(!stage_entered)begin
         drop_seed_valid<=0;
         stage_entered<=1;home_terminal<=0;terminal_integral<=0;terminal_settled<=0;
         stage_slewed<=motion_kind>=2?normal_command:move_slewed;
         arrival_begin<=1;arrival_active<=motion_kind>=2;arrival_kind<=motion_kind;arrival_ticks<=0;
       end
       if(stage_ready)begin
         stage_initialized<=1;stage_slewed<=stage_next;last_motion_goal<=is_servo?servo_goal:plan_goal;
         if(motion_finished)begin
           resume_integral<=-(is_servo?servo_integral:plan_integral);resume_command<=stage_next;
           if(motion_kind==1)begin home_terminal<=1;terminal_integral<=plan_integral;terminal_settled<=plan_settled;end
         end
       end
     end
     if(motion_resume&&mode==4)hold_goal<=last_motion_goal;
     // A new Swing session uses the normalized local origin, not a stale Hold target.
     if(session_entry)hold_goal<=0;
     if(mode==7&&drop_done&&drop_home_active)begin
       arrival_active<=0;arrival_ticks<=0;
       drop_seed_valid<=1;precise_terminal<=drop_precise_integral;
       home_terminal<=1;terminal_settled<=drop_settled_state;terminal_integral<=0;
       resume_command<=drop_command;last_motion_goal<=0;
     end
     if(is_home&&move_ready)last_motion_goal<=move_goal;
     if(rebase_commit)begin last_motion_goal<=last_motion_goal-rebase_shift;hold_goal<=hold_goal-rebase_shift;end
   end
 end
 always @(posedge clk)begin
   if(rst||!enable||!is_home)begin move_initialized<=0;move_ticks<=0;move_fault<=0;move_slewed<=0;move_arrived<=0;end
   else begin
     if(!(keep_session===1'b1))move_arrived<=0;
     else if(outer_start&&move_initialized&&move_settled)move_arrived<=1;
     if(motion_resume&&home_terminal)move_slewed<=drop_seed_valid?resume_command:stage_slewed;
     if(absq(move_goal)>(MULTITURN!=0?4194304:183501))move_fault<=1;
     if(outer_start)begin
       if(!home_deadline_exempt&&move_ticks<65535)move_ticks<=move_ticks+1'b1;
       if((MULTITURN!=0?(absq(q)>4194304||(frame_distance>183501||frame_distance< -183501)):absq(q)>183501)||absq(w)>98304||absq(v)>2293760||move_ticks>=40000&&!home_deadline_exempt)move_fault<=1;
     end
     if(move_ready)begin move_initialized<=1;move_slewed<=move_fault?0:clip(move_sum,-65536,65536);end
   end
 end
 foc_core foc(clk,rst,foc_start,enable,ia,ib,iq_ref,phase,phase,foc_busy,foc_done,id,iq,ud,uq,du,dv,dw);
 always @(posedge clk) begin
  if(rst) begin phase_input<=0;phase_signed<=0;upright_phase<=0;phase_mag<=0;phase_multiple<=0;phase_rem<=0;phase_product<=0;phase_sign<=0;frame_distance<=0;frame_turns<=0;rebase_previous<=0;rebase_home_previous<=0;rebase_servo_previous<=0;state<=0;previous<=0;initialized<=0;turn_counts<=0;position<=0;pend_angle<=0;step_delta<=0;
    obs_start<=0;outer_start<=0;foc_start<=0;phase<=0;mean_square<=0;sum_square<=0;sustained<=0;peak<=0;
    busy<=0;done<=0;fault<=0;reason<=0;ia<=0;ib<=0;raw_latched<=0;
    div_start<=0;subtract_rms<=0;obs_ready<=0;numerator<=0;denominator<=1;
    count_position<=0;cphase<=0;position_product<=0;sa<=0;sb<=0;sc<=0;partial_sum<=0;end
  else begin
    done<=0;obs_start<=0;outer_start<=0;foc_start<=0;div_start<=0;
    rebase_previous<=rebase_commit;rebase_home_previous<=rebase_commit&&enable&&is_home;rebase_servo_previous<=rebase_commit&&enable&&is_servo;
    if(rebase_commit)begin
      turn_counts<=turn_counts-rebase_counts;count_position<=count_position-rebase_counts;
      position<=position-rebase_shift;frame_turns<=frame_turns+rebase_turns;
    end
    if(rebase_rejected||(rebase_previous&&!arm_rebase_ack)||(rebase_home_previous&&!move_rebase_ack)||(rebase_servo_previous&&!servo_rebase_ack))begin fault<=1;reason<=8'h25;end
    // Evaluate both arithmetic branches in parallel; select only the final
    // predicate with live mode. This preserves the exact outer_start check.
    if(enable&&outer_start&&((MULTITURN!=0?(abs_gt(q,4194304)||(frame_distance>183501||frame_distance< -183501)):abs_gt(q,183501))||(servo_guard ? abs_gt(w,servo_speed_limit):abs_gt(w,917504))||abs_gt(v,2293760)))begin fault<=1;reason<=8'h21;end
    if(enable&&outer_start&&servo_guard&&(is_servo ? abs_gt(q,absq(servo_goal)+32768):abs_gt(q,absq(last_motion_goal)+32768)))begin fault<=1;reason<=8'h21;end
    if(enable&&outer_start&&(is_servo||servo_guard||is_plan&&motion_kind==2)&&!(servo_guard ? upright_le(alpha,22876):upright_le(alpha,39322)))begin fault<=1;reason<=8'h22;end
    if(enable&&arrival_active&&!motion_arrived&&arrival_ticks>=(arrival_kind==2?16000:12000))begin fault<=1;reason<=8'h24;end
    if(arm_done&&pend_done)obs_ready<=1;
    if(start&&busy) begin fault<=1;reason<=8'h10;end
    case(state)
      0: if(start&&!fault) begin
        busy<=1;raw_latched<=arm_raw;previous<=arm_raw;step_delta<=arm_raw-previous;
        if(!initialized) begin turn_counts<=$signed(initial_diff)-$signed({1'b0,arm_raw});initialized<=1;end
        else if(crossing < -8192) turn_counts<=turn_counts+16384;
        else if(crossing >=8192) turn_counts<=turn_counts-16384;
        if(initialized && ($signed(14'(arm_raw-previous))>100 || $signed(14'(arm_raw-previous))< -100)) begin fault<=1;reason<=8'h11;end
        ia<=mul(($signed({1'b0,ia_raw,16'b0})-offset_a),gain_a);
        ib<=mul(($signed({1'b0,ib_raw,16'b0})-offset_b),gain_b);
        pend_count<=pend_diff;
        phase<=calculated_phase;
        state<=1;
      end
      1: begin
        count_position<=turn_counts+$signed({1'b0,raw_latched});
        // 12-bit signed count times TAU fits signed 31 bits exactly.
        pend_product<=$signed(pend_count)*TAU;
        peak<=absq(ia)>absq(ib) ? (absq(ia)>absq(phase_c) ? absq(ia):absq(phase_c)):(absq(ib)>absq(phase_c) ? absq(ib):absq(phase_c));
        cphase<=phase_c;sa<=64'(ia)*64'(ia);sb<=64'(ib)*64'(ib);state<=9;
      end
      9: begin pend_angle<=-(pend_product>>>12);position_product<=64'(count_position)*TAU;sc<=64'(cphase)*64'(cphase);partial_sum<=sa+sb;state<=10;end
      10: begin position<=sat64(position_product>>>14)*(reverse ? -1:1);sum_square<=partial_sum+sc;state<=11;end
      11: begin obs_start<=1;obs_ready<=0;state<=2;end
      2: begin
        // Q32.32 accumulation avoids losing small per-tick I^2*t increments.
        if(enable) begin
          numerator<=sum_square*2;denominator<=3;div_start<=1;
          sustained<=peak>=127795 ? sustained+1'b1:0;
          // Current sample must still exceed 1.95 A for the twentieth
          // consecutive 0.5 ms interval. A low sample resets the dwell.
          if(peak>=137626 || (peak>=127795&&sustained>=19)) begin
            fault<=1;reason<=8'h12;
          end
          state<=6;
        end else begin mean_square<=0;sustained<=0;state<=3;end
      end
      6: if(div_done) begin
        subtract_rms<=quotient<mean_square;
        numerator<=quotient<mean_square ? mean_square-quotient:quotient-mean_square;
        denominator<=201;div_start<=1;state<=7;
      end
      7: if(div_done) begin
        mean_square<=updated_mean_square;
        // Original Protection::step updates the filter before comparing RMS.
        if(enable&&updated_mean_square>=64'd9663676416)begin fault<=1;reason<=8'h12;end
        numerator<=sum_square;denominator<=2000;div_start<=1;state<=8;
      end
      8: if(div_done) state<=12;
      12: if(dose_done||!enable) begin
        if(dose_fault) begin fault<=1;reason<=8'h12;end
        state<=3;
      end
      3: if(obs_ready) begin
        if(enable) begin frame_distance<=q-frame_reference;phase_input<=alpha-PI;state<=16;end
        else begin foc_start<=1;state<=5;end
      end
      16:begin phase_sign<=phase_input[31];phase_mag<=phase_input[31]?-$unsigned(phase_input):$unsigned(phase_input);state<=17;end
      17:begin phase_product<=phase_mag*14'd10430;state<=18;end
      18:begin phase_multiple<=phase_product[45:32]*19'd411775;state<=19;end
      19:begin phase_rem<=phase_mag-phase_multiple;state<=20;end
      20:begin phase_rem<=phase_rem>=32'd411775 ? phase_rem-32'd411775:phase_rem;state<=21;end
      21:begin phase_signed<=phase_sign?-$signed(phase_rem):$signed(phase_rem);state<=22;end
      22:begin upright_phase<=phase_signed>PI?phase_signed-TAU:phase_signed< -PI?phase_signed+TAU:phase_signed;state<=13;end
      13: begin outer_start<=1;state<=4;end
      4: begin
        if(completed_outer_fault) begin fault<=1;reason<=completed_outer_reason;end
        if(outer_done||!enable)begin foc_start<=1;state<=5;end
      end
      5: if(foc_done) begin done<=1;busy<=0;state<=0;end
    endcase
    // Preserve the first diagnostic until explicit core reset. Pipeline work
    // completing after an electrical trip must not relabel it as motion loss.
    if(fault)begin fault<=1;reason<=reason;end
  end
 end
endmodule
