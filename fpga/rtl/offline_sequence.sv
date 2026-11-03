// GUI90 v27_full: HOME, SWING, +90,-90,+180,-180,+360, PeakShowcase.
// Observes the existing atomic 30-word telemetry snapshot at 50 Hz. Transport
// capability reads are unnecessary for this statically bound B68 implementation;
// all motion commands still pass through the original supervisor admission.
module offline_sequence #(parameter integer TICK_HZ=2000,CLOCK_HZ=50000000)(
 input wire clk,rst,tick,observe,input wire [959:0] row,
 input wire offline,abort_local,link_failed,input wire [4:0] keys,
 input wire request_ready,response,response_ok,input wire [31:0] response_value,
 output reg request,output reg [7:0] request_op,request_addr,output reg [31:0] request_value,
 output reg local_stop,output wire activity,output reg failed,output reg [7:0] state);
 localparam [7:0] IDLE=0,GATE=1,HOME_WAIT=2,MOVE_WAIT=3,PREP_BEGIN=4,
   PREP_WAIT=5,SERVO_WAIT=6,PLATFORM=7,END_WAIT=8,DROP_WAIT=9,
   SEND=10,ACK=11,CLEAR_WAIT=12,DROP_POLL=13,SWING_WAIT=14,CANCEL_WAIT=15,DROP_DIRECT=16;
 reg [7:0] after_ack;
 reg demo,peak,return_drop,seen,previous_good,previous_valid,position_sent,direct_drop;
 reg [3:0] step;
 reg [20:0] total_ticks;
 reg [17:0] phase_ticks,peak_ticks;
 reg [15:0] prep_ticks,idle_ticks;
 reg [10:0] quiet_ticks;
 reg [7:0] drop_poll_ticks;
 reg [2:0] stable_count;
 reg [31:0] epoch,previous_stamp,previous_interval,platform_ticks,clear_stamp;
 reg signed [31:0] previous_q,previous_goal,previous_turns;
 wire [31:0] status=row[0+:32],motion=row[19*32+:32],flags=row[27*32+:32];
 wire run=status[0],fault=status[1],ready=status[2],calibrated=status[7];
 wire [2:0] mode=status[4+:3];
 wire [1:0] kind=motion[1:0];
 wire active=motion[3],arrived=motion[5],finite=motion[7],home_settled=motion[8],home_active=motion[9],keep=motion[10],settling=motion[11];
 wire fresh=flags[4]&&row[13*32+31];
 wire signed [31:0] q=row[32+:32],alpha=row[64+:32],w=row[96+:32],v=row[128+:32];
 wire signed [31:0] turns=row[17*32+:32],goal=row[18*32+:32];
 wire signed [31:0] ref_w=row[22*32+:32],sensor_w=row[24*32+:32];
 wire [31:0] stamp=row[16*32+:32],current_epoch=row[28*32+:32],dose=row[21*32+:32];
 wire [31:0] delta_stamp=stamp-previous_stamp;
 wire signed [31:0] delta_turns=turns-previous_turns;
 wire signed [31:0] delta_q=q-previous_q,delta_goal=goal-previous_goal;
 function automatic bit near(input signed [31:0] x,limit);near=x>=-limit&&x<=limit;endfunction
 function automatic bit up(input signed [31:0] x,lower,upper);
   up=(x>=lower&&x<=upper)||(x<=-lower&&x>=-upper);
 endfunction
 // Exact integer intervals for GUI90 floating comparisons at Q16 inputs.
 wire down_angle=near(alpha,571)||(alpha>=411203&&alpha<=412346)||(alpha<=-411203&&alpha>=-412346);
 wire down_quiet=run&&keep&&mode==0&&kind==0&&home_settled&&near(q-goal,571)&&down_angle&&near(w,3276)&&near(v,3276);
 wire up_quiet=run&&(mode==3||mode==4)&&!settling&&near(q-goal,2286)&&up(alpha,204774,207001)&&near(w,9830)&&near(v,13107);
 wire movement_done=run&&mode==4&&arrived&&near(q-goal,2287)&&up(alpha,203600,208175)&&near(w,9830);
 wire prepared=mode==4&&arrived&&!active&&near(q,return_drop?983:2287)&&near(goal,return_drop?983:2287)&&near(w,9830)&&near(v,13107)&&(!return_drop||!settling);
 wire platform_good=flags[0]&&flags[1]&&kind==3&&active&&!finite&&!settling&&
   ref_w>=1371897&&ref_w<=1373269&&sensor_w>=1338269&&sensor_w<=1406897;
 wire atomic_frame_ok=delta_turns==0||
   (delta_turns==1&&near(delta_q+411775,2287)&&near(delta_goal+411775,2287))||
   (delta_turns==-1&&near(delta_q-411775,2287)&&near(delta_goal-411775,2287));
 wire signed [31:0] zero_delta= -goal;
 wire signed [31:0] phase_zero=zero_delta>205888?zero_delta-411775:zero_delta< -205888?zero_delta+411775:zero_delta;
 wire [7:0] zero_bank=near(phase_zero,34314)?5:near(phase_zero,102943)?6:near(phase_zero,205887)?7:8;
 wire [7:0] down_bank=near(phase_zero,34314)?0:near(phase_zero,102943)?1:near(phase_zero,194448)?2:near(phase_zero,205887)?3:4;
 assign activity=state!=IDLE;
 task queue_command(input [7:0] operation,address,input [31:0] data,input [7:0] successor);
 begin
   request_op<=operation;request_addr<=address;request_value<=data;request<=1;
   state<=SEND;after_ack<=successor;phase_ticks<=0;seen<=0;
 end endtask
 task fail_stop;
 begin state<=IDLE;failed<=1;request<=0;local_stop<=1;demo<=0;peak<=0;end
 endtask
 task start_preparation(input drop_after);
 begin
   return_drop<=drop_after;state<=PREP_BEGIN;prep_ticks<=0;idle_ticks<=0;
   stable_count<=0;previous_valid<=0;position_sent<=0;
 end endtask
 always @(posedge clk)begin
   if(rst)begin
     state<=IDLE;after_ack<=IDLE;demo<=0;peak<=0;return_drop<=0;step<=0;
     seen<=0;previous_good<=0;previous_valid<=0;position_sent<=0;direct_drop<=0;
     total_ticks<=0;phase_ticks<=0;peak_ticks<=0;prep_ticks<=0;idle_ticks<=0;
     quiet_ticks<=0;drop_poll_ticks<=0;stable_count<=0;epoch<=0;previous_stamp<=0;previous_interval<=0;platform_ticks<=0;clear_stamp<=0;
     previous_q<=0;previous_goal<=0;previous_turns<=0;
     request<=0;request_op<=0;request_addr<=0;request_value<=0;local_stop<=0;failed<=0;
   end else begin
     local_stop<=0;
     if(tick&&drop_poll_ticks<TICK_HZ/10)drop_poll_ticks<=drop_poll_ticks+1'b1;
     if(tick&&activity)begin
       if(total_ticks<600*TICK_HZ+1)total_ticks<=total_ticks+1'b1;
       if(phase_ticks<120*TICK_HZ+1)phase_ticks<=phase_ticks+1'b1;
       if(peak&&peak_ticks<120*TICK_HZ+1)peak_ticks<=peak_ticks+1'b1;
       if(state==PREP_BEGIN||state==PREP_WAIT||state==DROP_POLL||
          (state==SEND||state==ACK)&&(after_ack==PREP_WAIT||after_ack==DROP_POLL))begin
         if(prep_ticks<18*TICK_HZ+1)prep_ticks<=prep_ticks+1'b1;
         if(idle_ticks!=0&&idle_ticks<2*TICK_HZ+1)idle_ticks<=idle_ticks+1'b1;
       end
     end
     if(state==SEND&&request_ready)begin request<=0;state<=ACK;end
     if(state==ACK&&response)begin
       if(!response_ok)fail_stop();
       else begin
         state<=after_ack;phase_ticks<=0;
         if(after_ack==CLEAR_WAIT)begin failed<=0;clear_stamp<=stamp;end
         if(after_ack==DROP_POLL)begin
           if(response_value[2:0]==3)queue_command(8'h22,7,32'h44525031,DROP_WAIT);
           else begin state<=direct_drop?DROP_DIRECT:PREP_WAIT;stable_count<=0;end
         end
       end
     end
     if(observe)begin
       case(state)
       GATE:begin
         if(fresh&&!fault&&(run?step<=1?down_quiet:up_quiet:ready))begin
           if(quiet_ticks<TICK_HZ*2/5+TICK_HZ/50)quiet_ticks<=quiet_ticks+TICK_HZ/50;
           if((step==0||quiet_ticks>=TICK_HZ*2/5)&&(!run||step==0||step==7||dose<41944))begin
             quiet_ticks<=0;
             case(step)
             0:begin
               if(run)begin
                 if(down_quiet&&(near(q,11)||near(goal,11)&&near(q,571)))begin step<=1;end
                 else if(down_quiet)begin
                   // Already enabled: one physical zero-position request.
                   queue_command(8'h27,down_bank,0,HOME_WAIT);
                 end else fail_stop();
               end else queue_command(8'h24,1,0,HOME_WAIT);
             end
             1:queue_command(8'h29,3,32'h53574731,SWING_WAIT);
             2:queue_command(8'h25,6,32'd102944,MOVE_WAIT);
             3:queue_command(8'h25,6,-32'sd102944,MOVE_WAIT);
             4:queue_command(8'h25,7,32'd205887,MOVE_WAIT);
             5:queue_command(8'h25,7,-32'sd205887,MOVE_WAIT);
             6:queue_command(8'h25,8,32'd411775,MOVE_WAIT);
             7:begin peak<=1;peak_ticks<=0;start_preparation(0);end
             default:fail_stop();
             endcase
           end
         end else quiet_ticks<=0;
       end
       HOME_WAIT:if(down_quiet&&home_active)begin step<=1;state<=GATE;quiet_ticks<=0;end
       SWING_WAIT:if(up_quiet)begin
         // GATE owns the one GUI90 0.4s dwell; do not add another dwell here.
         quiet_ticks<=TICK_HZ/50;
         if(demo)begin step<=2;state<=GATE;end else state<=IDLE;
       end
       MOVE_WAIT:begin
         if(active)seen<=1;
         if(seen&&movement_done)begin step<=step+1'b1;state<=GATE;quiet_ticks<=0;end
       end
       PREP_BEGIN:begin
         // Only one physical positioning command. Near a whole turn, leave
         // the command port idle for the original atomic rebase mechanism.
         previous_valid<=0;stable_count<=0;idle_ticks<=0;
         if(near(phase_zero,return_drop?983:2287))state<=PREP_WAIT;
         else begin position_sent<=1;queue_command(8'h27,zero_bank,0,PREP_WAIT);end
       end
       PREP_WAIT:begin
         if(mode==4&&arrived&&!active)begin if(idle_ticks==0)idle_ticks<=1;end
         else begin idle_ticks<=0;stable_count<=0;end
         if(!previous_valid||stamp!=previous_stamp)begin
           previous_valid<=1;previous_stamp<=stamp;previous_interval<=delta_stamp;
           previous_q<=q;previous_goal<=goal;previous_turns<=turns;
           if(previous_valid&&(delta_stamp==0||delta_stamp>CLOCK_HZ/4||!atomic_frame_ok))fail_stop();
           else if(prepared)begin
             if(stable_count<3)stable_count<=stable_count+1'b1;
             if(stable_count>=2&&{1'b0,delta_stamp}+{1'b0,previous_interval}<=CLOCK_HZ/4)begin
               if(return_drop&&drop_poll_ticks>=TICK_HZ/10)begin
                 drop_poll_ticks<=0;queue_command(8'h12,18,0,DROP_POLL);
               end
               else if(!return_drop&&flags[8])queue_command(8'h26,0,0,SERVO_WAIT);
             end
           end else stable_count<=0;
         end
       end
       SERVO_WAIT:if(kind==3&&active&&!finite&&!settling)begin
         previous_valid<=0;previous_good<=0;platform_ticks<=0;
         queue_command(8'h26,0,200,PLATFORM);
       end
       PLATFORM:if(!previous_valid||stamp!=previous_stamp)begin
         previous_valid<=1;previous_stamp<=stamp;previous_good<=platform_good;
         if(platform_good&&previous_valid&&previous_good&&delta_stamp!=0&&delta_stamp<=CLOCK_HZ*3/20)begin
           platform_ticks<=platform_ticks+delta_stamp;
           if(platform_ticks+delta_stamp>=CLOCK_HZ)queue_command(8'h26,1,0,END_WAIT);
         end else platform_ticks<=0;
       end
       END_WAIT:if(movement_done&&!active&&!settling)start_preparation(1);
       CANCEL_WAIT:if(movement_done&&!active&&!settling)start_preparation(1);
       DROP_DIRECT:if(up_quiet&&near(q,983)&&near(goal,983)&&drop_poll_ticks>=TICK_HZ/10)begin
         drop_poll_ticks<=0;queue_command(8'h12,18,0,DROP_POLL);
       end
       DROP_WAIT:if(down_quiet&&home_active)begin state<=IDLE;demo<=0;peak<=0;end
       CLEAR_WAIT:if(fresh&&!fault&&stamp!=clear_stamp)begin state<=IDLE;failed<=0;end
       default:begin end
       endcase
     end
     // All deadlines use local time, never depend on host polling or a later turn.
     if(activity)begin
       if(total_ticks>(demo?600:60)*TICK_HZ||peak&&peak_ticks>120*TICK_HZ)fail_stop();
       if(state==HOME_WAIT&&phase_ticks>TICK_HZ*39/2||state==MOVE_WAIT&&phase_ticks>19*TICK_HZ||
          state==SERVO_WAIT&&phase_ticks>2*TICK_HZ||state==PLATFORM&&phase_ticks>30*TICK_HZ||
          state==END_WAIT&&phase_ticks>25*TICK_HZ||state==CANCEL_WAIT&&phase_ticks>19*TICK_HZ||state==DROP_WAIT&&phase_ticks>TICK_HZ*41/2||
          state==CLEAR_WAIT&&phase_ticks>2*TICK_HZ)fail_stop();
       if((state==PREP_BEGIN||state==PREP_WAIT||state==DROP_POLL||
          (state==SEND||state==ACK)&&(after_ack==PREP_WAIT||after_ack==DROP_POLL))&&
          (prep_ticks>18*TICK_HZ||idle_ticks>2*TICK_HZ))fail_stop();
       if(state!=CLEAR_WAIT&&!(state==SEND||state==ACK)&&observe&&
          (fault||current_epoch!=epoch||!fresh||state!=GATE&&!run))fail_stop();
     end
     // Local transport failures cancel, never automatically reissue a motion.
     if(link_failed&&(activity||run))fail_stop();
     if(!offline||abort_local)begin state<=IDLE;request<=0;demo<=0;peak<=0;end
     else if(keys[1]&&!run)begin
       demo<=0;peak<=0;total_ticks<=0;epoch<=current_epoch;
       queue_command(3,0,32'h434c4541,CLEAR_WAIT);
     end else if(keys[3])begin
       // Cancel all demo successors; finish an active servo using its real END.
       demo<=0;peak<=0;total_ticks<=0;epoch<=current_epoch;quiet_ticks<=0;direct_drop<=0;
       if(!run||fault)begin state<=IDLE;request<=0;if(activity)local_stop<=1;end
       else if(kind==3&&active&&!finite)queue_command(8'h26,1,0,END_WAIT);
       else if(mode==3)begin direct_drop<=1;state<=DROP_DIRECT;phase_ticks<=0;request<=0;end
       else if(mode==4)start_preparation(1);
       else if(active)begin state<=CANCEL_WAIT;phase_ticks<=0;request<=0;end
       else if(mode==7)begin state<=DROP_WAIT;phase_ticks<=0;request<=0;end
       else if(down_quiet)begin state<=IDLE;request<=0;end
       else fail_stop();
     end else if((keys[2]||keys[4])&&!activity&&!failed&&!fault&&calibrated&&fresh&&
                  (!run||down_quiet))begin
       demo<=keys[4];peak<=0;total_ticks<=0;phase_ticks<=0;epoch<=current_epoch;direct_drop<=0;
       step<=0;state<=GATE;quiet_ticks<=0;request<=0;
     end
   end
 end
endmodule
