// Local producer uses the unchanged supervisor command/ACK interface.
// Online UART sequencing/heartbeat semantics remain unmodified. Entering local
// ownership performs stopped SYNC; returning online first expires the local HB.
// A reconnecting host must use its existing stopped SYNC handshake.
module offline_link(
 input wire clk,rst,tick,mode_key,recover_key,local_stop,run,activity,
 input wire host_cmd,input wire [7:0] host_op,host_addr,host_seq,input wire [31:0] host_value,
 output reg cmd,output reg [7:0] op,addr,seq,output reg [31:0] value,
 input wire guard_ack,input wire [7:0] guard_result,guard_addr,guard_seq,input wire [31:0] guard_value,
 output reg host_ack,output reg [7:0] host_result,host_ack_addr,host_ack_seq,output reg [31:0] host_ack_value,
 input wire request,input wire [7:0] request_op,request_addr,input wire [31:0] request_value,
 output wire request_ready,output reg response,output reg response_ok,output reg [31:0] response_value,
 output wire offline,output reg abort_local,output reg link_failed,
 input wire invalidate);
 localparam [1:0] ONLINE=0,ENTER=1,LOCAL=2,LEAVE=3;
 reg [1:0] state;
 reg [7:0] next_seq,tx_seq,tx_addr;
 reg pending,tx_user,tx_sync;
 reg [9:0] wait_ticks;
 reg [7:0] hb_ticks;
 reg [10:0] exit_ticks;
 reg [4:0] drain;
 reg send_local;reg [7:0] send_op,send_addr,send_seq;reg [31:0] send_value;
 // Stage only normal traffic. STOP and same-edge cancellation bypass it.
  reg normal_cmd;reg [7:0] normal_op,normal_addr,normal_seq;reg [31:0] normal_value;
  wire cancel_normal=invalidate===1'b1;
  wire host_stop=host_cmd&&host_op==2;
 wire key_stop=mode_key&&(run||activity);
 wire stop_now=host_stop||key_stop||local_stop;
 wire mode_change=mode_key&&!run&&!activity&&state!=LEAVE;
 assign offline=state!=ONLINE;
 assign request_ready=state==LOCAL&&!link_failed&&!pending&&!send_local&&drain==0&&hb_ticks<200&&!stop_now&&!mode_change&&!cancel_normal;
 // Normal payload is captured atomically before guard dispatch. The final
 // guard still checks live qualification; no cached admission permission.
 always @(posedge clk)begin
   if(rst)begin normal_cmd<=0;normal_op<=0;normal_addr<=0;normal_seq<=0;normal_value<=0;end
   else begin
     normal_cmd<=0;
     if(!stop_now&&!mode_change&&!cancel_normal&&drain==0)begin
       if(state==ONLINE)begin
         normal_cmd<=host_cmd;normal_op<=host_op;normal_addr<=host_addr;normal_seq<=host_seq;normal_value<=host_value;
       end else begin
         normal_cmd<=send_local;normal_op<=send_op;normal_addr<=send_addr;normal_seq<=send_seq;normal_value<=send_value;
       end
     end
   end
 end
 // Raw STOP preempts the staged producer at the original rising edge.
 always @* begin
   cmd=normal_cmd&&!mode_change&&!cancel_normal&&drain==0;
   op=normal_op;addr=normal_addr;seq=normal_seq;value=normal_value;
   if(host_stop)begin cmd=1;op=2;addr=host_addr;seq=host_seq;value=host_value;end
   else if(key_stop||local_stop)begin cmd=1;op=2;addr=0;seq=8'hfe;value=0;end
 end
 task emit(input [7:0] operation,address,input [31:0] data,input user_command,sync_command);
 begin
   send_local<=1;send_op<=operation;send_addr<=address;send_seq<=next_seq;send_value<=data;
   tx_seq<=next_seq;tx_addr<=address;next_seq<=next_seq+1'b1;
   pending<=1;tx_user<=user_command;tx_sync<=sync_command;wait_ticks<=0;
 end endtask
 always @(posedge clk)begin
   if(rst)begin
     state<=ONLINE;next_seq<=0;tx_seq<=0;tx_addr<=0;pending<=0;tx_user<=0;tx_sync<=0;
     wait_ticks<=0;hb_ticks<=200;exit_ticks<=0;drain<=0;send_local<=0;
     send_op<=0;send_addr<=0;send_seq<=0;send_value<=0;
     host_ack<=0;host_result<=0;host_ack_addr<=0;host_ack_seq<=0;host_ack_value<=0;
     response<=0;response_ok<=0;response_value<=0;abort_local<=0;link_failed<=0;
   end else begin
     send_local<=0;host_ack<=0;response<=0;abort_local<=0;
     if(drain!=0)drain<=drain-1'b1;
     if(tick)begin
       if(hb_ticks<200)hb_ticks<=hb_ticks+1'b1;
       if(pending&&wait_ticks<700)wait_ticks<=wait_ticks+1'b1;
       if(state==LEAVE&&exit_ticks<2000)exit_ticks<=exit_ticks+1'b1;
     end
     if(host_stop)begin
       host_ack<=1;host_result<=8'h81;host_ack_addr<=host_addr;host_ack_seq<=host_seq;host_ack_value<=0;
     end else if(state==ONLINE&&drain==0&&!mode_change&&!key_stop&&!local_stop)begin
       host_ack<=guard_ack;host_result<=guard_result;host_ack_addr<=guard_addr;
       host_ack_seq<=guard_seq;host_ack_value<=guard_value;
     end else if(host_cmd)begin
       // Deterministic ownership NACK; no offline host write/heartbeat/ARM.
       // STOP is still executed immediately and acknowledged independently.
       host_ack<=1;host_result<=host_stop?8'h81:8'h82;
       host_ack_addr<=host_addr;host_ack_seq<=host_seq;host_ack_value<=host_stop?0:32'd2;
     end
     if(state==ENTER&&!pending&&!send_local&&drain==0&&!link_failed)
       emit(4,0,32'h53594e43,0,1);
     if(state==LOCAL&&!pending&&!send_local&&drain==0&&!link_failed)begin
       if(hb_ticks>=200)begin emit(1,0,32'h48424154,0,0);hb_ticks<=0;end
       else if(request&&request_ready)emit(request_op,request_addr,request_value,1,0);
     end
     if(pending&&guard_ack&&guard_seq==tx_seq&&guard_addr==tx_addr)begin
       pending<=0;
       if(tx_user)begin response<=1;response_ok<=guard_result==8'h81;response_value<=guard_value;end
       if(tx_sync)begin
         if(guard_result==8'h81)begin state<=LOCAL;hb_ticks<=200;end
         else begin link_failed<=1;abort_local<=1;end
       end else if(!tx_user&&guard_result!=8'h81)begin link_failed<=1;abort_local<=1;end
     end
     if(pending&&wait_ticks>=700)begin
       pending<=0;link_failed<=1;abort_local<=1;
       response<=tx_user;response_ok<=0;response_value<=0;
     end
     if(state==LEAVE&&exit_ticks>=2000&&drain==0)state<=ONLINE;
     if(mode_change)begin
       state<=state==ONLINE?ENTER:LEAVE;exit_ticks<=0;drain<=16;
       pending<=0;send_local<=0;next_seq<=0;link_failed<=0;abort_local<=1;
     end
     // Explicit S1 may repair local protocol ownership before its CLEAR;
     // this SYNC alone never clears a fault, changes calibration or starts motion.
     if(recover_key&&offline&&!run&&link_failed)begin
       state<=ENTER;drain<=16;pending<=0;send_local<=0;next_seq<=0;link_failed<=0;
     end
     if(stop_now)begin
       pending<=0;send_local<=0;drain<=16;abort_local<=1;
       // Host STOP may consume a matching local sequence. Resynchronize while
       // stopped before another local command, never retry the canceled action.
       if(state==LOCAL||state==ENTER)begin state<=ENTER;next_seq<=0;end
     end
     if(cancel_normal)begin
      pending<=0;send_local<=0;drain<=16;abort_local<=1;
      // A canceled sequence may have reached guard before this edge. Start a
      // fresh stopped SYNC; this never clears a device fault or starts motion.
      if(state==LOCAL||state==ENTER)begin state<=ENTER;next_seq<=0;end
    end
  end
 end
endmodule
