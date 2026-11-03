// Protocol revision 2: atomic 17-word snapshots plus correlated command ACKs.
// Telemetry sequence is independent from the echoed command sequence.
module telemetry #(parameter integer PERIOD=1000000, WORDS=17)(input wire clk,rst,
 input wire [WORDS*32-1:0] values,input wire tx_busy,
 output reg tx_start,output reg [7:0] tx_data,
 input wire ack_valid,input wire [7:0] ack_result,ack_addr,ack_seq,input wire [31:0] ack_value);
 reg [31:0] timer;reg [3:0] index;reg [4:0] channel;reg [7:0] seq;reg [79:0] packet;
 reg sending,waiting,due;reg [WORDS*32-1:0] captured;
 reg [79:0] replies[0:7];reg [3:0] wr,rd;
 wire [3:0] used=wr-rd; // modular occupancy, including pointer wrap
 wire [31:0] selected=channel==0 ? values[31:0]:(captured>>(channel*32));
 function automatic [15:0] crc64(input [63:0] data);
 reg [15:0] c;integer i;begin c=16'hffff;for(i=63;i>=0;i=i-1)c=(c[15]^data[i]) ? (c<<1)^16'h1021:c<<1;crc64=c;end
 endfunction
 always @(posedge clk) begin
   if(rst) begin timer<=0;index<=0;seq<=0;packet<=0;sending<=0;waiting<=0;tx_start<=0;tx_data<=0;channel<=0;captured<=0;wr<=0;rd<=0;due<=0;end
   else begin
     tx_start<=0;
     timer<=timer==PERIOD-1 ? 0:timer+1'b1;
     if(timer==PERIOD-1)due<=1;
     // Bounded queue. A flooding peer must time out missing ACKs; never infer acceptance.
     if(ack_valid && used<8)begin
       replies[wr[2:0]]<={8'ha5,ack_result,ack_addr,ack_seq,ack_value,crc64({8'ha5,ack_result,ack_addr,ack_seq,ack_value})};wr<=wr+1'b1;
     end
     if(!sending)begin
       if(due)begin
         packet<={8'ha5,8'h80,3'b0,channel,seq,selected,crc64({8'ha5,8'h80,3'b0,channel,seq,selected})};
         if(channel==0)captured<=values;
         channel<=channel==WORDS-1 ? 0:channel+1'b1;
         seq<=seq+1'b1;index<=0;sending<=1;due<=0;
       end else if(wr!=rd)begin packet<=replies[rd[2:0]];rd<=rd+1'b1;index<=0;sending<=1;end
     end
     if(sending)begin
       if(!waiting&&!tx_busy)begin tx_data<=packet[79:72];packet<=packet<<8;tx_start<=1;waiting<=1;end
       else if(waiting&&tx_busy)begin waiting<=0;index<=index+1'b1;if(index==9)sending<=0;end
     end
   end
 end
endmodule
