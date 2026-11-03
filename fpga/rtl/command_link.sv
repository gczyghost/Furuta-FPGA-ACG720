// Experimental FPGA protocol, distinct from ESP32 CSV protocol.
// A5 op addr seq data[31:24]..data[7:0] CRC16[15:8] CRC16[7:0].
// CRC-CCITT-FALSE over first 8 bytes. Partial frame expires after 10 ms.
module command_link #(parameter integer BYTE_TIMEOUT=500000)(
 input wire clk,rst,byte_valid,framing_error,input wire [7:0] byte_data,
 output reg command_valid,output reg [7:0] op,addr,seq,output reg [31:0] value);
 reg [3:0] index;reg [15:0] crc;reg [7:0] crc_high;reg [55:0] payload;reg [31:0] timer;
 function automatic [15:0] crc_byte(input [15:0] seed,input [7:0] b);
   reg [15:0] c;integer i;begin c=seed^{b,8'b0};for(i=0;i<8;i=i+1)c=c[15] ? (c<<1)^16'h1021:c<<1;crc_byte=c;end
 endfunction
 always @(posedge clk) begin
   if(rst) begin index<=0;crc<=16'hffff;crc_high<=0;payload<=0;timer<=0;command_valid<=0;op<=0;addr<=0;seq<=0;value<=0;end
   else begin
     command_valid<=0;
     if(index!=0) begin timer<=timer+1'b1;if(timer>=BYTE_TIMEOUT||framing_error) index<=0;end
     if(byte_valid) begin
       timer<=0;
       if(index==0) begin if(byte_data==8'ha5) begin crc<=crc_byte(16'hffff,byte_data);index<=1;end end
       else if(index<8) begin payload<={payload[47:0],byte_data};crc<=crc_byte(crc,byte_data);index<=index+1'b1;end
       else if(index==8) begin crc_high<=byte_data;index<=9;end
       else begin
         if({crc_high,byte_data}==crc) begin
           op<=payload[55:48];addr<=payload[47:40];seq<=payload[39:32];value<=payload[31:0];command_valid<=1;
         end
         index<=0;
       end
     end
   end
 end
endmodule
