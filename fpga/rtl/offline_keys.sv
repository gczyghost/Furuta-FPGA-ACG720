// ACG720 S0..S4 are active-low, translated BANK2 inputs.
// Two clock synchronizers; 40 consecutive 2-kHz samples = 20 ms debounce.
// A key held through configuration/reset must first be released stably.
module offline_keys #(parameter integer SAMPLES=40)(
 input wire clk,rst,sample_tick,input wire [4:0] key_n,
 output reg [4:0] pressed,output reg [4:0] held);
 reg [4:0] meta,sync;
 reg [4:0] armed;
 localparam integer CW=$clog2(SAMPLES+1);
 reg [CW-1:0] count[0:4];
 integer k;
 always @(posedge clk)begin
   if(rst)begin
     meta<=5'b11111;sync<=5'b11111;pressed<=0;held<=0;armed<=0;
     for(k=0;k<5;k=k+1)count[k]<=0;
   end else begin
     meta<=key_n;sync<=meta;pressed<=0;
     if(sample_tick)for(k=0;k<5;k=k+1)begin
       if(!armed[k])begin
         if(sync[k])begin
           if(count[k]==SAMPLES-1)begin armed[k]<=1;count[k]<=0;end
           else count[k]<=count[k]+1'b1;
         end else count[k]<=0;
       end else if(sync[k]==!held[k])count[k]<=0;
       else if(count[k]==SAMPLES-1)begin
         count[k]<=0;held[k]<=!sync[k];
         if(!sync[k])pressed[k]<=1;
       end else count[k]<=count[k]+1'b1;
     end
   end
 end
endmodule
