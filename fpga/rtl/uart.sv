module uart_rx #(parameter integer DIV=109)(input wire clk,rst,rx,
 output reg valid,framing_error,output reg [7:0] data);
 reg [1:0] sync;reg [1:0] state;integer timer;reg [2:0] bitno;reg [7:0] shift;
 always @(posedge clk) begin
   sync<={sync[0],rx};
   if(rst) begin sync<=3;state<=0;timer<=0;bitno<=0;shift<=0;valid<=0;framing_error<=0;data<=0;end
   else begin valid<=0;framing_error<=0;
     case(state)
      0: if(!sync[1]) begin timer<=DIV/2;state<=1;end
      1: if(timer!=0) timer<=timer-1;else if(sync[1]) state<=0;
         else begin timer<=DIV-1;bitno<=0;state<=2;end
      2: if(timer!=0) timer<=timer-1;else begin
           shift[bitno]<=sync[1];timer<=DIV-1;
           if(bitno==7) state<=3;else bitno<=bitno+1'b1;
         end
      3: if(timer!=0) timer<=timer-1;else begin
           if(sync[1]) begin data<=shift;valid<=1;end else framing_error<=1;
           state<=0;
         end
     endcase
   end
 end
endmodule
module uart_tx #(parameter integer DIV=109)(input wire clk,rst,start,input wire [7:0] data,
 output reg tx,busy);
 reg [9:0] shift;integer timer;reg [3:0] bitno;
 always @(posedge clk) begin
   if(rst) begin tx<=1;busy<=0;shift<=10'h3ff;timer<=0;bitno<=0;end
   else if(start&&!busy) begin shift<={1'b1,data,1'b0};tx<=0;busy<=1;timer<=DIV-1;bitno<=0;end
   else if(busy) begin
     if(timer!=0) timer<=timer-1;
     else if(bitno==9) begin busy<=0;tx<=1;end
     else begin shift<={1'b1,shift[9:1]};tx<=shift[1];bitno<=bitno+1'b1;timer<=DIV-1;end
   end
 end
endmodule
