// Scheme A held-bundle transfer. Both clocks come from the same PLL and all
// paths remain timed. No false path or multicycle exception is authorized.
// Assert a common reset in BOTH domains, including when either clock stops;
// release it synchronously in each domain before offering new data.
// Destination consumes every dst_valid pulse. Source must honor src_busy.
// Overrun is explicit and sticky until reset; the first payload is preserved.
module scheme_a_mailbox #(parameter integer WIDTH=187)(
 input wire src_clk,src_rst,src_send,input wire [WIDTH-1:0] src_data,
 output wire src_busy,output reg src_overrun,
 input wire dst_clk,dst_rst,output reg dst_valid,
 output reg [WIDTH-1:0] dst_data
);
 reg request,acknowledge;
 reg [WIDTH-1:0] held;
 reg [1:0] ack_sync,request_sync;
 assign src_busy=request!=ack_sync[1];
 always @(posedge src_clk or posedge src_rst)begin
  if(src_rst)begin
   request<=0;held<=0;ack_sync<=0;src_overrun<=0;
  end else begin
   ack_sync<={ack_sync[0],acknowledge};
   if(src_send)begin
    if(src_busy)src_overrun<=1;
    else begin held<=src_data;request<=~request;end
   end
  end
 end
 always @(posedge dst_clk or posedge dst_rst)begin
  if(dst_rst)begin
   request_sync<=0;acknowledge<=0;dst_data<=0;dst_valid<=0;
  end else begin
   request_sync<={request_sync[0],request};dst_valid<=0;
   if(request_sync[1]!=acknowledge)begin
    dst_data<=held;dst_valid<=1;acknowledge<=request_sync[1];
   end
  end
 end
endmodule
