module encoder_as5048 #(parameter integer HALF_CLKS=100, GAP_CLKS=250)(
    input wire clk,rst,start,miso, input wire [31:0] clock_stamp,
    output wire sck,mosi,csn, output reg busy,done,valid,
    output reg [13:0] angle, output reg [31:0] stamp,
    output reg [15:0] diagnostic, output reg [7:0] errors
);
    reg launch; reg [15:0] tx; wire finished, spi_busy; wire [15:0] rx;
    reg [15:0] head,first,second;
    reg [2:0] index; reg waiting,prime;
    wire signed [13:0] pair_delta=second[13:0]-first[13:0];
    spi_master #(.BITS(16),.HALF_CLKS(HALF_CLKS),.CPHA(1),.GAP_CLKS(GAP_CLKS)) spi
       (clk,rst,launch,tx,miso,sck,csn,mosi,spi_busy,finished,rx);
    always @(posedge clk) begin
      if(rst) begin busy<=0;done<=0;valid<=0;angle<=0;stamp<=0;diagnostic<=0;errors<=0;
          launch<=0;tx<=0;head<=0;first<=0;second<=0;index<=0;waiting<=0;prime<=1;end
      else begin
        launch<=0;done<=0;
        if(start&&!busy) begin busy<=1;valid<=0;index<=prime ? 5:0;tx<=prime ? 16'h4001:16'h7ffd;prime<=0;launch<=1;waiting<=1;stamp<=clock_stamp;end
        if(busy && finished && waiting) begin
          case(index)
            // Explicit reset/clear only: read ERRFL then NOP. No automatic
            // retries or clearing a hardware error while the bridge is active.
            5: begin index<=6;tx<=0;launch<=1;end
            6: begin index<=0;tx<=16'h7ffd;launch<=1;end
            0: begin head<=rx;tx<=16'hffff;end
            1: begin diagnostic<=rx;tx<=16'hffff;end
            2: begin first<=rx;tx<=0;end
            3: begin second<=rx;tx<=0;end
            4: begin
              errors <= {3'b0, (pair_delta>32||pair_delta< -32),
                   (^first || ^second || first[14] || second[14]),
                   (^diagnostic || diagnostic[14] || (diagnostic&16'h3f00)!=16'h0100),
                   rx!=0, head!=0};
              valid <= head==0 && rx==0 && !(^first) && !(^second) && !first[14] && !second[14] &&
                   !(^diagnostic) && !diagnostic[14] && (diagnostic&16'h3f00)==16'h0100 &&
                   pair_delta<=32 && pair_delta>=-32;
              angle<=second[13:0];busy<=0;done<=1;waiting<=0;
            end
          endcase
          if(index<4) begin index<=index+1'b1; launch<=1;end
        end
      end
    end
endmodule
