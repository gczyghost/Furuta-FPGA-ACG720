// CPOL=0, MSB first. All state uses clk; pins are registered.
// Slow encoder inputs use 2-flop synchronization. The source-synchronous ADC
// bypasses that latency and captures directly on the opposite SCLK edge.
module spi_master #(
    parameter integer BITS=16, HALF_CLKS=100, CPHA=0, GAP_CLKS=250, SYNC_INPUT=2,
    CS_SETUP_CLKS=HALF_CLKS, RX_DELAY_CLKS=0, CAPTURE_NEGEDGE=0
)(input wire clk, rst, start, input wire [BITS-1:0] tx,
  input wire miso, output reg sck, csn, mosi,
  output reg busy, done, output reg [BITS-1:0] rx);
    reg [1:0] mi;
    reg [BITS-1:0] shift;
    localparam integer MAX_EDGE_WAIT=CS_SETUP_CLKS>HALF_CLKS?CS_SETUP_CLKS:HALF_CLKS;
    localparam integer MAX_WAIT=MAX_EDGE_WAIT>GAP_CLKS?MAX_EDGE_WAIT:GAP_CLKS;
    localparam integer TIMER_WIDTH=$clog2(MAX_WAIT)<1?1:$clog2(MAX_WAIT);
    localparam integer EDGE_WIDTH=$clog2(2*BITS+1);
    // Every timer load is0..MAX_WAIT-1 and decrement requires nonzero.
    // Edge count is0..2*BITS; no runtime operand or precision is truncated.
    reg [TIMER_WIDTH-1:0] timer;reg [EDGE_WIDTH-1:0] edge_count;
    reg [1:0] state;
    // CPHA0 may capture after the external rising edge, while DOUT remains
    // stable until the next falling edge. Encoders keep their original phase.
    wire sample_now=state==1 && (CPHA ? (sck && timer==0) :
      (RX_DELAY_CLKS==0 ? (!sck && timer==0) : (sck && timer==HALF_CLKS-RX_DELAY_CLKS)));
    // ADC25 only: a same-clock falling-edge register captures the stable
    // external window. Its transfer to the rising-edge shift register is a
    // real half-period path that must pass STA; no falsepath/multicycle.
    wire direct_miso;
    generate if(CAPTURE_NEGEDGE!=0)begin:falling_capture
      reg input_q;
      always @(negedge clk)begin
        if(rst)input_q<=0;else input_q<=miso;
      end
      assign direct_miso=input_q;
    end else begin:direct_capture
      assign direct_miso=miso;
    end endgenerate
    always @(posedge clk) begin
        mi <= {mi[0],miso};
        if(rst) begin
            mi<=0; state<=0; sck<=0; csn<=1; mosi<=0;
            busy<=0; done<=0; rx<=0; shift<=0; timer<=0; edge_count<=0;
        end else begin
            done<=0;
            if(sample_now)rx<={rx[BITS-2:0],(SYNC_INPUT==0 ? direct_miso:mi[1])};
            case(state)
              0: if(start) begin
                  busy<=1; csn<=0; sck<=0; shift<=tx; rx<=0;
                  mosi<=tx[BITS-1]; timer<=CS_SETUP_CLKS-1; edge_count<=0; state<=1;
              end
              1: if(timer!=0) timer<=timer-1;
                 else begin
                    timer<=HALF_CLKS-1;
                    sck<=~sck; edge_count<=edge_count+1;
                    if ((!CPHA && sck) || (CPHA && !sck)) begin
                        if(CPHA) begin mosi<=shift[BITS-1]; shift<={shift[BITS-2:0],1'b0}; end
                        else begin shift<={shift[BITS-2:0],1'b0}; mosi<=shift[BITS-2]; end
                    end
                    if(edge_count==2*BITS-1) state<=2;
                 end
              2: if(timer!=0) timer<=timer-1;
                 else begin csn<=1; sck<=0; timer<=GAP_CLKS-1; state<=3; end
              3: if(timer!=0) timer<=timer-1;
                 else begin busy<=0; done<=1; state<=0; end
            endcase
        end
    end
endmodule
