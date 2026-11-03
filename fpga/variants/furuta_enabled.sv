// Offline implementation candidate. This file is not in the default project.
// Enabling the physical path does not bypass calibration, ARM or fault gating.
module furuta_enabled(
 input wire CLK_50M,UART_RX,AS_MISO,AS_PWM,PEND_DO,ADC_DOUT,
 output wire UART_TX,PWM_U,PWM_V,PWM_W,EN_REQ,ENC_OE,
 output wire AS_SCK,AS_MOSI,AS_CSN,PEND_SCK,PEND_CSN,ADC_CSN,ADC_DIN,ADC_SCLK,
 output wire [7:0] LED,output wire SEG7_DIO,SEG7_SCLK,SEG7_RCLK,
 input wire [4:0] KEY_N);
 // This isolated candidate deliberately includes MOT2/ENA1. Never download
 // before this exact source/netlist/bitstream passes the full acceptance gate.
 wire clk50,clk25,clock_ready,clock_fault,clock_allowed;
 wire inner_en,inner_u,inner_v,inner_w,inner_oe;
 fpga_clock_25_50 clocks(CLK_50M,clk25,clock_ready,clock_fault,clock_allowed,clk50);
 furuta_scheme_a_top #(.CLOCK_HZ(25000000),.STAMP_HZ(50000000),.SENSOR_HALF(100),.SENSOR_GAP(250),.ADC_HALF(3),.ADC_NEGEDGE(0),.ADC_CS_SETUP(6),
  .EXTERNAL_CLOCK(1),.HARD_DISABLE(0),.MULTITURN(1)) dut(
  .CLK_50M(clk25),.PERIPHERAL_CLK(clk50),.CLOCK_READY(clock_ready),.CLOCK_FAULT(clock_fault),
  .PWM_U(inner_u),.PWM_V(inner_v),.PWM_W(inner_w),.EN_REQ(inner_en),.ENC_OE(inner_oe),.*);
 assign EN_REQ=clock_allowed&&inner_en;
 assign PWM_U=clock_allowed&&inner_u;
 assign PWM_V=clock_allowed&&inner_v;
 assign PWM_W=clock_allowed&&inner_w;
 assign ENC_OE=clock_allowed&&inner_oe;
endmodule
