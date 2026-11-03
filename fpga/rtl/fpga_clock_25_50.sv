// GW5AT native PLL: 50MHz /1 *1 *16 /32 =25MHz, VCO800MHz.
// UG306: qualify LOCK continuously for >=2ms. Tool computes analog ICP/LPF.
module fpga_clock_25_50(input wire ref_clk,output wire core_clk,ready,fault,output wire output_allowed,output wire peripheral_clk);
 wire pll_lock;
 PLL #(.FCLKIN("50.0"),.IDIV_SEL(1),.FBDIV_SEL(1),.MDIV_SEL(16),.ODIV0_SEL(32),.ODIV1_SEL(16),
  .CLKOUT0_EN("TRUE"),.CLKOUT1_EN("TRUE"),.CLKFB_SEL("INTERNAL")) pll(
  .CLKIN(ref_clk),.CLKFB(1'b0),.CLKOUT0(core_clk),.LOCK(pll_lock),
  .CLKOUT1(peripheral_clk),.CLKOUT2(),.CLKOUT3(),.CLKOUT4(),.CLKOUT5(),.CLKOUT6(),.CLKFBOUT(),
  .ENCLK0(1'b1),.ENCLK1(1'b1),.ENCLK2(1'b0),.ENCLK3(1'b0),.ENCLK4(1'b0),.ENCLK5(1'b0),.ENCLK6(1'b0),
  .FBDSEL(6'd0),.IDSEL(6'd0),.MDSEL(7'd0),.MDSEL_FRAC(3'd0),
  .ODSEL0(7'd0),.ODSEL0_FRAC(3'd0),.ODSEL1(7'd0),.ODSEL2(7'd0),.ODSEL3(7'd0),.ODSEL4(7'd0),.ODSEL5(7'd0),.ODSEL6(7'd0),
  .DT0(4'd0),.DT1(4'd0),.DT2(4'd0),.DT3(4'd0),.PSSEL(3'd0),.PSDIR(1'b0),.PSPULSE(1'b0),
  .SSCPOL(1'b0),.SSCON(1'b0),.SSCMDSEL(7'd0),.SSCMDSEL_FRAC(3'd0),
  .ICPSEL(6'd0),.LPFRES(3'd0),.LPFCAP(2'd0),.RESET(1'b0),.PLLPWD(1'b0),.RESET_I(1'b0),.RESET_O(1'b0));
 reg [16:0] lock_count=0;reg qualified=0,ever_ready=0,failed=0;
 // All monitor state is in the actual25MHz core domain. The reference clock
 // only feeds the PLL; no50MHz->25MHz data/reset release crossing remains.
 // Async assertion captures even a short unlock while core_clk is stopped.
 // Two-stage release holds the counter at0 during raw LOCK reset release.
 reg [1:0] lock_release=0;
 always @(posedge core_clk or negedge pll_lock)begin
  if(!pll_lock)lock_release<=0;else lock_release<={lock_release[0],1'b1};
 end
 always @(posedge core_clk or negedge pll_lock)begin
  if(!pll_lock)begin lock_count<=0;qualified<=0;end
  else if(!lock_release[1])begin lock_count<=0;qualified<=0;end
  else if(!qualified)begin
   if(lock_count==49999)qualified<=1;
   else lock_count<=lock_count+1'b1;
  end
 end
 // Once qualified, a lost clock clears qualified asynchronously. On the first
 // resumed core edge failed latches before the new50000-cycle qualification.
 always @(posedge core_clk)begin
  if(qualified)ever_ready<=1;
  if(ever_ready&&!qualified)failed<=1;
 end
 reg [1:0] ready_sync=0,fault_sync=0;
 always @(posedge core_clk or negedge qualified)begin
  if(!qualified)ready_sync<=0;else ready_sync<={ready_sync[0],1'b1};
 end
 always @(posedge core_clk)fault_sync<={fault_sync[0],failed};
 assign ready=ready_sync[1];assign fault=fault_sync[1];
 // Loss immediately removes physical enable even if core_clk has stopped.
 // failed is sticky until reconfiguration; LOCK recovery never resumes motion.
 assign output_allowed=pll_lock&&qualified&&!failed;
endmodule
