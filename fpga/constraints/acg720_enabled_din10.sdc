# Offline R1 candidate; budgets are engineering assumptions, not measured PCB delays.
# See docs/r1_bringup.md and tools/audit_r1_io.py. No blanket false/multicycle paths.
create_clock -name clk_50m -period 20.000 [get_ports {CLK_50M}]
# One native PLL: related 25 MHz control and 50 MHz peripheral clocks.
create_generated_clock -name core_25m -source [get_ports {CLK_50M}] -divide_by 2 [get_pins {clocks/pll/CLKOUT0}]
create_generated_clock -name peripheral_50m -source [get_ports {CLK_50M}] -divide_by 1 [get_pins {clocks/pll/CLKOUT1}]
# Absolute registered launch-to-pin limits; source SCLK is a clock-enable signal,
# not an FPGA clock tree. ADC50 uses60ns half-cycles; input capture is20ns after rising.
set_max_delay -to [get_ports {ADC_SCLK}] 6.000
set_max_delay -to [get_ports {ADC_CSN}] 8.000
set_max_delay -to [get_ports {ADC_DIN}] 10.000
set_min_delay -to [get_ports {ADC_SCLK}] 2.000
set_min_delay -to [get_ports {ADC_DIN ADC_CSN}] 0.000
# DOUT pin -> directly sampled data register (includes endpoint setup in STA).
# Round trip target:6+2(board out)+27(ADC)+2(board in)+15+2(reserve)=54<80ns.
set_max_delay -from [get_ports {ADC_DOUT}] 15.000
set_min_delay -from [get_ports {ADC_DOUT}] 0.000
# Slow control pins: bounded propagation, NOT a receiver-specific setup assertion.
set_max_delay -to [get_ports {AS_SCK AS_MOSI AS_CSN PEND_SCK PEND_CSN ENC_OE UART_TX PWM_U PWM_V PWM_W EN_REQ}] 10.000
set_min_delay -to [get_ports {AS_SCK AS_MOSI AS_CSN PEND_SCK PEND_CSN ENC_OE UART_TX PWM_U PWM_V PWM_W EN_REQ}] 0.000
# Asynchronous inputs enter only the first stage of a two-flop synchronizer.
# These constraints bound physical input routing; arbitrary-phase metastability
# is not signed off by STA. Peripheral synchronizers use normal20ns paths.
set_max_delay -from [get_ports {AS_MISO PEND_DO UART_RX}] 10.000
set_min_delay -from [get_ports {AS_MISO PEND_DO UART_RX}] 0.000
# AS_PWM is intentionally unused, physically connected but no sampling endpoint.

# Asynchronous main-board keys: constrain pin to first synchronizer only.
# Both synchronizer stages remain on the normal25MHz core clock.
set_max_delay -from [get_ports {KEY_N[0] KEY_N[1] KEY_N[2] KEY_N[3] KEY_N[4]}] 10.000
set_min_delay -from [get_ports {KEY_N[0] KEY_N[1] KEY_N[2] KEY_N[3] KEY_N[4]}] 0.000

# Additional reports only; no numerical budget relaxation or disabled checks.
report_timing -setup -max_paths 25
report_timing -hold -max_paths 25
report_timing -recovery -max_paths 25
report_timing -removal -max_paths 25
report_exceptions -setup -max_paths 128 -max_common_paths 8
report_exceptions -hold -max_paths 128 -max_common_paths 8
report_timing -setup -from [get_pins {clocks/pll/LOCK}] -to [get_ports {EN_REQ ENC_OE PWM_U PWM_V PWM_W}] -max_paths 8

# Scheme A: all 25/50 MHz crossings remain timed; no false paths or MCP.
# ADC50 capture80ns after prior fall,40ns before next fall; CS lead120ns.
# rawLOCK-to-five-pin engineering budget20ns per user revision; preserve raw reports.

# B110 display successor only: registered 250kHz595 outputs, 2us half-cycle.
# 100ns launch-to-pad allowance leaves >=1.9us relative edge margin before
# receiver setup/hold and board skew; no change to any existing IO budget.
# LEDs have no synchronous receiver. Bound their propagation without imposing
# the unrelated sensor/control 10ns engineering target on visual indicators.
set_max_delay -to [get_ports {SEG7_DIO SEG7_SCLK SEG7_RCLK LED[0] LED[1] LED[2] LED[3] LED[4] LED[5] LED[6] LED[7]}] 100.000
set_min_delay -to [get_ports {SEG7_DIO SEG7_SCLK SEG7_RCLK LED[0] LED[1] LED[2] LED[3] LED[4] LED[5] LED[6] LED[7]}] 0.000
