# B114: relocate with this directory; no machine-specific paths.
set source_root [file normalize [file dirname [info script]]]
set build_root [file join $source_root build]
if {[file exists $build_root]} {error "build already exists; use a fresh project copy for a new build"}
file mkdir [file join $build_root work]
cd [file join $build_root work]
create_project -force -name furuta_enabled -dir [file join $build_root gowin] -pn GW5AT-LV138PG484AC1/I0 -device_version B
add_file [file join $source_root {rtl/fixed_math.sv}]
add_file [file join $source_root {rtl/motion_metadata.sv}]
add_file [file join $source_root {rtl/adc128s102.sv}]
add_file [file join $source_root {rtl/angle_observer.sv}]
add_file [file join $source_root {rtl/command_link.sv}]
add_file [file join $source_root {rtl/commissioning.sv}]
add_file [file join $source_root {rtl/commutation_phase.sv}]
add_file [file join $source_root {rtl/controlled_drop.sv}]
add_file [file join $source_root {rtl/current_dose.sv}]
add_file [file join $source_root {rtl/encoder_as5048.sv}]
add_file [file join $source_root {rtl/encoder_pendulum.sv}]
add_file [file join $source_root {rtl/foc_core.sv}]
add_file [file join $source_root {rtl/furuta_top.sv}]
add_file [file join $source_root {rtl/home_controller.sv}]
add_file [file join $source_root {rtl/isqrt64.sv}]
add_file [file join $source_root {rtl/motor_control.sv}]
add_file [file join $source_root {rtl/pendulum_control.sv}]
add_file [file join $source_root {rtl/planned_motion.sv}]
add_file [file join $source_root {rtl/pwm3.sv}]
add_file [file join $source_root {rtl/rebase_transaction.sv}]
add_file [file join $source_root {rtl/sampling_frontend.sv}]
add_file [file join $source_root {rtl/spi_master.sv}]
add_file [file join $source_root {rtl/supervisor.sv}]
add_file [file join $source_root {rtl/telemetry.sv}]
add_file [file join $source_root {rtl/trig_lut.sv}]
add_file [file join $source_root {rtl/uart.sv}]
add_file [file join $source_root {rtl/udiv64.sv}]
add_file [file join $source_root {rtl/upright_arrival.sv}]
add_file [file join $source_root {rtl/velocity_servo.sv}]
add_file [file join $source_root {variants/furuta_enabled.sv}]
file mkdir [file join [pwd] rom]
file copy -force [file join $source_root {rom/boost_q16.hex}] [file join [pwd] rom {boost_q16.hex}]
file copy -force [file join $source_root {rom/drop_2khz_q16.hex}] [file join [pwd] rom {drop_2khz_q16.hex}]
file copy -force [file join $source_root {rom/motion_q16_p20.hex}] [file join [pwd] rom {motion_q16_p20.hex}]
file copy -force [file join $source_root {rom/servo_gains_q16.hex}] [file join [pwd] rom {servo_gains_q16.hex}]
file copy -force [file join $source_root {rom/servo_tanh_q16.hex}] [file join [pwd] rom {servo_tanh_q16.hex}]
file copy -force [file join $source_root {rom/sine_q16.hex}] [file join [pwd] rom {sine_q16.hex}]
file copy -force [file join $source_root {rom/voltage_cap_q16.hex}] [file join [pwd] rom {voltage_cap_q16.hex}]
add_file [file join $source_root {constraints/acg720_adc_din.cst}]
add_file [file join $source_root {constraints/acg720_enabled_din10.sdc}]
add_file [file join $source_root {rtl/sensor_velocity.sv}]
add_file [file join $source_root {rtl/offline_keys.sv}]
add_file [file join $source_root {rtl/offline_link.sv}]
add_file [file join $source_root {rtl/offline_sequence.sv}]
add_file [file join $source_root {rtl/fpga_clock_25.sv}]
add_file [file join $source_root {rtl/fpga_clock_25_50.sv}]
add_file [file join $source_root {rtl/scheme_a_mailbox.sv}]
add_file [file join $source_root {rtl/scheme_a_peripherals.sv}]
add_file [file join $source_root {rtl/furuta_scheme_a_top.sv}]
add_file [file join $source_root {rtl/board_display.sv}]
set_option -top_module furuta_enabled
set_option -synthesis_tool gowinsynthesis
set_option -verilog_std sysv2017
set_option -dsp_style dsp
set_option -rom_style block_rom
set_option -global_freq 50
set_option -route_option 2
set_option -correct_hold_violation 1
set_option -gen_sdf 1
set_option -gen_verilog_sim_netlist 1
run syn
run pnr
