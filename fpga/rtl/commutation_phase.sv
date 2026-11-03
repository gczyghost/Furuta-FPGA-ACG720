// Explicit modulo-65536 electrical angle. Keep the small 14x6 product in
// fabric: Gowin 1.9.12.03 fused the previous conditional +/- multiply into
// MULTALU27X18 with the direction selection reversed in the synthesized netlist.
module commutation_phase(input wire [13:0] raw,input wire [5:0] poles,
 input wire reverse,input wire [15:0] electrical_zero,output wire [15:0] phase);
 wire [19:0] product /* synthesis syn_dspstyle = "logic" */;
 wire [15:0] forward_phase={product[13:0],2'b00};
 wire [15:0] directed_phase=reverse ? (16'd0-forward_phase):forward_phase;
 assign product=raw*poles;
 assign phase=directed_phase-electrical_zero;
endmodule
