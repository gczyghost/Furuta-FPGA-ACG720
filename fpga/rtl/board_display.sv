// ACG720: 2441BB common-anode, U24 digit selects -> U25 segments.
// Read-only observer in the control clock domain. No new clock or control path.
module board_display #(
 parameter integer CLOCK_HZ=25000000,
 SAMPLE_CLKS=CLOCK_HZ/10, CHANNEL_CLKS=CLOCK_HZ*2,
 SLOT_CLKS=CLOCK_HZ/1000, SERIAL_HALF=CLOCK_HZ/500000
)(
 input wire clk,rst,angle_valid,offline,running,fault,activity,
 input wire signed [31:0] arm_q16,pend_q16,
 output reg [7:0] LED,
 output reg SEG7_DIO,SEG7_SCLK,SEG7_RCLK
);
 localparam integer SW=$clog2(SAMPLE_CLKS)<1?1:$clog2(SAMPLE_CLKS);
 localparam integer CW=$clog2(CHANNEL_CLKS)<1?1:$clog2(CHANNEL_CLKS);
 localparam integer TW=$clog2(SLOT_CLKS)<1?1:$clog2(SLOT_CLKS);
 localparam integer HW=$clog2(SERIAL_HALF)<1?1:$clog2(SERIAL_HALF);
 reg [SW-1:0] sample_count;
 reg [CW-1:0] channel_count;
 reg channel;
 reg [2:0] phase;
 reg [5:0] step;
 reg [25:0] factor;
 reg [57:0] product,addend;
 reg negative,work_channel,work_valid;
 reg [16:0] binary;
 reg [19:0] bcd;
 reg [19:0] bcd_adjusted;
 reg [63:0] pending_frame,scan_frame;
 reg pending_valid,pending_channel,scan_valid,scan_channel;
 reg pending_overflow,scan_overflow;
 wire [31:0] selected=channel?pend_q16:arm_q16;
 wire [31:0] magnitude=selected[31]?(~selected+32'd1):selected;
 // round(abs(Q16 radians) * 1800/pi / 65536) -> tenths of a degree.
 // 37549362 / 2^32 approximates that ratio to <0.0001deg at full display range.
 wire [25:0] rounded=product[57:32]+{25'd0,product[31]};
 integer i;
 function automatic [7:0] digit(input [3:0] n);
  case(n)
   0:digit=8'hc0;1:digit=8'hf9;2:digit=8'ha4;3:digit=8'hb0;4:digit=8'h99;
   5:digit=8'h92;6:digit=8'h82;7:digit=8'hf8;8:digit=8'h80;9:digit=8'h90;
   default:digit=8'hbf;
  endcase
 endfunction
 // Left->right A/P, sign, four integer digits, tenths, degree symbol.
 function automatic [63:0] number_frame(input ch,neg,input [19:0] digits);
  begin
   number_frame={ch?8'h8c:8'h88,neg?8'hbf:8'hff,
    digits[19:16]!=0?digit(digits[19:16]):8'hff,
    digits[19:12]!=0?digit(digits[15:12]):8'hff,
    digits[19:8]!=0?digit(digits[11:8]):8'hff,
    digit(digits[7:4])&8'h7f,digit(digits[3:0]),8'h9c};
  end
 endfunction
 function automatic [63:0] invalid_frame(input ch);
  invalid_frame={ch?8'h8c:8'h88,8'hff,8'hbf,8'hbf,8'hbf,8'hbf,8'hbf,8'h9c};
 endfunction
 always @*begin
  bcd_adjusted=bcd;
  for(i=0;i<5;i=i+1)if(bcd[i*4+:4]>=5)bcd_adjusted[i*4+:4]=bcd[i*4+:4]+4'd3;
 end
 always @(posedge clk)begin
  if(rst)begin
   sample_count<=0;channel_count<=0;channel<=0;phase<=0;step<=0;
   factor<=0;product<=0;addend<=0;negative<=0;work_channel<=0;work_valid<=0;
   binary<=0;bcd<=0;pending_frame<=invalid_frame(0);pending_valid<=0;pending_channel<=0;pending_overflow<=0;
   LED<=0;
  end else begin
   // LED0 online;1 offline;2 run;3 fault;4 valid displayed angle;
   // 5 local sequence active;6 displayed pendulum channel;7 slow heartbeat.
   LED<={channel_count>=CHANNEL_CLKS/2,scan_channel,activity,
    angle_valid&&scan_valid,fault,running,offline,!offline};
   if(channel_count==CHANNEL_CLKS-1)begin channel_count<=0;channel<=!channel;end
   else channel_count<=channel_count+1'b1;
   if(sample_count==SAMPLE_CLKS-1)sample_count<=0;
   else sample_count<=sample_count+1'b1;
   if(!angle_valid)begin pending_valid<=0;pending_overflow<=0;work_valid<=0;end
   case(phase)
    0:if(sample_count==0)begin
     work_channel<=channel;work_valid<=angle_valid;pending_overflow<=0;negative<=selected[31];
     factor<=26'd37549362;addend<={26'd0,magnitude};product<=0;step<=0;phase<=1;
    end
    1:begin
     if(factor[0])product<=product+addend;
     factor<=factor>>1;addend<=addend<<1;
     if(step==25)phase<=2;else step<=step+1'b1;
    end
    2:begin
     if(!work_valid||!angle_valid)begin pending_frame<=invalid_frame(work_channel);pending_valid<=0;pending_channel<=work_channel;phase<=0;end
     else if(rounded>99999)begin
      // Overflow is explicit, not wrapped/clamped to an apparently valid angle.
      pending_frame<={work_channel?8'h8c:8'h88,8'hff,8'hc0,8'h8e,8'hc7,8'hc0,8'hff,8'hff};
      pending_valid<=0;pending_overflow<=1;pending_channel<=work_channel;phase<=0;
     end else begin binary<=rounded[16:0];bcd<=0;step<=0;negative<=negative&&(rounded!=0);phase<=3;end
    end
    3:begin
     bcd<={bcd_adjusted[18:0],binary[16]};binary<=binary<<1;
     if(step==16)phase<=4;else step<=step+1'b1;
    end
    4:begin
     pending_frame<=number_frame(work_channel,negative,bcd);
     pending_valid<=work_valid&&angle_valid;pending_channel<=work_channel;phase<=0;
    end
    default:phase<=0;
   endcase
  end
 end
 // Each 1ms digit slot begins with a complete all-off latch, then a lit latch.
 // Serial clock=250kHz, data changes on falling edges; latch separated from SCLK.
 // Whole frame changes only at digit0 so channel/sign/value cannot tear.
 reg [TW-1:0] slot_count;
 reg [HW-1:0] half_count;
 reg [2:0] scan_digit,serial_state;
 reg [4:0] bit_count;
 reg [15:0] shift;
 reg lit_transfer;
 wire [63:0] visible_frame=(angle_valid&&(scan_valid||scan_overflow))?scan_frame:invalid_frame(scan_channel);
 wire [7:0] scan_segments=visible_frame[scan_digit*8+:8];
 always @(posedge clk)begin
  if(rst)begin
   slot_count<=0;half_count<=0;scan_digit<=7;serial_state<=0;bit_count<=0;
   shift<=16'hff00;lit_transfer<=0;SEG7_DIO<=1;SEG7_SCLK<=0;SEG7_RCLK<=0;
   scan_frame<=invalid_frame(0);scan_valid<=0;scan_channel<=0;scan_overflow<=0;
  end else begin
   if(!angle_valid)begin scan_valid<=0;scan_overflow<=0;end
   if(slot_count==SLOT_CLKS-1)slot_count<=0;else slot_count<=slot_count+1'b1;
   if(serial_state==0)begin
    if(slot_count==0)begin
     scan_digit<=scan_digit+1'b1;
     if(scan_digit==7)begin
      scan_frame<=pending_frame;scan_valid<=pending_valid&&angle_valid;scan_channel<=pending_channel;scan_overflow<=pending_overflow&&angle_valid;
     end
     shift<=16'hff00;SEG7_DIO<=1;SEG7_RCLK<=0;SEG7_SCLK<=0;
     bit_count<=0;half_count<=0;lit_transfer<=0;serial_state<=1;
    end
   end else if(half_count==SERIAL_HALF-1)begin
    half_count<=0;
    case(serial_state)
     1:begin SEG7_SCLK<=1;serial_state<=2;end
     2:begin
      SEG7_SCLK<=0;
      if(bit_count==15)serial_state<=3;
      else begin shift<=shift<<1;SEG7_DIO<=shift[14];bit_count<=bit_count+1'b1;serial_state<=1;end
     end
     3:begin SEG7_RCLK<=1;serial_state<=4;end
     4:begin
      SEG7_RCLK<=0;
      if(lit_transfer)serial_state<=0;
      else begin
       shift<={scan_segments,8'b1<<scan_digit};SEG7_DIO<=scan_segments[7];
       bit_count<=0;lit_transfer<=1;serial_state<=1;
      end
     end
     default:serial_state<=0;
    endcase
   end else half_count<=half_count+1'b1;
  end
 end
endmodule
