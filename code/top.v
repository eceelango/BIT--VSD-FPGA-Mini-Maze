`include "ultra_sonic_sensor.v"

module top (
  input  wire echo1, echo2, echo3, // Right, Front, Left ultrasonic sensors
  output wire trig1, trig2, trig3,
  output reg AIN1, AIN2, BIN1, BIN2, // Motor direction control
  output wire PWMA, PWMB,            // Motor speed control
  output wire STBY,                  // Motor driver standby
  output reg led1, led2, led3        // Debug LEDs
);

assign STBY = 1'b1; // Motor always enabled

// -----------------------------------------
// 1. Internal Oscillator (6 MHz)
// -----------------------------------------
wire int_osc;
SB_HFOSC #(.CLKHF_DIV("0b11")) u_SB_HFOSC (
  .CLKHFPU(1'b1),
  .CLKHFEN(1'b1),
  .CLKHF(int_osc)
);

// -----------------------------------------
// 2. Ultrasonic Triggering & Sensing
// -----------------------------------------
wire measure1, measure2, measure3;
refresher250ms refresher1 (.clk(int_osc), .en(1'b1), .measure(measure1));
refresher250ms refresher2 (.clk(int_osc), .en(1'b1), .measure(measure2));
refresher250ms refresher3 (.clk(int_osc), .en(1'b1), .measure(measure3));

wire [15:0] raw1, raw2, raw3;
hc_sr04 sensor1 (.clk(int_osc), .measure(measure1), .echo(echo1), .trig(trig1), .distance_cm(raw1)); // Right
hc_sr04 sensor2 (.clk(int_osc), .measure(measure2), .echo(echo2), .trig(trig2), .distance_cm(raw2)); // Front
hc_sr04 sensor3 (.clk(int_osc), .measure(measure3), .echo(echo3), .trig(trig3), .distance_cm(raw3)); // Left

// -----------------------------------------
// 3. Moving Average Filter (3 samples) - RESTORED
// -----------------------------------------
reg [15:0] sum1 = 0, sum2 = 0, sum3 = 0;
reg [15:0] dist1 = 0, dist2 = 0, dist3 = 0;
reg [1:0] count1 = 0, count2 = 0, count3 = 0;

always @(posedge int_osc) begin
  if (measure1 && raw1 < 200) begin
    sum1 <= sum1 + raw1;
    count1 <= count1 + 1;
    if (count1 == 3) begin
      dist1 <= (sum1 + raw1) >> 2;
      sum1 <= 0; count1 <= 0;
    end
  end

  if (measure2 && raw2 < 200) begin
    sum2 <= sum2 + raw2;
    count2 <= count2 + 1;
    if (count2 == 3) begin
      dist2 <= (sum2 + raw2) >> 2;
      sum2 <= 0; count2 <= 0;
    end
  end

  if (measure3 && raw3 < 200) begin
    sum3 <= sum3 + raw3;
    count3 <= count3 + 1;
    if (count3 == 3) begin
      dist3 <= (sum3 + raw3) >> 2;
      sum3 <= 0; count3 <= 0;
    end
  end
end

// -----------------------------------------
// 3.1 Out-of-range check
// -----------------------------------------
parameter MAX_DIST = 16'd200;
reg out_of_range = 0;

always @(posedge int_osc) begin
  if (dist1 > MAX_DIST || dist2 > MAX_DIST || dist3 > MAX_DIST)
    out_of_range <= 1;
  else
    out_of_range <= 0;
end

// -----------------------------------------
// 4. PWM Motor Speed Control with Bilateral PD
// -----------------------------------------
parameter BASE = 90;        // Increased base speed
parameter WALLP = 18;
parameter WALLD = 10;
parameter MAXCORR = 30;
parameter TURN_PWM = 30;
parameter PWM_MAX=90;
parameter PWM_MIN=30;    // Increased turn speed

reg signed [15:0] err = 0, prev_err = 0, corr = 0;
reg [7:0] pwm_left = BASE, pwm_right = BASE;
reg [7:0] pwm_counter = 0;
reg pwm_a = 0, pwm_b = 0;

reg [23:0] pd_blink_counter = 0;
reg led2_state = 0;

always @(posedge int_osc) begin
  if (!turning && !wait_turn && AIN1 == 0 && AIN2 == 1 && BIN1 == 0 && BIN2 == 1) begin
    // Bilateral wall-following: balance between right and left walls
    err <= dist1 - dist3;
    if (err > -2 && err < 2)
      corr <= 0;
    else
      corr <= ((WALLP * err + WALLD * (err - prev_err)) >>> 2);

    prev_err <= err;

    if (corr > MAXCORR) corr <= MAXCORR;
    if (corr < -MAXCORR) corr <= -MAXCORR;

    if (corr > 0) begin
      pwm_left  <= (BASE + (corr >>> 1) > PWM_MAX) ? PWM_MAX : BASE + (corr >>> 1);
      pwm_right <= (BASE - corr < PWM_MIN) ? PWM_MIN : BASE - corr;
    end else begin
      pwm_left  <= (BASE + corr < PWM_MIN) ? PWM_MIN : BASE + corr;
      pwm_right <= (BASE - (corr >>> 1) > PWM_MAX) ? PWM_MAX : BASE - (corr >>> 1);
    end

    pd_blink_counter <= pd_blink_counter + (corr[15] ? -corr : corr);
    if (pd_blink_counter > 100000) begin
      led2_state <= ~led2_state;
      pd_blink_counter <= 0;
    end
  end else begin
    pwm_left  <= TURN_PWM;
    pwm_right <= TURN_PWM;
    led2_state <= 0;
    pd_blink_counter <= 0;
  end
end

always @(posedge int_osc) begin
  pwm_counter <= pwm_counter + 1;
  pwm_a <= (pwm_counter < pwm_left);
  pwm_b <= (pwm_counter < pwm_right);
end

assign PWMA = pwm_a;
assign PWMB = pwm_b;

// -----------------------------------------
// 5. Movement Logic - Flag-based FSM
// -----------------------------------------
parameter THRESH = 16'd4;           // Adjusted threshold
parameter HYST = 16'd2;
parameter TURN_TIME = 24'd3600000;   // Adjusted for 6 MHz: ~0.6 seconds
parameter UTURN_TIME = 24'd6000000;  // Adjusted for 6 MHz: ~1.0 seconds
parameter MARGIN = 16'd4;

reg [23:0] turn_timer = 0;
reg turning = 0;
reg [1:0] dir = 0;
reg wait_turn = 0;
reg [23:0] pre_turn_delay = 0;

always @(posedge int_osc) begin
  led1 <= (dist2 < THRESH);
  led2 <= led2_state;
  led3 <= (dist3 >= (THRESH + HYST + MARGIN));

  if (turning) begin
    turn_timer <= turn_timer + 1;
    case (dir)
      2'b01: begin 
               AIN1 <= 0; AIN2 <= 1; 
               BIN1 <= 1; BIN2 <= 0; end // Right turn
      2'b10: begin
               AIN1 <= 1; AIN2 <= 0; 
               BIN1 <= 0; BIN2 <= 1; end // Left turn
      2'b11: begin 
               AIN1 <= 0; AIN2 <= 1;
               BIN1 <= 1; BIN2 <= 0; end // U-turn
      default: begin
                AIN1 <= 0; AIN2 <= 0; 
                BIN1 <= 0; BIN2 <= 0; end
    endcase
    if (turn_timer >= ((dir == 2'b11) ? UTURN_TIME : TURN_TIME)) begin
      turning <= 0;
      turn_timer <= 0;
    end
  end else begin
    if (out_of_range) begin
      AIN1 <= 0; AIN2 <= 0;
      BIN1 <= 0; BIN2 <= 0;
    end else if (!wait_turn) begin
      if ((dist1 >= (THRESH + HYST + MARGIN) && dist3 <= (THRESH + HYST + MARGIN) && dist2 <= (THRESH + HYST + MARGIN))||
          (dist1 >= (THRESH + HYST + MARGIN) && dist3 <= (THRESH + HYST + MARGIN) && dist2 >= (THRESH + HYST + MARGIN))||
          (dist1 >= (THRESH + HYST + MARGIN) && dist3 >= (THRESH + HYST + MARGIN) && dist2 >= (THRESH + HYST + MARGIN))||
          (dist1 >= (THRESH + HYST + MARGIN) && dist3 >= (THRESH + HYST + MARGIN) && dist2 <= (THRESH + HYST + MARGIN ))) begin
          wait_turn <= 1;
          dir <= 2'b01;
          pre_turn_delay <= 0; // Turn right
      end 
      
      else if (dist1 <= (THRESH + HYST + MARGIN) && dist3 <= (THRESH + HYST + MARGIN) && dist2 >= (THRESH + HYST + MARGIN)) begin
          AIN1 <= 0; AIN2 <= 1;
          BIN1 <= 0; BIN2 <= 1;//go forward 
      end
      else if (dist1 <= (THRESH + HYST + MARGIN) && dist3 >= (THRESH + HYST + MARGIN) && dist2 >= (THRESH + HYST + MARGIN)) begin
          AIN1 <= 0; AIN2 <= 1;
          BIN1 <= 0; BIN2 <= 1;//go forward 
      end 
      else if (dist1 <= (THRESH + HYST + MARGIN) && dist3 >= (THRESH + HYST + MARGIN) && dist2 <= (THRESH + HYST + MARGIN)) begin
          wait_turn <= 1; 
          dir <= 2'b10;
          pre_turn_delay <= 0; // Turn left
      end  
      else if (dist1 <= (THRESH + HYST + MARGIN) && dist3 >= (THRESH + HYST + MARGIN) && dist2 <= (THRESH + HYST + MARGIN))begin
          wait_turn <= 1;
          dir <= 2'b11;
          pre_turn_delay <= 0; // U-turn if both blocked
      end
      end
       else begin
      // Wait state before turning
      AIN1 <= 0; AIN2 <= 0;
      BIN1 <= 0; BIN2 <= 0;
      pre_turn_delay <= pre_turn_delay + 1;
      if (pre_turn_delay >= 24'd3000_000) begin // 0.5  second delay at 6 MHz
        turning <= 1;
        wait_turn <= 0;
        pre_turn_delay <= 0;
        turn_timer <= 0;
      end
    end
  end
end

endmodule
