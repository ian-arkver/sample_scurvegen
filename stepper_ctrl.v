/*
 * (C) 2020 Ian Jamison <ian.dev@arkver.com>
 *
 * Moves a stepper a programmed number of steps using an
 * S-curve fixed jerk algorithm.
 *
 * License: BSD 3 Clause
 */

module stepper_ctrl #(
    parameter PULSE_WIDTH_COUNT = 9'd75,   // 1.5us step pulse default (50Mhz clock)
    parameter HAS_ENABLE_ZERO = 0,      // Set this to enable "use_zero" control
    parameter DUAL_ZERO = 0,            // Set this for push head with two zero switches
    parameter ACC_SHIFT = 12,           // bit shift for acc before speed
    parameter SPD_SHIFT = 12,           // bit shift for speed before step accum
    parameter JERK_WIDTH = 9,           // width of jerk register
    parameter ACC_WIDTH = 26,           // width of accel counter
    parameter SPD_WIDTH = 30,           // width of speed counter
    parameter STEP_WIDTH = 20,          // width of step accumulator
    parameter STEP_THRESH = 'd381250)   // Step accum needed to trigger a step
(
    input   clk,
    input   clken_1meg,
    input   reset,
    input   estop,  // Emergency stop button
    input   swstop, // Software controlled abort-move
    input   [DUAL_ZERO:0] zero,   // Hardware endstop
    output reg move_done = 0,

    input   spi_write,
    input [3:0] spi_waddr,
    input [15:0] spi_wdata,
    input [3:0] spi_raddr,
    output reg [15:0] spi_rdata = 0,

    output reg drv_dir = 0,
    output reg drv_step = 0
);

reg [JERK_WIDTH-1:0] jerk = 0;
reg dir = 0;                      // NB zero-stop only checked when dir=0
reg zero_clear = 0;               // Move until zero is deasserted (only in +ve dir)
reg zero_stop = 0;                // Enable stop at zero (only used if HAS_ENABLE_ZERO)
reg [19:0] total_steps = 0;       // Round phases up so this is the real limit
reg [19:0] c_jerk_dur = 0;        // Phases 1, 3, 5 and 7
reg [19:0] c_accel_dur = 0;       // Phases 2 and 6
reg do_move = 0;

reg [19:0] steps_left = 0;
reg busy = 0;

always @(posedge clk)
    if (reset)
    begin
        jerk <= 0;
        dir <= 0;
        zero_clear <= 0;
        zero_stop <= 0;
        total_steps <= 0;
        c_jerk_dur <= 0;
        c_accel_dur <= 0;
    end
    else
    if (spi_write)
    begin
        case (spi_waddr)
        4'h0:   {zero_stop, zero_clear, dir, jerk} <= spi_wdata[JERK_WIDTH+2:0];
        4'h1:   total_steps[15:0] <= spi_wdata;
        4'h2:   c_jerk_dur[15:0]  <= spi_wdata;
        4'h3:   c_accel_dur[15:0] <= spi_wdata;
        4'h4:   {c_accel_dur[19:16], c_jerk_dur[19:16],
                 total_steps[19:16]} <= spi_wdata[11:0];
        endcase
    end

always @(posedge clk)
    do_move <= ~reset & spi_write & spi_wdata[0] & (spi_waddr == 4'h5);

localparam
    st_Idle = 4'h0,
    st_Phase1 = 4'h1,
    st_Phase2 = 4'h2,
    st_Phase3 = 4'h3,
    st_Phase4 = 4'h4,
    st_Phase5 = 4'h5,
    st_Phase6 = 4'h6,
    st_Phase7 = 4'h7,
    st_Done   = 4'hf;
reg [3:0] state = st_Idle;

always @(posedge clk)
begin
    spi_rdata <= 0;

    case (spi_raddr)
    4'h0:   spi_rdata[JERK_WIDTH+2:0] <= {zero_stop, zero_clear, dir, jerk};
    4'h1:   spi_rdata <= total_steps[15:0];
    4'h2:   spi_rdata <= steps_left[15:0];
    4'h3:   spi_rdata <= c_accel_dur[15:0];
    4'h4:   spi_rdata <= {c_accel_dur[19:16], steps_left[19:16],
                          total_steps[19:16]};
    4'h5:   spi_rdata <= {busy, 3'b0, state, 8'b0};
    endcase
end

// Allow movement away from endstop zero. Optional "stop when clear" mode.
// For head rotate and push steppers, allow moves to ignore zero markers.
// For the push stepper, the active stop depends on which direction the
// rotation is in and whether you're trying to clear the stop.
reg stop = 0;
reg did_last_step = 0;
wire zero_sel;
generate
    if (DUAL_ZERO)
    begin : g_zsel
        assign zero_sel = (dir ^ zero_clear) ? zero[0] : zero[1];
    end
    else
    begin : g_zsel
        assign zero_sel = zero[0];
    end
endgenerate

generate
    if (HAS_ENABLE_ZERO)
    begin : stops
        always @(posedge clk)
            stop <= reset | estop | swstop | did_last_step |
                (zero_stop & (zero_sel ^ zero_clear));
    end
    else
    begin : stops
        always @(posedge clk)
            stop <= reset | estop | swstop | did_last_step |
                (dir ? ~zero_sel & zero_clear : zero_sel);
    end
endgenerate

wire steps_done = ~|steps_left;
reg [19:0] phase_count = 0;
reg phase_done = 0;

/*
 * To figure out how many coast steps we need:
 * s_steps = total - left   (at the end of phase 3)
 * coast = total - 2 * s_steps  (we'll do the same number while decelerating)
 *       = total - 2 * total + 2 * left
 *       = 2*left - total
 * Add one to round up and ensure we don't stop moving with one step left
 */
wire [20:0] coast_steps_needed = {steps_left, 1'b1} - {1'b0, total_steps};
wire [19:0] coast_steps = {20{~coast_steps_needed[20]}} & coast_steps_needed[19:0];

reg motor_stopped = 0;
reg do_step_held = 0;

always @(posedge clk)
    phase_done <= ~|phase_count;

reg was_busy = 0;
always @(posedge clk)
begin
    if (do_move)
        busy <= 1'b1;
    else
    if (stop | (state == st_Done))
        busy <= 1'b0;

    did_last_step <= ~reset & (state != st_Idle) & steps_done;

    was_busy <= busy;

    // Flag move done even if it's immediately stopped
    move_done <= ~reset & (was_busy & ~busy);
end

// Main S-curve phase state machine
always @(posedge clk)
    if (stop)
    begin
        state <= st_Idle;
        phase_count <= 0;
    end
    else
    case (state)
    st_Idle:
        if (do_move)
        begin
            state <= st_Phase1;
            phase_count <= c_jerk_dur;
        end

    st_Phase1:
        if (clken_1meg)
        begin
            if (phase_done)
                state <= st_Phase2;

            if (phase_done)
                phase_count <= c_accel_dur;
            else
                phase_count <= phase_count - 1'b1;
        end

    st_Phase2:
        if (clken_1meg)
        begin
            if (phase_done)
                state <= st_Phase3;

            if (phase_done)
                phase_count <= c_jerk_dur;
            else
                phase_count <= phase_count - 1'b1;
        end

    st_Phase3:
        if (clken_1meg)
        begin
            if (phase_done)
                state <= st_Phase4;

            if (phase_done)
                phase_count <= coast_steps;
            else
                phase_count <= phase_count - 1'b1;
        end

    st_Phase4:
        begin
            // Hack to prevent HW bug from locking up state - abort with all steps remaining
            if (motor_stopped)
                state <= st_Done;
            else if (clken_1meg & phase_done)
            begin
                state <= st_Phase5;
                phase_count <= c_jerk_dur;
            end
            else if (do_step_held)
                phase_count <= phase_count - 1'b1;
        end

    st_Phase5:
        if (clken_1meg)
        begin
            if (phase_done)
                state <= st_Phase6;

            if (phase_done)
                phase_count <= c_accel_dur;
            else
                phase_count <= phase_count - 1'b1;
        end

    st_Phase6:
        if (clken_1meg)
        begin
            if (phase_done)
                state <= st_Phase7;

            if (phase_done)
                phase_count <= c_jerk_dur;
            else
                phase_count <= phase_count - 1'b1;
        end

    st_Phase7:
        begin
            /*
             * If the speed drops to zero and we have steps left, that's a fault.
             * Return to idle and leave the remaining count visible.
             */
            if (motor_stopped)
                state <= st_Done;
            else
            if (clken_1meg & phase_done)
                state <= st_Done;

            if (clken_1meg & ~phase_done)
                phase_count <= phase_count - 1'b1;
        end

    // st_Done:
    default:
        state <= st_Idle;

    endcase

reg [ACC_WIDTH-1:0] cur_accel = 0;

always @(posedge clk)
    if (stop)
        cur_accel <= 0;
    else
    case (state)
    st_Idle:   cur_accel <= 0;
    st_Phase1,
    st_Phase5:  // cur_accel +ve for both accel and decel
        if (clken_1meg & ~phase_done)
            cur_accel <= cur_accel + {1'b0, jerk};
    st_Phase3,
    st_Phase7:
        if (clken_1meg & ~phase_done)
            cur_accel <= cur_accel - {1'b0, jerk};
        else
        if (cur_accel[ACC_WIDTH-1])  // Prevent accel from going negative
            cur_accel <= 0;
    endcase

reg [SPD_WIDTH-1:0] cur_speed = 0;

always @(posedge clk)
    if (stop)
    begin
        cur_speed <= 0;
        motor_stopped <= 0;
    end
    else
    case (state)
    st_Idle:
        if (do_move)
        begin
            cur_speed <= 0;
            motor_stopped <= 0;
        end

    st_Phase1,
    st_Phase2,
    st_Phase3:
        if (clken_1meg & ~phase_done)
            cur_speed <= cur_speed + {1'b0, cur_accel[ACC_WIDTH-1:ACC_SHIFT]};

    st_Phase4:
        motor_stopped <= ~|cur_speed;

    st_Phase5,
    st_Phase6,
    st_Phase7:
        if (clken_1meg) // Decrement on last tick of prev phase to match values in accel phase
            cur_speed <= cur_speed - {1'b0, cur_accel[ACC_WIDTH-1:ACC_SHIFT]};
        else
        if (cur_speed[SPD_WIDTH-1] | (~|cur_speed[SPD_WIDTH-2:SPD_SHIFT]))
        begin
            cur_speed <= 0;
            motor_stopped <= 1'b1;
        end

    st_Done:
        cur_speed <= 0;

    endcase

wire [31:0] step_thresh = STEP_THRESH;
wire [STEP_WIDTH-1:0] full_step_thresh = step_thresh[STEP_WIDTH-1:0];
wire [STEP_WIDTH-1:0] half_step_thresh = {1'b0, step_thresh[STEP_WIDTH-1:1]};

reg [STEP_WIDTH-1:0] step_accum = 0;
reg clken_1meg_h = 0;
wire do_step = clken_1meg_h & (step_accum >= full_step_thresh);

always @(posedge clk)
begin
    clken_1meg_h <= clken_1meg;
    do_step_held <= do_step & ~stop & ~steps_done;
end

always @(posedge clk)
    if (stop)
        step_accum <= half_step_thresh;
    else
    case (state)
    st_Idle:
        step_accum <= half_step_thresh;

    st_Phase1,
    st_Phase2,
    st_Phase3,
    st_Phase4,
    st_Phase5,
    st_Phase6,
    st_Phase7:
        if (clken_1meg)
            step_accum <= step_accum + {1'b0, cur_speed[SPD_WIDTH-1:SPD_SHIFT]};
        else
        if (do_step_held)
            step_accum <= step_accum - full_step_thresh;
    endcase

// Don't clear this on stop so SW can see how many were left
always @(posedge clk)
    if (reset)
        steps_left <= 0;
    else
    if ((state == st_Idle) & do_move)
        steps_left <= total_steps;
    else
    if (do_step_held)
        steps_left <= steps_left - 1'b1;

reg [8:0] step_timer = 0;
wire step_timer_nonzero = |step_timer;

// Don't clear this on stop to avoid narrow pulses. Rising edge will have triggered a step anyway.
always @(posedge clk)
    if (reset)
        step_timer <= 0;
    else
    if (do_step_held)
        step_timer <= PULSE_WIDTH_COUNT;
    else
    if (step_timer_nonzero)
        step_timer <= step_timer - 1'b1;

always @(posedge clk)
    drv_step <= step_timer_nonzero;

always @(posedge clk)
    if (reset)
        drv_dir <= 0;
    else
    if ((state == st_Idle) & do_move)
        drv_dir <= dir;

endmodule
