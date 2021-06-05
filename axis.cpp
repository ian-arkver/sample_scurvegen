/*
 * Class to track all the data for a particular machine axis
 */

#include <cmath>

#include "axis.h"

axis::axis(char _letter,
           double _mj,
           double _ma,
           double _mv,
           double _md,
           double _steps_per_d,
           double _cmd_scale,
           double _home_spfrac,
           unsigned int _flags)
    : letter(_letter),
      max_j(_mj),
      max_a(_ma),
      max_v(_mv),
      max_d(_md),
      steps_per_d(_steps_per_d),
      cmd_scale(_cmd_scale),
      home_speed_frac(_home_spfrac),
      flags(_flags)
{
    cur_pos = 0;
    cur_steps = 0;
    step_offset = 0;

    max_steps = round(max_d * steps_per_d);
    if (flags & AXIS_BIDIRECTIONAL)
        min_steps = -max_steps;
    else
        min_steps = 0;

    last_delta = 0;
}

void axis::set_pos(double val, bool set_step)
{
    cur_pos = round(val * dist_scale);
    int val_steps = round(val * steps_per_d);
    if (set_step)
    {
        cur_steps = val_steps;
        step_offset = 0;
    }
    else
    {
        step_offset = cur_steps - val_steps;
    }
}

void axis::set_step_pos(int val)
{
    cur_steps = val;
}

/*
 * Rotational axes wrap around at 0/360 degrees.
 */
void axis::normalise_pos()
{
    if (!(flags & AXIS_ROTATION_WRAP))
        return;

    if (cur_pos >= max_d)
        cur_pos -= max_d;
    else if (cur_pos < 0)
        cur_pos += max_d;

    if (cur_steps >= max_steps)
        cur_steps -= max_steps;
    else if (cur_steps < 0)
        cur_steps += max_steps;
}

struct fpMove
{
    double j;
    double t0, t1, t2, thalf;
    double xt2;
};

struct fpMove
calcMove(double max_j, double max_a, double max_v, double speed_frac, double dist)
{
    struct fpMove move;
    // Hardware needs integer jerk value
    max_j = round(max_j * speed_frac);
    max_a *= speed_frac;
    max_v *= speed_frac;

    move.j = max_j;

    double half_dist = dist / 2.0;

    double t0 = max_a / max_j;
    double t1 = max_v / max_a;
    double t2 = t0 + t1;
    double vt0 = max_j * t0 * t0 / 2.0;
    double xt0 = max_j * t0 * t0 * t0 / 6.0;
    double vt1 = max_a * (t1 - t0) + vt0;
    double xt1 = max_a * (t1 - t0) * (t1 - t0) / 2.0 + vt0 * (t1 - t0) + xt0;
    double xt2 = -max_j * t0 * t0 * t0 / 6.0 + max_a * t0 * t0 / 2.0 + vt1 * t0 + xt1;

    if (half_dist < xt2)
    {
        /* Full S-curve is too many steps. Calc slower max_v version. */
        max_v = max_a * max_a / 2.0 / max_j * (sqrt(1.0 + 8.0 * max_j * max_j / (max_a * max_a * max_a) * half_dist) - 1.0);
        /* Recalc timings based on this new max_v */
        t1 = max_v / max_a;
        /* If const-accel portion no longer exists, need to adjust max_a */
        if (t1 < t0)
        {
            max_a = std::cbrt(max_j * max_j * half_dist);
            max_v = max_a * max_a / max_j;
            t0 = max_a / max_j;
            t1 = t0;
        }
        t2 = t0 + t1;

        vt0 = max_j * t0 * t0 / 2.0;
        xt0 = max_j * t0 * t0 * t0 / 6.0;
        vt1 = max_a * (t1 - t0) + vt0;
        xt1 = max_a * (t1 - t0) * (t1 - t0) / 2.0 + vt0 * (t1 - t0) + xt0;
        xt2 = -max_j * t0 * t0 * t0 / 6.0 + max_a * t0 * t0 / 2.0 + vt1 * t0 + xt1;
    }

    move.t0 = t0;
    move.t1 = t1;
    move.t2 = t2;
    move.thalf = t2 + (half_dist - xt2) / max_v;
    move.xt2 = xt2;
    return move;
}

axis::stepperMove axis::get_move(double tgt_pos, double speed_frac)
{
    axis::stepperMove move = {};

    if (flags & AXIS_ROTATION_WRAP)
    {
        // Normalise to a single turn
        double turns = tgt_pos / max_d;
        double int_turns;
        double turn_frac = modf(turns, &int_turns);
        if (turn_frac < 0.0)
            turn_frac += 1.0;
        tgt_pos = turn_frac * max_d;
    }

    // Clip to max unit limits
    if (tgt_pos > max_d)
    {
        move.status = moveStatus::OUT_OF_BOUNDS;
        tgt_pos = max_d;
    }
    else if ((flags & AXIS_BIDIRECTIONAL) && (tgt_pos < -max_d))
    {
        move.status = moveStatus::OUT_OF_BOUNDS;
        tgt_pos = -max_d;
    }

    // Clip to max step limts after offset applied (-ve dir only)
    int tgt_step = round(tgt_pos * steps_per_d) + step_offset;
    if (tgt_step < min_steps && !(flags & AXIS_ROTATION_WRAP))
    {
        move.status = moveStatus::OUT_OF_BOUNDS;
        tgt_step = min_steps;
        tgt_pos = double(-step_offset) / steps_per_d;
    }

    int tgt_int = round(tgt_pos * dist_scale);
    move.delta_int = tgt_int - cur_pos;

    if (flags & AXIS_ROTATION_WRAP)
    {
        // Pick the shortest way round the circle
        double half_turn = 0.5 * max_d;
        int half_turn_int = round(half_turn * dist_scale);
        if (move.delta_int > half_turn_int)
            move.delta_int = move.delta_int - (2 * half_turn_int);
        else if (move.delta_int < -half_turn_int)
            move.delta_int = (2 * half_turn_int) + move.delta_int;

        tgt_int = cur_pos + move.delta_int;
        tgt_pos = double(tgt_int) / dist_scale;
        tgt_step = round(tgt_pos * steps_per_d) + step_offset;
    }

    double dist = double(move.delta_int) / dist_scale;
    move.total_steps = tgt_step - cur_steps;

    if (move.total_steps == 0)
        return move;

    auto fp_move = calcMove(max_j, max_a, max_v, speed_frac, fabs(dist));

    move.jerk = round(fp_move.j);
    move.s_duration_us = ceil(fp_move.t0 * 1000000);
    move.ka_duration_us = round((fp_move.t1 - fp_move.t0) * 1000000);

    // These are now only needed for debug
    int s_steps = trunc(fp_move.xt2 * steps_per_d);
    int coast_steps = abs(move.total_steps) - (2 * s_steps);
    if (coast_steps < 0)
        coast_steps = 0;
    coast_steps++;

    DBG() << "Move " << dist << ", " << move.total_steps << " steps. kJ=" << move.jerk << " for "
          << fp_move.t0 << " sec, kA for " << (fp_move.t1 - fp_move.t0) << " sec. S steps="
          << s_steps << ", coast=" << coast_steps << " steps\n";

    move.status = moveStatus::VALID;
    return move;
}

void axis::commit_move(const stepperMove &move)
{
    last_delta = move.total_steps;
    cur_steps += move.total_steps;
    cur_pos += move.delta_int;
}

void axis::handle_remainder(int remain)
{
    if (last_delta < 0)
        remain = -remain;
    int remain_int = round(double(remain) / steps_per_d * dist_scale);
    cur_steps -= remain;
    cur_pos -= remain_int;
}
