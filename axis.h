/*
 * Class to track all the data for a particular machine axis
 */

#pragma once

#include "loggable.h"

class axis : public LogStream
{
public:
    // Allows -ve movement to -max_d
    static constexpr unsigned int AXIS_BIDIRECTIONAL = (1 << 0);

    // Wraps at max_d and max_steps in both directions
    static constexpr unsigned int AXIS_ROTATION_WRAP = (1 << 1);

    // Internal position units are integer microns
    // External units are meters
    static constexpr unsigned int dist_scale = 1000000;

    enum class moveStatus
    {
        NO_MOVE = 0,
        VALID,
        OUT_OF_BOUNDS,
    };

    struct stepperMove
    {
        unsigned int jerk;
        int total_steps;
        unsigned int s_duration_us;
        unsigned int ka_duration_us;
        moveStatus status;
        int delta_int; // Internal units
    };

    axis(
        char _letter,
        double _mj,
        double _ma,
        double _mv,
        double _md,
        double _steps_per_d,
        double _cmd_scale,
        double _home_spfrac = 0.25,
        unsigned int _flags = 0);

    void set_pos(double val, bool set_step = false);
    double get_pos() { return double(cur_pos) / double(dist_scale); }
    double get_max_pos() { return max_d; }
    void normalise_pos();

    void set_step_pos(int val);
    int get_step_pos() { return cur_steps; }

    double get_home_speed() { return home_speed_frac; }
    double get_cmd_scale() { return cmd_scale; }

    stepperMove get_move(double tgt_pos, double speed_frac);
    void commit_move(const stepperMove &move);
    void handle_remainder(int remain);

    char letter;

private:
    double max_j, max_a, max_v, max_d;
    double steps_per_d;
    double cmd_scale; // Gcode units are scaled up by this much
    double home_speed_frac;
    unsigned int flags;

    int cur_pos;     // This is microns
    int cur_steps;   // This counts microsteps, not full steps
    int step_offset; // Position of metric zero in steps

    int min_steps, max_steps;

    int last_delta;
};
