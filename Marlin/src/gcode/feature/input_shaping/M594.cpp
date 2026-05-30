/**
 * M594: Frequency Sweep
 *
 * This command is used to perform a frequency sweep to determine resonant frequencies.
 */
#include "../../../inc/MarlinConfig.h"
#include "../../gcode.h"

#if EITHER(INPUT_SHAPING_X, INPUT_SHAPING_Y)

#include "../../../module/planner.h"
#include "../../../module/stepper.h"
#include "../../../module/motion.h"
#include "../../../MarlinCore.h" // For idle()

// Store original settings
static float old_max_acceleration_x, old_max_acceleration_y;
static float old_max_feedrate_x, old_max_feedrate_y;
static float old_print_acceleration;
#if ENABLED(CLASSIC_JERK)
  static float old_jerk_x, old_jerk_y;
#else
  static float old_junction_deviation;
#endif
#if ENABLED(INPUT_SHAPING_X)
  static float old_shaping_freq_x;
#endif
#if ENABLED(INPUT_SHAPING_Y)
  static float old_shaping_freq_y;
#endif

void M594_setup() {
    SERIAL_ECHOLNPGM("M594: Preparing for frequency sweep...");

    // Save settings
    old_max_acceleration_x = planner.settings.max_acceleration_mm_per_s2[X_AXIS];
    old_max_acceleration_y = planner.settings.max_acceleration_mm_per_s2[Y_AXIS];
    old_print_acceleration = planner.settings.acceleration;
    SERIAL_ECHOPGM(" > Saved Max Acceleration: X=");
    SERIAL_ECHO(old_max_acceleration_x);
    SERIAL_ECHOPGM(" Y=");
    SERIAL_ECHOLN(old_max_acceleration_y);
    SERIAL_ECHOPGM(" > Saved print Acceleration:");
    SERIAL_ECHOLN(old_print_acceleration);

    old_max_feedrate_x = planner.settings.max_feedrate_mm_s[X_AXIS];
    old_max_feedrate_y = planner.settings.max_feedrate_mm_s[Y_AXIS];
    SERIAL_ECHOPGM(" > Saved Max Feedrate: X=");
    SERIAL_ECHO(old_max_feedrate_x);
    SERIAL_ECHOPGM(" Y=");
    SERIAL_ECHOLN(old_max_feedrate_y);

    #if ENABLED(CLASSIC_JERK)
    old_jerk_x = planner.max_jerk.x;
    old_jerk_y = planner.max_jerk.y;
    SERIAL_ECHOPGM(" > Saved Jerk: X=");
    SERIAL_ECHO(old_jerk_x);
    SERIAL_ECHOPGM(" Y=");
    SERIAL_ECHOLN(old_jerk_y);
    #else
    old_junction_deviation = planner.junction_deviation_mm;
    SERIAL_ECHOPGM(" > Saved Junction Deviation: ");
    SERIAL_ECHOLN(old_junction_deviation);
    #endif

  #if ENABLED(INPUT_SHAPING_X)
    old_shaping_freq_x = stepper.get_shaping_frequency(X_AXIS);
    SERIAL_ECHOPGM(" > Saved Input Shaping Freq X: ");
    SERIAL_ECHOLN(old_shaping_freq_x);
  #endif
  #if ENABLED(INPUT_SHAPING_Y)
    old_shaping_freq_y = stepper.get_shaping_frequency(Y_AXIS);
    SERIAL_ECHOPGM(" > Saved Input Shaping Freq Y: ");
    SERIAL_ECHOLN(old_shaping_freq_y);
  #endif

  // Apply overrides
  planner.set_max_acceleration(X_AXIS, 20000.0f);
  planner.set_max_acceleration(Y_AXIS, 20000.0f);
  SERIAL_ECHOPGM(" > Set Max Acceleration: X=");
  SERIAL_ECHO(planner.settings.max_acceleration_mm_per_s2[X_AXIS]);
  SERIAL_ECHOPGM(" Y=");
  SERIAL_ECHOLN(planner.settings.max_acceleration_mm_per_s2[Y_AXIS]);

  planner.set_max_feedrate(X_AXIS, 500.0f);
  planner.set_max_feedrate(Y_AXIS, 500.0f);
  SERIAL_ECHOPGM(" > Set Max Feedrate: X=");
  SERIAL_ECHO(planner.settings.max_feedrate_mm_s[X_AXIS]);
  SERIAL_ECHOPGM(" Y=");
  SERIAL_ECHOLN(planner.settings.max_feedrate_mm_s[Y_AXIS]);

  #if ENABLED(CLASSIC_JERK)
    planner.set_max_jerk(X_AXIS, 0.0f);
    planner.set_max_jerk(Y_AXIS, 0.0f);
    SERIAL_ECHOPGM(" > Set Jerk: X=");
    SERIAL_ECHO(planner.max_jerk.x);
    SERIAL_ECHOPGM(" Y=");
    SERIAL_ECHOLN(planner.max_jerk.y);
  #else
    planner.junction_deviation_mm = 5.0f;
    SERIAL_ECHOPGM(" > Set Junction Deviation: ");
    SERIAL_ECHOLN(planner.junction_deviation_mm);
  #endif

  #if ENABLED(INPUT_SHAPING_X)
    stepper.set_shaping_frequency(X_AXIS, 0.0f);
    SERIAL_ECHOPGM(" > Disabled Input Shaping X. New value: ");
    SERIAL_ECHOLN(stepper.get_shaping_frequency(X_AXIS));
  #endif
  #if ENABLED(INPUT_SHAPING_Y)
    stepper.set_shaping_frequency(Y_AXIS, 0.0f);
    SERIAL_ECHOPGM(" > Disabled Input Shaping Y. New value: ");
    SERIAL_ECHOLN(stepper.get_shaping_frequency(Y_AXIS));
  #endif

  #if ENABLED(S_CURVE_ACCELERATION)
    planner.settings.s_curve_enabled = false;
    SERIAL_ECHOLNPGM(" > Disabled S-Curve Acceleration");
  #endif
}

void M594_sweep(int axis) {
    if (axis == 0) {
        SERIAL_ECHOLNPGM("M594: Sweeping X axis...");
    } else {
        SERIAL_ECHOLNPGM("M594: Sweeping Y axis...");
    }

    const float start_freq = 5.0f;
    const float end_freq = 50.0f;
    const float accel_per_hz = 120.0f;
    const float hz_per_sec = 1.0f;

    float current_freq = start_freq;
    float sign = 1.0f;

    // Capture current center position
    xyze_pos_t center_pos = current_position;

    while (current_freq <= end_freq) {
        // Wait if the planner buffer is nearly full (leave a margin of 2 blocks)
        while (planner.movesplanned() >= BLOCK_BUFFER_SIZE - 2) {
            idle();
        }

        // Quarter-wave math
        const float t_seg = 0.25f / current_freq;
        const float accel = accel_per_hz * current_freq;

        // CRITICAL: Update the max acceleration for THIS specific frequency segment
        planner.set_max_acceleration(axis == 0 ? X_AXIS : Y_AXIS, accel);
        planner.settings.acceleration = accel;

        // Klipper's total displacement for the half-cycle (accel phase + decel phase)
        // 2 * (0.5 * accel * t_seg^2) = accel * t_seg^2
        const float dist = accel * sq(t_seg);
        const float velocity = accel * t_seg;

        // Calculate target oscillating around the center position
        xyze_pos_t target_pos = center_pos;
        if (axis == 0) {
            target_pos.x += dist * sign;
        } else {
            target_pos.y += dist * sign;
        }

        // Inject directly into the planner
        planner.buffer_line(target_pos, velocity, active_extruder);

        // Advance loop
        current_position = target_pos;
        current_freq += 2.0f * t_seg * hz_per_sec;
        sign = -sign; // Reverse direction for the next stroke
    }

    // Ensure all injected moves complete
    planner.synchronize();
}

void M594_teardown() {
    SERIAL_ECHOLNPGM("M594: Tearing down frequency sweep...");

    // Restore settings
    planner.set_max_acceleration(X_AXIS, old_max_acceleration_x);
    planner.set_max_acceleration(Y_AXIS, old_max_acceleration_y);
    planner.settings.acceleration = old_print_acceleration;
    SERIAL_ECHOPGM(" > Restored Max Acceleration: X=");
    SERIAL_ECHO(planner.settings.max_acceleration_mm_per_s2[X_AXIS]);
    SERIAL_ECHOPGM(" Y=");
    SERIAL_ECHOLN(planner.settings.max_acceleration_mm_per_s2[Y_AXIS]);
    SERIAL_ECHOPGM(" > Restored print Acceleration: X=");
    SERIAL_ECHOLN(planner.settings.acceleration);

  planner.set_max_feedrate(X_AXIS, old_max_feedrate_x);
  planner.set_max_feedrate(Y_AXIS, old_max_feedrate_y);
  SERIAL_ECHOPGM(" > Restored Max Feedrate: X=");
  SERIAL_ECHO(planner.settings.max_feedrate_mm_s[X_AXIS]);
  SERIAL_ECHOPGM(" Y=");
  SERIAL_ECHOLN(planner.settings.max_feedrate_mm_s[Y_AXIS]);

  #if ENABLED(CLASSIC_JERK)
    planner.set_max_jerk(X_AXIS, old_jerk_x);
    planner.set_max_jerk(Y_AXIS, old_jerk_y);
    SERIAL_ECHOPGM(" > Restored Jerk: X=");
    SERIAL_ECHO(planner.max_jerk.x);
    SERIAL_ECHOPGM(" Y=");
    SERIAL_ECHOLN(planner.max_jerk.y);
  #else
    planner.junction_deviation_mm = old_junction_deviation;
    SERIAL_ECHOPGM(" > Restored Junction Deviation: ");
    SERIAL_ECHOLN(planner.junction_deviation_mm);
  #endif

  #if ENABLED(INPUT_SHAPING_X)
    stepper.set_shaping_frequency(X_AXIS, old_shaping_freq_x);
    SERIAL_ECHOPGM(" > Restored Input Shaping Freq X: ");
    SERIAL_ECHOLN(stepper.get_shaping_frequency(X_AXIS));
  #endif
  #if ENABLED(INPUT_SHAPING_Y)
    stepper.set_shaping_frequency(Y_AXIS, old_shaping_freq_y);
    SERIAL_ECHOPGM(" > Restored Input Shaping Freq Y: ");
    SERIAL_ECHOLN(stepper.get_shaping_frequency(Y_AXIS));
  #endif

  #if ENABLED(S_CURVE_ACCELERATION)
    planner.settings.s_curve_enabled = true;
    SERIAL_ECHOLNPGM(" > Re-enabled S-Curve Acceleration");
  #endif
}

void GcodeSuite::M594() {
  if (!parser.seenval('S')) {
    SERIAL_ECHOLNPGM("M594 requires S parameter (S0 for X, S1 for Y)");
    return;
  }

  const int axis = parser.value_int();
  if (axis != 0 && axis != 1) {
    SERIAL_ECHOLNPGM("Invalid axis. Use S0 for X, S1 for Y");
    return;
  }

  planner.synchronize();
  M594_setup();
  M594_sweep(axis);
  planner.synchronize();
  M594_teardown();
}

#else

void GcodeSuite::M594() {
  SERIAL_ECHOLNPGM("M594 requires INPUT_SHAPING_X or INPUT_SHAPING_Y.");
}

#endif
