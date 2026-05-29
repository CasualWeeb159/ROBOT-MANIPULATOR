/**
 * M594: Frequency Sweep
 *
 * This command is used to perform a frequency sweep to determine resonant frequencies.
 */
#include "../../../inc/MarlinConfig.h"
#include "../../gcode.h"

#if HAS_SHAPING

#include "../../../module/planner.h"
#include "../../../module/stepper.h"

// Store original settings
static float old_max_acceleration_x, old_max_acceleration_y;
static float old_max_feedrate_x, old_max_feedrate_y;
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
  SERIAL_ECHOPGM(" > Saved Max Acceleration: X=");
  SERIAL_ECHO(old_max_acceleration_x);
  SERIAL_ECHOPGM(" Y=");
  SERIAL_ECHOLN(old_max_acceleration_y);

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
    planner.set_max_jerk(X_AXIS, 100.0f);
    planner.set_max_jerk(Y_AXIS, 100.0f);
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

void M594_sweep() {
  SERIAL_ECHOLNPGM("M594: Sweep placeholder...");
  // Future sweep logic will go here
  safe_delay(1000);
}

void M594_teardown() {
  SERIAL_ECHOLNPGM("M594: Tearing down frequency sweep...");

  // Restore settings
  planner.set_max_acceleration(X_AXIS, old_max_acceleration_x);
  planner.set_max_acceleration(Y_AXIS, old_max_acceleration_y);
  SERIAL_ECHOPGM(" > Restored Max Acceleration: X=");
  SERIAL_ECHO(planner.settings.max_acceleration_mm_per_s2[X_AXIS]);
  SERIAL_ECHOPGM(" Y=");
  SERIAL_ECHOLN(planner.settings.max_acceleration_mm_per_s2[Y_AXIS]);

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
  planner.synchronize();
  M594_setup();
  M594_sweep();
  planner.synchronize();
  M594_teardown();
}

#else

void GcodeSuite::M594() {
  SERIAL_ECHOLNPGM("M594 requires INPUT_SHAPING_X or INPUT_SHAPING_Y.");
}

#endif // HAS_SHAPING