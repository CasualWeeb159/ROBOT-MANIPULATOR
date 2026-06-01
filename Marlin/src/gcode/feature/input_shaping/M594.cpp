#include "../../../inc/MarlinConfig.h"
#include "../../gcode.h"

#if EITHER(INPUT_SHAPING_X, INPUT_SHAPING_Y)

#include "../../../module/planner.h"
#include "../../../module/stepper.h"
#include "../../../module/motion.h"
#include "../../../MarlinCore.h"

constexpr float SWEEP_START_FREQ      = 1.0f;
constexpr float SWEEP_END_FREQ        = 20.0f;
constexpr float SWEEP_HZ_PER_SEC      = 0.5f;
constexpr float RESONANCE_DURATION_S  = 2.5f;
constexpr float ACCEL_PER_HZ          = 100.0f;

constexpr float TEST_MAX_ACCEL        = 20000.0f;
constexpr float TEST_MAX_FEEDRATE     = 500.0f;
constexpr float TEST_JUNCTION_DEV     = 0.05f;

static float old_max_acceleration_x, old_max_acceleration_y;
static float old_max_feedrate_x, old_max_feedrate_y;
static float old_print_acceleration;
static float old_travel_acceleration;
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

void M594_setup(const bool shaping_on) {
  SERIAL_ECHOLNPGM("M594: Preparing for frequency sweep...");

  old_max_acceleration_x = planner.settings.max_acceleration_mm_per_s2[X_AXIS];
  old_max_acceleration_y = planner.settings.max_acceleration_mm_per_s2[Y_AXIS];
  old_print_acceleration = planner.settings.acceleration;
  old_travel_acceleration = planner.settings.travel_acceleration;
  old_max_feedrate_x = planner.settings.max_feedrate_mm_s[X_AXIS];
  old_max_feedrate_y = planner.settings.max_feedrate_mm_s[Y_AXIS];

  #if ENABLED(CLASSIC_JERK)
    old_jerk_x = planner.max_jerk.x;
    old_jerk_y = planner.max_jerk.y;
  #else
    old_junction_deviation = planner.junction_deviation_mm;
  #endif

  #if ENABLED(INPUT_SHAPING_X)
    old_shaping_freq_x = stepper.get_shaping_frequency(X_AXIS);
  #endif
  #if ENABLED(INPUT_SHAPING_Y)
    old_shaping_freq_y = stepper.get_shaping_frequency(Y_AXIS);
  #endif

  planner.set_max_acceleration(X_AXIS, TEST_MAX_ACCEL);
  planner.set_max_acceleration(Y_AXIS, TEST_MAX_ACCEL);
  planner.set_max_feedrate(X_AXIS, TEST_MAX_FEEDRATE);
  planner.set_max_feedrate(Y_AXIS, TEST_MAX_FEEDRATE);

  #if ENABLED(CLASSIC_JERK)
    planner.set_max_jerk(X_AXIS, 1.0f);
    planner.set_max_jerk(Y_AXIS, 1.0f);
  #else
    planner.junction_deviation_mm = TEST_JUNCTION_DEV;
  #endif

  if (shaping_on) {
    SERIAL_ECHOLNPGM(" > Input shaping: ON");
  } else {
    SERIAL_ECHOLNPGM(" > Input shaping: OFF");
    #if ENABLED(INPUT_SHAPING_X)
      stepper.set_shaping_frequency(X_AXIS, 0.0f);
    #endif
    #if ENABLED(INPUT_SHAPING_Y)
      stepper.set_shaping_frequency(Y_AXIS, 0.0f);
    #endif
  }

  #if ENABLED(S_CURVE_ACCELERATION)
    planner.settings.s_curve_enabled = false;
  #endif
}

void M594_sweep(int axis) {
  if (axis == 0) {
    SERIAL_ECHOLNPGM("M594: Sweeping X axis...");
  } else {
    SERIAL_ECHOLNPGM("M594: Sweeping Y axis...");
  }

  float current_freq = SWEEP_START_FREQ;
  float sign = 1.0f;

  xyze_pos_t center_pos = current_position;

  while (current_freq <= SWEEP_END_FREQ) {
    while (planner.movesplanned() >= BLOCK_BUFFER_SIZE - 4) {
      idle();
    }

    const float t_seg = 0.25f / current_freq;
    const float accel = ACCEL_PER_HZ * current_freq;

    planner.set_max_acceleration(axis == 0 ? X_AXIS : Y_AXIS, accel);
    planner.settings.acceleration = accel;
    planner.settings.travel_acceleration = accel;

    const float dist = accel * sq(t_seg);
    const float velocity = accel * t_seg;

    xyze_pos_t target_pos = center_pos;
    if (axis == 0) {
      target_pos.x += dist * sign;
    } else {
      target_pos.y += dist * sign;
    }

    planner.buffer_line(target_pos, velocity, active_extruder);

    current_position = target_pos;
    current_freq += 2.0f * t_seg * SWEEP_HZ_PER_SEC;
    sign = -sign;
  }

  planner.synchronize();
}

void M594_resonate(int axis, float freq) {
  if (axis == 0) {
    SERIAL_ECHOPGM("M594: Resonating X axis at ");
  } else {
    SERIAL_ECHOPGM("M594: Resonating Y axis at ");
  }
  SERIAL_ECHO(freq);
  SERIAL_ECHOLNPGM(" Hz...");

  float total_time = 0.0f;
  float sign = 1.0f;

  xyze_pos_t center_pos = current_position;

  const float t_seg = 0.25f / freq;
  const float accel = ACCEL_PER_HZ * freq;

  planner.set_max_acceleration(axis == 0 ? X_AXIS : Y_AXIS, accel);
  planner.settings.acceleration = accel;
  planner.settings.travel_acceleration = accel;

  const float dist = accel * sq(t_seg);
  const float velocity = accel * t_seg;

  while (total_time <= RESONANCE_DURATION_S) {
    while (planner.movesplanned() >= BLOCK_BUFFER_SIZE - 4) {
      idle();
    }

    xyze_pos_t target_pos = center_pos;
    if (axis == 0) {
      target_pos.x += dist * sign;
    } else {
      target_pos.y += dist * sign;
    }

    planner.buffer_line(target_pos, velocity, active_extruder);

    current_position = target_pos;
    total_time += 2.0f * t_seg;
    sign = -sign;
  }

  planner.synchronize();
}

void M594_teardown() {
  SERIAL_ECHOLNPGM("M594: Restoring original settings...");

  planner.set_max_acceleration(X_AXIS, old_max_acceleration_x);
  planner.set_max_acceleration(Y_AXIS, old_max_acceleration_y);
  planner.settings.acceleration = old_print_acceleration;
  planner.settings.travel_acceleration = old_travel_acceleration;
  planner.set_max_feedrate(X_AXIS, old_max_feedrate_x);
  planner.set_max_feedrate(Y_AXIS, old_max_feedrate_y);

  #if ENABLED(CLASSIC_JERK)
    planner.set_max_jerk(X_AXIS, old_jerk_x);
    planner.set_max_jerk(Y_AXIS, old_jerk_y);
  #else
    planner.junction_deviation_mm = old_junction_deviation;
  #endif

  #if ENABLED(INPUT_SHAPING_X)
    stepper.set_shaping_frequency(X_AXIS, old_shaping_freq_x);
  #endif
  #if ENABLED(INPUT_SHAPING_Y)
    stepper.set_shaping_frequency(Y_AXIS, old_shaping_freq_y);
  #endif

  #if ENABLED(S_CURVE_ACCELERATION)
    planner.settings.s_curve_enabled = true;
  #endif

  SERIAL_ECHOLNPGM("M594: Finished.");
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

  float freq = 0.0f;
  if (parser.seenval('F')) {
    freq = parser.value_float();
  }

  bool shaping_on = false;
  if (parser.seenval('P')) {
    shaping_on = (parser.value_int() == 1);
  }

  planner.synchronize();
  M594_setup(shaping_on);

  if (freq > 0.0f) {
    M594_resonate(axis, freq);
  } else {
    M594_sweep(axis);
  }

  planner.synchronize();
  M594_teardown();
  planner.reset_small_move_alert();
}

#else

void GcodeSuite::M594() {
  SERIAL_ECHOLNPGM("M594 requires INPUT_SHAPING_X or INPUT_SHAPING_Y.");
}

#endif