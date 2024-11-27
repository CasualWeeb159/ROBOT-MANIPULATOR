#include "../gcode.h"
#include "../../module/motion.h"

abc_pos_t robot_angles;
void GcodeSuite::G7() {
    inverse_kinematics(current_position);
    robot_angles.a = delta.a;
    robot_angles.b = delta.b;
    robot_angles.c = delta.c;

    if (parser.seenval('A')) robot_angles.a = parser.value_float();
    if (parser.seenval('B')) robot_angles.b = parser.value_float();
    if (parser.seenval('C')) robot_angles.c = parser.value_float();

    SERIAL_ECHOLNPGM("Robot angles set to A:", robot_angles.a, " B:", robot_angles.b, " C:", robot_angles.c);
    direct_angle_change(robot_angles);

}