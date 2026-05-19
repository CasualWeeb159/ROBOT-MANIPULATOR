#include "../gcode.h"
#include "../../module/motion.h"

void GcodeSuite::G7() {
    abc_pos_t robot_angles;
    robot_angles.a = delta.a;
    robot_angles.b = delta.b;
    robot_angles.c = delta.c;

    if (parser.seenval('A')) {
        float a_input = parser.value_float() * -1.0f; // Softwarové otočení osy A
        robot_angles.a = axis_is_relative(A_AXIS) ? robot_angles.a + a_input : a_input;
    }
    if (parser.seenval('B')) robot_angles.b = axis_is_relative(B_AXIS) ? robot_angles.b + parser.value_float() : parser.value_float();
    if (parser.seenval('C')) robot_angles.c = axis_is_relative(C_AXIS) ? robot_angles.c + parser.value_float() : parser.value_float();

    // Používej pouze pro úpravy o jednotky stupňů! 
    direct_angle_change(robot_angles);
}