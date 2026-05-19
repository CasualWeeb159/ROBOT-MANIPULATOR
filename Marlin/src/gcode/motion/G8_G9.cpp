#include "../gcode.h"
#include "../../module/motion.h"
#include "../../lcd/marlinui.h"

// Tyto proměnné z jádra Marlinu jsou klíčové pro správný chod
extern xyze_pos_t destination; 


/**
 * G8: POLÁRNÍ PTP NÁJEZD (Plynulý, rychlý, po oblouku)
 */
void GcodeSuite::G8() {
    destination = current_position;
    get_destination_from_command(); // Načte F a Z

    float current_A_rad = atan2(current_position.y, current_position.x);
    float current_R = HYPOT(current_position.x, current_position.y); // OPTIMALIZACE
    float target_A_rad = current_A_rad;
    float target_R = current_R;
    bool polar_changed = false;

    // OPRAVA PARSERU: Nyní to spolehlivě přečte číslo za písmenem A
    if (parser.seen('A')) {
        polar_changed = true;
        float a_deg = parser.value_float();
        float dir_multiplier = 1.0f; // Kladný směr doleva (tak jak potřebuješ)
        
        // Převedeme na radiány (pro ATC vždy používáme absolutní cílový úhel)
        target_A_rad = RADIANS(a_deg * dir_multiplier); 
    }
    
    // OPRAVA PARSERU: To samé pro poloměr R
    if (parser.seen('R')) {
        polar_changed = true;
        target_R = parser.value_float();
    }

    if (polar_changed) {
        destination.x = target_R * cos(target_A_rad);
        destination.y = target_R * sin(target_A_rad);
    }

    // Rychlý PTP přesun pomocí jádra Marlinu
    #if IS_SCARA
      prepare_fast_move_to_destination(); 
    #else
      prepare_line_to_destination();      
    #endif
}

/**
 * G9: POLÁRNÍ LINEÁRNÍ NÁJEZD (Pracovní pohyb, dokonalá přímka)
 */
void GcodeSuite::G9() {
    destination = current_position;
    get_destination_from_command(); // Načte F a Z

    float current_A_rad = atan2(current_position.y, current_position.x);
    float current_R = HYPOT(current_position.x, current_position.y); // OPTIMALIZACE
    float target_A_rad = current_A_rad;
    float target_R = current_R;
    bool polar_changed = false;

    // OPRAVA PARSERU:
    if (parser.seen('A')) {
        polar_changed = true;
        float a_deg = parser.value_float();
        float dir_multiplier = 1.0f; // Kladný směr doleva
        target_A_rad = RADIANS(a_deg * dir_multiplier); 
    }
    
    if (parser.seen('R')) {
        polar_changed = true;
        target_R = parser.value_float();
    }

    if (polar_changed) {
        destination.x = target_R * cos(target_A_rad);
        destination.y = target_R * sin(target_A_rad);
    }

    // Pomalý, vypočítaný lineární přesun
    prepare_line_to_destination();
}