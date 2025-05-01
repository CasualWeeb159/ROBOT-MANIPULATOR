#include "../gcode.h"

/*
M50 S0 ... přepne obě brzdy na driver (automaticky)
    S1 ... přepne obě brzdy na zdroj 24V (odbržděno)

M50.1 S1 ... přepne brzdu hlavního ramene na driver (automaticky)
      S0 ... přepne brzdu hlavního ramene na zdroj 24V (odbržděno)

M50.1 S1 ... přepne brzdu vedlejšího ramene na driver (automaticky)
      S0 ... přepne brzdu vedlejšího ramene na zdroj 24V (odbržděno)
*/

bool PE7_state = false; //stav pinu PE7 --> 0 ... brzdy hlavního ramene napojeny na driver   (odbržděny když jsou napájeny i motory)
                        //                  1 ... brzdy hlavního ramene napojeny na zdroj    (pořád odbržděny)
bool PE8_state = false; //stav pinu PE7 --> 0 ... brzdy vedlejšího ramene napojeny na driver (odbržděny když jsou napájeny i motory)
                        //                  1 ... brzdy vedlejšího ramene napojeny na zdroj  (pořád odbržděny)

extern bool break_command_pending;

void GcodeSuite::M50() {
    const uint8_t subcode_M50 = parser.subcode;

    if (!parser.seenval('S')) return;
    const byte pin_status = parser.value_byte();
    
    switch (subcode_M50){
        case 0:{
            PE7_state = pin_status;
            PE8_state = pin_status;
            break;
        }
        case 1:{
            PE7_state = pin_status;
            break;
        }
        case 2:{
            PE8_state = pin_status;
            break;
        }
    }

    SERIAL_ECHOLNPGM("Prosím potvrdtě změnu příkazem M51");

    break_command_pending = true;

}

//Příkaz na potvrzení změny zdroje pro brzdy
void GcodeSuite::M51() {

    pinMode(71,OUTPUT);
    pinMode(72,OUTPUT);

    extDigitalWrite(71,PE7_state);
    extDigitalWrite(72,PE8_state);

    break_command_pending = false;
}