#include "../gcode.h"

/*
M50 S0 ... přepne obě brzdy na driver (automaticky)
    S1 ... přepne obě brzdy na zdroj 24V (odbržděno)

M50.1 S1 ... přepne brzdu hlavního ramene na driver (automaticky)
      S0 ... přepne brzdu hlavního ramene na zdroj 24V (odbržděno)

M50.2 S1 ... přepne brzdu vedlejšího ramene na driver (automaticky)
      S0 ... přepne brzdu vedlejšího ramene na zdroj 24V (odbržděno)
*/

byte PE7_state;         //stav pinu PE7 --> 0 ... brzdy hlavního ramene napojeny na driver   (odbržděny když jsou napájeny i motory)
                        //                  1 ... brzdy hlavního ramene napojeny na zdroj    (pořád odbržděny)
byte PE8_state;         //stav pinu PE7 --> 0 ... brzdy vedlejšího ramene napojeny na driver (odbržděny když jsou napájeny i motory)
                        //                  1 ... brzdy vedlejšího ramene napojeny na zdroj  (pořád odbržděny)

extern bool break_command_pending;

void GcodeSuite::M50() {

    PE7_state = extDigitalRead(PE7);
    PE8_state = extDigitalRead(PE8);

    const uint8_t subcode_M50 = parser.subcode;

    if (!parser.seenval('S')) return;
    
    const byte pin_status = (parser.value_byte() == 0) ? HIGH : LOW;
    
    switch (subcode_M50){
        default: return;
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
    
    if (break_command_pending == false) {
        SERIAL_ECHOLNPGM("Není zadán žádný příkaz M50.");
        return;
    } //pokud nečeká žádný povel na potvrzení, funkce nic neudělá

    extDigitalWrite(PE7,PE7_state);
    extDigitalWrite(PE8,PE8_state);

    break_command_pending = false;
}