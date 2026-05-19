This is an overview of the ROBOT-MANIPULATOR project, a 3-axis palletizing robot using modified Marlin firmware. It adapts SCARA kinematics for its parallel linkage system.

### Key Configuration (`Configuration.h`, `Configuration_adv.h`)
- **Board**: `BOARD_BTT_OCTOPUS_PRO_V1_0`
- **Kinematics**: `MP_SCARA` (adapted for a palletizing robot).
- **Motors**: Nema 34 with `CL86T` closed-loop drivers.
- **Homing**: Uses min-endstops and has custom homing feedrates.
- **Features**: `EEPROM_SETTINGS`, `SAVED_POSITIONS`, `CNC_COORDINATE_SYSTEMS`, `DIRECT_PIN_CONTROL`, `BABYSTEPPING`.

### Core Logic & Kinematics
- **`MarlinCore.cpp`**: Initializes hardware, including brake controls and sensor pull-ups.
- **`gcode.cpp`**: Modified to handle custom G-codes and a two-step brake command system (`M50`/`M51`).
- **`scara.cpp`/`.h`**: **Crucially, the standard SCARA math is replaced with custom kinematics for the palletizing robot.** This includes Y-axis inversion and safety checks for movement boundaries. A `kinematic_calc_failiure` flag prevents unsafe moves.

### Motion Control
- **`motion.cpp`**: High-level movement functions now check the `kinematic_calc_failiure` flag. Includes `direct_angle_change()` for joint-specific moves (`G7`).
- **`G2_G3.cpp` (Arcs)**: Arc planning now validates moves against the custom kinematic limits.
- **New G-Codes**:
    - `G7`: Direct joint control (Joint JOG).
    - `G8`/`G9`: Polar coordinate movements (PTP and Linear).
    - `M50`/`M51`: Two-step electromagnetic brake control.

### Automatic Tool Changer (ATC) - `Custom_ATC.cpp`
- Implements ATC using a Maxwell kinematic coupling.
- Manages tool data (offsets, names) and dock positions.
- **`M6`**: Orchestrates the tool change sequence, including unloading the current tool, picking up a new one, and verifying the tool ID via POGO pins.
- **`M666`**: Manages the tool library (defining offsets, names).
- **`M667`**: Master ATC command for calibration, scanning docks, and unloading tools.

### Homing & Endstops
- **`endstops.cpp`**: Overhauled to use a cycle-based debouncing mechanism for inductive sensors, preventing false triggers.
- **`G28.cpp`**: Implements a custom, multi-stage homing routine ("Algorithm 2") specifically for the palletizing robot's mechanics, handling the complex interactions between the B and C axes.

### Key Custom G-Code Commands
- **`G7`**: Joint JOG (direct angle control).
- **`G8`/`G9`**: Polar coordinate moves.
- **`G28`**: Custom homing sequence.
- **`M6`**: Automatic Tool Change.
- **`M50`/`M51`**: Brake control.
- **`M666`**: Define tool properties.
- **`M667`**: Master ATC command (calibrate, scan, unload).