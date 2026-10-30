package org.firstinspires.ftc.teamcode.robot;
public class RobotConfig {

    // Timing constants (seconds)
    public static final double FIRE_TIME  = 0.3;

    // Distance-based power constants
    public static final double MIN_DISTANCE    = 50.0;
    public static final double MAX_DISTANCE    = 140.0;
    public static final double MIN_LAUNCH_POWER = 0.7;
    public static final double MAX_LAUNCH_POWER = 0.945;
    // - turret constants need to be tuned
    // actually make that everything needs to be tuned
    public static final double TURRET_TICK_TOLERANCE = 2;
    public static final double TURRET_HEADING_TOLERANCE = 0.05;
    public static final double TURRET_LIMIT_TICK_TOLERANCE = 10;
    public static final double MAX_TURRET_POWER = 0.9;
    public static final double MAX_TURRET_TICKS = 100;
    public static final double MIN_TURRET_TICKS = -100;

    // Trigger / gate -- TODO tune on the robot
    public static final double TRIGGER_POWER   = 1.0;   // feed power once the shot is good
    public static final double GATE_OPEN_POS   = 0.6;   // gate servo position, open
    public static final double GATE_CLOSED_POS = 0.2;   // gate servo position, closed
    public static final double GATE_OPEN_TIME  = 0.15;  // seconds for the gate to swing open before feeding

    // Intake / storage hardware config names
    public static final String INTAKE_MOTOR        = "intake_motor";
    public static final String FLOWER_RAMP_SERVO   = "flower_ramp_servo";
    public static final String INTAKE_COLOR_SENSOR = "intake_color";
    public static final String INTAKE_BEAM_LOW     = "intake_beam_low";   // any element breaks it
    public static final String INTAKE_BEAM_HIGH    = "intake_beam_high";  // ~8 cm up: above POLLEN, below NECTAR
    public static final String STORAGE_EXIT_BEAM   = "storage_exit_beam"; // elements leaving into the launcher

    // Intake -- TODO tune on the robot
    public static double INTAKE_POWER         = 0.8;
    public static double OUTTAKE_POWER        = -0.6;
    public static double INTAKE_SLOW_POWER    = 0.4;
    public static double INTAKE_JITTER_PERIOD = 0.15;  // updateIntake(jitter): flip direction this often (s)

    // Flower ramp servo positions -- TODO tune on the robot
    public static double FLOWER_RAMP_LOWERED   = 0.8;
    public static double FLOWER_RAMP_RETRACTED = 0.2;

    // Storage
    public static final int STORAGE_CAPACITY = 4;      // G407: may not CONTROL more than 4 SCORING ELEMENTS
    public static double STORAGE_DEBOUNCE_TIME = 0.12; // seconds between counted elements
    public static double EMERGENCY_MIN_OUTTAKE = 0.25; // emergency outtake runs at least this long (s)
    public static double EMERGENCY_MAX_OUTTAKE = 1.5;  // and gives up after this (s)
    // Hue windows in degrees [0, 360). POLLEN is yellow, NECTAR red or blue. TODO tune under event lighting
    public static double POLLEN_HUE_MIN = 35.0;
    public static double POLLEN_HUE_MAX = 75.0;
    public static double BLUE_NECTAR_HUE_MIN = 190.0;
    public static double BLUE_NECTAR_HUE_MAX = 255.0;
    // Red wraps around 0: anything >= MIN or <= MAX
    public static double RED_NECTAR_HUE_MIN = 335.0;
    public static double RED_NECTAR_HUE_MAX = 20.0;
    public static double STORAGE_MIN_SATURATION = 0.25;


}
