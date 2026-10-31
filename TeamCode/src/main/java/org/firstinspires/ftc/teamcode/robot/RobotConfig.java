package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
@Configurable
public class RobotConfig {

    // Hardware config names
    public static final String LIMELIGHT = "limelight";
    public static final String TURRET_SERVO_1 = "turret_servo_1";
    public static final String TURRET_SERVO_2 = "turret_servo_2";
    public static final String TURRET_ENCODER = "turret_encoder";
    public static final String TURRET_HALL = "turret_hall";
    public static final String PINPOINT = "pinpoint";
    public static final String PRE_SHOOTER_BEAM_BREAK = "pre_shooter_beam_break";
    public static final String FRONT_LEFT_MOTOR_NAME  = "front_left_motor";
    public static final String FRONT_RIGHT_MOTOR_NAME = "front_right_motor";
    public static final String BACK_LEFT_MOTOR_NAME   = "back_left_motor";
    public static final String BACK_RIGHT_MOTOR_NAME  = "back_right_motor";
    public static final String INTAKE_MOTOR_NAME      = "intake_motor";
    public static final String TRIGGER_MOTOR_NAME     = "trigger_motor";      // moves balls from intake to turret
    public static final String LAUNCH_MOTOR_NAME     = "launch_motor";       // motor with encoder plugged in
    public static final String TURRET_SERVO_NAME_1      = "turret_servo_1";     // continuous rotation (Axon) - change if your config name differs
    public static final String TURRET_SERVO_NAME_2      = "turret_servo_2";     // continuous rotation (Axon) - change if your config name differs

    public static final String HOOD_GAP_SERVO1 = "gap_servo_1";
    public static final String HOOD_GAP_SERVO2 = "gap_servo_2";
    public static final String HOOD_ANGLE_SERVO        = "hood_servo";


    public static final boolean SINGLE_GAMEPAD_MODE = false;

    public static final double STICK_DEADBAND     = 0.05;  // ignore stick values smaller than this
    public static final double TRIGGER_THRESHOLD  = 0.10;  // trigger must be pressed past this to count
    public static final double DRIVE_POWER_SCALE = 1.0;    // 1.0 = full speed, 0.5 = half speed
    public static final DcMotor.ZeroPowerBehavior DRIVE_ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.BRAKE;

    // ====================================================================================
    // 5. INTAKE AND TURRET SETTINGS
    // ====================================================================================
    public static final double INTAKE_IN_POWER  =  1.0;    // right trigger
    public static final double INTAKE_OUT_POWER = -1.0;    // left trigger

    public static final double TRIGGER_FEED_POWER    =  1.0;   // right stick pushed up
    public static final double TRIGGER_REVERSE_POWER = -1.0;   // right stick pushed down
    public static final double TRIGGER_STICK_THRESHOLD = 0.30; // stick must move past this to run the trigger motor

    public static final double TURRET_POWER = 0.5;        // CR servo power while D-pad left/right held (0 to 1)

    // ====================================================================================
    // 6. SERVO POSITION LIMITS  (apply to compression servos and the hood servo)
    //    The full 0 - 1 range is too far, so everything is clamped to this range.
    // ====================================================================================
    public static final double SERVO_MIN_POSITION = 0.0;
    public static final double SERVO_MAX_POSITION = 0.7;

    // ====================================================================================
    // 7. COMPRESSION SERVO SETTINGS  (positions must be between SERVO_MIN and SERVO_MAX)
    // ====================================================================================
    public static final double COMPRESSION_START_POSITION  = 0.0;   // where servos go on init
    public static final double COMPRESSION_OPEN_POSITION   = 0.0;   // Y button
    public static final double COMPRESSION_POLLEN_POSITION = 0.3;   // X button  (placeholder - tune me)
    public static final double COMPRESSION_NECTAR_POSITION = 0.6;   // B button  (placeholder - tune me)
    public static final double COMPRESSION_NUDGE_STEP      = 0.02;  // stick-click nudge size

    // ====================================================================================
    // 8. HOOD SERVO SETTINGS
    // ====================================================================================
    public static final double HOOD_START_POSITION    = 1.0;   // where the servo goes on init
    public static final double HOOD_SPEED_PER_SECOND  = 0.30;  // how fast D-pad up/down moves it (position units per second)

    // ====================================================================================
    // 9. LAUNCH MOTOR SETTINGS  (DcMotorEx, velocity control with PIDF tuning)
    // ====================================================================================
    // Encoder resolution of the launch motor itself (ticks per MOTOR output shaft revolution).
    // goBILDA 5202/5203 Yellow Jacket 1:1 (6000 RPM) = 28.  Check your motor's spec sheet.


    // --- PIDF TUNING VALUES ---
    // Tune in this order: F first (get actual RPM close to target), then P (tighten it up),
    // then I / D only if needed. Watch "Actual RPM" vs "Target RPM" in telemetry.
    // F default = 32767 / (max motor ticks per second), a standard starting point.
    public static final double LAUNCH_PF_P = 0.0123;
    public static final double LAUNCH_PF_F = 0.00045;
    public static final double LAUNCH_NOMINAL_VOLTAGE = 12.5;

    // FLOAT lets the flywheel coast down when switched off.
    public static final DcMotor.ZeroPowerBehavior LAUNCH_ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.FLOAT;


    // Limelight AprilTags (manual 9.9: 36h11, 3.25 in, under each HIVE cell)
    public static final int HIVE_PIPELINE = 0;
    public static final int RED_FIRST_TAG = 30;   // red cells: 30-37
    public static final int BLUE_FIRST_TAG = 38;  // blue cells: 38-45
    public static final int TAGS_PER_ALLIANCE = 8;

    // Timing constants (seconds)
    public static final double FIRE_TIME  = 0.3;

    // Distance-based power constants
    public static final double MIN_DISTANCE    = 50.0;
    public static final double MAX_DISTANCE    = 140.0;
    public static final double MIN_LAUNCH_POWER = 0.7;
    public static final double MAX_LAUNCH_POWER = 0.945;
    // - turret constants need to be tuned
    // actually make that everything needs to be tuned (~_-)
    public static final double TURRET_TICK_TOLERANCE = 2;
    public static final double TURRET_HEADING_TOLERANCE = 0.05;
    public static final double TURRET_LIMIT_TICK_TOLERANCE = 10;
    public static final double MAX_TURRET_POWER = 0.9;
    public static final double MAX_TURRET_TICKS = 5000;
    public static final double MIN_TURRET_TICKS = -5000;

    // Trigger / gate -- TODO tune on the robot
    public static final double TRIGGER_POWER   = 1.0;   // feed power once the shot is good
    public static final double GATE_OPEN_POS   = 0.6;   // gate servo position, open
    public static final double GATE_CLOSED_POS = 0.2;   // gate servo position, closed
    public static final double GATE_OPEN_TIME  = 0.15;  // seconds for the gate to swing open before feeding

    // Hardware config names -- TODO(config): match these to the robot's configuration
    public static final String LAUNCH_MOTOR       = "launch_motor";
    public static final String HOOD_ANGLE_SERVO   = "hood_angle_servo";
    public static final String HOOD_GAP_SERVO_1   = "hood_gap_servo_1";
    public static final String HOOD_GAP_SERVO_2   = "hood_gap_servo_2";
    public static final String PRE_SHOOTER_BEAM   = "pre_shooter_beam";
    public static final String TRIGGER_MOTOR      = "trigger_motor";
    public static final String GATE_SERVO         = "gate_servo";
    public static final String TURRET_SERVO       = "turret_servo";
    public static final String TURRET_ENCODER     = "turret_encoder";
    public static final String FRONT_LEFT_DRIVE   = "front_left_drive";
    public static final String BACK_LEFT_DRIVE    = "back_left_drive";
    public static final String FRONT_RIGHT_DRIVE  = "front_right_drive";
    public static final String BACK_RIGHT_DRIVE   = "back_right_drive";
    public static final String PINPOINT           = "pinpoint";
    public static final String LIMELIGHT          = "limelight";

    // Limelight AprilTags (manual 9.9: 36h11, 3.25 in, under each HIVE cell)
    public static final int HIVE_PIPELINE = 0;
    public static final int RED_FIRST_TAG = 30;   // red cells: 30-37
    public static final int BLUE_FIRST_TAG = 38;  // blue cells: 38-45
    public static final int TAGS_PER_ALLIANCE = 8;

    // Intake / storage hardware config names
    public static final String INTAKE_MOTOR        = "intake_motor";
    public static final String FLOWER_RAMP_SERVO   = "flower_ramp_servo";
    public static final String INTAKE_COLOR_SENSOR = "intake_color";
    // Beam pairs: LOW is broken by any element, HIGH (~8 cm up) only by NECTAR
    public static final String INTAKE_MOUTH_BEAM_LOW  = "intake_mouth_beam_low";  // intake mouth
    public static final String INTAKE_MOUTH_BEAM_HIGH = "intake_mouth_beam_high";
    public static final String INTAKE_QUEUE_BEAM_LOW  = "intake_queue_beam_low";  // inner station: counts elements
    public static final String INTAKE_QUEUE_BEAM_HIGH = "intake_queue_beam_high";
    public static final String STORAGE_EXIT_BEAM_LOW  = "storage_exit_beam_low";  // leaving into the shooter
    public static final String STORAGE_EXIT_BEAM_HIGH = "storage_exit_beam_high";

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
    // Normalized r + g + b at or below this is too dark to call a colour -- TODO tune
    public static double STORAGE_MIN_RGB_SUM = 0.05;

    /** Storage only rejects elements once the outtake times are set to real values. */
    public static boolean emergencyConfigured() {
        return Double.isFinite(EMERGENCY_MIN_OUTTAKE) && Double.isFinite(EMERGENCY_MAX_OUTTAKE)
                && EMERGENCY_MIN_OUTTAKE >= 0 && EMERGENCY_MAX_OUTTAKE > 0;
    }

    /** Storage only reads NECTAR colour once the hue windows and brightness floor are set. */
    public static boolean colourConfigured() {
        return Double.isFinite(RED_NECTAR_HUE_MIN) && Double.isFinite(RED_NECTAR_HUE_MAX)
                && Double.isFinite(BLUE_NECTAR_HUE_MIN) && Double.isFinite(BLUE_NECTAR_HUE_MAX)
                && Double.isFinite(STORAGE_MIN_RGB_SUM);
    }


    public static final double TURRET_GEAR_TICKS = 1000; //PLACEHOLDER
    public static final double SERVO_GEAR_TICKS = 350; //PLACEHOLDER

    //YKW EVERYTHING IS A PLACEHOLDER ;(
    public static final double INTAKE_POWER = 1.0;
    public static final double INTAKE_SLOW_POWER = 0.7;
    public static final double OUTTAKE_POWER = -1.0;
    public static final double INTAKE_JITTER_PERIOD = 0.2;
    public static final double HOOD_POLLEN_ANGLE_POS = 0.3;
    public static final double HOOD_NECTAR_ANGLE_POS = 0.1;
    public static final double HOOD_POLLEN_GAP = 0.1;
    public static final double HOOD_NECTAR_GAP = 0.5;
    public static final double FLYWHEEL_VELOCITY_TOLERANCE = 20;
    public static final double HOOD_RETRACTION_TOLERANCE = 0.05;
    public static final double HOOD_ANGLE_TOLERANCE = 0.05;
    public static final double HOOD_MOVEMENT_TIME = 0.05;
    public static final double FLOWER_RAMP_LOWERED = 0.9;
    public static final double FLOWER_RAMP_RETRACTED = 0.1;
}
