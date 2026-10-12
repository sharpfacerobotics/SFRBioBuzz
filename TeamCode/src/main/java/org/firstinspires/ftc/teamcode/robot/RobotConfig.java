package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Every constant the robot uses, in one place. Owner: Arnav.
 *
 * <p>Per team protocol, RobotConfig does NOT use the constructor pattern - everything is static.
 * {@code static final} values are fixed facts (hardware names, geometry). Plain {@code static}
 * values are tunable and show up live in Panels because of {@link Configurable}.</p>
 *
 * <p>Anything marked TODO(tune) is a placeholder and MUST be measured on the real robot.</p>
 */
@Configurable
public class RobotConfig {

    // ------------------------------------------------------------------ shared enums

    public enum Alliance { RED, BLUE }

    /** Game elements that can sit in storage. UNKNOWN = sensed but colour didn't match. */
    public enum Element { POLLEN, NECTAR, UNKNOWN }

    // ------------------------------------------------------------------ hardware map names
    // These must match the Driver Station robot configuration exactly.

    public static final String FRONT_LEFT_DRIVE   = "front_left_drive";
    public static final String BACK_LEFT_DRIVE    = "back_left_drive";
    public static final String FRONT_RIGHT_DRIVE  = "front_right_drive";
    public static final String BACK_RIGHT_DRIVE   = "back_right_drive";
    public static final String PINPOINT           = "pinpoint";
    public static final String INTAKE_MOTOR       = "intake_motor";
    public static final String LAUNCH_MOTOR       = "launch_motor";
    public static final String HOOD_ANGLE_SERVO_1 = "hood_angle_servo_1";
    public static final String HOOD_ANGLE_SERVO_2 = "hood_angle_servo_2";
    public static final String HOOD_GAP_SERVO_1   = "hood_gap_servo_1";
    public static final String HOOD_GAP_SERVO_2   = "hood_gap_servo_2";
    public static final String TURRET_SERVO       = "turret_servo";
    /** Turret's external encoder is plugged into this motor port. */
    public static final String TURRET_ENCODER     = "turret_encoder";
    public static final String TURRET_HALL        = "turret_hall";
    public static final String TRIGGER_MOTOR      = "trigger_motor";
    public static final String GATE_SERVO         = "gate_servo";
    public static final String STORAGE_SENSOR     = "storage_color";
    public static final String LIMELIGHT          = "limelight";
    public static final String VOLTAGE_SENSOR     = "Control Hub";

    // ------------------------------------------------------------------ field (Pedro coords, inches)
    // Pedro field: origin in a corner, 144 x 144 in, heading in radians.

    public static final double FIELD_SIZE = 144.0;
    // TODO(measure): hive centre positions from the BioBuzz field drawings.
    public static double BLUE_HIVE_X = 12.0;
    public static double BLUE_HIVE_Y = 132.0;
    public static double RED_HIVE_X  = 132.0;
    public static double RED_HIVE_Y  = 132.0;

    /** TeleOp starting pose when auto didn't run (PoseStorage empty). TODO(measure) */
    public static double TELEOP_DEFAULT_X = 72.0;
    public static double TELEOP_DEFAULT_Y = 72.0;
    public static double TELEOP_DEFAULT_HEADING = Math.PI / 2;

    // ------------------------------------------------------------------ drivetrain

    public static double DRIVE_SLOW_SCALE = 0.35;
    public static double DRIVE_DEADZONE   = 0.05;
    /** Driver heading offset for field centric (driver faces +Y from the blue wall). */
    public static double BLUE_DRIVER_HEADING = Math.PI / 2;
    public static double RED_DRIVER_HEADING  = Math.PI / 2;
    /** Hold position (Pedro) while firing so defense can't push us off our shot. Needs Foresight tuned. */
    public static boolean LOCK_POSITION_WHILE_FIRING = false;
    /** Stick deflection that cancels the position lock. */
    public static double LOCK_BREAK_STICK = 0.3;

    // ------------------------------------------------------------------ driver feedback / assists

    /** Spin the flywheel up on its own whenever storage has something in it. */
    public static boolean AUTO_SPINUP_WHEN_LOADED = true;
    /** Auto-fire / range check: only shoot between these distances (in) from the hive. */
    public static double MIN_SHOT_DISTANCE = 24.0;
    public static double MAX_SHOT_DISTANCE = 144.0;
    /** Chassis aim assist (turret-down fallback): turn = kP * heading error, capped. */
    public static double AIM_ASSIST_KP = 1.5;
    public static double AIM_ASSIST_MAX_TURN = 0.6;
    /** Manual turret: stick -> power scale. */
    public static double MANUAL_TURRET_SCALE = 0.6;
    // Health warnings shown on telemetry
    public static double LOW_BATTERY_VOLTAGE = 11.5;
    public static double SLOW_LOOP_MS = 40.0;

    public static final double TELEOP_LENGTH = 120.0;          // seconds
    public static double ENDGAME_WARNING_1 = 30.0;             // seconds left -> rumble
    public static double ENDGAME_WARNING_2 = 10.0;
    /** Snap odometry to Limelight automatically when stopped and the tag is clear. Off until verified. */
    public static boolean AUTO_RELOCALIZE = false;
    public static double AUTO_RELOCALIZE_MIN_ERROR = 2.0;      // inches before we bother snapping
    public static double AUTO_RELOCALIZE_MAX_SPEED = 2.0;      // in/s - only while basically stopped
    public static double AUTO_RELOCALIZE_PERIOD = 1.0;         // seconds between snaps

    // ------------------------------------------------------------------ intake

    public static double INTAKE_POWER         = 0.8;
    public static double OUTTAKE_POWER        = -0.6;
    public static boolean INTAKE_JAM_DETECTION = true;
    public static double INTAKE_JAM_CURRENT   = 6.0;   // amps TODO(tune)
    public static double INTAKE_JAM_TIME      = 0.25;  // seconds above jam current before we call it a jam
    public static double INTAKE_UNJAM_TIME    = 0.2;   // seconds to run backwards
    /** getCurrent() is a slow hub call, so only poll it this often. */
    public static double INTAKE_CURRENT_POLL  = 0.05;

    // ------------------------------------------------------------------ storage

    public static final int STORAGE_CAPACITY = 3;               // TODO(confirm) with game manual
    public static double STORAGE_PRESENT_DISTANCE_CM = 4.0;     // closer than this = element at entry
    public static double STORAGE_DEBOUNCE_TIME = 0.12;          // seconds between counted elements
    // Hue windows in degrees [0, 360). TODO(tune) under event lighting.
    public static double POLLEN_HUE_MIN = 35.0;
    public static double POLLEN_HUE_MAX = 75.0;
    public static double NECTAR_HUE_MIN = 250.0;
    public static double NECTAR_HUE_MAX = 320.0;
    public static double STORAGE_MIN_SATURATION = 0.25;

    // ------------------------------------------------------------------ trigger / gate

    public static double TRIGGER_POWER      = 1.0;
    public static double GATE_OPEN_POS      = 0.6;
    public static double GATE_CLOSED_POS    = 0.2;
    public static double GATE_OPEN_TIME     = 0.15;  // seconds for gate to physically swing open
    public static final double FIRE_TIME    = 0.3;   // seconds per element, used by auto timeouts

    // ------------------------------------------------------------------ launcher

    public static double FLYWHEEL_VELOCITY_TOLERANCE = 40.0;   // ticks/s
    /** A velocity dip bigger than this while feeding counts as one shot leaving. */
    public static double SHOT_DETECT_DROP = 120.0;              // ticks/s TODO(tune)
    /** A shot dip has to happen within this long of last being at speed, or it isn't a shot. */
    public static double SHOT_DETECT_WINDOW = 0.15;             // seconds
    public static double NOMINAL_VOLTAGE  = 12.0;
    public static double HOOD_SETTLE_TIME = 0.15;               // servos have no feedback, so wait
    public static double HOOD_ANGLE_SERVO_2_OFFSET = 0.0;       // trim so both hood servos agree
    public static double HOOD_ANGLE_MIN = 0.0;
    public static double HOOD_ANGLE_MAX = 1.0;
    // Hood gap per element (elements are different sizes). TODO(tune)
    public static double HOOD_POLLEN_GAP_1 = 0.40;
    public static double HOOD_POLLEN_GAP_2 = 0.40;
    public static double HOOD_NECTAR_GAP_1 = 0.55;
    public static double HOOD_NECTAR_GAP_2 = 0.55;
    public static double FLYWHEEL_IDLE_VELOCITY = 0.0;

    // ------------------------------------------------------------------ turret

    /** Encoder ticks per radian of turret rotation (encoder CPR * gear ratio / 2pi). TODO(measure) */
    public static double TURRET_TICKS_PER_RAD = 8192.0 * 5.0 / (2.0 * Math.PI);
    /** Flip if positive turret power makes the encoder count DOWN (check in SubsystemTest). */
    public static boolean TURRET_ENCODER_REVERSED = false;
    public static double TURRET_MIN_ANGLE = -Math.toRadians(190);  // soft limits, radians from forward
    public static double TURRET_MAX_ANGLE =  Math.toRadians(190);
    public static double TURRET_ANGLE_TOLERANCE  = Math.toRadians(1.5);
    /** If the move needed is bigger than this, we're unwinding the cable, not aiming. */
    public static double TURRET_UNWIND_THRESHOLD = Math.toRadians(120);
    public static double MAX_TURRET_POWER = 0.9;
    public static double TURRET_KP = 1.2;
    public static double TURRET_KD = 0.05;
    public static double TURRET_KS = 0.04;   // static friction kick
    /** Feedforward on robot yaw rate so the turret counter-rotates while the chassis spins. */
    public static double TURRET_KV_ROBOT_OMEGA = 0.15;
    // Homing ("cow path" search for the hall effect zero)
    public static double TURRET_HOMING_POWER     = 0.25;
    public static double TURRET_HOMING_FIRST_SWEEP = Math.toRadians(20);
    public static double TURRET_HOMING_MAX_SWEEP   = Math.toRadians(180); // keep inside the hard stops
    /** Give up homing after this long (e.g. encoder unplugged, so the sweep never ends). */
    public static double TURRET_HOMING_TIMEOUT = 5.0;
    /** Re-zero if the hall effect fires and we think we're further than this from zero. */
    public static double TURRET_RELOCALIZE_THRESHOLD = Math.toRadians(3);

    // ------------------------------------------------------------------ vision (Limelight)

    public static final int HIVE_PIPELINE    = 0;
    public static final int ELEMENT_PIPELINE = 1;
    public static int BLUE_HIVE_TAG_ID = 20;   // TODO(confirm) BioBuzz tag IDs
    public static int RED_HIVE_TAG_ID  = 24;
    public static final int TA_AVERAGE_WINDOW = 8;
    public static final double METERS_TO_INCHES = 39.3700787;
    // Camera mounting for element distance estimates. TODO(measure)
    public static double CAMERA_HEIGHT_IN   = 10.0;
    public static double CAMERA_PITCH_DEG   = -20.0;  // negative = tilted down
    public static double ELEMENT_HEIGHT_IN  = 2.0;
    public static double CAMERA_FORWARD_OFFSET_IN = 6.0;

    // ------------------------------------------------------------------ auto

    public static double AUTO_SHOOT_TIMEOUT  = 3.0;   // seconds max at a shoot pose
    public static double AUTO_PATH_TIMEOUT   = 4.0;   // seconds max per path before moving on
    public static double AUTO_INTAKE_DWELL   = 0.4;   // seconds to sit at a pickup spot
    /** Keep the intake running this long into the next step to finish swallowing. */
    public static double AUTO_INTAKE_CARRY   = 0.5;
    public static double AUTO_MAX_START_DELAY = 15.0;
    public static final int AUTO_PRELOAD_COUNT = 3;
}
