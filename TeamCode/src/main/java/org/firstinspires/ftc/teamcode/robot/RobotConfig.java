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
