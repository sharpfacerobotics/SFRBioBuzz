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


}
