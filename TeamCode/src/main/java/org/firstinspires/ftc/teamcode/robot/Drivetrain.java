package org.firstinspires.ftc.teamcode.robot;

/**
 * Mecanum driving for TeleOp. Owner: Arnav.
 *
 * <p>Pedro's follower owns the drive motors, so we drive through {@code follower.manual(...)}
 * instead of setting motor powers ourselves. That keeps localization and autos consistent.</p>
 *
 * <p>Sign convention (Pedro): forward = +x of the robot, strafe = +y = LEFT, turn = +CCW.
 * So from a gamepad pass {@code -left_stick_y, -left_stick_x, -right_stick_x}.</p>
 */
public class Drivetrain {
    private final RobotHardware robot;
    private boolean locked = false;

    public Drivetrain(RobotHardware robot) {
        this.robot = robot;
    }

    /**
     * @param forward      stick forward, [-1, 1]
     * @param strafe       stick left, [-1, 1]
     * @param turn         stick CCW, [-1, 1]
     * @param robotHeading current heading (rad) from the follower
     * @param driverHeading heading (rad) the driver is facing, used for field centric
     * @param fieldCentric true = stick up always drives away from the driver
     * @param slowMode     true = scale everything down for lining up
     */
    public void drive(double forward, double strafe, double turn, double robotHeading,
                      double driverHeading, boolean fieldCentric, boolean slowMode) {
        move(forward, strafe, shape(turn), true, robotHeading, driverHeading, fieldCentric, slowMode);
    }

    /**
     * Drive with the sticks but let aim assist control the turn. The turn is used as-is
     * (no cubing or slow-mode scaling) because it's already a controller output.
     *
     * @param aimTurn from {@link #aimTurn(double, double)}
     */
    public void driveAiming(double forward, double strafe, double aimTurn, double robotHeading,
                            double driverHeading, boolean fieldCentric, boolean slowMode) {
        move(forward, strafe, aimTurn, false, robotHeading, driverHeading, fieldCentric, slowMode);
    }

    private void move(double forward, double strafe, double turn, boolean slowTurn, double robotHeading,
                      double driverHeading, boolean fieldCentric, boolean slowMode) {
        forward = shape(forward);
        strafe = shape(strafe);

        if (fieldCentric) {
            // Rotate the driver-frame stick vector into the robot frame.
            double angle = driverHeading - robotHeading;
            double cos = Math.cos(angle);
            double sin = Math.sin(angle);
            double rotatedForward = forward * cos - strafe * sin;
            double rotatedStrafe = forward * sin + strafe * cos;
            forward = rotatedForward;
            strafe = rotatedStrafe;
        }

        if (slowMode) {
            forward *= RobotConfig.DRIVE_SLOW_SCALE;
            strafe *= RobotConfig.DRIVE_SLOW_SCALE;
            if (slowTurn) {
                turn *= RobotConfig.DRIVE_SLOW_SCALE;
            }
        }

        locked = false;
        robot.follower.manual(forward, strafe, turn);
    }

    /**
     * Hold the current spot with Pedro (pushes back against defense). Call every loop you want
     * to stay locked; it only grabs the pose once, when the lock starts.
     */
    public void lockPosition() {
        if (!locked) {
            robot.follower.hold(robot.follower.pose());
            locked = true;
        }
    }

    public boolean isLocked() {
        return locked;
    }

    /**
     * Turn command that points the robot's front at a field heading - used to aim the
     * chassis at the hive when the turret is down.
     *
     * @return turn power (CCW +), capped at AIM_ASSIST_MAX_TURN
     */
    public static double aimTurn(double targetHeading, double robotHeading) {
        double error = targetHeading - robotHeading;
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error <= -Math.PI) error += 2 * Math.PI;
        double turn = RobotConfig.AIM_ASSIST_KP * error;
        return Math.max(-RobotConfig.AIM_ASSIST_MAX_TURN, Math.min(RobotConfig.AIM_ASSIST_MAX_TURN, turn));
    }

    /** True if any stick is pushed past LOCK_BREAK_STICK - driver wants control back. */
    public static boolean sticksActive(double forward, double strafe, double turn) {
        return Math.abs(forward) > RobotConfig.LOCK_BREAK_STICK
                || Math.abs(strafe) > RobotConfig.LOCK_BREAK_STICK
                || Math.abs(turn) > RobotConfig.LOCK_BREAK_STICK;
    }

    public void stop() {
        robot.follower.manual(0, 0, 0);
    }

    /** Deadzone, then cube for fine control near centre (x * x * x, not Math.pow). */
    private double shape(double input) {
        if (Math.abs(input) < RobotConfig.DRIVE_DEADZONE) {
            return 0;
        }
        return input * input * input;
    }
}
