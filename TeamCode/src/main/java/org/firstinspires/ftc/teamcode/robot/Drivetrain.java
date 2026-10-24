package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.robot.helperClasses.FieldGeometry;

import static com.pedropathing.api.Paths.line;

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
    private final double[] hiveDelta = new double[2]; // reused: getPedroDistanceToHive allocates nothing

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
        driveCore(forward, strafe, shape(turn), true, robotHeading, driverHeading, fieldCentric, slowMode);
    }

    /**
     * Drive with the sticks but let aim assist control the turn. The turn is used as-is
     * (no cubing or slow-mode scaling) because it's already a controller output.
     *
     * @param aimTurn from {@link #aimTurn(double, double)}
     */
    public void driveAiming(double forward, double strafe, double aimTurn, double robotHeading,
                            double driverHeading, boolean fieldCentric, boolean slowMode) {
        driveCore(forward, strafe, aimTurn, false, robotHeading, driverHeading, fieldCentric, slowMode);
    }

    // ------------------------------------------------------------------ glossary methods

    /**
     * CONTINUOUS. Mecanum drive equations, robot centric. Loop call.
     * Standard FTC stick convention: {@code move(-gamepad.left_stick_y, gamepad.left_stick_x,
     * gamepad.right_stick_x)}.
     *
     * @param y   forward (+)
     * @param x   strafe right (+)
     * @param yaw turn clockwise (+)
     */
    public void move(double y, double x, double yaw) {
        locked = false;
        // Pedro's convention is strafe-left and counter-clockwise positive.
        robot.follower.manual(shape(y), -shape(x), -shape(yaw));
    }

    /** COMMAND. An Ivy command that follows the path; schedule it with Ivy. */
    public Command driveWith(Path path) {
        return PedroCommands.follow(robot.follower, path);
    }

    /** START. Fire and forget: drive from where we are to endPose. Check reachedAutoMovePoint(). */
    public void autoMoveTo(Pose endPose) {
        Pose from = robot.follower.pose();
        followPath(line(from, endPose).linear(from, endPose));
    }

    /** STATUS. True when the autoMoveTo path has finished. Loop check. */
    public boolean reachedAutoMovePoint() {
        return !robot.follower.isBusy();
    }

    /**
     * CONTINUOUS. Drive toward an element the camera sees: creep forward while turning to
     * centre it. Loop call while ElementVision.elementDetected().
     *
     * @param deltaX ElementVision.getElementDeltaX() (deg, + = right of centre)
     */
    public void autoGetElement(double deltaX) {
        locked = false;
        robot.follower.manual(RobotConfig.ELEMENT_APPROACH_POWER, 0, bearingTurn(deltaX));
    }

    /**
     * SET. Relocalize with the Limelight (FTC-frame MegaTag2 pose, e.g.
     * Vision.getAverageBotPose()). Call only with a valid read - null is ignored.
     */
    public void relocalize(Pose3D botpose) {
        if (botpose == null) {
            return;
        }
        Position p = botpose.getPosition();
        double ftcX = p.unit.toInches(p.x);
        double ftcY = p.unit.toInches(p.y);
        double yaw = botpose.getOrientation().getYaw(AngleUnit.RADIANS);
        robot.follower.setPose(new Pose(FieldGeometry.ftcToFieldX(ftcX, ftcY),
                FieldGeometry.ftcToFieldY(ftcX, ftcY), FieldGeometry.ftcToFieldHeading(yaw)));
        locked = false; // a hold target in the old coordinates would yank the robot
    }

    /**
     * STATUS. {dx, dy} (in, field frame) from currentPose to our alliance's HIVE (the pivot
     * between its two cells). The array is reused - copy it if you keep it.
     */
    public double[] getPedroDistanceToHive(Pose currentPose, String allianceColor) {
        boolean red = "RED".equals(allianceColor);
        hiveDelta[0] = FieldGeometry.cellX(red) - currentPose.x();
        hiveDelta[1] = FieldGeometry.CENTER - currentPose.y();
        return hiveDelta;
    }

    /**
     * {dx, dy} to the cell we're launching into (the upward one), from Vision.getHiveTipState().
     * This is what ShooterConfig should aim at; the 2-argument version gives the hive pivot.
     */
    public double[] getPedroDistanceToHive(Pose currentPose, String allianceColor, String hiveTipState) {
        boolean red = "RED".equals(allianceColor);
        hiveDelta[0] = FieldGeometry.cellX(red) - currentPose.x();
        hiveDelta[1] = FieldGeometry.cellY(FieldGeometry.cellForTipState(red, hiveTipState)) - currentPose.y();
        return hiveDelta;
    }

    private void driveCore(double forward, double strafe, double turn, boolean slowTurn, double robotHeading,
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

    /** Let Pedro drive a path (e.g. TeleOp park assist). Any drive() call takes control back. */
    public void followPath(com.pedropathing.paths.Path path) {
        locked = false;
        robot.follower.follow(path);
    }

    public boolean isPathBusy() {
        return robot.follower.isBusy();
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

    /**
     * Turn command toward something the camera sees tx degrees to the right (ElementVision).
     *
     * @return turn power (CCW +), capped at AIM_ASSIST_MAX_TURN
     */
    public static double bearingTurn(double txDegrees) {
        double turn = -RobotConfig.ELEMENT_ASSIST_KP * Math.toRadians(txDegrees); // right of centre -> turn CW
        return Math.max(-RobotConfig.AIM_ASSIST_MAX_TURN, Math.min(RobotConfig.AIM_ASSIST_MAX_TURN, turn));
    }

    /** True if any stick is pushed past LOCK_BREAK_STICK - driver wants control back. */
    public static boolean sticksActive(double forward, double strafe, double turn) {
        return Math.abs(forward) > RobotConfig.LOCK_BREAK_STICK
                || Math.abs(strafe) > RobotConfig.LOCK_BREAK_STICK
                || Math.abs(turn) > RobotConfig.LOCK_BREAK_STICK;
    }

    /** Forget the held spot (call whenever odometry is re-set, so a hold can't chase old coordinates). */
    public void unlock() {
        locked = false;
    }

    public void stop() {
        locked = false;
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
