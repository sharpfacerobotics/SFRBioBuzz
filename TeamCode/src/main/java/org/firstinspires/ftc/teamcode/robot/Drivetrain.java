package org.firstinspires.ftc.teamcode.robot;

import static com.pedropathing.ivy.groups.Groups.parallel;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Drivetrain {

    // Hive (goal) positions in Pedro field coordinates, as {x, y}.
    // Blue is the red position mirrored across the field centerline (x = 141.5 - redX).
    private static final double[] RED_HIVE_NEAR = {58.746, 24.076};
    private static final double[] RED_HIVE_FAR  = {58.746, 115.815};
    private static final double[] BLUE_HIVE_NEAR = {82.754, 24.076};
    private static final double[] BLUE_HIVE_FAR  = {82.754, 115.815};

    private final RobotHardware robot;
    public Follower follower;

    /** True while a Pedro path started by this class is being followed. */
    public boolean autoDriving = false;

    public Drivetrain(RobotHardware robot) {
        this.robot = robot;
    }

    public void init(HardwareMap hardwareMap) {
        follower = Constants.create(hardwareMap);
    }

    // ------------------------------------------------------------------
    // Manual (mecanum) driving
    // ------------------------------------------------------------------

    /**
     * Drives the mecanum wheels directly. Call every loop with joystick inputs
     * (with yaw already overridden if lock-on is active).
     *
     * @param axial   forward/back, -1..1
     * @param lateral strafe left/right, -1..1
     * @param yaw     rotation, -1..1
     */
    public void drive(double axial, double lateral, double yaw) {
        double frontLeft  = axial + lateral + yaw;
        double frontRight = axial - lateral - yaw;
        double backLeft   = axial - lateral + yaw;
        double backRight  = axial + lateral - yaw;

        // Normalize so no wheel exceeds 100% power while keeping the ratios between wheels.
        double max = Math.max(
                Math.max(Math.abs(frontLeft), Math.abs(frontRight)),
                Math.max(Math.abs(backLeft), Math.abs(backRight)));

        if (max > 1.0) {
            frontLeft  /= max;
            frontRight /= max;
            backLeft   /= max;
            backRight  /= max;
        }

        robot.frontLeftDrive.setPower(frontLeft);
        robot.frontRightDrive.setPower(frontRight);
        robot.backLeftDrive.setPower(backLeft);
        robot.backRightDrive.setPower(backRight);
    }

    public void stop() {
        drive(0, 0, 0);
    }

    // ------------------------------------------------------------------
    // Auto driving (Pedro Pathing)
    // ------------------------------------------------------------------

    /** Starts following a straight line from the current pose to the target pose. */
    private void startAutoDrive(Pose targetPose) {
        Pose currentPose = follower.pose();
        Path path = Paths.line(currentPose, targetPose)
                .linear(currentPose, targetPose); // heading interpolates from start to target
        beginFollowing(path);
    }

    /** Starts following a curve from the current pose to the target, bent toward the control point. */
    private void startAutoDrive(Pose targetPose, Pose controlPoint) {
        Pose currentPose = follower.pose();
        // NOTE: the original ignored controlPoint. Check this matches your Paths.curve signature.
        Path path = Paths.curve(currentPose, controlPoint, targetPose)
                .linear(currentPose, targetPose);
        beginFollowing(path);
    }

    private void beginFollowing(Path path) {
        follower.follow(path);
        autoDriving = true;
    }

    /** Stops the current path and clears the auto-driving flag. */
    public void cancelAutoDrive() {
        follower.stop();
        autoDriving = false;
    }

    /** Command that drives in a straight line to targetPose and finishes when the path ends. */
    public Command startAutoDriveTo(Pose targetPose) {
        return Command.build()
                .setStart(() -> startAutoDrive(targetPose))
                .setDone(() -> {
                    boolean finished = !autoDriving || !follower.isBusy();
                    if (finished) {
                        autoDriving = false;
                    }
                    return finished;
                });
    }

    /** Command that runs both commands at the same time. */
    public Command driveWith(Command command1, Command command2) {
        return parallel(command1, command2);
    }

    /** Command that feeds a manual drive input to Pedro every loop. */
    public Command move(double y, double x, double yaw) {
        return Command.build()
                .setExecute(() -> follower.manual(y, x, yaw));
    }

    // ------------------------------------------------------------------
    // Pose / localization helpers
    // ------------------------------------------------------------------

    /**
     * Returns the {dx, dy} vector from the robot to the nearest hive for the given alliance.
     */
    public double[] getPedroDistanceToHive(Pose currentPose, String allianceColor) {
        boolean isRed = allianceColor.equalsIgnoreCase("red");
        double[] nearHive = isRed ? RED_HIVE_NEAR : BLUE_HIVE_NEAR;
        double[] farHive  = isRed ? RED_HIVE_FAR  : BLUE_HIVE_FAR;

        double[] toNear = {nearHive[0] - currentPose.x(), nearHive[1] - currentPose.y()};
        double[] toFar  = {farHive[0]  - currentPose.x(), farHive[1]  - currentPose.y()};

        return Math.hypot(toNear[0], toNear[1]) < Math.hypot(toFar[0], toFar[1])
                ? toNear
                : toFar;
    }

    /** Robot heading in degrees. */
    public double getRobotHeading() {
        return Math.toDegrees(follower.pose().heading());
    }

    /** Robot velocity as {vx, vy}. */
    public double[] getRobotTranslationalVelocity() {
        Vector2D velocity = follower.velocity().toVector2D();
        return new double[] {velocity.x(), velocity.y()};
    }
}
