package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Launcher;
import org.firstinspires.ftc.teamcode.robot.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.ShooterConfig;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.Vision;
import org.firstinspires.ftc.teamcode.robot.helperClasses.FieldGeometry;
import org.firstinspires.ftc.teamcode.robot.helperClasses.FieldGeometry.Cell;
import org.firstinspires.ftc.teamcode.robot.helperClasses.HiveTracker;

import java.util.ArrayList;
import java.util.List;

/**
 * Test screen: AprilTag distance -> flywheel + hood, AprilTag bearing -> turret, odometry when
 * the tags are hidden, HIVE tip tracking, and a READY TO FIRE light. Shows on the Driver Station
 * and in Panels. Hood and drive/odometry are optional: missing ones are listed on the screen.
 *
 * <pre>
 * left stick / right stick X  drive / turn (with odometry)
 * X   alliance             A   flywheel on/off        B   turret auto-aim on/off
 * LB/RB (hold)  turret by hand when auto-aim is off   back  zero turret here (point it forward)
 * Y   log a row            left stick button  pollen / nectar
 * dpad up/down  velocity trim (table)   dpad R/L  power trim (no table yet)
 * right trigger  "I just shot" (hive model)   start  hive tipped (driver override)
 * right stick button  snap odometry to the tags
 * </pre>
 */
@Configurable
@TeleOp(name = "TagShotTest", group = "Test")
public class TagShotTest extends OpMode {
    public static double VELOCITY_TRIM_STEP = 25;
    public static double POWER_TRIM_STEP = 0.01;
    public static double MANUAL_TURRET_POWER = 0.4;
    public static double AIM_TRIM_DEG = 0.0;
    public static double DRIVE_SCALE = 0.5;
    // Home the turret with the hall sensor at START (needs Turret.HALL_ANGLE measured);
    // otherwise it must point straight forward at INIT
    public static boolean HOME_TURRET = false;
    // Start pose when no auto ran before (field frame, inches / degrees)
    public static double START_X = 36.0;
    public static double START_Y = 72.0;
    public static double START_HEADING_DEG = 0.0;

    private static final int MAX_LOGGED_ROWS = 10;
    private static final int RUMBLE_MS = 200;

    private final RobotHardware robot = new RobotHardware();
    private final Launcher launcher = new Launcher(robot);
    private final Turret turret = new Turret(robot);
    private final Vision vision = new Vision(robot);
    private final ElapsedTime clock = new ElapsedTime();
    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private HiveTracker hive;
    private final List<String> missing = new ArrayList<>();

    private boolean red = true;
    private boolean nectar = false;
    private boolean flywheelOn = false;
    private boolean autoAim = false;
    private boolean wasReady = false;
    private boolean haveSolution = false;
    private double velocityTrim = 0.0;
    private double powerTrim = 0.0;
    private String aimSource = "none yet";
    private final double[] goal = new double[2];
    private final double[] robotVelocity = new double[2];
    private final List<String> loggedRows = new ArrayList<>();

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.initVision(hardwareMap);
        robot.initTurret(hardwareMap);
        try {
            robot.initHood(hardwareMap);
        } catch (IllegalArgumentException e) {
            robot.hoodAngleServo = null;
            robot.hoodGapServo = null;
            missing.add("hood servos");
        }
        try {
            follower = Constants.create(hardwareMap);
            Pose start = PoseStorage.currentPose != null ? PoseStorage.currentPose
                    : new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
            follower.setPose(start);
        } catch (IllegalArgumentException e) {
            follower = null;
            missing.add("drive/pinpoint (camera-only aim)");
        }
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        PanelsConfigurables.INSTANCE.refreshClass(TagShotTest.class);
        vision.start();
        if (!HOME_TURRET) {
            turret.zeroHere();
        }
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) {
            red = !red;
        }
        panelsTelemetry.addData("Alliance (X)", red ? "RED, tags 30-37" : "BLUE, tags 38-45");
        panelsTelemetry.addData("Turret", HOME_TURRET ? "homes with the hall sensor at START"
                : "zeroed at INIT - it must have been pointing forward");
        panelsTelemetry.addData("Not in config", missing.isEmpty() ? "-" : missing.toString());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        hive = new HiveTracker(FieldGeometry.startingUpCell(red));
        clock.reset();
        turret.restartTimer();
        if (HOME_TURRET) {
            turret.startHoming();
        }
    }

    @Override
    public void loop() {
        double now = clock.seconds();
        handleButtons(now);

        double heading = 0.0; // camera-only: aim in the robot frame
        if (follower != null) {
            follower.update();
            heading = follower.pose().heading();
            follower.manual(-gamepad1.left_stick_y * DRIVE_SCALE, -gamepad1.left_stick_x * DRIVE_SCALE,
                    -gamepad1.right_stick_x * DRIVE_SCALE);
        }

        Cell up = hive.getUpCell();
        vision.setTargetCluster(FieldGeometry.tagBase(red, up), FieldGeometry.tagBase(red, FieldGeometry.other(up)));
        vision.update(heading, turret.getAngle());
        hive.update(vision.hasTarget(), vision.seesOtherCell(), now);

        solveShot(heading);
        updateFlywheelAndHood();
        updateTurret(heading);
        if (gamepad1.yWasPressed()) {
            logRow();
        }
        showTelemetry(now);
    }

    private void handleButtons(double now) {
        if (gamepad1.xWasPressed()) {
            red = !red;
            hive = new HiveTracker(FieldGeometry.startingUpCell(red));
        }
        if (gamepad1.aWasPressed()) {
            flywheelOn = !flywheelOn;
        }
        if (gamepad1.bWasPressed()) {
            autoAim = !autoAim;
        }
        if (gamepad1.backWasPressed()) {
            turret.zeroHere();
        }
        if (gamepad1.leftStickButtonWasPressed()) {
            nectar = !nectar;
        }
        if (gamepad1.startWasPressed()) {
            hive.flip();
        }
        if (gamepad1.rightTriggerWasPressed()) {
            hive.recordShot(nectar, now);
        }
        if (gamepad1.rightStickButtonWasPressed()) {
            snapOdometryToTags();
        }
        if (gamepad1.dpadUpWasPressed()) {
            velocityTrim += VELOCITY_TRIM_STEP;
        }
        if (gamepad1.dpadDownWasPressed()) {
            velocityTrim -= VELOCITY_TRIM_STEP;
        }
        if (gamepad1.dpadRightWasPressed()) {
            powerTrim += POWER_TRIM_STEP;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            powerTrim -= POWER_TRIM_STEP;
        }
    }

    /** Tags in view: measure straight from them. Hidden: odometry carries the aim. */
    private void solveShot(double heading) {
        robotVelocity[0] = 0.0;
        robotVelocity[1] = 0.0;
        if (follower != null) {
            // TODO(verify): Pedro reports this in the field frame
            Velocity v = follower.velocity();
            robotVelocity[0] = v.vx;
            robotVelocity[1] = v.vy;
        }
        if (vision.hasTarget()) {
            double bearing = vision.getTargetFieldHeading();
            goal[0] = vision.getDistanceInches() * Math.cos(bearing);
            goal[1] = vision.getDistanceInches() * Math.sin(bearing);
            aimSource = "AprilTags";
        } else if (follower != null) {
            Pose pose = follower.pose();
            goal[0] = FieldGeometry.cellX(red) - pose.x();
            goal[1] = FieldGeometry.cellY(hive.getUpCell()) - pose.y();
            aimSource = "odometry (tags hidden)";
        } else {
            aimSource = haveSolution ? "last tag sighting" : "none yet";
            return; // camera-only and nothing in view: keep the last solution
        }
        ShooterConfig.calculateShootingConfig(vision.getAverageTa(), goal, robotVelocity, Double.NaN, heading);
        haveSolution = true;
    }

    private void updateFlywheelAndHood() {
        String element = nectar ? "NECTAR" : "POLLEN";
        launcher.setHood(element);
        if (!flywheelOn) {
            launcher.stop();
            return;
        }
        if (Double.isNaN(ShooterConfig.targetVelocity)) {
            launcher.setLauncherPower(ShooterConfig.fallbackPowerForDistance(ShooterConfig.distanceToGoal) + powerTrim);
        } else {
            launcher.setLauncher(ShooterConfig.targetVelocity + velocityTrim);
        }
    }

    private void updateTurret(double heading) {
        if (turret.getState() == Turret.TurretState.LOCALIZING) {
            turret.updateHoming();
            return;
        }
        if (!autoAim) {
            double manual = (gamepad1.left_bumper ? 1.0 : 0.0) - (gamepad1.right_bumper ? 1.0 : 0.0);
            turret.setManualPower(manual * MANUAL_TURRET_POWER);
            return;
        }
        if (!haveSolution) {
            turret.stop();
            return;
        }
        turret.aimAt(FieldGeometry.wrap(ShooterConfig.aimHeading - heading + Math.toRadians(AIM_TRIM_DEG)));
    }

    /** Put the robot where the tags say it is (keeps the odometry heading). */
    private void snapOdometryToTags() {
        if (follower == null || !vision.hasTarget()) {
            return;
        }
        // TODO(measure): ignores the camera's offset from the robot centre
        double bearing = vision.getTargetFieldHeading();
        double d = vision.getDistanceInches();
        Pose pose = follower.pose();
        follower.setPose(new Pose(FieldGeometry.cellX(red) - d * Math.cos(bearing),
                FieldGeometry.cellY(hive.getUpCell()) - d * Math.sin(bearing), pose.heading()));
    }

    private boolean isReady(double now) {
        return flywheelOn && haveSolution && launcher.isAtSpeed() && turret.isOnTarget()
                && launcher.isHoodSettled() && !hive.isSettling(now) && ShooterConfig.isInRange();
    }

    private void logRow() {
        double distance = vision.hasTarget() ? vision.getDistanceInches() : ShooterConfig.distanceToGoal;
        loggedRows.add(String.format("%s %.1f in, ta %.3f -> %.0f ticks/s, hood %.2f",
                nectar ? "NECTAR" : "POLLEN", distance, vision.getAverageTa(),
                launcher.getCurrVelocity(), launcher.getHoodAnglePos()));
        if (loggedRows.size() > MAX_LOGGED_ROWS) {
            loggedRows.remove(0);
        }
    }

    private void showTelemetry(double now) {
        boolean ready = isReady(now);
        if (ready && !wasReady) {
            gamepad1.rumble(RUMBLE_MS);
        }
        wasReady = ready;

        panelsTelemetry.addData("READY TO FIRE", ready ? "YES" : "no");
        panelsTelemetry.addData("Alliance (X)", red ? "RED" : "BLUE");
        panelsTelemetry.addData("Element (L stick btn)", nectar ? "NECTAR" : "POLLEN");
        panelsTelemetry.addData("Not in config", missing.isEmpty() ? "-" : missing.toString());

        panelsTelemetry.addData("Up cell / hive", String.format("%s, load %.1f, tips %d%s", hive.getUpCell(),
                hive.getWeight(), hive.getTips(), hive.isSettling(now) ? ", SETTLING" : ""));
        panelsTelemetry.addData("Tags seen", vision.hasTarget() ? vision.getVisibleIds() : "none");
        panelsTelemetry.addData("Aim from", aimSource);
        panelsTelemetry.addData("Tag distance (in)", String.format("%.1f", vision.getDistanceInches()));
        panelsTelemetry.addData("Shot distance (in)", String.format("%.1f%s", ShooterConfig.distanceToGoal,
                ShooterConfig.isInRange() ? "" : " OUT OF RANGE"));
        if (follower != null) {
            Pose pose = follower.pose();
            panelsTelemetry.addData("Pose", String.format("x %.1f  y %.1f  h %.1f deg",
                    pose.x(), pose.y(), Math.toDegrees(pose.heading())));
        }

        boolean table = !Double.isNaN(ShooterConfig.targetVelocity);
        panelsTelemetry.addData("Flywheel (A)", !flywheelOn ? "off" : table ? "table velocity" : "no table yet: straight-line power");
        panelsTelemetry.addData("Flywheel target", table
                ? String.format("%.0f ticks/s (trim %.0f)", ShooterConfig.targetVelocity + velocityTrim, velocityTrim)
                : String.format("%.2f power (trim %.2f)",
                        ShooterConfig.fallbackPowerForDistance(ShooterConfig.distanceToGoal) + powerTrim, powerTrim));
        panelsTelemetry.addData("Flywheel velocity", String.format("%.0f ticks/s%s", launcher.getCurrVelocity(),
                launcher.isAtSpeed() ? " AT SPEED" : ""));
        panelsTelemetry.addData("Battery", String.format("%.2f V", launcher.getVoltage()));
        panelsTelemetry.addData("Hood angle / gap", String.format("%.2f / %.2f%s", launcher.getHoodAnglePos(),
                launcher.getHoodGapPos(), launcher.isHoodSettled() ? "" : " moving"));

        panelsTelemetry.addData("Turret (B)", turret.getState() + (autoAim ? ", AUTO-AIM" : ", manual (LB/RB)"));
        panelsTelemetry.addData("Turret angle / target", String.format("%.1f / %.1f deg",
                Math.toDegrees(turret.getAngle()), Math.toDegrees(turret.getTargetAngle())));
        panelsTelemetry.addData("Turret power", String.format("%.2f%s", turret.getPower(),
                turret.isHallTriggered() ? ", HALL" : ""));

        panelsTelemetry.addData("Logged rows (Y)", loggedRows.size());
        for (int i = 0; i < loggedRows.size(); i++) {
            panelsTelemetry.addData("Row " + (i + 1), loggedRows.get(i));
        }
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        launcher.stop();
        turret.stop();
        vision.stop();
        if (follower != null) {
            PoseStorage.currentPose = follower.pose();
        }
    }
}
