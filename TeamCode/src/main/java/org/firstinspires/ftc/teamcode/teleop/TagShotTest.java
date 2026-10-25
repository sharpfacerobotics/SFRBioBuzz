package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Launcher;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.ShooterConfig;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.Vision;

import java.util.ArrayList;
import java.util.List;

/**
 * Test screen: AprilTag distance -> flywheel, AprilTag bearing -> turret.
 * Shows on the Driver Station and in Panels. Point the turret straight forward before INIT.
 *
 * <pre>
 * X            switch alliance (init or running)   A  flywheel on/off
 * B            turret auto-aim on/off               left stick X  turret by hand (auto-aim off)
 * dpad up/down velocity trim (once the table has data)
 * dpad R/L     power trim (straight-line fallback, before the table has data)
 * Y            log a row: distance, ta, flywheel velocity   back  zero turret here
 * </pre>
 */
@Configurable
@TeleOp(name = "TagShotTest", group = "Test")
public class TagShotTest extends OpMode {
    public static double VELOCITY_TRIM_STEP = 25;
    public static double POWER_TRIM_STEP = 0.01;
    public static double MANUAL_TURRET_POWER = 0.4;
    // TODO(measure): true if the Limelight turns with the turret, false if it's on the chassis
    public static boolean CAMERA_ON_TURRET = false;
    // Chassis-mounted camera: where it points, degrees from the robot's front (+ = left)
    public static double CAMERA_YAW_DEG = 0.0;
    public static double AIM_TRIM_DEG = 0.0;

    private static final int MAX_LOGGED_ROWS = 10;

    private final RobotHardware robot = new RobotHardware();
    private final Launcher launcher = new Launcher(robot);
    private final Turret turret = new Turret(robot);
    private final Vision vision = new Vision(robot);
    private TelemetryManager panelsTelemetry;

    private boolean red = true;
    private boolean flywheelOn = false;
    private boolean autoAim = false;
    private double velocityTrim = 0.0;
    private double powerTrim = 0.0;
    private double lastDistance = RobotConfig.MIN_DISTANCE;
    private double aimTarget = 0.0;
    private String shotMode = "off";
    private double shotCommand = 0.0;
    private final List<String> loggedRows = new ArrayList<>();

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.initVision(hardwareMap);
        robot.initTurret(hardwareMap);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        PanelsConfigurables.INSTANCE.refreshClass(TagShotTest.class);
        vision.start();
        turret.zeroHere();
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) {
            red = !red;
        }
        panelsTelemetry.addData("Alliance (X)", red ? "RED, tags 30-37" : "BLUE, tags 38-45");
        panelsTelemetry.addData("Turret", "zeroed at INIT - it must have been pointing forward");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        turret.restartTimer();
    }

    @Override
    public void loop() {
        if (gamepad1.xWasPressed()) {
            red = !red;
        }
        if (gamepad1.aWasPressed()) {
            flywheelOn = !flywheelOn;
        }
        if (gamepad1.bWasPressed()) {
            autoAim = !autoAim;
            aimTarget = turret.getAngle();
        }
        if (gamepad1.backWasPressed()) {
            turret.zeroHere();
            aimTarget = 0.0;
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

        vision.setAlliance(red);
        vision.update();
        if (vision.hasTarget()) {
            lastDistance = vision.getDistanceInches();
        }
        updateFlywheel();
        updateTurret();
        if (gamepad1.yWasPressed()) {
            logRow();
        }
        showTelemetry();
    }

    private void updateFlywheel() {
        if (!flywheelOn) {
            launcher.stop();
            shotMode = "off (A)";
            shotCommand = 0.0;
            return;
        }
        double tableVelocity = ShooterConfig.velocityForDistance(lastDistance);
        if (Double.isNaN(tableVelocity)) {
            shotCommand = ShooterConfig.fallbackPowerForDistance(lastDistance) + powerTrim;
            launcher.setLauncherPower(shotCommand);
            shotMode = "no table yet: straight-line power";
        } else {
            shotCommand = tableVelocity + velocityTrim;
            launcher.setLauncher(shotCommand);
            shotMode = "table velocity";
        }
    }

    private void updateTurret() {
        if (!autoAim) {
            // stick right = turn right, and turret angles are + to the left
            turret.setManualPower(-gamepad1.left_stick_x * MANUAL_TURRET_POWER);
            return;
        }
        if (vision.hasTarget()) {
            // bearing is + to the right, turret angles are + to the left
            double bearing = Math.toRadians(AIM_TRIM_DEG - vision.getBearingDeg());
            aimTarget = CAMERA_ON_TURRET
                    ? turret.getAngle() + bearing
                    : Math.toRadians(CAMERA_YAW_DEG) + bearing;
        }
        turret.aimAt(aimTarget); // tags lost: hold the last target
    }

    private void logRow() {
        loggedRows.add(String.format("%.1f in, ta %.3f -> %.0f ticks/s",
                lastDistance, vision.getAverageTa(), launcher.getCurrVelocity()));
        if (loggedRows.size() > MAX_LOGGED_ROWS) {
            loggedRows.remove(0);
        }
    }

    private void showTelemetry() {
        panelsTelemetry.addData("Alliance (X)", red ? "RED, tags 30-37" : "BLUE, tags 38-45");
        panelsTelemetry.addData("Tags seen", vision.hasTarget() ? vision.getVisibleIds() : "none");
        panelsTelemetry.addData("Distance (in)", String.format("%.1f%s", lastDistance, vision.hasTarget() ? "" : " (last seen)"));
        panelsTelemetry.addData("Bearing (deg, + right)", String.format("%.1f", vision.getBearingDeg()));
        panelsTelemetry.addData("Avg ta (%)", String.format("%.3f", vision.getAverageTa()));

        panelsTelemetry.addData("Flywheel (A)", shotMode);
        panelsTelemetry.addData("Command", String.format("%.3f", shotCommand));
        panelsTelemetry.addData("Velocity trim (dpad U/D)", String.format("%.0f", velocityTrim));
        panelsTelemetry.addData("Power trim (dpad R/L)", String.format("%.2f", powerTrim));
        panelsTelemetry.addData("Flywheel velocity", String.format("%.0f ticks/s", launcher.getCurrVelocity()));

        panelsTelemetry.addData("Turret (B)", autoAim ? "AUTO-AIM" : "manual (left stick)");
        panelsTelemetry.addData("Turret angle (deg)", String.format("%.1f", Math.toDegrees(turret.getAngle())));
        panelsTelemetry.addData("Turret target (deg)", String.format("%.1f", Math.toDegrees(turret.getTargetAngle())));
        panelsTelemetry.addData("Turret error (deg)", String.format("%.1f", Math.toDegrees(turret.getError())));
        panelsTelemetry.addData("Turret power", String.format("%.2f", turret.getPower()));
        panelsTelemetry.addData("Turret hall", turret.isHallTriggered() ? "TRIGGERED" : "-");

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
    }
}
