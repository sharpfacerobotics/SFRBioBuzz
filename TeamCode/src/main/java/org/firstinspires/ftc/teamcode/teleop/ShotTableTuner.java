package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Launcher;
import org.firstinspires.ftc.teamcode.robot.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.RobotConfig.Element;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.ShooterConfig;
import org.firstinspires.ftc.teamcode.robot.Trigger;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.helperClasses.FieldGeometry;

/**
 * Builds the ShooterConfig distance table. Drive to a spot, set VELOCITY and HOOD_ANGLE in
 * Panels until shots go in, press A to log the row, move, repeat. Copy the logged rows into
 * ShooterConfig's arrays (sorted by distance).
 *
 * <p>The turret auto-aims (distance and angle come from odometry, so start the robot on a
 * known pose: set START_X/Y/HEADING or run an auto first).</p>
 *
 * <pre>
 * sticks       drive (robot centric, slow)   right trigger  fire (hold)
 * left trigger intake                        Y              element: pollen / nectar
 * A            log this row                  start          turret: zero here
 * </pre>
 */
@Configurable
@TeleOp(name = "ShotTableTuner", group = "Test")
public class ShotTableTuner extends OpMode {
    public static double VELOCITY = 1500;
    public static double HOOD_ANGLE = 0.4;
    public static boolean BLUE = false;
    public static boolean FAR_CELL = false;
    public static double START_X = 72;
    public static double START_Y = 72;
    public static double START_HEADING_DEG = 90;

    private final RobotHardware robot = new RobotHardware();
    private final Drivetrain drivetrain = new Drivetrain(robot);
    private final Intake intake = new Intake(robot);
    private final ShooterConfig shooterConfig = new ShooterConfig(robot);
    private final Launcher launcher = new Launcher(robot);
    private final Turret turret = new Turret(robot);
    private final Trigger trigger = new Trigger(robot);

    private Element element = Element.POLLEN;
    private int rowCount = 0;

    @Override
    public void init() {
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        robot.init(hardwareMap);
        robot.follower.setPose(PoseStorage.hasPose() ? PoseStorage.get()
                : new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)));
        trigger.stopFiring();
    }

    @Override
    public void start() {
        turret.relocalize();
        turret.restartTimer();
    }

    @Override
    public void loop() {
        robot.clearBulkCache();
        robot.follower.update();
        Pose pose = robot.follower.pose();
        Velocity velocity = robot.follower.velocity();
        // Shoot at the cell you pick in Panels (FAR_CELL) - it must be the one facing up.
        double goalX = FieldGeometry.cellX(!BLUE);
        double goalY = FieldGeometry.cellY(FAR_CELL ? FieldGeometry.Cell.FAR : FieldGeometry.Cell.AUDIENCE);

        drivetrain.drive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x,
                pose.heading(), 0, false, true);
        intake.update(gamepad1.left_trigger > 0.3, false, false);

        if (gamepad1.yWasPressed()) {
            element = element == Element.POLLEN ? Element.RED_NECTAR : Element.POLLEN;
        }
        if (gamepad1.startWasPressed()) {
            turret.forceZeroHere();
        }

        // Stationary tuning: no shoot-on-the-move offset, so the table is pure distance.
        shooterConfig.calculateShootingConfig(pose.x(), pose.y(), pose.heading(), 0, 0, goalX, goalY, 0);
        turret.updateTurret(shooterConfig.getTurretAngle(), velocity.omega);
        launcher.update(VELOCITY, HOOD_ANGLE, element, robot.getVoltage());

        if (gamepad1.right_trigger > 0.3) {
            trigger.fire(launcher.isReady(), turret.isOnTarget());
        } else {
            trigger.stopFiring();
        }

        double distance = shooterConfig.getDistanceToGoal();
        if (gamepad1.aWasPressed()) {
            rowCount++;
            String row = String.format("%s  dist %.1f  vel %.0f  hood %.3f", element, distance, VELOCITY, HOOD_ANGLE);
            telemetry.log().add(row); // the log stays on screen, so rows don't need re-adding each loop
            gamepad1.rumble(150);
        }

        telemetry.addData("Distance to hive", "%.1f in", distance);
        telemetry.addData("Element", element);
        telemetry.addData("Flywheel", "%.0f / %.0f %s", launcher.getCurrVelocity(), VELOCITY, launcher.isReady() ? "READY" : "");
        telemetry.addData("Turret", "%s %s", turret.getState(), turret.isOnTarget() ? "ON TARGET" : "");
        telemetry.addData("Logged rows", rowCount);
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.limelight.stop();
    }
}
