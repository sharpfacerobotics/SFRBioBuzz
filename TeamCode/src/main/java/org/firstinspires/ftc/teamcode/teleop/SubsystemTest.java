package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.ElementVision;
import org.firstinspires.ftc.teamcode.robot.HiveVision;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Launcher;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.Storage;
import org.firstinspires.ftc.teamcode.robot.Turret;

/**
 * Pit test: run every mechanism by hand and see every sensor. Use it to check wiring, motor
 * directions, the turret encoder direction, and to read element hues for RobotConfig.
 *
 * <pre>
 * sticks        drive (robot centric, slow)
 * right/left trigger  intake / outtake    right stick button (hold)  intake jitter
 * A (hold)      trigger feeder
 * B             gate open/closed
 * X             flywheel on/off at TEST_VELOCITY
 * Y             hood gap: pollen / nectar
 * dpad up/down  hood angle
 * bumpers       turret raw power left/right (NO soft limits - be careful)
 * left stick button  Limelight pipeline: hive tags / element detector
 * back          clear the storage queue
 *
 * Storage counting check: feed elements in by hand and watch the queue. The high beam should
 * break for NECTAR only; red NECTAR in should read "RED".
 * </pre>
 */
@Configurable
@TeleOp(name = "SubsystemTest", group = "Test")
public class SubsystemTest extends OpMode {
    public static double TEST_VELOCITY = 1500;
    public static double TURRET_TEST_POWER = 0.2;

    private final RobotHardware robot = new RobotHardware();
    private final Drivetrain drivetrain = new Drivetrain(robot);
    private final Intake intake = new Intake(robot);
    private final Storage storage = new Storage(robot);
    private final Launcher launcher = new Launcher(robot);
    private final Turret turret = new Turret(robot);
    private final HiveVision hiveVision = new HiveVision(robot);
    private final ElementVision elementVision = new ElementVision(robot);
    private boolean elementPipeline = false;

    private boolean gateOpen = false;
    private boolean flywheelOn = false;
    private String hoodElement = Storage.POLLEN;
    private double hoodAngle = 0.4;

    @Override
    public void init() {
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        robot.init(hardwareMap);
        robot.follower.setPose(new Pose(72, 72, 0));
        robot.gateServo.setPosition(RobotConfig.GATE_CLOSED_POS);
        telemetry.addLine("All hardware found. Press start.");
    }

    @Override
    public void loop() {
        robot.clearBulkCache();
        robot.follower.update();

        drivetrain.drive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x,
                0, 0, false, true);

        boolean emergency = storage.checkBeamBreaks("RED"); // RED: blue NECTAR should trigger emergency
        if (gamepad1.left_trigger > 0.3) {
            intake.setIntakePower(RobotConfig.OUTTAKE_POWER);
        } else {
            intake.updateIntake(gamepad1.right_trigger > 0.3, false, gamepad1.right_stick_button);
        }
        if (gamepad1.backWasPressed()) {
            storage.getElementQueue().clear();
        }
        robot.triggerMotor.setPower(gamepad1.a ? RobotConfig.TRIGGER_POWER : 0);
        if (gamepad1.bWasPressed()) {
            gateOpen = !gateOpen;
            robot.gateServo.setPosition(gateOpen ? RobotConfig.GATE_OPEN_POS : RobotConfig.GATE_CLOSED_POS);
        }

        if (gamepad1.xWasPressed()) {
            flywheelOn = !flywheelOn;
        }
        if (gamepad1.yWasPressed()) {
            hoodElement = Storage.POLLEN.equals(hoodElement) ? Storage.NECTAR : Storage.POLLEN;
        }
        if (gamepad1.dpadUpWasPressed()) {
            hoodAngle = Range.clip(hoodAngle + 0.05, RobotConfig.HOOD_ANGLE_MIN, RobotConfig.HOOD_ANGLE_MAX);
        }
        if (gamepad1.dpadDownWasPressed()) {
            hoodAngle = Range.clip(hoodAngle - 0.05, RobotConfig.HOOD_ANGLE_MIN, RobotConfig.HOOD_ANGLE_MAX);
        }
        launcher.update(flywheelOn ? TEST_VELOCITY : 0, hoodAngle, hoodElement, robot.getVoltage());

        double turretPower = 0;
        if (gamepad1.left_bumper) {
            turretPower = TURRET_TEST_POWER;
        } else if (gamepad1.right_bumper) {
            turretPower = -TURRET_TEST_POWER;
        }
        turret.setRawPower(turretPower);

        if (gamepad1.leftStickButtonWasPressed()) {
            elementPipeline = !elementPipeline;
            if (elementPipeline) {
                elementVision.activate();
            } else {
                hiveVision.activate();
            }
        }

        Pose pose = robot.follower.pose();
        telemetry.addData("Pinpoint pose", "x %.1f  y %.1f  h %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Intake", "power %.2f  %.2f A", robot.intakeMotor.getPower(),
                robot.intakeMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Beams", "intake low %s  high %s  exit %s",
                robot.intakeBeamLow.getState() ? "-" : "BROKEN", robot.intakeBeamHigh.getState() ? "-" : "BROKEN",
                robot.storageExitBeam.getState() ? "-" : "BROKEN");
        NormalizedRGBA c = robot.intakeColorSensor.getNormalizedColors();
        telemetry.addData("Colour", "hue %.0f  nectar=%s", Storage.hue(c.red, c.green, c.blue), storage.readNectarColor());
        telemetry.addData("Queue", "%s%s", storage.getElementQueue(), emergency ? "  EMERGENCY (would outtake)" : "");
        telemetry.addData("Gate", gateOpen ? "OPEN" : "closed");
        telemetry.addData("Flywheel", "%.0f / %.0f ticks/s", launcher.getCurrVelocity(), flywheelOn ? TEST_VELOCITY : 0);
        telemetry.addData("Hood", "angle %.2f  gap %s", hoodAngle, hoodElement);
        telemetry.addData("Turret", "power %.2f  raw ticks %.0f  hall %s", turretPower, turret.getRawTicks(),
                turret.isHallTriggered() ? "MAGNET" : "-");
        telemetry.addLine("  + power (left bumper) should make raw ticks go UP;");
        telemetry.addLine("  if they go down set RobotConfig.TURRET_ENCODER_REVERSED = true");
        telemetry.addData("Limelight", "%s  pipeline %d", robot.limelight.isConnected() ? "connected" : "NOT CONNECTED",
                robot.limelight.getLatestResult() == null ? -1 : robot.limelight.getLatestResult().getPipelineIndex());
        if (elementPipeline) {
            elementVision.update(null, null);
            telemetry.addData("Element", elementVision.hasElement()
                    ? String.format("%s  tx %.1f°  ty %.1f°  ~%.0f in", elementVision.getType(),
                    elementVision.getTx(), elementVision.getTy(), elementVision.getDistance())
                    : "none");
        } else {
            // show which hive tag clusters are visible (red far 30, red aud 34, blue aud 38, blue far 42)
            hiveVision.update(30, 34, pose.heading(), 0);
            telemetry.addData("Hive tags", "red far %s  red audience %s",
                    hiveVision.hasTarget() ? "SEEN" : "-", hiveVision.seesOtherCell() ? "SEEN" : "-");
            telemetry.addData("MegaTag2", hiveVision.hasBotpose()
                    ? String.format("x %.1f  y %.1f  h %.0f°", hiveVision.getBotposeX(), hiveVision.getBotposeY(),
                    Math.toDegrees(hiveVision.getBotposeHeading()))
                    : "no pose");
        }
        telemetry.addData("Battery", "%.2f V", robot.getVoltage());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.limelight.stop();
    }
}
