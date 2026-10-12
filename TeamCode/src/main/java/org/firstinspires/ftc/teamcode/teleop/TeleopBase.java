package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.HiveVision;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Launcher;
import org.firstinspires.ftc.teamcode.robot.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.RobotConfig.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotConfig.Element;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.ShooterConfig;
import org.firstinspires.ftc.teamcode.robot.Storage;
import org.firstinspires.ftc.teamcode.robot.Trigger;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.helperClasses.FieldDrawing;
import org.firstinspires.ftc.teamcode.robot.helperClasses.MatchLogger;

import java.io.File;

/**
 * Shared competition TeleOp. CompTeleopRed / CompTeleopBlue only pick the alliance. Owner: Arnav.
 *
 * <p>This is the only place subsystems talk to each other: the OpMode reads one subsystem's
 * output and passes it into the next as a parameter.</p>
 *
 * <pre>
 * Driver 1 (gamepad1)                      Driver 2 (gamepad2)
 *   left stick   drive                       A            flywheel on/off
 *   right stick  turn                        right trig   FIRE (hold)
 *   right bump   slow mode                   X            auto-fire on/off
 *   right trig   intake                      B            manual turret on/off (right stick X)
 *   left trig    outtake                     dpad L/R     turret trim
 *   A (hold)     aim chassis at hive         dpad down    clear trim
 *   left bump    snap pose to Limelight      Y            re-home turret
 *   options      reset field-centric heading start        turret: call current spot zero
 *   back         toggle field / robot centric back        reset storage count
 *
 * Auto-fire: fires whenever storage has elements, we're in range, and the shot is ready.
 * Flywheel spins up on its own when loaded (AUTO_SPINUP_WHEN_LOADED).
 * Every TeleOp writes a CSV match log to /sdcard/FIRST/data/matchlogs.
 *
 * Feedback: gamepad1 double-buzz when storage fills; gamepad2 light is green when a shot is
 * ready, yellow while the turret is homing, red if homing failed. Both buzz at 30 s and 10 s left.
 * </pre>
 */
public abstract class TeleopBase extends OpMode {
    protected abstract Alliance alliance();

    private final RobotHardware robot = new RobotHardware();
    private final Drivetrain drivetrain = new Drivetrain(robot);
    private final Intake intake = new Intake(robot);
    private final Storage storage = new Storage(robot);
    private final ShooterConfig shooterConfig = new ShooterConfig(robot);
    private final Launcher launcher = new Launcher(robot);
    private final Turret turret = new Turret(robot);
    private final Trigger trigger = new Trigger(robot);
    private final HiveVision hiveVision = new HiveVision(robot);
    private FieldDrawing fieldDrawing;
    private MatchLogger log;

    private final ElapsedTime voltageTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime matchTimer = new ElapsedTime();
    private final ElapsedTime relocalizeTimer = new ElapsedTime();

    private double goalX, goalY, driverHeading;
    private int hiveTagId;
    private boolean poseFromAuto;
    private boolean started = false;
    private boolean flywheelOn = false;
    private boolean fieldCentric = true;
    private double turretTrim = 0;
    private double voltage = RobotConfig.NOMINAL_VOLTAGE;
    private boolean autoFire = false;
    private boolean manualTurret = false;

    // stats
    private int shotsFired = 0;
    private double lastEmptyTime = -1;
    private double lastCycleTime = 0;
    private boolean wasEmpty = true;
    private double loopMs = 0;

    // feedback edge tracking
    private boolean wasFull = false;
    private int ledState = -1;
    private boolean warned1 = false;
    private boolean warned2 = false;

    private static final int LED_IDLE = 0, LED_READY = 1, LED_HOMING = 2, LED_FAILED = 3;

    @Override
    public void init() {
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        robot.init(hardwareMap);
        fieldDrawing = new FieldDrawing();

        boolean blue = alliance() == Alliance.BLUE;
        goalX = blue ? RobotConfig.BLUE_HIVE_X : RobotConfig.RED_HIVE_X;
        goalY = blue ? RobotConfig.BLUE_HIVE_Y : RobotConfig.RED_HIVE_Y;
        hiveTagId = blue ? RobotConfig.BLUE_HIVE_TAG_ID : RobotConfig.RED_HIVE_TAG_ID;
        driverHeading = blue ? RobotConfig.BLUE_DRIVER_HEADING : RobotConfig.RED_DRIVER_HEADING;

        poseFromAuto = PoseStorage.hasPose();
        robot.follower.setPose(poseFromAuto ? PoseStorage.get()
                : new Pose(RobotConfig.TELEOP_DEFAULT_X, RobotConfig.TELEOP_DEFAULT_Y, RobotConfig.TELEOP_DEFAULT_HEADING));

        trigger.stopFiring();
        hiveVision.activate();

        log = new MatchLogger(new File(AppUtil.ROBOT_DATA_DIR, "matchlogs"), "teleop_" + alliance(),
                "x", "y", "heading", "vx", "vy", "goalDist", "flyTarget", "flyVel", "flyReady",
                "turretAngle", "turretTarget", "turretOnTarget", "storage", "firing", "shot", "voltage", "loopMs");
    }

    @Override
    public void init_loop() {
        telemetry.addData("Alliance", alliance());
        telemetry.addData("Start pose", poseFromAuto ? "from auto" : "DEFAULT (auto didn't run)");
        telemetry.update();
    }

    @Override
    public void start() {
        started = true;
        turret.relocalize(); // home the turret on the hall effect while we drive off
        turret.restartTimer();
        loopTimer.reset();
        matchTimer.reset();
    }

    @Override
    public void loop() {
        robot.clearBulkCache();
        robot.follower.update();
        Pose pose = robot.follower.pose();
        Velocity velocity = robot.follower.velocity(); // field frame (twist rotated by heading)
        double heading = pose.heading();

        if (voltageTimer.seconds() > 0.25) { // voltage isn't in the bulk read
            voltageTimer.reset();
            voltage = robot.getVoltage();
        }
        hiveVision.update(hiveTagId, heading, velocity.omega);
        autoRelocalize(pose, velocity);

        // ---------------- driver 1: drive + intake
        if (gamepad1.optionsWasPressed()) {
            robot.follower.setHeading(driverHeading);
        }
        if (gamepad1.backWasPressed()) {
            fieldCentric = !fieldCentric;
        }
        if (gamepad1.leftBumperWasPressed() && hiveVision.hasBotpose()) {
            robot.follower.setPose(new Pose(hiveVision.getBotposeX(), hiveVision.getBotposeY(), hiveVision.getBotposeHeading()));
            gamepad1.rumble(200);
        }

        boolean wantIntake = gamepad1.right_trigger > 0.3;
        boolean wantOuttake = gamepad1.left_trigger > 0.3;
        // An unjam runs the rollers backwards, which can push an element back out.
        boolean reversing = wantOuttake || intake.isUnjamming();
        storage.update(wantIntake && !reversing && !storage.isFull(), reversing);
        intake.update(wantIntake, wantOuttake, storage.isFull());

        // ---------------- driver 2: shooter
        if (gamepad2.aWasPressed()) {
            flywheelOn = !flywheelOn;
        }
        if (gamepad2.dpadLeftWasPressed()) {
            turretTrim += Math.toRadians(1);
        }
        if (gamepad2.dpadRightWasPressed()) {
            turretTrim -= Math.toRadians(1);
        }
        if (gamepad2.dpadDownWasPressed()) {
            turretTrim = 0;
        }
        if (gamepad2.yWasPressed()) {
            turret.relocalize();
        }
        if (gamepad2.startWasPressed()) {
            turret.forceZeroHere(); // fallback if the hall sensor is dead: point it forward first
        }
        if (gamepad2.backWasPressed()) {
            storage.clear();
        }
        if (gamepad2.xWasPressed()) {
            autoFire = !autoFire;
            gamepad2.rumble(autoFire ? 300 : 100);
        }
        if (gamepad2.bWasPressed()) {
            manualTurret = !manualTurret;
        }

        shooterConfig.calculateShootingConfig(pose.x(), pose.y(), heading, velocity.vx, velocity.vy,
                goalX, goalY, hiveVision.getAverageTa());
        boolean loaded = !storage.isEmpty();
        boolean firing = gamepad2.right_trigger > 0.3
                || (autoFire && loaded && shooterConfig.isInRange());

        // ---------------- drive (after we know whether we're firing)
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;
        if (gamepad1.a) {
            // Turret-down fallback: point the chassis at the (shoot-on-the-move) aim heading.
            drivetrain.driveAiming(forward, strafe, Drivetrain.aimTurn(shooterConfig.getFieldAimHeading(), heading),
                    heading, driverHeading, fieldCentric, gamepad1.right_bumper);
        } else if (firing && RobotConfig.LOCK_POSITION_WHILE_FIRING && !Drivetrain.sticksActive(forward, strafe, turn)) {
            drivetrain.lockPosition();
        } else {
            drivetrain.drive(forward, strafe, turn, heading, driverHeading, fieldCentric, gamepad1.right_bumper);
        }

        // ---------------- shooter
        if (manualTurret) {
            turret.updateManual(-gamepad2.right_stick_x);
        } else {
            turret.updateTurret(shooterConfig.getTurretAngle() + turretTrim, velocity.omega);
        }
        // Manual turret / chassis aim: the driver is the judge of "on target".
        boolean aimed = manualTurret || gamepad1.a || turret.isOnTarget();

        Element next = storage.peekNext();
        boolean spinUp = flywheelOn || firing || (RobotConfig.AUTO_SPINUP_WHEN_LOADED && loaded);
        double targetVel = spinUp ? shooterConfig.getTargetVelocity() : RobotConfig.FLYWHEEL_IDLE_VELOCITY;
        launcher.update(targetVel, shooterConfig.getHoodAngle(next), next, voltage);

        boolean shot = false;
        if (firing) {
            trigger.fire(launcher.isReady(), aimed);
            if (launcher.shotDetected()) {
                storage.removeNext();
                shotsFired++;
                shot = true;
            }
        } else {
            trigger.stopFiring();
        }

        updateStats();
        driverFeedback(aimed);
        if (log.shouldLog(matchTimer.seconds())) {
            log.value(pose.x()).value(pose.y()).value(heading).value(velocity.vx).value(velocity.vy)
                    .value(shooterConfig.getDistanceToGoal()).value(launcher.getTargetVelocity())
                    .value(launcher.getCurrVelocity()).value(launcher.isReady())
                    .value(turret.getAngle()).value(turret.getTargetAngle()).value(aimed)
                    .value(storage.count()).value(firing).value(shot).value(voltage).value(loopMs)
                    .endRow();
        }

        // ---------------- telemetry
        fieldDrawing.draw(pose.x(), pose.y(), heading, shooterConfig.getFieldAimHeading(),
                shooterConfig.getDistanceToGoal(), goalX, goalY);
        telemetry.addData("Time left", "%.0f s", RobotConfig.TELEOP_LENGTH - matchTimer.seconds());
        telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f°", pose.x(), pose.y(), Math.toDegrees(heading));
        telemetry.addData("Drive", "%s%s", fieldCentric ? "field centric" : "robot centric",
                drivetrain.isLocked() ? "  LOCKED" : "");
        telemetry.addData("Storage", "%d  next=%s  last hue=%.0f", storage.count(), next, storage.getLastHue());
        telemetry.addData("Intake", intake.getState());
        telemetry.addData("Goal dist", "%.1f in", shooterConfig.getDistanceToGoal());
        telemetry.addData("Flywheel", "%.0f / %.0f %s", launcher.getCurrVelocity(), launcher.getTargetVelocity(),
                launcher.isReady() ? "READY" : "");
        telemetry.addData("Turret", "%s  %.1f° → %.1f°  trim %.0f°", turret.getState(),
                Math.toDegrees(turret.getAngle()), Math.toDegrees(turret.getTargetAngle()), Math.toDegrees(turretTrim));
        telemetry.addData("Hive tag", hiveVision.hasTarget() ? "seen" : "-");
        telemetry.addData("Modes", "%s%s%s", autoFire ? "AUTO-FIRE " : "", manualTurret ? "MANUAL-TURRET " : "",
                gamepad1.a ? "CHASSIS-AIM" : "");
        telemetry.addData("Stats", "shots %d  intaken %d  last cycle %.1f s", shotsFired,
                storage.getTotalIntaken(), lastCycleTime);
        addHealthWarnings();
        loopMs = loopTimer.milliseconds();
        telemetry.addData("Loop", "%.0f ms", loopMs);
        loopTimer.reset();
        telemetry.update();
    }

    @Override
    public void stop() {
        // Only forget the auto pose if this TeleOp actually ran. If the driver just init'd and
        // stopped to re-init, keep it so the real TeleOp still starts from auto's pose.
        if (started) {
            PoseStorage.clear();
        }
        log.close();
        robot.limelight.stop();
    }

    /** Cycle time = time between storage emptying out, i.e. one full collect-and-score loop. */
    private void updateStats() {
        boolean empty = storage.isEmpty();
        if (empty && !wasEmpty) {
            double now = matchTimer.seconds();
            if (lastEmptyTime >= 0) {
                lastCycleTime = now - lastEmptyTime;
            }
            lastEmptyTime = now;
        }
        wasEmpty = empty;
    }

    private void addHealthWarnings() {
        if (voltage < RobotConfig.LOW_BATTERY_VOLTAGE) {
            telemetry.addData("⚠ BATTERY", "%.2f V", voltage);
        }
        if (!robot.limelight.isConnected()) {
            telemetry.addLine("⚠ LIMELIGHT DISCONNECTED");
        }
        if (loopMs > RobotConfig.SLOW_LOOP_MS) {
            telemetry.addData("⚠ SLOW LOOP", "%.0f ms", loopMs);
        }
        if (!log.isLogging() && MatchLogger.ENABLED) {
            telemetry.addLine("⚠ match log not writing");
        }
    }

    /** Snap odometry to MegaTag2 when we're nearly stopped and it disagrees by enough to matter. */
    private void autoRelocalize(Pose pose, Velocity velocity) {
        if (!RobotConfig.AUTO_RELOCALIZE || !hiveVision.hasBotpose()
                || relocalizeTimer.seconds() < RobotConfig.AUTO_RELOCALIZE_PERIOD) {
            return;
        }
        if (Math.hypot(velocity.vx, velocity.vy) > RobotConfig.AUTO_RELOCALIZE_MAX_SPEED) {
            return;
        }
        double error = Math.hypot(hiveVision.getBotposeX() - pose.x(), hiveVision.getBotposeY() - pose.y());
        if (error > RobotConfig.AUTO_RELOCALIZE_MIN_ERROR) {
            robot.follower.setPose(new Pose(hiveVision.getBotposeX(), hiveVision.getBotposeY(), pose.heading()));
            relocalizeTimer.reset();
        }
    }

    /** Rumble and lights. Only sends commands on changes - they go over the network. */
    private void driverFeedback(boolean aimed) {
        boolean full = storage.isFull();
        if (full && !wasFull) {
            gamepad1.rumbleBlips(2);
        }
        wasFull = full;

        int led;
        if (turret.getState() == Turret.TurretState.HOMING_FAILED) {
            led = LED_FAILED;
        } else if (turret.getState() == Turret.TurretState.LOCALIZING) {
            led = LED_HOMING;
        } else if (launcher.isReady() && aimed) {
            led = LED_READY;
        } else {
            led = LED_IDLE;
        }
        if (led != ledState) {
            ledState = led;
            if (led == LED_READY) {
                gamepad2.setLedColor(0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
            } else if (led == LED_HOMING) {
                gamepad2.setLedColor(1, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
            } else if (led == LED_FAILED) {
                gamepad2.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            } else if (alliance() == Alliance.BLUE) {
                gamepad2.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
            } else {
                gamepad2.setLedColor(1, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
            }
        }

        double left = RobotConfig.TELEOP_LENGTH - matchTimer.seconds();
        if (!warned1 && left < RobotConfig.ENDGAME_WARNING_1) {
            warned1 = true;
            gamepad1.rumbleBlips(1);
            gamepad2.rumbleBlips(1);
        }
        if (!warned2 && left < RobotConfig.ENDGAME_WARNING_2) {
            warned2 = true;
            gamepad1.rumbleBlips(3);
            gamepad2.rumbleBlips(3);
        }
    }
}
