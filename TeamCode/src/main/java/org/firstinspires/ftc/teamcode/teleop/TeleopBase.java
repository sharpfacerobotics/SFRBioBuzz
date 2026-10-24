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
import org.firstinspires.ftc.teamcode.robot.ElementVision;
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
import org.firstinspires.ftc.teamcode.robot.Vision;
import org.firstinspires.ftc.teamcode.robot.helperClasses.FieldDrawing;
import org.firstinspires.ftc.teamcode.robot.helperClasses.FieldGeometry;
import org.firstinspires.ftc.teamcode.robot.helperClasses.HiveTracker;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.robot.helperClasses.MatchLogger;

import java.io.File;
import java.util.Queue;

import static com.pedropathing.api.Paths.line;

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
 *   right trig   intake (half = slow)        B            manual turret on/off (right stick X)
 *   left trig    outtake                     dpad L/R     turret trim
 *   A (hold)     aim chassis at hive         dpad down    clear trim
 *   B (hold)     auto-drive to LOADING ZONE  Y            re-home turret
 *                                            left trig    WALL SHOT preset (fire with right trig)
 *   X (hold)     turn toward nearest element (ElementVision; drive forward into it)
 *   dpad down (hold) intake jitter, to free a stuck element
 *   left bump    snap pose to Limelight      start        turret: call current spot zero
 *   options      reset field-centric heading back         reset storage count
 *   back         toggle field / robot centric left bump  "hive tipped" (flip target cell)
 *
 * BIOBUZZ: we aim at whichever of our two CELLS faces up - tracked from the AprilTags under
 * the cells, with driver 2's left bumper as an override. Opponent NECTAR (G408) or a 5th
 * element (G407) is spat straight back out. Rumble at 60 s (FLOWERS open) and 15 s (go park).
 *
 * Auto-fire: fires whenever storage has elements, we're in range, and the shot is ready.
 * Flywheel spins up on its own when loaded (AUTO_SPINUP_WHEN_LOADED).
 * Every TeleOp writes a CSV match log to /sdcard/FIRST/data/matchlogs.
 *
 * Feedback: gamepad1 double-buzz when storage fills; gamepad2 light is green when a shot is
 * ready, yellow while the turret is homing, red if homing failed. Both buzz at 60 s (FLOWERS) and 15 s (park) left.
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
    private final Vision vision = new Vision(robot);
    private final ElementVision elementVision = new ElementVision(robot);
    private boolean elementAssist = false;
    private FieldDrawing fieldDrawing;
    private MatchLogger log;

    private final ElapsedTime voltageTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime matchTimer = new ElapsedTime();
    private final ElapsedTime relocalizeTimer = new ElapsedTime();

    private double goalX, goalY, driverHeading;
    /** Field-centric "away from the driver" direction; options re-sets it without touching odometry. */
    private double fieldCentricHeading;
    private boolean red;
    // reused every loop (no allocation)
    private final double[] robotVelocity = new double[2];
    private Pose parkPose;
    private boolean parking = false;
    private String allianceColor;
    private Queue<String> elements;
    private boolean emergencyWas = false;
    private String intakeMode = "off";
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

        red = alliance() == Alliance.RED;
        goalX = FieldGeometry.cellX(red);
        goalY = FieldGeometry.cellY(FieldGeometry.startingUpCell(red));
        allianceColor = red ? "RED" : "BLUE";
        elements = storage.getElementQueue();
        driverHeading = red ? RobotConfig.RED_DRIVER_HEADING : RobotConfig.BLUE_DRIVER_HEADING;
        fieldCentricHeading = driverHeading;

        // Only trust what auto handed over if it's ours and recent (not a pit run of the other
        // alliance's auto an hour ago).
        poseFromAuto = PoseStorage.isFresh(allianceColor);
        // Continue from wherever auto left the hive (it almost always tips it)...
        if (poseFromAuto && PoseStorage.getHiveUpCell() != null) {
            vision.restoreHive(allianceColor, PoseStorage.getHiveUpCell(), PoseStorage.getHiveWeight());
        }
        elementVision.setAllianceColor(allianceColor);
        // ...and with what auto left in storage, so the G407 count of 4 is right from the start.
        if (poseFromAuto) {
            for (int i = 0; i < PoseStorage.getElementCount(); i++) {
                elements.add(PoseStorage.getElement(i));
            }
        }
        parkPose = red ? new Pose(14, FieldGeometry.RED_LOADING_ZONE_Y, 0)
                : new Pose(FieldGeometry.rotateX(14), FieldGeometry.rotateY(FieldGeometry.RED_LOADING_ZONE_Y), Math.PI);

        if (poseFromAuto) {
            robot.follower.setPose(PoseStorage.getPoseStoragePose());
        } else if (red) {
            robot.follower.setPose(new Pose(RobotConfig.TELEOP_DEFAULT_X, RobotConfig.TELEOP_DEFAULT_Y,
                    RobotConfig.TELEOP_DEFAULT_HEADING));
        } else {
            robot.follower.setPose(new Pose(FieldGeometry.rotateX(RobotConfig.TELEOP_DEFAULT_X),
                    FieldGeometry.rotateY(RobotConfig.TELEOP_DEFAULT_Y),
                    FieldGeometry.rotateHeading(RobotConfig.TELEOP_DEFAULT_HEADING)));
        }

        trigger.stopFiring();
        vision.activate();

        log = new MatchLogger(new File(AppUtil.ROBOT_DATA_DIR, "matchlogs"), "teleop_" + alliance(),
                "x", "y", "heading", "vx", "vy", "goalDist", "flyTarget", "flyVel", "flyReady",
                "turretAngle", "turretTarget", "turretOnTarget", "storage", "firing", "shotsTotal", "voltage", "loopMs");
    }

    @Override
    public void init_loop() {
        telemetry.addData("Alliance", alliance());
        telemetry.addData("Start pose", poseFromAuto ? "from auto" : "DEFAULT (no recent auto for this alliance)");
        telemetry.addData("Storage", "%d element(s) carried over", elements.size());
        telemetry.update();
    }

    @Override
    public void start() {
        started = true;
        if (poseFromAuto && PoseStorage.hasTurretZero()) {
            // Reuse auto's zero (the encoder kept counting). Homing from wherever auto left the
            // turret could sweep into a hard stop.
            turret.restoreZero(PoseStorage.getTurretZeroTicks());
        } else {
            turret.localizeTurret(); // home the turret on the hall effect while we drive off
        }
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
        int tipsBefore = vision.getHiveTips();
        vision.updateRead(allianceColor); // Limelight reads + hive tip state
        if (vision.getHiveTips() > tipsBefore) {
            gamepad2.rumbleBlips(1); // tip detected - now aiming at the other cell
        }
        if (gamepad2.leftBumperWasPressed()) {
            vision.flipHiveTipState();
        }
        goalY = FieldGeometry.cellY(vision.getHiveUpCell());
        autoRelocalize(pose, velocity);

        // ---------------- driver 1: drive + intake
        if (gamepad1.optionsWasPressed()) {
            // "Whichever way the robot faces now is away from me." Only the driving frame changes:
            // the odometry heading (which aims the turret) is left alone.
            fieldCentricHeading = heading;
        }
        if (gamepad1.backWasPressed()) {
            fieldCentric = !fieldCentric;
        }
        if (gamepad1.leftBumperWasPressed()
                && Math.hypot(velocity.vx, velocity.vy) < RobotConfig.AUTO_RELOCALIZE_MAX_SPEED * 3) {
            // (only while nearly stopped - the camera frame is ~40 ms old)
            Pose3D botpose = vision.getAverageBotPose();
            if (botpose != null) {
                drivetrain.relocalize(botpose);
                gamepad1.rumble(200);
            }
        }

        boolean wantOuttake = gamepad1.left_trigger > 0.3;

        // Storage: beam breaks update the queue; true = emergency outtake (G407 / G408).
        String headBefore = elements.peek();
        int launchedBefore = storage.getLaunchedCount();
        boolean emergency = storage.checkBeamBreaks(allianceColor);
        // The storage exit beam is our shot counter: an element left into the launcher.
        if (storage.getLaunchedCount() > launchedBefore) { // only the exit beam - jitter/outtake can't fake a shot
            int tips = vision.getHiveTips();
            vision.recordLaunch(headBefore);
            if (vision.getHiveTips() > tips) {
                gamepad2.rumbleBlips(2); // our shot tipped it: now aiming at the other cell
            }
            shotsFired++; // logged as a running total: a 1-loop flag would fall between 20 Hz log rows
        }

        boolean notFull = elements.size() < RobotConfig.STORAGE_CAPACITY;
        if (emergency) {
            intake.emergencyOuttake();
            intakeMode = "EMERGENCY OUTTAKE";
            if (!emergencyWas) {
                gamepad1.rumble(150);
            }
        } else if (wantOuttake) {
            intake.setIntakePower(RobotConfig.OUTTAKE_POWER);
            intakeMode = "outtake";
        } else {
            boolean full = gamepad1.right_trigger > 0.8 && notFull;
            // jitter alone wiggles at slow power (and never when full - G407)
            boolean slow = (gamepad1.right_trigger > 0.3 || gamepad1.dpad_down) && !full && notFull;
            intake.updateIntake(full, slow, gamepad1.dpad_down);
            intakeMode = gamepad1.dpad_down ? "jitter" : full ? "full" : slow ? "slow" : "off";
        }
        emergencyWas = emergency;

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
            turret.localizeTurret();
        }
        if (gamepad2.startWasPressed()) {
            turret.forceZeroHere(); // fallback if the hall sensor is dead: point it forward first
        }
        if (gamepad2.backWasPressed()) {
            elements.clear();
        }
        if (gamepad2.xWasPressed()) {
            autoFire = !autoFire;
            gamepad2.rumble(autoFire ? 300 : 100);
        }
        if (gamepad2.bWasPressed()) {
            manualTurret = !manualTurret;
        }

        robotVelocity[0] = velocity.vx;
        robotVelocity[1] = velocity.vy;
        shooterConfig.calculateShootingConfig(vision.getAverageTa(),
                drivetrain.getPedroDistanceToHive(pose, allianceColor, vision.getHiveTipState()),
                robotVelocity, vision.getTargetHeading());
        boolean loaded = !elements.isEmpty();
        boolean wallShot = gamepad2.left_trigger > 0.3;
        boolean firing = gamepad2.right_trigger > 0.3
                || (autoFire && loaded && ShooterConfig.isInRange());

        // ---------------- drive (after we know whether we're firing)
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;
        // Element assist: the Limelight switches to the element pipeline while X is held
        // (hive tags aren't tracked meanwhile - HiveTracker's shot model still runs).
        if (gamepad1.x != elementAssist) {
            elementAssist = gamepad1.x;
            if (elementAssist) {
                elementVision.beginAutoFind();
            } else {
                elementVision.stopAutoFind();
            }
        }

        // elementDetected() never reports opponent NECTAR (G408)
        if (elementAssist && elementVision.elementDetected()) {
            parking = false;
            drivetrain.driveAiming(forward, strafe, Drivetrain.bearingTurn(elementVision.getElementDeltaX()),
                    heading, fieldCentricHeading, fieldCentric, gamepad1.right_bumper);
        } else if (gamepad1.b) {
            // Park assist: Pedro drives us into the LOADING ZONE (PARK = 5 pts). Release = manual.
            if (!parking) {
                parking = true;
                drivetrain.followPath(line(pose, parkPose).linear(pose, parkPose));
            }
        } else if (gamepad1.a) {
            parking = false;
            // Turret-down fallback: point the chassis at the (shoot-on-the-move) aim heading.
            drivetrain.driveAiming(forward, strafe, Drivetrain.aimTurn(ShooterConfig.aimHeading(), heading),
                    heading, fieldCentricHeading, fieldCentric, gamepad1.right_bumper);
        } else if (parkingReleased()) {
            // just let go of B: fall through to manual next loop
        } else if (firing && RobotConfig.LOCK_POSITION_WHILE_FIRING && !Drivetrain.sticksActive(forward, strafe, turn)) {
            drivetrain.lockPosition();
        } else {
            drivetrain.drive(forward, strafe, turn, heading, fieldCentricHeading, fieldCentric, gamepad1.right_bumper);
        }

        // ---------------- shooter
        if (manualTurret) {
            turret.updateManual(-gamepad2.right_stick_x);
        } else {
            turret.setTrim(turretTrim);
            turret.updateTurret(); // aims along ShooterConfig's launch vector
        }
        // Manual turret / chassis aim: the driver is the judge of "on target".
        // (chassis aim assumes the turret is parked at 0 - not mid-homing-sweep)
        // A wall shot is lined up by the driver against the wall, so it doesn't wait on the turret.
        boolean aimed = (manualTurret || wallShot || (gamepad1.a && turret.getState() != Turret.TurretState.LOCALIZING)
                || turret.goodTurretConfig())
                && !vision.isHiveSettling(); // don't launch into a swinging hive

        String next = elements.peek();
        boolean spinUp = flywheelOn || firing || (RobotConfig.AUTO_SPINUP_WHEN_LOADED && loaded);
        if (wallShot) {
            launcher.setShooterWallSpeed();
        } else if (spinUp) {
            launcher.setLauncher(elements); // speed + hood from ShooterConfig, element from .peek()
        } else {
            launcher.stop();
        }

        if (firing) {
            trigger.fire(launcher.goodShooterConfig(), aimed);
        } else {
            trigger.stopFiring();
        }

        updateStats();
        driverFeedback(aimed);
        if (log.shouldLog(matchTimer.seconds())) {
            log.value(pose.x()).value(pose.y()).value(heading).value(velocity.vx).value(velocity.vy)
                    .value(ShooterConfig.distanceToGoal).value(launcher.getTargetVelocity())
                    .value(launcher.getCurrVelocity()).value(launcher.goodShooterConfig())
                    .value(turret.getAngle()).value(turret.getTargetAngle()).value(turret.goodTurretConfig())
                    .value(elements.size()).value(firing).value(shotsFired).value(voltage).value(loopMs)
                    .endRow();
        }

        // ---------------- telemetry
        fieldDrawing.draw(pose.x(), pose.y(), heading, ShooterConfig.aimHeading(),
                ShooterConfig.distanceToGoal, goalX, goalY);
        telemetry.addData("Time left", "%.0f s", RobotConfig.TELEOP_LENGTH - matchTimer.seconds());
        telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f°", pose.x(), pose.y(), Math.toDegrees(heading));
        telemetry.addData("Drive", "%s%s", fieldCentric ? "field centric" : "robot centric",
                drivetrain.isLocked() ? "  LOCKED" : "");
        telemetry.addData("Storage", "%d  next=%s  intake beam %s", elements.size(), next,
                storage.anyIntakeBeamBroken() ? "BROKEN" : "-");
        telemetry.addData("Intake", intakeMode);
        telemetry.addData("Goal dist", "%.1f in%s", ShooterConfig.distanceToGoal, wallShot ? "  WALL SHOT" : "");
        telemetry.addData("Flywheel", "%.0f / %.0f %s", launcher.getCurrVelocity(), launcher.getTargetVelocity(),
                launcher.goodShooterConfig() ? "READY" : "");
        telemetry.addData("Turret", "%s  %.1f° → %.1f°  trim %.0f°", turret.getState(),
                Math.toDegrees(turret.getAngle()), Math.toDegrees(turret.getTargetAngle()), Math.toDegrees(turretTrim));
        if (turret.encoderLooksReversed()) {
            telemetry.addLine("⚠ TURRET: encoder counts backwards - flip RobotConfig.TURRET_ENCODER_REVERSED");
        }
        telemetry.addData("Hive", "%s (%s cell)  load %.1f/%.0f  tags %s  tips %d", vision.getHiveTipState(),
                vision.getHiveUpCell(), vision.getHiveLoad(), HiveTracker.TIP_WEIGHT,
                vision.hasTarget() ? "seen" : "-", vision.getHiveTips());
        if (parking) {
            telemetry.addLine("PARK ASSIST driving to loading zone");
        }
        telemetry.addData("Modes", "%s%s%s", autoFire ? "AUTO-FIRE " : "", manualTurret ? "MANUAL-TURRET " : "",
                gamepad1.a ? "CHASSIS-AIM" : "");
        telemetry.addData("Stats", "shots %d  last cycle %.1f s", shotsFired, lastCycleTime);
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

    /** Clears the park-assist flag the loop B is released; true only on that loop. */
    private boolean parkingReleased() {
        if (parking) {
            parking = false;
            return true;
        }
        return false;
    }

    /** Cycle time = time between storage emptying out, i.e. one full collect-and-score loop. */
    private void updateStats() {
        boolean empty = elements.isEmpty();
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
        if (!RobotConfig.AUTO_RELOCALIZE || relocalizeTimer.seconds() < RobotConfig.AUTO_RELOCALIZE_PERIOD) {
            return;
        }
        if (Math.hypot(velocity.vx, velocity.vy) > RobotConfig.AUTO_RELOCALIZE_MAX_SPEED) {
            return;
        }
        relocalizeTimer.reset(); // (getAverageBotPose allocates - at most once per period)
        Pose3D botpose = vision.getAverageBotPose();
        if (botpose == null) {
            return;
        }
        double ftcX = botpose.getPosition().unit.toInches(botpose.getPosition().x);
        double ftcY = botpose.getPosition().unit.toInches(botpose.getPosition().y);
        double error = Math.hypot(FieldGeometry.ftcToFieldX(ftcX, ftcY) - pose.x(),
                FieldGeometry.ftcToFieldY(ftcX, ftcY) - pose.y());
        if (error > RobotConfig.AUTO_RELOCALIZE_MIN_ERROR) {
            drivetrain.relocalize(botpose);
        }
    }

    /** Rumble and lights. Only sends commands on changes - they go over the network. */
    private void driverFeedback(boolean aimed) {
        boolean full = elements.size() >= RobotConfig.STORAGE_CAPACITY;
        if (full && !wasFull) {
            gamepad1.rumbleBlips(2);
        }
        wasFull = full;

        int led;
        if (turret.getState() == Turret.TurretState.HOMING_FAILED) {
            led = LED_FAILED;
        } else if (turret.getState() == Turret.TurretState.LOCALIZING) {
            led = LED_HOMING;
        } else if (launcher.goodShooterConfig() && aimed) {
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
