package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Continuous-rotation-servo turret with an external encoder and a hall effect zero. Owner: Alex.
 *
 * <p>States:</p>
 * <ul>
 *   <li>LOCALIZING - find the hall effect zero with a "cow path" search: sweep right by s,
 *       left to -2s, right to +4s, ... (doubling) until the magnet is found. Doubling keeps
 *       total travel within ~9x of the true distance whichever side zero is on.</li>
 *   <li>AIMING - PD + static friction + feedforward on the chassis yaw rate.</li>
 *   <li>UNWINDING - the target is on the far side of a soft limit, so we go the long way
 *       round rather than wrap the cable. Not "on target" until done.</li>
 * </ul>
 *
 * <p>Angles are radians relative to robot forward, CCW positive.</p>
 */
public class Turret {
    private final RobotHardware robot;

    public enum TurretState { LOCALIZING, AIMING, UNWINDING, HOMING_FAILED }
    private TurretState turretState = TurretState.LOCALIZING;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime homingTimer = new ElapsedTime();
    private double zeroOffsetTicks = 0;
    private double angle = 0;
    private double lastAngle = 0;
    private double targetAngle = 0;
    private double error = 0;
    private double power = 0;
    private boolean hallWasActive = false;

    // cow path search state
    private double homingStartTicks = 0;
    private double homingSweep = 0;
    private int homingDirection = 1;
    private boolean homingStarted = false;
    private int homingFullSweeps = 0;
    private double trim = 0;
    private double stallCheckTicks = 0;
    private double segmentStartTicks = 0;
    private final ElapsedTime stallTimer = new ElapsedTime();
    private final ElapsedTime segmentTimer = new ElapsedTime();
    private boolean encoderReversedSuspected = false;

    public Turret(RobotHardware robot) {
        this.robot = robot;
    }

    /**
     * Call once per loop.
     *
     * @param desiredAngle      turret angle to aim at (ShooterConfig.getTurretAngle() + any driver trim)
     * @param robotAngularVel   chassis yaw rate (rad/s, CCW+) from the follower, for feedforward
     */
    // Glossary: updateTurret() UPDATE, localizeTurret() START, goodTurretConfig() STATUS.

    /**
     * UPDATE. Loop call. Handles homing, aiming and unwinding. Aims along ShooterConfig's
     * launch vector (team design: read from ShooterConfig) relative to the robot's heading
     * from the follower, plus any driver trim.
     */
    public void updateTurret() {
        double robotHeading = robot.follower.pose().heading();
        double desired = ShooterConfig.aimHeading() - robotHeading + trim;
        while (desired > Math.PI) desired -= 2 * Math.PI;
        while (desired <= -Math.PI) desired += 2 * Math.PI;
        updateTurretToAngle(desired, robot.follower.velocity().omega);
    }

    /** START. Begin the hall-effect search for zero; check goodTurretConfig()/getState(). */
    public void localizeTurret() {
        relocalize();
    }

    /** STATUS. True if the turret is aimed within tolerance - check before shooting. */
    public boolean goodTurretConfig() {
        return isOnTarget();
    }

    /** SET. Driver aim trim (rad, CCW +) added to the computed aim. */
    public void setTrim(double trimRadians) {
        trim = trimRadians;
    }

    /**
     * Aim at a turret angle directly (TurretTuner, tests).
     *
     * @param desiredAngle    turret angle relative to robot forward (rad)
     * @param robotAngularVel chassis yaw rate (rad/s, CCW+), for feedforward
     */
    public void updateTurretToAngle(double desiredAngle, double robotAngularVel) {
        double deltaTime = timer.seconds();
        timer.reset();
        double ticks = readTicks();
        boolean hallActive = isHallActive();

        if (turretState == TurretState.LOCALIZING) {
            searchForZero(ticks, hallActive);
            hallWasActive = hallActive;
            return;
        }
        if (turretState == TurretState.HOMING_FAILED) {
            robot.turretServo.setPower(0);
            return;
        }

        relocalizeIfHallHit(ticks, hallActive);
        hallWasActive = hallActive;

        angle = (ticks - zeroOffsetTicks) / RobotConfig.TURRET_TICKS_PER_RAD;
        double angularVel = deltaTime > 0 ? (angle - lastAngle) / deltaTime : 0;
        lastAngle = angle;

        aim(desiredAngle, angularVel, robotAngularVel);
    }

    public boolean isOnTarget() {
        return turretState == TurretState.AIMING
                && Math.abs(error) < RobotConfig.TURRET_ANGLE_TOLERANCE;
    }

    public boolean isLocalized() {
        return turretState == TurretState.AIMING || turretState == TurretState.UNWINDING;
    }

    /**
     * Driver-controlled turret (fallback when auto-aim is wrong). Still respects the soft
     * limits once the turret knows where zero is.
     *
     * @param stick [-1, 1], + = CCW
     */
    public void updateManual(double stick) {
        timer.reset();
        if (turretState == TurretState.LOCALIZING) {
            // Driver took over mid-search: restart the search cleanly when auto-aim resumes
            // (otherwise its timer runs out and it fails), and take the magnet if we pass it.
            homingStarted = false;
            if (isHallActive()) {
                forceZeroHere();
            }
        }
        if (isLocalized()) {
            angle = (readTicks() - zeroOffsetTicks) / RobotConfig.TURRET_TICKS_PER_RAD;
            lastAngle = angle;
        }
        power = Range.clip(stick * RobotConfig.MANUAL_TURRET_SCALE,
                -RobotConfig.MAX_TURRET_POWER, RobotConfig.MAX_TURRET_POWER);
        if (isLocalized() && ((angle >= RobotConfig.TURRET_MAX_ANGLE && power > 0)
                || (angle <= RobotConfig.TURRET_MIN_ANGLE && power < 0))) {
            power = 0;
        }
        robot.turretServo.setPower(power);
    }

    /** Restart the hall effect search (e.g. driver button if the turret got bumped). */
    public void relocalize() {
        turretState = TurretState.LOCALIZING;
        homingStarted = false;
    }

    /** Skip homing and call the current position zero (use if the hall sensor dies). */
    public void forceZeroHere() {
        zeroOffsetTicks = readTicks();
        lastAngle = 0;
        turretState = TurretState.AIMING;
    }

    /**
     * Encoder ticks (raw, as read) at turret zero, to hand from auto to TeleOp through
     * PoseStorage. The encoder keeps counting between OpModes, so TeleOp can reuse it instead
     * of homing again - homing from wherever auto left the turret could sweep into a hard stop.
     */
    public double getZeroOffsetTicks() {
        return zeroOffsetTicks;
    }

    /** Resume with a zero found earlier (see getZeroOffsetTicks). */
    public void restoreZero(double zeroOffsetTicks) {
        this.zeroOffsetTicks = zeroOffsetTicks;
        lastAngle = (readTicks() - zeroOffsetTicks) / RobotConfig.TURRET_TICKS_PER_RAD;
        turretState = TurretState.AIMING;
    }

    /** True if homing gave up because the encoder seemed to count the wrong way. */
    public boolean encoderLooksReversed() {
        return encoderReversedSuspected;
    }

    public void stop() {
        power = 0;
        robot.turretServo.setPower(0);
    }

    public void restartTimer() {
        timer.reset();
    }

    public TurretState getState() {
        return turretState;
    }

    public double getAngle() {
        return angle;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getError() {
        return error;
    }

    public double getPower() {
        return power;
    }

    // ------------------------------------------------------------------ aiming

    private void aim(double desiredAngle, double angularVel, double robotAngularVel) {
        targetAngle = chooseReachableTarget(desiredAngle);
        error = targetAngle - angle;
        turretState = Math.abs(error) > RobotConfig.TURRET_UNWIND_THRESHOLD
                ? TurretState.UNWINDING : TurretState.AIMING;

        // D on measurement (not error) so a jump in target doesn't kick the turret.
        power = RobotConfig.TURRET_KP * error
                - RobotConfig.TURRET_KD * angularVel
                - RobotConfig.TURRET_KV_ROBOT_OMEGA * robotAngularVel;
        if (Math.abs(error) > RobotConfig.TURRET_ANGLE_TOLERANCE) {
            power += Math.signum(error) * RobotConfig.TURRET_KS;
        }
        power = Range.clip(power, -RobotConfig.MAX_TURRET_POWER, RobotConfig.MAX_TURRET_POWER);

        // Never drive further past a soft limit.
        if ((angle >= RobotConfig.TURRET_MAX_ANGLE && power > 0)
                || (angle <= RobotConfig.TURRET_MIN_ANGLE && power < 0)) {
            power = 0;
        }
        robot.turretServo.setPower(power);
    }

    private double chooseReachableTarget(double desired) {
        return chooseReachableTarget(desired, angle);
    }

    /**
     * The same heading can be reached at angle, angle + 2pi or angle - 2pi. Pick whichever is
     * inside the soft limits and closest to where we are now; if none is, clamp to a limit.
     */
    static double chooseReachableTarget(double desired, double current) {
        double best = Double.NaN;
        double bestDistance = Double.MAX_VALUE;
        for (int k = -1; k <= 1; k++) {
            double candidate = desired + k * 2.0 * Math.PI;
            if (candidate < RobotConfig.TURRET_MIN_ANGLE || candidate > RobotConfig.TURRET_MAX_ANGLE) {
                continue;
            }
            double distance = Math.abs(candidate - current);
            if (distance < bestDistance) {
                bestDistance = distance;
                best = candidate;
            }
        }
        if (Double.isNaN(best)) {
            best = Range.clip(desired, RobotConfig.TURRET_MIN_ANGLE, RobotConfig.TURRET_MAX_ANGLE);
        }
        return best;
    }

    // ------------------------------------------------------------------ localization

    private void searchForZero(double ticks, boolean hallActive) {
        if (!homingStarted) {
            homingStarted = true;
            homingStartTicks = ticks;
            homingSweep = RobotConfig.TURRET_HOMING_FIRST_SWEEP;
            homingDirection = 1;
            homingFullSweeps = 0;
            homingTimer.reset();
            stallTimer.reset();
            stallCheckTicks = ticks;
            segmentStartTicks = ticks;
            segmentTimer.reset();
        }

        if (hallActive) {
            zeroOffsetTicks = ticks;
            lastAngle = 0;
            angle = 0;
            homingStarted = false;
            turretState = TurretState.AIMING;
            robot.turretServo.setPower(0);
            return;
        }

        // If the encoder is unplugged or reversed the sweep never "ends", so cap it in time too.
        if (homingTimer.seconds() > RobotConfig.TURRET_HOMING_TIMEOUT) {
            failHoming();
            return;
        }

        double travelled = (ticks - homingStartTicks) / RobotConfig.TURRET_TICKS_PER_RAD;

        // Wrong way: powered for a while but the encoder counts against the power - the
        // encoder direction is wrong, so stop now instead of driving one way until the timeout.
        // (Measured from the start of THIS sweep - the cow path crosses back over the start.)
        double segment = homingDirection * (ticks - segmentStartTicks) / RobotConfig.TURRET_TICKS_PER_RAD;
        if (segment < -RobotConfig.TURRET_HOMING_WRONG_WAY) {
            encoderReversedSuspected = true;
            failHoming();
            return;
        }

        // Stalled against something (a hard stop we didn't expect): treat it as this sweep's end.
        // Only once the sweep is properly under way - right after reversing it barely moves.
        boolean stalled = false;
        if (segmentTimer.seconds() > 2 * RobotConfig.TURRET_STALL_TIME
                && stallTimer.seconds() > RobotConfig.TURRET_STALL_TIME) {
            stalled = Math.abs(ticks - stallCheckTicks) / RobotConfig.TURRET_TICKS_PER_RAD
                    < RobotConfig.TURRET_STALL_ANGLE;
            stallTimer.reset();
            stallCheckTicks = ticks;
        }

        if (homingDirection * travelled >= homingSweep || stalled) {
            // Reached this sweep's end without finding zero: turn round and go twice as far,
            // but never further than the turret can physically travel.
            if (homingSweep >= RobotConfig.TURRET_HOMING_MAX_SWEEP || stalled) {
                homingFullSweeps++; // this side is searched to its limit
                if (homingFullSweeps >= 2) { // searched the whole range both ways
                    failHoming();
                    return;
                }
            }
            homingDirection = -homingDirection;
            homingSweep = Math.min(homingSweep * 2.0, RobotConfig.TURRET_HOMING_MAX_SWEEP);
            segmentStartTicks = ticks;
            segmentTimer.reset();
            stallTimer.reset();
            stallCheckTicks = ticks;
        }
        robot.turretServo.setPower(homingDirection * RobotConfig.TURRET_HOMING_POWER);
    }

    /** Correct encoder drift whenever we pass the magnet. */
    private void relocalizeIfHallHit(double ticks, boolean hallActive) {
        if (hallActive && !hallWasActive) {
            double angleNow = (ticks - zeroOffsetTicks) / RobotConfig.TURRET_TICKS_PER_RAD;
            if (Math.abs(angleNow) > RobotConfig.TURRET_RELOCALIZE_THRESHOLD) {
                zeroOffsetTicks = ticks;
                lastAngle = 0;
            }
        }
    }

    private void failHoming() {
        homingStarted = false;
        turretState = TurretState.HOMING_FAILED;
        robot.turretServo.setPower(0);
    }

    /** Encoder ticks with the sign fixed so +power always means +ticks. */
    private double readTicks() {
        double ticks = robot.turretEncoder.getCurrentPosition();
        return RobotConfig.TURRET_ENCODER_REVERSED ? -ticks : ticks;
    }

    /** Raw encoder, for SubsystemTest. */
    public double getRawTicks() {
        return robot.turretEncoder.getCurrentPosition();
    }

    public boolean isHallTriggered() {
        return isHallActive();
    }

    /** Open-loop power for SubsystemTest only (bypasses limits). */
    public void setRawPower(double power) {
        this.power = power;
        robot.turretServo.setPower(power);
    }

    /** REV magnetic limit switch reads LOW when the magnet is present. */
    private boolean isHallActive() {
        return !robot.turretHallEffect.getState();
    }
}
