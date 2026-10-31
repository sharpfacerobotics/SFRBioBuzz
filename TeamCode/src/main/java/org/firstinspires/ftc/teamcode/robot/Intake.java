package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Intake roller and flower ramp. Call robot.initIntake(hardwareMap) before using it.
 *
 * <ul>
 *   <li>{@link #emergencyOuttake()}: CONTINUOUS. Call while Storage.checkBeamBreaks() returns true.</li>
 *   <li>{@link #setIntakePower(double)}: SET. Call once with gamepad or other input.</li>
 *   <li>{@link #updateIntake(boolean, boolean, boolean)}: CONTINUOUS. Runs full / slow, with
 *       jitter (a quick forward-back wiggle) to free a stuck element if needed.</li>
 *   <li>{@link #lowerRamp()} / {@link #retractRamp()}: SET. The flower ramp (one servo).</li>
 * </ul>
 *
 * <p>The OpMode decides when not to intake (e.g. storage already holds 4, G407) and passes
 * that in. Intake never reads another subsystem.</p>
 */
public class Intake {
    private final RobotHardware robot;

    // Made once here, never inside the loop.
    private final ElapsedTime jitterTimer = new ElapsedTime();
    private boolean jittering = false;
    private boolean rampLowered = false;

    public Intake(RobotHardware robot) {
        this.robot = robot;
    }

    /** CONTINUOUS. Spit out whatever is in the intake (opponent NECTAR, a 5th element). */
    public void emergencyOuttake() {
        apply(RobotConfig.OUTTAKE_POWER);
    }

    /** SET. Raw intake power, [-1, 1] (negative = outtake). */
    public void setIntakePower(double power) {
        apply(power);
    }

    /**
     * CONTINUOUS. Loop call.
     *
     * @param runFull intake at INTAKE_POWER
     * @param runSlow intake at INTAKE_SLOW_POWER (lining up, or nearly full)
     * @param jitter  alternate in and out every INTAKE_JITTER_PERIOD to free a stuck element
     */
    public void updateIntake(boolean runFull, boolean runSlow, boolean jitter) {
        double power;
        if (runFull) {
            power = RobotConfig.INTAKE_POWER;
        } else if (runSlow) {
            power = RobotConfig.INTAKE_SLOW_POWER;
        } else {
            power = 0;
        }

        if (jitter && !jittering) {
            jitterTimer.reset(); // start every jitter on a forward phase
        }
        jittering = jitter;
        if (jitter && power != 0) {
            // Every other period, reverse. Only wiggles at the power the OpMode allowed, so a
            // full storage (power 0) never gets a 5th element pulled in by jitter (G407).
            boolean reverse = ((int) (jitterTimer.seconds() / RobotConfig.INTAKE_JITTER_PERIOD)) % 2 == 1;
            power = reverse ? -power : power;
        }
        apply(power);
    }

    /** SET. Put the flower ramp down. */
    public void lowerRamp() {
        robot.flowerRampServo.setPosition(RobotConfig.FLOWER_RAMP_LOWERED);
        rampLowered = true;
    }

    /** SET. Bring the flower ramp back up. */
    public void retractRamp() {
        robot.flowerRampServo.setPosition(RobotConfig.FLOWER_RAMP_RETRACTED);
        rampLowered = false;
    }

    /** STATUS. Last ramp command (the servo gives no feedback). */
    public boolean isRampLowered() {
        return rampLowered;
    }

    private void apply(double power) {
        robot.intakePowerCommand = power; // Storage reads this to know which way elements move
        robot.intakeMotor.setPower(power);
    }
}