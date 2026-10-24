package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Intake roller. Owner: Vibhav.
 *
 * <p>Glossary methods (Software Documentation, "Intake"):</p>
 * <ul>
 *   <li>{@link #emergencyOuttake()}: CONTINUOUS. Call while {@code Storage.checkBeamBreaks()}
 *       returns true.</li>
 *   <li>{@link #setIntakePower(double)}: SET. Call once with gamepad or other input.</li>
 *   <li>{@link #updateIntake(boolean, boolean, boolean)}: CONTINUOUS. Runs full / slow, with
 *       jitter (a quick forward-back wiggle) to free a stuck element if needed.</li>
 * </ul>
 *
 * <p>The OpMode decides when not to intake (e.g. storage already holds 4, G407) and passes
 * that in. Intake never reads another subsystem.</p>
 */
public class Intake {
    private final RobotHardware robot;

    // Made once here, never inside the loop.
    private final ElapsedTime jitterTimer = new ElapsedTime();

    public Intake(RobotHardware robot) {
        this.robot = robot;
    }

    /** CONTINUOUS. Spit out whatever is in the intake (opponent NECTAR, a 5th element). */
    public void emergencyOuttake() {
        robot.intakeMotor.setPower(RobotConfig.OUTTAKE_POWER);
    }

    /** SET. Raw intake power, [-1, 1] (negative = outtake). */
    public void setIntakePower(double power) {
        robot.intakeMotor.setPower(power);
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

        if (jitter) {
            // Every other period, reverse. Uses the full power if nothing else was asked for.
            double magnitude = power != 0 ? power : RobotConfig.INTAKE_POWER;
            boolean reverse = ((int) (jitterTimer.seconds() / RobotConfig.INTAKE_JITTER_PERIOD)) % 2 == 1;
            power = reverse ? -magnitude : magnitude;
        }
        robot.intakeMotor.setPower(power);
    }
}
