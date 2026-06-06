package org.firstinspires.ftc.teamcode.hardwaresystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

/**
 * Claw with one continuous rotation servo to intake game elements.
 * <p>
 * See {@link DoubleServoIntakeClaw} for the double servo version.
 */
public class SingleServoIntakeClaw extends Claw {
    public static class Builder extends Claw.Builder {
        /**
         * Used to set {@link #INTAKE_SERVO}.
         */
        protected CRServo intakeServo;
        /**
         * Used to set {@link #INTAKE_SENSOR}.
         */
        protected DigitalChannel intakeSensor;

        /**
         * See {@link #INTAKE_POWER}.
         */
        protected double intakePower;
        /**
         * See {@link #EJECT_POWER}.
         */
        protected double ejectPower;

        /**
         * Instantiate a {@link SingleServoIntakeClaw} with no movement servos,
         * intake CR servo, or touch sensor; intake power set to 1.0; and eject
         * power set to 0.5.
         */
        public Builder() {
            super();
            intakeServo = null;
            intakeSensor = null;

            intakePower = 1.0;
            ejectPower = 0.5;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public Claw build() {
            return null;
        }
    }

    /**
     * How much power the intake spins with when intaking.
     */
    private static final double INTAKE_POWER = 0.5;
    /**
     * How much power the intake spines with when ejecting.
     */
    private static final double EJECT_POWER = -1.0;

    /**
     * The servo that spins the intake.
     */
    private final CRServo INTAKE_SERVO;
    /**
     * The touch sensor that touches whether there is a piece in the intake.
     */
    private final DigitalChannel INTAKE_SENSOR;

    /**
     * Instantiate a new {@link SingleServoIntakeClaw} object with three servos
     * to control rotation in three degrees of freedom and a servo to control
     * intake and output.
     * <p>
     * The servo arguments may be {@code null}. If so, any commands to a
     * {@code null} servo are safe, but will do nothing.
     *
     * @param rollServo   The servo that controls roll, i.e., rotation along the
     *                    x-axis.
     * @param pitchServo  The servo that controls pitch, i.e., rotation along
     *                    the y-axis.
     * @param yawServo    The servo that controls yaw, i.e., rotation along the
     *                    z-axis.
     * @param intakeServo The servo that controls intake and output.
     */
    public SingleServoIntakeClaw(
        Servo rollServo,
        Servo pitchServo,
        Servo yawServo,
        CRServo intakeServo
    ) {
        this(rollServo, pitchServo, yawServo, intakeServo, null);
    }

    /**
     * Instantiate a new {@link SingleServoIntakeClaw} object with three servos
     * to control rotation in three degrees of freedom, a servo to control
     * intake and output, and a touch sensor to detect whether an object has
     * been picked up.
     * <p>
     * The servo arguments may be {@code null}. If so, any commands to a
     * {@code null} servo are safe, but will do nothing.
     *
     * @param rollServo    The servo that controls roll, i.e., rotation along
     *                     the x-axis.
     * @param pitchServo   The servo that controls pitch, i.e., rotation along
     *                     the y-axis.
     * @param yawServo     The servo that controls yaw, i.e., rotation along the
     *                     z-axis.
     * @param intakeServo  The servo that controls intake and output.
     * @param intakeSensor The touch sensor that detects whether an object has
     *                     been picked up.
     */
    public SingleServoIntakeClaw(
        Servo rollServo,
        Servo pitchServo,
        Servo yawServo,
        CRServo intakeServo,
        DigitalChannel intakeSensor
    ) {
        super(rollServo, pitchServo, yawServo);

        INTAKE_SERVO = intakeServo;
        INTAKE_SENSOR = intakeSensor;
    }

    /**
     * Get the power that the {@link #INTAKE_SERVO} intakes with.
     *
     * @return The power that the {@link #INTAKE_SERVO} intakes with.
     */
    public double getIntakePower() {
        return INTAKE_POWER;
    }

    /**
     * Get the power that the {@link #INTAKE_SERVO} ejects with.
     *
     * @return The power that the {@link #INTAKE_SERVO} ejects with.
     */
    public double getEjectPower() {
        return EJECT_POWER;
    }

    /**
     * Get all the {@link CRServo}s used by this claw, which should just be the
     * {@link #INTAKE_SERVO}.
     *
     * @return all the {@link CRServo}s used by this claw, which should just be
     * the {@link #INTAKE_SERVO}.
     */
    public Set<CRServo> getCrServos() {
        return new HashSet<>(Collections.singletonList(INTAKE_SERVO));
    }

    /**
     * Get the intake servo, which draws objects into the claw.
     *
     * @return The intake servo, which draws objects into the claw.
     */
    public CRServo getIntakeServo() {
        return INTAKE_SERVO;
    }

    /**
     * Set the {@link #INTAKE_SERVO} to draw in any objects that come into
     * contact with it.
     */
    public void startIntake() {
        if (INTAKE_SERVO != null) {
            INTAKE_SERVO.setPower(INTAKE_POWER);
        }
    }

    /**
     * Get whether the intake servo is currently running. If the servo is
     * {@code null}, trivially return {@code false}.
     *
     * @return {@code true} if the intake servo's power is non-zero.
     * <p>
     * {@code false} otherwise, including if the servo is {@code null}.
     */
    public boolean isIntakeActive() {
        return INTAKE_SERVO != null && INTAKE_SERVO.getPower() != 0;
    }

    /**
     * Stop the {@link #INTAKE_SERVO} from running. Do nothing if the servo is
     * {@code null}.
     */
    public void stopIntake() {
        if (INTAKE_SERVO != null) {
            INTAKE_SERVO.setPower(0);
        }
    }

    /**
     * Make the intake spin in reverse and eject the object.
     */
    public void ejectIntake() {
        if (INTAKE_SERVO != null) {
            INTAKE_SERVO.setPower(EJECT_POWER);
        }
    }

    /**
     * Get whether the sensor on the claw is pressed or not. If the sensor is
     * {@code null}, trivially return {@code false}.
     *
     * @return {@code true} when the sensor is pressed.
     * <p>
     * {@code false} otherwise, including if the sensor is {@code null}.
     */
    public boolean isSensorPressed() {
        // Returns true when the sensor is not pressed.
        return INTAKE_SENSOR != null && !INTAKE_SENSOR.getState();
    }
}