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
@SuppressWarnings("unused")
public class SingleServoIntakeClaw extends Claw {
    /**
     * Simplifies the instantiation of {@link SingleServoIntakeClaw} objects.
     */
    public static class Builder extends Claw.Builder {
        /**
         * The continuous rotation servo that takes in objects. Used to set
         * {@link #INTAKE_SERVO}.
         */
        protected CRServo intakeServo;

        /**
         * The power used by the {@link #intakeServo} to take in objects. Used
         * to set {@link #INTAKE_POWER}.
         */
        protected double intakePower;
        /**
         * The power used by the {@link #intakeServo} to eject objects. Used to
         * set {@link #EJECT_POWER}.
         */
        protected double ejectPower;

        /**
         * The sensor that detects whether an object has been taken in. Used to
         * set {@link #INTAKE_SENSOR}.
         */
        protected DigitalChannel intakeSensor;

        /**
         * Instantiate a {@link SingleServoIntakeClaw} with no movement servos,
         * intake CR servo, or touch sensor; intake power set to 0.5; and eject
         * power set to -1.0.
         */
        public Builder() {
            super();
            intakeServo = null;
            intakeSensor = null;

            intakePower = 0.5;
            ejectPower = -1.0;
        }


        /**
         * Set the {@link Servo} used to control roll (see
         * {@link #ROLL_SERVO}).
         *
         * @param rollServo The servo used to control roll.
         * @return This {@link DoubleServoIntakeClaw.Builder} to allow for
         * chaining setters.
         */
        @Override
        public Builder setRollServo(Servo rollServo) {
            return (Builder) super.setRollServo(rollServo);
        }

        /**
         * Set the {@link Servo} used to control pitch (see
         * {@link #PITCH_SERVO}).
         *
         * @param pitchServo The servo used to control pitch.
         * @return This {@link DoubleServoIntakeClaw.Builder} to allow for
         * chaining setters.
         */
        @Override
        public Builder setPitchServo(Servo pitchServo) {
            return (Builder) super.setPitchServo(pitchServo);
        }

        /**
         * Set the {@link Servo} used to control yaw (see {@link #YAW_SERVO}).
         *
         * @param yawServo The servo used to control yaw.
         * @return This {@link DoubleServoIntakeClaw.Builder} to allow for
         * chaining setters.
         */
        @Override
        public Builder setYawServo(Servo yawServo) {
            return (Builder) super.setYawServo(yawServo);
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public Builder setServoIncrement(double servoIncrement) {
            return (Builder) super.setServoIncrement(servoIncrement);
        }

        /**
         * Return whether the current attributes are valid, which is
         * {@code true} if and only if {@link #servoIncrement} is positive, both
         * powers are positive, and the intake servo is non-{@code null}.
         * <p>
         * {@code null} values for {@link #rollServo}, {@link #pitchServo},
         * {@link #yawServo}, {@link #intakeSensor} are acceptable, indicating
         * that they are not needed. However, because of this, all methods
         * <em><strong>must</strong></em> check for {@code null} {@link Servo}
         * or {@link DigitalChannel} values.
         *
         * @return Whether the current attributes are valid, which is
         * {@code true} if and only if {@link #servoIncrement} is positive, both
         * powers are positive, and the intake servo is non-{@code null}.
         */
        @Override
        public boolean isValid() {
            return super.isValid()
                   && intakeServo != null
                   && intakePower > 0
                   && ejectPower > 0;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public Claw build() {
            return isValid() ? new SingleServoIntakeClaw(this) : null;
        }

        /**
         * Set the continuous rotation servo used to control the intake (see
         * {@link #INTAKE_SERVO}).
         *
         * @param intakeServo The servo used to take in objects.
         * @return This {@link Builder} to allow for chaining setters.
         */
        public Builder setIntakeServo(CRServo intakeServo) {
            this.intakeServo = intakeServo;
            return this;
        }

        /**
         * Set the touch sensor used to detect whether an object has entered the
         * claw (see {@link #ROLL_SERVO}).
         *
         * @param intakeSensor The touch sensor used to detect whether an object
         *                     has entered the claw.
         * @return This {@link Builder} to allow for chaining setters.
         */
        public Builder setIntakeSensor(DigitalChannel intakeSensor) {
            this.intakeSensor = intakeSensor;
            return this;
        }

        /**
         * Set the power used by the {@link #intakeServo} to take in objects
         * (see {@link #intakePower}).
         *
         * @param intakePower The power used by the {@link #intakeServo} to take
         *                    in objects.
         * @return This {@link Builder} to allow for chaining setters.
         */
        public Builder setIntakePower(double intakePower) {
            this.intakePower = intakePower;
            return this;
        }

        /**
         * Set the power used by {@link #intakeServo} to eject objects (see
         * {@link #ejectPower}).
         *
         * @param ejectPower The power used by {@link #intakeServo} to take in
         *                   objects.
         * @return This {@link Builder} to allow for chaining setters.
         */
        public Builder setEjectPower(double ejectPower) {
            this.ejectPower = ejectPower;
            return this;
        }
    }

    /**
     * The continuous rotation servo that spins the intake.
     */
    private final CRServo INTAKE_SERVO;
    /**
     * How much power the {@link #INTAKE_SERVO} spins with when taking in
     * objects.
     */
    private final double INTAKE_POWER;
    /**
     * How much power the intake spins with when ejecting objects.
     */
    private final double EJECT_POWER;

    /**
     * The touch sensor that detects whether there is an object in the intake.
     */
    private final DigitalChannel INTAKE_SENSOR;

    /**
     * Instantiate a new {@link SingleServoIntakeClaw} object based on the
     * values set in a {@link Builder}. It is presumed that the {@link Builder}
     * has already checked its own validity in {@link Builder#build()}.
     *
     * @param builder The builder that contains the parameters to instantiate a
     *                new {@link SingleServoIntakeClaw} object.
     * @throws IllegalArgumentException If the {@link Builder#intakeServo} is
     *                                  {@code null}.
     */
    protected SingleServoIntakeClaw(Builder builder) throws IllegalArgumentException {
        super(builder);

        if (builder.intakeServo == null) {
            throw new IllegalArgumentException("Intake servo cannot be null.");
        }

        INTAKE_SERVO = builder.intakeServo;
        INTAKE_SENSOR = builder.intakeSensor;

        INTAKE_POWER = builder.intakePower;
        EJECT_POWER = builder.ejectPower;
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