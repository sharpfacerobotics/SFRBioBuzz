package org.firstinspires.ftc.teamcode.hardwaresystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashSet;
import java.util.Set;

/**
 * Servo with two continuous rotation servos to pick up objects.
 * <p>
 * See {@link SingleServoIntakeClaw} for the single-servo version.
 */
@SuppressWarnings("unused")
public class DoubleServoIntakeClaw extends Claw {
    /**
     * Simplifies the instantiation of {@link DoubleServoIntakeClaw} objects.
     */
    public static class Builder extends Claw.Builder {
        /**
         * The continuous rotation servo that spins the left part of the
         * intake.
         */
        protected CRServo leftIntakeServo;
        /**
         * The continuous rotation servo that spins the right part of the
         * intake.
         */
        protected CRServo rightIntakeServo;

        /**
         * The power used by the intake servos to take in objects. Used to set
         * {@link #INTAKE_POWER}.
         */
        protected double intakePower;
        /**
         * The power used by the intake servos to eject objects. Used to set
         * {@link #EJECT_POWER}.
         */
        protected double ejectPower;

        /**
         * The sensor that detects whether an object has been taken in. Used to
         * set {@link #INTAKE_SENSOR}. May be {@code null}.
         */
        protected DigitalChannel intakeSensor;

        /**
         * Instantiate a {@link SingleServoIntakeClaw} with no movement servos,
         * intake CR servo, or touch sensor; intake power set to 0.5; and eject
         * power set to -1.0.
         */
        public Builder() {
            super();
            leftIntakeServo = null;
            rightIntakeServo = null;
            intakeSensor = null;

            intakePower = 0.5;
            ejectPower = -1.0;
        }

        /**
         * Set the {@link Servo} used to control roll (see
         * {@link Claw.Builder#setRollServo(Servo)}).
         *
         * @param rollServo The servo used to control roll.
         * @return This {@link Builder} to allow for chaining setters.
         */
        @Override
        public Builder setRollServo(Servo rollServo) {
            return (Builder) super.setRollServo(rollServo);
        }

        /**
         * Set the {@link Servo} used to control pitch (see
         * {@link Claw.Builder#setPitchServo(Servo)}).
         *
         * @param pitchServo The servo used to control pitch.
         * @return This {@link Builder} to allow for chaining setters.
         */
        @Override
        public Builder setPitchServo(Servo pitchServo) {
            return (Builder) super.setPitchServo(pitchServo);
        }

        /**
         * Set the {@link Servo} used to control yaw (see
         * {@link Claw.Builder#setYawServo(Servo)}).
         *
         * @param yawServo The servo used to control yaw.
         * @return This {@link Builder} to allow for chaining setters.
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
         * powers are positive, and both intake servos are non-{@code null}.
         * <p>
         * {@code null} values for {@link #rollServo}, {@link #pitchServo},
         * {@link #yawServo}, {@link #intakeSensor} are acceptable, indicating
         * that they are not needed. However, because of this, all methods
         * <em><strong>must</strong></em> check for {@code null} {@link Servo}
         * or {@link DigitalChannel} values.
         *
         * @return Whether the current attributes are valid, which is
         * {@code true} if and only if {@link #servoIncrement} is positive, both
         * powers are positive, and both intake servos are non-{@code null}.
         */
        @Override
        public boolean isValid() {
            return super.isValid()
                   && leftIntakeServo != null
                   && rightIntakeServo != null
                   && intakePower > 0
                   && ejectPower > 0;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public DoubleServoIntakeClaw build() {
            return isValid() ? new DoubleServoIntakeClaw(this) : null;
        }

        /**
         * Set the continuous rotation servo used to the left part of the intake
         * (see {@link #LEFT_INTAKE_SERVO}).
         *
         * @param leftIntakeServo The servo used to intake objects.
         * @return This {@link Builder} to allow for chaining setters.
         */
        public Builder setLeftIntakeServo(CRServo leftIntakeServo) {
            this.leftIntakeServo = leftIntakeServo;
            return this;
        }

        /**
         * Set the continuous rotation servo used to the left part of the intake
         * (see {@link #LEFT_INTAKE_SERVO}).
         *
         * @param rightIntakeServo The servo used to intake objects.
         * @return This {@link Builder} to allow for chaining setters.
         */
        public Builder setRightIntakeServo(CRServo rightIntakeServo) {
            this.rightIntakeServo = rightIntakeServo;
            return this;
        }

        /**
         * Set the touch sensor used to detect whether an object has entered the
         * claw (see {@link #intakeSensor}).
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
         * Set the power used by the intake servos to take in objects (see
         * {@link #intakePower}).
         *
         * @param intakePower The power used by the intake servos to take in
         *                    objects.
         * @return This {@link Builder} to allow for chaining setters.
         */
        public Builder setIntakePower(double intakePower) {
            this.intakePower = intakePower;
            return this;
        }

        /**
         * Set the power used by the intake servos to eject objects (see
         * {@link #ejectPower}).
         *
         * @param ejectPower The power used by the intake servos to take in
         *                   objects.
         * @return This {@link Builder} to allow for chaining setters.
         */
        public Builder setEjectPower(double ejectPower) {
            this.ejectPower = ejectPower;
            return this;
        }
    }

    /**
     * The left servo that spins the intake.
     */
    private final CRServo LEFT_INTAKE_SERVO;
    /**
     * The right servo that spins the intake.
     */
    private final CRServo RIGHT_INTAKE_SERVO;

    /**
     * How much power the intake spins with when taking in objects.
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
     * @throws IllegalArgumentException If either intake servo
     *                                  ({@link Builder#leftIntakeServo} or
     *                                  {@link Builder#rightIntakeServo}) is
     *                                  {@code null}.
     */
    protected DoubleServoIntakeClaw(Builder builder) throws IllegalArgumentException {
        super(builder);

        if (!builder.isValid()) {
            throw new IllegalArgumentException(
                "DoubleServoIntakeClaw builder is invalid.");
        }

        LEFT_INTAKE_SERVO = builder.leftIntakeServo;
        RIGHT_INTAKE_SERVO = builder.rightIntakeServo;
        INTAKE_SENSOR = builder.intakeSensor;

        INTAKE_POWER = builder.intakePower;
        EJECT_POWER = builder.ejectPower;
    }

    /**
     * Get the power used to intake an object, i.e., pull it into the claw.
     *
     * @return The power used to intake an object, i.e., pull it into the claw.
     */
    public double getIntakePower() {
        return INTAKE_POWER;
    }

    /**
     * Get the power used to eject an object, i.e., push it out of the claw.
     *
     * @return The power used to intake an object, i.e., push it out of the
     * claw.
     */
    public double getEjectPower() {
        return EJECT_POWER;
    }

    /**
     * Get all the continuous rotation servos ({@link CRServo}s) used by this
     * {@link Claw}, i.e., the left and right intake servos.
     *
     * @return All the continuous rotation servos ({@link CRServo}s) used by
     * this {@link Claw}, i.e., the left and right intake servos.
     */
    public Set<CRServo> getCrServos() {
        return new HashSet<>(Set.of(
            LEFT_INTAKE_SERVO,
            RIGHT_INTAKE_SERVO
        ));
    }

    /**
     * Get the continuous rotation servo ({@link CRServo}) used to run the left
     * intake.
     *
     * @return The continuous rotation servo ({@link CRServo}) used to run the
     * left intake.
     */
    public CRServo getLeftIntakeServo() {
        return LEFT_INTAKE_SERVO;
    }

    /**
     * Get the continuous rotation servo ({@link CRServo}) used to run the right
     * intake.
     *
     * @return The continuous rotation servo ({@link CRServo}) used to run the
     * right intake.
     */
    public CRServo getRightIntakeServo() {
        return RIGHT_INTAKE_SERVO;
    }

    /**
     * Spin both intake servos to pull in an object.
     */
    public void startIntake() {
        if (LEFT_INTAKE_SERVO != null) {
            LEFT_INTAKE_SERVO.setPower(INTAKE_POWER);
        }
        if (RIGHT_INTAKE_SERVO != null) {
            RIGHT_INTAKE_SERVO.setPower(INTAKE_POWER);
        }
    }

    /**
     * Get whether <em>both</em> intake servos are currently running. If either
     * {@link Servo} is {@code null}, trivially return {@code false}.
     *
     * @return {@code true} if the intake servos' powers are <em>both</em>
     * non-zero.
     * <p>
     * {@code false} otherwise, including if either {@link Servo} is
     * {@code null}.
     */
    public boolean isIntakeActive() {
        return LEFT_INTAKE_SERVO != null
               && LEFT_INTAKE_SERVO.getPower() != 0
               && RIGHT_INTAKE_SERVO != null
               && RIGHT_INTAKE_SERVO.getPower() != 0;
    }

    /**
     * Stop both intake servos from either intaking or ejecting.
     */
    public void stopIntake() {
        if (LEFT_INTAKE_SERVO != null) {
            LEFT_INTAKE_SERVO.setPower(0);
        }
        if (RIGHT_INTAKE_SERVO != null) {
            RIGHT_INTAKE_SERVO.setPower(0);
        }
    }

    /**
     * Make the intake spin in reverse and eject the object.
     */
    public void ejectIntake() {
        if (LEFT_INTAKE_SERVO != null) {
            LEFT_INTAKE_SERVO.setPower(EJECT_POWER);
        }
        if (RIGHT_INTAKE_SERVO != null) {
            RIGHT_INTAKE_SERVO.setPower(EJECT_POWER);
        }
    }

    /**
     * Get whether the sensor on the claw is pressed or not. If there is no
     * sensor, i.e., the sensor is {@code null}, then trivially return
     * {@code false}.
     *
     * @return {@code true} when the sensor is pressed.
     * <p>
     * {@code false} otherwise, including if the {@link #INTAKE_SENSOR} is
     * {@code null}.
     */
    public boolean isSensorPressed() {
        //  Returns true when the sensor is not pressed.
        return INTAKE_SENSOR != null && !INTAKE_SENSOR.getState();
    }
}