package org.firstinspires.ftc.teamcode.hardwaresystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Set;

/**
 * Represent any possible system of wheels on the robot.
 */
@SuppressWarnings("unused")
public abstract class Wheels {
    /**
     * Builder class to simplify the process of creating {@link Wheels}
     * subclasses. Subclasses that extend {@link Wheels} can also have an inner
     * class that extends {@link Builder}.
     */
    public static abstract class Builder extends HardwareSystemBuilder {
        /**
         * The motors used by
         */

        /**
         * The distance between the left and right wheels, measured in inches
         * from their centers.
         */
        protected double lateralWheelDistance;
        /**
         * The distance between the front and back wheels, measured in inches
         * from their centers.
         */
        protected double longitudinalWheelDistance;

        /**
         * The number of ticks it takes for the motors of the new {@link Wheels}
         * object to travel one (1) inch.
         */
        protected double ticksPerInch;
        /**
         * The maximum power sent to the {@link DcMotor}s. Should be from 0.0
         * (exclusive) to 1.0 (inclusive).
         */
        protected double maxMotorPower;

        /**
         * Instantiate the {@link Builder} with default values. The default
         * values are invalid and will not pass the validity check for
         * {@link Builder#build()}.
         */
        public Builder() {
            super();

            lateralWheelDistance = -1.0;
            longitudinalWheelDistance = -1.0;
            ticksPerInch = -1.0;
            maxMotorPower = 1.0;
        }

        /**
         * Set the {@link #lateralWheelDistance}
         *
         * @param lateralWheelDistance The distance between the left and right
         *                             wheels in inches.
         * @return Any builder class that extends {@link Builder} should
         * implement the abstract setter methods with a <em>subclass</em> of
         * {@link Wheels.Builder} as the return type. This allows for chaining
         * of {@link Builder} methods without the need to cast the type.
         * <p>
         * See {@link MecanumWheels.Builder} for an example.
         */
        public abstract Builder setLateralWheelDistance(double lateralWheelDistance);

        /**
         * Set the {@link #longitudinalWheelDistance} property.
         *
         * @param longitudinalWheelDistance The distance between the left and
         *                                  right wheels in inches.
         * @return Any builder class that extends {@link Builder} should
         * implement the abstract setter methods with a <em>subclass</em> of
         * {@link Wheels.Builder} as the return type. This allows for chaining
         * of {@link Builder} methods without the need to cast the type.
         * <p>
         * See {@link MecanumWheels.Builder} for an example.
         */
        public abstract Builder setLongitudinalWheelDistance(double longitudinalWheelDistance);

        /**
         * Set the number of motor ticks per inches of distance
         * ({@link #ticksPerInch}).
         *
         * @param ticksPerInch The number of motor ticks per inches of
         *                     distance.
         * @return Any builder class that extends {@link Builder} should
         * implement the abstract setter methods with a <em>subclass</em> of
         * {@link Wheels.Builder} as the return type. This allows for chaining
         * of {@link Builder} methods without the need to cast the type.
         * <p>
         * See {@link MecanumWheels.Builder} for an example.
         */
        public Builder setTicksPerInch(double ticksPerInch) {
            this.ticksPerInch = ticksPerInch;
            return this;
        }

        /**
         * Set the maximum amount of power that can be sent to the wheel
         * motors.
         *
         * @param maxMotorPower The maximum amount of power that can be sent to
         *                      the wheel motors. Should be between 0.0
         *                      (exclusive) to 1.0 (inclusive).
         * @return This {@link Builder} so that setters can be chained.
         */
        public Builder setMaxMotorPower(double maxMotorPower) {
            this.maxMotorPower = maxMotorPower;
            return this;
        }

        @Override
        public boolean isValid() {
            return lateralWheelDistance > 0.0
                   && longitudinalWheelDistance > 0.0
                   && ticksPerInch > 0.0
                   && 0.0 < maxMotorPower && maxMotorPower < 1.0;
        }

        /**
         * Construct a new instance of {@link Wheels}, using the given wheel
         * distances and {@link #ticksPerInch}.
         *
         * @return If the state of the {@link Builder} is valid, return a
         * <em>subclass</em> of {@link Wheels}. Any class that extends
         * {@link Builder} should replace the return type with a more specific
         * one (i.e., the corresponding subclass of {@link Wheels}).
         * <p>
         * See {@link MecanumWheels.Builder#build()} for an example.
         */
        @Override
        public abstract Wheels build();
    }

    /**
     * A {@link Set} of all the motors included by this wheel system.
     */
    protected final Set<DcMotor> MOTORS;
    /**
     * A multiplier for how much power the wheels run with. The value should be
     * between 0.0 (exclusive) and 1.0 (inclusive).
     */
    protected final double MAX_MOTOR_POWER;

    /**
     * The distance between the left and right wheels, measured in inches from
     * their centers.
     */
    protected final double LATERAL_DISTANCE;
    /**
     * The distance between the front and back wheels, measured in inches from
     * their centers.
     */
    protected final double LONGITUDINAL_DISTANCE;
    /**
     * The number of ticks needed to move the robot by 1 inch.
     */
    protected final double TICKS_PER_INCH;

    protected Wheels(Builder builder) {
        MOTORS = builder.build().getMotors();
        for (DcMotor motor : MOTORS) {
            // Allow wheels to roll freely.
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            // Reset position to 0.
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        LATERAL_DISTANCE = builder.lateralWheelDistance;
        LONGITUDINAL_DISTANCE = builder.longitudinalWheelDistance;

        TICKS_PER_INCH = builder.ticksPerInch;

        MAX_MOTOR_POWER = builder.maxMotorPower;
    }

    /**
     * Get the power that the motors drive with.
     *
     * @return The power that the motors drive with.
     */
    public double getMotorPower() {
        return MAX_MOTOR_POWER;
    }

    /**
     * Get all the {@link DcMotor}s that are used by this wheel system.
     *
     * @return A {@link Set} that contains every {@link DcMotor} included by
     * this wheel system.
     */
    public Set<DcMotor> getMotors() {
        return MOTORS;
    }

    /**
     * Start to drive forwards or backwards.
     *
     * @param forwardPower The power to set the motors to as a proportion of
     *                     {@link #MAX_MOTOR_POWER}.
     */
    public void drive(double forwardPower) {
        /*
         * Forward/backwards movement does not involve sideways x‑motion,
         * so supply 0 for the xPower argument.
         * The previous implementation erroneously passed the forward power
         * as the xPower,
         * causing the robot to strafe instead of drive forward.
         */
        drive(0, forwardPower, 0);
    }

    /**
     * Set the wheels to drive forward with a certain power.
     *
     * @param forwardPower Forward power as a proportion of
     *                     {@link #MAX_MOTOR_POWER}. Positive is forward,
     *                     negative is backward.
     * @param turn         Rotation power. Positive is clockwise, negative is
     *                     counterclockwise.
     */
    public void drive(double forwardPower, double turn) {
        drive(0, forwardPower, turn);
    }

    /**
     * Set the wheels to drive with a given power in for x, y,and turn.
     *
     * @param xPower     Sideways power as a proportion of
     *                   {@link #MAX_MOTOR_POWER}. Positive is rightward,
     *                   negative is leftward.
     * @param yPower     Forward power as a proportion of
     *                   {@link #MAX_MOTOR_POWER}. Positive is forward, negative
     *                   is backward.
     * @param thetaPower Rotation power as a proportion of
     *                   {@link #MAX_MOTOR_POWER}. Positive is clockwise,
     *                   negative is counterclockwise.
     */
    public abstract void drive(double xPower, double yPower, double thetaPower);

    /**
     * Drive the robot a certain distance forward with power
     * {@link #MAX_MOTOR_POWER}.
     *
     * @param forwardDistance The distance that the robot travels in inches.
     *                        Positive is forward, negative is backward.
     */
    public void driveDistance(double forwardDistance) {
        driveDistance(0, forwardDistance);
    }

    /**
     * Drive the robot a certain distance in two dimensions with power
     * {@link #MAX_MOTOR_POWER}.
     *
     * @param sidewaysDistance The distance that the robot travels sideways in
     *                         inches. Positive is rightward, negative is
     *                         leftward.
     * @param forwardDistance  The distance that the robot travels forward in
     *                         inches. Positive is forward, negative is
     *                         backward.
     */
    public abstract void driveDistance(
        double sidewaysDistance,
        double forwardDistance
    );

    /**
     * Rotate the robot a certain number of degrees.
     *
     * @param degrees How many degrees the robot turns. Positive is clockwise,
     *                negative is counterclockwise.
     */
    public abstract void turn(double degrees);
}