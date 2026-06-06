package org.firstinspires.ftc.teamcode.hardwaresystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Set;

/**
 * Represent any possible system of wheels on the robot.
 */
public abstract class Wheels {
    /**
     * Contains the distances between wheels. Necessary for calculating
     * rotation.
     */
    protected static class WheelDistances extends BuilderParameters {
        /**
         * The distance between the left and right wheels, measured in inches
         * from their centers.
         */
        protected double lateralDistance;
        /**
         * The distance between the front and back wheels, measured in inches
         * from their centers.
         */
        protected double longitudinalDistance;

        /**
         * Define the wheel's distances.
         *
         * @param lateralDistance      The distance between the left and right
         *                             wheels, measured in inches from their
         *                             centers.
         * @param longitudinalDistance The distance between the front and back
         *                             wheels, measured in inches from their
         *                             centers.
         */
        public WheelDistances(
            double lateralDistance,
            double longitudinalDistance
        ) {
            this.longitudinalDistance = longitudinalDistance;
            this.lateralDistance = lateralDistance;
        }

        /**
         * Check if the distances({@link #longitudinalDistance} and
         * {@link #lateralDistance}) are physically possible (i.e., they must be
         * positive values).
         *
         * @return {@code true} if both {@link #longitudinalDistance} and
         * {@link #lateralDistance} are positive.
         * <p>
         * {@code false} otherwise.
         */
        public boolean isValid() {
            return longitudinalDistance > 0 && lateralDistance > 0;
        }
    }

    /**
     * Builder class to simplify the process of creating {@link Wheels}
     * subclasses. Subclasses that extend {@link Wheels} can also have an inner
     * class that extends {@link Builder}.
     */
    public static abstract class Builder extends HardwareSystemBuilder {
        /**
         * The distances between the wheels of the {@link Wheels} object to be
         * created.
         */
        protected WheelDistances wheelDistances;
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
            wheelDistances = new WheelDistances(-1.0, -1.0);
            ticksPerInch = -1.0;
            maxMotorPower = 1.0;
        }

        /**
         * Set the {@link WheelDistances#lateralDistance} property of
         * {@link #wheelDistances}.
         *
         * @param lateralDistance The distance between the left and right wheels
         *                        in inches.
         * @return Any builder class that extends {@link Builder} should
         * implement the abstract setter methods with a <em>subclass</em> of
         * {@link Wheels.Builder} as the return type. This allows for chaining
         * of {@link Builder} methods without the need to cast the type.
         * <p>
         * See {@link MecanumWheels.Builder} for an example.
         */
        public abstract Builder setLateralDistance(double lateralDistance);

        /**
         * Set the {@link WheelDistances#lateralDistance} property of
         * {@link #wheelDistances}.
         *
         * @param longitudinalDistance The distance between the left and right
         *                             wheels in inches.
         * @return Any builder class that extends {@link Builder} should
         * implement the abstract setter methods with a <em>subclass</em> of
         * {@link Wheels.Builder} as the return type. This allows for chaining
         * of {@link Builder} methods without the need to cast the type.
         * <p>
         * See {@link MecanumWheels.Builder} for an example.
         */
        public abstract Builder setLongitudinalDistance(double longitudinalDistance);

        /**
         * Set the {@link WheelDistances#lateralDistance} property of
         * {@link #wheelDistances}.
         *
         * @param ticksPerInch The distance between the left and right wheels in
         *                     inches.
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

        /**
         * Using the given {@link WheelDistances}, and {@link #ticksPerInch},
         * construct a new instance of {@link Wheels}.
         *
         * @return If the state of the {@link Builder} is valid, return a
         * <em>subclass</em> of {@link Wheels}. Any class that extends
         * {@link Builder} should replace the return type with a more specific
         * one (i.e., the corresponding subclass of {@link Wheels}).
         * <p>
         * See {@link MecanumWheels.Builder#build()} for an example.
         */
        public abstract Wheels build();
    }

    /**
     * A {@link Set} of all the motors included by the wheel system.
     */
    protected final Set<DcMotor> motors;
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

    /**
     * Instantiate a {@link Wheels} object with only motors, wheel distances,
     * and ticks per inch. Assume the maximum power to be 1.0.
     *
     * @param motors         All the motors used by the robot.
     * @param wheelDistances The distances between the wheels, which is used for
     *                       determining turning angles.
     * @param ticksPerInch   The number of ticks needed to move the robot by one
     *                       inch.
     */
    protected Wheels(
        Set<DcMotor> motors,
        WheelDistances wheelDistances,
        double ticksPerInch
    ) {
        this(motors, wheelDistances, ticksPerInch, 1.0);
    }

    /**
     * Instantiate a {@link Wheels} object with motors, wheel distances, ticks
     * per inch, and motor powers all set up.
     *
     * @param motors         All the motors used by the robot.
     * @param wheelDistances The distances between the wheels, which is used for
     *                       determining turning angles.
     * @param ticksPerInch   The number of ticks needed to move the robot by one
     *                       inch.
     * @param maxMotorPower  The maximum power that the wheel motors drive with.
     *                       Should be between 0.0 (exclusive) and 1.0
     *                       (inclusive).
     */
    protected Wheels(
        Set<DcMotor> motors,
        WheelDistances wheelDistances,
        double ticksPerInch,
        double maxMotorPower
    ) {
        this.motors = motors;
        // Allow wheels to roll freely.
        for (DcMotor motor : this.motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        LATERAL_DISTANCE = wheelDistances.lateralDistance;
        LONGITUDINAL_DISTANCE = wheelDistances.longitudinalDistance;

        TICKS_PER_INCH = ticksPerInch;

        MAX_MOTOR_POWER = maxMotorPower;
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
        return motors;
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