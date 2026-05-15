package org.firstinspires.ftc.teamcode.hardwaresystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.HashSet;

/**
 * Represent any possible system of wheels on the robot.
 */
public abstract class Wheels {
    /**
     * A modifier for much power the wheels run with (0.0 - 1.0)
     */
    protected final static double MOTOR_POWER = 1.0;
    /**
     * A HashSet of all the motors included by the wheel system.
     */
    protected final HashSet<DcMotor> MOTORS;
    /**
     * The distance between the left and right wheels, measured in inches from their centers.
     */
    protected final double LATERAL_DISTANCE;
    /**
     * The distance between the front and back wheels, measured in inches from their centers.
     */
    protected final double LONGITUDINAL_DISTANCE;
    /**
     * The number of ticks needed to move the robot by 1 inch.
     */
    protected final double TICKS_PER_INCH;

    /**
     * Contains the distances between wheels. Necessary for calculating rotation.
     */
    public static class WheelDistances {
        /*
         * The distance between the left and right wheels,
         * measured in inches from their centers.
         */
        protected final double LATERAL_DISTANCE;
        /*
         * The distance between the front and back wheels,
         * measured in inches from their centers.
         */
        protected final double LONGITUDINAL_DISTANCE;

        /**
         * Define the wheel's distances.
         *
         * @param lateralDistance      The distance between the left and right wheels, measured in inches from their
         *                             centers.
         * @param longitudinalDistance The distance between the front and back wheels, measured in inches from their
         *                             centers.
         */
        public WheelDistances(double lateralDistance,
                              double longitudinalDistance) {
            LONGITUDINAL_DISTANCE = longitudinalDistance;
            LATERAL_DISTANCE = lateralDistance;
        }
    }

    /**
     * Instantiate a {@link Wheels} object.
     *
     * @param motors       All the motors used by the robot.
     * @param ticksPerInch The number of ticks needed to move the robot by one inch.
     */
    public Wheels(HashSet<DcMotor> motors, WheelDistances wheelDistances,
                  double ticksPerInch) {
        MOTORS = motors;
        // Allow wheels to roll freely.
        for (DcMotor motor : MOTORS) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        this.LATERAL_DISTANCE = wheelDistances.LATERAL_DISTANCE;
        this.LONGITUDINAL_DISTANCE = wheelDistances.LONGITUDINAL_DISTANCE;

        TICKS_PER_INCH = ticksPerInch;
    }

    public double getMotorPower() {
        return MOTOR_POWER;
    }

    /**
     * Get all the {@link DcMotor}s that are used by this wheel system.
     *
     * @return A {@link HashSet} that contains every {@link DcMotor} included by this wheel system.
     */
    public HashSet<DcMotor> getMotors() {
        return MOTORS;
    }

    /**
     * Drive forwards and backwards.
     *
     * @param forwardPower What power to set the motors to.
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
     * Drive the wheels.
     *
     * @param forwardPower Forward power. Positive is forward, negative is backward.
     * @param turn         Rotation power. Positive is clockwise, negative is counterclockwise.
     */
    public void drive(double forwardPower, double turn) {
        drive(0, forwardPower, turn);
    }

    /**
     * Drive the wheels.
     *
     * @param xPower Sideways power. Positive is rightward, negative is leftward.
     * @param yPower Forward power. Positive is forward, negative is backward.
     * @param turn   Rotation power. Positive is clockwise, negative is counterclockwise.
     */
    public abstract void drive(double xPower, double yPower, double turn);

    /**
     * Drive the robot a certain distance forward.
     *
     * @param forwardDistance The distance that the robot travels in inches. Positive is forward, negative is backward.
     */
    public void driveDistance(double forwardDistance) {
        driveDistance(0, forwardDistance);
    }

    /**
     * Drive the robot a certain distance in two dimensions.
     *
     * @param sidewaysDistance The distance that the robot travels sideways in inches. Positive is rightward, negative
     *                         is leftward.
     * @param forwardDistance  The distance that the robot travels forward in inches. Positive is forward, negative is
     *                         backward.
     */
    public abstract void driveDistance(double sidewaysDistance,
                                       double forwardDistance);

    /**
     * Rotate the robot a certain number of degrees.
     *
     * @param degrees How many degrees the robot turns. Positive is clockwise, negative is counterclockwise.
     */
    public abstract void turn(double degrees);
}