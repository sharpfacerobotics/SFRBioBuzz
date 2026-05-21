package org.firstinspires.ftc.teamcode.hardwaresystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.HashSet;

/**
 * A set of four mecanum wheels, each controlled separately.
 */
public class MecanumWheels extends Wheels {
    public static class Builder {
        private static MotorSet motorSet;
        private static WheelDistances wheelDistances;
        private static double ticksPerInch;

        public static void setMotorSet(MotorSet newMotorSet) {
            motorSet = newMotorSet;
        }

        public static void setWheelDistances(WheelDistances newWheelDistances) {
            wheelDistances = newWheelDistances;
        }

        public static void setTicksPerInch(double newTicksPerInch) {
            ticksPerInch = newTicksPerInch;
        }

        public static MecanumWheels build() {
            return new MecanumWheels(motorSet, wheelDistances, ticksPerInch);
        }
    }

    /**
     * Passed into the
     * {@link MecanumWheels#MecanumWheels(MotorSet, WheelDistances, double)}
     * constructor. Contains all four motors.
     */
    public static class MotorSet {
        public final HashSet<DcMotor> MOTORS;
        /**
         * The {@link DcMotor}s powering the front left wheel.
         */
        private final DcMotor FRONT_LEFT_MOTOR;
        /**
         * The {@link DcMotor}s powering the front right wheel.
         */
        private final DcMotor FRONT_RIGHT_MOTOR;
        /**
         * The {@link DcMotor}s powering the back left wheel.
         */
        private final DcMotor BACK_LEFT_MOTOR;
        /**
         * The {@link DcMotor}s powering the back right wheel.
         */
        private final DcMotor BACK_RIGHT_MOTOR;

        /**
         * Instantiate a set of four {@link MecanumWheels}.
         *
         * @param frontLeftMotor  The motor that controls the front left wheel.
         * @param frontRightMotor The motor that controls the front right
         *                        wheel.
         * @param backLeftMotor   The motor that controls the back left wheel.
         * @param backRightMotor  The motor that controls the back right wheel.
         */
        public MotorSet(
            DcMotor frontLeftMotor,
            DcMotor frontRightMotor,
            DcMotor backLeftMotor,
            DcMotor backRightMotor
        ) {
            MOTORS = new HashSet<>();
            MOTORS.add(frontLeftMotor);
            MOTORS.add(frontRightMotor);
            MOTORS.add(backLeftMotor);
            MOTORS.add(backRightMotor);

            FRONT_LEFT_MOTOR = frontLeftMotor;
            FRONT_RIGHT_MOTOR = frontRightMotor;
            BACK_LEFT_MOTOR = backLeftMotor;
            BACK_RIGHT_MOTOR = backRightMotor;
        }

        /**
         * Instantiate an empty motor set. Mostly used for dummy purposes.
         */
        public MotorSet() {
            MOTORS = new HashSet<>();

            FRONT_LEFT_MOTOR = null;
            FRONT_RIGHT_MOTOR = null;
            BACK_LEFT_MOTOR = null;
            BACK_RIGHT_MOTOR = null;
        }
    }

    /**
     * The motor powering the front left wheel.
     */
    private final DcMotor FRONT_LEFT_MOTOR;
    /**
     * The motor powering the front right wheel.
     */
    private final DcMotor FRONT_RIGHT_MOTOR;
    /**
     * The motor powering the back left wheel.
     */
    private final DcMotor BACK_LEFT_MOTOR;
    /**
     * The motor powering the back right wheel.
     */
    private final DcMotor BACK_RIGHT_MOTOR;

    public MecanumWheels(
        MotorSet motorSet,
        WheelDistances wheelDistances,
        double ticksPerInch
    ) {
        super(motorSet.MOTORS, wheelDistances, ticksPerInch);

        this.FRONT_LEFT_MOTOR = motorSet.FRONT_LEFT_MOTOR;
        this.FRONT_RIGHT_MOTOR = motorSet.FRONT_RIGHT_MOTOR;
        this.BACK_LEFT_MOTOR = motorSet.BACK_LEFT_MOTOR;
        this.BACK_RIGHT_MOTOR = motorSet.BACK_RIGHT_MOTOR;

        // Reset position to 0
        for (DcMotor motor : MOTORS) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        /*
         * Set the directions of the motors.
         * The right and left motors run in opposite directions of each other.
         * Positive is forward for all motors.
         *
         * In some cases, it may be necessary to reverse the signs.
         */
        FRONT_LEFT_MOTOR.setDirection(DcMotorSimple.Direction.REVERSE);
        FRONT_RIGHT_MOTOR.setDirection(DcMotorSimple.Direction.FORWARD);
        BACK_LEFT_MOTOR.setDirection(DcMotorSimple.Direction.REVERSE);
        BACK_RIGHT_MOTOR.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public DcMotor getFrontLeftMotor() {
        return FRONT_LEFT_MOTOR;
    }

    public DcMotor getFrontRightMotor() {
        return FRONT_RIGHT_MOTOR;
    }

    public DcMotor getBackLeftMotor() {
        return BACK_LEFT_MOTOR;
    }

    public DcMotor getBackRightMotor() {
        return BACK_RIGHT_MOTOR;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void drive(double xPower, double yPower, double thetaPower) {
        for (DcMotor motor : MOTORS) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        /*
         * The code below should be correct. If for some reason the robot is
         * moving in the wrong direction, reverse
         * the direction of the motors in {@link MecanumWheels#MecanumWheels
         * (MotorSet, WheelDistances, ticksPerInch)}.
         *
         * As a last resort, try
         * ```
         * double frontLeftPower = -thetaPower + xPower + yPower;
         * double frontRightPower = thetaPower + xPower + yPower;
         * double backLeftPower = -thetaPower - xPower + yPower;
         * double backRightPower = thetaPower - xPower + yPower;
         * ```
         * It worked in a previous season, but it probably is not a good idea.
         */
        double frontLeftPower = yPower + xPower + thetaPower;
        double frontRightPower = yPower - xPower - thetaPower;
        double backLeftPower = yPower - xPower + thetaPower;
        double backRightPower = yPower + xPower - thetaPower;

        // Scale the motor powers to be within +/- 1.0.
        // Use the absolute maximum magnitude rather than the algebraic maximum
        // to ensure all motors are scaled properly.
        // For example, a power set of [-0.8, 0.2, 0.5, 0.4] should be scaled
        // by 0.8, not 0.5.
        double maxMagnitude = Math.max(
            Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
            Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );
        if (maxMagnitude > 1.0) {
            frontLeftPower /= maxMagnitude;
            frontRightPower /= maxMagnitude;
            backLeftPower /= maxMagnitude;
            backRightPower /= maxMagnitude;
        }

        FRONT_LEFT_MOTOR.setPower(frontLeftPower);
        FRONT_RIGHT_MOTOR.setPower(frontRightPower);
        BACK_LEFT_MOTOR.setPower(backLeftPower);
        BACK_RIGHT_MOTOR.setPower(backRightPower);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void driveDistance(double forwardDistance) {
        driveDistance(0, forwardDistance);
    }

    /**
     * {@inheritDoc}
     *
     * <strong><i>THIS METHOD IS STILL EXPERIMENTAL!</i></strong>
     */
    @Override
    public void driveDistance(double sidewaysDistance, double forwardDistance) {
        /*
         * Apply Pythagorean's Theorem to find the Euclidean distance. Use
         * Math.hypot() to avoid overflow.
         */
        double totalDistance = Math.hypot(forwardDistance, sidewaysDistance);

        // If both distances are zero there is nothing to do.
        // Guard against division by zero in the scaling logic below and halt
        // the drive.
        if (totalDistance == 0) {
            // Set all motor powers to zero to stop the robot cleanly.
            drive(0);
            return;
        }

        // Scale the motor power based on trigonometry. Multiply by
        // `MOTOR_POWER` after normalizing by the total distance so that
        // larger requested distances do not inadvertently increase motor power.
        double xPower = (sidewaysDistance / totalDistance) * MOTOR_POWER;
        double yPower = (forwardDistance / totalDistance) * MOTOR_POWER;
        drive(xPower, yPower, 0);

        int frontLeftTickPosition =
            FRONT_LEFT_MOTOR.getCurrentPosition() + (int) (
                (sidewaysDistance - forwardDistance) * TICKS_PER_INCH
            );
        int frontRightTickPosition =
            FRONT_RIGHT_MOTOR.getCurrentPosition() - (int) (
                (-sidewaysDistance + forwardDistance) * TICKS_PER_INCH
            );
        int backLeftTickPosition =
            BACK_LEFT_MOTOR.getCurrentPosition() + (int) (
                (-sidewaysDistance - forwardDistance) * TICKS_PER_INCH
            );
        int backRightTickPosition =
            BACK_RIGHT_MOTOR.getCurrentPosition() - (int) (
                (sidewaysDistance + forwardDistance) * TICKS_PER_INCH
            );

        FRONT_LEFT_MOTOR.setTargetPosition(frontLeftTickPosition);
        FRONT_RIGHT_MOTOR.setTargetPosition(frontRightTickPosition);
        BACK_LEFT_MOTOR.setTargetPosition(backLeftTickPosition);
        BACK_RIGHT_MOTOR.setTargetPosition(backRightTickPosition);

        for (DcMotor motor : MOTORS) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void turn(double degrees) {
        // The diameter of the circle that the wheels make when rotating 360
        // degrees.
        double diameter = Math.sqrt(
            Math.pow(LATERAL_DISTANCE, 2)
            + Math.pow(LONGITUDINAL_DISTANCE, 2)
        );
        double circumference = diameter * Math.PI;

        // How far the wheels have to move.
        double arcLength = (degrees / 360.0) * circumference;
        int ticks = (int) Math.round(arcLength * TICKS_PER_INCH) * 4 / 3;

        // Left wheels
        FRONT_LEFT_MOTOR.setTargetPosition(
            FRONT_LEFT_MOTOR.getCurrentPosition() - ticks
        );
        FRONT_LEFT_MOTOR.setPower(-MOTOR_POWER);
        BACK_LEFT_MOTOR.setTargetPosition(
            BACK_LEFT_MOTOR.getCurrentPosition() - ticks
        );
        BACK_LEFT_MOTOR.setPower(-MOTOR_POWER);

        // Right wheels
        FRONT_RIGHT_MOTOR.setTargetPosition(
            FRONT_RIGHT_MOTOR.getCurrentPosition() + ticks
        );
        FRONT_RIGHT_MOTOR.setPower(MOTOR_POWER);
        BACK_RIGHT_MOTOR.setTargetPosition(
            BACK_RIGHT_MOTOR.getCurrentPosition() + ticks
        );
        BACK_RIGHT_MOTOR.setPower(MOTOR_POWER);

        for (DcMotor motor : MOTORS) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
}