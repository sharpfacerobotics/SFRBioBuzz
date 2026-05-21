package org.firstinspires.ftc.teamcode.hardwaresystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.HashSet;

/**
 * A set of four mecanum wheels, each controlled separately.
 */
public class MecanumWheels extends Wheels {
    /**
     *
     */
    public static class Builder {
        private MotorSet motorSet;
        private WheelDistances wheelDistances;
        private double ticksPerInch;

        public Builder setMotorSet(MotorSet newMotorSet) {
            motorSet = newMotorSet;
            return this;
        }

        public Builder setFrontLeftMotor(DcMotor frontLeftMotor) {
            if (motorSet == null) {
                motorSet = new MotorSet();
            }

            motorSet.frontLeftMotor = frontLeftMotor;
            return this;
        }

        public Builder setFrontRightMotor(DcMotor frontRightMotor) {
            if (motorSet == null) {
                motorSet = new MotorSet();
            }

            motorSet.frontRightMotor = frontRightMotor;
            return this;
        }

        public Builder setBackLeftMotor(DcMotor backLeftMotor) {
            if (motorSet == null) {
                motorSet = new MotorSet();
            }

            motorSet.backLeftMotor = backLeftMotor;
            return this;
        }

        public Builder setBackRightMotor(DcMotor backRightMotor) {
            if (motorSet == null) {
                motorSet = new MotorSet();
            }

            motorSet.backRightMotor = backRightMotor;
            return this;
        }

        public Builder setWheelDistances(WheelDistances newWheelDistances) {
            wheelDistances = newWheelDistances;
            return this;
        }

        public Builder setLongitudinalDistance(double longitudinalDistance) {
            if (wheelDistances == null) {
                wheelDistances = new WheelDistances(-1.0, longitudinalDistance);

            } else {
                wheelDistances.longitudinalDistance = longitudinalDistance;
            }

            return this;
        }

        public Builder setLateralDistance(double lateralDistance) {
            if (wheelDistances == null) {
                wheelDistances = new WheelDistances(lateralDistance, -1.0);

            } else {
                wheelDistances.lateralDistance = lateralDistance;
            }

            return this;
        }

        public Builder setTicksPerInch(double newTicksPerInch) {
            ticksPerInch = newTicksPerInch;
            return this;
        }

        /**
         * Using the given {@link MotorSet}, {@link WheelDistances}, and
         * {@link #ticksPerInch}, construct a new instance of
         * {@link MecanumWheels}.
         *
         * @return A new instance of {@link MecanumWheels} if the values are
         * valid.
         * <p>
         * Else, {@code null}.
         */
        public MecanumWheels build() {
            if (
                motorSet == null
                || motorSet.containsNull()
                || wheelDistances == null
                || wheelDistances.isInvalid()
                || ticksPerInch <= 0
            ) {
                return null;
            }

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
        private DcMotor frontLeftMotor;
        /**
         * The {@link DcMotor}s powering the front right wheel.
         */
        private DcMotor frontRightMotor;
        /**
         * The {@link DcMotor}s powering the back left wheel.
         */
        private DcMotor backLeftMotor;
        /**
         * The {@link DcMotor}s powering the back right wheel.
         */
        private DcMotor backRightMotor;

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

            this.frontLeftMotor = frontLeftMotor;
            this.frontRightMotor = frontRightMotor;
            this.backLeftMotor = backLeftMotor;
            this.backRightMotor = backRightMotor;
        }

        /**
         * Instantiate an empty motor set. Mostly used for dummy purposes.
         */
        public MotorSet() {
            MOTORS = new HashSet<>();

            frontLeftMotor = null;
            frontRightMotor = null;
            backLeftMotor = null;
            backRightMotor = null;
        }

        public boolean containsNull() {
            return frontLeftMotor == null
                   || frontRightMotor == null
                   || backLeftMotor == null
                   || backRightMotor == null;
        }
    }

    /**
     * The motor powering the front right wheel.
     */
    private DcMotor frontRightMotor;
    /**
     * The motor powering the back left wheel.
     */
    private DcMotor backLeftMotor;
    /**
     * The motor powering the back right wheel.
     */
    private DcMotor backRightMotor;
    /**
     * The motor powering the front left wheel.
     */
    private DcMotor frontLeftMotor;

    public MecanumWheels(
        MotorSet motorSet,
        WheelDistances wheelDistances,
        double ticksPerInch
    ) {
        super(motorSet.MOTORS, wheelDistances, ticksPerInch);

        frontLeftMotor = motorSet.frontLeftMotor;
        frontRightMotor = motorSet.frontRightMotor;
        backLeftMotor = motorSet.backLeftMotor;
        backRightMotor = motorSet.backRightMotor;

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
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public DcMotor getFrontLeftMotor() {
        return frontLeftMotor;
    }

    public DcMotor getFrontRightMotor() {
        return frontRightMotor;
    }

    public DcMotor getBackLeftMotor() {
        return backLeftMotor;
    }

    public DcMotor getBackRightMotor() {
        return backRightMotor;
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
            Math.max(
                Math.abs(frontLeftPower),
                Math.abs(frontRightPower)
            ),
            Math.max(
                Math.abs(backLeftPower),
                Math.abs(backRightPower)
            )
        );
        if (maxMagnitude > 1.0) {
            frontLeftPower /= maxMagnitude;
            frontRightPower /= maxMagnitude;
            backLeftPower /= maxMagnitude;
            backRightPower /= maxMagnitude;
        }

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
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
            frontLeftMotor.getCurrentPosition() + (int) (
                (sidewaysDistance - forwardDistance) * TICKS_PER_INCH
            );
        int frontRightTickPosition =
            frontRightMotor.getCurrentPosition() - (int) (
                (-sidewaysDistance + forwardDistance) * TICKS_PER_INCH
            );
        int backLeftTickPosition =
            backLeftMotor.getCurrentPosition() + (int) (
                (-sidewaysDistance - forwardDistance) * TICKS_PER_INCH
            );
        int backRightTickPosition =
            backRightMotor.getCurrentPosition() - (int) (
                (sidewaysDistance + forwardDistance) * TICKS_PER_INCH
            );

        frontLeftMotor.setTargetPosition(frontLeftTickPosition);
        frontRightMotor.setTargetPosition(frontRightTickPosition);
        backLeftMotor.setTargetPosition(backLeftTickPosition);
        backRightMotor.setTargetPosition(backRightTickPosition);

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
        frontLeftMotor.setTargetPosition(
            frontLeftMotor.getCurrentPosition() - ticks
        );
        frontLeftMotor.setPower(-MOTOR_POWER);
        backLeftMotor.setTargetPosition(
            backLeftMotor.getCurrentPosition() - ticks
        );
        backLeftMotor.setPower(-MOTOR_POWER);

        // Right wheels
        frontRightMotor.setTargetPosition(
            frontRightMotor.getCurrentPosition() + ticks
        );
        frontRightMotor.setPower(MOTOR_POWER);
        backRightMotor.setTargetPosition(
            backRightMotor.getCurrentPosition() + ticks
        );
        backRightMotor.setPower(MOTOR_POWER);

        for (DcMotor motor : MOTORS) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
}