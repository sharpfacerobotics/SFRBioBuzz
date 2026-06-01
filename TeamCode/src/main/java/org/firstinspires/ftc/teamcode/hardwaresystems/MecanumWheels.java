package org.firstinspires.ftc.teamcode.hardwaresystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * A system of four mecanum wheels, each controlled separately by their own
 * motor.
 * <p>
 * Mecanum wheels are capable of moving in any direction, whether horizontally,
 * vertically, or diagonally.
 */
public class MecanumWheels extends Wheels {
    /**
     * Passed into the
     * {@link MecanumWheels#MecanumWheels(MecanumWheelMotors, WheelDistances,
     * double)} constructor. Contains all four motors.
     */
    protected static class MecanumWheelMotors implements BuilderParameters {
        /**
         * The {@link DcMotor}s powering the front left wheel.
         */
        protected DcMotor frontLeftMotor;
        /**
         * The {@link DcMotor}s powering the front right wheel.
         */
        protected DcMotor frontRightMotor;
        /**
         * The {@link DcMotor}s powering the back left wheel.
         */
        protected DcMotor backLeftMotor;
        /**
         * The {@link DcMotor}s powering the back right wheel.
         */
        protected DcMotor backRightMotor;

        /**
         * Instantiate a set of four {@link MecanumWheels}.
         *
         * @param frontLeftMotor  The motor that controls the front left wheel.
         * @param frontRightMotor The motor that controls the front right
         *                        wheel.
         * @param backLeftMotor   The motor that controls the back left wheel.
         * @param backRightMotor  The motor that controls the back right wheel.
         */
        protected MecanumWheelMotors(
            DcMotor frontLeftMotor,
            DcMotor frontRightMotor,
            DcMotor backLeftMotor,
            DcMotor backRightMotor
        ) {
            motors.add(frontLeftMotor);
            motors.add(frontRightMotor);
            motors.add(backLeftMotor);
            motors.add(backRightMotor);

            this.frontLeftMotor = frontLeftMotor;
            this.frontRightMotor = frontRightMotor;
            this.backLeftMotor = backLeftMotor;
            this.backRightMotor = backRightMotor;
        }

        /**
         * Instantiate an empty motor set. Mostly used for dummy purposes.
         */
        public MecanumWheelMotors() {
            frontLeftMotor = null;
            frontRightMotor = null;
            backLeftMotor = null;
            backRightMotor = null;
        }

        /**
         * Check whether any of the given motors ({@link #frontLeftMotor},
         * {@link #frontRightMotor}, {@link #backLeftMotor}, or
         * {@link #backRightMotor}) are {@code null}.
         *
         * @return {@code true} if none of the given motors
         * ({@link #frontLeftMotor}, {@link #frontRightMotor},
         * {@link #backLeftMotor}, or {@link #backRightMotor}) are
         * {@code null}.
         * <p>
         * {@code false} otherwise.
         */
        @Override
        public boolean isValid() {
            return !(
                frontLeftMotor == null
                || frontRightMotor == null
                || backLeftMotor == null
                || backRightMotor == null
            );
        }

        /**
         * Set the value of {@link #frontLeftMotor} while also updating
         * {@link #motors} to remove the old motor and include the new motor.
         *
         * @param frontLeftMotor The new motor to set as the front left motor.
         */
        protected void setFrontLeftMotor(DcMotor frontLeftMotor) {
            motors.remove(this.frontLeftMotor);

            this.frontLeftMotor = frontLeftMotor;
            motors.add(frontLeftMotor);
        }

        /**
         * Set the value of {@link #frontRightMotor} while also updating
         * {@link #motors} to remove the old motor and include the new motor.
         *
         * @param frontRightMotor The new motor to set as the front right
         *                        motor.
         */
        protected void setFrontRightMotor(DcMotor frontRightMotor) {
            motors.remove(this.frontRightMotor);

            this.frontRightMotor = frontRightMotor;
            motors.add(frontRightMotor);
        }

        /**
         * Set the value of {@link #backLeftMotor} while also updating
         * {@link #motors} to remove the old motor and include the new motor.
         *
         * @param backLeftMotor The new motor to set as the back left motor.
         */
        protected void setBackLeftMotor(DcMotor backLeftMotor) {
            motors.remove(this.backLeftMotor);

            this.backLeftMotor = backLeftMotor;
            motors.add(backLeftMotor);
        }

        /**
         * Set the value of {@link #backRightMotor} while also updating
         * {@link #motors} to remove the old motor and include the new motor.
         *
         * @param backRightMotor The new motor to set as the back right motor.
         */
        protected void setBackRightMotor(DcMotor backRightMotor) {
            motors.remove(this.backRightMotor);

            this.backRightMotor = backRightMotor;
            motors.add(backRightMotor);
        }
    }

    /**
     * Builder to simplify the construction of {@link MecanumWheels} objects.
     * <p>
     * <h1>Example</h1>
     * <pre>
     * {@code
     * MecanumWheels mecanumWheels =
     *      new MecanumWheels.Builder()
     *                       .setFrontLeftMotor(frontLeftMotor)
     *                       .setFrontRightMotor(frontRightMotor)
     *                       .setBackLeftMotor(backLeftMotor)
     *                       .setBackRightMotor(backRightMotor)
     *                       .setLateralDistance(36.0)
     *                       .setLongitudinalDistance(36.0)
     *                       .build()
     * }
     * </pre>
     */
    public static class Builder extends Wheels.Builder {
        protected final MecanumWheelMotors mecanumWheelMotors;

        public Builder() {
            mecanumWheelMotors = new MecanumWheelMotors();
            wheelDistances = new WheelDistances(-1.0, -1.0);
            ticksPerInch = -1.0;
        }

        @SuppressWarnings("UnusedReturnValue")
        public Builder setFrontLeftMotor(DcMotor frontLeftMotor) {
            mecanumWheelMotors.setFrontLeftMotor(frontLeftMotor);
            return this;
        }

        @SuppressWarnings("UnusedReturnValue")
        public Builder setFrontRightMotor(DcMotor frontRightMotor) {
            mecanumWheelMotors.setFrontRightMotor(frontRightMotor);
            return this;
        }

        @SuppressWarnings("UnusedReturnValue")
        public Builder setBackLeftMotor(DcMotor backLeftMotor) {
            mecanumWheelMotors.setBackLeftMotor(backLeftMotor);
            return this;
        }

        @SuppressWarnings("UnusedReturnValue")
        public Builder setBackRightMotor(DcMotor backRightMotor) {
            mecanumWheelMotors.setBackRightMotor(backRightMotor);
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public Builder setLateralDistance(double lateralDistance) {
            wheelDistances.lateralDistance = lateralDistance;
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public Builder setLongitudinalDistance(double longitudinalDistance) {
            wheelDistances.longitudinalDistance = longitudinalDistance;
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public Builder setTicksPerInch(double ticksPerInch) {
            this.ticksPerInch = ticksPerInch;
            return this;
        }

        /**
         * Using the given {@link MecanumWheelMotors}, {@link WheelDistances},
         * and {@link #ticksPerInch}, construct a new instance of
         * {@link MecanumWheels}.
         *
         * @return A new instance of {@link MecanumWheels} if the values are
         * valid.
         * <p>
         * Else, {@code null}.
         */
        @Override
        public MecanumWheels build() {
            if (
                !mecanumWheelMotors.isValid()
                || !wheelDistances.isValid()
                || ticksPerInch <= 0
            ) {
                return null;
            }

            return new MecanumWheels(
                mecanumWheelMotors,
                wheelDistances,
                ticksPerInch
            );
        }
    }

    /**
     * The motor powering the front left wheel.
     */
    protected final DcMotor FRONT_LEFT_MOTOR;
    /**
     * The motor powering the front right wheel.
     */
    protected final DcMotor FRONT_RIGHT_MOTOR;
    /**
     * The motor powering the back left wheel.
     */
    protected final DcMotor BACK_LEFT_MOTOR;
    /**
     * The motor powering the back right wheel.
     */
    protected final DcMotor BACK_RIGHT_MOTOR;

    protected MecanumWheels(
        MecanumWheelMotors mecanumWheelMotors,
        WheelDistances wheelDistances,
        double ticksPerInch
    ) {
        super(mecanumWheelMotors.motors, wheelDistances, ticksPerInch);

        FRONT_LEFT_MOTOR = mecanumWheelMotors.frontLeftMotor;
        FRONT_RIGHT_MOTOR = mecanumWheelMotors.frontRightMotor;
        BACK_LEFT_MOTOR = mecanumWheelMotors.backLeftMotor;
        BACK_RIGHT_MOTOR = mecanumWheelMotors.backRightMotor;

        // Reset position to 0.
        for (DcMotor motor : motors) {
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
        for (DcMotor motor : motors) {
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

        for (DcMotor motor : motors) {
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

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
}