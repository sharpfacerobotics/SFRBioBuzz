package org.firstinspires.ftc.teamcode.hardwaresystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.HashSet;
import java.util.Set;

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
    protected static class MecanumWheelMotors extends BuilderParameters {
        /**
         * All the motors that are used by this {@link Wheels} system. Should
         * never contain {@code null}.
         */
        protected Set<DcMotor> motors = new HashSet<>();

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
            if (frontLeftMotor != null) {
                motors.add(frontLeftMotor);
            }
            if (frontRightMotor != null) {
                motors.add(frontRightMotor);
            }
            if (backLeftMotor != null) {
                motors.add(backLeftMotor);
            }
            if (backRightMotor != null) {
                motors.add(backRightMotor);
            }

            this.frontLeftMotor = frontLeftMotor;
            this.frontRightMotor = frontRightMotor;
            this.backLeftMotor = backLeftMotor;
            this.backRightMotor = backRightMotor;
        }

        /**
         * Instantiate an empty motor set. Mostly used for dummy purposes.
         */
        public MecanumWheelMotors() {
            this(null, null, null, null);
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
            if (frontLeftMotor != null) {
                motors.add(frontLeftMotor);
            }
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
            if (frontRightMotor != null) {
                motors.add(frontRightMotor);
            }
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
            if (backLeftMotor != null) {
                motors.add(backLeftMotor);
            }
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
            if (backRightMotor != null) {
                motors.add(backRightMotor);
            }
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
        /**
         * The current state of the {@link Builder}.
         */
        protected final MecanumWheelMotors mecanumWheelMotors;

        /**
         * Instantiate a new {@link FoldingArm.Builder} with all
         * {@link BuilderParameters} set to their default value.
         */
        public Builder() {
            super();
            mecanumWheelMotors = new MecanumWheelMotors();
        }

        /**
         * Set the motor that controls the front-left mecanum wheel.
         *
         * @param frontLeftMotor The front-left mecanum wheel.
         * @return This {@link Builder} so that setters can be chained.
         */
        @SuppressWarnings("UnusedReturnValue")
        public Builder setFrontLeftMotor(DcMotor frontLeftMotor) {
            mecanumWheelMotors.setFrontLeftMotor(frontLeftMotor);
            return this;
        }

        /**
         * Set the motor that controls the front-right mecanum wheel.
         *
         * @param frontRightMotor The front-right mecanum wheel.
         * @return This {@link Builder} so that setters can be chained.
         */
        @SuppressWarnings("UnusedReturnValue")
        public Builder setFrontRightMotor(DcMotor frontRightMotor) {
            mecanumWheelMotors.setFrontRightMotor(frontRightMotor);
            return this;
        }

        /**
         * Set the motor that controls the back-right mecanum wheel.
         *
         * @param backLeftMotor The back-right mecanum wheel.
         * @return This {@link Builder} so that setters can be chained.
         */
        @SuppressWarnings("UnusedReturnValue")
        public Builder setBackLeftMotor(DcMotor backLeftMotor) {
            mecanumWheelMotors.setBackLeftMotor(backLeftMotor);
            return this;
        }

        /**
         * Set the motor that controls the back-right mecanum wheel.
         *
         * @param backRightMotor The back-right mecanum wheel.
         * @return This {@link Builder} so that setters can be chained.
         */
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
            super.setTicksPerInch(ticksPerInch);
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
                || maxMotorPower <= 0
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

    /**
     * Instantiate a new {@link MecanumWheels} object with the motors,
     * distances, and ticks per inch all set.
     *
     * @param mecanumWheelMotors THe motors used to drive the mecanum wheels.
     * @param wheelDistances     The distances between the mecanum wheels for
     *                           calculations involving distances and turning.
     * @param ticksPerInch       The number of ticks per inch, assuming that
     *                           each wheel is identical.
     */
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

    /**
     * Get the motor that drives the front-left mecanum wheel.
     *
     * @return The motor that drives the front-left mecanum wheel.
     */
    public DcMotor getFrontLeftMotor() {
        return FRONT_LEFT_MOTOR;
    }

    /**
     * Get the motor that drives the front-right mecanum wheel.
     *
     * @return The motor that drives the front-right mecanum wheel.
     */
    public DcMotor getFrontRightMotor() {
        return FRONT_RIGHT_MOTOR;
    }

    /**
     * Get the motor that drives the back-left mecanum wheel.
     *
     * @return The motor that drives the back-left mecanum wheel.
     */
    public DcMotor getBackLeftMotor() {
        return BACK_LEFT_MOTOR;
    }

    /**
     * Get the motor that drives the back-right mecanum wheel.
     *
     * @return The motor that drives the back-right mecanum wheel.
     */
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
        if (maxMagnitude > MAX_MOTOR_POWER) {
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
        double xPower = (sidewaysDistance / totalDistance) * MAX_MOTOR_POWER;
        double yPower = (forwardDistance / totalDistance) * MAX_MOTOR_POWER;
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
        FRONT_LEFT_MOTOR.setPower(-MAX_MOTOR_POWER);
        BACK_LEFT_MOTOR.setTargetPosition(
            BACK_LEFT_MOTOR.getCurrentPosition() - ticks
        );
        BACK_LEFT_MOTOR.setPower(-MAX_MOTOR_POWER);

        // Right wheels
        FRONT_RIGHT_MOTOR.setTargetPosition(
            FRONT_RIGHT_MOTOR.getCurrentPosition() + ticks
        );
        FRONT_RIGHT_MOTOR.setPower(MAX_MOTOR_POWER);
        BACK_RIGHT_MOTOR.setTargetPosition(
            BACK_RIGHT_MOTOR.getCurrentPosition() + ticks
        );
        BACK_RIGHT_MOTOR.setPower(MAX_MOTOR_POWER);

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
}