package org.firstinspires.ftc.teamcode.hardwaresystems;

import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashSet;
import java.util.Set;

/**
 * Abstract class to represent all possible robot claws and their common
 * characteristics.
 */
@SuppressWarnings("unused")
public abstract class Claw {
    /**
     * Builder for {@link Claw} to control the setting of {@link #ROLL_SERVO},
     * {@link #PITCH_SERVO}, and {@link #YAW_SERVO}.
     */
    public abstract static class Builder extends HardwareSystemBuilder {
        /**
         * The servo that controls the {@link Claw}'s roll (rotation about the
         * front-to-back axis). Used to set {@link #ROLL_SERVO}.
         */
        protected Servo rollServo;
        /**
         * The servo that controls the {@link Claw}'s pitch (rotation about the
         * side-to-side axis). Used to set {@link #PITCH_SERVO}.
         */
        protected Servo pitchServo;
        /**
         * The servo that controls the {@link Claw}'s roll (rotation about the
         * top-to-bottom axis). Used to set {@link #YAW_SERVO}.
         */
        protected Servo yawServo;

        /**
         * The number of motor ticks that the {@link Servo}s move with every
         * loop. Essentially serves as the "speed" or "power"　of the servo. Used
         * to set {@link #MAX_SERVO_INCREMENT}.
         */
        protected double servoIncrement;

        /**
         * Instantiate a new {@link Claw} object with no {@link Servo}s.
         */
        public Builder() {
            super();
            rollServo = null;
            pitchServo = null;
            yawServo = null;

            servoIncrement = 0.1;
        }

        /**
         * Set the {@link Servo} used to control roll (see
         * {@link #ROLL_SERVO}).
         *
         * @param rollServo The servo used to control roll.
         * @return This {@link Builder} to allow for chaining setters.
         */
        public Builder setRollServo(Servo rollServo) {
            this.rollServo = rollServo;
            return this;
        }

        /**
         * Set the {@link Servo} used to control pitch (see
         * {@link #PITCH_SERVO}).
         *
         * @param pitchServo The servo used to control pitch.
         * @return This {@link Builder} to allow for chaining setters.
         */
        public Builder setPitchServo(Servo pitchServo) {
            this.pitchServo = pitchServo;
            return this;
        }

        /**
         * Set the {@link Servo} used to control yaw (see {@link #YAW_SERVO}).
         *
         * @param yawServo The servo used to control yaw.
         * @return This {@link Builder} to allow for chaining setters.
         */
        public Builder setYawServo(Servo yawServo) {
            this.yawServo = rollServo;
            return this;
        }

        /**
         * Set he number of ticks the {@link Servo}s move with every loop.
         * Essentially serves as the "speed" or "power"　of the servo.
         */
        public Builder setServoIncrement(double servoIncrement) {
            this.servoIncrement = servoIncrement;
            return this;
        }

        /**
         * Return whether the current attributes are valid, which is
         * {@code true} if and only if {@link #servoIncrement} is positive.
         * <p>
         * {@code null} values for {@link #rollServo}, {@link #pitchServo},
         * {@link #yawServo} are acceptable, indicating that the given axis is
         * not used. However, because of this, all methods
         * <em><strong>must</strong></em> check for {@code null} {@link Servo}
         * values.
         *
         * @return Whether the current parameters (i.e., {@link #rollServo},
         * {@link #pitchServo} {@link #yawServo}, {@link #servoIncrement}) are
         * valid, which is {@code true} if and only if {@link #servoIncrement}
         * is positive.
         */
        @Override
        public boolean isValid() {
            return servoIncrement > 0;
        }

        /**
         * Instantiate a new {@link Claw} object based on the given parameters
         * <p>
         * If the given attributes are invalid as defined by {@link #isValid()},
         * fail and return {@code null}.
         *
         * @return A new {@link Claw} object based the given parameters.
         */
        @Override
        public abstract Claw build();
    }

    /**
     * A {@link Set} of all the {@link Servo}s that are in this claw.
     */
    private final Set<Servo> SERVOS;

    /**
     * The servo that rotates the claw about the x-axis (roll).
     */
    private final Servo ROLL_SERVO;
    /**
     * The servo that rotates the claw about the y-axis (pitch).
     */
    private final Servo PITCH_SERVO;
    /**
     * The servo that rotates the claw about the z-axis (yaw).
     */
    private final Servo YAW_SERVO;

    /**
     * The number of ticks that the {@link Servo}s move with every loop.
     * Essentially serves as the "speed" or "power"　of the servo.
     */
    private final double MAX_SERVO_INCREMENT;

    /**
     * Instantiate a new {@link Claw} with up to three servos and a given servo
     * increment value.
     *
     * @param builder The {@link Builder} object that contains the values to use
     *                in instantiation. May be invalid.
     * @throws IllegalArgumentException If the {@link Builder} object is invalid
     *                                  as defined by
     *                                  {@link Builder#isValid()}.
     */
    protected Claw(Builder builder) {
        if (!builder.isValid()) {
            throw new IllegalArgumentException("Claw builder is invalid.");
        }

        SERVOS = new HashSet<>();

        if (builder.rollServo != null) {
            SERVOS.add(builder.rollServo);
        }
        if (builder.pitchServo != null) {
            SERVOS.add(builder.pitchServo);
        }
        if (builder.yawServo != null) {
            SERVOS.add(builder.yawServo);
        }

        ROLL_SERVO = builder.rollServo;
        PITCH_SERVO = builder.pitchServo;
        YAW_SERVO = builder.yawServo;

        MAX_SERVO_INCREMENT = builder.servoIncrement;
    }

    /**
     * {@return a {@link Set} that contains every {@code Claw} included in this
     * claw system}
     */
    public Set<Servo> getServos() {
        return SERVOS;
    }

    /**
     * {@return the fraction of its range that the {@link Servo}s move with
     * every loop}
     */
    public double getMaxServoIncrement() {
        return MAX_SERVO_INCREMENT;
    }

    /**
     * Rotate the {@link #ROLL_SERVO} to a relative position specified by
     * {@link #MAX_SERVO_INCREMENT} in a given direction.
     *
     * @param direction The direction to rotate the servo in relative to
     *                  {@link Servo#getDirection()}. As seen from the front of
     *                  the {@link Servo}, {@link Servo.Direction#FORWARD} makes
     *                  the {@link Servo} rotate in the direction of
     *                  {@link Servo#getDirection()}, and
     *                  {@link Servo.Direction#REVERSE} makes the {@link Servo}
     *                  rotate in the opposite direction.
     */
    public void rotateRollServo(Servo.Direction direction) {
        int directionSign = direction == Servo.Direction.FORWARD ? 1 : -1;
        rotateRollServo(directionSign * MAX_SERVO_INCREMENT);
    }

    /**
     * Rotate the {@link #ROLL_SERVO} to a relative position specified by a
     * proportion of {@link #MAX_SERVO_INCREMENT}.
     *
     * @param power How far to rotate the {@link #ROLL_SERVO} by as a proportion
     *              of {@link #MAX_SERVO_INCREMENT}. Negative values will make
     *              the {@link #ROLL_SERVO} rotate backwards.
     */
    public void rotateRollServo(double power) {
        // Clamp the absolute value of the power to be less than 
        // MAX_SERVO_INCREMENT.
        double clampedPower = Math.min(
            Math.max(power, -MAX_SERVO_INCREMENT),
            MAX_SERVO_INCREMENT
        );
        // Rotate to the relative position.
        if (ROLL_SERVO != null) {
            double targetPosition = ROLL_SERVO.getPosition()
                                    + power * MAX_SERVO_INCREMENT;
            ROLL_SERVO.setPosition(targetPosition);
        }
    }

    /**
     * Rotate the {@link #ROLL_SERVO} to an absolute position specified in
     * degrees.
     *
     * @param degrees The target angle of the {@link #ROLL_SERVO} in degrees.
     */
    public void rotateRollServoToAngle(double degrees) {
        if (ROLL_SERVO != null) {
            ROLL_SERVO.setPosition(degrees / 360.0);
        }
    }

    /**
     * Rotate the {@link #PITCH_SERVO} to a relative position specified by
     * {@link #MAX_SERVO_INCREMENT} in a given direction.
     *
     * @param direction The direction to rotate the servo in relative to
     *                  {@link Servo#getDirection()}. As seen from the front of
     *                  the {@link Servo}, {@link Servo.Direction#FORWARD} makes
     *                  the {@link Servo} rotate in the direction of
     *                  {@link Servo#getDirection()}, and
     *                  {@link Servo.Direction#REVERSE} makes the {@link Servo}
     *                  rotate in the opposite direction.
     */
    public void rotatePitchServo(Servo.Direction direction) {
        int directionSign = direction == Servo.Direction.FORWARD ? 1 : -1;
        rotatePitchServo(directionSign * MAX_SERVO_INCREMENT);
    }

    /**
     * Rotate the {@link #PITCH_SERVO} to a relative position specified by a
     * proportion of {@link #MAX_SERVO_INCREMENT}.
     *
     * @param power How far to rotate the {@link #PITCH_SERVO} by as a
     *              proportion of {@link #MAX_SERVO_INCREMENT}. Negative values
     *              will make the {@link #PITCH_SERVO} rotate backwards.
     */
    public void rotatePitchServo(double power) {
        // Clamp the absolute value of the power to be less than 
        // MAX_SERVO_INCREMENT.
        double clampedPower = Math.min(
            Math.max(power, -MAX_SERVO_INCREMENT),
            MAX_SERVO_INCREMENT
        );
        // Rotate to the relative position.
        if (PITCH_SERVO != null) {
            double targetPosition = PITCH_SERVO.getPosition()
                                    + power * MAX_SERVO_INCREMENT;
            PITCH_SERVO.setPosition(targetPosition);
        }
    }

    /**
     * Rotate the {@link #PITCH_SERVO} to an absolute position specified in
     * degrees.
     *
     * @param degrees The target angle of the {@link #PITCH_SERVO} in degrees.
     */
    public void rotatePitchServoToAngle(double degrees) {
        if (PITCH_SERVO != null) {
            PITCH_SERVO.setPosition(degrees / 360.0);
        }
    }

    /**
     * Rotate the {@link  #YAW_SERVO} to a relative position specified by
     * {@link #MAX_SERVO_INCREMENT} in a given direction.
     *
     * @param direction The direction to rotate the servo in relative to
     *                  {@link Servo#getDirection()}. As seen from the front of
     *                  the {@link Servo}, {@link Servo.Direction#FORWARD} makes
     *                  the {@link Servo} rotate in the direction of
     *                  {@link Servo#getDirection()}, and
     *                  {@link Servo.Direction#REVERSE} makes the {@link Servo}
     *                  rotate in the opposite direction.
     */
    public void rotateYawServo(Servo.Direction direction) {
        int directionSign = direction == Servo.Direction.FORWARD ? 1 : -1;
        rotateYawServo(directionSign * MAX_SERVO_INCREMENT);
    }

    /**
     * Rotate the {@link #YAW_SERVO} to a relative position specified by a
     * proportion of {@link #MAX_SERVO_INCREMENT}.
     *
     * @param power How far to rotate the {@link #YAW_SERVO} by as a proportion
     *              of {@link #MAX_SERVO_INCREMENT}. Negative values will make
     *              the {@link #YAW_SERVO} rotate backwards.
     */
    public void rotateYawServo(double power) {
        // Clamp the absolute value of the power to be less than 
        // MAX_SERVO_INCREMENT.
        double clampedPower = Math.min(
            Math.max(power, -MAX_SERVO_INCREMENT),
            MAX_SERVO_INCREMENT
        );
        // Rotate to the relative position.
        if (PITCH_SERVO != null) {
            double targetPosition = YAW_SERVO.getPosition()
                                    + power * MAX_SERVO_INCREMENT;
            PITCH_SERVO.setPosition(targetPosition);
        }
    }

    /**
     * Rotate the {@link #YAW_SERVO} to an absolute position specified in
     * degrees.
     *
     * @param degrees The target angle of the {@link #YAW_SERVO} in degrees.
     */
    public void rotateYawServoToAngle(double degrees) {
        if (YAW_SERVO != null) {
            YAW_SERVO.setPosition(degrees / 360.0);
        }
    }
}