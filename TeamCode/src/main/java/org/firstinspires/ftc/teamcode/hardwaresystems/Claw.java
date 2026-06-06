package org.firstinspires.ftc.teamcode.hardwaresystems;

import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashSet;
import java.util.Set;

/**
 * Abstract class to represent all possible robot claws and their common
 * characteristics.
 */
public abstract class Claw {
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
         * to set {@link Claw#servoIncrement}.
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
        }

        /**
         * Return whether the current parameters (i.e., {@link #rollServo},
         * {@link #pitchServo} {@link #yawServo}, {@link #servoIncrement}) are
         * valid, which is {@code true} if and only if {@link #servoIncrement}
         * is positive.
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
         * If the given {@link BuilderParameters} are invalid as defined by
         * {@link BuilderParameters#isValid()}, fail and return {@code null}.
         *
         * @return A new {@link Claw} object based the given parameters.
         */
        @Override
        public abstract Claw build();
    }

    /**
     * The servo that rotates the claw about the x-axis (roll).
     */
    protected final Servo ROLL_SERVO;
    /**
     * The servo that rotates the claw about the y-axis (pitch).
     */
    protected final Servo PITCH_SERVO;
    /**
     * The servo that rotates the claw about the z-axis (yaw).
     */
    protected final Servo YAW_SERVO;
    /**
     * A {@link Set} of all the {@link Servo}s that are in this claw.
     */
    private final Set<Servo> servos;
    /**
     * The number of motor ticks that the {@link Servo}s move with every loop.
     * Essentially serves as the "speed" or "power"　of the servo.
     */
    private double servoIncrement;

    /**
     * Instantiate a new {@link Claw} with three servos.
     *
     * @param rollServo      The servo that controls the claw's roll.
     * @param pitchServo     The servo that controls the claw's pitch.
     * @param yawServo       The servo that controls the claw's yaw.
     * @param servoIncrement The increment that the servos use per robot loop.
     */
    public Claw(
        Servo rollServo,
        Servo pitchServo,
        Servo yawServo,
        double servoIncrement
    ) {
        servos = new HashSet<>();
        if (rollServo != null) {
            servos.add(rollServo);
        }
        if (pitchServo != null) {
            servos.add(pitchServo);
        }
        if (yawServo != null) {
            servos.add(yawServo);
        }

        ROLL_SERVO = rollServo;
        PITCH_SERVO = pitchServo;
        YAW_SERVO = yawServo;

        this.servoIncrement = servoIncrement;
    }

    /**
     * Overload {@link Claw#Claw(Servo, Servo, Servo, double)} with
     * {@link #servoIncrement} defaulting to 0.1.
     *
     * @param rollServo  The servo that controls the claw's roll.
     * @param pitchServo The servo that controls the claw's pitch.
     * @param yawServo   The servo that controls the claw's yaw.
     */
    public Claw(Servo rollServo, Servo pitchServo, Servo yawServo) {
        this(rollServo, pitchServo, yawServo, 0.1);
    }

    /**
     * Get all the {@link Servo}s that are included in this claw system.
     *
     * @return A {@link Set} that contains every {@code Claw} included in this
     * claw system.
     */
    public Set<Servo> getServos() {
        return servos;
    }

    /**
     * Get the number of motor ticks that the {@link Servo}s move with every
     * loop.
     *
     * @return The number of motor ticks that the {@link Servo}s move with every
     * loop.
     * @see #servoIncrement
     */
    public double getServoIncrement() {
        return servoIncrement;
    }

    /**
     * Get the number of motor ticks that the {@link Servo}s move with every
     * loop.
     *
     * @see #servoIncrement
     */
    public void setServoIncrement(double servoIncrement) {
        this.servoIncrement = servoIncrement;
    }

    /**
     * Rotate the {@link #ROLL_SERVO} in a certain direction by
     * {@link Claw#servoIncrement}.
     *
     * @param direction The direction to rotate the servo in, as seen from the
     *                  <em>front</em> of the servo. Positive values rotate it
     *                  counterclockwise, and negative values rotate it
     *                  clockwise.
     */
    public void rotateRollServo(double direction) {
        if (ROLL_SERVO != null) {
            double targetPosition = ROLL_SERVO.getPosition()
                                    + Math.signum(direction) * servoIncrement;
            ROLL_SERVO.setPosition(targetPosition);
        }
    }

    /**
     * Rotate the {@link #ROLL_SERVO} to a position specified in degrees.
     *
     * @param degrees The target angle of the {@link #ROLL_SERVO} in degrees.
     */
    public void rotateRollServoToAngle(double degrees) {
        if (ROLL_SERVO != null) {
            ROLL_SERVO.setPosition(degrees / 360.0);
        }
    }

    /**
     * Rotate the {@link #PITCH_SERVO} in a certain direction by
     * {@link #servoIncrement}.
     *
     * @param direction The direction to rotate the servo in, as seen from the
     *                  <em>front</em> of the servo. Positive values rotate it
     *                  counterclockwise, and negative values rotate it
     *                  clockwise.
     */
    public void rotatePitchAxisServo(double direction) {
        if (PITCH_SERVO != null) {
            double targetPosition = PITCH_SERVO.getPosition()
                                    + Math.signum(direction) * servoIncrement;
            PITCH_SERVO.setPosition(targetPosition);
        }
    }

    /**
     * Rotate the {@link #PITCH_SERVO} to a position specified in degrees.
     *
     * @param degrees The target angle of the {@link #PITCH_SERVO} in degrees.
     */
    public void rotatePitchServoToAngle(double degrees) {
        if (PITCH_SERVO != null) {
            PITCH_SERVO.setPosition(degrees / 360.0);
        }
    }

    /**
     * Rotate the {@link  #YAW_SERVO} in a certain direction by
     * {@link #servoIncrement}.
     *
     * @param direction The direction to rotate the servo in, as seen from the
     *                  <em>front</em> of the servo. Positive values rotate it
     *                  counterclockwise, and negative values rotate it
     *                  clockwise.
     */
    public void rotateYawServo(double direction) {
        if (YAW_SERVO != null) {
            double targetPosition = YAW_SERVO.getPosition()
                                    + Math.signum(direction) * servoIncrement;
            YAW_SERVO.setPosition(targetPosition);
        }
    }

    /**
     * Rotate the {@link #YAW_SERVO} to a position specified in degrees.
     *
     * @param degrees The target angle of the {@link #YAW_SERVO} in degrees.
     */
    public void rotateYawServoToAngle(double degrees) {
        if (YAW_SERVO != null) {
            YAW_SERVO.setPosition(degrees / 360.0);
        }
    }
}