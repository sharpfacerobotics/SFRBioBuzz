package org.firstinspires.ftc.teamcode.hardwaresystems;

import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashSet;

/**
 * Abstract class to define what methods all robot claws should be capable of.
 */
public abstract class Claw {
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
     * A {@link HashSet} of all the {@link Servo}s that are in this claw.
     */
    private final HashSet<Servo> SERVOS;
    /**
     * How much to gradually move the servo.
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
        Servo rollServo, Servo pitchServo, Servo yawServo,
        double servoIncrement
    ) {
        SERVOS = new HashSet<>();
        SERVOS.add(rollServo);
        SERVOS.add(pitchServo);
        SERVOS.add(yawServo);

        ROLL_SERVO = rollServo;
        PITCH_SERVO = pitchServo;
        YAW_SERVO = yawServo;

        this.servoIncrement = servoIncrement;
    }

    /**
     * Overloads {@link Claw#Claw(Servo, Servo, Servo, double)} with {@link #servoIncrement} defaulting to 0.1.
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
     * @return A {@link HashSet} that contains every {@code Claw} included in this claw system.
     */
    public HashSet<Servo> getServos() {
        return SERVOS;
    }

    public double getServoIncrement() {
        return servoIncrement;
    }

    public void setServoIncrement(double servoIncrement) {
        this.servoIncrement = servoIncrement;
    }

    /**
     * Rotate the {@link #ROLL_SERVO} in a certain direction by {@link Claw#servoIncrement}.
     *
     * @param direction The direction to rotate the servo in. Positive values rotate it clockwise; negative values
     *                  rotate it counterclockwise.
     */
    public void rotateRollServo(double direction) {
        double targetPosition =
            ROLL_SERVO.getPosition() + Math.signum(direction) * servoIncrement;
        ROLL_SERVO.setPosition(targetPosition);
    }

    /**
     * Rotate the {@link #ROLL_SERVO} to a position specified in degrees.
     *
     * @param degrees The target angle of the {@link #ROLL_SERVO} in degrees.
     */
    public void rotateRollServoToAngle(double degrees) {
        ROLL_SERVO.setPosition(degrees / 360.0);
    }

    /**
     * Rotate the {@link #PITCH_SERVO} in a certain direction by {@link #servoIncrement}.
     *
     * @param direction The direction to rotate the servo in. Positive values rotate it clockwise; negative values
     *                  rotate it counterclockwise.
     */
    public void rotatePitchAxisServo(double direction) {
        double targetPosition = PITCH_SERVO.getPosition() + Math.signum(direction) * servoIncrement;
        PITCH_SERVO.setPosition(targetPosition);
    }

    /**
     * Rotate the {@link #PITCH_SERVO} to a position specified in degrees.
     *
     * @param degrees The target angle of the {@link #PITCH_SERVO} in degrees.
     */
    public void rotatePitchServoToAngle(double degrees) {
        PITCH_SERVO.setPosition(degrees / 360.0);
    }

    /**
     * Rotate the {@link  #YAW_SERVO} in a certain direction by {@link #servoIncrement}.
     *
     * @param direction The direction to rotate the servo in. Positive values rotate it clockwise; negative values
     *                  rotate it counterclockwise.
     */
    public void rotateYawServo(double direction) {
        double targetPosition = YAW_SERVO.getPosition() + Math.signum(direction) * servoIncrement;
        YAW_SERVO.setPosition(targetPosition);
    }

    /**
     * Rotate the {@link #YAW_SERVO} to a position specified in degrees.
     *
     * @param degrees The target angle of the {@link #YAW_SERVO} in degrees.
     */
    public void rotateYawServoToAngle(double degrees) {
        YAW_SERVO.setPosition(degrees / 360.0);
    }
}