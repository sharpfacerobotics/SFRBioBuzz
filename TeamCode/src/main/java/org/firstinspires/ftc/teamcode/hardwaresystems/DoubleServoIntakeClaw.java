package org.firstinspires.ftc.teamcode.hardwaresystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashSet;
import java.util.Set;

/**
 * Servo with two continuous rotation servos to pick up objects.
 * <p>
 * See {@link SingleServoIntakeClaw} for the single-servo version.
 */
public class DoubleServoIntakeClaw extends Claw {
    /**
     * How much power the intake spins with when intaking.
     */
    private static final double INTAKE_POWER = 1.0;
    /**
     * How much power the intake spines with when ejecting.
     */
    private static final double EJECT_POWER = -1.0;
    /**
     * The left servo that spins the intake.
     */
    private final CRServo LEFT_INTAKE_SERVO;
    /**
     * The right servo that spins the intake.
     */
    private final CRServo RIGHT_INTAKE_SERVO;

    /**
     * The touch sensor that touches whether there is a piece in the intake.
     */
    private final DigitalChannel INTAKE_SENSOR;

    public DoubleServoIntakeClaw(
        CRServo leftIntakeServo,
        CRServo rightIntakeServo
    ) {
        this(null, null, null, leftIntakeServo, rightIntakeServo, null);
    }

    public DoubleServoIntakeClaw(
        CRServo leftIntakeServo,
        CRServo rightIntakeServo,
        DigitalChannel intakeSensor
    ) {
        this(null, null, null, leftIntakeServo, rightIntakeServo, intakeSensor);
    }

    public DoubleServoIntakeClaw(
        Servo rollServo,
        Servo pitchServo,
        Servo yawServo,
        CRServo leftIntakeServo,
        CRServo rightIntakeServo
    ) {
        this(
            rollServo,
            pitchServo,
            yawServo,
            leftIntakeServo,
            rightIntakeServo,
            null
        );
    }

    public DoubleServoIntakeClaw(
        Servo rollServo,
        Servo pitchServo,
        Servo yawServo,
        CRServo leftIntakeServo,
        CRServo rightIntakeServo,
        DigitalChannel intakeSensor
    ) {
        super(rollServo, pitchServo, yawServo);

        super.ROLL_SERVO.setDirection(Servo.Direction.REVERSE);

        LEFT_INTAKE_SERVO = leftIntakeServo;
        RIGHT_INTAKE_SERVO = rightIntakeServo;
        RIGHT_INTAKE_SERVO.setDirection(DcMotorSimple.Direction.REVERSE);
        INTAKE_SENSOR = intakeSensor;
    }

    /**
     * Get the power used to intake an object, i.e., pull it into the claw.
     *
     * @return The power used to intake an object, i.e., pull it into the claw.
     */
    public double getIntakePower() {
        return INTAKE_POWER;
    }

    /**
     * Get the power used to eject an object, i.e., push it out of the claw.
     *
     * @return The power used to intake an object, i.e., push it out of the
     * claw.
     */
    public double getEjectPower() {
        return EJECT_POWER;
    }

    /**
     * Get all the continuous rotation servos ({@link CRServo}s) used by this
     * {@link Claw}, i.e., the left and right intake servos.
     *
     * @return All the continuous rotation servos ({@link CRServo}s) used by
     * this {@link Claw}, i.e., the left and right intake servos.
     */
    public Set<CRServo> getCrServos() {
        return new HashSet<>(Set.of(
            LEFT_INTAKE_SERVO,
            RIGHT_INTAKE_SERVO
        ));
    }

    /**
     * Get the continuous rotation servo ({@link CRServo}) used to run the left
     * intake.
     *
     * @return The continuous rotation servo ({@link CRServo}) used to run the
     * left intake.
     */
    public CRServo getLeftIntakeServo() {
        return LEFT_INTAKE_SERVO;
    }

    /**
     * Get the continuous rotation servo ({@link CRServo}) used to run the right
     * intake.
     *
     * @return The continuous rotation servo ({@link CRServo}) used to run the
     * right intake.
     */
    public CRServo getRightIntakeServo() {
        return RIGHT_INTAKE_SERVO;
    }

    /**
     * Spin both intake servos to pull in an object.
     */
    public void startIntake() {
        if (LEFT_INTAKE_SERVO != null) {
            LEFT_INTAKE_SERVO.setPower(INTAKE_POWER);
        }
        if (RIGHT_INTAKE_SERVO != null) {
            RIGHT_INTAKE_SERVO.setPower(INTAKE_POWER);
        }
    }

    /**
     * Get whether <em>both</em> intake servos are currently running. If either
     * {@link Servo} is {@code null}, trivially return {@code false}.
     *
     * @return {@code true} if the intake servos' powers are <em>both</em>
     * non-zero.
     * <p>
     * {@code false} otherwise, including if either {@link Servo} is
     * {@code null}.
     */
    public boolean isIntakeActive() {
        return LEFT_INTAKE_SERVO != null
               && LEFT_INTAKE_SERVO.getPower() != 0
               && RIGHT_INTAKE_SERVO != null
               && RIGHT_INTAKE_SERVO.getPower() != 0;
    }

    /**
     * Stop both intake servos from either intaking or ejecting.
     */
    public void stopIntake() {
        if (LEFT_INTAKE_SERVO != null) {
            LEFT_INTAKE_SERVO.setPower(0);
        }
        if (RIGHT_INTAKE_SERVO != null) {
            RIGHT_INTAKE_SERVO.setPower(0);
        }
    }

    /**
     * Make the intake spin in reverse and eject the object.
     */
    public void ejectIntake() {
        if (LEFT_INTAKE_SERVO != null) {
            LEFT_INTAKE_SERVO.setPower(EJECT_POWER);
        }
        if (RIGHT_INTAKE_SERVO != null) {
            RIGHT_INTAKE_SERVO.setPower(EJECT_POWER);
        }
    }

    /**
     * Get whether the sensor on the claw is pressed or not. If there is no
     * sensor, i.e., the sensor is {@code null}, then trivially return
     * {@code false}.
     *
     * @return {@code true} when the sensor is pressed.
     * <p>
     * {@code false} otherwise, including if the {@link #INTAKE_SENSOR} is
     * {@code null}.
     */
    public boolean isSensorPressed() {
        //  Returns true when the sensor is not pressed.
        return INTAKE_SENSOR != null && !INTAKE_SENSOR.getState();
    }
}