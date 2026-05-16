package org.firstinspires.ftc.teamcode.hardwaresystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashSet;

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

    public DoubleServoIntakeClaw(CRServo leftIntakeServo, CRServo rightIntakeServo) {
        this(null, null, null, leftIntakeServo, rightIntakeServo, null);
    }

    public DoubleServoIntakeClaw(CRServo leftIntakeServo, CRServo rightIntakeServo, DigitalChannel intakeSensor) {
        this(null, null, null, leftIntakeServo, rightIntakeServo, intakeSensor);
    }

    public DoubleServoIntakeClaw(
        Servo rollServo,
        Servo pitchServo,
        Servo yawServo,
        CRServo leftIntakeServo,
        CRServo rightIntakeServo
    ) {
        this(rollServo, pitchServo, yawServo, leftIntakeServo, rightIntakeServo, null);
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

    public double getIntakePower() {
        return INTAKE_POWER;
    }

    public double getEjectPower() {
        return EJECT_POWER;
    }

    public HashSet<CRServo> getCrServos() {
        HashSet<CRServo> crServos = new HashSet<>();
        crServos.add(LEFT_INTAKE_SERVO);
        crServos.add(RIGHT_INTAKE_SERVO);

        return new HashSet<>(crServos);
    }

    public CRServo getLeftIntakeServo() {
        return LEFT_INTAKE_SERVO;
    }

    public CRServo getRightIntakeServo() {
        return RIGHT_INTAKE_SERVO;
    }

    public void startIntake() {
        LEFT_INTAKE_SERVO.setPower(INTAKE_POWER);
        RIGHT_INTAKE_SERVO.setPower(INTAKE_POWER);
    }

    /**
     * Get whether the intake servo is currently running.
     *
     * @return true if the intake servo's power is non-zero, false otherwise.
     */
    public boolean isIntakeActive() {
        return LEFT_INTAKE_SERVO.getPower() != 0 && RIGHT_INTAKE_SERVO.getPower() != 0;
    }

    public void stopIntake() {
        LEFT_INTAKE_SERVO.setPower(0);
        RIGHT_INTAKE_SERVO.setPower(0);
    }

    /**
     * Make the intake spin in reverse and eject the object.
     */
    public void ejectIntake() {
        LEFT_INTAKE_SERVO.setPower(EJECT_POWER);
        RIGHT_INTAKE_SERVO.setPower(EJECT_POWER);
    }

    /**
     * Get whether the sensor on the claw is pressed or not.
     *
     * @return true when the sensor is pressed, false otherwise.
     */
    public boolean isSensorPressed() {
        //  returns true when the sensor is not pressed.
        return !INTAKE_SENSOR.getState();
    }
}