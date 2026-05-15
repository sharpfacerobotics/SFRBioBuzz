package org.firstinspires.ftc.teamcode.hardwaresystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

/**
 * Claw with one continuous rotation servo to intake game elements.
 * <p>
 * See {@link DoubleServoIntakeClaw} for the double servo version.
 */
public class SingleServoIntakeClaw extends Claw {
    /**
     * How much power the intake spins with when intaking.
     */
    private static final double INTAKE_POWER = 0.5;
    /**
     * How much power the intake spines with when ejecting.
     */
    private static final double EJECT_POWER = -1.0;
    /**
     * The servo that spins the intake.
     */
    private final CRServo INTAKE_SERVO;
    /**
     * The touch sensor that touches whether there is a piece in the intake.
     */
    private final DigitalChannel INTAKE_SENSOR;

    public SingleServoIntakeClaw(Servo xAxisServo, Servo yAxisServo,
                                 Servo zAxisServo, CRServo intakeServo) {
        this(xAxisServo, yAxisServo, zAxisServo, intakeServo, null);
    }

    public SingleServoIntakeClaw(Servo xAxisServo, Servo yAxisServo,
                                 Servo zAxisServo, CRServo intakeServo,
                                 DigitalChannel intakeSensor) {
        super(xAxisServo, yAxisServo, zAxisServo);

        INTAKE_SERVO = intakeServo;
        INTAKE_SENSOR = intakeSensor;
    }

    public double getIntakePower() {
        return INTAKE_POWER;
    }

    public double getEjectPower() {
        return EJECT_POWER;
    }

    public Set<CRServo> getCrServos() {
        return new HashSet<>(Collections.singletonList(INTAKE_SERVO));
    }

    public CRServo getIntakeServo() {
        return INTAKE_SERVO;
    }

    public void startIntake() {
        INTAKE_SERVO.setPower(INTAKE_POWER);
    }

    /**
     * Get whether the intake servo is currently running.
     *
     * @return true if the intake servo's power is non-zero, false otherwise.
     */
    public boolean isIntakeActive() {
        return INTAKE_SERVO.getPower() != 0;
    }

    public void stopIntake() {
        INTAKE_SERVO.setPower(0);
    }

    /**
     * Make the intake spin in reverse and eject the object.
     */
    public void ejectIntake() {
        INTAKE_SERVO.setPower(EJECT_POWER);
    }

    /**
     * Get whether the sensor on the claw is pressed or not.
     *
     * @return true when the sensor is pressed, false otherwise.
     */
    public boolean isSensorPressed() {
        // `getState()` returns true when the sensor is not pressed.
        return !INTAKE_SENSOR.getState();
    }
}