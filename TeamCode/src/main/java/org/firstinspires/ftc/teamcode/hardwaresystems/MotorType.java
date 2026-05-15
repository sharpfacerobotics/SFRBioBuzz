package org.firstinspires.ftc.teamcode.hardwaresystems;

/**
 * The brand and model of motor. Stores the specifications for various motors.
 */
public enum MotorType {
    TETRIX_TORQUENADO(1440);

    /**
     * The number of ticks it takes for a full rotation.
     */
    private final int TICKS_PER_ROTATION;

    MotorType(int ticksPerRotation) {
        this.TICKS_PER_ROTATION = ticksPerRotation;
    }

    public int getTicksPerRotation() {
        return TICKS_PER_ROTATION;
    }
}