package org.firstinspires.ftc.teamcode.hardwaresystems;

/**
 * The brand and model of motor. Stores the specifications for various motors.
 */
public enum MotorType {
    TETRIX_TORQUENADO(1440),

    /**
     * REV UltraPlanetary motor encoder counts per output shaft revolution.
     *
     * <p>The REV UltraPlanetary gearbox pairs an HD Hex motor with one
     * or more interchangeable gear cartridges.  The HD Hex motor’s
     * integrated encoder produces 28 counts per motor shaft revolution.
     * When combined with a 20:1 gear stack (for example a 4:1 and 5:1
     * cartridge), the effective output shaft encoder count becomes
     * {@code 28 × 20 = 560} counts per revolution.  This value is a
     * common configuration used for FTC drivetrains and provides a
     * reasonable default for robots built with the UltraPlanetary system.
     *
     * <p>If your robot uses a different gear ratio (e.g. 15:1, 60:1,
     * etc.), adjust this value accordingly.  For example, a 60:1
     * UltraPlanetary stack would yield {@code 28 × 60 = 1680} counts
     * per output shaft revolution.  Having the correct tick count is
     * important for accurate odometry and distance control.</p>
     */
    REV_ULTRA_PLANETARY(560);

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