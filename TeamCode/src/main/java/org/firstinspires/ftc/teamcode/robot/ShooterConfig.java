package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.robot.RobotConfig.Element;
import org.firstinspires.ftc.teamcode.robot.helperClasses.InterpLUT;

/**
 * Works out how to shoot from where the robot is. Owner: Alex.
 *
 * <p>Inputs: robot pose + velocity (Pedro) and hive position (RobotConfig, per alliance).
 * Outputs: flywheel velocity, hood angle per element, and the turret angle (relative to the
 * robot's front) to aim at.</p>
 *
 * <p>Shoot on the move: an element keeps the robot's velocity after it leaves, so we aim at
 * where the hive will be <i>relative to us</i> after the flight time:
 * {@code virtualGoal = goal - robotVelocity * timeOfFlight}. Time of flight depends on
 * distance, which depends on the virtual goal, so we iterate a couple of times.</p>
 */
public class ShooterConfig {
    private final RobotHardware robot;

    // ------------------------------------------------------------------ lookup tables
    // TODO(tune): PLACEHOLDER DATA. Shoot from each distance, record what works, replace.
    // InterpLUT needs at least 2 points with strictly increasing x, or it throws at startup.
    public static double[] distance = {24, 48, 72, 96, 120, 144};              // inches
    public static double[] velocity = {1100, 1300, 1500, 1700, 1900, 2100};    // ticks/s
    public static double[] pollenHoodAngle = {0.20, 0.30, 0.40, 0.48, 0.55, 0.60};
    public static double[] nectarHoodAngle = {0.25, 0.35, 0.45, 0.52, 0.58, 0.63};
    public static double[] timeOfFlight = {0.35, 0.45, 0.55, 0.65, 0.75, 0.85}; // seconds
    // Limelight target area (%) -> velocity. Area shrinks with distance, so x must be
    // listed small (far) to large (close).
    public static double[] apriltagTa = {0.15, 0.25, 0.4, 0.7, 1.2, 2.5};
    public static double[] taVelocity = {2100, 1900, 1700, 1500, 1300, 1100};
    /** How much to trust ta over odometry distance when both are available. */
    public static double TA_WEIGHT = 0.1;
    public static int SHOOT_ON_MOVE_ITERATIONS = 3;

    private InterpLUT distToVel;
    private InterpLUT distToPollenAngle;
    private InterpLUT distToNectarAngle;
    private InterpLUT distToTof;
    private InterpLUT apriltagTaToVel;

    // ------------------------------------------------------------------ outputs
    private double targetVelocity;
    private double pollenAngle;
    private double nectarAngle;
    private double distanceToGoal;
    private double fieldAimHeading;
    private double turretAngle;

    public ShooterConfig(RobotHardware robot) {
        this.robot = robot;
        rebuildTables();
    }

    /**
     * Rebuild the interpolation tables from the static arrays. The tables are precomputed for
     * speed, so editing the arrays at runtime does nothing until this is called. Not for the
     * loop - it allocates.
     */
    public void rebuildTables() {
        distToVel = new InterpLUT(distance, velocity);
        distToPollenAngle = new InterpLUT(distance, pollenHoodAngle);
        distToNectarAngle = new InterpLUT(distance, nectarHoodAngle);
        distToTof = new InterpLUT(distance, timeOfFlight);
        apriltagTaToVel = new InterpLUT(apriltagTa, taVelocity);
    }

    /**
     * Call once per loop before Launcher/Turret update.
     *
     * @param robotX       field x (in)
     * @param robotY       field y (in)
     * @param robotHeading field heading (rad)
     * @param robotVx      field-frame x velocity (in/s), 0 to disable shoot-on-the-move
     * @param robotVy      field-frame y velocity (in/s)
     * @param goalX        hive x (in) for our alliance
     * @param goalY        hive y (in)
     * @param averageTa    HiveVision average target area, or 0 if no tag in view
     */
    public void calculateShootingConfig(double robotX, double robotY, double robotHeading,
                                        double robotVx, double robotVy,
                                        double goalX, double goalY, double averageTa) {
        double dx = goalX - robotX;
        double dy = goalY - robotY;
        double dist = Math.hypot(dx, dy);

        // Shoot on the move: shift the goal opposite our velocity by the flight time.
        for (int i = 0; i < SHOOT_ON_MOVE_ITERATIONS; i++) {
            double tof = distToTof.get(dist);
            dx = goalX - robotVx * tof - robotX;
            dy = goalY - robotVy * tof - robotY;
            dist = Math.hypot(dx, dy);
        }

        distanceToGoal = dist;
        fieldAimHeading = Math.atan2(dy, dx);
        turretAngle = normalize(fieldAimHeading - robotHeading);

        double odoVelocity = distToVel.get(dist);
        if (averageTa > 0.0) {
            targetVelocity = (1.0 - TA_WEIGHT) * odoVelocity + TA_WEIGHT * apriltagTaToVel.get(averageTa);
        } else {
            targetVelocity = odoVelocity;
        }
        pollenAngle = distToPollenAngle.get(dist);
        nectarAngle = distToNectarAngle.get(dist);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    /** Hood angle for the element about to be shot (null/UNKNOWN use pollen). */
    public double getHoodAngle(Element next) {
        return next == Element.NECTAR ? nectarAngle : pollenAngle;
    }

    /** Close enough (and far enough) to shoot - the distance table only covers this band. */
    public boolean isInRange() {
        return distanceToGoal >= RobotConfig.MIN_SHOT_DISTANCE && distanceToGoal <= RobotConfig.MAX_SHOT_DISTANCE;
    }

    public double getDistanceToGoal() {
        return distanceToGoal;
    }

    /** Field heading (rad) from the robot to the (virtual) goal. */
    public double getFieldAimHeading() {
        return fieldAimHeading;
    }

    /** Turret angle (rad) relative to robot forward, in (-pi, pi]. */
    public double getTurretAngle() {
        return turretAngle;
    }

    private static double normalize(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
