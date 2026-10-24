package org.firstinspires.ftc.teamcode.robot;

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

    /** Max correction (rad) the Limelight bearing may apply to the odometry aim. */
    public static double MAX_VISION_CORRECTION = Math.toRadians(8);

    // ------------------------------------------------------------------ outputs
    // Static, as in the team's design: Launcher.setLauncher(q) and Turret.updateTurret() read
    // them (the glossary gives those methods no parameters for it).

    /**
     * [vx, vy, pollenHoodAngle, nectarHoodAngle]: the launch velocity as a field-frame vector
     * (direction = where to aim, length = flywheel ticks/s) plus the hood angle per element.
     */
    public static final double[] elementVelocity = new double[4];
    /** Distance (in) to the (shoot-on-the-move) aim point. */
    public static double distanceToGoal;

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
     * SET. Loop call. Calculates the target launch velocity and angle as a vector.
     *
     * @param averageTa                  Vision.getAverageTa(), 0 if no tag in view
     * @param dist                       {dx, dy} (in, field frame) from the robot to the target cell
     * @param robotTranslationalVelocity {vx, vy} (in/s, field frame) from the follower
     * @param targetHeading              Vision.getTargetHeading(): field heading to the target as
     *                                   the camera sees it, or NaN - corrects small odometry error
     */
    public void calculateShootingConfig(double averageTa, double[] dist, double[] robotTranslationalVelocity,
                                        double targetHeading) {
        double gx = dist[0];
        double gy = dist[1];
        double vx = robotTranslationalVelocity[0];
        double vy = robotTranslationalVelocity[1];

        // If the camera sees the target, trust its bearing over odometry's (a little).
        double bias = 0;
        if (!Double.isNaN(targetHeading)) {
            bias = normalize(targetHeading - Math.atan2(gy, gx));
            bias = Math.max(-MAX_VISION_CORRECTION, Math.min(MAX_VISION_CORRECTION, bias));
        }

        // Shoot on the move: shift the goal opposite our velocity by the flight time.
        double dx = gx;
        double dy = gy;
        double d = Math.hypot(dx, dy);
        for (int i = 0; i < SHOOT_ON_MOVE_ITERATIONS; i++) {
            double tof = distToTof.get(d);
            dx = gx - vx * tof;
            dy = gy - vy * tof;
            d = Math.hypot(dx, dy);
        }

        double odoVelocity = distToVel.get(d);
        double speed = averageTa > 0.0
                ? (1.0 - TA_WEIGHT) * odoVelocity + TA_WEIGHT * apriltagTaToVel.get(averageTa)
                : odoVelocity;
        double aim = Math.atan2(dy, dx) + bias;

        distanceToGoal = d;
        elementVelocity[0] = speed * Math.cos(aim);
        elementVelocity[1] = speed * Math.sin(aim);
        elementVelocity[2] = distToPollenAngle.get(d);
        elementVelocity[3] = distToNectarAngle.get(d);
    }

    /** Flywheel target (ticks/s) = length of the launch vector. */
    public static double targetVelocity() {
        return Math.hypot(elementVelocity[0], elementVelocity[1]);
    }

    /** Field heading (rad) to aim along = direction of the launch vector. */
    public static double aimHeading() {
        return Math.atan2(elementVelocity[1], elementVelocity[0]);
    }

    /** Hood angle for the element about to be shot ("NECTAR" / anything else = pollen). */
    public static double hoodAngleFor(String next) {
        return "NECTAR".equals(next) ? elementVelocity[3] : elementVelocity[2];
    }

    /** Close enough (and far enough) to shoot - the distance table only covers this band. */
    public static boolean isInRange() {
        return distanceToGoal >= RobotConfig.MIN_SHOT_DISTANCE && distanceToGoal <= RobotConfig.MAX_SHOT_DISTANCE;
    }

    private static double normalize(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
