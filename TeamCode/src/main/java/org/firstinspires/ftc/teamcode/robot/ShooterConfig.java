package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.robot.helperClasses.InterpLUT;

/**
 * Shot solver: distance to the target cell -> flywheel velocity and hood angle, leading the
 * target while the robot moves. Fill the tables from TagShotTest's logged rows; each table is
 * skipped until it has at least two points.
 */
// Not @Configurable: the tables are built once at load, so Panels edits to them would do nothing
public class ShooterConfig {
    public static double currentRobotHeading;
    public static double[] elementVelocity = new double[4]; //[double x, double y, double hoodPollenAngle, double hoodNectarAngle]
    public static double[] distance = {

    };
    public static double[] apriltagTa = {

    };
    public static double[] velocity = {

    };
    public static double[] pollenHoodAngle = {

    };
    public static double[] nectarHoodAngle = {

    };
    public static double[] timeOfFlight = { // seconds, same distances as above

    };
    // How much to trust the AprilTag area over the distance when both give a velocity
    public static double TA_WEIGHT = 0.1;
    // Max correction the camera's bearing may apply to the odometry aim
    public static double MAX_VISION_CORRECTION = Math.toRadians(8);
    // TODO(measure): average ball speed, used for flight time until timeOfFlight has data
    public static double BALL_SPEED_IN_PER_S = 300.0;
    public static int SHOOT_ON_MOVE_ITERATIONS = 3;

    // Outputs of calculateShootingConfig
    public static double distanceToGoal = RobotConfig.MIN_DISTANCE;
    public static double aimHeading = 0.0;
    public static double targetVelocity = Double.NaN;  // NaN until the velocity table has data

    // InterpLUT needs at least two points, so a table stays null until its data is filled in
    private static final InterpLUT distToVel = buildLut(distance, velocity);
    private static final InterpLUT apriltagTaToVel = buildLut(apriltagTa, velocity);
    private static final InterpLUT distToPollenAngle = buildLut(distance, pollenHoodAngle);
    private static final InterpLUT distToNectarAngle = buildLut(distance, nectarHoodAngle);
    private static final InterpLUT distToTof = buildLut(distance, timeOfFlight);

    private static InterpLUT buildLut(double[] x, double[] y) {
        if (x.length < 2 || y.length < 2) {
            return null;
        }
        return new InterpLUT(x, y);
    }

    /** Flywheel velocity (ticks/s) for this distance (in), or NaN until the table has data. */
    public static double velocityForDistance(double inches) {
        return distToVel == null ? Double.NaN : distToVel.get(inches);
    }

    /** Stand-in until the table has data: power on a straight line between the RobotConfig limits. */
    public static double fallbackPowerForDistance(double inches) {
        double t = (inches - RobotConfig.MIN_DISTANCE) / (RobotConfig.MAX_DISTANCE - RobotConfig.MIN_DISTANCE);
        t = Math.max(0.0, Math.min(1.0, t));
        return RobotConfig.MIN_LAUNCH_POWER + t * (RobotConfig.MAX_LAUNCH_POWER - RobotConfig.MIN_LAUNCH_POWER);
    }

    /** Ball flight time (s) for this distance: table, or distance / BALL_SPEED_IN_PER_S. */
    public static double timeOfFlightFor(double inches) {
        return distToTof == null ? inches / BALL_SPEED_IN_PER_S : distToTof.get(inches);
    }

    /**
     * Solve the shot. All vectors are in one frame (field frame with odometry, or the robot
     * frame with heading 0 when only the camera is available).
     *
     * @param averageTa                  Vision.getAverageTa(), 0 if no tag in view
     * @param dist                       {dx, dy} (in) from the robot to the target cell
     * @param robotTranslationalVelocity {vx, vy} (in/s) of the robot
     * @param targetLLHeading            heading to the target as the camera sees it, or NaN
     * @param currRobotHeading           robot heading (rad) in the same frame
     */
    public static void calculateShootingConfig(double averageTa, double[] dist, double[] robotTranslationalVelocity,
                                               double targetLLHeading, double currRobotHeading) {
        currentRobotHeading = currRobotHeading;
        double gx = dist[0];
        double gy = dist[1];
        double vx = robotTranslationalVelocity[0];
        double vy = robotTranslationalVelocity[1];

        // When the camera sees the cell, let its bearing correct small odometry errors
        double bias = 0.0;
        if (!Double.isNaN(targetLLHeading)) {
            bias = normalize(targetLLHeading - Math.atan2(gy, gx));
            bias = Math.max(-MAX_VISION_CORRECTION, Math.min(MAX_VISION_CORRECTION, bias));
        }

        // Shoot on the move: aim at where the cell will be relative to us when the ball lands
        double dx = gx;
        double dy = gy;
        double d = Math.hypot(dx, dy);
        for (int i = 0; i < SHOOT_ON_MOVE_ITERATIONS; i++) {
            double tof = timeOfFlightFor(d);
            dx = gx - vx * tof;
            dy = gy - vy * tof;
            d = Math.hypot(dx, dy);
        }

        distanceToGoal = d;
        aimHeading = normalize(Math.atan2(dy, dx) + bias);
        targetVelocity = velocityForDistance(d);
        if (!Double.isNaN(targetVelocity) && averageTa > 0.0 && apriltagTaToVel != null) {
            targetVelocity = (1.0 - TA_WEIGHT) * targetVelocity + TA_WEIGHT * apriltagTaToVel.get(averageTa);
        }

        double magnitude = Double.isNaN(targetVelocity) ? 1.0 : targetVelocity;
        elementVelocity[0] = magnitude * Math.cos(aimHeading);
        elementVelocity[1] = magnitude * Math.sin(aimHeading);
        elementVelocity[2] = distToPollenAngle == null ? Double.NaN : distToPollenAngle.get(d);
        elementVelocity[3] = distToNectarAngle == null ? Double.NaN : distToNectarAngle.get(d);
    }

    /** Hood angle for the next element ("NECTAR" / anything else = pollen), NaN without a table. */
    public static double hoodAngleFor(String next) {
        return "NECTAR".equals(next) ? elementVelocity[3] : elementVelocity[2];
    }

    /** Inside the distance band the shooter is set up for. */
    public static boolean isInRange() {
        return distanceToGoal >= RobotConfig.MIN_DISTANCE && distanceToGoal <= RobotConfig.MAX_DISTANCE;
    }

    private static double normalize(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
