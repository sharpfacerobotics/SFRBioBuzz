package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.robot.helperClasses.InterpLUT;

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
    // InterpLUT needs at least two points, so a table stays null until its data is filled in
    private static final InterpLUT distToVel = buildLut(distance, velocity);
    private static final InterpLUT apriltagTaToVel = buildLut(apriltagTa, velocity);
    private static final InterpLUT distToPollenAngle = buildLut(distance, pollenHoodAngle);
    private static final InterpLUT distToNectarAngle = buildLut(distance, nectarHoodAngle);

    private static InterpLUT buildLut(double[] x, double[] y) {
        if (x.length < 2) {
            return null;
        }
        return new InterpLUT(x, y);
    }
    public void calculateShootingConfig(double averageTa, double[] dist, double[] robotTranslationalVelocity, double targetLLHeading, double currRobotHeading) {
        //use the InterpLUT class to get the shooter velocity from averageTa and dist, then AVERAGE the two.
        currentRobotHeading = currRobotHeading;
        if (distToVel == null) {
            return; // no shooting data yet
        }
        double distance = Math.hypot(dist[0], dist[1]);
        double magnitude = distToVel.get(distance);
        if (averageTa != 0.0 && apriltagTaToVel != null) {
            magnitude = 0.9 * magnitude + 0.1 * apriltagTaToVel.get(averageTa);
        }

        //then do vector math for x and y

        //update the elementVelocity list

    }
}
