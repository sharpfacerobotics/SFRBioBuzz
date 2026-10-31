package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.robot.helperClasses.InterpLUT;

public class ShooterConfig {
    public static double currentRobotHeading;
    public static double[] elementVelocity = {0, 1200, 0.9, 0.95}; //[double x, double y, double hoodPollenAngle, double hoodNectarAngle]
    public static double[] distance = {
        0, 20, 40, 60, 70
    };
    public static double[] apriltagTa = {
        0, 0.2, 0.4, 0.6, 0.7 //PLACEHOLDER
    };
    public static double[] velocity = {
        1200, 1200, 1600, 1630, 1650
    };
    public static double[] pollenHoodAngle = {
        0.9, 0.85, 0.8, 0.75, 0.73
    };
    public static double[] nectarHoodAngle = {
        0.95, 0.9, 0.85, 0.8, 0.77
    };
    private static final InterpLUT distToVel = new InterpLUT(distance, velocity);
    private static final InterpLUT apriltagTaToVel = new InterpLUT(apriltagTa, velocity);
    private static final InterpLUT distToPollenAngle = new InterpLUT(distance, pollenHoodAngle);
    private static final InterpLUT distToNectarAngle = new InterpLUT(distance, nectarHoodAngle);
    public void calculateShootingConfig(double averageTa, double[] dist, double[] robotTranslationalVelocity, double targetLLHeading, double currRobotHeading) {
        //use the InterpLUT class to get the shooter velocity from averageTa and dist, then AVERAGE the two.
        double distance = Math.hypot(dist[0], dist[1]);
        double[] dirc;
        if (distance == 0) {
            dirc = new double[] {0,0};
        } else {
            dirc = new double[]{dist[0] / distance,
                    dist[1] / distance};
        }

        double magnitude = distToVel.get(distance);
        /*
        if (averageTa != 0.0) {
            magnitude = 0.9 * (distToVel.get(distance)) + 0.1 * (apriltagTaToVel.get(averageTa));
        }
        */

        currentRobotHeading = currRobotHeading;

        //then do vector math for x and y
        //update the elementVelocity list
        elementVelocity = new double[] {magnitude * dirc[0], magnitude * dirc[1],
                distToPollenAngle.get(distance), distToNectarAngle.get(distance)};
    }
}