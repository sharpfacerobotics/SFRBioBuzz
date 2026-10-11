package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.robot.helperClasses.InterpLUT;

public class ShooterConfig {
    public static double currentRobotHeading;
    public static double[] elementVelocity; //[double x, double y, double hoodPollenAngle, double hoodNectarAngle]
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
    private static final InterpLUT distToVel = new InterpLUT(distance, velocity);
    private static final InterpLUT apriltagTaToVel = new InterpLUT(apriltagTa, velocity);
    private static final InterpLUT distToPollenAngle = new InterpLUT(distance, pollenHoodAngle);
    private static final InterpLUT distToNectarAngle = new InterpLUT(distance, nectarHoodAngle);
    public void calculateShootingConfig(double averageTa, double[] dist, double[] robotTranslationalVelocity, double targetLLHeading, double currRobotHeading) {
        //use the InterpLUT class to get the shooter velocity from averageTa and dist, then AVERAGE the two.
        double distance = Math.hypot(dist[0], dist[1]);
        double magnitude = distToVel.get(distance);
        if (averageTa != 0.0) {
            magnitude = 0.9 * (distToVel.get(distance)) + 0.1 * (apriltagTaToVel.get(averageTa));
        }
        currentRobotHeading = currRobotHeading;

        //then do vector math for x and y

        //update the elementVelocity list

    }
}
