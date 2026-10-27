package org.firstinspires.ftc.teamcode.robot;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class ShooterConfigTest {
    private static final double EPS = 1e-9;

    @Test
    public void fallbackPowerIsStraightLineAndClamped() {
        assertEquals(RobotConfig.MIN_LAUNCH_POWER, ShooterConfig.fallbackPowerForDistance(0), EPS);
        assertEquals(RobotConfig.MAX_LAUNCH_POWER, ShooterConfig.fallbackPowerForDistance(500), EPS);
        double mid = (RobotConfig.MIN_DISTANCE + RobotConfig.MAX_DISTANCE) / 2;
        assertEquals((RobotConfig.MIN_LAUNCH_POWER + RobotConfig.MAX_LAUNCH_POWER) / 2,
                ShooterConfig.fallbackPowerForDistance(mid), EPS);
    }

    @Test
    public void emptyTablesGiveNaNVelocityAndHood() {
        assertTrue(Double.isNaN(ShooterConfig.velocityForDistance(80)));
        ShooterConfig.calculateShootingConfig(0, new double[] {80, 0}, new double[] {0, 0}, Double.NaN, 0);
        assertTrue(Double.isNaN(ShooterConfig.targetVelocity));
        assertTrue(Double.isNaN(ShooterConfig.hoodAngleFor("POLLEN")));
    }

    @Test
    public void standingStillAimsStraightAtTheGoal() {
        ShooterConfig.calculateShootingConfig(0, new double[] {60, 60}, new double[] {0, 0}, Double.NaN, 0);
        assertEquals(Math.hypot(60, 60), ShooterConfig.distanceToGoal, 1e-6);
        assertEquals(Math.PI / 4, ShooterConfig.aimHeading, 1e-9);
    }

    @Test
    public void drivingSidewaysLeadsTheShot() {
        // Goal 100 in ahead (+x), robot moving +y: aim must swing to -y, by the flight time
        ShooterConfig.calculateShootingConfig(0, new double[] {100, 0}, new double[] {0, 30}, Double.NaN, 0);
        assertTrue(ShooterConfig.aimHeading < 0);
        double tof = ShooterConfig.timeOfFlightFor(ShooterConfig.distanceToGoal);
        assertEquals(Math.atan2(-30 * tof, 100), ShooterConfig.aimHeading, 1e-6);
    }

    @Test
    public void cameraBearingCorrectionIsCapped() {
        ShooterConfig.calculateShootingConfig(0, new double[] {100, 0}, new double[] {0, 0}, Math.toRadians(30), 0);
        assertEquals(ShooterConfig.MAX_VISION_CORRECTION, ShooterConfig.aimHeading, 1e-9);
    }

    @Test
    public void rangeCheckUsesRobotConfigBand() {
        ShooterConfig.calculateShootingConfig(0, new double[] {RobotConfig.MIN_DISTANCE - 1, 0}, new double[] {0, 0}, Double.NaN, 0);
        assertFalse(ShooterConfig.isInRange());
        ShooterConfig.calculateShootingConfig(0, new double[] {RobotConfig.MIN_DISTANCE + 1, 0}, new double[] {0, 0}, Double.NaN, 0);
        assertTrue(ShooterConfig.isInRange());
    }
}
