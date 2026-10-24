package org.firstinspires.ftc.teamcode.robot;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.junit.Test;

/** ShooterConfig.calculateShootingConfig(averageTa, dist[], robotTranslationalVelocity[], targetHeading). */
public class ShooterConfigTest {
    private static final double[] STILL = {0, 0};
    private final ShooterConfig sc = new ShooterConfig(null);

    @Test
    public void straightAheadNoMotion() {
        sc.calculateShootingConfig(0, new double[]{0, 48}, STILL, Double.NaN);
        assertEquals(48, ShooterConfig.distanceToGoal, 1e-9);
        assertEquals(Math.PI / 2, ShooterConfig.aimHeading(), 1e-9);            // +y
        assertEquals(ShooterConfig.velocity[1], ShooterConfig.targetVelocity(), 1e-6); // table row at 48 in
    }

    @Test
    public void launchVectorEncodesSpeedAndDirection() {
        sc.calculateShootingConfig(0, new double[]{-48, 0}, STILL, Double.NaN);   // goal toward -x
        assertEquals(-ShooterConfig.targetVelocity(), ShooterConfig.elementVelocity[0], 1e-6);
        assertEquals(0, ShooterConfig.elementVelocity[1], 1e-6);
    }

    @Test
    public void movingRightAimsLeftOfTheGoal() {
        sc.calculateShootingConfig(0, new double[]{0, 48}, new double[]{20, 0}, Double.NaN);
        assertTrue(ShooterConfig.aimHeading() > Math.PI / 2 + 0.05); // rotated CCW (toward -x)
    }

    @Test
    public void visionBearingCorrectsOdometryButIsClamped() {
        sc.calculateShootingConfig(0, new double[]{0, 48}, STILL, Math.PI / 2 + Math.toRadians(3));
        assertEquals(Math.PI / 2 + Math.toRadians(3), ShooterConfig.aimHeading(), 1e-9);
        sc.calculateShootingConfig(0, new double[]{0, 48}, STILL, Math.PI / 2 + Math.toRadians(40));
        assertEquals(Math.PI / 2 + ShooterConfig.MAX_VISION_CORRECTION, ShooterConfig.aimHeading(), 1e-9);
    }

    @Test
    public void hoodDependsOnElement() {
        sc.calculateShootingConfig(0, new double[]{0, 48}, STILL, Double.NaN);
        assertNotEquals(ShooterConfig.hoodAngleFor(Storage.POLLEN), ShooterConfig.hoodAngleFor(Storage.NECTAR), 1e-9);
        assertEquals(ShooterConfig.hoodAngleFor(Storage.POLLEN), ShooterConfig.hoodAngleFor(null), 1e-12); // empty -> pollen
    }

    @Test
    public void rangeCheckUsesDistanceBand() {
        sc.calculateShootingConfig(0, new double[]{0, 48}, STILL, Double.NaN);
        assertTrue(ShooterConfig.isInRange());
        sc.calculateShootingConfig(0, new double[]{0, 10}, STILL, Double.NaN);
        assertFalse(ShooterConfig.isInRange());
        sc.calculateShootingConfig(0, new double[]{144, 144}, STILL, Double.NaN);
        assertFalse(ShooterConfig.isInRange());
    }

    @Test
    public void rebuildTablesPicksUpEdits() {
        double old = ShooterConfig.velocity[1];
        try {
            ShooterConfig.velocity[1] = 9999;
            sc.calculateShootingConfig(0, new double[]{0, 48}, STILL, Double.NaN);
            assertEquals(old, ShooterConfig.targetVelocity(), 1e-6); // tables are cached...
            sc.rebuildTables();
            sc.calculateShootingConfig(0, new double[]{0, 48}, STILL, Double.NaN);
            assertEquals(9999, ShooterConfig.targetVelocity(), 1e-6); // ...until rebuilt
        } finally {
            ShooterConfig.velocity[1] = old;
            sc.rebuildTables();
        }
    }

    @Test
    public void drivetrainDistanceToTheUpwardCell() {
        Drivetrain dt = new Drivetrain(null);
        Pose robot = new Pose(30, 72, 0);
        double[] d = dt.getPedroDistanceToHive(robot, "RED", "GARDEN_SIDE"); // red garden side = audience cell
        assertEquals(59 - 30, d[0], 1e-9);
        assertTrue(d[1] < 0); // audience cell is below centre
        d = dt.getPedroDistanceToHive(robot, "RED");                       // hive pivot
        assertEquals(0, d[1], 1e-9);
    }
}
