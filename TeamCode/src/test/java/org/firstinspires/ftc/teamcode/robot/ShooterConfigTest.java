package org.firstinspires.ftc.teamcode.robot;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class ShooterConfigTest {
    private final ShooterConfig sc = new ShooterConfig(null);

    @Test
    public void facingTheGoalMeansTurretZero() {
        sc.calculateShootingConfig(72, 72, Math.PI / 2, 0, 0, 72, 120, 0);
        assertEquals(48, sc.getDistanceToGoal(), 1e-9);
        assertEquals(0, sc.getTurretAngle(), 1e-9);
        assertEquals(ShooterConfig.velocity[1], sc.getTargetVelocity(), 1e-6); // table row at 48 in
    }

    @Test
    public void goalOnTheLeftIsPlus90() {
        sc.calculateShootingConfig(72, 72, Math.PI / 2, 0, 0, 24, 72, 0);
        assertEquals(Math.PI / 2, sc.getTurretAngle(), 1e-9);
    }

    @Test
    public void turretAngleIsNormalized() {
        sc.calculateShootingConfig(72, 72, Math.PI / 2, 0, 0, 72, 24, 0); // goal behind
        assertTrue(sc.getTurretAngle() > -Math.PI && sc.getTurretAngle() <= Math.PI);
        assertEquals(Math.PI, Math.abs(sc.getTurretAngle()), 1e-9);
    }

    @Test
    public void movingRightAimsLeftOfTheGoal() {
        sc.calculateShootingConfig(72, 72, Math.PI / 2, 20, 0, 72, 120, 0);
        assertTrue(sc.getTurretAngle() > 0.05);
    }

    @Test
    public void hoodDependsOnElement() {
        sc.calculateShootingConfig(72, 72, 0, 0, 0, 72, 120, 0);
        assertNotEquals(sc.getHoodAngle(Storage.POLLEN), sc.getHoodAngle(Storage.NECTAR), 1e-9);
        assertEquals(sc.getHoodAngle(Storage.POLLEN), sc.getHoodAngle(null), 1e-12); // empty -> pollen
    }

    @Test
    public void rebuildTablesPicksUpEdits() {
        double old = ShooterConfig.velocity[1];
        try {
            ShooterConfig.velocity[1] = 9999;
            sc.calculateShootingConfig(72, 72, Math.PI / 2, 0, 0, 72, 120, 0);
            assertEquals(old, sc.getTargetVelocity(), 1e-6); // tables are cached...
            sc.rebuildTables();
            sc.calculateShootingConfig(72, 72, Math.PI / 2, 0, 0, 72, 120, 0);
            assertEquals(9999, sc.getTargetVelocity(), 1e-6); // ...until rebuilt
        } finally {
            ShooterConfig.velocity[1] = old;
        }
    }

    @Test
    public void rangeCheckUsesDistanceBand() {
        sc.calculateShootingConfig(72, 72, 0, 0, 0, 72, 120, 0);     // 48 in
        assertTrue(sc.isInRange());
        sc.calculateShootingConfig(72, 110, 0, 0, 0, 72, 120, 0);    // 10 in - too close
        assertFalse(sc.isInRange());
        sc.calculateShootingConfig(0, 0, 0, 0, 0, 144, 144, 0);      // ~204 in - too far
        assertFalse(sc.isInRange());
    }
}
