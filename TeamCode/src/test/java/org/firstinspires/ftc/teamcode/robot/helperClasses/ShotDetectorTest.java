package org.firstinspires.ftc.teamcode.robot.helperClasses;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class ShotDetectorTest {
    private static final double DROP = 120, WINDOW = 0.15;
    private final ShotDetector d = new ShotDetector();

    @Test
    public void suddenDipIsOneShot() {
        assertFalse(d.update(true, 1500, 0.00, DROP, WINDOW));
        assertFalse(d.update(false, 1440, 0.02, DROP, WINDOW)); // dipping
        assertTrue(d.update(false, 1350, 0.04, DROP, WINDOW));  // shot
        assertFalse(d.update(false, 1300, 0.06, DROP, WINDOW)); // same dip, not a 2nd shot
    }

    @Test
    public void reArmsAfterRecovery() {
        d.update(true, 1500, 0, DROP, WINDOW);
        assertTrue(d.update(false, 1350, 0.02, DROP, WINDOW));
        d.update(true, 1500, 0.30, DROP, WINDOW);
        assertTrue(d.update(false, 1350, 0.32, DROP, WINDOW));
    }

    @Test
    public void slowDriftIsNotAShot() {
        // e.g. target changed while driving: out of tolerance but no sudden drop
        d.update(true, 1500, 0, DROP, WINDOW);
        assertFalse(d.update(false, 1470, 0.10, DROP, WINDOW));
        assertFalse(d.update(false, 1440, 0.20, DROP, WINDOW)); // window expired -> disarm
        assertFalse(d.update(false, 1300, 0.40, DROP, WINDOW)); // big gap later still not a shot
    }

    @Test
    public void neverReadyNeverShoots() {
        assertFalse(d.update(false, 0, 0, DROP, WINDOW));
        assertFalse(d.update(false, -500, 0.01, DROP, WINDOW));
    }
}
