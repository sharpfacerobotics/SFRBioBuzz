package org.firstinspires.ftc.teamcode.robot;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/**
 * Storage.checkBeamBreaks() queue logic (via the hardware-free process()), per the glossary:
 * UPDATE, returns true = emergency outtake.
 */
public class StorageQueueTest {
    private final Storage s = new Storage(null);
    private double t = 10; // seconds; each step() advances one 20 ms loop

    private boolean step(boolean low, boolean high, boolean exit, boolean outtaking, String colour, String alliance) {
        t += 0.02;
        return s.process(low, high, exit, outtaking, colour, alliance, t);
    }

    /** One element passes the intake beams in (rising then falling edge), then a gap. */
    private boolean feed(boolean nectar, String colour, String alliance) {
        boolean e = step(true, nectar, false, false, nectar ? colour : null, alliance);
        step(false, false, false, false, null, alliance);
        t += 0.2; // past the debounce before the next one
        return e;
    }

    @Test
    public void countsPollenAndNectarInOrder() {
        assertFalse(feed(false, null, "RED"));
        assertFalse(feed(true, "RED", "RED"));
        assertEquals(2, s.getElementQueue().size());
        assertEquals(Storage.POLLEN, s.getElementQueue().peek());
    }

    @Test
    public void opponentNectarIsEmergencyAndNotCounted() { // G408
        assertTrue(step(true, true, false, false, "BLUE", "RED"));
        assertEquals(0, s.getElementQueue().size());
        // beams clear but we haven't been outtaking yet: it may still be inside - keep going
        assertTrue(step(false, false, false, false, null, "RED"));
        t += 0.3; // outtake has run past the minimum
        assertFalse(step(false, false, false, true, null, "RED"));
        assertEquals(0, s.getElementQueue().size());
    }

    @Test
    public void emergencyGivesUpAfterMaxTime() {
        assertTrue(step(true, true, false, false, "BLUE", "RED"));
        t += RobotConfig.EMERGENCY_MAX_OUTTAKE + 0.1;
        assertFalse(step(true, true, false, true, null, "RED")); // still blocked, but don't reverse forever
    }

    @Test
    public void fifthElementIsEmergency() { // G407
        for (int i = 0; i < 4; i++) {
            assertFalse(feed(false, null, "BLUE"));
        }
        assertTrue(step(true, false, false, false, null, "BLUE"));
        assertEquals(4, s.getElementQueue().size());
    }

    @Test
    public void exitBeamCountsElementsIntoTheLauncher() {
        feed(true, "BLUE", "BLUE");
        feed(false, null, "BLUE");
        step(false, false, true, false, null, "BLUE");  // exit beam broken
        step(false, false, false, false, null, "BLUE"); // ...and cleared: one launched
        assertEquals(1, s.getElementQueue().size());
        assertEquals(Storage.POLLEN, s.getElementQueue().peek());
        assertEquals(1, s.getLaunchedCount());
    }

    @Test
    public void exitBeamIgnoredWhileOuttaking() {
        feed(false, null, "BLUE");
        step(false, false, true, true, null, "BLUE");
        step(false, false, false, true, null, "BLUE"); // pulled back across the exit beam
        assertEquals(1, s.getElementQueue().size());
        assertEquals(0, s.getLaunchedCount());
    }

    @Test
    public void outtakeRemovesTheLastElement() {
        feed(false, null, "RED");
        feed(true, "RED", "RED");
        step(true, true, false, true, null, "RED");   // nectar back at the beams, outtaking
        step(false, false, false, true, null, "RED"); // and gone
        assertEquals(1, s.getElementQueue().size());
        assertEquals(Storage.POLLEN, s.getElementQueue().peek());
        assertEquals(0, s.getLaunchedCount());
    }

    @Test
    public void lateHighBeamUpgradesToNectar() {
        step(true, false, false, false, null, "RED");   // low first: looks like POLLEN
        step(true, true, false, false, "RED", "RED");   // high breaks a loop later: NECTAR
        step(false, false, false, false, null, "RED");
        assertEquals(1, s.getElementQueue().size());
        assertEquals(Storage.NECTAR, s.getElementQueue().peek());
    }

    @Test
    public void lateHighBeamOpponentNectarIsRejected() {
        step(true, false, false, false, null, "RED");
        assertTrue(step(true, true, false, false, "BLUE", "RED"));
        assertEquals(0, s.getElementQueue().size());
    }

    @Test
    public void unclearColourIsReReadBeforeDeciding() {
        assertFalse(step(true, true, false, false, null, "RED")); // colour unclear on entry
        assertEquals(0, s.getElementQueue().size());             // not counted yet
        assertTrue(step(true, true, false, false, "BLUE", "RED")); // next read: opponent
        assertEquals(0, s.getElementQueue().size());
    }

    @Test
    public void debounceIgnoresBeamFlicker() {
        step(true, false, false, false, null, "RED");
        step(false, false, false, false, null, "RED");
        step(true, false, false, false, null, "RED"); // 40 ms later: flicker, same element
        assertEquals(1, s.getElementQueue().size());
    }

    @Test
    public void autoPreload() {
        s.preloadQueue(4, Storage.POLLEN);
        assertEquals(4, s.getElementQueue().size());
    }
}
