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
    private static final double T = 1.0; // past the debounce

    private final Storage s = new Storage(null);

    /** One element passes the intake beams (rising then falling edge). */
    private boolean feed(boolean nectar, String colour, String alliance) {
        boolean e = s.process(true, nectar, false, false, nectar ? colour : null, alliance, T);
        s.process(false, false, false, false, null, alliance, T);
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
        assertTrue(s.process(true, true, false, false, "BLUE", "RED", T));
        assertEquals(0, s.getElementQueue().size());
        // still in the intake: keep asking for emergency outtake
        assertTrue(s.process(true, true, false, true, null, "RED", T));
        // beams clear: it's out
        assertFalse(s.process(false, false, false, true, null, "RED", T));
        assertEquals(0, s.getElementQueue().size());
    }

    @Test
    public void fifthElementIsEmergency() { // G407
        for (int i = 0; i < 4; i++) {
            assertFalse(feed(false, null, "BLUE"));
        }
        assertTrue(s.process(true, false, false, false, null, "BLUE", T));
        assertEquals(4, s.getElementQueue().size());
    }

    @Test
    public void exitBeamCountsElementsIntoTheLauncher() {
        feed(true, "BLUE", "BLUE");
        feed(false, null, "BLUE");
        s.process(false, false, true, false, null, "BLUE", T);  // exit beam broken
        s.process(false, false, false, false, null, "BLUE", T); // ...and cleared: one launched
        assertEquals(1, s.getElementQueue().size());
        assertEquals(Storage.POLLEN, s.getElementQueue().peek());
    }

    @Test
    public void outtakeRemovesTheLastElement() {
        feed(false, null, "RED");
        feed(true, "RED", "RED");
        s.process(true, true, false, true, null, "RED", T);   // nectar back at the beams, outtaking
        s.process(false, false, false, true, null, "RED", T); // and gone
        assertEquals(1, s.getElementQueue().size());
        assertEquals(Storage.POLLEN, s.getElementQueue().peek());
    }

    @Test
    public void debounceIgnoresBeamFlicker() {
        s.process(true, false, false, false, null, "RED", 0.0); // within debounce of start
        assertEquals(0, s.getElementQueue().size());
    }

    @Test
    public void autoPreload() {
        s.preloadQueue(4, Storage.POLLEN);
        assertEquals(4, s.getElementQueue().size());
    }
}
