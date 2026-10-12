package org.firstinspires.ftc.teamcode.robot.helperClasses;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.robot.helperClasses.FieldGeometry.Cell;
import org.junit.Test;

public class HiveTrackerTest {
    @Test
    public void thirdPreloadPollenTipsTheStartingCell() {
        // starting up cell holds 3 NECTAR; calibrated to tip at 3 POLLEN + 3 NECTAR
        HiveTracker t = new HiveTracker(Cell.AUDIENCE);
        assertFalse(t.recordShot(false, 1));
        assertFalse(t.recordShot(false, 2));
        assertTrue(t.recordShot(false, 3));
        assertEquals(Cell.FAR, t.getUpCell());
        assertEquals(0, t.getWeight(), 1e-9); // newly upward cell starts empty
    }

    @Test
    public void emptyCellTipsAtEightPollen() {
        HiveTracker t = new HiveTracker(Cell.FAR, 0);
        for (int i = 1; i < 8; i++) {
            assertFalse("tipped early at " + i, t.recordShot(false, i));
        }
        assertTrue(t.recordShot(false, 8));
    }

    @Test
    public void threePollenPlusThreeNectarTips() {
        HiveTracker t = new HiveTracker(Cell.FAR, 0);
        t.recordShot(true, 1);
        t.recordShot(true, 2);
        t.recordShot(true, 3);
        t.recordShot(false, 4);
        assertFalse(t.recordShot(false, 5));
        assertTrue(t.recordShot(false, 6));
    }

    @Test
    public void holdsFireWhileSettling() {
        HiveTracker t = new HiveTracker(Cell.AUDIENCE);
        t.recordShot(false, 1);
        t.recordShot(false, 2);
        t.recordShot(false, 10.0);
        assertTrue(t.isSettling(10.2));
        assertFalse(t.isSettling(10.0 + HiveTracker.SETTLE_TIME + 0.01));
    }

    @Test
    public void visionFlipsAfterOtherCellSeenLongEnough() {
        HiveTracker t = new HiveTracker(Cell.AUDIENCE);
        assertFalse(t.update(false, true, 0.0));
        assertFalse(t.update(false, true, 0.2));
        assertTrue(t.update(false, true, 0.5));
        assertEquals(Cell.FAR, t.getUpCell());
        assertEquals(1, t.getTips());
    }

    @Test
    public void glimpseOfOtherCellDoesNotFlip() {
        HiveTracker t = new HiveTracker(Cell.AUDIENCE);
        t.update(false, true, 0.0);
        t.update(true, false, 0.2);
        assertFalse(t.update(false, true, 0.5));
        assertEquals(Cell.AUDIENCE, t.getUpCell());
    }

    @Test
    public void manualFlipDoesNotHoldFire() {
        HiveTracker t = new HiveTracker(Cell.FAR);
        t.flip();
        assertEquals(Cell.AUDIENCE, t.getUpCell());
        assertFalse(t.isSettling(0));
    }
}
