package org.firstinspires.ftc.teamcode.robot.helperClasses;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.robot.helperClasses.FieldGeometry.Cell;
import org.junit.Test;

public class HiveTrackerTest {
    @Test
    public void flipsAfterOtherCellSeenLongEnough() {
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
        t.update(true, false, 0.2);  // target seen again -> reset
        assertFalse(t.update(false, true, 0.5));
        assertEquals(Cell.AUDIENCE, t.getUpCell());
    }

    @Test
    public void seeingBothMeansNoTip() {
        HiveTracker t = new HiveTracker(Cell.FAR);
        assertFalse(t.update(true, true, 0));
        assertFalse(t.update(true, true, 5));
        assertEquals(Cell.FAR, t.getUpCell());
    }

    @Test
    public void manualFlip() {
        HiveTracker t = new HiveTracker(Cell.FAR);
        t.flip();
        assertEquals(Cell.AUDIENCE, t.getUpCell());
    }
}
