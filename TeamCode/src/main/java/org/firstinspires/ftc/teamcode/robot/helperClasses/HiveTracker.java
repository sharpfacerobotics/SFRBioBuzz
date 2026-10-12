package org.firstinspires.ftc.teamcode.robot.helperClasses;

import org.firstinspires.ftc.teamcode.robot.helperClasses.FieldGeometry.Cell;

/**
 * Keeps track of which of our HIVE's two CELLS is facing up - the only one we may launch into
 * (G417). Every HIVE TIP flips it, and other robots tip it too, so we can't just count our shots.
 *
 * <p>How we tell: each cell's AprilTag cluster is on its bottom face, pointing down. The
 * upward cell's tags face the floor, where our camera can see them; a tipped-down cell's tags
 * point away. So if we keep seeing the OTHER cell's tags (and not the target's), the hive has
 * tipped. Drivers can also flip it by hand.</p>
 *
 * <p>Time is passed in so this can be unit tested.</p>
 */
public class HiveTracker {
    /** Other cell must be seen (target not seen) this long before we believe a tip. */
    public static double CONFIRM_TIME = 0.4;

    private Cell upCell;
    private double otherSeenSince = -1;
    private int tips = 0;

    public HiveTracker(Cell startingUpCell) {
        upCell = startingUpCell;
    }

    /**
     * @param seeTarget we can see the up cell's tag cluster
     * @param seeOther  we can see the other cell's tag cluster
     * @param now       time (s)
     * @return true on the loop a tip is detected
     */
    public boolean update(boolean seeTarget, boolean seeOther, double now) {
        if (seeTarget || !seeOther) {
            otherSeenSince = -1;
            return false;
        }
        if (otherSeenSince < 0) {
            otherSeenSince = now;
        }
        if (now - otherSeenSince >= CONFIRM_TIME) {
            flip();
            return true;
        }
        return false;
    }

    /** Driver override: "the hive just tipped". */
    public void flip() {
        upCell = FieldGeometry.other(upCell);
        otherSeenSince = -1;
        tips++;
    }

    public Cell getUpCell() {
        return upCell;
    }

    /** Tips seen this OpMode (POLLINATOR RPs need 4 and 7 alliance tips). */
    public int getTips() {
        return tips;
    }
}
