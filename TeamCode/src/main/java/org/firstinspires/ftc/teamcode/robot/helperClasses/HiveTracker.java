package org.firstinspires.ftc.teamcode.robot.helperClasses;

import org.firstinspires.ftc.teamcode.robot.helperClasses.FieldGeometry.Cell;

/**
 * Keeps track of which of our HIVE's two CELLS is facing up - the only one we may launch into
 * (G417). Every HIVE TIP flips it. Two independent ways to know it tipped:
 *
 * <ol>
 *   <li><b>Weight model (predicts OUR tips).</b> The Event Field Setup Guide (12) calibrates
 *   every hive to tip at 8 POLLEN, or 3 POLLEN + 3 NECTAR. So one NECTAR weighs about 5/3 of a
 *   POLLEN and a tip happens at 8 "pollen-equivalents". The upward cell starts the match
 *   holding 3 NECTAR (= 5), so <b>the 3rd POLLEN we launch tips it</b>. A cell that just
 *   turned upward was upside down, so it starts empty.</li>
 *   <li><b>Vision (catches our PARTNER's tips).</b> Each cell's AprilTags are on its bottom,
 *   facing down: the upward cell shows them to our camera, a tipped-down cell hides them. If we
 *   keep seeing the other cell's tags and not the target's, it tipped.</li>
 * </ol>
 *
 * <p>After a tip we hold fire for {@link #SETTLE_TIME} - the manual warns launching while a
 * hive is swinging can stop the TIP from counting. Drivers can also flip it by hand.</p>
 *
 * <p>Time is passed in so this can be unit tested.</p>
 */
public class HiveTracker {
    /** Other cell must be seen (target not seen) this long before vision believes a tip. */
    public static double CONFIRM_TIME = 0.4;
    /** Hold fire this long after a tip while the hive swings and settles. */
    public static double SETTLE_TIME = 0.6;
    /** Tip threshold in POLLEN-equivalents (8 POLLEN tips a calibrated hive). */
    public static double TIP_WEIGHT = 8.0;
    /** 3 POLLEN + 3 NECTAR also tips: 3 * NECTAR_WEIGHT = 8 - 3. */
    public static double NECTAR_WEIGHT = 5.0 / 3.0;
    /** Upward cell at match start holds 3 NECTAR (Competition Manual 10.3.1). */
    public static double START_WEIGHT = 3 * (5.0 / 3.0);

    private Cell upCell;
    private double weight = START_WEIGHT;
    private double otherSeenSince = -1;
    private double lastTipTime = -1e9;
    private int tips = 0;

    public HiveTracker(Cell startingUpCell) {
        upCell = startingUpCell;
    }

    /** Resume from a saved state (TeleOp continuing where auto left the hive). */
    public HiveTracker(Cell upCell, double weight) {
        this.upCell = upCell;
        this.weight = weight;
    }

    /**
     * Call once per loop with what the camera sees.
     *
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
            tip(now);
            return true;
        }
        return false;
    }

    /**
     * Call when WE launch an element into the up cell.
     *
     * @param nectar true for NECTAR (heavier), false for POLLEN / unknown
     * @return true if this shot should tip the hive (target switched)
     */
    public boolean recordShot(boolean nectar, double now) {
        weight += nectar ? NECTAR_WEIGHT : 1.0;
        if (weight >= TIP_WEIGHT - 1e-9) {
            tip(now);
            return true;
        }
        return false;
    }

    /** Driver override: "the hive just tipped". */
    public void flip() {
        tip(-1e9); // manual flips don't hold fire - the driver already saw it settle
    }

    /** True for SETTLE_TIME after a tip: don't launch while the hive is swinging. */
    public boolean isSettling(double now) {
        return now - lastTipTime < SETTLE_TIME;
    }

    private void tip(double now) {
        upCell = FieldGeometry.other(upCell);
        weight = 0; // the newly upward cell was upside down, so it's empty
        otherSeenSince = -1;
        lastTipTime = now;
        tips++;
    }

    public Cell getUpCell() {
        return upCell;
    }

    /** Estimated load in the up cell, in POLLEN-equivalents (tips at TIP_WEIGHT). */
    public double getWeight() {
        return weight;
    }

    /** Tips seen this OpMode (POLLINATOR RPs need 4 and 7 alliance tips). */
    public int getTips() {
        return tips;
    }
}
