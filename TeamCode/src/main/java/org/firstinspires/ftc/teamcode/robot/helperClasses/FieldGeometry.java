package org.firstinspires.ftc.teamcode.robot.helperClasses;

/**
 * BIOBUZZ field layout (Competition Manual sections 9 and 10).
 *
 * <p><b>Our field frame</b> (what Pedro uses for us): inches, origin at the corner where the
 * <b>audience wall</b> meets the <b>red alliance wall</b>. +x points toward the blue alliance
 * wall, +y points away from the audience. Heading 0 = facing the blue wall, CCW positive.</p>
 *
 * <pre>
 *            far wall (y = 144)
 *   +---------F------------------G(blue)+
 *   |                                   |
 *  L(red)       [R]   [B]  <- far cells F
 *   |           HIVE frame              |
 *   F           [R]   [B]  <- audience cells
 *   |                                L(blue)
 *   +G(red)------------F----------------+
 *            audience wall (y = 0)
 *   red wall x = 0                  blue wall x = 144
 *   F = FLOWER, G = GARDEN, L = LOADING ZONE
 * </pre>
 *
 * <p>The field is <b>180-degree rotationally symmetric</b>: every red element maps onto its
 * blue counterpart by (x, y, h) -> (144 - x, 144 - y, h + pi). It is NOT mirrored.</p>
 *
 * <p>Numbers measured from the manual's figures are +/- ~2 in. TODO(measure) on a real field
 * or the official CAD before trusting them to the inch.</p>
 */
public final class FieldGeometry {
    private FieldGeometry() {
    }

    public static final double SIZE = 144.0;
    public static final double CENTER = SIZE / 2;

    // ------------------------------------------------------------------ HIVE (red values)
    /** x of the red HIVE's cells (the blue HIVE is the rotation, x = 144 - this). */
    public static double RED_HIVE_X = 59.0;
    /** Cells sit this far either side of field centre along y. Manual: cells ~18.8 in apart. */
    public static double CELL_OFFSET_Y = 11.0;

    /** Which of an alliance's two cells: the one nearer the audience or the far one. */
    public enum Cell { AUDIENCE, FAR }

    /** At match start the red AUDIENCE cell and the blue FAR cell face up (Figure 10-2). */
    public static Cell startingUpCell(boolean red) {
        return red ? Cell.AUDIENCE : Cell.FAR;
    }

    public static Cell other(Cell cell) {
        return cell == Cell.AUDIENCE ? Cell.FAR : Cell.AUDIENCE;
    }

    public static double cellX(boolean red) {
        return red ? RED_HIVE_X : SIZE - RED_HIVE_X;
    }

    public static double cellY(Cell cell) {
        return cell == Cell.AUDIENCE ? CENTER - CELL_OFFSET_Y : CENTER + CELL_OFFSET_Y;
    }

    // ------------------------------------------------------------------ AprilTags (36h11, 3.25 in)
    // Four tags per cell, on the cell's BOTTOM face, facing down (Figure 9-17).
    public static final int RED_FAR_TAGS = 30;       // 30-33
    public static final int RED_AUDIENCE_TAGS = 34;  // 34-37
    public static final int BLUE_AUDIENCE_TAGS = 38; // 38-41
    public static final int BLUE_FAR_TAGS = 42;      // 42-45
    public static final int TAGS_PER_CELL = 4;

    /** First tag ID of the cluster under this cell (the cluster is first..first+3). */
    public static int tagBase(boolean red, Cell cell) {
        if (red) {
            return cell == Cell.FAR ? RED_FAR_TAGS : RED_AUDIENCE_TAGS;
        }
        return cell == Cell.FAR ? BLUE_FAR_TAGS : BLUE_AUDIENCE_TAGS;
    }

    public static boolean inCluster(int id, int base) {
        return id >= base && id < base + TAGS_PER_CELL;
    }

    // ------------------------------------------------------------------ zones (red values)
    // Tile grid (Event Field Setup Guide 8.3/8.4): columns A-F run from the red wall (x) and
    // rows 1-6 from the audience wall (y), 24 in each. Red LOADING ZONE = tile A5, red
    // GARDEN = tile A1; blue's are F2 and F6 - exactly the 180 degree rotation.
    /** Red LOADING ZONE (tile A5: 11 in deep from the red wall along the A5 seams): PARK here. */
    public static double RED_LOADING_ZONE_X = 5.5;   // centre; zone spans x 0..11
    public static double RED_LOADING_ZONE_Y = 108.0; // centre; zone spans y 96..120 (tile seams)
    /** Red GARDEN: tile A1, strip along the audience wall from the red corner, x 0..23. */
    public static double RED_GARDEN_X = 11.5;
    public static double RED_GARDEN_Y = 1.0;
    /** The two FLOWERS on red's side (each starts with 4 POLLEN to retrieve from the bottom). */
    public static double RED_WALL_FLOWER_Y = 48.0;   // on the red wall, x = 0
    public static double FAR_WALL_FLOWER_X = 48.0;   // on the far wall, y = 144

    /** True if (x, y) is on this alliance's half (G304/G402: red = columns A-C = x < 72). */
    public static boolean onOwnSide(boolean red, double x) {
        return red ? x < CENTER : x > CENTER;
    }

    /** Red -> blue (or blue -> red): 180 degree rotation about field centre. */
    public static double rotateX(double x) {
        return SIZE - x;
    }

    public static double rotateY(double y) {
        return SIZE - y;
    }

    public static double rotateHeading(double heading) {
        double h = heading + Math.PI;
        while (h > Math.PI) h -= 2 * Math.PI;
        while (h <= -Math.PI) h += 2 * Math.PI;
        return h;
    }
}
