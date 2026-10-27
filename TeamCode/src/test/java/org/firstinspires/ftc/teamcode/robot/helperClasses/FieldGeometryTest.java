package org.firstinspires.ftc.teamcode.robot.helperClasses;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.robot.helperClasses.FieldGeometry.Cell;
import org.junit.Test;

public class FieldGeometryTest {
    @Test
    public void startingUpCellsAreRotationsOfEachOther() {
        // red audience cell (up at start) must map onto blue far cell (up at start)
        double rx = FieldGeometry.cellX(true), ry = FieldGeometry.cellY(FieldGeometry.startingUpCell(true));
        double bx = FieldGeometry.cellX(false), by = FieldGeometry.cellY(FieldGeometry.startingUpCell(false));
        assertEquals(bx, FieldGeometry.rotateX(rx), 1e-9);
        assertEquals(by, FieldGeometry.rotateY(ry), 1e-9);
    }

    @Test
    public void hivesAreEitherSideOfCentre() {
        assertTrue(FieldGeometry.cellX(true) < 72);
        assertTrue(FieldGeometry.cellX(false) > 72);
        assertTrue(FieldGeometry.cellY(Cell.AUDIENCE) < 72);
        assertTrue(FieldGeometry.cellY(Cell.FAR) > 72);
    }

    @Test
    public void tagClustersMatchManualFigure917() {
        assertEquals(30, FieldGeometry.tagBase(true, Cell.FAR));
        assertEquals(34, FieldGeometry.tagBase(true, Cell.AUDIENCE));
        assertEquals(38, FieldGeometry.tagBase(false, Cell.AUDIENCE));
        assertEquals(42, FieldGeometry.tagBase(false, Cell.FAR));
        assertTrue(FieldGeometry.inCluster(33, 30));
        assertFalse(FieldGeometry.inCluster(34, 30));
    }

    @Test
    public void sides() {
        assertTrue(FieldGeometry.onOwnSide(true, 10));
        assertFalse(FieldGeometry.onOwnSide(true, 100));
        assertTrue(FieldGeometry.onOwnSide(false, 100));
    }

    @Test
    public void headingRotationWraps() {
        assertEquals(Math.PI, FieldGeometry.rotateHeading(0), 1e-9);
        assertEquals(-Math.PI / 2, FieldGeometry.rotateHeading(Math.PI / 2), 1e-9);
    }

    @Test
    public void tipStateIsRelativeToOurGarden() {
        // red GARDEN is on the audience wall, blue's on the far wall
        assertEquals(FieldGeometry.GARDEN_SIDE, FieldGeometry.tipState(true, Cell.AUDIENCE));
        assertEquals(FieldGeometry.GARDEN_SIDE, FieldGeometry.tipState(false, Cell.FAR));
        assertEquals(FieldGeometry.OPPOSITE_GARDEN, FieldGeometry.tipState(true, Cell.FAR));
        assertEquals(Cell.FAR, FieldGeometry.cellForTipState(true, FieldGeometry.OPPOSITE_GARDEN));
        // both alliances start with the garden-side cell up
        assertEquals(FieldGeometry.GARDEN_SIDE, FieldGeometry.tipState(true, FieldGeometry.startingUpCell(true)));
        assertEquals(FieldGeometry.GARDEN_SIDE, FieldGeometry.tipState(false, FieldGeometry.startingUpCell(false)));
    }

    @Test
    public void ftcFrameConversion() {
        // FTC origin = field centre
        assertEquals(72, FieldGeometry.ftcToFieldX(0, 0), 1e-9);
        assertEquals(72, FieldGeometry.ftcToFieldY(0, 0), 1e-9);
        // FTC +Y points away from the red wall = our +x
        assertEquals(72 + 10, FieldGeometry.ftcToFieldX(0, 10), 1e-9);
        // FTC +X is to the right seen from the red wall = toward the audience = our -y
        assertEquals(72 - 10, FieldGeometry.ftcToFieldY(10, 0), 1e-9);
        // facing FTC +Y (yaw 90 deg) = facing the blue wall = our heading 0
        assertEquals(0, FieldGeometry.ftcToFieldHeading(Math.PI / 2), 1e-9);
        // and the result is wrapped into (-pi, pi]
        assertEquals(Math.PI, FieldGeometry.ftcToFieldHeading(-Math.PI / 2), 1e-9);
    }
}
