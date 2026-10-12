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
}
