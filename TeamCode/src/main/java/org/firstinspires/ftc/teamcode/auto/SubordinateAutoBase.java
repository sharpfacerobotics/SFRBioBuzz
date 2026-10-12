package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

/**
 * "Subordinate" auto: stays out of a partner's way. Owner: Alex.
 *
 * <p>Starts on the far wall, scores the 4 POLLEN pre-loads into our upward CELL, and parks in
 * the LOADING ZONE - leaving every pickup on our half for the partner. Guaranteed LEAVE 3 +
 * PARK 5 + cell elements, and it helps toward a 20 pt TIP. Use the init start-delay menu to
 * wait until the partner has cleared the shooting spot.</p>
 *
 * <p>Poses are RED (see FieldPoses); blue is rotated automatically.</p>
 */
public abstract class SubordinateAutoBase extends AutoBase {
    @Override
    protected Pose startPose() {
        return FieldPoses.START_FAR_WALL;
    }

    @Override
    protected void buildRoutine() {
        drive("to shoot spot", FieldPoses.SHOOT_CENTER);
        shoot("pre-loads");

        drive("park in loading zone", FieldPoses.PARK);
    }
}
