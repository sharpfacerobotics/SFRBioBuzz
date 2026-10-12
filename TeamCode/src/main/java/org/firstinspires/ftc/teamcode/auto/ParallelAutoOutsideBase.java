package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

/**
 * "Parallel" auto from the OUTSIDE start (far from our hive). Owner: Vivek.
 *
 * <p>Shoots preloads from the far zone, then works the far and middle spike marks so it
 * stays out of the way of a partner running the inside auto, then parks.</p>
 *
 * <p>Poses are for BLUE in Pedro coordinates (inches, degrees). Red is mirrored automatically.
 * TODO(measure): every pose below is a placeholder until the BioBuzz field is measured.</p>
 */
public abstract class ParallelAutoOutsideBase extends AutoBase {
    protected Pose startPose       = pose(56, 9, 90);
    protected Pose farShootPose    = pose(56, 20, 90);
    protected Pose farSpikeLineUp  = pose(48, 36, 180);
    protected Pose farSpikeEnd     = pose(18, 36, 180);
    protected Pose midSpikeLineUp  = pose(48, 60, 180);
    protected Pose midSpikeEnd     = pose(18, 60, 180);
    protected Pose midReturnControl = pose(50, 40, 180);
    protected Pose parkPose        = pose(36, 12, 90);

    @Override
    protected Pose startPose() {
        return startPose;
    }

    @Override
    protected void buildRoutine() {
        drive("to far shoot zone", farShootPose);
        shoot("preloads");

        drive("line up far spike", farSpikeLineUp);
        driveIntaking("intake far spike", farSpikeEnd);
        drive("back to far shoot zone", farShootPose);
        shoot("far spike");

        drive("line up middle spike", midSpikeLineUp);
        driveIntaking("intake middle spike", midSpikeEnd);
        drive("back to far shoot zone", midReturnControl, farShootPose);
        shoot("middle spike");

        drive("park", parkPose);
    }
}
