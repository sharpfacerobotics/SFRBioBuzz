package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

/**
 * "Subordinate" auto: plays around an alliance partner who runs the main cycles. Owner: Alex.
 *
 * <p>Scores preloads from the close zone, takes ONLY the far spike mark (least likely to be
 * contested by our partner), scores it, then parks out of the partner's lanes.</p>
 *
 * <p>Poses are for BLUE in Pedro coordinates (inches, degrees). Red is mirrored automatically.
 * TODO(measure): every pose below is a placeholder until the BioBuzz field is measured.</p>
 */
public abstract class SubordinateAutoBase extends AutoBase {
    protected Pose startPose     = pose(56, 9, 90);
    protected Pose shootPose     = pose(56, 20, 90);
    protected Pose farSpikeLineUp = pose(48, 36, 180);
    protected Pose farSpikeEnd   = pose(18, 36, 180);
    protected Pose parkPose      = pose(36, 12, 90);

    @Override
    protected Pose startPose() {
        return startPose;
    }

    @Override
    protected void buildRoutine() {
        drive("to shoot zone", shootPose);
        shoot("preloads");

        drive("line up far spike", farSpikeLineUp);
        driveIntaking("intake far spike", farSpikeEnd);
        drive("back to shoot zone", farSpikeLineUp, shootPose);
        shoot("far spike");

        drive("park", parkPose);
    }
}
