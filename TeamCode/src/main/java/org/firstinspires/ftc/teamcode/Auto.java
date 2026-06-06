package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto")
public class Auto extends CustomLinearOp {
    /**
     * Automatically runs after pressing the "Init" button on the Control Hub
     */
    @Override
    public void runOpMode() {
        super.runOpMode();

        // Drives forward 100.
        Pose2d startPose = new Pose2d(0.0, 0.0, 0.0);
        MECANUM_DRIVE.actionBuilder(startPose)
                     .lineToY(100)
                     .build();

        telemetry.update();
    }
}