package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class ParallelAutoOutsideBase extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(31.5677, 8.5936, 180);
    private final Pose path1 = poseFactory.of(7.9534, 8.5448, 180);
    private final Pose point2Start = poseFactory.of(7.9534, 8.5448, 180);
    private final Pose point2 = poseFactory.of(31.3937, 81.6325, 180);
    private final Pose point3 = poseFactory.of(47.7761, 121.431, 86.1928);
    private final Pose point3Control1 = poseFactory.of(47.5187, 116.5197, 0);
    private final Pose point4 = poseFactory.of(47.097, 129.0541, 95.0907);
    private final Pose point5Start = poseFactory.of(47.097, 129.0541, 90);
    private final Pose point5 = poseFactory.of(36.5373, 112.1269, 90);
    private final Pose point6 = poseFactory.of(17.2332, 47.0877, -174.6763);
    private final Pose point6Control1 = poseFactory.of(34.847, 48.4096, 0);
    private final Pose point7 = poseFactory.of(58.6493, 10.8507, 12.2797);
    private final Pose point7Control1 = poseFactory.of(16.3675, 1.419, 0);
    private final Pose point8 = poseFactory.of(12.1213, 95.0784, -61.0834);

    // Autonomous routine incorporating paths, parallel actions, and waits
    public Command autoRoutine() {
        return sequential(
                // Path 1: Shoot while intaking during path, then wait 500 ms
                parallel(
                        follow(follower, path1()),
                        parallel(shootCommand(), intakeCommand())
                ),
                waitMs(500),

                // Path 2: Intake running during path, 0 ms wait
                parallel(
                        follow(follower, path2()),
                        intakeCommand()
                ),

                // Path 3: Wait 1000 ms while deploying flower ramp
                follow(follower, path3()),
                race(
                        deployFlowerRampCommand(),
                        waitMs(1000)
                ),

                // Path 4: Keep flower ramp down, run intake, 1000 ms wait while shooting everything intook
                follow(follower, path4()),
                race(
                        parallel(
                                keepFlowerRampDownCommand(),
                                intakeCommand(),
                                shootCommand()
                        ),
                        waitMs(1000)
                ),

                // Path 5: Run intake during path with no wait
                parallel(
                        follow(follower, path5()),
                        intakeCommand()
                ),

                // Path 6: Bring flower ramp back up during a 400 ms wait
                follow(follower, path6()),
                race(
                        restoreFlowerRampCommand(),
                        waitMs(400)
                ),

                // Path 7: Run intake and keep shooter on during path, plus 500 ms wait
                parallel(
                        follow(follower, path7()),
                        parallel(intakeCommand(), shooterOnCommand())
                ),
                waitMs(500),

                // Path 8: Run intake during path and nothing else
                parallel(
                        follow(follower, path8()),
                        intakeCommand()
                )
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        if (follower == null) {
            telemetry.addLine("Constants.create() returned null - set up Pedro constants first");
            telemetry.update();
            waitForStart();
            return;
        }
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.curve(point2, point3Control1, point3).tangent();
    }

    public Path path4() {
        return Paths.line(point3, point4).tangent();
    }

    public Path path5() {
        return Paths.line(point5Start, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.curve(point5, point6Control1, point6).tangent();
    }

    public Path path7() {
        return Paths.curve(point6, point7Control1, point7).tangent();
    }

    public Path path8() {
        return Paths.line(point7, point8).reverseTangent();
    }

    /* ========================================================================
       SUBSYSTEM COMMAND PLACEHOLDERS
       Replace the telemetry lines with your actual motor/servo controls.
       ======================================================================== */

    private Command shootCommand() {
        return instant(() -> {
            telemetry.addLine("Action: Shooting...");
        });
    }

    private Command intakeCommand() {
        return instant(() -> {
            telemetry.addLine("Action: Running intake...");
        });
    }

    private Command deployFlowerRampCommand() {
        return instant(() -> {
            telemetry.addLine("Action: Deploying flower ramp...");
        });
    }

    private Command keepFlowerRampDownCommand() {
        return instant(() -> {
            telemetry.addLine("Action: Holding flower ramp down...");
        });
    }

    private Command restoreFlowerRampCommand() {
        return instant(() -> {
            telemetry.addLine("Action: Bringing flower ramp back up...");
        });
    }

    private Command shooterOnCommand() {
        return instant(() -> {
            telemetry.addLine("Action: Keeping shooter on...");
        });
    }
}