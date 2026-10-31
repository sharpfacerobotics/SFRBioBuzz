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
import static com.pedropathing.ivy.groups.Groups.*; // sequential, parallel, race, deadline, ...
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class AutoSolo extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(41.7444, 8.5448, 180);
    private final Pose path1 = poseFactory.of(7.4254, 8.5448, 180);
    private final Pose point2Start = poseFactory.of(7.4254, 8.5448, 180);
    private final Pose point2 = poseFactory.of(33.791, 113.1758, 180);
    private final Pose point3 = poseFactory.of(47.0243, 128.8153, 91.695);
    private final Pose point3Control1 = poseFactory.of(47.6175, 108.7668, 0);
    private final Pose point4Start = poseFactory.of(47.0243, 128.8153, 90);
    private final Pose point4 = poseFactory.of(41.1455, 117.7593, 180);
    private final Pose point5Start = poseFactory.of(41.1455, 117.7593, -148);
    private final Pose point5 = poseFactory.of(23.7929, 47.1679, 180);
    private final Pose point6 = poseFactory.of(13.4011, 47.1772, 179.9486);
    private final Pose point7 = poseFactory.of(23.9851, 46.9534, 178.7882);
    private final Pose point8 = poseFactory.of(59.569, 8.7705, 5.0713);
    private final Pose point8Control1 = poseFactory.of(21.4328, 5.3862, 0);
    private final Pose point9 = poseFactory.of(59.7369, 20.5224, 0);
    private final Pose point10 = poseFactory.of(59.1544, 128.1717, 90.3101);
    private final Pose point11 = poseFactory.of(15.6567, 120.6306, 9.8355);

    // ------------------------------------------------------------------------
    // Requirements: any object can be a requirement. Swap each of these for
    // the real motor / servo / subsystem object once your hardware is wired in.
    // Two commands that share a requirement can't run at the same time.
    // ------------------------------------------------------------------------
    private static final Object SHOOTER = new Object();
    private static final Object INTAKE = new Object();
    private static final Object RAMP = new Object();

    // ------------------------------------------------------------------------
    // Timing (ms). TODO: run the auto once and time how long path 8 and path 10
    // actually take, then put the real numbers here.
    // ------------------------------------------------------------------------
    private static final long FIRST_SHOT_MS = 1000;          // shoot for the first 1000 ms of path 1
    private static final long PATH_8_DURATION_MS = 2000;     // TODO: measure
    private static final long PATH_10_DURATION_MS = 2500;    // TODO: measure
    private static final long PATH_10_SHOOT_WINDOW_MS = 1300; // shoot during the last 1.3 s of path 10

    // Autonomous routine
    //
    // The whole sequence is the "deadline" of an outer deadline(), with
    // intakeCommand() as the other member. That means the intake starts when
    // path 1 starts, runs through every path, and is cancelled (and turned off
    // by its setEnd) when path 11 finishes.
    public Command autoRoutine() {
        return deadline(
                sequential(
                        // Path 1: shoot for the first 1000 ms of the path (intake runs the whole time).
                        // parallel() waits for both, so if the path is shorter than 1000 ms
                        // the robot waits at the end while the shot finishes.
                        parallel(
                                follow(follower, path1()),
                                race(shootCommand(), waitMs(FIRST_SHOT_MS))
                        ),

                        // Path 2, then deploy the flower ramp
                        follow(follower, path2()),
                        deployFlowerRampCommand(),

                        // Paths 3 and 4: shoot continuously across both paths
                        deadline(
                                sequential(
                                        follow(follower, path3()),
                                        follow(follower, path4())
                                ),
                                shootCommand()
                        ),

                        // Paths 5-7: intake only
                        follow(follower, path5()),
                        follow(follower, path6()),
                        follow(follower, path7()),

                        // Path 8: shoot during the first half only
                        // (deadline() so the shot can never outlast the path)
                        deadline(
                                follow(follower, path8()),
                                race(shootCommand(), waitMs(PATH_8_DURATION_MS / 2))
                        ),

                        // Path 9: intake only
                        follow(follower, path9()),

                        // Paths 10 and 11: start shooting 1.3 s before the end of path 10
                        // and keep shooting (no stop/restart) until path 11 ends
                        deadline(
                                sequential(
                                        follow(follower, path10()),
                                        follow(follower, path11())
                                ),
                                sequential(
                                        waitMs(Math.max(0, PATH_10_DURATION_MS - PATH_10_SHOOT_WINDOW_MS)),
                                        shootCommand()
                                )
                        )
                ),
                intakeCommand()
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
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
        return Paths.line(point4Start, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.line(point5Start, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point5, point6).tangent();
    }

    public Path path7() {
        return Paths.line(point6, point7).reverseTangent();
    }

    public Path path8() {
        return Paths.curve(point7, point8Control1, point8).tangent();
    }

    public Path path9() {
        return Paths.line(point8, point9).constant(point9);
    }

    public Path path10() {
        return Paths.line(point9, point10).tangent();
    }

    public Path path11() {
        return Paths.line(point10, point11).reverseTangent();
    }

    /* ========================================================================
       SUBSYSTEM COMMANDS (Ivy Command Builder API)

       CONTINUOUS  - runs until something else ends it (race / deadline / until).
                     setStart   -> turn the mechanism on
                     setExecute -> optional per-loop logic (PID, etc.)
                     setDone    -> () -> false  (never finishes on its own)
                     setEnd     -> turn the mechanism off (runs when cancelled)

       ONE-SHOT    - sets a state once and finishes right away (e.g. a servo move).
                     setStart   -> set the position / state
                     setDone    -> () -> true

       Replace the TODO comments with your motor/servo code.
       ======================================================================== */

    /** CONTINUOUS: intake runs until the surrounding deadline()/race() cancels it. */
    private Command intakeCommand() {
        return Command.build()
                .setStart(() -> {
                    // TODO: intakeMotor.setPower(1.0);
                })
                .setDone(() -> false)
                .setEnd(endCondition -> {
                    // TODO: intakeMotor.setPower(0);
                })
                .requiring(INTAKE);
    }

    /** CONTINUOUS: shooter runs until the surrounding deadline()/race() cancels it. */
    private Command shootCommand() {
        return Command.build()
                .setStart(() -> {
                    // TODO: spin up shooter flywheel(s)
                })
                .setExecute(() -> {
                    // TODO: per-loop shooter logic (velocity PID, feeding, etc.)
                })
                .setDone(() -> false)
                .setEnd(endCondition -> {
                    // TODO: stop shooter flywheel(s)
                })
                .requiring(SHOOTER);
    }

    /** ONE-SHOT: move the ramp servo to its deployed position. */
    private Command deployFlowerRampCommand() {
        return Command.build()
                .setStart(() -> {
                    // TODO: rampServo.setPosition(RAMP_DEPLOYED);
                })
                .setDone(() -> true)
                .requiring(RAMP);
    }
}