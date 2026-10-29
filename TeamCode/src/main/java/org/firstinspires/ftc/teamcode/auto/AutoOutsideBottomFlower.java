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
import static com.pedropathing.ivy.groups.Groups.*; // sequential, parallel, race, ...
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class AutoOutsideBottomFlower extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(55.472, 8.528, 180);
    private final Pose path1 = poseFactory.of(8.4813, 8.5448, 180);
    private final Pose point2Start = poseFactory.of(8.4813, 8.5448, 180);
    private final Pose point2 = poseFactory.of(33.2892, 113.5597, 180);
    private final Pose point3 = poseFactory.of(15.4571, 46.8414, 174.3949);
    private final Pose point3Control1 = poseFactory.of(37.6418, 44.6642, 0);
    private final Pose point4 = poseFactory.of(16.778, 116.334, -91.0889);

    // ------------------------------------------------------------------------
    // Requirements: any object can be a requirement. Swap each of these for
    // the real motor / servo / subsystem object once your hardware is wired in.
    // Two commands that share a requirement can't run at the same time.
    // ------------------------------------------------------------------------
    private static final Object SHOOTER = new Object();
    private static final Object RAMP = new Object();

    // Timing (ms)
    private static final long PRELOAD_SHOT_MS = 1000;   // before path 1: wait + shoot
    private static final long PATH_2_WAIT_MS = 2000;    // after path 2: wait + shoot + deploy ramp
    private static final long PATH_3_WAIT_MS = 1000;    // after path 3: wait only
    private static final long FINAL_SHOT_MS = 3000;     // after path 4: shoot, then stop

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                // Before path 1: wait 1000 ms, shooting
                race(shootCommand(), waitMs(PRELOAD_SHOT_MS)),

                follow(follower, path1()),

                follow(follower, path2()),
                // After path 2: wait 2000 ms, shooting and deploying the flower ramp
                race(
                        parallel(
                                shootCommand(),
                                deployFlowerRampCommand()
                        ),
                        waitMs(PATH_2_WAIT_MS)
                ),

                follow(follower, path3()),
                // After path 3: wait 1000 ms
                waitMs(PATH_3_WAIT_MS),

                follow(follower, path4()),
                // After path 4: shoot for 3000 ms, then stop (shootCommand's setEnd turns the shooter off)
                race(shootCommand(), waitMs(FINAL_SHOT_MS))
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
        return Paths.line(point3, point4).reverseTangent();
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

    /** CONTINUOUS: shooter runs until the surrounding race() cancels it. */
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