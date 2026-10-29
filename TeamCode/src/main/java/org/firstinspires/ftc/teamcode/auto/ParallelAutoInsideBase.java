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
import static com.pedropathing.ivy.groups.Groups.*; // sequential, parallel, race, repeat, ...
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.pedro.Constants;


@Autonomous(name = "AutoPath", group = "Autonomous")
public class ParallelAutoInsideBase extends LinearOpMode {


    private Follower follower;


    private final PoseFactory poseFactory = PoseFactory.degrees();


    private final Pose start = poseFactory.of(54.944, 7.472, 90);
    private final Pose path1 = poseFactory.of(55.472, 16.4646, 90);
    private final Pose point2Start = poseFactory.of(55.472, 16.4646, 90);
    private final Pose point2 = poseFactory.of(55.1922, 8.4888, 90);
    private final Pose point3Start = poseFactory.of(55.1922, 8.4888, 90);
    private final Pose point3 = poseFactory.of(47.7826, 111.7678, 90);
    private final Pose point3Control1 = poseFactory.of(64.4328, 118.791, 0);
    private final Pose point4Start = poseFactory.of(47.7826, 111.7678, 90);
    private final Pose point4 = poseFactory.of(47.2547, 130.7753, 90);
    private final Pose point5Start = poseFactory.of(47.2547, 130.7753, 90);
    private final Pose point5 = poseFactory.of(47.2547, 123.3835, 90);
    private final Pose point6Start = poseFactory.of(47.2547, 123.3835, 90);
    private final Pose point6 = poseFactory.of(14.6903, 106.1194, 90);


    // ------------------------------------------------------------------------
    // Requirements: any object can be a requirement. Swap each of these for
    // the real motor / servo / subsystem object once your hardware is wired in.
    // Two commands that share a requirement can't run at the same time.
    // ------------------------------------------------------------------------
    private static final Object SHOOTER = new Object();
    private static final Object FEEDER = new Object();
    private static final Object INTAKE = new Object();
    private static final Object RAMP = new Object();


    // Tunable timings (ms)
    private static final long SPIN_UP_MS = 400;     // time for the shooter to reach speed
    private static final long FEED_MS = 120;        // feeder extended time per pollen
    private static final long FEED_RESET_MS = 120;  // feeder retract time per pollen


    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                // Path 1 -> Shoot for 1000ms
                follow(follower, path1()),
                race(
                        shootCommand(),
                        waitMs(1000)
                ),


                // Path 2 -> Wait 3500ms for hive to tip
                follow(follower, path2()),
                waitMs(3500),


                // Path 3 -> Shoot & Deploy flower ramp for 1000ms simultaneously
                follow(follower, path3()),
                race(
                        parallel(
                                shootCommand(),
                                deployFlowerRampCommand()
                        ),
                        waitMs(1000)
                ),


                // Path 4 -> Collect pollen for 1000ms
                follow(follower, path4()),
                race(
                        collectPollenCommand(),
                        waitMs(1000)
                ),


                // Path 5 -> Restore flower ramp *during* path execution
                parallel(
                        follow(follower, path5()),
                        restoreFlowerRampCommand()
                ),


                // Path 6 -> Shoot last 4 pollen *during* path execution
                parallel(
                        follow(follower, path6()),
                        shootLastPollenCommand()
                )
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
        return Paths.curve(point3Start, point3Control1, point3).linear(point3Start, point3);
    }


    public Path path4() {
        return Paths.line(point4Start, point4).linear(point4Start, point4);
    }


    public Path path5() {
        return Paths.line(point5Start, point5).linear(point5Start, point5);
    }


    public Path path6() {
        return Paths.line(point6Start, point6).linear(point6Start, point6);
    }


    /* ========================================================================
       SUBSYSTEM COMMANDS (Ivy Command Builder API)

       Two templates are used below:

       CONTINUOUS  - runs until something else ends it (race / deadline / until).
                     setStart   -> turn the mechanism on
                     setExecute -> optional per-loop logic (PID, etc.)
                     setDone    -> () -> false  (never finishes on its own)
                     setEnd     -> turn the mechanism off (runs when cancelled)

       ONE-SHOT    - sets a state once and finishes right away (e.g. a servo move).
                     setStart   -> set the position
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
                    // TODO: per-loop shooter logic (velocity PID, etc.)
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


    /** CONTINUOUS: intake runs until the surrounding race() cancels it. */
    private Command collectPollenCommand() {
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


    /** ONE-SHOT: move the ramp servo back to its stowed position. */
    private Command restoreFlowerRampCommand() {
        return Command.build()
                .setStart(() -> {
                    // TODO: rampServo.setPosition(RAMP_STOWED);
                })
                .setDone(() -> true)
                .requiring(RAMP);
    }


    /**
     * COMPOSITION: spin up the shooter, feed 4 pollen one at a time, then stop.
     * The race ends as soon as the 4th feed cycle finishes, which cancels
     * shootCommand() and triggers its setEnd (shooter off).
     */
    private Command shootLastPollenCommand() {
        return race(
                shootCommand(),
                sequential(
                        waitMs(SPIN_UP_MS),
                        repeat(feedOnePollenCommand(), 4)
                )
        );
    }


    /** One feed cycle: extend feeder, wait, retract feeder, wait. */
    private Command feedOnePollenCommand() {
        return sequential(
                Command.build()
                        .setStart(() -> {
                            // TODO: feederServo.setPosition(FEED_EXTENDED);
                        })
                        .setDone(() -> true)
                        .requiring(FEEDER),
                waitMs(FEED_MS),
                Command.build()
                        .setStart(() -> {
                            // TODO: feederServo.setPosition(FEED_RETRACTED);
                        })
                        .setDone(() -> true)
                        .requiring(FEEDER),
                waitMs(FEED_RESET_MS)
        );
    }
}