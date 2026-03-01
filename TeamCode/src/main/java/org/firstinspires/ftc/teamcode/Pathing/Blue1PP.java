package org.firstinspires.ftc.teamcode.Pathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.states.Intake;
import org.firstinspires.ftc.teamcode.states.Shooter;

@TeleOp
public class Blue1PP extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    Intake intake = null;
    Shooter shooter = null;

    public enum PathState{
        START_POSITIONPOS_SHOOTPOS,

        SHOOTING,

        COLLECT_THE_FIRST_SEQUENCE_BALL,

        COLLETING,

        OPENGATE,

        SHOOTING2,

        STOP,


    }

    PathState pathState;

    boolean actionStarted = false;

    private final Pose startPose = new Pose(16.685714285714287,130.28571428571428,Math.toRadians(140));

    private final Pose shotingPose = new Pose(35.20000000000001,111.42857142857142,Math.toRadians(140));

    private final Pose firstSequenceBall =  new Pose (35.42857142857142,89.65714285714284,Math.toRadians(179));

    private final Pose forwardToCollect =  new Pose (15.314285714285724,89.71428571428572,Math.toRadians(179));
    private final Pose openGate =  new Pose (0.9142857142857268,78.74285714285713,Math.toRadians(90));

    private final Pose shootingPose2 =  new Pose (35.20000000000001,111.42857142857142,Math.toRadians(140));



    private PathChain driveStartPosShoot,driveShootPosCollect1Ball,driveCollectFirstSequence,driveOpenGate,driveShootPos2;


    public void buildPaths(){
        driveStartPosShoot =follower.pathBuilder()
                .addPath(new BezierLine(startPose,shotingPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shotingPose.getHeading())
                .build();

        driveShootPosCollect1Ball = follower.pathBuilder()
                .addPath(new BezierLine(shotingPose,firstSequenceBall))
                .setLinearHeadingInterpolation(shotingPose.getHeading(), firstSequenceBall.getHeading())
                .build();

        driveCollectFirstSequence = follower.pathBuilder()
                .addPath(new BezierLine(firstSequenceBall,forwardToCollect))
                .setLinearHeadingInterpolation(firstSequenceBall.getHeading(),forwardToCollect.getHeading())
                .build();

        driveOpenGate = follower.pathBuilder()
                .addPath(new BezierLine(forwardToCollect,openGate))
                .setLinearHeadingInterpolation(forwardToCollect.getHeading(), openGate.getHeading())
                .build();

        driveShootPos2 = follower.pathBuilder()
                .addPath(new BezierLine(openGate,shootingPose2))
                .setLinearHeadingInterpolation(openGate.getHeading(), shootingPose2.getHeading())
                .build();

    }

    public void statePathUpdate(){
        switch(pathState){
            case START_POSITIONPOS_SHOOTPOS:
                follower.followPath(driveStartPosShoot,true);
                pathState = PathState.SHOOTING;
                break;
            case SHOOTING:
            if(!follower.isBusy() && !actionStarted){
                shooter.startChargingAuto(1200);
                actionStarted = true;
            }

            if(actionStarted && !shooter.isBusy()){
                actionStarted = false;
                setPathState(PathState.COLLECT_THE_FIRST_SEQUENCE_BALL);
            }

            break;
            case COLLECT_THE_FIRST_SEQUENCE_BALL:

                if(!actionStarted){
                    follower.followPath(driveShootPosCollect1Ball,true);
                    actionStarted = true;
                }

                if(actionStarted && !follower.isBusy()){
                    shooter.startChargingAuto(950);
                    intake.starCollectBall();
                    actionStarted = false;
                    setPathState(PathState.COLLETING);
                }

                break;
            case COLLETING:

                if(!actionStarted){
                    follower.followPath(driveCollectFirstSequence,true);
                    actionStarted = true;
                }

                if(actionStarted && !follower.isBusy()){
                    intake.starCollectBall();
                    setPathState(PathState.OPENGATE);
                }
                break;
            case OPENGATE:
                if(!actionStarted){
                    follower.followPath(driveOpenGate,true);
                    actionStarted = true;
                }

                if(actionStarted && !follower.isBusy()){
                    actionStarted = false;
                    setPathState(PathState.SHOOTING2);
                }
                break;
            case SHOOTING2:
                if(!actionStarted){
                    follower.followPath(driveShootPos2,true);
                    actionStarted = true;
                }

                if(actionStarted && !follower.isBusy()){
                    actionStarted = false;
                    intake.stopCollectBall();
                    shooter.startChargingAuto(300);
                }
                if(!actionStarted && pathTimer.getElapsedTimeSeconds() > 0.4){
                    setPathState(PathState.STOP);
                }
                break;

            case STOP:
                follower.breakFollowing(); // Para qualquer path ativo
                intake.stopCollectBall();
                shooter.stop();
                break;








        }

        }

        public void setPathState(PathState newState){
        pathState = newState;
        pathTimer = new Timer();
        }



    @Override
    public void init(){
        pathState = PathState.START_POSITIONPOS_SHOOTPOS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        buildPaths();
        follower.setPose(startPose);




    }

    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop(){
    follower.update();
    statePathUpdate();
    shooter.update();


        telemetry.addData("pathState",pathState.toString());
    telemetry.addData("X",follower.getPose().getX());
    telemetry.addData("y",follower.getPose().getY());
    telemetry.addData("heading",follower.getPose().getHeading());
    telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());


    }
}
