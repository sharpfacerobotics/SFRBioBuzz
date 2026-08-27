package org.firstinspires.ftc.teamcode.Pathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.states.Intake;
import org.firstinspires.ftc.teamcode.states.Shooter;

@Configurable
@Autonomous(name = "Blue 1 PedroPathing", group = "Autonomous")
public class Blue1PP extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    Intake intake = null;
    Shooter shooter = null;

    public enum PathState{
        START_POSITIONPOS_SHOOTPOS,

        SHOOTING,

        COLLECT_THE_FIRST_SEQUENCE_BALL,

        FORWARD_FIRST_SEQUENCE,

        OPENGATE,

        SHOOTING_FIRST_SEQUENCE_POS,

        SHOOTING_FIRST_SEQUENCE,

        COLLECT_THE_SECOND_SEQUENCE_BALL,

        FORWARD_SECOND_SEQUENCE,

        BACKWARD,

        SHOOTING_SECOND_SEQUENCE_BALL_POS,

        SHOOTING_SECOND_SEQUENCE_BALL,

        COLLECT_THE_THIRD_SEQUENCE_BALL,

        FORWARD_THIRD_SEQUENCE,

        SHOOTING_THIRD_SEQUENCE_BALL_POS,

        SHOOTING_THIRD_SEQUENCE_BALL,

        OUTLINERP,

        STOP,


    }


    PathState pathState;

    boolean actionStarted = false;

    private final Pose startPose = new Pose(16.685714285714287,130.28571428571428,Math.toRadians(140));

    private final Pose shootingPreSet = new Pose(37.20000000000001,109.42857142857142,Math.toRadians(140));

    private final Pose firstSequenceBallPos =  new Pose (35.42857142857142,92.6285714285714,Math.toRadians(179));

    private final Pose forwardToCollect =  new Pose (17.576,89.71428571428572,Math.toRadians(179));
    private final Pose openGate =  new Pose (11.256,77.25714285714284,Math.toRadians(80));

    private final Pose shootingFirstSequencePos =  new Pose (36.742857142857154,109.1142857142857,Math.toRadians(140));

    private final Pose secondSequenceBallPos =  new Pose (36.085714285714296,68.22857142857141,Math.toRadians(179));

    private final Pose forwardToCollectSecondSequence =  new Pose (10.371428571428564,68.74285714285712,Math.toRadians(173));

    private final Pose backwardToCollectSecondSequence =  new Pose (40.428571428571438,68.74285714285712,Math.toRadians(173));


    private final Pose shootingSecondSequencePose =  new Pose (36.742857142857154,109.1142857142857,Math.toRadians(140));

    private final Pose thirdSequenceBallPos =  new Pose (34.085714285714296,48.342857142857156,Math.toRadians(175));

    private final Pose forwardThirdSequenceBallPos =  new Pose (8.771428571428574,44.11428571428572,Math.toRadians(179));

    private final Pose shootingThirdSequenceBall =  new Pose (36.742857142857154,109.1142857142857,Math.toRadians(140));

    private final Pose outlineRP =  new Pose (52.114285714285714,130.65714285714284,Math.toRadians(179));



    private PathChain driveStartPosShoot,
            driveShootPosCollect1Ball,
            driveCollectFirstSequence,
            driveOpenGate,
            driveShootPos2,
            driveCollectSecondSequence,
            driveCollectingSecondSequence,
            driveShootingSecondSequence,
            driveBackwardSequence,

            driveCollectThirdSequence,

            driveForwardThirdSequence,

            driveShootingThirdSequence,

    driveOutLineRP
                    ;


    public void buildPaths(){
        driveStartPosShoot =follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootingPreSet))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootingPreSet.getHeading())
                .build();

        driveShootPosCollect1Ball = follower.pathBuilder()
                .addPath(new BezierLine(shootingPreSet,firstSequenceBallPos))
                .setLinearHeadingInterpolation(shootingPreSet.getHeading(), firstSequenceBallPos.getHeading())
                .build();

        driveCollectFirstSequence = follower.pathBuilder()
                .addPath(new BezierLine(firstSequenceBallPos,forwardToCollect))
                .setLinearHeadingInterpolation(firstSequenceBallPos.getHeading(),forwardToCollect.getHeading())
                .build();

        driveOpenGate = follower.pathBuilder()
                .addPath(new BezierLine(backwardToCollectSecondSequence,openGate))
                .setLinearHeadingInterpolation(backwardToCollectSecondSequence.getHeading(), openGate.getHeading())
                .build();

        driveShootPos2 = follower.pathBuilder()
                .addPath(new BezierLine(openGate,shootingFirstSequencePos))
                .setLinearHeadingInterpolation(openGate.getHeading(), shootingFirstSequencePos.getHeading())
                .build();

        driveCollectingSecondSequence = follower.pathBuilder()
                .addPath((new BezierLine(secondSequenceBallPos,forwardToCollectSecondSequence)))
                .setLinearHeadingInterpolation(secondSequenceBallPos.getHeading(), forwardToCollectSecondSequence.getHeading())
                .build();

        driveCollectSecondSequence = follower.pathBuilder()
                .addPath(new BezierLine(shootingFirstSequencePos,secondSequenceBallPos))
                .setLinearHeadingInterpolation(forwardToCollect.getHeading(), secondSequenceBallPos.getHeading())
                .build();

        driveBackwardSequence = follower.pathBuilder()
                .addPath((new BezierLine(forwardToCollectSecondSequence,backwardToCollectSecondSequence)))
                .setLinearHeadingInterpolation(forwardToCollectSecondSequence.getHeading(), backwardToCollectSecondSequence.getHeading())
                .build();


        driveShootingSecondSequence = follower.pathBuilder()
                .addPath((new BezierLine(backwardToCollectSecondSequence,shootingSecondSequencePose)))
                .setLinearHeadingInterpolation(backwardToCollectSecondSequence.getHeading(), shootingSecondSequencePose.getHeading())
                .build();


        driveCollectThirdSequence = follower.pathBuilder()
                .addPath(new BezierLine(shootingSecondSequencePose,thirdSequenceBallPos))
                .setLinearHeadingInterpolation(shootingSecondSequencePose.getHeading(),thirdSequenceBallPos.getHeading())
                .build();

        driveForwardThirdSequence = follower.pathBuilder()
                .addPath(new BezierLine(thirdSequenceBallPos,forwardThirdSequenceBallPos))
                .setLinearHeadingInterpolation(thirdSequenceBallPos.getHeading(), forwardToCollectSecondSequence.getHeading())
                .build();

        driveShootingThirdSequence = follower.pathBuilder()
                .addPath((new BezierLine(forwardThirdSequenceBallPos,shootingThirdSequenceBall)))
                .setLinearHeadingInterpolation(forwardThirdSequenceBallPos.getHeading(), shootingThirdSequenceBall.getHeading())
                .build();

        driveOutLineRP = follower.pathBuilder()
                .addPath((new BezierLine(shootingThirdSequenceBall,outlineRP)))
                .setLinearHeadingInterpolation(shootingThirdSequenceBall.getHeading(), outlineRP.getHeading())
                .build();


    }

    public void statePathUpdate(){
        switch(pathState){


            // PRE SET SHOOTING
            case START_POSITIONPOS_SHOOTPOS:
                follower.setMaxPower(1);
                follower.followPath(driveStartPosShoot,true);
                pathState = PathState.SHOOTING;
                break;
            case SHOOTING:
                follower.setMaxPower(1);
                if(!follower.isBusy() && !actionStarted){
                shooter.startChargingAuto(1200);
                actionStarted = true;
            }

            if(actionStarted && !shooter.isBusy()) {
                actionStarted = false;
                setPathState(PathState.COLLECT_THE_FIRST_SEQUENCE_BALL);
            }
            break;





            case COLLECT_THE_FIRST_SEQUENCE_BALL:
                follower.setMaxPower(1);
                if(!actionStarted){
                    follower.followPath(driveShootPosCollect1Ball,true);
                    actionStarted = true;
                }

                if(actionStarted && !follower.isBusy()){
                    shooter.startChargingAuto(800);
                    intake.starCollectBall();
                    actionStarted = false;
                    setPathState(PathState.FORWARD_FIRST_SEQUENCE);
                }

                break;



                //FIRST SEQUENCE OF BALLS
            case FORWARD_FIRST_SEQUENCE:
                follower.setMaxPower(0.5);

                if(!actionStarted){
                    follower.followPath(driveCollectFirstSequence,true);
                    actionStarted = true;
                }

                if(actionStarted && !follower.isBusy()){
                    intake.starCollectBall();
                    actionStarted = false;
                    setPathState(PathState.SHOOTING_FIRST_SEQUENCE_POS);
                }
                break;
            case SHOOTING_FIRST_SEQUENCE_POS:
                follower.setMaxPower(0.8);
                if(!actionStarted){
                    follower.followPath(driveShootPos2,true);
                    actionStarted = true;
                }

                if(actionStarted && !follower.isBusy()){
                    actionStarted = false;
                    setPathState(PathState.SHOOTING_FIRST_SEQUENCE);
                }
                break;

            case SHOOTING_FIRST_SEQUENCE:
                follower.setMaxPower(1);
                if(!follower.isBusy() && !actionStarted){
                    shooter.startChargingAuto(500);
                    actionStarted = true;
                }

                if(actionStarted && !shooter.isBusy()) {

                    if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                        actionStarted = false;
                        setPathState(PathState.COLLECT_THE_SECOND_SEQUENCE_BALL);
                    }
                }
                    break;


                // SECOND SEQUENCE OF BALL
            case COLLECT_THE_SECOND_SEQUENCE_BALL:
                follower.setMaxPower(1);

                if(!actionStarted){
                    follower.followPath(driveCollectSecondSequence,true);
                    actionStarted = true;
                }
                if(actionStarted && !follower.isBusy()){
                    intake.starCollectBall();
                    shooter.startChargingAuto(750);
                    actionStarted = false;
                    setPathState(PathState.FORWARD_SECOND_SEQUENCE);
                }
                break;

            case FORWARD_SECOND_SEQUENCE:
                follower.setMaxPower(0.5);

                if(!actionStarted){
                    follower.followPath(driveCollectingSecondSequence,true);
                    actionStarted = true;
                }
                if(actionStarted && !follower.isBusy()){
                    intake.starCollectBall();
                    actionStarted = false;
                    setPathState(PathState.BACKWARD);
                }
                break;

            case BACKWARD:
                follower.setMaxPower(1);

                if(!actionStarted){
                    follower.followPath(driveBackwardSequence,true);
                    actionStarted = true;
                }
                if(actionStarted && !follower.isBusy()){
                    actionStarted = false;
                    setPathState(PathState.OPENGATE); //SHOOTING_SECOND_SEQUENCE_BALL_POS
                }
                break;

            case OPENGATE:
                follower.setMaxPower(1);
                if (!actionStarted){
                    follower.followPath(driveOpenGate,true);
                    pathTimer.resetTimer();
                    actionStarted = true;}

                if(actionStarted && !follower.isBusy()){

                    if(pathTimer.getElapsedTimeSeconds() > 3) {
                        actionStarted = false;
                        setPathState(PathState.SHOOTING_SECOND_SEQUENCE_BALL_POS);
                    }
                }
                break;

            case SHOOTING_SECOND_SEQUENCE_BALL_POS:
                follower.setMaxPower(1);
                if(!actionStarted){
                    follower.followPath(driveShootingSecondSequence,true);
                    actionStarted = true;
                }

                if(actionStarted && !follower.isBusy()){
                    actionStarted = false;
                    setPathState(PathState.SHOOTING_SECOND_SEQUENCE_BALL);
                }
                break;
            case SHOOTING_SECOND_SEQUENCE_BALL:
                follower.setMaxPower(1);
                if(!follower.isBusy() && !actionStarted){
                    shooter.startChargingAuto(350);
                    actionStarted = true;
                }

                if(actionStarted && !shooter.isBusy()) {
                    actionStarted = false;
                    setPathState(PathState.COLLECT_THE_THIRD_SEQUENCE_BALL);
                }

                break;


                // THIRD SEQUENCE BALL

            case COLLECT_THE_THIRD_SEQUENCE_BALL:
                follower.setMaxPower(1);

                if(!actionStarted){
                    follower.followPath(driveCollectThirdSequence,true);
                    actionStarted = true;
                }
                if(actionStarted && !follower.isBusy()){
                    intake.starCollectBall();
                    shooter.startChargingAuto(800);
                    actionStarted = false;
                    setPathState(PathState.FORWARD_THIRD_SEQUENCE);
                }
                break;

            case FORWARD_THIRD_SEQUENCE:
                follower.setMaxPower(0.5);

                if(!actionStarted){
                    follower.followPath(driveForwardThirdSequence,true);
                    actionStarted = true;
                }
                if(actionStarted && !follower.isBusy()){
                    intake.starCollectBall();
                    actionStarted = false;
                    setPathState(PathState.SHOOTING_THIRD_SEQUENCE_BALL_POS);
                }
                break;

            case SHOOTING_THIRD_SEQUENCE_BALL_POS:
                follower.setMaxPower(1);

                if(!actionStarted){
                    follower.followPath(driveShootingThirdSequence,true);
                    actionStarted = true;
                }

                if(actionStarted && !follower.isBusy()){
                    actionStarted = false;
                    setPathState(PathState.SHOOTING_THIRD_SEQUENCE_BALL);
                }
                break;
            case SHOOTING_THIRD_SEQUENCE_BALL:

                if(!follower.isBusy() && !actionStarted){
                    shooter.startChargingAuto(350);
                    actionStarted = true;
                }

                if(actionStarted && !shooter.isBusy()) {
                    actionStarted = false;
                    setPathState(PathState.OUTLINERP);
                }

                break;

            case OUTLINERP:

                if(!actionStarted){
                    follower.followPath(driveOutLineRP,true);
                    actionStarted = true;
                }

                if(actionStarted && !follower.isBusy()) {
                    actionStarted = false;

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
    shooter.update();
    statePathUpdate();


        telemetry.addData("pathState",pathState.toString());
    telemetry.addData("X",follower.getPose().getX());
    telemetry.addData("y",follower.getPose().getY());
    telemetry.addData("heading",follower.getPose().getHeading());
    telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());


    }
}
