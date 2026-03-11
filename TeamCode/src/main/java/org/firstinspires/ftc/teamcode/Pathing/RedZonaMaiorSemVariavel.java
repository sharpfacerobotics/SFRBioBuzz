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
@Autonomous(name = "Red 1 PedroPathing", group = "Autonomous")
public class RedZonaMaiorSemVariavel extends OpMode {

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

        OUTLINE,

        STOP,


    }

    PathState pathState;

    boolean actionStarted = false;


    private final Pose startPose = new Pose(122.74285714285713,
            126.62857142857143,
            Math.toRadians(40));

    private final Pose shootingPreSet = new Pose(102.17142857142858,
            105.68571428571427,
            Math.toRadians(40));

    private final Pose firstSequenceBallPos =  new Pose (100,92.22857142857141,Math.toRadians(3));

    private final Pose forwardToCollect =  new Pose (124.34285714285716,90.22857142857141,Math.toRadians(3));
    private final Pose openGate =  new Pose (125.85714285714286,77.42857142857143,Math.toRadians(3));

    private final Pose shootingFirstSequencePos =  new Pose (100.17142857142858,105.68571428571427,Math.toRadians(40));

    private final Pose secondSequenceBallPos =  new Pose (100,70,Math.toRadians(3));

    private final Pose forwardToCollectSecondSequence =  new Pose (125,63,Math.toRadians(3));

    private final Pose backwardToCollectSecondSequence =  new Pose (110.97142857142858,67,Math.toRadians(3));

    private final Pose shootingSecondSequencePose =  new Pose (100.17142857142858,105.68571428571427,Math.toRadians(40));

    private final Pose thirdSequenceBallPos =  new Pose (103,42.485714285714295,Math.toRadians(1));

    private final Pose forwardThirdSequenceBallPos =  new Pose (130,42.485714285714295,Math.toRadians(1));

    private final Pose shootingThirdSequenceBall =  new Pose (100.22857142857143,105.68571428571427,Math.toRadians(40));

    private final Pose outlineRP =  new Pose (93.4857142857143,128.68571428571427,Math.toRadians(0));




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
            driveOutLineRp
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
                .setLinearHeadingInterpolation(shootingFirstSequencePos.getHeading(), secondSequenceBallPos.getHeading())
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

        driveOutLineRp = follower.pathBuilder()
                .addPath((new BezierLine(shootingThirdSequenceBall,outlineRP)))
                .setLinearHeadingInterpolation(shootingThirdSequenceBall.getHeading(), outlineRP.getHeading())
                .build();


    }

    public void statePathUpdate(){
        switch(pathState){


            // PRE SET SHOOTING
            case START_POSITIONPOS_SHOOTPOS:
                follower.followPath(driveStartPosShoot,true);
                pathState = PathState.SHOOTING;
                break;
            case SHOOTING:
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

                if(!actionStarted){
                    follower.followPath(driveShootPosCollect1Ball,true);
                    actionStarted = true;
                }

                if(actionStarted && !follower.isBusy()){
                    shooter.startChargingAuto(750);
                    intake.starCollectBall();
                    actionStarted = false;
                    setPathState(PathState.FORWARD_FIRST_SEQUENCE);
                }

                break;


            //FIRST SEQUENCE OF BALLS
            case FORWARD_FIRST_SEQUENCE:

                if(!actionStarted){
                    follower.setMaxPower(0.5);
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
                follower.setMaxPower(1);
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
                    shooter.startChargingAuto(750);
                    actionStarted = false;
                    setPathState(PathState.FORWARD_SECOND_SEQUENCE);
                }
                break;

            case FORWARD_SECOND_SEQUENCE:
                if(!actionStarted){
                    follower.setMaxPower(0.5);
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

                if(!actionStarted){
                    follower.setMaxPower(1);
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

                    if(pathTimer.getElapsedTimeSeconds() > 4.5) {
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
                    shooter.startChargingAuto(300);
                    actionStarted = true;
                }

                if(actionStarted && !shooter.isBusy()) {
                    actionStarted = false;
                    setPathState(PathState.COLLECT_THE_THIRD_SEQUENCE_BALL);
                }

                break;

            case COLLECT_THE_THIRD_SEQUENCE_BALL:
                follower.setMaxPower(1);
                if(!actionStarted){
                    follower.followPath(driveCollectThirdSequence,true);
                    actionStarted = true;
                }
                if(actionStarted && !follower.isBusy()){
                    intake.starCollectBall();
                    shooter.startChargingAuto(750);
                    actionStarted = false;
                    setPathState(PathState.FORWARD_THIRD_SEQUENCE);
                }
                break;

            case FORWARD_THIRD_SEQUENCE:
                if(!actionStarted){
                    follower.setMaxPower(0.5);
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
                follower.setMaxPower(1);
                if(!follower.isBusy() && !actionStarted){
                    shooter.startChargingAuto(350);
                    actionStarted = true;
                }

                if(actionStarted && !shooter.isBusy()) {
                    actionStarted = false;
                    setPathState(PathState.OUTLINE);
                }

                break;

            case OUTLINE:
                follower.setMaxPower(1);
                if(!actionStarted){
                    follower.followPath(driveOutLineRp,true);
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
