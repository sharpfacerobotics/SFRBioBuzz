package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Auto {

    public DcMotor leftFront, leftBack, rightFront, rightBack;
    public IMU imu;

    public LinearOpMode opMode;

    private double headingError = 0;
    private double targetHeading = 0;
    private double turnSpeed = 1;

    private int leftFrontTarget = 0;
    private int leftBackTarget = 0;
    private int rightFrontTarget = 0;
    private int rightBackTarget = 0;

    static final double COUNTS_PER_MOTOR_REV = 318;
    static final double WHEEL_DIAMETER_CM = 9.6;
    static final double COUNTS_PER_CM = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_CM * Math.PI);

    static final int DRIVE_SPEED = 1;
    static final double HEADING_THRESHOLD = 2;
    static final double P_TURN_GAIN = 0.01;
    static final double P_DRIVE_GAIN = 0.003;

    public Auto(HardwareMap hardwareMap, LinearOpMode opMode) {

        this.opMode = opMode;

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoDirection, usbDirection)));

        resetEncoders();

        imu.resetYaw();
    }

    private void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private boolean opModeIsActive() {
        return opMode.opModeIsActive();
    }

    public void driveStraight(double maxDriveSpeed, double distance, double heading, double SPEED_STRAIGHT) {

        if (opModeIsActive()) {

            int moveCounts = (int)(distance * COUNTS_PER_CM);

            leftFrontTarget  = leftFront.getCurrentPosition()  + moveCounts;
            leftBackTarget   = leftBack.getCurrentPosition()   + moveCounts;
            rightFrontTarget = rightFront.getCurrentPosition() + moveCounts;
            rightBackTarget  = rightBack.getCurrentPosition()  + moveCounts;

            leftFront.setTargetPosition(leftFrontTarget);
            leftBack.setTargetPosition(leftBackTarget);
            rightFront.setTargetPosition(rightFrontTarget);
            rightBack.setTargetPosition(rightBackTarget);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            moveRobot(maxDriveSpeed, 0, 0, SPEED_STRAIGHT);

            while (opModeIsActive() &&
                    leftFront.isBusy() && leftBack.isBusy() &&
                    rightFront.isBusy() && rightBack.isBusy()) {

                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                if (distance < 0) turnSpeed *= -1;

                moveRobot(DRIVE_SPEED, 0, turnSpeed, SPEED_STRAIGHT);
                //sendTelemetry(true);
            }

            moveRobot(0,0,0,0);
            resetEncoders();
        }
    }

    public void turnToHeading(double maxTurnSpeed, double heading, double TURN_SPEED) {

        while (opModeIsActive()) {

            headingError = heading - getHeading();
            while (headingError > 180) headingError -= 360;
            while (headingError <= -180) headingError += 360;

            if (Math.abs(headingError) <= HEADING_THRESHOLD)
                break;

            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            moveRobot(0, 0, turnSpeed, TURN_SPEED);
            //sendTelemetry(false);
        }

        moveRobot(0, 0, 0, 0);
    }

    public void strafe(double speed, double distanceCm, double STRAFE) {

        if (!opModeIsActive()) return;

        int moveCounts = (int)(distanceCm * COUNTS_PER_CM);

        leftFrontTarget  = leftFront.getCurrentPosition()  + moveCounts;
        leftBackTarget   = leftBack.getCurrentPosition()   - moveCounts;
        rightFrontTarget = rightFront.getCurrentPosition() - moveCounts;
        rightBackTarget  = rightBack.getCurrentPosition()  + moveCounts;

        leftFront.setTargetPosition(leftFrontTarget);
        leftBack.setTargetPosition(leftBackTarget);
        rightFront.setTargetPosition(rightFrontTarget);
        rightBack.setTargetPosition(rightBackTarget);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        moveRobot(0, speed, 0, STRAFE);

        while (opModeIsActive() &&
                leftFront.isBusy() && rightFront.isBusy() &&
                leftBack.isBusy() && rightBack.isBusy()) {

            //sendTelemetry(true);
        }

        moveRobot(0,0,0,0);
        resetEncoders();
    }

    public void moveRobot(double forward, double strafe, double rotate, double speed) {

        double leftFrontPower  = forward + strafe + rotate;
        double rightFrontPower = forward - strafe - rotate;
        double leftBackPower   = forward - strafe + rotate;
        double rightBackPower  = forward + strafe - rotate;

        double maxPower = Math.max(1.0,
                Math.max(Math.abs(leftFrontPower),
                        Math.max(Math.abs(rightFrontPower),
                                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)))));

        leftFront.setPower(speed * (leftFrontPower / maxPower));
        leftBack.setPower(speed * (leftBackPower / maxPower));
        rightFront.setPower(speed * (rightFrontPower / maxPower));
        rightBack.setPower(speed * (rightBackPower / maxPower));
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {

        targetHeading = desiredHeading;

        headingError = targetHeading - getHeading();
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void sendTelemetry(boolean straight) {

        if (straight) {
            opMode.telemetry.addData("Motion", "Drive Straight");
            opMode.telemetry.addData("Target Pos", "%7d | %7d", leftFrontTarget, rightFrontTarget);
            opMode.telemetry.addData("Actual Pos", "%7d | %7d",
                    leftFront.getCurrentPosition(), rightFront.getCurrentPosition());
        } else {
            opMode.telemetry.addData("Motion", "Turning");
        }

        opMode.telemetry.addData("Heading Target : Current", "%5.1f : %5.1f",
                targetHeading, getHeading());
        opMode.telemetry.addData("Error : Steer Pwr", "%5.1f : %5.1f", headingError, turnSpeed);
        opMode.telemetry.update();
    }
}
