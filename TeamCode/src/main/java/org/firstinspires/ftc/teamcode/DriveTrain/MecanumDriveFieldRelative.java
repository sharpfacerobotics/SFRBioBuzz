package org.firstinspires.ftc.teamcode.DriveTrain;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDriveFieldRelative {

    private DcMotor leftFront, rightFront, leftBack, rightBack;
    public IMU imu;

    public double maxSpeedDefault = 0.8;


    public void init(HardwareMap hardwareMap) {

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot revHubOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );

        imu.initialize(new IMU.Parameters(revHubOrientation));
    }


    public void drive(double forward, double strafe, double rotate) {

        double leftFrontPower = forward + strafe + rotate;
        double leftBackPower = forward - strafe + rotate;
        double rightFrontPower = forward - strafe - rotate;
        double rightBackPower = forward + strafe - rotate;

        double maxPower = 1.0;
        double limit = this.maxSpeedDefault;

        maxPower = Math.max(maxPower, Math.abs(leftFrontPower));
        maxPower = Math.max(maxPower, Math.abs(leftBackPower));
        maxPower = Math.max(maxPower, Math.abs(rightFrontPower));
        maxPower = Math.max(maxPower, Math.abs(rightBackPower));

        leftFront.setPower(limit * (leftFrontPower / maxPower));
        leftBack.setPower(limit * (leftBackPower / maxPower));
        rightFront.setPower(limit * (rightFrontPower / maxPower));
        rightBack.setPower(limit * (rightBackPower / maxPower));
    }


   public void driveFieldRelative(double forward, double strafe, double rotate) {

        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(
                theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
        );

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        drive(newForward, newStrafe, rotate);
    }

    public void setMaxSpeed(double speed) {
        this.maxSpeedDefault = speed;
    }
}