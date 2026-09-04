package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

// MUST HAVE MOTOR AND SENSOR IN RobotHardware
public class LinearExtension {
    private RobotHardware robot;

    private int MIN_TICKS = 0;
    private int MAX_TICKS = 3000; // do live changes, may vary across different extensions

    private double nudgePower = 0.5;
    private double extensionPower = 0.9;
    private DcMotorEx extensionMotor;
    private TouchSensor lowerLimitSwitch;
    private int targetExtensionTicks = 0;
    private boolean wasAtLowerLimit = false;

    public LinearExtension(RobotHardware robot, int MIN_TICKS, int MAX_TICKS, double nudgePower, double extensionPower) {
        this.robot = robot;
        this.MIN_TICKS = MIN_TICKS;
        this.MAX_TICKS = MAX_TICKS;
        this.nudgePower = nudgePower;
        this.extensionPower = extensionPower;

        extensionMotor = robot.extensionMotor;
        lowerLimitSwitch = robot.lowerLimitSwitch;

        extensionMotor.setDirection(DcMotorEx.Direction.FORWARD); //Change accordingly
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetExtensionMotor(extensionMotor);
    }

    public void update() {
        boolean atLowerLimit = lowerLimitSwitch.isPressed();
        if (atLowerLimit && !wasAtLowerLimit) {
            resetExtensionMotor(extensionMotor);
        }
        wasAtLowerLimit = atLowerLimit;

    }

    private void resetExtensionMotor(DcMotorEx extensionMotor) {
        extensionMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setTargetPosition(0);
        extensionMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void setTargetExtensionTicks(int ticks) {
        targetExtensionTicks = Range.clip(ticks, MIN_TICKS, MAX_TICKS);

        extensionMotor.setTargetPosition(targetExtensionTicks);
        if (extensionMotor.getMode() != DcMotorEx.RunMode.RUN_TO_POSITION) {
            extensionMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }
        extensionMotor.setPower(extensionPower);
    }

    public void setTargetExtensionTicks(int ticks, double extensionPower) {
        targetExtensionTicks = Range.clip(ticks, MIN_TICKS, MAX_TICKS);

        extensionMotor.setTargetPosition(targetExtensionTicks);
        if (extensionMotor.getMode() != DcMotorEx.RunMode.RUN_TO_POSITION) {
            extensionMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }
        extensionMotor.setPower(Math.abs(clipPower(extensionPower, 1.00)));
    }

    public void setTargetExtensionPercent(int percent) {
        targetExtensionTicks = (int) Range.scale(percent, 0, 100, MIN_TICKS, MAX_TICKS);
        setTargetExtensionTicks(targetExtensionTicks);
    }

    public boolean isBusy() { return extensionMotor.isBusy();}
    public int getCurrentPositionTicks() { return extensionMotor.getCurrentPosition();}
    public int getCurrentPositionPercentage() {
        return (int) Range.clip(Range.scale(getCurrentPositionTicks(), MIN_TICKS, MAX_TICKS, 0, 100), 0, 100);
    }
    public int getTargetTicks() { return targetExtensionTicks;}
    public boolean lowerLimitHit() { return lowerLimitSwitch.isPressed();}

    public void setNudgePower(double requestedPower) {
        if (extensionMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // USING or WITHOUT? Test to ensure
        }

        extensionMotor.setPower(Math.abs(clipPower(requestedPower, nudgePower)));
    }

    private double clipPower(double requestedPower, double limitMag) {
        double clippedPower = Range.clip(requestedPower, -limitMag, limitMag);

        if (clippedPower < 0 && getCurrentPositionTicks() <= MIN_TICKS) {
            clippedPower = 0.0;
        } else if (clippedPower > 0 && getCurrentPositionTicks() >= MAX_TICKS) {
            clippedPower = 0.0;
        }
        return clippedPower;
    }
}
