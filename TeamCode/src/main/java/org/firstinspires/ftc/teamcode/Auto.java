package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Auto {

    public DcMotorEx leftFront, leftBack, rightFront, rightBack;
    public BNO055IMU imu;

    public LinearOpMode opMode;

    private double headingError = 0;
    private double targetHeading = 0;
    private double turnSpeed = 0;
    private double lastHeading = 0;
    private Orientation angles;

    private int leftFrontTarget = 0;
    private int leftBackTarget = 0;
    private int rightFrontTarget = 0;
    private int rightBackTarget = 0;

    static final double COUNTS_PER_MOTOR_REV = 318;
    static final double WHEEL_DIAMETER_CM = 9.6;
    static final double COUNTS_PER_CM = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_CM * Math.PI);

    static final double DRIVE_SPEED = 1.0;
    static final double HEADING_THRESHOLD = 2.0;  // Graus
    static final double P_TURN_GAIN = 0.01;      // Ganho proporcional para giro
    static final double P_DRIVE_GAIN = 0.03;     // Ganho proporcional para dirigir
    static final double MIN_TURN_SPEED = 0.08;    // Velocidade mínima de giro
    static final double MAX_TURN_SPEED = 0.6;     // Velocidade máxima de giro

    public Auto(HardwareMap hardwareMap, LinearOpMode opMode) {
        this.opMode = opMode;

        // Inicializa motores
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Ajusta direções dos motores
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);

        // Inicializa IMU goBILDA (BNO055)
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Aguarda calibração do IMU
        opMode.telemetry.addData("Status", "Calibrando IMU...");
        opMode.telemetry.update();

        while (!opMode.isStopRequested() && !imu.isGyroCalibrated()) {
            opMode.sleep(50);
            opMode.idle();
        }

        opMode.telemetry.addData("Status", "IMU Pronto!");
        opMode.telemetry.update();

        resetEncoders();
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

    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading,
                              double speedMultiplier,
                              Runnable parallelAction) {

        if (!opModeIsActive()) return;

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

        moveRobot(maxDriveSpeed, 0, 0, speedMultiplier);

        while (opModeIsActive() &&
                leftFront.isBusy() && leftBack.isBusy() &&
                rightFront.isBusy() && rightBack.isBusy()) {

            turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

            if (distance < 0) turnSpeed *= -1;

            moveRobot(DRIVE_SPEED, 0, turnSpeed, speedMultiplier);

            if (parallelAction != null) {
                parallelAction.run();
            }
        }

        moveRobot(0, 0, 0, 0);
        resetEncoders();
    }

    public void turnToHeading(double maxTurnSpeed, double heading, Runnable parallelAction) {

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        double currentHeading;
        int stableCount = 0;
        int requiredStableReadings = 4;

        getSteeringCorrection(heading, P_TURN_GAIN);

        while (opModeIsActive() && timer.seconds() < 3.0) {

            currentHeading = getHeading();
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Verifica se está dentro do threshold
            if (Math.abs(headingError) <= HEADING_THRESHOLD) {
                stableCount++;
                if (stableCount >= requiredStableReadings) {
                    break;  // Saiu do loop, atingiu o target
                }
            } else {
                stableCount = 0;
            }

            // Aplica velocidade mínima para vencer fricção
            if (Math.abs(turnSpeed) > 0.002) {
                if (turnSpeed > 0) {
                    turnSpeed = Math.max(turnSpeed, MIN_TURN_SPEED);
                } else {
                    turnSpeed = Math.min(turnSpeed, -MIN_TURN_SPEED);
                }
            }

            // Limita velocidade máxima
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Adiciona zona morta final
            if (Math.abs(headingError) < 1.0) {
                turnSpeed = 0;
            }

            moveRobot(0, 0, turnSpeed, 0.3);

            if (parallelAction != null) {
                parallelAction.run();
            }

            opMode.telemetry.addData("Target", heading);
            opMode.telemetry.addData("Current", currentHeading);
            opMode.telemetry.addData("Error", headingError);
            opMode.telemetry.addData("Turn Speed", turnSpeed);
            opMode.telemetry.addData("Stable Count", stableCount);
            opMode.telemetry.update();

            opMode.sleep(50);  // Loop delay para estabilidade
        }

        // Para completamente
        moveRobot(0, 0, 0, 0);
        opMode.sleep(100);  // Pausa para o robô parar completamente
    }

    public void strafe(double speed,
                       double distanceCm,
                       double speedMultiplier,
                       Runnable parallelAction) {

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

        moveRobot(0, speed, 0, speedMultiplier);

        while (opModeIsActive() &&
                leftFront.isBusy() && rightFront.isBusy() &&
                leftBack.isBusy() && rightBack.isBusy()) {

            if (parallelAction != null) {
                parallelAction.run();
            }
        }

        moveRobot(0, 0, 0, 0);
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
        double currentHeading = getHeading();

        headingError = targetHeading - currentHeading;

        // Normaliza o erro entre -180 e 180
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        return Range.clip(headingError * proportionalGain, -2, 2);
    }

    public double getHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.normalize(angles.firstAngle);
    }

    public void resetHeading() {
        lastHeading = getHeading();
    }

    public void sendTelemetry() {
        opMode.telemetry.addData("Heading", getHeading());
        opMode.telemetry.addData("Target", targetHeading);
        opMode.telemetry.addData("Error", headingError);
        opMode.telemetry.update();
    }
}