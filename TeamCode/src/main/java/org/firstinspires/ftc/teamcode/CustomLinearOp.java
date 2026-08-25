package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardwaresystems.Arm;
import org.firstinspires.ftc.teamcode.hardwaresystems.Claw;
import org.firstinspires.ftc.teamcode.hardwaresystems.MecanumWheels;
import org.firstinspires.ftc.teamcode.hardwaresystems.Webcam;
import org.firstinspires.ftc.teamcode.hardwaresystems.Wheels;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/**
 * A custom linear opmode to be used as a basis for {@link Auto} and
 * {@link DriverMode}.
 */
public class CustomLinearOp extends LinearOpMode {
    /* Robot systems */

    /**
     * Whether the robot will automatically sleep after each command.
     */
    protected boolean autoSleepEnabled;

    /**
     * Use for our own system.
     * TODO: By default, the class is set to {@link Wheels}. Replace as
     *  necessary.
     */
    protected Wheels WHEELS;

    /**
     * Use for RoadRunner.
     * TODO: By default, the class is set to {@link MecanumDrive}. Replace as
     *  necessary.
     */
    protected MecanumDrive MECANUM_DRIVE;

    /**
     * The arm used by the robot.
     * TODO: By default, the type is set to {@link Arm}. Replace or delete as
     *  necessary.
     */
    protected Arm ARM;

    /**
     * The claw used by the robot.
     * TODO: By default, the type is set to {@link Claw}. Replace or delete
     *  as necessary.
     */
    protected Claw CLAW;

    /**
     * The webcam used by the robot.
     */
    protected Webcam WEBCAM;

    /**
     * The AprilTag ID for the red alliance.
     * TODO: Replace with your real AprilTag IDs for this season.
     */
    protected Set<Integer> RED_APRILTAG_IDS = Set.of(-1);
    /**
     * The AprilTag ID for the red alliance.
     * TODO: Replace with your real AprilTag IDs for this season.
     */
    protected Set<Integer> BLUE_APRILTAG_IDS = Set.of(-1);

    /**
     * Store the data required to run {@link Auto}.
     */
    protected AutoConfigurator.AutoConfig AUTO_CONFIG;

    public Set<DcMotor> getAllDcMotors() {
        HashSet<DcMotor> motors = new HashSet<>();
        // hardware.dcMotor stores all the DcMotors as name-device pairs.
        for (Map.Entry<String, DcMotor> ele : hardwareMap.dcMotor.entrySet()) {
            motors.add(ele.getValue());
        }

        return motors;
    }

    /**
     * Get all {@link CRServo}s if they are present.
     *
     * @return A {@link Set} containing all the {@link CRServo}s used by this
     * robot.
     */
    public Set<CRServo> getAllCrServos() {
        HashSet<CRServo> crServos = new HashSet<>();
        // hardwareMap.crservo stores all the CRServos as name-device pairs.
        for (Map.Entry<String, CRServo> hardwareDevice :
            hardwareMap.crservo.entrySet()) {
            crServos.add(hardwareDevice.getValue());
        }

        return crServos;
    }

    /**
     * Get all the names in the {@link HardwareMap} that are not connected to a
     * device.
     * <p>
     * TODO: <em><strong>THIS METHOD IS NOT WORKING CURRENTLY!!!</strong></em>
     *
     * @return A {@link Set} of all the hardware devices that can not be found.
     */
    public Set<String> getMissingHardwareDevices() {
        HashSet<String> missingHardwareDevices = new HashSet<>();

        // Loop through each DeviceMapping (e.g., Servos and DcMotors).
        for (HardwareMap.DeviceMapping<? extends HardwareDevice> deviceMapping : hardwareMap.allDeviceMappings) {
            // Check if each device in the mapping is null.
            for (Map.Entry<String, ? extends HardwareDevice> hardwareDevice :
                deviceMapping.entrySet()) {
                if (hardwareDevice.getValue() == null) {
                    missingHardwareDevices.add(hardwareDevice.getKey());
                }
            }
        }

        return missingHardwareDevices;
    }

    /**
     * Try to retrieve a {@link DcMotor} from the hardware map using one of
     * several candidate names. This helper makes the drive train code resilient
     * to different naming conventions in the Robot Controller configuration. It
     * iterates through the provided names and returns the first motor that
     * exists. If none of the names are present, an
     * {@link IllegalArgumentException} is thrown.
     *
     * @deprecated
     * The motor names should be known. Do not query for the motor.
     *
     * @param candidates One or more hardware device names to try.
     * @return The {@link DcMotor} associated with the first name found.
     * @throws IllegalArgumentException if no candidate names match a motor.
     */
    @Deprecated
    private DcMotor pickMotor(String... candidates) {
        for (String name : candidates) {
            try {
                return hardwareMap.get(DcMotor.class, name);
            } catch (IllegalArgumentException e) {
                // Continue to next candidate
            }
        }
        throw new IllegalArgumentException(
            "Unable to find a hardware device with names "
            + java.util.Arrays.toString(candidates));
    }

    /**
     * Sleeps the robot while any motors are running.
     */
    public void autoSleep() {
        autoSleep(getAllDcMotors());
    }

    /**
     * Sleeps the robot while the given motors are running.
     *
     * @param motors The motors to wait for.
     */
    public void autoSleep(DcMotor... motors) {
        autoSleep(new HashSet<>(Arrays.asList(motors)));
    }

    /**
     * Sleeps the robot while the given motors are running.
     *
     * @param motors The motors that are running.
     */
    public void autoSleep(Set<DcMotor> motors) {
        // Sleep while any of the motors are still running.
        while (motors.stream().anyMatch(DcMotor::isBusy)) {
            sleep(1);
        }
    }

    /**
     * Initiates all hardware needed for the wheels.
     * <p>
     * <strong>When starting a new season, change the return type from
     * {@link Wheels} to the desired type.</strong>
     */
    private void initWheels() {
        // Prevent multiple instantiation.
        if (WHEELS != null) {
            return;
        }

        /*
         * Instantiate the wheel system. For a mecanum drive robot we use the
         * MecanumWheels implementation. The expected hardware names for the
         * four wheel motors are "frontLeftWheel", "frontRightWheel",
         * "backLeftWheel", and "backRightWheel". Adjust these names to
         * match your robot configuration. The wheel distances and ticks per
         * inch are approximate; tune them for your specific robot.
         */
        try {
            /*
             * Acquire each of the four drive motors. To be tolerant of
             * different naming conventions in the Robot Controller config,
             * we attempt to fetch several candidate names for each motor.
             * Update the candidate lists if your team uses different
             * names (for example, "frontLeft", "lf", "leftFront", etc.).
             */
            DcMotor frontLeftMotor = pickMotor(
                "frontLeftWheel",
                "frontLeftMotor",
                "frontLeft",
                "lf",
                "leftFront"
            );
            DcMotor frontRightMotor = pickMotor(
                "frontRightWheel",
                "frontRightMotor",
                "frontRight",
                "rf",
                "rightFront"
            );
            DcMotor backLeftMotor = pickMotor(
                "backLeftWheel",
                "backLeftMotor",
                "backLeft",
                "lb",
                "leftBack"
            );
            DcMotor backRightMotor = pickMotor(
                "backRightWheel",
                "backRightMotor",
                "backRight",
                "rb",
                "rightBack"
            );

            // Approximate measurements from the CAD model (in inches).
            // The wheel circumference is 4 inches in diameter multiplied by π.
            double wheelCircumference = 4.0 * Math.PI;
            double gearRatio = 1.0;
            // TODO: Change the motor type as necessary.
            double ticksPerInch = frontLeftMotor.getMotorType().getTicksPerRev()
                                  * gearRatio / wheelCircumference;


            // TODO: Replace with the necessary constructor.
            WHEELS = new MecanumWheels.Builder()
                // TODO: Approximate distances between wheels. Adjust as
                //  necessary if your robot's chassis dimensions differ.
                .setLateralWheelDistance(8.5)
                .setLongitudinalWheelDistance(14.5)
                .setTicksPerInch(ticksPerInch)
                // TODO: Change as necessary in accordance with your type of
                //  wheel system.
                .setFrontLeftMotor(frontLeftMotor)
                .setFrontRightMotor(frontRightMotor)
                .setBackLeftMotor(backLeftMotor)
                .setBackRightMotor(backRightMotor)
                .build();

        } catch (Exception e) {
            /*
             * If any motor could not be found, report the error. This keeps
             * the telemetry output informative and
             * avoids a null pointer exception later on. Leave WHEELS as null
             *  to signal an initialization failure.
             */
            telemetry.addLine("ERROR: Failed to initialize wheels: \n"
                              + e.getMessage());
        }

        /*
         * Assume the robot starts at (0, 0, 0) in the RoadRunner field
         * coordinate frame.
         * TODO: If your autonomous program uses a different starting pose,
         *  modify the pose here accordingly.
         */
        MECANUM_DRIVE = new MecanumDrive(
            hardwareMap,
            new Pose2d(0.0, 0.0, 0.0)
        );
    }

    /**
     * Initiate all hardware needed for the arm.
     * <p>
     * <strong>When starting a new season, change the type from {@link Arm} to
     * the desired type.</strong>
     */
    private void initArm() {
        // Prevent multiple instantiation.
        if (ARM != null) {
            return;
        }

        /*
         * TODO: Replace Arm.Builder() with a constructor of the desired
         *  builder subclass (e.g., FoldingArm.Builder()).
         *  You might want to look at the class and code from previous years
         *  for reference.
         */
        ARM = new Arm.Builder().build();
    }

    /**
     * Initiate all hardware needed for the claw.
     * <p>
     * <strong>When starting a new season, change the return type from
     * {@link Claw} to the desired type.</strong>
     */
    public void initClaw() {
        // Prevent multiple instantiation.
        if (CLAW != null) {
            return;
        }

        /*
         * TODO: Replace Claw.Builder() with a constructor of the desired
         *  builder subclass (e.g., SingleServoIntakeClaw.Builder())
         *  You might want to look at the class and code from previous years
         *  for reference.
         */
        CLAW = new Claw.Builder().build();
    }

    /**
     * Apply the currently selected alliance to the webcam’s color target.
     * Called in both {@link DriverMode} and {@link Auto} after
     * {@link AutoConfigurator#readConfigFile()}.
     */
    protected void applyAllianceToWebcam() {
        if (WEBCAM == null) {
            // No camera configured
            return;
        }

        // Map alliance to webcam color enum.
        Webcam.Color color =
            (
                AUTO_CONFIG.getAllianceColor()
                == AutoConfigurator.AllianceColor.RED
            )
            ? Webcam.Color.RED :
            Webcam.Color.BLUE;
        WEBCAM.setTargetColor(color);
    }

    /**
     * Initiate the webcam.
     * <p>
     * This method ignores the supplied {@code cameraMonitorViewId} and always
     * constructs the {@link Webcam} using the default EasyOpenCV behavior
     * (i.e., no custom viewport container). Passing a non‑empty container ID
     * into EasyOpenCV can lead to the exception "Viewport container specified
     * by user is not empty". By always using the three‑argument {@link Webcam}
     * constructor, we avoid that error.
     *
     * @param cameraMonitorViewId An unused resource ID. Kept for compatibility
     *                            with existing call sites.
     */
    public void initWebcam(int cameraMonitorViewId) {
        // Lenovo webcams typically support 640×480 resolution. Use this as a
        // sensible default. If your camera can benefit from higher
        // resolution, adjust the numbers here.
        int[] resolution = {640, 480};

        // Adjust the camera pose offsets in inches. Positive x is to the
        // right, positive y is forward, and positive z is up. Tweak
        // these values based on the physical mounting of your webcam.
        double[] poseAdjust = new double[]{
            0.0, // x offset (inches)
            0.0, // y offset (forward/back)
            0.0  // z offset (height)
        };

        // Initialize the webcam without specifying a viewport container.
        // The fourth parameter (view ID) is intentionally omitted to
        // prevent the OpenCvCameraException related to a non‑empty
        // container. VisionPortal manages its own view.
        WEBCAM = new Webcam(
            hardwareMap.get(WebcamName.class, "Webcam"),
            resolution,
            poseAdjust
        );

        applyAllianceToWebcam();
    }

    /**
     * Run automatically after pressing "Init." Initiate all the robot's
     * hardware. Wait until the driver presses "Start."
     */
    @Override
    public void runOpMode() {
        autoSleepEnabled = true;

        AUTO_CONFIG = AutoConfigurator.parseConfigFile();
        telemetry.addData(
            "Starting position",
            AUTO_CONFIG.getAllianceColor()
            + ", "
            + AUTO_CONFIG.getAllianceSide().name()
        );

        initWheels();
        initArm();
        initClaw();

        /*
         * Get camera ID to stream.
         * TODO: Currently not working.
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                                                        .getIdentifier(
                                                            "cameraMonitorViewId",
                                                            "id",
                                                            hardwareMap.appContext.getPackageName()
                                                        );
        initWebcam(cameraMonitorViewId);

        telemetry.addData("cameraMonitorViewId", cameraMonitorViewId);
        telemetry.update();

        waitForStart();
    }
}