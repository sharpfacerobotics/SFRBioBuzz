//100% need to change for bioBuzz
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.975)
            .forwardZeroPowerAcceleration(-70)
            .lateralZeroPowerAcceleration(-55)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.09, 0, 0.0000, 0.025))
            .headingPIDFCoefficients(new PIDFCoefficients(0.55, 0, 0, 0.025))
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            1,
            1);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("front_right_drive") //2
            .rightRearMotorName("back_right_drive") //3
            .leftRearMotorName("back_left_drive") //1
            .leftFrontMotorName("front_left_drive")  //0
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(85.501311)
            .yVelocity(49.63637);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-6.3)
            .strafePodX(-0.75)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
