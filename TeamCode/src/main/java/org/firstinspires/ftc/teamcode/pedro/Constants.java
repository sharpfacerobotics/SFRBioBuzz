package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;

/**
 * Pedro Pathing setup. Run the tuners in pedro/procedures and paste the code they print
 * over the matching config below. Every value marked TODO(tune) is a placeholder.
 */
public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(RobotConfig.FRONT_LEFT_MOTOR_NAME);
        c.frontRightName.set(RobotConfig.FRONT_RIGHT_MOTOR_NAME);
        c.backLeftName.set(RobotConfig.BACK_LEFT_MOTOR_NAME);
        c.backRightName.set(RobotConfig.BACK_RIGHT_MOTOR_NAME);
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    // TODO(tune): run PinpointTuner for directions + offsets.
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set(RobotConfig.PINPOINT);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(0.0);
        c.yPodOffset.set(0.0);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // TODO(tune): run ForesightTuner and replace this whole block with its output.
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller primaryTranslationalForward = Controller.proportional(0.1);
        Controller secondaryTranslationalForward = Controller.proportional(0.02);
        Controller primaryTranslationalLateral = Controller.proportional(0.1);
        Controller secondaryTranslationalLateral = Controller.proportional(0.02);

        c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
        c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

        c.coast.set(Controller.proportionalFeedforward(0.01));
        c.brake.set(Controller.proportionalFeedforward(0.01));

        c.headingFeedback.set(Controller.proportional(1.0));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.1, 0.0));

        c.linearBrakeCoefficients.set(Matrix.diag(0.1, 0.1));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.002, 0.002));

        c.maxAchievableForwardVelocity.set(60.0);
        c.maxAchievableStrafeVelocity.set(50.0);
        c.naturalForwardDeceleration.set(40.0);
        c.naturalStrafeDeceleration.set(40.0);
    });

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig));
    }
}
