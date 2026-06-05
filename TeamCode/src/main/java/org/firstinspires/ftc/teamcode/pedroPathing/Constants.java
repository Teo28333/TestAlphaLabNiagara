package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;

public class Constants {
    // Pedro follower tuning: robot mass, coast deceleration, hold behavior, and heading control.
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.6078)
            .forwardZeroPowerAcceleration(-35.6374)
            .lateralZeroPowerAcceleration(-59.6914)
            .automaticHoldEnd(true)

            .headingPIDFCoefficients(new PIDFCoefficients(0.85 ,0,0.1,0.01))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.2,0.06234,0.001635))
            .centripetalScaling(0.0);



    // Mecanum drivetrain configuration: hardware names, motor directions, and measured speeds.
    public static MecanumConstants driveConstants = new MecanumConstants()
            // Hardware map motor names.
            .rightFrontMotorName(RobotConstants.FRONT_RIGHT)
            .rightRearMotorName(RobotConstants.BACK_RIGHT)
            .leftRearMotorName(RobotConstants.BACK_LEFT)
            .leftFrontMotorName(RobotConstants.FRONT_LEFT)
            // Motor directions make positive commands drive the robot correctly.
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            // Drive power and voltage behavior.
            .maxPower(1)
            .useBrakeModeInTeleOp(true)
            .useVoltageCompensation(true)
            .nominalVoltage(13)

            // Measured maximum robot velocities used by Pedro path following.
            .xVelocity(63.9810)
            .yVelocity(53.4758)
            .useBrakeModeInTeleOp(true);


    // Pinpoint localizer geometry and encoder settings.
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-3.9652)
            .strafePodX(-6.3849)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


    // Default path constraints: max power, velocity/accel-ish limits, and end tolerance.
    public static PathConstraints pathConstraints = new PathConstraints(0.95, 50, 4, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        // Build one Pedro follower with this robot's localizer and mecanum drivetrain.
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();

    }


}
