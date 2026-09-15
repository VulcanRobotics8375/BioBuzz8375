package org.firstinspires.ftc.teamcode.auton.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
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

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5.15) //configurable
//            .headingPIDFCoefficients(new PIDFCoefficients(2.4, 0, 0.214, 0.025))
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2.8, 0, 0.18, 0.015))
            .headingPIDFCoefficients(new PIDFCoefficients(2.4, 0, 0.18, 0.025)) //configurable
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2.78, 0, 0.15, 0.015)) //configurable
            .translationalPIDFCoefficients(new PIDFCoefficients(0.6, 0, 0.08, 0.025)) //configurable
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.304, 0, 0.026, 0.015)) //configurable
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0061, 0, 0.000006, 0.6,0.01)) //configurable
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.0065, 0, 0.0000046, 0.6 ,0.01)) //configurable
            .forwardZeroPowerAcceleration(-56.74221031313573) //configurable
            .lateralZeroPowerAcceleration(-64.36155902361646); //configurable


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.6, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(61.291057436485005) //configurable
            .yVelocity(44.33464903343381); //configurable

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("odo")
            .customEncoderResolution(74.5027025034)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .forwardPodY(-156.626489257812) //configurable
            .strafePodX(42.521887207031); //configurable
    //strafepodx: 1.674090047520913
    //forwardpody: -6.166397214874507
//            .forwardPodY(95.246069335937)
//            .strafePodX(109.0403808593);



}
