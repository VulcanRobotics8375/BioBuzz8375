package org.firstinspires.ftc.teamcode.auton.pedroPathing;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        // TODO: fill in once Pedro Pathing 3.x API is confirmed
        return null;
    }

    // --- Previously tuned values (from old FollowerBuilder API) ---
    // mass: 5.15 kg
    // headingPIDF:              kP=2.4,  kI=0, kD=0.18,  kF=0.025
    // secondaryHeadingPIDF:     kP=2.78, kI=0, kD=0.15,  kF=0.015
    // translationalPIDF:        kP=0.6,  kI=0, kD=0.08,  kF=0.025
    // secondaryTranslationalPIDF: kP=0.304, kI=0, kD=0.026, kF=0.015
    // drivePIDF (filtered):     kP=0.0061,  kI=0, kD=0.000006, kF=0.6, filter=0.01
    // secondaryDrivePIDF:       kP=0.0065, kI=0, kD=0.0000046, kF=0.6, filter=0.01
    // forwardZeroPowerAccel:  -56.74221031313573
    // lateralZeroPowerAccel:  -64.36155902361646
    // pathConstraints: maxVel=0.99, maxAccel=100, maxAngVel=1.6, maxAngAccel=1
    // xVelocity: 61.291057436485005
    // yVelocity: 44.33464903343381

    // --- Motor names ---
    // rightFront, rightBack, leftBack, leftFront
    // leftFront: REVERSE, leftRear: REVERSE, rightFront: FORWARD, rightRear: FORWARD

    // --- Pinpoint localizer ---
    // hardwareMapName: "odo"
    // encoderResolution: 74.5027025034
    // forwardEncoderDirection: FORWARD, strafeEncoderDirection: FORWARD
    // forwardPodY: 0 (measured: -6.166397214874507 mm)
    // strafePodX:  0 (measured:  1.674090047520913 mm)
}
