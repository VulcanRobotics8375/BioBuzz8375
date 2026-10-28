package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import tech.singularity.rtpaxon.RTPAxon;

import java.util.List;

import robotcore.Subsystem;

public class AprilTag extends Subsystem {

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    private RTPAxon axon = null;

    private RTPAxon axonTwo = null;


    @Override
    public void init(OpMode opMode) {
        initAprilTag();
        CRServo turretServo1 = hardwareMap.get(CRServo.class, "Turret_Servo1");
        CRServo turretServo2 = hardwareMap.get(CRServo.class, "Turret_Servo2");
        axon = new RTPAxon(turretServo, turretEncoder);
//      axon.setPidCoeffs(0.019, 0.0428, 0.0001);
        axon.setPidCoeffs(0.021, 0.002, 0.0003);
        axonTwo = new RTPAxon(turretServo, turretEncoder);
//      axonTwo.setPidCoeffs(0.019, 0.0428, 0.0001);
        axonTwo.setPidCoeffs(0.021, 0.002, 0.0003);


    }

    public void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam1"))
                .addProcessor(aprilTag)
                .build();
    }

    public void turret() {
        List<AprilTagDetection> tagsDetected = aprilTag.getDetections();
        int update = 0;
        int amountOfAprilTags = 0;
        int count = 0;
        AprilTagDetection closestTag = null;
        double minValue = 1000;

        if (gamepad2.rightBumperWasPressed()) {
            axon.setTargetRotation(0);
            axonTwo.setTargetRotation(0);
            return;
        }
        for (AprilTagDetection detection : tagsDetected) {
            double distance = detection.ftcPose.range;
            if (detection.metadata != null) {
                amountOfAprilTags += detection.ftcPose.range;
                count++;
            }
            if (closestTag != null) {
                update = amountOfAprilTags / count; // this variable holds the avg distance
                // telemetry?
            }
            if (distance < minValue) {
                minValue = distance;
                closestTag = detection;
            }

//           next few lines kinda confuse me; from my understanding if the distance is within 2 degrees it doesnt move otherwise it can adjust speed????
//           also the axon turning???
                if (bearing > 2) {
                    if (bearing * 0.2 > 0.9) {
                        axon.setPower(0.9);
                        axon.setPower(-0.9);
                    } else {
                        axon.setPower(bearing * 0.2);
                    }

                } else if (bearing < -2) {
                    if (bearing * 0.2 < -0.9) {
                        axon.setPower(-0.9);
                        axonTwo.setPower(0.9);
                    } else {
                        axon.setPower(bearing * 0.2);
                        axonTwo.setPower(bearing * -0.2);
                    }

                } else {
                    axon.setPower(0.0);
                    axonTwo.setPower(0.0);
                }
            }
        if (closestTag == null) {
            axon.setPower(0.0);
            axonTwo.setPower(0.0);
             }
        }

    public void localize() {
        List<AprilTagDetection> tagsDetected = aprilTag.getDetections();

        for(AprilTagDetection detection : tagsDetected) {
            if (detection.metadata != null && detection.robotPose != null) {
                double x = detection.robotPose.getPosition().x;
                double y = detection.robotPose.getPosition().y;
                // if heading is neccesary double heading = detection.robotPose.getOriantation;
            }
        }
    }

    }


