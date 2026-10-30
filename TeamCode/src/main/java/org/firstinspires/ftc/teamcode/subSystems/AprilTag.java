package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.auton.GoBildaPinpointDriver;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.List;

import robotcore.Subsystem;

public class AprilTag extends Subsystem {

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;
    GoBildaPinpointDriver odo;




    @Override
    public void init(OpMode opMode) {
        initAprilTag();
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
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

    public void Averaging() {
        List<AprilTagDetection> tagsDetected = aprilTag.getDetections();
        int update = 0;
        int amountOfAprilTags = 0;
        int count = 0;
        AprilTagDetection closestTag = null;
        double minValue = 1000;

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
        }
    }


            public void localize () {
                List<AprilTagDetection> tagsDetected = aprilTag.getDetections();
                double X;
                double Y;
                int count = 0;

                for (AprilTagDetection detection : tagsDetected )
                    if (detection.metadata != null && detection.robotPose != null) {
                         X += detection.robotPose.getPosition().x;
                         Y += detection.robotPose.getPosition().y;
                         count++;
                    }
                X = X / count;
                Y = Y / count;

                telemetry.addData("X coordinate", X);
                telemetry.addData("Y coordinate", Y);
                }


                    }






