package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.auton.GoBildaPinpointDriver;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.List;

import robotcore.Subsystem;

public class AprilTag extends Subsystem {

    private AprilTagProcessor aprilTag;
    double X = 0.0;
    double Y = 0.0;
    double H = 0.0;
    int count = 0;
    public double camX, camY, camH;
    double x;
    double y;
    double z;


    private VisionPortal visionPortal;

    @Override
    public void init(OpMode opMode) {
        initAprilTag();
    }

    public void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setCameraPose(
                        new Position(DistanceUnit.INCH, x, y, z, 0),
                        new YawPitchRollAngles(AngleUnit.DEGREES, yaw, pitch, roll, 0))
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam1"))
                .addProcessor(aprilTag)
                .build();
    }

//    public void averaging() {
//        List<AprilTagDetection> tagsDetected = aprilTag.getDetections();
//        int update = 0;
//        int amountOfAprilTags = 0;
//        int count = 0;
//        AprilTagDetection closestTag = null;
//        double minValue = 1000;
//
//        for (AprilTagDetection detection : tagsDetected) {
//            double x = detection.robotPose.getPosition().x;
//            if (detection.metadata != null) {
//                amountOfAprilTags += detection.ftcPose.range;
//                count++;
//            }
//            if (closestTag != null) {
//                update = amountOfAprilTags / count; // this variable holds the avg distance
//                // telemetry?
//            }
//            if (distance < minValue) {
//                minValue = distance;
//                closestTag = detection;
//            }
//        }
//    }


    public void localize () {
        List<AprilTagDetection> tagsDetected = aprilTag.getDetections();

        X = 0;
        Y = 0;
        H = 0;
        count = 0;

        for (AprilTagDetection detection : tagsDetected )
            if (detection.metadata != null && detection.robotPose != null) {
                X += detection.robotPose.getPosition().x;
                Y += detection.robotPose.getPosition().y;
                H += detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);
                count++;
            }

        if (count ==0) {
            return;
        }

        camX = X / count;
        camY = Y / count;
        camH = H /count;

        telemetry.addData("X coordinate", camX);
        telemetry.addData("Y coordinate", camY);
        telemetry.addData("H coordinate", camH);

    }
}