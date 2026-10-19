package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import robotcore.Subsystem;

public class AprilTag extends Subsystem {

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;





    @Override
    public void init(OpMode opMode) {
        initAprilTag();
        CRServo turretServo1 = hardwareMap.get(CRServo.class,"Turret_Servo1");
        CRServo turretServo2 = hardwareMap.get(CRServo.class,"Turret_Servo2");
        // add encoder(S)???
    }
    public void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class,"Webcam1"))
                .addProcessor(aprilTag)
                .build();
    }

    public int turret() {
        List<AprilTagDetection> tagsDetected = aprilTag.getDetections();
        int update = 0;
        int amountOfAprilTags = 0;
        int count = 0;
        for (AprilTagDetection detection : tagsDetected) {
            if (detection.metadata != null) {
                amountOfAprilTags += detection.ftcPose.range;
                count++;
            }
            if (count == 0) {
                return -1; // something else could be used, -1 just means there is no apriltags found
            } else {
                update = amountOfAprilTags/count; // this variable holds the avg distance
                return update;
            }

        }
        return (amountOfAprilTags / count);
    }

}