package org.firstinspires.ftc.teamcode.subSystems;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterMemberMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

import robotcore.Subsystem;

public class AprilTag extends Subsystem {

    public AprilTagProcessor aprilTag;
    double X = 0.0;
    double Y = 0.0;
    double H = 0.0;
    int count = 0;
    public double camX, camY, camH;
    static final double CAM_R=-3.658, TURRET_DX=-0.835, TURRET_DY=0, CAM_HEIGHT=16.921;

    private VisionPortal visionPortal;

    @Override
    public void init(OpMode opMode) {
        initAprilTag();
    }

    public void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(blueAud())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setLensIntrinsics(888.2, 888.2, 640,400)
                .setCameraPose(
                        new Position (DistanceUnit.INCH, 0,0, CAM_HEIGHT, 0),
                        new YawPitchRollAngles(AngleUnit.DEGREES, 0, -54.4, 0,0)
                )
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "cam"))
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(1280, 800))
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


    public boolean localize (double turretAngle, double heading) {

        List<AprilTagDetection> tagsDetected = aprilTag.getDetections();

        X = 0;
        Y = 0;
        H = 0;
        count = 0;

        for (AprilTagDetection detection : tagsDetected )
            if (detection.robotPose != null) {
                double camOffX = TURRET_DX + CAM_R * Math.cos(turretAngle); //offset from turret center
                double camOffY = TURRET_DY + CAM_R * Math.sin(turretAngle);

                // rotate offset to field frame using robot heading from odometry:
                X += detection.robotPose.getPosition().x - (camOffX * Math.cos(heading) - camOffY * Math.sin(heading));
                Y += detection.robotPose.getPosition().y - (camOffX * Math.sin(heading) + camOffY * Math.cos(heading));
                H += AngleUnit.normalizeRadians(detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS) - turretAngle-heading);

//                X += detection.robotPose.getPosition().x;
//                Y += detection.robotPose.getPosition().y;
//                H += detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);
                count++;
            }

        if (count ==0) {
            return false;
        }

        camX = X / count;
        camY = Y / count;
        camH = AngleUnit.normalizeRadians(heading + (H/count));

        telemetry.addData("X coordinate", camX);
        telemetry.addData("Y coordinate", camY);
        telemetry.addData("H coordinate", camH);

        return true;
    }
    private static AprilTagLibrary blueSco() {
        // ---- measured ----
        final int    FIRST_ID = 42;                       // leftmost tag as seen from the field
        final double GX = 12.75, GY = 12.3555844358, GZ = 62.005;           // center of tag row
        final double PSI = Math.toRadians(-90);           // direction tags face

        // tag frame: x = viewer's right, y = down, z = into the goal
        final double TILT = Math.toRadians(30);   // tag plane's angle from horizontal (90 = wall, 0 = flat facing floor)

        double a = PSI + Math.PI / 2, b = -TILT;
        double ca = Math.cos(a / 2), sa = Math.sin(a / 2);
        double cb = Math.cos(b / 2), sb = Math.sin(b / 2);
        Quaternion q = new Quaternion(
                (float) (ca * cb), (float) (ca * sb),
                (float) (sa * sb), (float) (sa * cb), 0);


        ArrayList<AprilTagClusterMemberMetadata> members = new ArrayList<>();
        double[] xs = {-6.50, -2.75, 2.75, 6.50};          // tag spacing from SDK
        for (int i = 0; i < 4; i++)
            members.add(new AprilTagClusterMemberMetadata(
                    FIRST_ID + i, new VectorF((float) xs[i], 0, 0), 3.25));

        return new AprilTagLibrary.Builder()
                .addCluster(new AprilTagClusterMetadata(members, "Blue Goal Up", "BGU",
                        new VectorF((float) GX, (float)GY, (float) GZ), DistanceUnit.INCH, q))
                .build();
    }
    private static AprilTagLibrary blueAud() {
        // ---- measured ----
        final int    FIRST_ID = 38;                       // leftmost tag as seen from the field
        final double GX = 12.75, GY = -12.3555844358, GZ = 62.005;           // center of tag row
        final double PSI = Math.toRadians(-90);           // direction tags face

        // tag frame: x = viewer's right, y = down, z = into the goal
        final double TILT = Math.toRadians(30);   // tag plane's angle from horizontal (90 = wall, 0 = flat facing floor)

        double a = PSI + Math.PI / 2, b = -TILT;
        double ca = Math.cos(a / 2), sa = Math.sin(a / 2);
        double cb = Math.cos(b / 2), sb = Math.sin(b / 2);
        Quaternion q = new Quaternion(
                (float) (ca * cb), (float) (ca * sb),
                (float) (sa * sb), (float) (sa * cb), 0);


        ArrayList<AprilTagClusterMemberMetadata> members = new ArrayList<>();
        double[] xs = {-6.50, -2.75, 2.75, 6.50};          // tag spacing from SDK
        for (int i = 0; i < 4; i++)
            members.add(new AprilTagClusterMemberMetadata(
                    FIRST_ID + i, new VectorF((float) xs[i], 0, 0), 3.25));

        return new AprilTagLibrary.Builder()
                .addCluster(new AprilTagClusterMetadata(members, "Blue Goal Up", "BGU",
                        new VectorF((float) GX, (float)GY, (float) GZ), DistanceUnit.INCH, q))
                .build();
    }
}