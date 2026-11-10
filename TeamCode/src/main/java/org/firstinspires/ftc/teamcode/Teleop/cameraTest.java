package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subSystems.AprilTag;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

@TeleOp(name = "camTest", group = "Test")
public class cameraTest extends OpMode {

    AprilTag APRILTAG = new AprilTag();
    @Override
    public void init() {
        APRILTAG.instantiateSubsystem(this);
        APRILTAG.init(this);
    }

    @Override
    public void loop() {
        List<AprilTagDetection> tagsDetected = APRILTAG.aprilTag.getDetections();
        telemetry.addData("count", tagsDetected.size());

        for (AprilTagDetection det : tagsDetected) {
            if (det instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection c = (AprilTagClusterDetection) det;
                telemetry.addData("name:", c.metadata.name).addData("percentClusterFound:", c.percentClusterFound);
                // c.metadata.name, c.percentClusterFound
            } else if (det instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection s = (AprilTagSingleDetection) det;
                telemetry.addData("s.id:", s.id);

                // s.id  (a tag that's NOT in your library, e.g. the other goal)
            }
            if (det.robotPose != null) {
                telemetry.addData("robotPoseX:", "%.2f", det.robotPose.getPosition().x);
                telemetry.addData("robotPoseY:", "%.2f", det.robotPose.getPosition().y);
                telemetry.addData("robotPoseZ:", "%.2f", det.robotPose.getPosition().z);
                telemetry.addData("robotPoseH:","%.2f",  det.robotPose.getOrientation().getYaw(AngleUnit.DEGREES));
            }
            if (det.ftcPose != null) {
                telemetry.addData("ftcPoseX:", "%.2f", det.ftcPose.x);
                telemetry.addData("ftcPoseY:", "%.2f", det.ftcPose.y);
                telemetry.addData("range:", "%.2f", det.ftcPose.range)
                        .addData("bearing:", Math.toDegrees(det.ftcPose.bearing))
                        .addData("elevation:", Math.toDegrees(det.ftcPose.elevation));
            }
        }
        telemetry.update();
    }

}
