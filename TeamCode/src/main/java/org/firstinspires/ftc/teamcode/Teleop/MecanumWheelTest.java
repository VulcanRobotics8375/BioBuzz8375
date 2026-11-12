package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subSystems.fieldCentricMecanum;

/**
 * Finds "forward is fine but strafing is weird" problems. Uses the SAME motor names/directions
 * as fieldCentricMecanum. Put the robot on blocks for part 1.
 *
 * PART 1 (on blocks), hold one button, ONE wheel spins at TEST_POWER:
 *   X = leftFront   Y = rightFront   A = leftBack   B = rightBack
 *   -> the named wheel must spin, and spin "forward" (top of wheel moving toward the robot front).
 *   Wrong wheel = config names swapped. Backward = direction wrong. Both look fine driving forward!
 *
 * PART 2 (on the floor), robot-centric, NO heading lock, NO field-centric math:
 *   dpad up = forward   dpad right = strafe right   dpad left = strafe left
 *   right bumper = diagonal forward-right
 *   Compare the "speed" line: strafe should be ~80-90% of forward, diagonal ~70%.
 *   "rotation" should stay near 0 while strafing (if not -> weight/roller/friction imbalance).
 */
@TeleOp(name = "Mecanum Wheel Test", group = "Test")
public class MecanumWheelTest extends OpMode {
    public static double TEST_POWER = 0.3;
    public static double DRIVE_POWER = 0.6;
    fieldCentricMecanum drive = new fieldCentricMecanum();
    double peakSpeed = 0;

    @Override
    public void init() {
        drive.instantiateSubsystem(this);
        drive.init(this);
    }

    @Override
    public void loop() {
        drive.odo.update();
        double fl = 0, fr = 0, bl = 0, br = 0;
        if (gamepad1.x) fl = TEST_POWER;
        if (gamepad1.y) fr = TEST_POWER;
        if (gamepad1.a) bl = TEST_POWER;
        if (gamepad1.b) br = TEST_POWER;

        boolean driving = true;
        if (gamepad1.dpad_up)            { fl = fr = bl = br = DRIVE_POWER; }
        else if (gamepad1.dpad_right)    { fl = DRIVE_POWER; fr = -DRIVE_POWER; bl = -DRIVE_POWER; br = DRIVE_POWER; }
        else if (gamepad1.dpad_left)     { fl = -DRIVE_POWER; fr = DRIVE_POWER; bl = DRIVE_POWER; br = -DRIVE_POWER; }
        else if (gamepad1.right_bumper)  { fl = DRIVE_POWER; br = DRIVE_POWER; }
        else driving = false;

        drive.frontLeftDrive.setPower(fl);
        drive.frontRightDrive.setPower(fr);
        drive.backLeftDrive.setPower(bl);
        drive.backRightDrive.setPower(br);

        // robot-frame velocity from the Pinpoint
        double h = drive.odo.getHeading(AngleUnit.RADIANS);
        double vx = drive.odo.getVelX(DistanceUnit.INCH), vy = drive.odo.getVelY(DistanceUnit.INCH);
        double fwd = vx * Math.cos(h) + vy * Math.sin(h);
        double left = -vx * Math.sin(h) + vy * Math.cos(h);
        double speed = Math.hypot(fwd, left);
        if (driving) peakSpeed = Math.max(peakSpeed, speed); else if (gamepad1.back) peakSpeed = 0;

        telemetry.addData("powers FL FR BL BR", "%.2f %.2f %.2f %.2f", fl, fr, bl, br);
        telemetry.addData("robot vel fwd / left (in/s)", "%.1f / %.1f", fwd, left);
        telemetry.addData("speed / peak (in/s)  [BACK resets peak]", "%.1f / %.1f", speed, peakSpeed);
        telemetry.addData("rotation (deg/s)", "%.1f",
                Math.toDegrees(drive.odo.getHeadingVelocity(org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit.RADIANS)));
        telemetry.addLine("Part 1 (blocks): X=FL Y=FR A=BL B=BR   Part 2 (floor): dpad up/right/left, RB = diagonal");
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.frontLeftDrive.setPower(0); drive.frontRightDrive.setPower(0);
        drive.backLeftDrive.setPower(0); drive.backRightDrive.setPower(0);
    }
}
