package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subSystems.turret;


@TeleOp(name = "Turret Rotation Test", group = "Test")
public class TurretRotationTest extends OpMode {
    turret t = new turret();
    @Override
    public void init() {
        t.instantiateSubsystem(this);
        t.init(this);
    }

    @Override
    public void loop() {
        t.turretServo1.setPower(gamepad1.right_stick_x);
        t.turretServo2.setPower(gamepad1.right_stick_x);

        telemetry.addData("Angle (deg)", Math.toDegrees(t.turretAngle()));
        telemetry.addData("Velocity (rad/s)", t.tVelocity());
        telemetry.addData("Encoder ticks", t.encoder.getCurrentPosition());
        telemetry.addData("Reachable:", t.reachable);

        // 1. Drive servos directly with right stick X (no PID yet)
        //    so you can verify direction and encoder sign

        // 2. Display on telemetry:
        //    - current angle in degrees
        //    - current velocity in rad/s
        //    - whether reachable is true/false
        //    - raw encoder ticks (for sanity check)

        telemetry.update();
    }
}

