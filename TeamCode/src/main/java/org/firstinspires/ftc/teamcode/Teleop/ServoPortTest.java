package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Hardware check: no gamepad, no turret code, no PID.
 * Drives tServo1 and tServo2 at +0.3 for 1 s, then -0.3 for 1 s, forever.
 * If nothing moves here, it's the port / hub servo power / config, not the code.
 */
@TeleOp(name = "Servo Port Test", group = "Test")
public class ServoPortTest extends OpMode {
    CRServo s1, s2;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        s1 = hardwareMap.get(CRServo.class, "tServo1");
        s2 = hardwareMap.get(CRServo.class, "tServo2");
        telemetry.addLine("Servos will rock +/-0.3 automatically after START.");
        telemetry.addLine("TAKE THE SERVOS OFF THE TURRET (or check wires have slack) first.");
        telemetry.update();
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        double p = ((int) timer.seconds()) % 2 == 0 ? 0.3 : -0.3;
        s1.setPower(p);
        s2.setPower(p);
        telemetry.addData("power sent", "%.1f", p);
        telemetry.addData("tServo1 port", s1.getPortNumber());
        telemetry.addData("tServo2 port", s2.getPortNumber());
        telemetry.addData("controller", s1.getController().getDeviceName());
        telemetry.addData("hub voltage (V)", "%.2f",
                hardwareMap.voltageSensor.iterator().next().getVoltage());
        telemetry.update();
    }

    @Override
    public void stop() {
        s1.setPower(0);
        s2.setPower(0);
    }
}
