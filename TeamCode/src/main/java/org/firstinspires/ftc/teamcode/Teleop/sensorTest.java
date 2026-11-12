package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "sensorTest")
public class sensorTest extends OpMode {
private DcMotor intakeMotor;

double time = 0.0;

DigitalChannel beamBreak;
boolean objectDetected = false;

@Override
public void init() {
    beamBreak = hardwareMap.get(DigitalChannel.class,"break_beam");
    beamBreak.setMode(DigitalChannel.Mode.INPUT);
    intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
}
@Override
public void loop() {
    boolean beamBroken = !beamBreak.getState();

    if (beamBroken) {

        if (time == 0.0) {
            time = System.currentTimeMillis();
        }

        double finalTime = System.currentTimeMillis() - time;

        if (finalTime >= 500 && !objectDetected) {
            objectDetected = true;
            telemetry.addData("Ramp is full", beamBreak.getState()); // idk what other data to put here (ig that it will be smth in another class
            intakeMotor.setPower(0.0);

        }
    } else {
        time = 0.0;
        objectDetected = false;
    }
}

}
