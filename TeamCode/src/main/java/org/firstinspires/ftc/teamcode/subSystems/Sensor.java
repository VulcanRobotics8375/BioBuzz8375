package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import robotcore.Subsystem;

public class Sensor extends Subsystem {

private DcMotor intakeMotor;

double time = 0.0;

DigitalChannel beamBreak;

boolean objectDetected = false;


    @Override
public void init(OpMode opMode) {
    beamBreak = hardwareMap.get(DigitalChannel.class,"break_beam");
    beamBreak.setMode(DigitalChannel.Mode.INPUT);
    intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
}
public void breakBeam() {
    boolean beamBroken = !beamBreak.getState();

    if (beamBroken) {

        if (time == 0.0) {
            time = System.currentTimeMillis();
        }

        double finalTime = System.currentTimeMillis() - time;

        if (finalTime >= 500 && !objectDetected) {
            objectDetected = true;
            telemetry.addData("Ramp is full", objectDetected); // idk what other data to put here (ig that it will be smth in another class
            intakeMotor.setPower(0.0);

        }
    } else {
        time = 0.0;
        objectDetected = false;
    }
}



}




