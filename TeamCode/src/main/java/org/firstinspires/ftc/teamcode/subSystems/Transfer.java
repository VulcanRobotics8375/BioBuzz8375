package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import robotcore.Subsystem;

public class Transfer extends Subsystem {

    private Servo TransferServo = null;


    @Override
    public void init(OpMode opMode) {
        instantiateSubsystem(opMode);
        TransferServo = hardwareMap.get(Servo.class, "transfer_servo");
    }

    public void runTransfer() {
        if (gamepad1.x) {
            TransferServo.setPosition(0.1);
        } else {
            TransferServo.setPosition(0.0);
        }
    }
}