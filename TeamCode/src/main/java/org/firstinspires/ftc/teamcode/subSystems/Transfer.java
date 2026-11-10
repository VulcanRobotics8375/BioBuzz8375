package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import robotcore.Subsystem;

public class Transfer extends Subsystem {

    private Servo TransferServo = null; //actually its the adaptive hood servo


    @Override
    public void init(OpMode opMode) {
        instantiateSubsystem(opMode);
        TransferServo = hardwareMap.get(Servo.class, "transfer_servo");
    }
    public void runTransfer() {
        TransferServo.getPosition();
        telemetry.addData("ServoAngleOne", TransferServo);

        if (gamepad2.left_bumper) {
            TransferServo.setPosition(0.65);
        } else {
            TransferServo.setPosition(0.4);
        }
    }
}