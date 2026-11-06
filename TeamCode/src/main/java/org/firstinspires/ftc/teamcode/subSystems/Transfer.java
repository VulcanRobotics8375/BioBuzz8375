package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import robotcore.Subsystem;

public class Transfer extends Subsystem {

    private Servo TransferServo = null; //actually its the adaptive hood servo
    private Servo HoodServo = null;


    @Override
    public void init(OpMode opMode) {
        instantiateSubsystem(opMode);
        TransferServo = hardwareMap.get(Servo.class, "transfer_servo");
        HoodServo = hardwareMap.get(Servo.class, "hood_servo");
    }
    public void runTransfer() {
        TransferServo.getPosition();
        telemetry.addData("ServoAngleOne", TransferServo);
        if (gamepad1.rightBumperWasPressed()) {
            double update = TransferServo.getPosition();
            TransferServo.setPosition(update + 0.01);
        }
        if (gamepad1.leftBumperWasPressed()) {
            double update2 = TransferServo.getPosition();
            TransferServo.setPosition(update2 - 0.01);
        }
        if(gamepad1.bWasPressed()) {
            telemetry.addData("ServoAngleTwo", TransferServo);
        }

        if (gamepad1.x) {
            TransferServo.setPosition(0.9);
            HoodServo.setPosition(0.0f);
        } else {
            TransferServo.setPosition(0.0);
            HoodServo.setPosition(0.3);
        }
    }
}