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
        if (gamepad1.x) {
            TransferServo.setPosition(0.9);
            HoodServo.setPosition(0.0fa);
        } else {
            TransferServo.setPosition(0.0);
            HoodServo.setPosition(0.3);
        }
    }
}