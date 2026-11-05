package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import robotcore.Subsystem;
@TeleOp(name= "TransferTest")
public class TransferTest  {

        private Servo TransferServo = null; //actually its the adaptive hood servo
        private Servo HoodServo = null;

        public void init(OpMode opMode) {
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

        if (gamepad1.x) {
            TransferServo.setPosition(0.9);
            HoodServo.setPosition(0.0f);
        } else {
            TransferServo.setPosition(0.0);
            HoodServo.setPosition(0.3);
        }
    }
    }

